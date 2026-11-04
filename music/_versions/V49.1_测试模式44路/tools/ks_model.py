# -*- coding: utf-8 -*-
"""ks_model.py —— 吉他音色（Karplus-Strong 拨弦物理模型）的【定点模型 + 参数表】

这个脚本干三件事：
  1) 用和 RTL【逐位一致】的整数运算，把 36 个键各自的弦模型算出来；
  2) 量出每个键的【实际音高（自相关）】和【衰减时间】，确认不跑调、能消下去；
  3) 导出试听文件 ks_guitar.wav + Verilog 参数表 ks_tables.v

RTL 侧的算法（top.v 里 gen_ks 那段，逐条对应）：

    r0 = ram[ptr]                          // 延迟线整数抽头
    r1 = ram[ptr+1]（到 Ni 回绕）
    xi = r0 + ((r1-r0)*frac6 >> 6)         // 线性插值 -> 小数延迟
    y  = (xi + xi_1) >> 1                  // 环路低通（经典 KS 的两点平均）
    y2 = (ptr == Ni-1) ? y - (y>>damp)     // ★ 每绕一圈掉一次（移位阻尼，
                                           //   不用乘法器；damp 由参数表给）
                        : y
    ram[ptr] = 起振期 ? 噪声 : y2
    xi_1 = xi ;  ptr 前进一格（回绕点 Ni）

环路总延迟 = Ni + frac6/64 + 0.5（平均滤波器那半拍）→ 等于 fs/f 就准。

用法：
    python ks_model.py            算参数 + 出 wav + 出 Verilog 表
    python ks_model.py --notable  不出 Verilog 表
"""

import math
import os
import sys
import wave

import numpy as np

FS = 12_500_000.0 / 128.0          # ★ V49：97.656 kHz（top.v: BCLK_HALF=1 -> 128 拍）
                                   #   V48 及以前是 12_500_000.0 / 256.0 = 48.828 kHz

# 板子 delta[] 表的 36 个频率（键号 = 模块丝印键号-1，不是音高顺序！）
FREQ = [117, 131, 156, 175, 196, 110, 123, 139, 147, 165, 185, 208,
        233, 262, 311, 349, 392, 220, 247, 277, 294, 330, 370, 415,
        466, 523, 622, 698, 784, 440, 494, 554, 587, 659, 740, 831]
NAME = ["A#2", "C3", "D#3", "F3", "G3", "A2", "B2", "C#3", "D3", "E3", "F#3", "G#3",
        "A#3", "C4", "D#4", "F4", "G4", "A3", "B3", "C#4", "D4", "E4", "F#4", "G#4",
        "A#4", "C5", "D#5", "F5", "G5", "A4", "B4", "C#5", "D5", "E5", "F#5", "G#5"]

MAX_DEPTH = 1024        # ★ V49：512 -> 1024。
                        #   96 kHz 下最低音 A2 需要 888 格（48 kHz 时只要 444），
                        #   而 1024×16 bit = 16 Kbit 仍然只占 1 块 SDPB（18 Kbit），
                        #   所以加深延迟线【不额外占 BSRAM】（36 条弦仍是 36 块）。
EXC_SHIFT = 4           # 激励幅度 = (noise8 × vel[7:4]) << EXC_SHIFT


# ============================================================
# 1) 参数表：每个键的 (Ni, frac6, damp)
# ============================================================
def delay_params(f):
    """把目标周期 T = fs/f 拆成 (整数回绕长度 Ni, 小数插值系数 frac6)

    环路延迟 = Ni - frac6/64 + 0.5（平均滤波器那半拍 + 插值读在 ptr..ptr+1 上
    取加权，读 ptr+1 就等于少等一拍），所以：
        Nf = T - 0.5 ; Ni = ceil(Nf) ; frac6 = round((Ni - Nf)*64)
    frac6 凑到 64 时等价于"读满 ptr+1"，规约成 Ni-1 / frac6=0（少一格）。
    """
    T = FS / f
    Nf = T - 0.5
    Ni = int(math.ceil(Nf - 1e-12))
    frac6 = int(round((Ni - Nf) * 64))
    if frac6 >= 64:
        frac6 -= 64
        Ni -= 1
    if frac6 < 0:
        frac6 = 0
    return Ni, frac6


def build_table():
    """阻尼：每绕一圈乘 (1 - 2^-damp)，damp 由音高分组。

    T60 ≈ 6.9078 / (f × 2^-damp)            （只算这一份，平均滤波器那份另算）
    低音弦长、绕一圈慢 → 同一个 2^-damp 在低音上衰减更慢，所以：
        f <= 200 Hz   damp = 6   -> 每圈 -0.0156 -> T60 约 4 s
        f <= 400 Hz   damp = 6
        其它          damp = 7   -> 每圈 -0.0078 -> 高音约 1.1 s
    """
    d = {}
    for g, f in enumerate(FREQ):
        Ni, frac6 = delay_params(f)
        damp = 6 if f <= 400 else 7
        assert 2 <= Ni < MAX_DEPTH, (g, f, Ni)
        d[g] = (Ni, frac6, damp)
    return d


# ============================================================
# 2) 定点弦模型（与 RTL 逐位一致）
# ============================================================
class String:
    def __init__(self, Ni, frac6, damp, seed=0x1234):
        self.ram = [0] * MAX_DEPTH
        self.Ni, self.frac6, self.damp = Ni, frac6, damp
        self.ptr = 0
        self.xi1 = 0
        self.exc = 0
        self.lfsr = seed & 0xFFFF

    def _n8(self):
        # x^16 + x^14 + x^13 + x^11 + 1（RTL 同一条），返回低 8 位再取中值
        b = ((self.lfsr) ^ (self.lfsr >> 2) ^ (self.lfsr >> 3) ^ (self.lfsr >> 5)) & 1
        self.lfsr = ((self.lfsr >> 1) | (b << 15)) & 0xFFFF
        return ((self.lfsr >> 8) & 0xFF) - 128       # -128..127

    def pluck(self, key_mask_vel=220):
        self.ptr = 0
        self.xi1 = 0
        self.exc = self.Ni

    def step(self, vel=220):
        Ni, frac6, damp = self.Ni, self.frac6, self.damp
        p1 = 0 if (self.ptr + 1 >= Ni) else self.ptr + 1
        r0 = self.ram[self.ptr]
        r1 = self.ram[p1]
        xi = r0 + (((r1 - r0) * frac6) >> 6)
        y = (xi + self.xi1) >> 1
        if self.ptr == Ni - 1:
            y2 = y - (y >> damp)                     # 绕圈损耗（移位，无乘法器）
        else:
            y2 = y

        if self.exc > 0:
            w = (self._n8() * (vel >> 4)) << EXC_SHIFT
            self.exc -= 1
        else:
            w = y2

        self.ram[self.ptr] = w
        self.xi1 = xi
        self.ptr = p1
        return y2                                    # 起振期也只出"环内"的值


# ============================================================
# 3) 量音准 / 衰减
# ============================================================
def measure(g, table, n=None):
    Ni, frac6, damp = table[g]
    n = int(FS * 8)
    s = String(Ni, frac6, damp)
    s.pluck()
    out = np.empty(n, dtype=np.float64)
    for i in range(n):
        out[i] = s.step()
    # 自相关测周期（整数峰 + 抛物线插值，避免 1/周期 的量化误差）
    seg = out[int(FS * 0.5):int(FS * 0.5) + 32768]
    seg = seg - seg.mean()
    ac = np.correlate(seg, seg, 'full')[len(seg) - 1:]
    lo, hi = int(FS / 1200), int(FS / 80)
    L = lo + int(np.argmax(ac[lo:hi]))
    if lo < L < hi - 1:
        a, b, c = ac[L - 1], ac[L], ac[L + 1]
        L = L + 0.5 * (a - c) / (a - 2 * b + c + 1e-30)
    f_est = FS / L
    cents = 1200 * math.log2(f_est / FREQ[g])
    # T60：包络（RMS）下降 60 dB
    blk = 4096
    e = np.array([np.sqrt((out[i:i + blk] ** 2).mean())
                  for i in range(0, len(out) - blk, blk)])
    t60 = float('nan')
    idx = np.argmax(e < e[0] * 1e-3)
    if idx > 0:
        t60 = idx * blk / FS
    return f_est, cents, t60, float(np.max(np.abs(out))), float(np.sqrt((out ** 2).mean()))


# ============================================================
# 4) 试听
# ============================================================
def render_note(g, table, dur, vel=220):
    Ni, frac6, damp = table[g]
    s = String(Ni, frac6, damp, seed=0x1234 + g * 7919)
    s.pluck(vel)
    out = np.empty(int(dur * FS), dtype=np.float64)
    for i in range(len(out)):
        out[i] = s.step(vel)
    return out


def mix_into(total, x, t, gain=1.0):
    i = int(t * FS)
    x = x * gain
    n = min(len(x), len(total) - i)
    if n > 0:
        total[i:i + n] += x[:n]


def write_wav(path, x):
    x = np.clip(x, -32760, 32760).astype('<i2')
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(round(FS)))
        w.writeframes(x.tobytes())


def make_demo(table, path):
    total = np.zeros(int(FS * 11.0))
    t = 0.6
    for g in [5, 0, 6, 1, 7, 8, 2, 9, 3, 10, 4, 11]:      # A2 起一个八度音阶
        mix_into(total, render_note(g, table, 0.42), t, 0.9)
        t += 0.4
    t += 0.3
    for g in [5, 7, 9, 17]:                                # A2+C#3+E3+A3 和弦
        mix_into(total, render_note(g, table, 4.0), t, 0.55)
    t += 4.3
    for vel in (90, 220):                                  # 力度对比
        mix_into(total, render_note(5, table, 1.8, vel), t, 0.9)
        t += 1.9
    write_wav(path, total)


# ============================================================
# 5) 导出 Verilog 表（贴进 top.v）
# ============================================================
def dump_verilog(table, path):
    L = []
    L.append("    // ===== 由 tools\\ks_model.py 自动生成：36 键弦参数 =====")
    L.append("    //   NI_TAB : 延迟线回绕长度（环路延迟 = NI + FR/64 + 0.5 个采样）")
    L.append("    //   FR_TAB : 线性插值的小数系数（1/64 采样），用来把音调修准")
    L.append("    //   DMP_TAB: 每绕一圈的损耗位移（gain = 1 - 2^-DMP），定衰减时间")
    L.append("    function [9:0] NI_TAB(input [5:0] k);")
    L.append("        case (k)")
    for i in range(0, 36, 4):
        s = "            "
        for j in range(i, min(i + 4, 36)):
            s += "6'd%-2d: NI_TAB = 10'd%-4d;  " % (j, table[j][0])
        L.append(s)
    L.append("            default: NI_TAB = 10'd444;")
    L.append("        endcase")
    L.append("    endfunction")
    L.append("")
    L.append("    function [5:0] FR_TAB(input [5:0] k);")
    L.append("        case (k)")
    for i in range(0, 36, 6):
        s = "            "
        for j in range(i, min(i + 6, 36)):
            s += "6'd%-2d: FR_TAB = 6'd%-2d;  " % (j, table[j][1])
        L.append(s)
    L.append("            default: FR_TAB = 6'd0;")
    L.append("        endcase")
    L.append("    endfunction")
    L.append("")
    L.append("    function [3:0] DMP_TAB(input [5:0] k);")
    L.append("        case (k)")
    for i in range(0, 36, 9):
        s = "            "
        for j in range(i, min(i + 9, 36)):
            s += "6'd%-2d: DMP_TAB = 4'd%-2d;  " % (j, table[j][2])
        L.append(s)
    L.append("            default: DMP_TAB = 4'd6;")
    L.append("        endcase")
    L.append("    endfunction")
    with open(path, 'w', encoding='utf-8') as f:
        f.write("\n".join(L) + "\n")


# ============================================================
def main():
    table = build_table()
    print("键号 音名   目标Hz   实测Hz  偏差(音分)   Ni  frac  damp   T60(s)  峰值   RMS")
    print("-" * 84)
    worst = 0.0
    for g in range(36):
        f_est, cents, t60, peak, rms = measure(g, table)
        worst = max(worst, abs(cents))
        print("%3d  %-4s %8.1f %8.1f %8.2f %5d %5d %5d %8.2f %6d %6d"
              % (g, NAME[g], FREQ[g], f_est, cents, table[g][0], table[g][1],
                 table[g][2], t60, peak, rms))
    print("-" * 84)
    print("最大音高偏差 = %.2f 音分  (5 音分以内基本听不出来)" % worst)

    here = os.path.dirname(os.path.abspath(__file__))
    wav = os.path.join(here, "ks_guitar.wav")
    make_demo(table, wav)
    print("试听文件：", wav)
    if "--notable" not in sys.argv:
        v = os.path.join(here, "ks_tables.v")
        dump_verilog(table, v)
        print("Verilog 表：", v)


if __name__ == "__main__":
    main()
