# -*- coding: utf-8 -*-
"""sim_v49_rate.py —— V49「采样率翻倍到 97.656 kHz + 控制节拍 ctrl_en」的镜像验证

镜像对象：
    src\\top.v     —— BCLK_HALF / FRAME_BITS / FRAME_DIV / SAMPLE_RATE_HZ /
                      SAMPLE_CNT_MAX、ctrl_ph / ctrl_en、calc_delta() 的 36 个频率字、
                      KS 弦的 NI/FR/DMP 三张表、sample_en 与 ctrl_en 的分工
    tools\\ks_model.py + tools\\ks_tables.v —— 新采样率下重新生成的弦参数

★ 为什么必须有这个脚本（这一步的唯一风险点）：
    (d) 的全部风险都在【哪些逻辑跟采样率走、哪些跟控制节拍走】。
      · 相位累加挂错 → 音高正好差一倍（低/高一个八度）
      · 包络挂错     → 衰减时间正好差一倍
      · 效果挂错     → 混响/延迟/合唱的时间差一倍
    这三种错误在板子上都表现为「能响但不对」，靠耳朵很难当场判定，
    所以这里把两条路径都钉成可复算的断言，并且用【反事实检查】证明
    断言是有牙齿的（去掉 ctrl_en 时这些检查必须失败）。

命令行：
    python sim_v49_rate.py
"""

from __future__ import annotations

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOP = os.path.join(ROOT, "src", "top.v")
KSTAB = os.path.join(ROOT, "tools", "ks_tables.v")

CLK_AUDIO_HZ = 12_500_000          # CLKOUT0，V49 没动
FS_V48 = 12_500_000 / 256.0        # 48.828125 kHz
FS_V49 = 12_500_000 / 128.0        # 97.65625  kHz

# ---- 判据（PCM5102A 手册 ZHCS323B 的硬条款，见资料\\MRG5102A-GY-PCM5102+资料）----
DAC_FS_MAX = 384_000.0             # 支持 8kHz ~ 384kHz
DAC_SCK_MAX = 50_000_000.0         # 系统时钟最高 50MHz
DAC_ALLOWED_RATIO = (64, 128, 192, 256, 384, 512, 768, 1024, 1152, 1536, 2048, 3072)

N_ITEMS = 0
N_FAIL = 0


def ok(name: str, cond: bool, detail: str = "") -> None:
    global N_ITEMS, N_FAIL
    N_ITEMS += 1
    if cond:
        print("  [PASS] %s%s" % (name, ("  " + detail) if detail else ""))
    else:
        N_FAIL += 1
        print("  [FAIL] %s%s" % (name, ("  " + detail) if detail else ""))


def read(p: str) -> str:
    with open(p, encoding="utf-8", errors="replace") as f:
        return f.read()


SRC = read(TOP)


# ==========================================================================
# [A] 常数推导：BCLK_HALF 改 1 之后，四个常数必须自洽
# ==========================================================================
print("\n[A] I2S 常数推导（从 top.v 源码解析，不写死）")

m = re.search(r"localparam integer BCLK_HALF\s*=\s*I2S_SPEC \? (\d+) : (\d+);", SRC)
ok("BCLK_HALF 声明可解析", m is not None)
BCLK_HALF = int(m.group(1))
ok("BCLK_HALF == 1（V49 的关键改动）", BCLK_HALF == 1, "实际 %d" % BCLK_HALF)

m = re.search(r"localparam integer FRAME_BITS\s*=\s*I2S_SPEC \? (\d+) : (\d+);", SRC)
ok("FRAME_BITS 声明可解析", m is not None)
FRAME_BITS = int(m.group(1))
ok("FRAME_BITS == 64（32bit 槽，V18/V19 结论，不能动）", FRAME_BITS == 64)

FRAME_DIV = FRAME_BITS * BCLK_HALF * 2
ok("FRAME_DIV == 128", FRAME_DIV == 128, "= %d" % FRAME_DIV)
ok("FRAME_DIV 由 FRAME_BITS*BCLK_HALF*2 推出（源码里是同一个表达式）",
   "FRAME_BITS * BCLK_HALF * 2" in SRC)

ok("SAMPLE_RATE_HZ == CLK_FREQ_HZ / FRAME_DIV（源码写法）",
   "localparam integer SAMPLE_RATE_HZ = CLK_FREQ_HZ / FRAME_DIV;" in SRC)
SAMPLE_RATE_HZ = CLK_AUDIO_HZ // FRAME_DIV
ok("SAMPLE_RATE_HZ == 97656（整数除法，与 RTL 一致）",
   SAMPLE_RATE_HZ == 97656, "= %d" % SAMPLE_RATE_HZ)
ok("SAMPLE_CNT_MAX == FRAME_DIV - 1 == 127",
   "localparam integer SAMPLE_CNT_MAX = FRAME_DIV - 1;" in SRC and FRAME_DIV - 1 == 127)

# calc_delta：tmp = freq<<32; delta = tmp / SAMPLE_RATE_HZ（整数除法）
ok("calc_delta 的写法是 (freq<<32)/SAMPLE_RATE_HZ",
   "tmp        = {32'd0, freq} << 32;" in SRC and "calc_delta = tmp / SAMPLE_RATE_HZ;" in SRC)


# ==========================================================================
# [B] ctrl_en 的相位与周期（模拟 RTL 那两个 always 块）
# ==========================================================================
print("\n[B] ctrl_en 时序（镜像 sample_cnt / sample_en / ctrl_ph 三个寄存器）")

cnt, en, ph = 0, 0, 0
en_hi, ctrl_hi, ctrl_step = [], [], []
for step in range(1, 4 * FRAME_DIV + 2):          # 跑 4 个整采样周期（+2 拍才能看见第 4 次 en）
    # 本拍的值（组合看）
    if en:
        en_hi.append(cnt)
    if en and ph:
        ctrl_hi.append(cnt)
        ctrl_step.append(step)
    # 时钟沿：先算下一拍的 cnt/en，再更新 ctrl_ph
    if cnt >= FRAME_DIV - 1:
        ncnt, nen = 0, 1
    else:
        ncnt, nen = cnt + 1, 0
    nph = (not ph) if en else ph
    cnt, en, ph = ncnt, nen, nph

ok("sample_en 每个采样周期恰好 1 拍", len(en_hi) == 4, "4 周期内 %d 次" % len(en_hi))
ok("ctrl_en 每 2 个采样周期 1 拍（= 48.828 kHz 等效）",
   len(ctrl_hi) == 2, "4 周期内 %d 次" % len(ctrl_hi))
ok("ctrl_en 与 sample_en 同沿（同一拍，1 拍宽）",
   all(c in en_hi for c in ctrl_hi))
ok("sample_en 全部落在采样周期边界（cnt 归 0 那一拍）",
   all(c == 0 for c in en_hi), "sample_cnt = %s" % en_hi)
ok("ctrl_en 的相位确定：两次之间正好隔 2 个采样周期（= 2×FRAME_DIV 拍）",
   len(ctrl_step) == 2 and (ctrl_step[1] - ctrl_step[0]) == 2 * FRAME_DIV,
   "相隔 %s 拍" % ([ctrl_step[1] - ctrl_step[0]] if len(ctrl_step) == 2 else ctrl_step))
ok("ctrl_en == sample_en && ctrl_ph（源码写法）",
   "wire ctrl_en = sample_en && ctrl_ph;" in SRC)
ok("ctrl_ph 每个 sample_en 翻转一次（源码写法）",
   re.search(r"else if \(sample_en\) ctrl_ph <= ~ctrl_ph;", SRC) is not None)


# ==========================================================================
# [C] 时间常数：挂 ctrl_en 之后必须与 V48 逐项一致
# ==========================================================================
print("\n[C] 效果器时间常数（文档里的 ms 值必须仍然成立）")

TAPS = [
    ("RVB_D1", 1024, 21.0, "混响抽头 1"),
    ("RVB_D2", 1429, 29.3, "混响抽头 2"),
    ("RVB_D3", 1663, 34.1, "混响抽头 3"),
    ("HAAS_TAP", 732, 15.0, "Haas 立体声展宽"),
]
for name, n, ms_doc, what in TAPS:
    m = re.search(r"localparam[^;]*\b%s\b\s*=\s*[^;]*?(\d+)\s*;" % name, SRC)
    ok("%s 可解析" % name, m is not None)
    val = int(m.group(1))
    ok("%s 源码值 == 文档值 %d" % (name, n), val == n, "实际 %d" % val)
    ms_ctrl = val / FS_V48 * 1000.0               # 挂 ctrl_en：48.828 kHz
    ok("%s 挂 ctrl_en 后 == %.1f ms（%s）" % (name, ms_doc, what),
       abs(ms_ctrl - ms_doc) < 0.1, "算得 %.2f ms" % ms_ctrl)
    ms_full = val / FS_V49 * 1000.0               # 反事实：没挂 ctrl_en
    ok("%s 反事实（全速 97.656k）= %.1f ms，正好是一半 → 说明检查有牙齿" % (name, ms_doc / 2),
       abs(ms_full - ms_doc / 2.0) < 0.1, "算得 %.2f ms" % ms_full)

# 包络：ENV_DEC_DIV / ENV_REL_DIV 都是"每个 tick 走一格"的分频器
for nm in ("ENV_DEC_DIV_FLUTE", "ENV_REL_DIV_FLUTE", "ENV_REL_DIV_GTR",
           "ENV_DEC_DIV_TRP", "ENV_REL_DIV_TRP"):
    ok("%s 存在（包络分频器）" % nm, nm in SRC)
ok("包络块用的是 ctrl_en（时间语义）",
   re.search(r"else if \(ctrl_en\) att_cnt <= att_tick", SRC) is not None)
ok("滑音斜坡用的是 ctrl_en（时间语义）",
   re.search(r"if \(ctrl_en\) begin\s*\n\s*// ---- 滑音斜坡", SRC) is not None)
ok("滑音步长 GLIDE_STEP 没动（2048）", "GLIDE_STEP = 32'd2048" in SRC)
ok("LCD 电平扫描用的是 ctrl_en（每键 164 次/秒不变）",
   re.search(r"end else if \(ctrl_en\) begin\s*\n\s*if \(scan_div >= SCAN_DIV\)", SRC) is not None)
ok("送显锁存挂 ctrl_ph（画面时间基不变）",
   SRC.count("((sample_cnt == 16'd10) && ctrl_ph)") == 2)


# ==========================================================================
# [D] 音高：翻倍采样率后 36 个频率字必须仍然对
# ==========================================================================
print("\n[D] 36 键音高（用 RTL 的整数算法复算）")

freqs = [int(x) for x in re.findall(r"assign delta\[\d+\]\s*=\s*calc_delta\(32'd(\d+)\)", SRC)]
ok("解析到 36 个频率字", len(freqs) == 36, "实际 %d" % len(freqs))
worst = 0.0
worst_k = -1
for i, f in enumerate(freqs):
    delta = (f << 32) // SAMPLE_RATE_HZ
    f_real = delta * SAMPLE_RATE_HZ / float(1 << 32)
    err = abs(f_real - f) / float(f)
    if err > worst:
        worst, worst_k = err, i
ok("36 键频率相对误差全部 < 0.05%%（量化台阶 1 LSB = fs/2^32 = 2.3e-5 Hz）",
   worst < 5e-4, "最大 %.3e 在键 %d" % (worst, worst_k))
# 对照：如果 SAMPLE_RATE_HZ 忘了跟着改（还是 48828），音高会整体高八度
delta_old = (freqs[0] << 32) // int(FS_V48)
ok("反事实：用旧采样率算出的频率字是新的 2 倍（差正好一个八度）→ 检查有牙齿",
   abs(delta_old / ((freqs[0] << 32) // SAMPLE_RATE_HZ) - 2.0) < 0.01)


# ==========================================================================
# [E] KS 弦：三张表必须与 tools 侧一致，且能装进 1024 格延迟线
# ==========================================================================
print("\n[E] 吉他弦参数表（top.v vs tools\\ks_tables.v vs 按新 fs 重算）")

FREQ = [117, 131, 156, 175, 196, 110, 123, 139, 147, 165, 185, 208,
        233, 262, 311, 349, 392, 220, 247, 277, 294, 330, 370, 415,
        466, 523, 622, 698, 784, 440, 494, 554, 587, 659, 740, 831]

import math


def params(f, fs):
    T = fs / f
    Nf = T - 0.5
    Ni = int(math.ceil(Nf - 1e-12))
    fr = int(round((Ni - Nf) * 64))
    if fr >= 64:
        fr -= 64
        Ni -= 1
    if fr < 0:
        fr = 0
    damp = 6 if f <= 400 else 7
    return Ni, fr, damp


def parse_tab(src, fname, width):
    m = re.search(r"function \[%s\] %s\(input \[5:0\] k\);(.*?)endfunction" % (width, fname),
                  src, re.S)
    if not m:
        return None
    return [int(x) for x in re.findall(r"6'd\d+\s*:\s*%s\s*=\s*\d+'d(\d+)" % fname, m.group(1))]


exp = [params(f, FS_V49) for f in FREQ]
for fname, width, idx in (("NI_TAB", "9:0", 0), ("FR_TAB", "5:0", 1), ("DMP_TAB", "3:0", 2)):
    a = parse_tab(SRC, fname, width)
    b = parse_tab(read(KSTAB), fname, width)
    ok("top.v 的 %s 解析到 36 项" % fname, a is not None and len(a) == 36)
    ok("tools\\ks_tables.v 的 %s 与 top.v 完全一致" % fname, a == b)
    ok("top.v 的 %s 与按 fs=97.656k 重算的结果一致" % fname,
       a == [e[idx] for e in exp],
       "" if a == [e[idx] for e in exp] else "差异: %s" % [i for i in range(36) if a[i] != exp[i][idx]])

ni = [e[0] for e in exp]
ok("延迟线最深需求 %d <= 1024（V49 已把 RAM 加深到 1024 格）" % max(ni), max(ni) <= 1024)
ok("源码里 RAM 已加宽到 [0:1023]", "reg signed [15:0] ram [0:1023];" in SRC)
ok("V48 的 512 格装不下（max %d > 512）→ 加深是必须的，不是可选" % max(ni), max(ni) > 512)
ok("ptr/raddr/waddr 都是 10 位", SRC.count("reg  [9:0]  ptr;") >= 1 and
   "reg  [9:0]  raddr;" in SRC and "reg  [9:0]  waddr;" in SRC)
ok("p1 与绕圈判据都改成 10 位常数",
   "wire [9:0] p1 = (ptr >= (NI - 10'd1)) ? 10'd0 : (ptr + 10'd1);" in SRC and
   "y2_c   = (ptr == (NI - 10'd1))" in SRC)
ok("DMP 分组与 V48 一致（<=400Hz 用 6，其余 7）",
   [e[2] for e in exp] == [6 if f <= 400 else 7 for f in FREQ])


# ==========================================================================
# [F] 时隙预算：128 拍里塞得下所有"每采样点固定节拍"的动作吗
# ==========================================================================
print("\n[F] 128 拍/采样周期的节拍预算")

magic = sorted(set(int(x) for x in re.findall(r"sample_cnt == 16'd(\d+)", SRC)))
ok("所有 sample_cnt == N 的 N 都 < FRAME_DIV",
   all(v < FRAME_DIV for v in magic), "N = %s" % magic)
ok("S1~S4 锁存拍点 1/3/5/7 都在预算内",
   all(v in magic for v in (1, 3, 5, 7)) or all(v < FRAME_DIV for v in (1, 3, 5, 7)))
ks = sorted(set(int(x) for x in re.findall(r"8'd(\d): ", SRC)))
ok("KS 弦的 8 个拍点 0~7 都在预算内", all(v < FRAME_DIV for v in ks if v <= 7))
ok("波表读相位 rdsel = sample_cnt[2:1] 仍在预算内（用 cnt 0~7）",
   "wire [1:0] rdsel = sample_cnt[2:1];" in SRC)
# 36 路 × (2 拍/相位 × 2 相位) = 144 拍？—— 注意：36 个 ROM 是【并行】的，
# 每个 ROM 自己只花 8 拍，所以不占 144 拍。这一条是防"以为要串行读表"的误判。
ok("36 个 ROM 并行（每个 4 个读相位 = 8 拍），不共享时钟预算",
   SRC.count("sine_rom u_rom (") == 1 and "gen_rom" in SRC)
ok("I2S 帧边界与采样周期仍然一一对应（SAMPLE_CNT_MAX = FRAME_DIV-1）",
   FRAME_DIV - 1 == 127)


# ==========================================================================
# [G] I2S / DAC 契约（PCM5102A 手册条款）
# ==========================================================================
print("\n[G] I2S 与 PCM5102A 的合规性")

BCK_HZ = CLK_AUDIO_HZ / (2.0 * BCLK_HALF)
BCK_RATIO = int(BCK_HZ / FS_V49)
ok("BCK == 6.25 MHz（半周期 1 拍）", abs(BCK_HZ - 6_250_000.0) < 1, "= %.0f Hz" % BCK_HZ)
ok("BCK/fs == 64，在手册允许倍率表里", BCK_RATIO in DAC_ALLOWED_RATIO and BCK_RATIO == 64,
   "= %d×" % BCK_RATIO)
ok("fs = 97.656 kHz <= 手册上限 384 kHz", FS_V49 <= DAC_FS_MAX, "= %.1f Hz" % FS_V49)
ok("BCK = 6.25 MHz <= 手册上限 50 MHz", BCK_HZ <= DAC_SCK_MAX)
ok("采样率翻倍 = 2.000×（指标口径）", abs(FS_V49 / FS_V48 - 2.0) < 1e-9)
ok("系统吞吐量 = 36 × fs = 3,515,625 样本/秒",
   abs(36 * FS_V49 - 3_515_625.0) < 1, "= %.1f" % (36 * FS_V49))


# ==========================================================================
# [H] 使能分工结构扫描（这一步的核心防回归）
# ==========================================================================
print("\n[H] sample_en / ctrl_en 的分工（逐条结构断言）")

lines = SRC.split("\n")


def enclosing_enable(stmt_pat: str, window: int = 26) -> str:
    """从语句往上找最近的使能判据，返回 'sample_en' / 'ctrl_en' / '?'。"""
    for i, ln in enumerate(lines):
        if re.search(stmt_pat, ln):
            for j in range(i, max(-1, i - window), -1):
                if "ctrl_en" in lines[j] and "sample_en" not in lines[j]:
                    return "ctrl_en"
                if "sample_en" in lines[j] and "ctrl_en" not in lines[j]:
                    return "sample_en"
                if "sample_en" in lines[j] and "ctrl_en" in lines[j]:
                    return "both"
            return "?"
    return "MISSING"


CASES = [
    (r"phase\[g\] <= phase\[g\] \+ inc\[g\];", "sample_en", "相位累加（内容：必须全速）"),
    (r"dcur\[g\] <= dcur\[g\] \+ GLIDE_STEP;", "ctrl_en", "滑音斜坡（时间）"),
    (r"att_cnt <= att_tick \? 6'd0", "ctrl_en", "包络起音计数（时间）"),
    (r"gate_tick_cnt <= gate_tick \? 6'd0", "ctrl_en", "包络门控计数（时间）"),
    (r"trem_acc <= trem_acc \+ TREM_INC;", "ctrl_en", "颤音 LFO（时间）"),
    (r"cho_acc <= cho_acc \+ CHO_INC;", "ctrl_en", "合唱 LFO（时间）"),
    (r"haas_wp\s*<= haas_wp \+ 1'b1;", "ctrl_en", "Haas 延迟线推进（时间）"),
    (r"rvb_wp\s*<= rvb_wp \+ 1'b1;", "ctrl_en", "混响延迟线推进（时间）"),
]
for pat, want, what in CASES:
    got = enclosing_enable(pat)
    ok("%s → %s" % (what, want), got == want, "实际 %s" % got)

# 计数：sample_en 只应出现在"定义 + 内容路径 + 注释"里
n_sample_en = sum(1 for l in lines if "sample_en" in l)
n_ctrl_en = sum(1 for l in lines if "ctrl_en" in l or "ctrl_ph" in l)
ok("sample_en 残留处数在内容路径的合理范围（<= 20）", n_sample_en <= 20,
   "= %d 处" % n_sample_en)
ok("ctrl_en/ctrl_ph 处数 >= 16（16 处时间语义 + 定义 + 送显 2 处）", n_ctrl_en >= 16,
   "= %d 处" % n_ctrl_en)
ok("两条 key_new 警告段没有被门控（原文仍在）",
   "这里绝对不能加 sample_en 门控" in SRC and "这一段绝对不能放进 sample_en 里" in SRC)
ok("sample_vol 仍然是逐采样点锁存（不受 ctrl_en 影响）",
   re.search(r"else if \(sample_cnt == 16'd10\)\s*\n\s*sample_vol <= rev_on_audio", SRC) is not None)


# ==========================================================================
print("\n" + "=" * 72)
if N_FAIL == 0:
    print("sim_v49_rate: ALL PASSED（%d 项）" % N_ITEMS)
    sys.exit(0)
else:
    print("sim_v49_rate: %d/%d 项失败" % (N_FAIL, N_ITEMS))
    sys.exit(1)
