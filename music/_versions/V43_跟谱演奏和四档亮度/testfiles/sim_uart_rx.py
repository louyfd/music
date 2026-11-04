# -*- coding: utf-8 -*-
"""sim_uart_rx.py —— 逐拍仿真 src\\uart_rx.v 的采样状态机（V38 增补）

为什么要有这个脚本
──────────────────
V38 给 uart_rx.v 加了「每一位采 3 次取多数」+「起始位中点复检」。
这是整个工程里【离数据正确性最近】的一处改动 —— 它一旦算错，
整条蓝牙链路收到的字节全是垃圾，比原来的单点采样还糟。

而 Verilog 的非阻塞赋值（<=）有个陷阱：同一拍里写进去的值，
【下一拍】才成立。第一次实现就踩了：

    if(bps_cnt == samp_c)  samp_v[2] <= rx_i;      // 下一拍才生效
    ...
    if(bps_cnt == samp_c)  recv_reg <= {maj(samp_v), ...};   // 同一拍就用 -> 读到旧值

这个脚本就是用来抓这类错误的：它把 uart_rx.v 逐条【精确】翻译成 Python，
每一拍算一次 next-state，语义与 Verilog 的非阻塞赋值一一对应。
★ 改了 uart_rx.v 之后请务必重跑本脚本。

跑法
────
    python testfiles\\sim_uart_rx.py            # 全跑（= selftest）
    python testfiles\\sim_uart_rx.py selftest
"""

import sys

# ★ 控制台默认是 cp1252（本机 ACP=1252），直接 print 中文会抛
#   UnicodeEncodeError 把整个脚本打断。这里强制走 UTF-8：
#   显示不乱码的前提是控制台也设成 UTF-8（chcp 65001 / PS 里设 OutputEncoding），
#   但至少不会再因为编码把验证脚本本身弄崩。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# 与 uart_packet_echo.v / uart.v 一致：50MHz / 9600 = 5208
CLK_HZ = 50_000_000
BAUD_DIV = 5208
KEY_BITS = 16          # 不参与本仿真，只为和 RTL 的常数表对齐

_n_pass = 0
_n_fail = 0


def check(desc, cond, detail=None):
    global _n_pass, _n_fail
    if cond:
        _n_pass += 1
        print("  ok   %s" % desc)
    else:
        _n_fail += 1
        print("  FAIL %s%s" % (desc, "" if detail is None else "  -> %r" % (detail,)))


# ==========================================================================
# uart_rx.v 的逐拍模型
# ==========================================================================
class UartRx:
    """逐拍复刻 uart_rx.v。每个 step() = 一个 clk 上升沿。

    ★ 所有赋值都先算进 n[]，最后统一应用 —— 这就是 Verilog 非阻塞赋值的语义。
    """

    def __init__(self, baud=BAUD_DIV):
        self.baud = baud
        self.bit_max = 9                 # parity_i = 0 -> 无校验 -> 起始+8 数据+停止
        # 采样点（与 RTL 的 wire 一一对应）
        self.samp_a = baud // 2 - baud // 8
        self.samp_b = baud // 2
        self.samp_c = baud // 2 + baud // 8
        self.samp_d = self.samp_c + 1    # ★ 移入点：比第三次采样晚一拍
        self.reset()

    def reset(self):
        self.recv_start = 0
        self.bps_cnt = 0
        self.recv_bit_cnt = 0
        self.recv_reg = 0                # 9 位：起始位 + 8 数据位
        self.samp_v = 0                  # 3 位：本位的三次采样
        self.rvalid_o = 0
        self.rdata_o = 0
        self.bytes_out = []

    def _maj(self):
        """samp_maj = (v0&v1) | (v1&v2) | (v0&v2)"""
        v0 = self.samp_v & 1
        v1 = (self.samp_v >> 1) & 1
        v2 = (self.samp_v >> 2) & 1
        return 1 if ((v0 & v1) | (v1 & v2) | (v0 & v2)) else 0

    def step(self, rx_i, rready_i=True):
        # ★ 全部先读进局部变量、算完再统一写回 —— 等价于 Verilog 的非阻塞赋值。
        #   （不用字典是为了速度：一次完整字节要跑 5 万拍，字典版慢 2~3 倍。）
        recv_start = self.recv_start
        bps_cnt = self.bps_cnt
        rbc = self.recv_bit_cnt
        baud = self.baud
        bit_max = self.bit_max
        samp_v = self.samp_v
        recv_reg = self.recv_reg

        # ---- recv_start ----
        if (not rx_i) and (not recv_start):
            n_rs = 1
        elif recv_start and rbc == 0 and bps_cnt == baud // 2 and rx_i:
            n_rs = 0                     # ★ V38：起始位中点复检失败 -> 作废
        elif rbc == bit_max and bps_cnt == baud // 2:
            n_rs = 0
        else:
            n_rs = recv_start

        # ---- bps_cnt ----
        if not recv_start:
            n_bc = 0
        elif bps_cnt >= baud:
            n_bc = 0
        else:
            n_bc = bps_cnt + 1

        # ---- recv_bit_cnt ----
        if not recv_start:
            n_rbc = 0
        elif bps_cnt == baud:
            n_rbc = (rbc + 1) & 0xF
        else:
            n_rbc = rbc

        # ---- samp_v（三次采样）----
        n_sv = samp_v
        if recv_start and rbc != bit_max:
            if bps_cnt == self.samp_a:
                n_sv = (samp_v & 0b110) | (1 if rx_i else 0)
            elif bps_cnt == self.samp_b:
                n_sv = (samp_v & 0b101) | ((1 if rx_i else 0) << 1)
            elif bps_cnt == self.samp_c:
                n_sv = (samp_v & 0b011) | ((1 if rx_i else 0) << 2)

        # ---- recv_reg：在 samp_d 移入，此时三次采样都已站住 ----
        n_rr = recv_reg
        if rbc != bit_max and bps_cnt == self.samp_d:
            v0 = samp_v & 1
            v1 = (samp_v >> 1) & 1
            v2 = (samp_v >> 2) & 1
            maj = 1 if ((v0 & v1) | (v1 & v2) | (v0 & v2)) else 0
            n_rr = ((recv_reg >> 1) | (maj << 8)) & 0x1FF

        # ---- 输出（与 RTL 一致：只看停止位是否为高）----
        if rbc == bit_max and bps_cnt == baud // 2 and rx_i:
            self.bytes_out.append((recv_reg >> 1) & 0xFF)

        self.recv_start = n_rs
        self.bps_cnt = n_bc
        self.recv_bit_cnt = n_rbc
        self.samp_v = n_sv
        self.recv_reg = n_rr


# ==========================================================================
# 波形生成
# ==========================================================================
def uart_wave(payload, baud=BAUD_DIV, idle_before=3, idle_after=3, glitches=None):
    """把一串字节编成 UART 波形（每拍一个电平）。

    payload      : bytes
    glitches     : {绝对拍号: 电平} —— 在这些拍上强行翻转电平，用来模拟毛刺
    返回 (wave, mark) ；mark 记录每个字节的起始拍号，方便定位
    """
    wave = [1] * idle_before
    marks = []
    for b in payload:
        marks.append(len(wave))
        bits = [0]                                    # 起始位
        for i in range(8):
            bits.append((b >> i) & 1)                 # LSB first
        bits.append(1)                                # 停止位
        for bit in bits:
            wave.extend([bit] * baud)
    wave.extend([1] * idle_after)
    if glitches:
        for pos, lvl in glitches.items():
            if 0 <= pos < len(wave):
                wave[pos] = lvl
    return wave, marks


def run_rx(wave, baud=BAUD_DIV):
    rx = UartRx(baud)
    for lvl in wave:
        rx.step(lvl)
    return rx.bytes_out


# ==========================================================================
# 用例
# ==========================================================================
def main():
    print("[1] 采样点位置（必须都落在位周期内、且互不相同）")
    b = BAUD_DIV
    sa, sb, sc, sd = b // 2 - b // 8, b // 2, b // 2 + b // 8, b // 2 + b // 8 + 1
    check("baud = %d，一个位 %d 拍" % (b, b), b == 5208)
    check("三个采样点都 > 0", sa > 0, sa)
    check("三个采样点都 < baud（没跨到位边界）", sc < b, (sa, sb, sc))
    check("采样点递增且互不相同", sa < sb < sc, (sa, sb, sc))
    check("移入点 samp_d = samp_c + 1（比第三次采样晚一拍）", sd == sc + 1, sd)
    check("移入点仍 < baud", sd < b, sd)
    print("      samp_a=%d  samp_b=%d  samp_c=%d  samp_d=%d" % (sa, sb, sc, sd))

    print("[2] 正常接收：每个字节都要原样收回来")
    payload = bytes([0x00, 0xFF, 0xC9, 0xAA, 0x55, 0x01, 0x80, 0x7F])
    wave, _ = uart_wave(payload)
    got = run_rx(wave)
    check("8 个字节全部收到（%s）" % payload.hex(" "), got == list(payload),
          [hex(x) for x in got])
    check("0xC9 能原样收到（这一版最关心的那个命令字）",
          len(got) == 8 and got[2] == 0xC9, [hex(x) for x in got])

    print("[3] 全 0 / 全 1 数据位（最容易被位对齐问题坑的两种图案）")
    for val in (0x00, 0xFF):
        w, _ = uart_wave(bytes([val, val, val]))
        g = run_rx(w)
        check("%02X 连收 3 次都对" % val, g == [val, val, val], [hex(x) for x in g])

    print("[4] 单点毛刺：3 取 2 表决必须能挡掉")
    #   在某个字节的数据位中间打一个 1 拍宽的毛刺，看收回来还是不是原值
    payload = bytes([0x00])
    base, marks = uart_wave(payload)
    start = marks[0]
    #   数据位 bit3 的中点附近（起始位之后第 4 个位周期的中点）
    glitch_at = start + b * 4 + b // 2
    for name, lvl in (("拉高", 1), ("拉低", 0)):
        wave = list(base)
        wave[glitch_at] = lvl
        got = run_rx(wave)
        check("数据位中间插 1 拍%s毛刺 -> 仍然收到 0x00" % name,
              got == [0x00], [hex(x) for x in got])

    print("[5] 起始位中点复检：假起始位（窄毛刺）必须被作废")
    #   在空闲高电平里插一个「1 拍低」的毛刺，它会被当成起始位启动；
    #   但到中点时电平已经是高 -> 复检失败 -> 这一轮作废，不该吐出垃圾字节
    wave = [1] * 20 + [0] + [1] * (BAUD_DIV * 4)
    got = run_rx(wave)
    check("窄毛刺不会吐出一个垃圾字节", got == [], [hex(x) for x in got])

    print("[6] 真实起始位不能误伤")
    w, _ = uart_wave(bytes([0x5A]))
    g = run_rx(w)
    check("正常起始位照收（0x5A）", g == [0x5A], [hex(x) for x in g])

    print("[7] 采样窗口在【实际 baud】下的相对位置（必须在位中间，离边界越远越安全）")
    #   ★ V38 把移入点从 baud/2 挪到了 baud/2 + baud/8 + 1（为了等第三次采样站住），
    #     所以"+1 拍"这件事必须在这里量出来：位宽只有 4~8 拍时 1 拍偏移就占比过大、
    #     会跨进下一位（那是理想化的极端值，实际永远是 5208）。
    rel_a = sa / float(b)
    rel_c = sc / float(b)
    rel_d = sd / float(b)
    check("第一个采样点落在位周期的前中段（0.30~0.45）", 0.30 <= rel_a <= 0.45, rel_a)
    check("最后一个采样点落在位周期的中后段（0.55~0.70）", 0.55 <= rel_c <= 0.70, rel_c)
    check("移入点仍在位周期内且离后边界 > 30%%（%.3f）" % rel_d, rel_d < 0.70, rel_d)
    check("三个采样点跨度不超过位宽的 1/3（抖动余量足够）",
          (sc - sa) / float(b) <= 0.34, (sc - sa) / float(b))
    print("      采样点占位: %.1f%% / %.1f%% / %.1f%%   移入: %.1f%%"
          % (rel_a * 100, sb / float(b) * 100, rel_c * 100, rel_d * 100))

    print("[7b] 白噪声容限：任意一个采样点上打毛刺，都不能改变这一位的判决")
    #   对每一个数据位的每一个采样点，各注入一次 1 拍毛刺，结果必须始终是原值。
    #   这正是"3 取 2"的意义：单点被污染不影响多数。
    #   ★ 只发 1 个字节：一次 run_rx 要跑 10 位 × 5208 = 5.2 万拍，
    #     8 位 × 3 点 = 24 次已经足够覆盖，再多只是白等。
    payload = bytes([0x00])
    base, marks = uart_wave(payload)
    bad = []
    for bit_i in range(8):
        cyc = marks[0] + b * (bit_i + 1)     # 起始位占第 0 个位周期
        for off in (sa, sb, sc):
            w = list(base)
            pos = cyc + off
            if pos < len(w):
                w[pos] = 1 - w[pos]          # 翻掉这一个采样点
                if run_rx(w) != [0x00]:
                    bad.append((bit_i, off))
    check("8 个数据位 × 3 个采样点 全部注入毛刺 -> 结果一个都没变", not bad, bad[:3])

    print("[7c] 极小 baud 只要求『不崩』（那不是真实工况，不作为正确性判据）")
    for small in (16, 8, 4):
        try:
            w, _ = uart_wave(bytes([0xC3]), baud=small)
            run_rx(w, baud=small)
            check("baud=%d 跑完不抛异常" % small, True)
        except Exception as ex:            # noqa: BLE001
            check("baud=%d 跑完不抛异常" % small, False, repr(ex))

    print("[8] 长流：32 个随机字节连收，一个都不能错")
    #   ★ 32 个而不是 256 个：一次 run_rx 是 5.2 万拍 × 字节数，
    #     纯 Python 逐拍跑 256 个要一分多钟。32 个足以暴露位对齐/累计漂移问题。
    import random
    rnd = random.Random(20261007)
    payload = bytes(rnd.randrange(256) for _ in range(32))
    w, _ = uart_wave(payload)
    g = run_rx(w)
    check("32 字节全对", g == list(payload),
          "收到 %d 个，第一处不同 @%s" % (
              len(g),
              next((i for i in range(min(len(g), len(payload)))
                    if g[i] != payload[i]), "无")))

    print()
    print("=" * 60)
    if _n_fail:
        print("失败 %d 项 / 通过 %d 项" % (_n_fail, _n_pass))
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % _n_pass)
    return 0


if __name__ == "__main__":
    sys.exit(main())
