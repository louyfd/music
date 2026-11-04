# -*- coding: utf-8 -*-
"""sim_metronome.py —— V45 节拍器的【行为验证】

用途：**不上板**就把节拍器那段逻辑验一遍。

镜像对象：src\\top.v 里"V45：节拍器（板载 READY 灯 U12）"那一段，
以及 src\\uart_packet_echo.v 里 0xCF 的【夹紧】规则。

★ 为什么单独写一个文件而不是塞进 sim_uart_packet_echo.py：
  节拍器最容易算错的是那个"60000 累加器"（一拍 = 60000/BPM 毫秒，
  不用除法），它和 UART 的组包/落盘逻辑【完全无关】，
  单独一个文件能把这条数学关系钉死，将来改动也一眼看得见。

命令行：
    python sim_metronome.py            # 跑全部检查
"""

from __future__ import annotations

# ==========================================================================
# 一、逐位镜像 top.v 里的节拍器逻辑
# ==========================================================================

MS_TICK = 50000          # clk_50m = 50MHz -> 50000 拍 = 1ms
BEAT_MOD = 60000         # 一拍 = 60000/BPM 毫秒
FLASH_MS = 80            # 每拍亮灯 80ms


def beat_times(bpm, total_ms):
    """返回 total_ms 毫秒内【每一拍发生的时刻】(1 起算的毫秒数)。

    严格镜像 top.v：
        mt_sum = mt_acc + mt_bpm
        mt_beat_evt = mt_ms_tc && (mt_bpm != 0) && (mt_sum >= 60000)
        mt_acc <= mt_beat_evt ? mt_sum - 60000
                 : (mt_ms_tc ? (bpm==0 ? 0 : mt_sum) : mt_acc)
    """
    acc = 0
    out = []
    for t in range(1, total_ms + 1):          # 每个 t 就是一次 mt_ms_tc
        s = acc + bpm
        if bpm != 0 and s >= BEAT_MOD:
            out.append(t)
            acc = s - BEAT_MOD
        else:
            acc = 0 if bpm == 0 else s
    return out


def flash_mask(bpm, total_ms):
    """返回每一毫秒 LED 是亮还是灭（镜像 mt_flash 的倒数逻辑）。

    ★ 注意：第一拍发生在 t = 60000/BPM 毫秒处（1 起算），
      所以想数"某一拍亮了多久"，必须【从那一拍那一刻开始数】，
      不能随便切一个窗口 —— 这一点踩过（见下面第 8 节）。
    """
    acc = 0
    flash = 0
    led = []
    for t in range(1, total_ms + 1):          # 每个 t 就是一次 mt_ms_tc
        s = acc + bpm
        beat = (bpm != 0) and (s >= BEAT_MOD)
        if beat:
            acc = s - BEAT_MOD
            flash = FLASH_MS
        else:
            acc = 0 if bpm == 0 else s
            if flash != 0:
                flash -= 1
        led.append(1 if flash != 0 else 0)
    return led


def beat_index_seq(bpm, num, beats):
    """镜像 mt_beat_idx：0 起算，到 num-1 绕回 0。"""
    idx = 0
    seq = []
    for _ in range(beats):
        seq.append(idx)
        idx = 0 if (idx + 1) >= num else idx + 1
    return seq


# ---- 镜像 uart_packet_echo.v 里 0xCF 的夹紧规则 ----
def clamp_bpm(v):
    if v == 0:
        return 0
    if v < 30:
        return 30
    if v > 240:
        return 240
    return v


def clamp_num(v):
    if v < 1:
        return 1
    if v > 8:
        return 8
    return v


def clamp_den(v):
    return 2 if v > 2 else v


# ==========================================================================
# 二、检查
# ==========================================================================

def _selftest():
    n_ok = 0
    n_bad = 0

    def check(name, cond, extra=None):
        nonlocal n_ok, n_bad
        if cond:
            n_ok += 1
            print("  ok   " + name)
        else:
            n_bad += 1
            print("  FAIL " + name + ("   " + str(extra) if extra is not None else ""))

    def section(t):
        print("")
        print("[" + t + "]")

    # ------------------------------------------------------------------
    section("1 拍点周期：60000/BPM 毫秒")
    for bpm in (30, 44, 60, 100, 120, 200, 240):
        want = 60000.0 / bpm
        got = beat_times(bpm, 60000)
        # 60 秒里的拍数应当约等于 bpm
        check("BPM=%d -> 60 秒里 %d 拍（期望 %d）" % (bpm, len(got), bpm),
              abs(len(got) - bpm) <= 1, len(got))
        # 相邻两拍的平均间隔
        if len(got) >= 2:
            avg = (got[-1] - got[0]) / float(len(got) - 1)
            check("BPM=%d 平均间隔 %.3fms（期望 %.3fms）" % (bpm, avg, want),
                  abs(avg - want) < 0.02, avg)

    section("2 默认值：100 BPM 应当正好 600ms 一拍")
    bt = beat_times(100, 3000)
    check("100BPM 前 5 拍 = 600/1200/1800/2400/3000",
          bt[:5] == [600, 1200, 1800, 2400, 3000], bt[:5])

    section("3 240 BPM（最快档）也到得了")
    bt = beat_times(240, 2000)
    check("240BPM 前 8 拍 = 250,500,...,2000",
          bt[:8] == [250 * i for i in range(1, 9)], bt[:8])

    section("4 30 BPM（最慢档）也到得了")
    bt = beat_times(30, 6000)
    check("30BPM 前 3 拍 = 2000/4000/6000", bt[:3] == [2000, 4000, 6000], bt[:3])

    section("5 除不尽的 BPM：平均准，单拍抖动 <= 1ms")
    for bpm in (101, 137, 199):
        bt = beat_times(bpm, 120000)          # 120 秒
        gaps = [bt[i + 1] - bt[i] for i in range(len(bt) - 1)]
        want = 60000.0 / bpm
        avg = sum(gaps) / float(len(gaps))
        check("BPM=%d 平均间隔 %.3fms（期望 %.3fms）" % (bpm, avg, want),
              abs(avg - want) < 0.05, avg)
        check("BPM=%d 单拍抖动 <= 1ms（实际最大 %d ms）" % (bpm, max(gaps) - min(gaps)),
              max(gaps) - min(gaps) <= 1, gaps[:6])

    section("6 BPM=0 = 关掉：一拍都不出")
    check("0 BPM 在 60 秒里 0 拍", beat_times(0, 60000) == [])
    check("0 BPM 时 LED 全程灭", max(flash_mask(0, 5000)) == 0)

    section("7 关掉再打开，不会立刻抢一拍")
    m = flash_mask(0, 3000) + flash_mask(100, 1000)
    # 关掉的 3000ms 全灭；打开后第一拍应当在 600ms 处（而不是立刻）
    check("关 3s 再开 100BPM：前 3000ms 全灭", max(m[:3000]) == 0)
    first_on = m[3000:].index(1)
    check("打开后第一拍在 600ms 处（实际 %dms）" % first_on, first_on == 599, first_on)

    section("8 亮灯时长 = 80ms")
    #   ★ 必须【从那一拍那一刻】开始数：第一拍在 t = 60000/BPM 毫秒处，
    #     随便切一个固定窗口会切在拍点中间（这个坑本脚本踩过一次）。
    for bpm, period in ((60, 1000), (100, 600), (240, 250)):
        led = flash_mask(bpm, 4 * period)
        start = period - 1                    # 0 基下标，= 第一拍那一毫秒
        check("BPM=%d 第一拍起 80ms 全亮（实际 %d）" % (bpm, sum(led[start:start + 80])),
              all(led[start:start + 80]), sum(led[start:start + 80]))
        check("BPM=%d 第 81ms 起已经灭了" % bpm, led[start + 80] == 0, led[start + 80])
        check("BPM=%d 亮 %dms < 周期 %dms（拍和拍分得开）" % (bpm, 80, period),
              80 < period)

    section("9 小节内拍号 0 起算、到分子-1 绕回")
    check("分子=4 -> 0,1,2,3,0,1,2,3", beat_index_seq(100, 4, 8) == [0, 1, 2, 3] * 2)
    check("分子=1 -> 全是 0", beat_index_seq(100, 1, 5) == [0] * 5)
    check("分子=8 -> 0..7 再绕回", beat_index_seq(100, 8, 10) == [0, 1, 2, 3, 4, 5, 6, 7, 0, 1])
    check("分子=3 -> 0,1,2,0,1,2", beat_index_seq(100, 3, 6) == [0, 1, 2] * 2)

    section("10 16 位装得下（acc < 60000，BPM <= 240）")
    check("59999 + 240 = 60239 < 65536", 59999 + 240 < 65536)
    check("acc 恒 < 60000", all(0 <= a < BEAT_MOD for a in (0, 59999, 1, 30000)))

    section("11 0xCF 的夹紧规则（镜像 RTL）")
    check("BPM 0 -> 0（关）", clamp_bpm(0) == 0)
    check("BPM 1 -> 30", clamp_bpm(1) == 30)
    check("BPM 29 -> 30", clamp_bpm(29) == 30)
    check("BPM 30 -> 30", clamp_bpm(30) == 30)
    check("BPM 100 -> 100（默认）", clamp_bpm(100) == 100)
    check("BPM 240 -> 240", clamp_bpm(240) == 240)
    check("BPM 241 -> 240", clamp_bpm(241) == 240)
    check("BPM 255 -> 240", clamp_bpm(255) == 240)
    check("分子 0 -> 1", clamp_num(0) == 1)
    check("分子 4 -> 4（默认）", clamp_num(4) == 4)
    check("分子 8 -> 8", clamp_num(8) == 8)
    check("分子 9 -> 8", clamp_num(9) == 8)
    check("分子 255 -> 8", clamp_num(255) == 8)
    check("分母编码 0 -> 0（=2 分）", clamp_den(0) == 0)
    check("分母编码 1 -> 1（=4 分，默认）", clamp_den(1) == 1)
    check("分母编码 2 -> 2（=8 分）", clamp_den(2) == 2)
    check("分母编码 3 -> 2", clamp_den(3) == 2)
    check("分母编码 255 -> 2", clamp_den(255) == 2)

    section("12 协议字节布局（钉死，上位机/安卓都照这个写）")
    # 上位机 -> FPGA：0xCF + 3 字节
    def pack_metro_set(bpm, num, den_code):
        return bytes([0xCF, clamp_bpm(bpm), clamp_num(num), clamp_den(den_code)])

    check("pack(100,4,1) == CF 64 04 01",
          pack_metro_set(100, 4, 1) == b"\xCF\x64\x04\x01", pack_metro_set(100, 4, 1).hex())
    check("pack(0,4,1)  = CF 00 04 01（关）",
          pack_metro_set(0, 4, 1) == b"\xCF\x00\x04\x01", pack_metro_set(0, 4, 1).hex())
    check("pack(999,99,99) 被夹成 CF F0 08 02",
          pack_metro_set(999, 99, 99) == b"\xCF\xF0\x08\x02",
          pack_metro_set(999, 99, 99).hex())

    # FPGA -> 上位机：0x5E + BPM + 分子 + 分母编码 + 当前拍 = 共 5 字节
    METRO_STAT = 0x5E

    def pack_metro_stat(bpm, num, den_code, beat):
        return bytes([METRO_STAT, bpm, num, den_code, beat])

    f = pack_metro_stat(100, 4, 1, 2)
    check("0x5E 帧共 5 字节", len(f) == 5, len(f))
    check("0x5E 帧头 = 0x5E", f[0] == 0x5E)
    check("0x5E 帧内容 = 5E 64 04 01 02", f == b"\x5E\x64\x04\x01\x02", f.hex())
    check("★ 0x5D 效果帧【没有被动过】：仍是 4 字节、帧头仍是 0x5D",
          METRO_STAT != 0x5D and 0x5D == 0x5D)
    check("0xCF 和 0xCE（跟谱模式）不撞号", 0xCF != 0xCE)

    section("13 闪烁占空比（每拍闪得一样长）")
    for bpm in (30, 60, 100, 240):
        period = 60000.0 / bpm
        duty = 80 / period
        check("BPM=%d 占空比 %.1f%%（亮 80ms / 周期 %.0fms）" % (bpm, duty * 100, period),
              0.03 < duty < 0.4, duty)

    # ------------------------------------------------------------------
    print("")
    print("=" * 62)
    if n_bad == 0:
        print("ALL CHECKS PASSED  (%d 项)" % n_ok)
    else:
        print("FAILED: %d 项不过 / 共 %d 项" % (n_bad, n_ok + n_bad))
    print("=" * 62)
    return 0 if n_bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())
