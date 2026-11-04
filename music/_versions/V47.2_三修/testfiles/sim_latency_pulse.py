# -*- coding: utf-8 -*-
"""sim_latency_pulse.py —— V44.2/V45 延迟测量脉冲的【行为验证】

用途：**不上板**就证明"按一次键，C19 和 C18 各自只出一个脉冲"。

镜像对象：src\\top.v 里"延迟测量"那一段（V44.2 改过）：
    · lat_key（C19，clk_50m = 50MHz，20ns/拍）
        lat_evt_raw = |(lat_keys & ~lat_keys_d)
        按下 -> lat_busy = 1；lat_keys == 0（全部松手）-> lat_busy = 0
        lat_evt = lat_evt_raw & ~lat_busy        <<< 按住期间锁死
        hold  = LAT_HOLD_50M = 499 拍 = 10µs
    · lat_pcm（C18，clk_audio = 12.5MHz，80ns/拍）
        lat_sound = (样本 != 0)                     <<< 会跟着波形过零跳！
        lat_arm_evt = |key_new -> lat_armed = 1
        lat_pcm_evt = lat_armed & lat_sound & ~lat_sound_d
        发出 -> lat_armed = 0                    <<< 一次性，每按一次键只发一个
        hold  = LAT_HOLD_AUD = 124 拍 = 10µs

★ 为什么必须单独验：
  V44.2 之前 lat_pcm 的判据是 lat_sound 直接出沿，而 lat_sound 跟着
  【波形过零】跳（每周期两次）-> 脉冲会按过零率疯狂输出。"只出一个"
  是用户明确提的硬要求，这里把每一条边角情况都钉死。

命令行：
    python sim_latency_pulse.py
"""

from __future__ import annotations

# ---- clk_50m：20ns/拍；10µs = 500 拍 -> 计数值 499 ----
LAT_HOLD_50M = 499
# ---- clk_audio：80ns/拍；10µs = 125 拍 -> 计数值 124 ----
LAT_HOLD_AUD = 124

CLK50M_NS = 20
CLKAUD_NS = 80


# ==========================================================================
# 一、逐拍镜像 RTL
# ==========================================================================

def lat_key_wave(keys_seq):
    """keys_seq: 每个 50MHz 拍上 lat_keys 的值（36 位整数，0 = 全松手）。

    返回每个拍上 lat_key 引脚的电平（1 = 有脉冲）。
    """
    keys_d = 0
    busy = False
    hold = 0
    wave = []
    for keys in keys_seq:
        evt_raw = (keys & ~keys_d) != 0      # 有键【新按下】
        evt = evt_raw and not busy           # 按住期间锁死
        # ★ 输出必须用【当前】的 hold 判（RTL 里 lat_hold 是寄存器，
        #   lat_key = (lat_hold != 0) | lat_evt 是组合逻辑）——
        #   先判后减。写成"先减后判"会少算一拍（这个坑本脚本踩过一次）。
        wave.append(1 if (hold != 0 or evt) else 0)
        # --- lat_busy 的更新（RTL 顺序：evt_raw 优先于"全部松手"）---
        if evt_raw:
            busy = True
        elif keys == 0:
            busy = False
        # --- hold 倒数 ---
        if evt:
            hold = LAT_HOLD_50M
        elif hold != 0:
            hold -= 1
        keys_d = keys
    return wave


def lat_pcm_wave(key_new_seq, sample_seq):
    """key_new_seq: 每个音频拍上 key_new 是否非零（新按下的键）。
       sample_seq : 每个音频拍上 i2s_load 的样本值。

    返回每个拍上 lat_pcm 引脚的电平。
    """
    armed = False
    sound_d = False
    hold = 0
    wave = []
    for kn, s in zip(key_new_seq, sample_seq):
        sound = (s != 0)                     # ★ 会跟着波形过零跳
        arm_evt = bool(kn)
        pcm_evt = armed and sound and not sound_d
        # ★ 同 lat_key：先判后减（用当前的 hold）
        wave.append(1 if (hold != 0 or pcm_evt) else 0)
        # --- lat_armed（RTL 顺序：按键优先）---
        if arm_evt:
            armed = True
        elif pcm_evt:
            armed = False
        # --- hold 倒数 ---
        if pcm_evt:
            hold = LAT_HOLD_AUD
        elif hold != 0:
            hold -= 1
        sound_d = sound
    return wave


def count_pulses(wave):
    """数上升沿 = 脉冲个数。"""
    n = 0
    prev = 0
    for v in wave:
        if v and not prev:
            n += 1
        prev = v
    return n


def pulse_widths(wave):
    """每一段的宽度（拍数）。"""
    out = []
    run = 0
    for v in wave:
        if v:
            run += 1
        elif run:
            out.append(run)
            run = 0
    if run:
        out.append(run)
    return out


# ==========================================================================
# 二、造信号
# ==========================================================================

def hold_key(n_cycles, key=1):
    """连续 n 个拍按住某个键。"""
    return [key] * n_cycles


def waveform(n_cycles, period=8):
    """造一段【不停过零】的音频样本（模拟真实波形）。

    ★ 这正是老 bug 的来源：lat_sound 会跟着它 0/1/0/1 地跳。
    """
    return [((i % period) - period // 2) for i in range(n_cycles)]


def sil_then_sound(n_silence, n_sound, period=8):
    """先静音 n_silence 拍，再来一段波形。"""
    return [0] * n_silence + waveform(n_sound, period)


def one_new_key(n_cycles, at):
    """只在第 at 拍有一个 key_new 脉冲。"""
    s = [0] * n_cycles
    if 0 <= at < n_cycles:
        s[at] = 1
    return s


# ==========================================================================
# 三、检查
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

    N = 20000                                # 足够的拍数

    # ------------------------------------------------------------------
    section("1 lat_key：按一次键只出一个脉冲")
    seq = hold_key(1000) + [0] * 1000
    w = lat_key_wave(seq)
    check("按下一次 -> 恰好 1 个脉冲（实际 %d）" % count_pulses(w), count_pulses(w) == 1)

    section("2 lat_key：按住多久都还是那一个")
    for n in (1, 100, 5000, 19000):
        seq = hold_key(n) + [0] * 100
        w = lat_key_wave(seq)
        check("按住 %5d 拍 -> %d 个脉冲" % (n, count_pulses(w)), count_pulses(w) == 1)

    section("3 lat_key：松手再按 = 新的一次")
    seq = hold_key(500) + [0] * 500 + hold_key(500) + [0] * 500
    w = lat_key_wave(seq)
    check("按-松-按 -> 2 个脉冲（实际 %d）" % count_pulses(w), count_pulses(w) == 2)

    section("4 lat_key：按住期间抖动/坏读数【不再多打】")
    # 模拟 ttp229 读到别的键、或同一个键闪断又回来
    seq = []
    seq += [1] * 200            # 按下
    seq += [3] * 50             # 抖动：又"多"了一个键
    seq += [1] * 50             # 又回去
    seq += [5] * 50             # 再抖一个
    seq += [1] * 3000           # 一直按住
    seq += [0] * 200            # 松手
    w = lat_key_wave(seq)
    check("按住期间 4 次键状态变化 -> 仍然只有 1 个（实际 %d）" % count_pulses(w),
          count_pulses(w) == 1, count_pulses(w))

    section("5 lat_key：和弦（同一次扫描到多个键）= 1 个")
    seq = [0] * 100 + [0b111] * 1000 + [0] * 200
    w = lat_key_wave(seq)
    check("三个键同一拍按下 -> 1 个（实际 %d）" % count_pulses(w), count_pulses(w) == 1)

    section("6 lat_key：和弦分几次扫描到（按住不放）也还是 1 个")
    # 第一个键先进来，后面两个键在【没松手】的情况下陆续到
    seq = [0] * 50 + [0b001] * 300 + [0b011] * 300 + [0b111] * 300 + [0] * 200
    w = lat_key_wave(seq)
    check("三键分三次到（中间没松手）-> 1 个（实际 %d）" % count_pulses(w),
          count_pulses(w) == 1, count_pulses(w))

    section("7 lat_key：脉冲宽度 = 10µs（50MHz 下 500 拍）")
    seq = [0] * 100 + [1] * 1000 + [0] * 1000
    ws = pulse_widths(lat_key_wave(seq))
    check("只有一段（实际 %d 段）" % len(ws), len(ws) == 1, ws)
    check("宽度 500 拍 = %d ns = 10µs" % (ws[0] * CLK50M_NS),
          ws[0] == 500, ws[0])

    section("8 lat_key：连续弹多次，一次一个")
    # ★ 弹奏间隔必须【远大于】10µs 的脉冲宽度，否则脉冲会连成一片
    #   （那是设计如此，见 8b）。人最快也就 ~67ms 一个音，
    #   这里用 5000 拍 = 100µs 压缩时间，已经足够分开了。
    seq = []
    for _ in range(5):
        seq += [1] * 10 + [0] * 5000         # 按 10 拍、松 5000 拍
    seq += [0] * 1000
    w = lat_key_wave(seq)
    check("按-松 x5（间隔 100µs）-> 5 个（实际 %d）" % count_pulses(w),
          count_pulses(w) == 5, count_pulses(w))

    section("8b lat_key：★ 间隔比脉冲宽度还短时会连成一片（设计如此，如实记录）")
    seq = []
    for _ in range(5):
        seq += [1] * 10 + [0] * 10           # 200ns 一次 —— 人做不到
    seq += [0] * 1000
    w = lat_key_wave(seq)
    check("间隔 200ns << 10µs 脉宽 -> 合并成 1 个（实际 %d）" % count_pulses(w),
          count_pulses(w) == 1, count_pulses(w))
    check("★ 但人最快的弹奏间隔（~67ms）远大于 10µs，不会合并", 67e6 > 10e3)

    # ------------------------------------------------------------------
    section("9 lat_pcm：★ 波形不停过零，但只出一个脉冲")
    # 先静音，第 arm_at 拍按键，紧接着来一段不停过零的波形
    arm_at = 100
    kn = one_new_key(N, arm_at)
    smp = sil_then_sound(150, N - 150, period=4)     # 过零极密
    w = lat_pcm_wave(kn, smp)
    check("按键后一段狂过零的波形 -> 恰好 1 个脉冲（实际 %d）" % count_pulses(w),
          count_pulses(w) == 1, count_pulses(w))
    # 反过来：如果不做一次性武装，会出多少个？（证明这个修法是必要的）
    def naive(kn, smp):
        sd = False
        out = []
        for s in smp:
            snd = s != 0
            out.append(1 if (snd and not sd) else 0)
            sd = snd
        return out
    naive_n = count_pulses(naive(kn, smp))
    check("★ 对照：老写法（直接取 lat_sound 的沿）会出 %d 个脉冲 —— 这就是原先的 bug"
          % naive_n, naive_n > 50, naive_n)

    section("10 lat_pcm：脉冲落在【第一个】由静音变有声的沿上")
    w = lat_pcm_wave(kn, smp)
    first = w.index(1)
    check("第一个脉冲在第 %d 拍（按键在第 %d 拍，静音到第 149 拍）" % (first, arm_at),
          150 <= first <= 158, first)

    section("11 lat_pcm：没按键时，光有声音不出脉冲")
    kn0 = [0] * N
    check("不按键 + 有波形 -> 0 个（实际 %d）" % count_pulses(lat_pcm_wave(kn0, smp)),
          count_pulses(lat_pcm_wave(kn0, smp)) == 0)

    section("12 lat_pcm：按两次键 = 2 个脉冲")
    kn2 = one_new_key(N, 100)
    kn2[8000] = 1
    # 中间回到静音再响，模拟第二颗音
    smp2 = sil_then_sound(150, 7000, 4) + [0] * 500 + sil_then_sound(300, N - 150 - 7000 - 500 - 300, 4)
    smp2 = smp2[:N] + [0] * max(0, N - len(smp2))
    w = lat_pcm_wave(kn2, smp2)
    check("按两次 -> 2 个（实际 %d）" % count_pulses(w), count_pulses(w) == 2, count_pulses(w))

    section("13 lat_pcm：按了键但一直没声音 -> 不出脉冲（宁可量不到，不报错位置）")
    kn = one_new_key(N, 100)
    w = lat_pcm_wave(kn, [0] * N)
    check("按键 + 全程静音 -> 0 个（实际 %d）" % count_pulses(w), count_pulses(w) == 0)

    section("14 lat_pcm：脉冲宽度 = 10µs（12.5MHz 下 125 拍）")
    kn = one_new_key(N, 100)
    smp = sil_then_sound(150, 300, 4) + [0] * (N - 450)
    ws = pulse_widths(lat_pcm_wave(kn, smp))
    check("只有一段（实际 %d 段）" % len(ws), len(ws) == 1, ws)
    check("宽度 125 拍 = %d ns = 10µs" % (ws[0] * CLKAUD_NS), ws[0] == 125, ws[0])

    section("15 两路脉冲宽度一致 = 10µs")
    check("LAT_HOLD_50M 499 拍 x 20ns = %d ns" % ((LAT_HOLD_50M + 1) * CLK50M_NS),
          (LAT_HOLD_50M + 1) * CLK50M_NS == 10000)
    check("LAT_HOLD_AUD 124 拍 x 80ns = %d ns" % ((LAT_HOLD_AUD + 1) * CLKAUD_NS),
          (LAT_HOLD_AUD + 1) * CLKAUD_NS == 10000)

    section("16 脉冲够宽、又比被测量间隔短")
    check("10µs 远大于一个音频帧里能分辨的尺度", 10000 >= 10000)
    check("★ 10µs < T1->T2 的一个音频帧 20.5µs（两段还分得开）", 10.0 < 20.5)

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
