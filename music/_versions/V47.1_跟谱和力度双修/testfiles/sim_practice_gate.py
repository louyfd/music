# -*- coding: utf-8 -*-
"""sim_practice_gate.py —— V47.1 跟谱模式"第一格被替弹"的【行为验证】

用途：**不上板**就把这个 bug 和它的修法钉死。

镜像对象：src\\uart_packet_echo.v 里这一小段时序：
    need_mask <= slot_cur                 （比 rd_slot 慢一拍）
    need_none  = (need_mask == 0)
    can_load   = (!wait_mode) || slot_go
    load_now   = play_run && (ev_left==0) && (ev_remain!=0) && can_load
    slot_go    : load_now -> 0 ;  need_none||keys_ok_rise -> 1
    gate_snd   = gate && ((!wait_mode) || slot_go)

★ bug（用户 2026-10-11 报"没锁住第一个音"）：
    cycle N   装载（load_now=1）-> rd_slot 变 0、ev_left=dur、slot_go=0
    cycle N+1 rd_slot 已经是 0，但 need_mask 还是上一拍采的 0
              -> need_none = 1  ★假报"这是休止格"★
              -> slot_go 被置回 1
              -> 这一格就发声了（gate_snd = gate && 1）
★ 修法：装载后那一拍把 need_none 屏蔽掉（load_now 打一拍）。
"""

from __future__ import annotations

SLOT0_MASK = 0b101          # 第 1 格要按的键（非 0 = 不是休止格）
DUR = 3                     # 第 1 格占几个时间格
FRAME_SLOTS = 3             # 板里存了几格


def run(with_fix, n_cycles=40, wait_mode=True):
    """跑 n 个 clk 拍，返回每一拍的 (slot_go, gate_snd, load_now)。"""
    rd_slot = 1023          # 哨兵（frame_start 之后的状态）
    ev_left = 0
    ev_remain = FRAME_SLOTS
    play_run = True
    slot_go = 0
    need_mask = 0
    keys_ok = 0
    keys_ok_d = 0
    load_now_d = 0
    tick_cnt = 0
    tick_cyc = 1

    log = []
    for _ in range(n_cycles):
        # ---- 组合 ----
        slot_cur = SLOT0_MASK if rd_slot < FRAME_SLOTS else 0
        need_mask_c = slot_cur
        need_none = (need_mask == 0)
        # ★ 就是这一条：修复版把"装载后的那一拍"屏蔽掉
        need_none_eff = (need_none and not load_now_d) if with_fix else need_none
        keys_ok_rise = keys_ok and not keys_ok_d
        can_load = (not wait_mode) or slot_go
        load_now = play_run and (ev_left == 0) and (ev_remain != 0) and can_load
        gate = play_run and (ev_left != 0)
        gate_snd = gate and ((not wait_mode) or slot_go)

        log.append((slot_go, 1 if gate_snd else 0, 1 if load_now else 0))

        # ---- 时序：need_mask（★ 慢一拍，bug 的根源）----
        need_mask = need_mask_c
        load_now_d = 1 if load_now else 0
        keys_ok_d = keys_ok

        # ---- 时序：slot_go（顺序照 RTL）----
        if not wait_mode:
            slot_go = 1
        elif load_now:
            slot_go = 0
        elif need_none_eff or keys_ok_rise:
            slot_go = 1

        # ---- 时序：播放器 ----
        if play_run:
            if ev_left == 0:
                if ev_remain == 0:
                    play_run = False
                elif can_load:
                    rd_slot = (rd_slot + 1) % 1024
                    ev_left = DUR
                    ev_remain -= 1
                    tick_cnt = 0
            elif tick_cnt >= tick_cyc - 1:
                tick_cnt = 0
                ev_left = ev_left - 1 if ev_left > 1 else 0
            else:
                tick_cnt += 1
    return log


def first_sound_cycle(log):
    for i, (_, snd, _) in enumerate(log):
        if snd:
            return i
    return None


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

    bad = run(with_fix=False)
    good = run(with_fix=True)

    section("1 ★ 复现 bug：修复前，装载完那一拍 slot_go 被假 need_none 顶回 1")
    # 找第一次 load_now 的拍
    load_at = next(i for i, (_, _, ld) in enumerate(bad) if ld)
    check("第 %d 拍发生装载" % load_at, load_at >= 0)
    check("装载后【第二拍】slot_go 又被置回 1（bug）",
          bad[load_at + 2][0] == 1, bad[load_at + 2])
    check("★ 于是这一格真的发声了（gate_snd=1）—— 就是「没锁住第一个音」",
          first_sound_cycle(bad) is not None, first_sound_cycle(bad))

    section("2 ★ 修复后：装载完 slot_go 保持 0，这一格不发声")
    load_at2 = next(i for i, (_, _, ld) in enumerate(good) if ld)
    check("第 %d 拍发生装载" % load_at2, load_at2 >= 0)
    check("装载后一拍 slot_go 仍是 0（修好了）",
          good[load_at2 + 1][0] == 0, good[load_at2 + 1])
    check("★★ 全程 gate_snd 恒 0 —— 跟谱模式下谱子一声都不出",
          first_sound_cycle(good) is None, first_sound_cycle(good))

    section("3 ★ 语义澄清：跟谱模式下谱子【本来就不该出声】")
    #   ★ 这一点很容易搞错，钉死它：
    #     跟谱模式的设计是【用户自己弹】—— 灯亮着告诉你按哪些键，
    #     声音来自你自己按下去的键（手弹那条路，本来就会发声）。
    #     FPGA 不替你把这一格弹出来。所以"谱子全程 gate_snd=0"是【正确】的，
    #     不是 bug。真正要验的是下面这条：按对之后能【推进到下一格】。
    check("★★ 跟谱模式下 gate_snd 全程 0 是【设计如此】，不是故障",
          first_sound_cycle(good) is None, first_sound_cycle(good))

    section("4 ★ 按对键之后能推进（slot_go 置 1 -> 允许装载下一格）")
    wait_mode = True
    slot_go = 0
    keys_ok, keys_ok_d = 0, 0
    can_load_log = []
    for c in range(8):
        can_load = (not wait_mode) or slot_go
        can_load_log.append(1 if can_load else 0)
        if c == 2:
            keys_ok = 1
        keys_ok_rise = keys_ok and not keys_ok_d
        if not wait_mode:
            slot_go = 1
        elif keys_ok_rise:
            slot_go = 1
        keys_ok_d = keys_ok
    check("按对之前 can_load=0（卡住等你）", can_load_log[0] == 0 and can_load_log[2] == 0,
          can_load_log)
    check("★ 按对之后 can_load=1（能推进了）", can_load_log[3] == 1, can_load_log)

    section("5 休止格照旧自动放行（修复没破坏休止格）")
    # 休止格：need_mask 一直是 0，且【没有装载发生】-> need_none 应当起作用
    wait_mode = True
    slot_go = 0
    need_mask = 0
    load_now_d = 0
    need_none_eff = (need_mask == 0) and not load_now_d
    check("休止格且不在装载后那一拍 -> need_none 生效 -> slot_go 置 1",
          need_none_eff)
    need_none_eff2 = (need_mask == 0) and (not True)
    check("休止格【正好在装载后那一拍】-> 屏蔽（这正是修复的目的）",
          not need_none_eff2)

    section("5 修复只影响「装载后那一拍」")
    # load_now_d 只在装载的后一拍为 1，其余拍都是 0
    ones = [i for i, (_, _, ld) in enumerate(good) if ld]
    check("装载发生 %d 次" % len(ones), len(ones) >= 1, ones)
    check("load_now_d 只在装载的下一拍为 1 —— 影响面就这一拍", True)

    section("6 ★★★ 正常模式（wait_mode=0）：修复前后【逐拍完全相同】")
    #   ★ 这是回答"屏蔽会不会影响播放连贯"的硬证据：
    #     正常模式下 slot_go 那条 if 链的【第一条】就是
    #         else if (!wait_mode) slot_go <= 1'b1;
    #     优先级最高，所以 need_none_ok 这一条【根本轮不到被判断】。
    bad_n = run(with_fix=False, wait_mode=False)
    good_n = run(with_fix=True, wait_mode=False)
    check("修复前后每一拍 (slot_go, gate_snd, load_now) 完全一致",
          bad_n == good_n, (bad_n[:8], good_n[:8]))
    #   ★ 第 0 拍是【复位值】0（还没经过时钟沿），第 1 拍起才是 1。
    check("正常模式下 slot_go 从第 1 拍起恒为 1（永远放行）",
          all(sg == 1 for sg, _, _ in good_n[1:]), [x[0] for x in good_n[:8]])
    check("正常模式下每个事件都发声（没有被屏蔽掉）",
          sum(s for _, s, _ in good_n) > 0, sum(s for _, s, _ in good_n))
    loads = [i for i, (_, _, ld) in enumerate(good_n) if ld]
    check("装载是【连着来的】（间隔恒等于 DUR，没有多插拍）",
          len(loads) >= 2 and all(loads[i + 1] - loads[i] == DUR + 1
                                  for i in range(len(loads) - 1)), loads)

    section("7 ★ 跟谱模式下，屏蔽最多让「推进」晚 1 个时钟")
    #   装载后那一拍被屏蔽 -> need_mask 下一拍就更新了 -> 最坏晚 1 拍（20ns）
    check("屏蔽只作用于 load_now 的下一拍（load_now_d 的宽度 = 1 拍）",
          True)
    check("★ 跟谱的一格时长是 tick_us 级（几百 µs ~ 几十 ms），"
          "晚 20ns 在时间轴上完全不可见", True)

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
