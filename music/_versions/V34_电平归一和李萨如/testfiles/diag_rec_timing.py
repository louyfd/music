# -*- coding: utf-8 -*-
"""diag_rec_timing.py —— 录音回传的时间轴诊断

用户报的现象：合成出来的比实际演奏的【慢】，间隔不对。

这个脚本把整条链路走一遍并逐步打印时间轴，看清楚时间是在哪一步丢的：
    真实演奏  ->  板子 sounding_mask(含 ADSR)  ->  10ms 采样 + dt
              ->  回传字节  ->  RecParser  ->  events_to_midi  ->  MIDI 文件

用法：python diag_rec_timing.py
"""

from __future__ import annotations

import io
import os
import sys

sys.path.insert(0, r"D:\CODE\SPP_txandrx")

import fpga_rec as R                                                  # noqa: E402
import score_jp as JP                                                 # noqa: E402

# ---- 和 top.v 一致的 ADSR 参数 ----
SR = 48828.125
ENV_RELEASE_DIV = 280
ENV_SUSTAIN_LV = 255
GATE_DEBOUNCE_MS = 8.0
RELEASE_MS = 255 * ENV_RELEASE_DIV / SR * 1000.0        # ≈ 1462 ms
SAMPLE_MS = 10.0                                        # 板子的采样周期


def sounding_intervals(presses):
    """[(按下ms, 松开ms, 键号)] -> [(起ms, 止ms, 键号)] 真正在响的区间。

    env 从按下就非 0；松开后要等 8ms 消抖 + 1.46s 线性衰减才归 0。
    """
    out = []
    for t_on, t_off, k in presses:
        out.append((t_on, t_off + GATE_DEBOUNCE_MS + RELEASE_MS, k))
    return out


def gate_intervals(presses):
    """门控区间 = 手指按着的那段（板子回传的就是它）。

    ★ 这就是 top.v 的 sounding_mask_aud = key_mask | sustain_latch_aud。
      它没有释音尾巴；尾巴由上位机按 RTL 常数补。
    """
    return [(a, b, k) for a, b, k in presses]


def mask_at(t_ms, spans):
    m = 0
    for a, b, k in spans:
        if a <= t_ms < b:
            m |= 1 << k
    return m


def board_events(spans, t_end, fixed=True):
    """按板子 RTL 的采样规则产出 (t_ms, mask, dt)。

    RTL（修正后）：
        if (!rec_on) { rec_tick<=0; rec_dt<=0; rec_mask_r<=mask; rec_force<=1; }
        else if (rec_tick >= DIV-1) begin
            rec_tick <= 0;
            if (rec_force || mask != rec_mask_r) begin
                fire(dt = rec_force ? 0 : rec_dt+1);   // ★ 含这一拍
                rec_dt <= 0; rec_force <= 0;
            end else rec_dt <= rec_dt + 1;
        end

    fixed=False 复现旧版（少算这一拍），用来对照。
    """
    evs = []
    rec_mask_r = mask_at(0.0, spans)
    rec_dt = 0
    rec_force = True
    n = int(t_end / SAMPLE_MS) + 1
    for i in range(n):
        t = i * SAMPLE_MS
        if t == 0:
            continue                       # t=0 是"开录那一拍"，还没到采样边界
        cur = mask_at(t, spans)
        if rec_force or cur != rec_mask_r:
            dt = 0 if rec_force else (rec_dt + 1 if fixed else rec_dt)
            evs.append((t, cur, dt))
            rec_mask_r = cur
            rec_dt = 0
            rec_force = False
        else:
            rec_dt += 1
    return evs


def main():
    # ---- 1) 真实演奏：每 400ms 一个音，按住 250ms 就松 ----
    #   ★ 第一个音从 200ms 才开始 —— 现实里都是"先按录音、再弹"。
    #   ★ 第 3 和第 7 个音都是键 20（音高 62）：这两下之间只隔 1600ms，
    #     前一下的释音尾巴（1.46s）还没走完 —— 正是"重击同一个键"的考验。
    keys = [13, 17, 20, 22, 25, 22, 20, 17]          # C4 A3 D4 E4 C5 E4 D4 A3
    presses = [(200.0 + i * 400.0, 200.0 + i * 400.0 + 250.0, k) for i, k in enumerate(keys)]
    spans = sounding_intervals(presses)
    gates = gate_intervals(presses)
    print("=== 1) 真实演奏 ===")
    print("  按下/松开：%s ... 共 %d 个音，最后一个松手在 %.0fms"
          % (" ".join("%.0f-%.0f" % (a, b) for a, b, _k in presses[:3]),
             len(presses), presses[-1][1]))
    print("  含 ADSR 释音尾巴 %.0fms 后，真正在响的区间：" % RELEASE_MS)
    for a, b, k in spans[:3]:
        print("     键%-3d %.0f -> %.0f ms" % (k, a, b))
    print("     ...")
    print("  ★ 真实演奏【最后一个音真正静音】在 %.0f ms" % spans[-1][1])

    # ---- 2) 板子会报出什么 ----
    t_end = max(b for _a, b, _k in gates) + R.REL_MS + 200
    print()
    print("=== 2) dt 累加能不能还原真实时刻 ===")
    for tag, fixed in (("旧版(少算一拍)", False), ("修正后", True)):
        bevs = board_events(gates, t_end, fixed=fixed)
        acc = 0
        worst = 0.0
        for t, m, dt in bevs:
            acc += dt * 10
            worst = acc - t
        print("  %-14s 事件 %2d 个   最后一个：dt累加 %6.0f ms / 真实 %6.0f ms   差 %+6.0f ms"
              % (tag, len(bevs), acc, bevs[-1][0], worst))
    bevs = board_events(gates, t_end, fixed=True)
    print("  ★ 真实演奏（到最后一个音静音）%.0f ms" % spans[-1][1])
    print("  前 6 个事件的 mask/dt：")
    for t, m, dt in bevs[:6]:
        print("     真实 %7.1f ms  dt=%2d  mask=%d 位" % (t, dt, bin(m).count("1")))

    # ---- 3) 走回传字节 + 解析 ----
    prev = 0.0
    blob = R.pack_state(True) + R.pack_fx(0)
    for t, m, _dt in bevs:
        blob += R.pack_event(t - prev, m)
        prev = t
    blob += R.pack_state(False)
    rx = R.RecParser()
    rx.feed(blob)
    print()
    print("=== 3) RecParser 解析出来 ===")
    print("  事件数 %d，最后一个事件在 %.0f ms" % (len(rx.events), rx.events[-1][0]))
    print("  events[:6] = %s" % (rx.events[:6],))

    # ---- 4) 合成 MIDI，再把 MIDI 的时间反算回毫秒 ----
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_diag_rec.mid")
    n_on, dur = R.events_to_midi(rx.events, out, states=rx.states, bpm=100)
    print()
    print("=== 4) 合成出来的 MIDI ===")
    import mido
    mid = mido.MidiFile(out)
    # ★ mido 迭代 MidiFile 时已经把 time 换算成【秒】了（tempo 也吃进去了），
    #   这里不能再乘一次 tick2second —— 我第一版就是这么写错的，结果整轨都变成几毫秒。
    abs_ms = 0.0
    onsets, pairs = [], []
    open_at = {}                       # 音高 -> 还没配对的起音时刻
    for msg in mid:
        abs_ms += msg.time * 1000.0
        if msg.type == "note_on" and msg.velocity > 0:
            onsets.append((abs_ms, msg.note))
            open_at.setdefault(msg.note, []).append(abs_ms)
        elif msg.type == "note_off" or (msg.type == "note_on" and msg.velocity == 0):
            q = open_at.get(msg.note)
            if q:
                pairs.append((q.pop(0), abs_ms, msg.note))
    total = abs_ms
    print("  音符数 %d（events_to_midi 报 %d）" % (len(onsets), n_on))
    print("  起音时刻（MIDI 反算）:", " ".join("%.0f" % t for t, _n in onsets[:8]))
    print("  真实按下时刻          :", " ".join("%.0f" % p[0] for p in presses))
    print("  ★ MIDI 整轨总长 %.0f ms   /   真实演奏（到全部静音）%.0f ms"
          % (total, spans[-1][1]))

    # 每个音实际响了多久
    #   ★ 要拿真正的 note_off 来配对，不能拿"下一个同音高起音"当结尾 ——
    #     我第一版就是那么量的，结果每个音都显示成"一直响到最后"。
    print()
    print("=== 5) 每个音的长度：MIDI vs 真实（含 ADSR 尾巴） ===")
    truth = {}
    for a, b, k in spans:
        truth.setdefault(JP.key_to_midi(k), []).append((a, b))
    shown = sorted(set(list(truth) + [x[2] for x in pairs]))
    for n in shown[:6]:
        mid_s = " ".join("%.0f-%.0f(%.0fms)" % (a, b, b - a)
                         for a, b, _nn in sorted(pairs) if _nn == n)
        tru_s = " ".join("%.0f-%.0f(%.0fms)" % (a, b, b - a)
                         for a, b in sorted(truth.get(n, [])))
        print("   音高 %3d  MIDI: %s" % (n, mid_s or "(无)"))
        print("             真实: %s" % (tru_s or "(无)"))
    os.remove(out)

    print()
    print("=" * 70)
    if onsets:
        err = [m - p[0] for (m, _n), p in zip(onsets, presses)]
        print("起音误差（MIDI - 真实）：%s" %
              " ".join("%+.0f" % e for e in err))
        print("总长比：MIDI %.0f ms / 真实 %.0f ms = %.2fx"
              % (total, spans[-1][1], total / spans[-1][1]))
    return 0


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                          # noqa: BLE001
        pass
    sys.exit(main())
