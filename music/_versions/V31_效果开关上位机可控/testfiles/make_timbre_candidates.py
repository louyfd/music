# -*- coding: utf-8 -*-
"""make_timbre_candidates.py —— 生成"候选音色"对照 MIDI，让人耳挑

为什么要有这个脚本
------------------
板子的默认音色是 sine_rom 那张 256 点整周期正弦表出来的【纯正弦】。
GM 里哪个音色最像"柔和的正弦"，光看名字是判断不了的 —— 得听。
所以这里把同一句用各个候选音色依次弹一遍，生成一个 MIDI：

    python testfiles\\make_timbre_candidates.py

产物落在 D:\\CODE\\SPP_txandrx\\MIDI\\_候选音色.mid，每段前面有 marker
（播放器一般能看到），段与段之间隔 1 秒，直接双击用系统播放器听。

听完把编号告诉开发（或直接改 D:\\CODE\\SPP_txandrx\\fpga_rec.py 里的
FX_LEAD_PROG / FX_TIMBRE_PROG / FX_FLUTE_PROG 三个常数）。

★ 顺序要和脚本打印出来的编号一致 —— 报告编号时按打印的来。
"""

from __future__ import annotations

import io
import os
import sys

sys.path.insert(0, r"D:\CODE\SPP_txandrx")

MIDI_DIR = r"D:\CODE\SPP_txandrx\MIDI"
OUT = os.path.join(MIDI_DIR, "_候选音色.mid")

# (GM 程序号, 显示名, 说明)
CANDIDATES = [
    (82, "Lead 3 (Calliope)", "正弦感口哨 —— 当前默认"),
    (79, "Ocarina",           "基波+2次谐波 —— 当前音色档"),
    (73, "Flute",             "长笛 —— 当前笛子档"),
    (16, "Drawbar Organ",     "管风琴，起音干脆、很柔"),
    (89, "Pad 2 (Warm)",      "暖垫，更柔但起音偏慢"),
    (74, "Pan Flute",         "排箫"),
    (19, "Church Organ",      "教堂管风琴"),
    (0,  "Acoustic Grand Piano", "钢琴 —— 反面教材：自带衰减，按住会越来越轻"),
    (80, "Lead 1 (Square)",   "方波 —— 之前那版，尖"),
    (81, "Lead 2 (Sawtooth)", "锯齿 —— 之前那版音色档，更尖"),
]

# 同一句：C4 E4 G4 C5，每个按住 350ms（和真人弹的时长量级一致）
PHRASE = [60, 64, 67, 72]
HOLD_MS = 350.0
GAP_MS = 150.0
SECTION_GAP_MS = 1000.0


def main():
    try:
        import mido
    except ImportError:
        print("需要 mido")
        return 1

    tpb = 480
    bpm = 100
    ms_per_tick = (60_000.0 / bpm) / tpb

    def tk(ms):
        return int(round(ms / ms_per_tick))

    raw = []                                    # (tick, 次序, 消息)
    t = 0.0
    print("候选顺序（报告编号时按这个来）：")
    for i, (prog, name, note) in enumerate(CANDIDATES, 1):
        print("  %2d. program %-3d  %-24s %s" % (i, prog, name, note))
        raw.append((tk(t), 0, mido.MetaMessage("marker", text="%d %s" % (i, name))))
        raw.append((tk(t), 0, mido.Message("program_change", program=prog)))
        for n in PHRASE:
            # 和板子一致：按住 HOLD_MS，再拖一条释音尾巴（这里统一给 1.2 秒听个大概）
            raw.append((tk(t + HOLD_MS + 1200), 2,
                        mido.Message("note_off", note=n, velocity=0)))
            raw.append((tk(t), 3, mido.Message("note_on", note=n, velocity=80)))
            t += HOLD_MS + GAP_MS
        # 每段之间把上一个音彻底放干净，再静一会儿
        raw.append((tk(t + 1200), 0, mido.MetaMessage("marker", text="---")))
        t += 1200 + SECTION_GAP_MS

    raw.sort(key=lambda x: (x[0], x[1]))
    mid = mido.MidiFile(ticks_per_beat=tpb)
    tr = mido.MidiTrack()
    mid.tracks.append(tr)
    tr.append(mido.MetaMessage("set_tempo", tempo=int(round(60_000_000.0 / bpm)), time=0))
    prev = 0
    for tick, _order, msg in raw:
        msg.time = max(0, tick - prev)
        prev = max(prev, tick)
        tr.append(msg)

    if not os.path.isdir(MIDI_DIR):
        os.makedirs(MIDI_DIR, exist_ok=True)
    mid.save(OUT)
    print()
    print("已写入: %s" % OUT)
    print("总长约 %.1f 秒" % (t / 1000.0))
    print("双击用系统播放器听，或在这个助手里的 MIDI 目录选它。")
    return 0


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                            # noqa: BLE001
        pass
    sys.exit(main())
