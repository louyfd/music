# make_star_midi.py
"""
生成《小星星》(Twinkle Twinkle Little Star) 的 MIDI 文件
文件名: star.mid
"""

import mido
import os

try:
    _HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _HERE = r"D:\FPGA_25K\bluetooth"

MIDI_FILE = os.path.join(_HERE, "star.mid")

# ============================================================
# 配置
# ============================================================
# 你的解析代码里 NOTE_OFFSET = 48，键号 0~35 对应 MIDI 48~83
# 小星星主旋律音域：C4(60) ~ A4(69)，完全落在范围内 ✓
NOTE_OFFSET = 48

TEMPO_BPM = 100          # 速度
TICKS_PER_BEAT = 480     # 每拍的分辨率

# 音名 -> MIDI 音高
NOTE_MAP = {
    'C4': 60, 'D4': 62, 'E4': 64, 'F4': 65, 'G4': 67, 'A4': 69, 'B4': 71,
    'C5': 72,
    'R':  None,   # 休止符
}

# ============================================================
# 小星星简谱（每个元素 = (音名, 拍数)）
# 4/4 拍：1 = 四分音符 = 1拍
# ============================================================
MELODY = [
    # 第一句: 一闪一闪亮晶晶
    ('C4', 1), ('C4', 1), ('G4', 1), ('G4', 1),
    ('A4', 1), ('A4', 1), ('G4', 2),

    # 第二句: 满天都是小星星
    ('F4', 1), ('F4', 1), ('E4', 1), ('E4', 1),
    ('D4', 1), ('D4', 1), ('C4', 2),

    # 第三句: 挂在天上放光明
    ('G4', 1), ('G4', 1), ('F4', 1), ('F4', 1),
    ('E4', 1), ('E4', 1), ('D4', 2),

    # 第四句: 好像许多小眼睛
    ('G4', 1), ('G4', 1), ('F4', 1), ('F4', 1),
    ('E4', 1), ('E4', 1), ('D4', 2),

    # 第五句: 一闪一闪亮晶晶
    ('C4', 1), ('C4', 1), ('G4', 1), ('G4', 1),
    ('A4', 1), ('A4', 1), ('G4', 2),

    # 第六句: 满天都是小星星
    ('F4', 1), ('F4', 1), ('E4', 1), ('E4', 1),
    ('D4', 1), ('D4', 1), ('C4', 2),
]


def build_midi():
    mid = mido.MidiFile(ticks_per_beat=TICKS_PER_BEAT)
    track = mido.MidiTrack()
    mid.tracks.append(track)

    # 速度
    tempo = mido.bpm2tempo(TEMPO_BPM)
    track.append(mido.MetaMessage('set_tempo', tempo=tempo, time=0))
    track.append(mido.MetaMessage('time_signature', numerator=4, denominator=4, time=0))

    ticks_per_note = TICKS_PER_BEAT  # 1拍

    for name, beats in MELODY:
        note = NOTE_MAP[name]
        duration = int(ticks_per_note * beats)

        if note is None:
            # 休止符：直接推进时间
            track.append(mido.Message('note_off', note=0, velocity=0, time=duration))
            continue

        # note_on（力度 100）
        track.append(mido.Message('note_on', note=note, velocity=100, time=0))
        # note_off
        track.append(mido.Message('note_off', note=note, velocity=0, time=duration))

    mid.save(MIDI_FILE)
    print(f"[OK] 已生成 {MIDI_FILE}")
    print(f"     共 {len(MELODY)} 个音符事件，BPM={TEMPO_BPM}")


if __name__ == '__main__':
    build_midi()
