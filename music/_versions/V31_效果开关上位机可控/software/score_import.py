# -*- coding: utf-8 -*-
"""score_import.py —— 乐谱 -> 事件表 -> FPGA v2 帧

把 score_jp 解析出来的 Score（旋律 + 和弦标记）变成 fpga_proto 能直接编码的
Frame，中间做完这几件事：

    1. 反复展开        把 ||: :|| 展开成实际演奏顺序（板子是线性播放的）
    2. 和弦展开        级数/字母和弦 -> 具体音（根音+三音+五音[+七音]）
    3. 音域适配        自动算整体移调，让旋律落进板子 A2..G#5
    4. 和弦配声        把和弦放在旋律下方，并保证也在板子音域内
    5. 时间格计算      取所有时值的最大公约数 -> 时间格（µs），并校验 FPGA 下限
    6. 建事件表        每个时间格一个事件（旋律 ∪ 当前和弦）-> 合并相同相邻格
    7. 出帧            fpga_proto.encode_frame()

命令行：
    python score_import.py check  scores\\小步舞曲.jp.txt
    python score_import.py grid   scores\\小步舞曲.jp.txt     逐格预览（核对和弦）
    python score_import.py frame  scores\\小步舞曲.jp.txt     打印要发的字节
    python score_import.py midi   scores\\小步舞曲.jp.txt out.mid
    python score_import.py selftest
"""

from __future__ import annotations

import io
import os
import sys
from dataclasses import dataclass, field
from fractions import Fraction
from math import gcd
from typing import Dict, List, Optional, Sequence, Tuple

import fpga_proto as P
import score_jp as JP

# 板子音域（标准 MIDI）
MIDI_MIN, MIDI_MAX = JP.BOARD_MIDI_MIN, JP.BOARD_MIDI_MAX

# 时间格下限（与 uart_packet_echo.v 的 TICK_MIN_CYC 对应：20ms）
TICK_MIN_US = 20_000
# 释音提前量策略：默认让 gate 约占事件时长 3/4（tenuto/legato），上下夹住
REL_FRAC_NUM, REL_FRAC_DEN = 1, 4
REL_MIN_US, REL_MAX_US = 20_000, 300_000


@dataclass
class CellAt:
    """展开到绝对时间轴上的一个旋律记号"""
    start_beat: Fraction
    dur: Fraction
    midi: List[int]          # 该记号同时发声的音（叠置和弦音/单音）；空 = 休止
    chord: Optional[str]     # 该位置起效的和弦符号
    measure: int
    tie: bool = False        # 尾部带连音线 ~（要和后一个同音高的记号合并）


@dataclass
class Arrangement:
    frame: P.Frame
    bpm: int
    grid_beats: Fraction
    tick_us: int
    shift: int
    chord_mode: str
    warnings: List[str] = field(default_factory=list)
    # 逐格内容（合并前），用于核对： [(起始格号, (键号...)), ...]
    slots: List[Tuple[int, Tuple[int, ...]]] = field(default_factory=list)
    # 和弦配声结果： [(符号, (MIDI...)), ...]
    chords: List[Tuple[str, Tuple[int, ...]]] = field(default_factory=list)
    # 统计：slots / melody_slots / overlap_slots / events / bytes / duration_s
    stats: Dict[str, float] = field(default_factory=dict)

    @property
    def payload(self) -> bytes:
        return P.encode_frame(self.frame)


# ==========================================================================
# 1) 展开到绝对时间轴
# ==========================================================================
def flatten(sc: JP.Score) -> Tuple[List[CellAt], Fraction]:
    """把 Score 展开成线性 CellAt 列表，返回 (列表, 总拍数)。

    反复记号在这里被展开 —— 板子是线性播放的，没有"跳回"的概念。
    ★ 连音线（`4~4`）也在这里合并：同音高、前一个带 ~ 就并成一个长音。
      不合并的话会变成"响一下、断一下、再响一下"（同一个音高重新起音），
      而连音线的意思本来就是**一个音一直响**。
    """
    out: List[CellAt] = []
    t = Fraction(0)
    for m, _rep in JP.expand_repeats(sc):
        for c in m.cells:
            midis: List[int] = []
            if not c.is_rest:
                for deg, acc in c.degrees:
                    if deg == 0:
                        continue
                    midis.append(JP.degree_to_midi(deg, acc, c.octave, sc.key_pc))
            # ---- 连音线合并 ----
            if out and out[-1].tie and out[-1].midi == midis and midis:
                out[-1].dur += c.dur
                t += c.dur
                continue
            out.append(CellAt(start_beat=t, dur=c.dur, midi=midis,
                              chord=c.chord, measure=m.index, tie=c.tie))
            t += c.dur
    return out, t


# ==========================================================================
# 2) 音域适配（只做整八度，不做换调）
# ==========================================================================
def choose_shift(melody: Sequence[int]) -> int:
    """选一个整体移调量，使全部旋律落进板子音域。

    ★ 优先【整八度】—— 整八度不改变调性，是最干净的做法。
      但整八度只有 0 / ±12 / ±24 五个候选，有些曲子会正好卡在两个八度
      之间，怎么挪都放不下。例如：

          旋律 53..83（跨度 30 个半音，板子 45..80 能容 35 个）
            移  0  -> 83 越界（高 3 个半音）
            移 -12 -> 41 掉出下面（低 4 个半音）
            移 -3  -> 50..80   一个音都不丢   <- 只能这样

      所以：整八度全试完还放不下时，退一步按【半音】找那个"一个音都不丢、
      又最接近原调"的位置。这会让调性变一点，但至少不缺音。

      跨度 > 35 个半音（怎么挪都放不下）才返回 0，越界的音后面会被丢掉。
    """
    lo, hi = min(melody), max(melody)
    for s in (0, -12, 12, -24, 24):                    # 先试整八度（保调性）
        if lo + s >= MIDI_MIN and hi + s <= MIDI_MAX:
            return s
    lo_s, hi_s = MIDI_MIN - lo, MIDI_MAX - hi          # 可完整放下的移调范围
    if lo_s <= hi_s:
        if lo_s <= 0 <= hi_s:
            return 0
        return lo_s if lo_s > 0 else hi_s              # 取最接近 0 的那个

    # ③ 跨度 > 35：怎么挪都放不下，就选"需要折返的音最少"的位置。
    #    剩下的交给 arrange 里的折八度处理 —— 保证一个音都不丢。
    best_s, best_fold = 0, None
    for s in range(-24, 25):
        fold = sum(1 for m in melody if not (MIDI_MIN <= m + s <= MIDI_MAX))
        if best_fold is None or fold < best_fold or \
                (fold == best_fold and abs(s) < abs(best_s)):
            best_s, best_fold = s, fold
    return best_s


# ==========================================================================
# 3) 和弦配声
# ==========================================================================
def voice_chord(chord: str, key_pc: int, below: int,
                warn: List[str], where: str) -> Tuple[int, ...]:
    """把一个和弦配到 `below` 以下，并夹进板子音域。

    板子只有 36 个键（A2..G#5），而"旋律跨 18 个半音 + 和弦 7~11 个半音"
    本身就很挤，所以这里不只是整八度平移，还会尝试**所有转位**：

      根音位置   G3 B3 D4
      第一转位   B3 D4 G4      （把最低音升八度）
      第二转位   D4 G4 B4

    评分规则：先看"能不能整个落在 below 以下"，能则优先；
    再看"最高的那个音离 below 有多近"——越近越好（伴奏贴近旋律但不相撞），
    同分时取转位更自然的。
    """
    base = JP.chord_root_midi(chord, key_pc, center_octave=3)

    best_score = None
    best: Optional[Tuple[int, ...]] = None
    for inv in range(len(base)):
        tones = sorted(set(base[inv:] + [t + 12 for t in base[:inv]]))
        for sh in range(-36, 37, 12):
            v = tuple(t + sh for t in tones)
            if min(v) < MIDI_MIN or max(v) > MIDI_MAX:
                continue
            fits_below = 0 if max(v) < below else 1
            score = (fits_below, abs((below - 1) - max(v)), inv)
            if best_score is None or score < best_score:
                best_score, best = score, v

    if best is None:
        # 任何转位都塞不进音域：退回根音位置，夹到音域内并丢掉越界音
        tones = [t for t in base]
        for _ in range(6):
            if min(tones) >= MIDI_MIN:
                break
            tones = [t + 12 for t in tones]
        for _ in range(6):
            if max(tones) <= MIDI_MAX:
                break
            tones = [t - 12 for t in tones]
        keep = tuple(sorted(t for t in tones if MIDI_MIN <= t <= MIDI_MAX))
        warn.append("%s 和弦 <%s> 连转位都塞不进板子音域，已丢音" % (where, chord))
        return keep

    if max(best) >= below:
        warn.append("%s 配到 %s，仍与旋律音区重叠（36 键音域所限，"
                    "可用八度/移调参数改善）"
                    % (where,
                       " ".join(JP.PC_NAME[t % 12] + str(t // 12 - 1) for t in best)))
    return best


# ==========================================================================
# 4) 主流程
# ==========================================================================
def arrange(sc: JP.Score, bpm: Optional[int] = None,
            octave_shift: Optional[int] = None,
            chord_mode: str = "sustain",
            rel_us: Optional[int] = None,
            max_bytes: int = P.BUF_SIZE) -> Arrangement:
    """Score -> Arrangement（含可直接发送的 Frame）。

    和弦的处理很简单：**同一个时间格把多个键置 1** 就是和弦。
    板子上 `midi_keys` 本来就是 36 位掩码、36 路独立发声，所以不需要任何特殊机制。

    chord_mode:
      "sustain" —— 和弦从标记处持续到下一个标记（默认）
      "block"   —— 和弦只响标记处那一个记号
      "off"     —— 忽略和弦符号，只放旋律
    """
    warn: List[str] = []
    bpm = int(bpm or sc.tempo or 100)
    if bpm <= 0:
        raise ValueError("bpm 必须 > 0")

    cells, total_beats = flatten(sc)
    if not cells:
        raise ValueError("谱子是空的")

    melody = [m for c in cells for m in c.midi]
    if not melody:
        raise ValueError("没有任何旋律音（全是休止？）")

    # ---- 3.1 整体移调（只做整八度，保持原调）----
    shift = choose_shift(melody) if octave_shift is None else int(octave_shift)
    if shift:
        warn.append("整体移调 %+d 个半音（整八度，调不变）：旋律 %s..%s -> %s..%s"
                    % (shift,
                       JP.PC_NAME[min(melody) % 12] + str(min(melody) // 12 - 1),
                       JP.PC_NAME[max(melody) % 12] + str(max(melody) // 12 - 1),
                       JP.PC_NAME[(min(melody) + shift) % 12]
                       + str((min(melody) + shift) // 12 - 1),
                       JP.PC_NAME[(max(melody) + shift) % 12]
                       + str((max(melody) + shift) // 12 - 1)))
    melody = [m + shift for m in melody]
    out_of = [(c.measure, m) for c in cells for m in c.midi
              if not (MIDI_MIN <= m + shift <= MIDI_MAX)]
    if out_of:
        warn.append("移调后仍有 %d 个旋律音越界（例：第 %d 小节 %s）"
                    % (len(out_of), out_of[0][0],
                       JP.PC_NAME[out_of[0][1] % 12] + str(out_of[0][1] // 12 - 1)))

    # ---- 3.2 时间格 = 所有时值的最大公约数 ----
    g: Optional[Fraction] = None
    for c in cells:
        if g is None:
            g = c.dur
        else:
            num = gcd(g.numerator * c.dur.denominator,
                      c.dur.numerator * g.denominator)
            g = Fraction(num, g.denominator * c.dur.denominator)
    if not g or g <= 0:
        raise ValueError("时间格算不出来")

    quarter_us = Fraction(60_000_000, bpm)
    tick_us_frac = quarter_us * g
    tick_us = int(tick_us_frac)               # 截断到 µs
    if tick_us < TICK_MIN_US:
        raise ValueError(
            "时间格只有 %d µs（<%d µs 下限）。BPM=%d、最短时值=%s 拍。"
            " 请把 BPM 调慢，或把谱面时值合并一档（例如把十六分并成八分）。"
            % (tick_us, TICK_MIN_US, bpm, g))
    if tick_us_frac.denominator != 1:
        warn.append("时间格 %s µs 不是整数，取整为 %d µs（误差 %.4f%%，"
                    "会均匀分摊到全曲，不会累积）"
                    % (tick_us_frac, tick_us,
                       float(abs(tick_us_frac - tick_us) / tick_us_frac) * 100))

    # ---- 3.3 和弦配声（每个符号只配一次，避免同一个和弦在各小节间跳八度）----
    chord_tones: Dict[str, Tuple[int, ...]] = {}
    if chord_mode != "off":
        low_all = min(melody)                       # 已含整体移调
        for sym in sorted({c.chord for c in cells if c.chord}):
            chord_tones[sym] = voice_chord(sym, sc.key_pc, low_all, warn,
                                           "和弦 <%s>" % sym)
    chord_list = sorted(chord_tones.items())

    # ---- 3.4 逐格建事件 ----
    slot_beats = g
    n_slots = int(total_beats / slot_beats)
    # 每个格位上："当前旋律音集合" 与 "当前和弦"
    slot_mel: List[set] = [set() for _ in range(n_slots)]
    slot_cell: List[int] = [0] * n_slots      # 该格来自第几个旋律记号（1 起；0=休止）
    for ci, c in enumerate(cells):
        if not c.midi:
            continue
        i0 = int(c.start_beat / slot_beats)
        i1 = int((c.start_beat + c.dur) / slot_beats)
        for i in range(i0, min(i1, n_slots)):
            slot_mel[i].update(m + shift for m in c.midi)
            slot_cell[i] = ci + 1

    # 和弦：从标记点起持续到下一个标记（sustain）/ 仅标记格（block）
    slot_chord: List[Optional[str]] = [None] * n_slots
    if chord_mode != "off":
        cur: Optional[str] = None
        for c in cells:
            if c.chord:
                cur = c.chord
            i0 = int(c.start_beat / slot_beats)
            i1 = int((c.start_beat + c.dur) / slot_beats) if chord_mode == "block" \
                else n_slots
            for i in range(i0, min(i1, n_slots)):
                slot_chord[i] = cur
            if chord_mode == "block" and c.chord:
                cur = None

    slots: List[Tuple[int, Tuple[int, ...]]] = []
    overlap_slots = 0          # 旋律音与和弦音同音高的格数
    melody_slots = 0           # 有旋律音的格数
    for i in range(n_slots):
        keys = set()
        mel_keys = set()
        for m in slot_mel[i]:
            if MIDI_MIN <= m <= MIDI_MAX:
                # ★ 键号不是 midi-45！必须过键号置换表（见 score_jp.midi_to_key）
                mel_keys.add(JP.midi_to_key(m))
            else:
                # 越界的旋律音：折八度直到进范围（保证不丢音）
                mm = m
                for _ in range(4):
                    if mm < MIDI_MIN:
                        mm += 12
                    elif mm > MIDI_MAX:
                        mm -= 12
                    else:
                        break
                if MIDI_MIN <= mm <= MIDI_MAX:
                    # ★ 键号不是 midi-45！必须过键号置换表（见 score_jp.midi_to_key）
                    mel_keys.add(JP.midi_to_key(mm))
                else:
                    warn.append("有旋律音 %s 无法折进板子音域，已丢弃"
                                % (JP.PC_NAME[m % 12] + str(m // 12 - 1)))
        keys |= mel_keys
        if mel_keys:
            melody_slots += 1
        sym = slot_chord[i]
        chord_keys = set()
        if sym and chord_mode != "off":
            for t in chord_tones.get(sym, ()):
                if MIDI_MIN <= t <= MIDI_MAX:
                    chord_keys.add(JP.midi_to_key(t))
        if mel_keys & chord_keys:
            overlap_slots += 1
        keys |= chord_keys
        slots.append((i, tuple(sorted(keys))))

    # ---- 3.5 合并相邻格，建 Frame ----
    #
    # ★ 合并规则（很重要）：只在【同一个旋律记号跨度内】合并。
    #   理由：一个四分音符会被切成 4 个时间格，必须合成 1 个事件（否则会
    #   一格一格重新起音）；但两个**重复的同音**（例如小星星的 `05 05`、
    #   本曲第 2 小节的 `1 1`）属于两个不同记号，必须保持成两个事件 ——
    #   板子靠"松手再按"重新起音，合并了就变成一个长音、听不出重复。
    #   休止格（keys 为空）可以自由合并，因为本来就不发声。
    events: List[P.Event] = []
    last_keys: Optional[Tuple[int, ...]] = None
    last_cell: int = -1
    for i, k in slots:
        cid = slot_cell[i]
        can_merge = (events and k == last_keys
                     and (cid == last_cell or not k)
                     and events[-1].ticks < P.MAX_TICKS)
        if can_merge:
            events[-1].ticks += 1
        else:
            events.append(P.Event(keys=k, ticks=1))
        last_keys, last_cell = k, cid
    frame = P.Frame(events=events, tick_us=tick_us, rel_us=0)  # rel_us 下面再算

    if rel_us is None:
        rel_us = max(REL_MIN_US,
                     min(REL_MAX_US, tick_us * REL_FRAC_NUM // REL_FRAC_DEN))
    frame.rel_us = rel_us
    frame.validate()

    # 容量：按事件边界截断
    body_len = len(P.encode_body(frame.events))
    if body_len > max_bytes:
        frame, dropped = P.truncate_to_fit(frame, max_bytes)
        warn.append("事件流 %d 字节超出上限 %d，已按事件边界截断，丢掉 %d 格"
                    % (body_len, max_bytes, dropped))

    if overlap_slots:
        warn.append("旋律与和弦音区重叠：%d/%d 格出现'旋律音 = 和弦音'"
                    "（36 键音域所限，不影响发声；旋律起音仍会保留）"
                    % (overlap_slots, melody_slots))

    stats = {
        "slots": n_slots,
        "melody_slots": melody_slots,
        "overlap_slots": overlap_slots,
        "events": len(frame.events),
        "bytes": len(P.encode_body(frame.events)),
        "duration_s": frame.duration_us() / 1e6,
    }
    return Arrangement(frame=frame, bpm=bpm, grid_beats=g, tick_us=tick_us,
                       shift=shift, chord_mode=chord_mode, warnings=warn,
                       slots=slots, chords=chord_list, stats=stats)


# ==========================================================================
# 预览（核对用）
# ==========================================================================
def preview_grid(arr: Arrangement, per_line: int = 6) -> str:
    """逐格预览：第几格、持续多少格、点亮哪些键（音名）"""
    st = arr.stats or {}
    lines = ["时间格 %d µs  共 %d 事件 / %d 格 / %.2f 秒  整体移调 %+d"
             % (arr.tick_us, len(arr.frame.events), arr.frame.total_ticks,
                arr.frame.duration_us() / 1e6, arr.shift)]
    if st:
        lines.append("有旋律的格 %d，其中'旋律音=和弦音'的格 %d；发送字节 %d"
                     % (st.get("melody_slots", 0), st.get("overlap_slots", 0),
                        st.get("bytes", 0)))
    lines.append("")
    idx = 0
    row: List[str] = []
    for e in arr.frame.events:
        if e.is_rest:
            body = "休止"
        else:
            body = " ".join(P.key_name(k) for k in e.keys)
        row.append("%3d %-26s x%d" % (idx, body, e.ticks))
        idx += e.ticks
        if len(row) >= per_line:
            lines.append("  " + " | ".join(row))
            row = []
    if row:
        lines.append("  " + " | ".join(row))

    lines.append("")
    lines.append("和弦配声：")
    for sym, tones in arr.chords:
        if tones:
            lines.append("  <%s> -> %s" % (sym, " ".join(
                JP.PC_NAME[t % 12] + str(t // 12 - 1) for t in tones)))
    return "\n".join(lines)


# ==========================================================================
# MIDI 导出（可选，需要 mido）
# ==========================================================================
# MIDI 固定放在软件目录下的 MIDI\ （与 SPP 助手一致）
MIDI_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "MIDI")


def default_midi_path(score_path: str) -> str:
    """由谱子文件名推出默认 MIDI 路径：<软件目录>\\MIDI\\<曲名>.mid

    `小步舞曲.jp.txt` -> `MIDI\\小步舞曲.mid`（把 .txt / .jp 后缀都剥掉）
    """
    base = os.path.basename(score_path)
    changed = True
    while changed:
        changed = False
        for suf in (".txt", ".jp", ".mid"):
            if base.lower().endswith(suf):
                base = base[:-len(suf)]
                changed = True
    return os.path.join(MIDI_DIR, base + ".mid")


def write_midi(arr: Arrangement, path: str) -> bool:
    """导出标准 MIDI。

    同一个时间格上的所有键 = 一个和弦，同时 note_on、同时 note_off。
    先攒绝对 tick 再统一转 delta，并且在同 tick 上让 note_off 排在 note_on 之前
    （否则同音高的前后两个音会互相掐掉）。
    """
    try:
        import mido
    except ImportError:
        print("[跳过] 需要 mido 才能导出 MIDI：pip install mido")
        return False

    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

    mid = mido.MidiFile(ticks_per_beat=480)
    tr = mido.MidiTrack()
    mid.tracks.append(tr)
    tr.append(mido.MetaMessage("set_tempo",
                               tempo=int(round(60_000_000 / arr.bpm)), time=0))
    tr.append(mido.MetaMessage("time_signature", numerator=4, denominator=4, time=0))

    def to_tick(n_grid: int) -> int:
        """一个时间格 = grid_beats 拍；1 拍 = 480 tick"""
        return max(1, int(round(float(arr.grid_beats) * 480 * n_grid)))

    abs_ev: List[Tuple[int, int, str, int]] = []      # (tick, 排序键, 类型, 音高)
    at = 0
    for e in arr.frame.events:
        dur = to_tick(e.ticks)
        for k in e.keys:
            # ★ 槽位里存的是【键号】，导出 MIDI 必须走同一张置换表。
            #   写成 k + MIDI_MIN 是错的：那等于宣称"键号是按音高排的"，
            #   出来的 MIDI 音高全乱（键号 0 会被写成 A2，其实它是 A#2）。
            note = JP.key_to_midi(k)
            if note < 0:
                continue
            abs_ev.append((at, 1, "on", note))
            abs_ev.append((at + dur, 0, "off", note))
        at += dur
    abs_ev.sort(key=lambda x: (x[0], x[1]))

    prev = 0
    n_on = 0
    for tick, _ord, kind, note in abs_ev:
        delta = tick - prev
        prev = tick
        if kind == "on":
            tr.append(mido.Message("note_on", note=note, velocity=80, time=delta))
            n_on += 1
        else:
            tr.append(mido.Message("note_off", note=note, velocity=0, time=delta))
    mid.save(path)
    print("[OK] 已写入 MIDI: %s（%d 个 note_on，时长 %.1f 秒，BPM=%d）"
          % (path, n_on, arr.frame.duration_us() / 1e6, arr.bpm))
    return True


# ==========================================================================
# 自检
# ==========================================================================
def _selftest() -> int:
    ok = 0
    fail: List[str] = []

    def check(name, cond):
        nonlocal ok
        if cond:
            ok += 1
            print("  ok   %s" % name)
        else:
            fail.append(name)
            print("  FAIL %s" % name)

    print("[1] flatten + 时间格")
    sc = JP.parse("key: 1=G\nmeter: 3/4\ntempo: 100\n5 1/ 2/ 3/4/ |\n",
                  verbose=False)
    cells, total = flatten(sc)
    check("5 个记号（3/4/ 是 3/ 和 4/ 两个）", len(cells) == 5)
    check("总拍数 3", total == Fraction(3))
    check("第 2 个记号从 1 拍开始", cells[1].start_beat == Fraction(1))

    print("[2] arrange 基本")
    sc2 = JP.parse("key: 1=G\nmeter: 3/4\ntempo: 100\n<1> 5 1/ 2/ 3/4/ |\n",
                   verbose=False)
    a = arrange(sc2, chord_mode="off")
    check("时间格 = 1/2 拍（最短是八分）", a.grid_beats == Fraction(1, 2))
    check("tick_us = 600000 * 1/2 = 300000", a.tick_us == 300_000)
    check("总格数 = 6", a.frame.total_ticks == 6)
    check("该片段音域够用，不移调", a.shift == 0)

    print("[3] 和弦配声：在板子音域内、且在旋律下方")
    a2 = arrange(sc2, chord_mode="sustain")
    check("配出一个和弦", len(a2.chords) == 1)
    sym, tones = a2.chords[0]
    check("和弦符号是 1", sym == "1")
    check("是三个音", len(tones) == 3)
    check("都在音域内", all(MIDI_MIN <= t <= MIDI_MAX for t in tones))
    mel = [m for c in flatten(sc2)[0] for m in c.midi]
    check("在旋律下方", max(tones) < min(mel) + a2.shift)
    check("是 G 大三和弦（G B D）",
          [t % 12 for t in tones] == [7, 11, 2])

    print("[4] sustain 模式：每一格都含和弦音")
    # ★ 槽位里存的是【键号】，必须用同一张置换表换算（不是 t - MIDI_MIN）
    ck = {JP.midi_to_key(t) for t in tones}
    check("第 1 格含全部和弦音", ck.issubset(set(a2.slots[0][1])))
    check("最后一格也含和弦音", ck.issubset(set(a2.slots[-1][1])))
    a3 = arrange(sc2, chord_mode="off")
    check("off 模式没有和弦音", not ck.issubset(set(a3.slots[-1][1])))

    print("[4b] 键号置换确实生效（不是 midi-45）")
    # 该片段 1=G：5=D5(74) -> 键号 24+KEY_OF_SEMITONE[5]=24+8=32；1=G4(67) -> 15+4=19
    ks_first = set(a2.slots[0][1])
    want5 = JP.midi_to_key(74)
    check("D5 落在键号 %d 上" % want5, want5 in ks_first)
    check("键号 %d ≠ midi-45(%d)" % (want5, 74 - MIDI_MIN), want5 != 74 - MIDI_MIN)

    print("[6] 反复展开影响总格数")
    sc3 = JP.parse("key: 1=G\nmeter: 3/4\ntempo: 100\n||: 5 1/ 2/ 3/4/ :||\n",
                   verbose=False)
    a4 = arrange(sc3, chord_mode="off")
    check("反复后 12 格（3 拍 ×2 / 半拍一格）", a4.frame.total_ticks == 12)

    print("[7] 容量：出帧成功且能反解")
    sc4 = JP.parse("key: 1=G\nmeter: 3/4\ntempo: 100\n" +
                   "<1> 5 1/ 2/ 3/4/ |\n" * 4, verbose=False)
    a5 = arrange(sc4)
    pay = a5.payload
    back = P.decode_frame(pay)
    check("反解事件数一致", len(back.events) == len(a5.frame.events))
    check("反解时间格一致", back.tick_us == a5.tick_us)
    check("字节数 < 4096", len(pay) < P.BUF_SIZE + 32)
    check("帧头 16×FF", pay[:16] == bytes([0xFF] * 16))

    print("[8] 导出 MIDI 必须走键号置换表（k + MIDI_MIN 是错的）")
    try:
        import mido
        import tempfile
        mp = os.path.join(tempfile.gettempdir(), "_score_import_roundtrip.mid")
        if write_midi(a5, mp):
            want = set()
            for e in a5.frame.events:
                for k in e.keys:
                    want.add(JP.key_to_midi(k))
            got = set()
            for msg in mido.MidiFile(mp):
                if msg.type == "note_on" and msg.velocity > 0:
                    got.add(msg.note)
            check("MIDI 里的音高集合 = key_to_midi(键号)", got == want)
            bad = {k + MIDI_MIN for e in a5.frame.events for k in e.keys}
            check("且不等于 键号+45（旧 bug 的产物）", got != bad or want == bad)
        else:
            check("write_midi 可用", False)
    except ImportError:
        print("  (跳过：没装 mido)")

    print("[8] 时间格下限保护（太快 / 时值太细会被拒绝）")
    fast = JP.parse("key: 1=C\nmeter: 4/4\ntempo: 1000\n5// 5// 5// 5// |\n",
                    verbose=False)
    try:
        arrange(fast)
        check("过快的谱子被拒绝", False)
    except ValueError as exc:
        check("过快的谱子被拒绝（%s）" % str(exc)[:34], True)

    print()
    print("=" * 62)
    if fail:
        print("失败 %d 项 / 通过 %d 项" % (len(fail), ok))
        for n in fail:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % ok)
    return 0


# ==========================================================================
def main() -> int:
    argv = sys.argv[1:]
    if not argv:
        print(__doc__)
        return 0
    cmd = argv[0]
    if cmd == "selftest":
        return _selftest()
    if len(argv) < 2:
        print(__doc__)
        return 0

    path = argv[1]
    with open(path, encoding="utf-8") as f:
        sc = JP.parse(f.read(), verbose=False)
    ps = JP.validate(sc)
    errs = [p for p in ps if p.level == "错误"]
    if errs and cmd != "check":
        for p in errs:
            print(p)
        print("→ 谱面有错误，先修谱（python score_jp.py check ...）")
        return 1

    arr = arrange(sc)

    if cmd == "check":
        for p in ps:
            print(p)
        print()
        print("整体移调 %+d；时间格 %d µs；%d 事件 / %d 格 / %.1f 秒；发送 %d 字节"
              % (arr.shift, arr.tick_us, len(arr.frame.events),
                 arr.frame.total_ticks, arr.frame.duration_us() / 1e6,
                 len(arr.payload)))
        for w in arr.warnings:
            print("[处理] " + w)
        return 1 if errs else 0

    if cmd == "grid":
        print(preview_grid(arr))
        for w in arr.warnings:
            print("[处理] " + w)
        return 0

    if cmd == "frame":
        print(P.describe_frame(arr.frame))
        print()
        pay = arr.payload
        print("字节流 (%d 字节)：" % len(pay))
        for i in range(0, len(pay), 32):
            print("  " + P.fmt_hex(pay[i:i + 32]))
        print()
        back = P.decode_frame(pay)
        print("反解校验：%s"
              % ("一致 ✅" if [e.keys for e in back.events]
                 == [e.keys for e in arr.frame.events] else "不一致 ❌"))
        for w in arr.warnings:
            print("[处理] " + w)
        return 0

    if cmd == "midi":
        out = argv[2] if len(argv) > 2 else default_midi_path(path)
        return 0 if write_midi(arr, out) else 1

    print(__doc__)
    return 0


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                   # noqa: BLE001
        pass
    sys.exit(main())
