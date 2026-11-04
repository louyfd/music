# -*- coding: utf-8 -*-
"""fpga_midi.py —— MIDI 文件 -> FPGA 乐谱帧（uart_packet_echo.v 认的那套）

★ 为什么需要它 ★
    板子【不认裸 MIDI】。直接把 mido 的 msg.bytes() 发出去（90 32 50 / 80 32 00），
    FPGA 收到的是：
        0x90 / 0x32 / 0x50 / 0x80  -> 不是协议里任何字节 -> 丢
        0x00（note-off 的力度）    -> 看着像"键号 0"，但窗口没开 -> 丢
    结果就是一个字节都不进缓冲，一声不响。必须由上位机做这三件事：
        1) 把 Note On/Off 折成"每个时间格点亮哪些键"
        2) 把 MIDI 音符号换成【板子键号】（置换表，不是 MIDI-48）
        3) 套上帧头 0xAA、帧尾 0xFF

    本模块就是这三步。产出直接交给 fpga_proto.encode_frame()。

★ 与"简谱直转"（score_import.py）的分工 ★
    .jp.txt  -> score_import.arrange()  ：和弦是谱子上标好的，最准
    .mid     -> 本模块 midi_to_frame()  ：和弦 = 同一时刻起音的那几个音（通吃）

命令行：
    python fpga_midi.py selftest
    python fpga_midi.py show <file.mid> [--out frame.bin] [--rel-us N] [--tick-us N]
    python fpga_midi.py frame <file.mid> <out.bin>
"""

from __future__ import annotations

import io
import os
import sys
from dataclasses import dataclass, field
from fractions import Fraction
from math import gcd
from typing import Dict, List, Optional, Sequence, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fpga_proto as P                     # noqa: E402
import score_jp as JP                      # noqa: E402

BOARD_LO = JP.BOARD_MIDI_MIN               # 45 = A2
BOARD_HI = JP.BOARD_MIDI_MAX               # 80 = G#5
SPAN = BOARD_HI - BOARD_LO                 # 35 个半音
GAP_MIN_US = 10_000                        # RTL 里 GAP_MIN_CYC 对应的 10ms
TICK_MIN_US = P.TICK_MIN_US                # 20 ms
UART_BPS = 9600.0
UART_BYTE_S = 10.0 / UART_BPS


class MidiError(ValueError):
    pass


# ==========================================================================
# 1) MIDI 文件 -> 音符区间
# ==========================================================================
@dataclass
class Note:
    start: float
    end: float
    midi: int
    vel: int = 80

    @property
    def dur(self) -> float:
        return max(0.0, self.end - self.start)


def load_notes(path: str) -> Tuple[List[Note], dict]:
    """把 .mid 里所有音轨合成 (起, 止, 音符号) 区间。时间单位 = 秒。"""
    try:
        import mido
    except ImportError as exc:                                  # noqa: BLE001
        raise MidiError("需要 mido 库：python -m pip install mido") from exc

    mid = mido.MidiFile(path)
    notes: List[Note] = []
    opened: Dict[int, List[Tuple[float, int]]] = {}
    t = 0.0
    for msg in mid:
        t += float(getattr(msg, "time", 0.0) or 0.0)
        if getattr(msg, "is_meta", False):
            continue
        if msg.type == "note_on" and msg.velocity > 0:
            opened.setdefault(msg.note, []).append((t, msg.velocity))
        elif msg.type in ("note_off",) or (msg.type == "note_on" and msg.velocity == 0):
            stack = opened.get(msg.note)
            if stack:
                st, vel = stack.pop(0)
                notes.append(Note(st, t, msg.note, vel))
    # 没收到 note_off 的（被截断/踩住）用文件末尾收尾
    all_t = [t] + [n.end for n in notes] + [n.start for n in notes]
    end_t = max(all_t) if all_t else t
    for note, stack in opened.items():
        for st, vel in stack:
            notes.append(Note(st, max(end_t, st), note, vel))

    notes.sort(key=lambda n: (n.start, n.midi))

    # ---- 再走一遍【整数 tick】的路径，专门用来算时间格 ----
    #   ★ 为什么非得用 tick：mido 给的秒是浮点，round 到 6 位小数之后再转分数，
    #     limit_denominator 会给出 27581/62517 这种巨大分母的分母；
    #     逐个取 GCD 时分母相乘直接爆炸（1e-40），栅格被下限钳到 20ms，
    #     整首曲子的节奏就全错了。68 BPM 这种"除不尽"的拍速必然踩到
    #     （100/90 BPM 恰好能整除，之前侥幸躲过）。
    #   MIDI 本身就是整数 tick 网格，tick 之间的 GCD 是精确的。
    tk = 0
    onset_ticks: List[int] = []
    for msg in mido.merge_tracks(mid.tracks):
        tk += int(getattr(msg, "time", 0) or 0)
        if msg.type == "note_on" and msg.velocity > 0:
            onset_ticks.append(tk)
    tempo_us = 500000
    for tr in mid.tracks:
        hit = False
        for msg in tr:
            if msg.type == "set_tempo":
                tempo_us = int(msg.tempo)
                hit = True
                break
        if hit:
            break

    info = {
        "tracks": len(mid.tracks),
        "length_s": float(mid.length),
        "ticks_per_beat": mid.ticks_per_beat,
        "onset_ticks": sorted(set(onset_ticks)),
        "us_per_tick": float(tempo_us) / float(mid.ticks_per_beat),
    }
    return notes, info


# ==========================================================================
# 2) 量化栅格：把"秒"变成"时间格"
# ==========================================================================
def _gcd_frac(a: Fraction, b: Fraction) -> Fraction:
    """两个分数的最大公约数：gcd(p1/q1, p2/q2) = gcd(p1·q2, p2·q1) / (q1·q2)"""
    n = gcd(a.numerator * b.denominator, b.numerator * a.denominator)
    return Fraction(n, a.denominator * b.denominator)


def choose_tick_us(onsets: Sequence[float], info: Optional[dict] = None,
                   max_events: int = P.SLOT_MAX) -> int:
    """自动挑一个时间格长度（µs）= 【所有相邻起音间隔的最大公约数】。

    ★ 主路径走 MIDI 的【整数 tick】求 GCD（精确到 1 tick）。
      秒那条路只在拿不到 tick 时兜底 —— 见 load_notes 里的说明：
      浮点秒转分数会让分母爆炸，栅格算出个 1e-40，被下限钳到 20ms。

    为什么必须用最大公约数而不是"最小间隔"：
      谱子里常见"附点八分 + 十六分"，间隔集合是 {1/2, 3/4} 拍；
      最小间隔 1/2 拍并不是栅格（栅格是 1/4），用最小值会让附点音符
      落在 1.5 个格上，round 之后就偏了。

    跟"总格数"无关：FPGA 的限制只有【事件数 ≤ 1024】和【单事件 ≤ 255 格】，
    总格数没有上限（播放器里压根没有总格计数器，tick_cnt 每格重新计数）。
    """
    # ---- ① 整数 tick（首选，精确）----
    if info:
        ticks = info.get("onset_ticks") or []
        upt = info.get("us_per_tick") or 0.0
        if len(ticks) >= 2 and upt > 0:
            diffs = [b - a for a, b in zip(ticks, ticks[1:]) if b > a]
            if diffs:
                g = 0
                for d in diffs:
                    g = gcd(g, d)              # 整数 GCD，精确
                if g > 0:
                    tick_us = int(round(g * upt))
                    return max(P.TICK_MIN_US, min(P.US_PER_TICK_LIMIT, tick_us))

    # ---- ② 兜底：从秒反推（浮点，只在没有 tick 信息时用）----
    if len(onsets) < 2:
        return 500_000
    diffs_f = [b - a for a, b in zip(onsets, onsets[1:]) if b - a > 1e-9]
    if not diffs_f:
        return 500_000
    g: Optional[Fraction] = None
    for d in diffs_f:
        f = Fraction(d).limit_denominator(100000)
        if f <= 0:
            continue
        g = f if g is None else _gcd_frac(g, f)
    if g is None or g <= 0:
        return 500_000
    tick_us = int(round(float(g) * 1e6))
    return max(P.TICK_MIN_US, min(P.US_PER_TICK_LIMIT, tick_us))


def _fold_into_board(m: int) -> int:
    """把超出 A2..G#5 的音【按八度折返】进来 —— 音名不变，只换八度。

    ★ 为什么折返，而不是丢掉 ★
      丢掉一个音 = 旋律缺一块，听起来就是"这一拍漏了"；
      折返只是那个音掉/升了个八度，旋律线还连得上。
      板子只有 36 键（3 个八度 = 45..80），而 12 个半音一组，
      所以任何音高都能折进这个范围（最多折 2 次）。
    """
    for _ in range(6):                       # 45..80 宽 35 > 12，最多 2 次就够
        if m < BOARD_LO:
            m += 12
        elif m > BOARD_HI:
            m -= 12
        else:
            break
    return m


def choose_event_oct(midis: Sequence[int]) -> int:
    """★ V32：给【一格事件】挑一个整体八度偏移（让尽量多的音落进 A2..G#5）。

    板子只有 36 个声部（3 个八度），超出范围的音原来一律折八度 ——
    一句"高两个八度"的旋律被折回来就完全不是那个味道了。
    现在这一格可以整体升/降 n 个八度（板子上就是相位增量移 n 位，很便宜），
    于是旋律能按原八度弹出来。

    返回 0..±3 里"折返音最少"的那个；同分取 |o| 最小的、并且优先 0 ——
    ★ 这一条很重要：本来就在音域内的曲子，每一格都会算出 0，
      于是上位机一个字节都不多发，编出来的帧和以前【逐字节相同】。
    """
    if not midis:
        return 0
    best_o, best_bad = 0, None
    for o in (0, 1, -1, 2, -2, 3, -3):
        bad = sum(1 for m in midis if not (BOARD_LO + 12 * o <= m <= BOARD_HI + 12 * o))
        if best_bad is None or bad < best_bad:
            best_o, best_bad = o, bad
    return best_o


def _residual_fold(notes: Sequence[Note], k: int) -> int:
    """整体移调 k 个半音后，再让每一格自己搬八度，还剩几个音必须折返。

    ★ 这是 V32 之后 choose_shift 的新判据：八度偏移能救的，就不要动整曲音高。
    """
    groups: Dict[float, List[int]] = {}
    for n in notes:
        groups.setdefault(round(n.start, 6), []).append(n.midi + k)
    bad = 0
    for ms in groups.values():
        o = choose_event_oct(ms)
        bad += sum(1 for m in ms
                   if not (BOARD_LO + 12 * o <= m <= BOARD_HI + 12 * o))
    return bad


def choose_shift(notes: Sequence[Note], span: int = SPAN) -> int:
    """挑一个整体移调量（半音）—— "自动"档用的就是它。

    ★★ V32 改过判据（这一点很关键）★★
      以前：优先整八度、再半音级，总之想办法把【整首曲子】塞进 3 个八度。
            塞不下的就折八度 —— 于是"高两个八度的那句"被折回来，旋律就变味了。
      现在：每一格事件都能自己整体搬八度（板子上就是相位增量移位），
            所以整体移调只在【连一格一格搬都救不了】的时候才用。
            判据：
              ① 先比"还剩几个音必须折返"（_residual_fold，越小越好）
              ② 再比 |k|（越小越好，0 优先）—— 能不整曲移调就不移，
                 这样曲子按【原调】弹出来，只是高低八度由每一格自己认领。
      结果：正常曲子 k=0（帧和以前逐字节一样）；宽音域的曲子也不再整曲被挪低。
    """
    if not notes:
        return 0
    best_k, best_key = 0, None
    for k in (0, -12, 12, -24, 24, -36, 36):
        key = (_residual_fold(notes, k), abs(k))
        if best_key is None or key < best_key:
            best_k, best_key = k, key
    # 整八度都救不了 -> 再在半音范围里找一次（老逻辑留着，别丢功能）
    if best_key is not None and best_key[0] > 0:
        lo = min(n.midi for n in notes)
        hi = max(n.midi for n in notes)
        lo_s, hi_s = BOARD_LO - lo, BOARD_HI - hi
        cands = list(range(-24, 25))
        if lo_s <= hi_s:                      # 能完整放下：取最靠 0 的那一档
            cands = [0, lo_s, hi_s]
        for k in cands:
            if not -24 <= k <= 24:
                continue
            key = (_residual_fold(notes, k), abs(k))
            if key < best_key:
                best_k, best_key = k, key
    return best_k


# ==========================================================================
# 3) 组装
# ==========================================================================
@dataclass
class MidiReport:
    path: str = ""
    n_notes: int = 0
    n_onsets: int = 0
    n_events: int = 0
    shift: int = 0
    tick_us: int = 0
    rel_us: int = 0
    frame_bytes: int = 0
    send_seconds: float = 0.0
    play_seconds: float = 0.0
    notes_lo: int = 0
    notes_hi: int = 0
    used_lo: int = 0
    used_hi: int = 0
    dropped: int = 0
    folded: int = 0            # 超出音域、按八度折返进来的音数（不是丢掉）
    truncated_ticks: int = 0
    max_chord: int = 0
    # ★ V32：八度偏移的统计
    oct_events: int = 0        # 有多少格用了八度偏移
    oct_notes: int = 0         # 有多少个音是靠它才留在正确八度上的
    warnings: List[str] = field(default_factory=list)

    def text(self) -> str:
        L = []
        L.append("MIDI -> FPGA 帧")
        L.append("  文件           : %s" % self.path)
        L.append("  音符数 / 起音点: %d / %d" % (self.n_notes, self.n_onsets))
        L.append("  音域(MIDI)     : %d..%d  (%s..%s)"
                 % (self.notes_lo, self.notes_hi, _nm(self.notes_lo), _nm(self.notes_hi)))
        L.append("  整体移调       : %+d 半音  -> 实际发声 %d..%d (%s..%s)"
                 % (self.shift, self.used_lo, self.used_hi, _nm(self.used_lo), _nm(self.used_hi)))
        L.append("  时间格         : %d µs  释音提前 %d µs" % (self.tick_us, self.rel_us))
        L.append("  事件数         : %d 格   （上限 %d）" % (self.n_events, P.SLOT_MAX))
        L.append("  最大同时发声   : %d 个键" % self.max_chord)
        L.append("  帧字节数       : %d  （9600 8N1 需 %.2f 秒发完）"
                 % (self.frame_bytes, self.send_seconds))
        L.append("  播放时长       : %.1f 秒" % self.play_seconds)
        if self.dropped:
            L.append("  ★ 丢掉的音     : %d（超出 A2..G#5 音域）" % self.dropped)
        if self.oct_events:
            L.append("  ★ 八度偏移     : %d 格用了（共 %d 个音按原八度发声，"
                     "板子会在那一格整体升/降八度）" % (self.oct_events, self.oct_notes))
        if self.folded:
            L.append("  ★ 折返的音     : %d 个（连整格搬八度都放不下 —— 比如同一格"
                     "里跨了 4 个八度；按八度挪进来了，一个都没丢）" % self.folded)
        if self.truncated_ticks:
            L.append("  ★ 超长截断     : %d 格（超过 %d 事件上限）"
                     % (self.truncated_ticks, P.SLOT_MAX))
        for w in self.warnings:
            L.append("  ! %s" % w)
        return "\n".join(L)


def _nm(m: int) -> str:
    if not 0 <= m <= 127:
        return "?"
    return JP._oct_name(m)


def midi_to_frame(path: str, tick_us: Optional[int] = None,
                  rel_us: Optional[int] = None,
                  shift: Optional[int] = None,
                  max_events: int = P.SLOT_MAX,
                  head: bytes = bytes([P.CLR_BYTE]),
                  tail: bytes = bytes([P.END_BYTE]),
                  verbose: bool = False) -> Tuple[P.Frame, bytes, MidiReport]:
    """MIDI 文件 -> (Frame, 帧字节, 报告)。"""
    notes, info = load_notes(path)
    rep = MidiReport(path=os.path.basename(path), n_notes=len(notes))
    if not notes:
        raise MidiError("这个 MIDI 里一个音都没有")

    rep.notes_lo = min(n.midi for n in notes)
    rep.notes_hi = max(n.midi for n in notes)

    # ---- 移调 ----
    sh = choose_shift(notes) if shift is None else int(shift)
    rep.shift = sh
    # ★ V32：这里【先不折八度】—— 折返挪到"每一格"里去算，因为现在每一格
    #   可以自己带一个八度偏移（下面 choose_event_oct）。原来是在这里一次性
    #   折完，折掉的音就再也回不来了。
    kept: List[Note] = []
    for n in notes:
        kept.append(Note(n.start, n.end, n.midi + sh, n.vel))
    if not kept:
        raise MidiError("移调 %+d 之后一个音都没有" % sh)

    # ---- 栅格 ----
    onsets = sorted({round(n.start, 6) for n in kept})
    rep.n_onsets = len(onsets)
    tick = int(tick_us) if tick_us else choose_tick_us(onsets, info, max_events)
    tick = max(TICK_MIN_US, tick)
    rep.tick_us = tick

    # ---- 每个起音点一格 ----
    slot_of: Dict[int, int] = {}
    for t in onsets:
        slot_of[t] = int(round(t * 1e6 / tick))
    slots = sorted(set(slot_of.values()))
    by_slot: Dict[int, set] = {s: set() for s in slots}
    oct_of_slot: Dict[int, int] = {s: 0 for s in slots}
    end_of_slot: Dict[int, float] = {s: 0.0 for s in slots}
    # ★ V32：先把每一格"原始音高"收集起来，用它挑这一格的八度偏移
    raw_of_slot: Dict[int, List[int]] = {s: [] for s in slots}
    for n in kept:
        raw_of_slot[slot_of[round(n.start, 6)]].append(n.midi)
    for s in slots:
        oct_of_slot[s] = choose_event_oct(raw_of_slot[s])
        if oct_of_slot[s]:
            rep.oct_events += 1
    for n in kept:
        s = slot_of[round(n.start, 6)]
        o = oct_of_slot[s]
        m2 = n.midi - 12 * o                       # 板子会在这一格里升/降 o 个八度
        if BOARD_LO <= m2 <= BOARD_HI:
            pass
        else:
            # ★ 越界不丢：折八度挪进来（见 _fold_into_board 的说明）。
            #   ★ V32 之后这里剩下的，是"连整格搬八度都放不下"的那些音
            #     （比如同时按下的低音 + 高音跨了 4 个八度）。
            if o:
                rep.oct_notes += 1                 # 这一格靠偏移救回来的音
            rep.folded += 1
            m2 = _fold_into_board(m2)
        k = JP.midi_to_key(m2)
        if k < 0:
            continue
        by_slot[s].add(k)
        end_of_slot[s] = max(end_of_slot[s], n.end)
    rep.used_lo = min((min(v) for v in raw_of_slot.values() if v), default=0)
    rep.used_hi = max((max(v) for v in raw_of_slot.values() if v), default=0)

    events: List[P.Event] = []
    # ★ 开头的静音必须补上 ★
    #   协议里事件是从第 0 格开始【连续排】的，没有"从第 15 格开始"这回事。
    #   而 MIDI 里"前面两秒没音符"在起音点列表里压根不存在 ——
    #   不补的话整首曲子会提前开始（后面所有音都往前挪），而且对不齐。
    if slots and slots[0] > 0:
        rest = slots[0]
        while rest > 0:
            r = min(rest, P.MAX_TICKS)
            events.append(P.Event(keys=(), ticks=r))
            rest -= r
    for i, s in enumerate(slots):
        keys = sorted(by_slot[s])
        # ★ 事件时长 = min(下一个起音点, 本起音点上这些音真正的结束点) ★
        #   只看到下一个起音点会【把休止符吃掉】：
        #     音A 占 0..4 格、静音 4..8 格、音B 在 8 格起音
        #     -> 只按起音点分格的话，A 会被写成 8 格长的音，
        #        听感从"A 响一下，空一拍，再 B"变成"A 一直拖到 B"。
        #   用 note_off 的真实时刻封顶就对了。
        #   和弦里各音时长不同时取 max：宁可让短的那个多响一点，也不要截断长音。
        end_slot = int(round((end_of_slot[s] - s * tick / 1e6) * 1e6 / tick))
        if i + 1 < len(slots):
            gap = slots[i + 1] - s
            ticks = min(gap, max(1, end_slot))
            rest = gap - ticks
        else:                                     # 最后一个：只能按真实音长
            ticks = max(1, end_slot)
            rest = 0
        ticks = max(1, min(P.MAX_TICKS, ticks))
        events.append(P.Event(keys=tuple(keys), ticks=ticks, oct=oct_of_slot[s]))
        rep.max_chord = max(rep.max_chord, len(keys))
        # ★ 空出来的时间必须用【休止事件】补上 ★
        #   协议里事件是顺序连续占格的，没有"跳到第 N 格"这回事 ——
        #   少补一个休止，后面所有音就整体提前。0xEE 就是干这个的。
        while rest > 0:
            r = min(rest, P.MAX_TICKS)
            events.append(P.Event(keys=(), ticks=r))
            rest -= r

    # ---- 释音提前量：默认留 1/4 格当断句 ----
    if rel_us is None:
        rel = tick // 4
        rel = max(GAP_MIN_US, min(rel, tick - GAP_MIN_US))
    else:
        rel = int(rel_us)
    if rel >= tick:
        rel = max(0, tick - GAP_MIN_US)
        rep.warnings.append("释音提前量被夹到 %d µs（必须小于时间格 %d µs）" % (rel, tick))
    rep.rel_us = rel

    frame = P.Frame(events=events, tick_us=tick, rel_us=rel)
    if len(frame.events) > max_events:
        frame, dropped_ticks = P.truncate_to_fit(frame, limit=max_events, verbose=verbose)
        rep.truncated_ticks = dropped_ticks
        rep.warnings.append("事件数超过 %d，已按事件边界截断" % max_events)
    rep.n_events = len(frame.events)
    try:
        blob = P.encode_frame(frame, head=head, tail=tail)
    except P.ProtoError as exc:
        raise MidiError("编码失败：%s" % exc) from exc
    rep.frame_bytes = len(blob)
    rep.send_seconds = len(blob) * UART_BYTE_S
    rep.play_seconds = frame.total_ticks * tick / 1e6
    return frame, blob, rep


# ==========================================================================
# 自检
# ==========================================================================
def _mkmid(path: str, groups) -> str:
    """写一个测试用 .mid：groups = [(拍位置, [音符号...], 时值拍)]，1 拍 = 0.5 s"""
    import mido
    mid = mido.MidiFile(ticks_per_beat=480)
    tr = mido.MidiTrack()
    mid.tracks.append(tr)
    TPB, SEC_PER_BEAT = 480, 0.5
    tr.append(mido.MetaMessage("set_tempo", tempo=int(SEC_PER_BEAT * 1e6), time=0))
    evs = []
    for beat, notes, dur_beat in groups:
        for n in notes:
            evs.append((beat, 1, n, 80))
            evs.append((beat + dur_beat, 0, n, 0))
    # 同一位置先 note_off 再 note_on（不然重音会被当成同一个音）
    evs.sort(key=lambda x: (x[0], x[1]))
    last = 0.0
    for beat, on, n, v in evs:
        # beat 是【拍】，1 拍 = TPB 个 tick
        tr.append(mido.Message("note_on" if on else "note_off", note=n, velocity=v,
                               time=max(0, int(round((beat - last) * TPB)))))
        last = beat
    mid.save(path)
    return path


def _selftest() -> int:
    import tempfile
    ok = 0
    fail: List[str] = []

    def check(name, cond, extra=None):
        nonlocal ok
        if cond:
            ok += 1
            print("  ok   %s" % name)
        else:
            fail.append(name)
            print("  FAIL %s%s" % (name, ("  -> " + repr(extra)) if extra is not None else ""))

    tmp = os.path.join(tempfile.gettempdir(), "_fpga_midi_test.mid")

    print("[1] 键号换算：走置换表，不是 MIDI-48")
    check("MIDI 45(A2) -> 键号 5", JP.midi_to_key(45) == 5)
    check("MIDI 46(A#2) -> 键号 0", JP.midi_to_key(46) == 0)
    check("MIDI 50(D3) -> 键号 8", JP.midi_to_key(50) == 8)
    check("MIDI 57(A3) -> 键号 17", JP.midi_to_key(57) == 17)
    check("MIDI 69(A4) -> 键号 29", JP.midi_to_key(69) == 29)
    check("MIDI 80(G#5) -> 键号 35", JP.midi_to_key(80) == 35)
    check("键号 0 不是 A2（那是 MIDI-48 的错误说法）", JP.key_to_midi(0) == 46)

    print("[2] 单音 + 时值")
    _mkmid(tmp, [(0, [69], 1), (1, [71], 1)])          # A4, B4，各 1 拍 = 0.5s
    fr, blob, rep = midi_to_frame(tmp)
    check("2 个事件", len(fr.events) == 2)
    check("时间格 = 0.5s", rep.tick_us == 500_000)
    check("键号 = 29 / 30", fr.events[0].keys == (29,) and fr.events[1].keys == (30,))
    check("帧头是 16×FF + AA", blob[:17] == bytes([0xFF] * 16 + [0xAA]))
    check("帧尾是 FF", blob[-1] == 0xFF)

    print("[3] 和弦 = 同一时刻的多个音，进 0xC0..0xC1")
    _mkmid(tmp, [(0, [55, 59, 62], 2)])                # G3 B3 D4 同时
    fr, blob, rep = midi_to_frame(tmp)
    check("1 个事件", len(fr.events) == 1)
    check("3 个键同时", len(fr.events[0].keys) == 3)
    check("用了 0xC0/0xC1", 0xC0 in blob and 0xC1 in blob)
    check("报告里最大同时发声 = 3", rep.max_chord == 3)

    print("[4] 超音域：★ V32 之后不再整曲移调，改成每一格自己搬八度")
    _mkmid(tmp, [(0, [93], 1), (1, [95], 1)])          # A6 B6
    fr, blob, rep = midi_to_frame(tmp)
    check("整曲移调 = 0（按原调弹，不再整曲挪低）", rep.shift == 0, rep.shift)
    check("每一格用 +2 个八度偏移", [e.oct for e in fr.events] == [2, 2],
          [e.oct for e in fr.events])
    check("落到 A4/B4 的键位（发声 = 原音高 A6/B6）",
          fr.events[0].keys == (29,) and fr.events[1].keys == (30,),
          [e.keys for e in fr.events])
    check("没有丢音", rep.dropped == 0)

    print("[4b] ★ V32：同一格整体搬八度（音域拓宽）")
    # ① 本来就在音域内的曲子：一字节都不能变（老谱子必须逐字节相同）
    _mkmid(tmp, [(0, [60], 1), (1, [64], 1), (2, [67, 72], 1)])
    fr_in, blob_in, rep_in = midi_to_frame(tmp)
    check("音域内的谱子：没有任何事件用八度偏移", rep_in.oct_events == 0)
    check("音域内的谱子：帧里一个 0xCC 都没有",
          P.OCT_BYTE not in blob_in, blob_in.hex())
    check("音域内的谱子：键位就是原来那几个（C4/E4/G4+C5）",
          [e.keys for e in fr_in.events] == [(13,), (21,), (16, 25)],
          [e.keys for e in fr_in.events])
    check("音域内的谱子：整曲移调 = 0", rep_in.shift == 0, rep_in.shift)

    # ② 宽音域：一句低、一句高（差 4 个八度）—— 这是原来的痛点是吧？
    #    现在每一格各按各的八度弹，两句都落在【同一批键】上，音高各自正确。
    _mkmid(tmp, [(0, [36], 1), (1, [84], 1), (2, [36], 1), (3, [84], 1)])
    fr_wide2, blob_w2, rep_w2 = midi_to_frame(tmp)
    check("宽音域：整曲移调 = 0（不再整曲挪走）", rep_w2.shift == 0, rep_w2.shift)
    #   ★ 偏移取的是【能让这个音落进窗口的最小 |o|】：
    #     C2(36) -> o=-1（窗口 33..68），C6(84) -> o=+1（窗口 57..92）
    check("宽音域：低音格 -1 八度、高音格 +1 八度",
          [e.oct for e in fr_wide2.events] == [-1, 1, -1, 1],
          [e.oct for e in fr_wide2.events])
    check("宽音域：一个音都没被折返", rep_w2.folded == 0, rep_w2.folded)
    check("宽音域：帧里发了 [0xCC][07](-1) 和 [0xCC][01](+1)",
          bytes([P.OCT_BYTE, 0x07]) in blob_w2 and bytes([P.OCT_BYTE, 0x01]) in blob_w2,
          blob_w2.hex())
    dec_w2 = P.decode_frame(blob_w2)
    check("解回来八度偏移对得上",
          [e.oct for e in dec_w2.events] == [-1, 1, -1, 1],
          [e.oct for e in dec_w2.events])

    # ③ 低两个八度
    _mkmid(tmp, [(0, [60], 1), (1, [64], 1)])
    fr_lo, _, _ = midi_to_frame(tmp)
    _mkmid(tmp, [(0, [36], 1), (1, [40], 1)])           # C2 E2 = 低两个八度
    fr_dn, _, rep_dn = midi_to_frame(tmp)
    check("低八度的旋律：用 -1 八度偏移（最小的那档就够）",
          [e.oct for e in fr_dn.events] == [-1, -1],
          [e.oct for e in fr_dn.events])
    #   ★ 真正要守住的不变量是【发声音高 = 谱面音高】：
    #     键号会变（八度偏移就是让声部换个键去发），但 key_to_midi + 12×偏移
    #     必须回到原来那个音。
    def _sounding(fr_):
        return [JP.key_to_midi(k) + 12 * e.oct for e in fr_.events for k in e.keys]
    check("★ 低八度那句的发声音高 = 谱面音高（36/40）",
          _sounding(fr_dn) == [36, 40], _sounding(fr_dn))
    check("★ 原句的发声音高也还是 60/64", _sounding(fr_lo) == [60, 64],
          _sounding(fr_lo))

    # ④ 同一格里跨了 4 个八度：整格搬八度也救不了，仍要折返（而且不丢音）
    _mkmid(tmp, [(0, [36, 84], 1)])                     # C2 + C6
    fr_w, _, rep_w = midi_to_frame(tmp)
    check("跨 4 个八度的一格：仍然折返，但一个音都没丢",
          rep_w.folded >= 1 and len(fr_w.events[0].keys) == 2,
          (rep_w.folded, fr_w.events[0].keys))

    # ⑤ 有休止的谱子不受影响
    _mkmid(tmp, [(0, [60], 1), (3, [62], 1)])
    fr_r, _, rep_r = midi_to_frame(tmp)
    check("有休止的谱子：休止事件不带八度偏移",
          all(e.oct == 0 for e in fr_r.events if e.is_rest) and rep_r.oct_events == 0)

    print("[5] 容量与栅格：总格数不设限，栅格按最大公约数取")
    many = [(i, [69], 1) for i in range(P.SLOT_MAX + 40)]
    _mkmid(tmp, many)
    fr, blob, rep = midi_to_frame(tmp)
    check("1064 个起音点被截到 <= SLOT_MAX 个事件",
          len(fr.events) <= P.SLOT_MAX and rep.n_events == len(fr.events))
    check("★ 栅格不会被总格数粗化（1 拍间隔 -> 就是 1 拍）",
          rep.tick_us == 500_000, rep.tick_us)
    fr2, blob2, rep2 = midi_to_frame(tmp, tick_us=P.TICK_MIN_US, max_events=64)
    check("强制细栅格 + 上限 64 -> 截到 64 个事件", len(fr2.events) == 64)
    check("报告里记了丢多少格", rep2.truncated_ticks > 0)

    print("[5b] 附点音符：栅格必须是公约数，不能取最小间隔")
    # 4 个起音点：附点八分、十六分、附点八分、十六分
    # 间隔集合 {3/4, 1/4} 拍 -> gcd = 1/4 拍
    _mkmid(tmp, [(0, [69], Fraction(3, 4)), (Fraction(3, 4), [71], Fraction(1, 4)),
                 (1, [69], Fraction(3, 4)), (Fraction(7, 4), [71], Fraction(1, 4))])
    fr3, blob3, rep3 = midi_to_frame(tmp)
    check("栅格 = 1/4 拍（0.5s/4 = 125ms）", rep3.tick_us == 125_000, rep3.tick_us)
    check("附点八分 = 3 格", fr3.events[0].ticks == 3, fr3.events[0].ticks)
    check("十六分 = 1 格", fr3.events[1].ticks == 1, fr3.events[1].ticks)

    print("[5c] 休止符不能被吃掉，空隙要用 0xEE 补齐")
    # 起音点 0 / 1 / 3 拍，每音只占半拍 -> 第 2~3 拍之间是空档
    # 间隔 {1, 2} 拍 -> 栅格 = 1 拍
    _mkmid(tmp, [(0, [69], Fraction(1, 2)), (1, [71], Fraction(1, 2)),
                 (3, [69], Fraction(1, 2))])
    fr4, blob4, rep4 = midi_to_frame(tmp)
    check("栅格 = 1 拍", rep4.tick_us == 500_000, rep4.tick_us)
    check("每条音都只占 1 格（没有被拉长）",
          [e.ticks for e in fr4.events if e.keys] == [1, 1, 1],
          [e.ticks for e in fr4.events])
    check("空档补了一条休止事件",
          any(e.keys == () for e in fr4.events), [e.keys for e in fr4.events])
    check("空档确实写成了 0xEE", blob4.count(P.REST_BYTE) >= 1)
    check("总格数 = 4（3 音 + 1 休止）",
          round(rep4.play_seconds * 1e6 / rep4.tick_us) == 4,
          rep4.play_seconds * 1e6 / rep4.tick_us)

    print("[6] 释音提前量：必须 < 时间格")
    _mkmid(tmp, [(0, [69], 1), (1, [71], 1)])
    fr, blob, rep = midi_to_frame(tmp, rel_us=999_999_999)
    check("被夹到 < 时间格", rep.rel_us < rep.tick_us)

    print("[7] 空文件要报错")
    empty = os.path.join(tempfile.gettempdir(), "_fpga_midi_empty.mid")
    try:
        import mido
        m = mido.MidiFile()
        m.tracks.append(mido.MidiTrack())
        m.save(empty)
        got = False
        try:
            midi_to_frame(empty)
        except MidiError:
            got = True
        check("空 MIDI 抛 MidiError", got)
    except Exception as exc:                            # noqa: BLE001
        check("空 MIDI 抛 MidiError (%s)" % exc, False)

    print()
    print("=" * 62)
    if fail:
        print("失败 %d 项 / 通过 %d 项" % (len(fail), ok))
        for n in fail:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % ok)
    return 0


def _show(path, out=None, tick_us=None, rel_us=None):
    fr, blob, rep = midi_to_frame(path, tick_us=tick_us, rel_us=rel_us, verbose=True)
    print(rep.text())
    print()
    print(P.describe_frame(fr))
    print()
    head = P.fmt_hex(blob[:48])
    tail = P.fmt_hex(blob[-16:])
    print("帧前 48 字节: %s" % head)
    print("帧后 16 字节: %s" % tail)
    if out:
        with open(out, "wb") as fh:
            fh.write(blob)
        print("已写出: %s (%d 字节)" % (out, len(blob)))
    return 0


def main() -> int:
    argv = sys.argv[1:]
    if not argv or argv[0] == "selftest":
        return _selftest()
    if argv[0] in ("show", "frame") and len(argv) > 1:
        path = argv[1]
        out = argv[2] if (len(argv) > 2 and not argv[2].startswith("-")) else None
        tick = rel = None
        for i, a in enumerate(argv):
            if a == "--tick-us" and i + 1 < len(argv):
                tick = int(argv[i + 1])
            if a == "--rel-us" and i + 1 < len(argv):
                rel = int(argv[i + 1])
        return _show(path, out, tick, rel)
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
