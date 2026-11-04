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
# ★ V35：和弦归并窗口（毫秒）。起音差在这个窗口内、且【真的在重叠】、且音高不同的音
#   并到同时发声的那一组。想听原样（完全不并）就设 0。
CHORD_MERGE_MS = 25.0
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


def merge_chord_onsets(notes: Sequence[Note], window_ms: float = 25.0):
    """把"真的同时响"的音并到同一个起音点上（治"和弦被拆成一串先后弹"）。

    ★ 只用时间窗（比如 ±25ms）当判据是不靠谱的，会误伤三种东西：
        快速重复音/颤音（同音高每 30ms 敲一次）、装饰音/倚音、故意滚奏。
      所以要求【三个条件同时满足】：
        ① 起音相差 <= window_ms
        ② ★ 前一个音在后一个音起音的那一刻【还在响】（真重叠）——
           和弦的定义就是"同时发声"，快速音阶/颤音是【不重叠】的。
           这一条比时间窗硬得多，是主判据。
        ③ 音高不同（同音高在窗口内重复 = 重复音/颤音，绝不并）
      起音完全相同的音（差 < 1ms）不在这里处理 —— 它们本来就是同一格，
      交给后面的分格逻辑。

    返回 (新的音符列表, (并了几处, 其中靠"重叠"判定的几处))
    """
    win = max(0.0, float(window_ms)) / 1000.0
    out: List[Note] = []
    merged = 0
    for n in sorted(notes, key=lambda x: (x.start, x.midi)):
        placed = False
        if win > 0:
            # 只往回看最近那一批（按时间倒着找，超出窗口就停）
            for i in range(len(out) - 1, -1, -1):
                m = out[i]
                gap = n.start - m.start
                if gap > win:
                    break
                if gap < 1e-6 or m.midi == n.midi:
                    continue                       # 同一起音 / 同音高 -> 不并
                # ★ 主判据：这一组（起音相同的算一组）到 n 起音时还在响吗
                still = max(o.end for o in out if abs(o.start - m.start) < 1e-6)
                if still > n.start + 1e-6:
                    out[i] = Note(m.start, max(m.end, n.end), m.midi, m.vel)
                    out.append(Note(m.start, n.end, n.midi, n.vel))
                    merged += 1
                    placed = True
                    break
        if not placed:
            out.append(n)
    out.sort(key=lambda x: (x.start, x.midi))
    return out, (merged, merged)


def choose_event_oct(midis: Sequence[int], prev_oct: int = 0,
                     busy: Sequence[int] = ()) -> int:
    """★ V32：给【一格事件】挑一个整体八度偏移（让尽量多的音落进 A2..G#5）。

    板子只有 36 个声部（3 个八度），超出范围的音原来一律折八度 ——
    一句"高两个八度"的旋律被折回来就完全不是那个味道了。
    现在这一格可以整体升/降 n 个八度（板子上就是相位增量移 n 位，很便宜），
    于是旋律能按原八度弹出来。

    ★★ V33：判据从"只看折返数"改成三级（这一条专治"大跳被截断"）★★
      用户报过"音阶跳跃大的时候有截断和丢音"。根因不是板子来不及切（它 2.9µs
      就切完了），而是【大跳的两个音会落到同一个声部上】：
        同一个音名，用不同八度偏移就落在【不同的键】上，例如 C6
          o=+1 -> 键25(发 C6)     o=+2 -> 键13(发 C6，因为 60+24=84)
      而板子一个声部同时只能有一个音：后一个音一按下去，前一个音的余音就被
      掐掉了（听感就是"截断"，短音甚至像"丢了"）。
      所以挑偏移时按这个顺序比：
        ① 折返音最少
        ② 不撞【上一个事件正在响的那些键】（busy 里传进来）
        ③ 尽量和上一个事件用同一个偏移（少来回换 = 少撞车）
        ④ |o| 最小、优先 0
      ★ ④ 里的"优先 0"很重要：本来就在音域内的曲子，每一格都会算出 0，
        于是上位机一个字节都不多发，编出来的帧和以前【逐字节相同】。
    """
    if not midis:
        return 0
    busy_set = set(busy)
    best = None
    for o in (0, 1, -1, 2, -2, 3, -3):
        bad = sum(1 for m in midis
                  if not (BOARD_LO + 12 * o <= m <= BOARD_HI + 12 * o))
        keys = set()
        for m in midis:
            k = JP.midi_to_key(_fold_into_board(m - 12 * o))
            if k >= 0:
                keys.add(k)
        hit = len(keys & busy_set)
        # ★★ 排序：折返 -> 【稳定】 -> 避让 -> |o| ★★
        #   ★ "稳定"必须排在"避让"前面（V33 一开始写反了，用户听出来"不连贯"）：
        #     同一个偏移下，"音高 -> 键"是一一对应的，相邻音高天然落在不同的键上；
        #     真正会撞、会串味的，恰恰是【偏移来回跳】的时候 —— 一换偏移，
        #     同一个音高就落到另一个声部，衔接/音头/余音全跟着变。
        #     所以先让偏移尽量别变，避让只在"反正都要变"的时候才起作用。
        rank = (bad, 0 if o == prev_oct else 1, hit, abs(o))
        if best is None or rank < best[0]:
            best = (rank, o)
    return best[1]


# ==========================================================================
# ★★★ V37：逐音分配 (键, 八度) —— 本版的核心 ★★★
# ==========================================================================
# 板子上"实际发出来的音高" = 键的固有音高 + 12 × 这个键的八度偏移：
#       key_to_midi(键) + 12 × o
# 所以同一个音高 m 往往有【好几个】键都能精确到达它：
#       m = 84(C6)  ->  (键 C5 的位置, o=+1) 或 (键 C4 的位置, o=+2) ...
# V32 只能给【一整格】挑一个 o，于是"同一格里跨 4 个八度"的和弦
# （左手 C2 + 右手 C6）必然有一部分音只能折返 —— 旋律就断了。
# V37 改成逐音挑：每个音自己选一个能【精确】到达它的 (键, o)，
# 唯一的约束是这些键互不相同（一个声部同时只能发一个音）。
_OCT_STEPS = (0, 1, -1, 2, -2, 3, -3, 4, -4)


def assign_keys(midis: Sequence[int],
                prev_assign: Optional[Dict[int, Tuple[int, int]]] = None,
                busy: Sequence[int] = ()
                ) -> Tuple[Tuple[int, ...], Dict[int, int],
                           Dict[int, Tuple[int, int]], int, int]:
    """★ V37：给一格的音逐个分配 (键, 八度)。

    参数
        midis       : 这一格的原始音高（已经加过整曲移调）
        prev_assign : 上一格的 {音高: (键, 八度)}，用来让同一个音高【别换键】
        busy        : 上一格还在响的键号（尽量避开，免得掐掉它的余音）
    返回
        (keys, key_octs, pitch_assign, n_folded, n_oct_notes)
        keys         : 升序键号元组
        key_octs     : {键号: 八度}，只含非零项（直接喂给 fpga_proto.Event）
        pitch_assign : {音高: (键, 八度)}，原样传给下一格当 prev_assign
        n_folded     : 连 ±4 个八度都够不到、只能折返的音数
        n_oct_notes  : 靠八度偏移才落在正确音高上的音数

    ★ 排序偏好（越靠前越优先）：
        ① 与上一格【同一个音高】用同一个 (键, 八度)
        ② 避开上一格还在响的键
        ③ |八度| 小    ④ 优先原调(0)、再升、再降    ⑤ 键号小（保证确定性）
    ★ ① 排在 ② 前面，是 V33 拿听感换来的教训：同一个音高在两格之间【换了键】，
      前一个音的余音会被后按下的那个音掐掉（听感就是"截断"）。
      避让只在"反正都要换"的时候才起作用。
    ★ 分配用二分图匹配（Kuhn 增广路）：n <= 36、每个音 <= 3 个候选，
      代价可以忽略，而且只要存在"不撞键"的方案就一定找得到。
    """
    ms = sorted({int(m) for m in midis})
    prev_assign = dict(prev_assign or {})
    busy_set = {int(k) for k in busy}

    # ---- 1) 每个音的候选 (键, 八度) ----
    cand: List[List[Tuple[int, int]]] = []
    n_folded = 0
    for m in ms:
        opts: List[Tuple[int, int]] = []
        for o in _OCT_STEPS:
            km = m - 12 * o
            if BOARD_LO <= km <= BOARD_HI:
                k = JP.midi_to_key(km)
                if 0 <= k < P.N_KEYS:
                    opts.append((k, o))
        if not opts:
            # 连 ±4 都够不到（m < -3 或 m > 116）：折八度挪进来，一个音都不丢
            opts.append((JP.midi_to_key(_fold_into_board(m)), 0))
            n_folded += 1
        cand.append(opts)

    # ---- 2) 候选按偏好排序 ----
    def _rank(m: int, k: int, o: int):
        pk, po = prev_assign.get(m, (-1, None))
        return (0 if (k, o) == (pk, po) else 1,
                1 if k in busy_set else 0,
                abs(o),
                0 if o == 0 else (1 if o > 0 else 2),
                k)

    for i, m in enumerate(ms):
        cand[i].sort(key=lambda t, m=m: _rank(m, t[0], t[1]))

    # ---- 3) 二分图匹配（Kuhn 增广路）----
    owner: Dict[int, int] = {}          # 键 -> 音的序号
    pick: Dict[int, Tuple[int, int]] = {}

    def _augment(i: int, seen: set) -> bool:
        for (k, o) in cand[i]:
            if k in seen:
                continue
            seen.add(k)
            j = owner.get(k)
            if j is None or _augment(j, seen):
                owner[k] = i
                pick[i] = (k, o)
                return True
        return False

    # 候选少的先安排（最受限优先）→ 匹配成功率最高；同数量按音高排 = 确定性
    for i in sorted(range(len(ms)), key=lambda i: (len(cand[i]), ms[i])):
        _augment(i, set())

    # 极端情况（比如 8 个同音名的音来抢同一个音名的 3 个键）：
    # 给没匹配上的音找一个【还空着】的键折返 —— 音高差八度，但至少不撞、不丢音。
    for i, m in enumerate(ms):
        if i in pick:
            continue
        base = _fold_into_board(m)
        chosen = None
        for d in (0, 12, -12, 24, -24):
            km = base + d
            if BOARD_LO <= km <= BOARD_HI:
                k = JP.midi_to_key(km)
                if k not in owner:
                    chosen = k
                    break
        if chosen is None:
            # 同一个音名的键全被占满了 —— 这是【物理上限】：36 个键只覆盖
            # 3 个八度，所以任何音名最多只有 3 个键能精确到达它。
            # 退而求其次：在整个音域里找一个还空着的键，保证"一个音都不丢"
            # （音高会差，但 5 个同音名同时响本来就不可能全对）。
            for _k in range(P.N_KEYS):
                if _k not in owner:
                    chosen = _k
                    break
            if chosen is None:
                chosen = JP.midi_to_key(base)     # 36 个键全满了：只好撞
                owner.pop(chosen, None)
        owner[chosen] = i
        pick[i] = (chosen, 0)
        n_folded += 1

    # ---- 4) 汇总 ----
    key_octs: Dict[int, int] = {}
    pitch_assign: Dict[int, Tuple[int, int]] = {}
    n_oct_notes = 0
    for i, m in enumerate(ms):
        k, o = pick[i]
        pitch_assign[m] = (k, o)
        if o != 0:
            key_octs[k] = o
            n_oct_notes += 1
    return (tuple(sorted(owner.keys())), key_octs, pitch_assign,
            n_folded, n_oct_notes)


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
    merged_notes: int = 0      # ★ V35：被并进"同时发声"那一组的音数
    merged_by_overlap: int = 0 # （判据里"真重叠"是主判据，留着给以后细分）
    oct_notes: int = 0         # 有多少个音是靠它才留在正确八度上的
    # ★ V45：逐事件力度的统计
    vel_events: int = 0        # 有多少格带了力度（= 发了 0xC3 的那些格）
    vel_disabled: bool = False # 界面上把「MIDI 力度」关了（一个 0xC3 都不发）
    vel_max: int = 0           # 这一首里最大的 velocity（0 = 一个都没带）
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
        if self.merged_notes:
            L.append("  ★ 和弦归并     : %d 个音并到了同时发声的那一组"
                     "（判据 = 时间窗 %.0fms + 真重叠 + 音高不同）"
                     % (self.merged_notes, CHORD_MERGE_MS))
        if self.oct_events:
            L.append("  ★ 逐音八度     : %d 格用了（共 %d 个音靠它落在正确音高上 ——"
                     " 板子会给这些音各发一个 0xCD）" % (self.oct_events, self.oct_notes))
        # ★ V45：力度一行 —— 三种情况分开说，A/B 对比时看这一行就够
        if self.vel_disabled:
            L.append("  力度           : 关（一个 0xC3 都不发，板子按默认 127 满幅；"
                     "帧与加这个功能之前逐字节相同）")
        elif self.vel_events:
            L.append("  ★ 逐事件力度   : %d 格带了 velocity（最高 %d；每格取这一格里"
                     "各音的最大值，板子收到 0xC3 只作用于紧接着的那一个事件）"
                     % (self.vel_events, self.vel_max))
        else:
            L.append("  力度           : 全部 127（一个 0xC3 都没发，帧与加这个功能"
                     "之前逐字节相同）")
        if self.folded:
            L.append("  ★ 折返的音     : %d 个（连 ±4 个八度都够不到，即 m < -3 或 "
                     "m > 116；按八度挪进来了，一个都没丢）" % self.folded)
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
                  verbose: bool = False,
                  use_velocity: bool = True) -> Tuple[P.Frame, bytes, MidiReport]:
    """MIDI 文件 -> (Frame, 帧字节, 报告)。

    use_velocity ★ V45：
        True （默认）= 把 note_on 的 velocity 带进帧里：每个事件前面发
                       [0xC3][VEL]（板子只作用于紧接着的那一个事件）。
        False        = 一个 0xC3 都不发 —— 板子按默认 127（满幅）处理，
                       产出的帧和加这个功能之前【逐字节相同】，专门给
                       界面上的「MIDI 力度」开关关掉时做 A/B 对比用。
    """
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

    # ★ V35：和弦归并（只用时间窗会误伤颤音/装饰音/滚奏，所以要求"真重叠"）
    #   放在分格之前：并进同一批之后，分格自然就把它们放进同一格 = 一个和弦。
    kept, _mg = merge_chord_onsets(kept, window_ms=CHORD_MERGE_MS)
    rep.merged_notes, rep.merged_by_overlap = _mg
    if _mg[0]:
        rep.warnings.append("和弦归并：把 %d 个音并到了同时发声的那一组里"
                            "（判据=起音差 <= %.0fms 且【真的在重叠】且音高不同）"
                            % (_mg[0], CHORD_MERGE_MS))

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
    end_of_slot: Dict[int, float] = {s: 0.0 for s in slots}
    # ★ V32：先把每一格"原始音高"收集起来 —— V37 用它做【逐音】分配
    raw_of_slot: Dict[int, List[int]] = {s: [] for s in slots}
    for n in kept:
        raw_of_slot[slot_of[round(n.start, 6)]].append(n.midi)
    # ★★ V45：每一格的力度 = 这一格里所有音 velocity 的【最大值】★★
    #   ★ 协议是【逐事件】力度（一个和弦只带一个 VEL），所以必须挑一个代表：
    #     取 max 而不是 min/平均值 —— 和弦里只要有一个音是重音，这一格就该是
    #     重音；取平均会把重音削弱，取 min 会让"左手轻、右手重"的和弦整个变轻。
    #   ★ velocity == 0 的 note_on 在 load_notes 里已经当成 note_off 处理掉了
    #     （MIDI 的规矩），所以这里的 vel 全是 1..127，不存在"力度 0"。
    #   ★ 休止事件（0xEE）没有力度：板子的 0xC3 是"挂着等下一个事件"的，
    #     挂在休止上就等于被休止吃掉了 —— 所以休止一律用默认 127，不发 0xC3。
    #   ★★ use_velocity=False 时一格都不填（全 127）→ 帧里一个 0xC3 都没有。
    vel_of_slot: Dict[int, int] = {}
    for n in kept:
        s = slot_of[round(n.start, 6)]
        v = P.clamp_vel(n.vel)
        if v > vel_of_slot.get(s, 0):
            vel_of_slot[s] = v
    # ★★ V37：逐音分配 (键, 八度) ★★
    #   原来这里是"给一整格挑一个 o，然后 m2 = m - 12o 必须落进 45..80，
    #   否则折返"。现在每个音自己挑一个能【精确】到达它的 (键, o) ——
    #   于是"同一格里跨 4 个八度"的和弦再也不用折返，这正是 V32 留下的那个洞。
    #   ★ busy 仍然是"上一格在响的键"（避让）；prev_assign 是"上一格同一个音高
    #     的落法"（稳定优先 —— V33 的教训是换键会掐掉前一个音的余音）。
    keys_of_slot: Dict[int, Tuple[int, ...]] = {}
    key_oct_of_slot: Dict[int, Dict[int, int]] = {}
    _prev_assign: Dict[int, Tuple[int, int]] = {}
    _prev_keys: set = set()
    for s in slots:
        _ks, _ko, _pa, _nfold, _noct = assign_keys(raw_of_slot[s],
                                                   prev_assign=_prev_assign,
                                                   busy=sorted(_prev_keys))
        keys_of_slot[s] = _ks
        key_oct_of_slot[s] = _ko
        rep.folded += _nfold
        rep.oct_notes += _noct
        # ★ 统计口径必须用 _noct（这一格有几个音真的用了八度），不能用
        #   "key_octs 非空" —— 单音事件的 key_octs 会被 Event 提升成整格 oct，
        #   用后者会把"单音搬八度"那些格子漏掉，报告的数字就不对了。
        if _noct:
            rep.oct_events += 1
        _prev_assign = _pa
        _prev_keys = set(_ks)
    # 每一格的真实结束时刻（下面算事件时长要用它封顶）
    for n in kept:
        s = slot_of[round(n.start, 6)]
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
        keys = list(keys_of_slot[s])
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
        # ★ V37：整格八度一律用 0，所有八度都走 key_octs（逐键）。
        #   音域内的曲子 key_octs 是空的 → 与 V36 编出来【逐字节相同】。
        # ★ V45：力度也挂在【这一个事件】上 —— Event 里 127 = 不发 0xC3。
        _vel = vel_of_slot.get(s, P.VEL_DEFAULT) if use_velocity else P.VEL_DEFAULT
        events.append(P.Event(keys=tuple(keys), ticks=ticks,
                              oct=0, key_octs=key_oct_of_slot[s], vel=_vel))
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
    # ★ V45：报告里的力度统计（放在截断之后算，报的和真正发出去的一致）
    rep.vel_disabled = not bool(use_velocity)
    rep.vel_events = sum(1 for e in frame.events if e.vel != P.VEL_DEFAULT)
    rep.vel_max = max((e.vel for e in frame.events), default=0)
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
    """写一个测试用 .mid：groups = [(拍位置, [音符号...], 时值拍[, 力度])]，1 拍 = 0.5 s

    ★ V45：第 4 项（力度）是可选的，不写就是老行为 velocity=80 ——
      已有用例一个字都不用改。
    """
    import mido
    mid = mido.MidiFile(ticks_per_beat=480)
    tr = mido.MidiTrack()
    mid.tracks.append(tr)
    TPB, SEC_PER_BEAT = 480, 0.5
    tr.append(mido.MetaMessage("set_tempo", tempo=int(SEC_PER_BEAT * 1e6), time=0))
    evs = []
    for grp in groups:
        beat, notes, dur_beat = grp[0], grp[1], grp[2]
        vel = int(grp[3]) if len(grp) > 3 else 80
        for n in notes:
            evs.append((beat, 1, n, vel))
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
    #   ★ V37：每个键的八度先看 key_octs（逐键），没有才落到整格 e.oct。
    def _sounding(fr_):
        out = []
        for e in fr_.events:
            for k in e.keys:
                out.append(JP.key_to_midi(k) + 12 * e.key_octs.get(k, e.oct))
        return out
    check("★ 低八度那句的发声音高 = 谱面音高（36/40）",
          _sounding(fr_dn) == [36, 40], _sounding(fr_dn))
    check("★ 原句的发声音高也还是 60/64", _sounding(fr_lo) == [60, 64],
          _sounding(fr_lo))

    # ④ ★★ V37：同一格里跨 4 个八度 —— V32 只能折返，现在两个音都按原音高发 ★★
    #    这是 V32 留下的那个洞：整格只能有一个八度，所以"左手 C2 + 右手 C6"
    #    必然有一个音要折八度（旋律就断了）。逐键八度之后两个音各走各的。
    _mkmid(tmp, [(0, [36, 84], 1)])                     # C2 + C6
    fr_w, blob_w, rep_w = midi_to_frame(tmp)
    check("★ 跨 4 个八度的一格：一个音都不用折返", rep_w.folded == 0, rep_w.folded)
    check("★ 跨 4 个八度的一格：两个音都发出来了（两个不同声部）",
          len(fr_w.events[0].keys) == 2, fr_w.events[0].keys)
    check("★★ 跨 4 个八度的一格：发声音高 = 谱面音高（36 / 84）",
          sorted(_sounding(fr_w)) == [36, 84], _sounding(fr_w))
    check("★ 和弦内确实发了逐键八度前缀 0xCD",
          P.KEY_OCT_BYTE in blob_w, blob_w.hex())
    dec_w = P.decode_frame(blob_w)
    check("★ 逐键八度 encode -> decode 往返一致",
          [e.key_octs for e in dec_w.events] == [e.key_octs for e in fr_w.events],
          [e.key_octs for e in dec_w.events])
    check("★ 解回来的键号 / 整格八度也一致",
          [e.keys for e in dec_w.events] == [e.keys for e in fr_w.events]
          and [e.oct for e in dec_w.events] == [e.oct for e in fr_w.events])

    # ④b ★ V37：逐键八度的几组硬用例
    #   (a) 三个八度叠置：三个音各占一个声部、各带各的八度
    _mkmid(tmp, [(0, [36, 60, 84], 1)])
    fr3o, _, rep3o = midi_to_frame(tmp)
    check("★ 三个八度叠置：零折返", rep3o.folded == 0, rep3o.folded)
    check("★ 三个八度叠置：发声音高 = 36/60/84",
          sorted(_sounding(fr3o)) == [36, 60, 84], _sounding(fr3o))
    #   (b) 跨 5 个八度的五音和弦（低音 + 三和弦 + 高音）
    _mkmid(tmp, [(0, [24, 60, 64, 67, 108], 1)])
    fr5o, _, rep5o = midi_to_frame(tmp)
    check("★ 跨 5 个八度的五音和弦：零折返", rep5o.folded == 0, rep5o.folded)
    check("★ 跨 5 个八度的五音和弦：发声音高全对",
          sorted(_sounding(fr5o)) == [24, 60, 64, 67, 108], _sounding(fr5o))
    #   (c) 音域内 → 一个 0xCD 都不该有（这才叫"逐字节兼容 V36"）
    _mkmid(tmp, [(0, [60, 64, 67], 1)])                 # C4 E4 G4，全在范围内
    _, blob_in3, _ = midi_to_frame(tmp)
    check("★ 音域内的和弦：一个 0xCD 都不发（与 V36 逐字节相同）",
          P.KEY_OCT_BYTE not in blob_in3, blob_in3.hex())
    check("★ 音域内的和弦：逐键八度表是空的",
          all(not e.key_octs for e in P.decode_frame(blob_in3).events))
    #   (d) 极端：同一个音名的 5 个八度同时响 —— 只有 3 个键能到达它们，
    #       匹配必然不够，但【一个音都不能丢】，多出来的折返保底
    _mkmid(tmp, [(0, [36, 48, 60, 72, 84], 1)])         # C2..C6
    fr_e, _, rep_e = midi_to_frame(tmp)
    check("★ 同音名 5 个八度：一个音都没丢（5 个声部都在响）",
          len(fr_e.events[0].keys) == 5, fr_e.events[0].keys)
    check("★ 同音名 5 个八度：正确音高达到物理上限 3 个（同音名只有 3 个键）",
          len(set(_sounding(fr_e)) & {36, 48, 60, 72, 84}) == 3,
          sorted(_sounding(fr_e)))

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

    # ======================================================================
    print("[8] ★ V45：MIDI 逐事件力度（note_on 的第 3 个字节 -> 0xC3）")
    #   规则（和用户/FPGA 侧钉死的那几条逐条对应）：
    #     ① note_on 且 velocity > 0 -> 用这个 velocity
    #     ② velocity == 0 在 MIDI 里是 note_off，【不是力度 0】
    #     ③ 同一格里多个音（和弦）-> 取最大的那个 velocity
    #     ④ 没有力度信息的来源 -> 一个 0xC3 都不发（板子按 127）
    #     ⑤ 开关关掉 -> 一个 0xC3 都不发，且与改造前【逐字节相同】
    _mkmid(tmp, [(0, [69], 1, 100), (1, [71], 1, 40)])
    fr_v, blob_v, rep_v = midi_to_frame(tmp)
    check("① 力度带进了 Event.vel（100 / 40）",
          [e.vel for e in fr_v.events] == [100, 40], [e.vel for e in fr_v.events])
    check("① 帧里每个事件前面都有 C3 64 / C3 28",
          bytes([P.VEL_BYTE, 100]) in blob_v and bytes([P.VEL_BYTE, 40]) in blob_v,
          blob_v.hex())
    check("① 字节顺序：C3 64 紧挨在键号前面（不在音符后面）",
          bytes([P.VEL_BYTE, 100, fr_v.events[0].keys[0]]) in blob_v, blob_v.hex())
    check("① encode -> decode 力度还原",
          [e.vel for e in P.decode_frame(blob_v).events] == [100, 40],
          [e.vel for e in P.decode_frame(blob_v).events])
    check("① 报告里记了力度格数（2 格，最高 100）",
          rep_v.vel_events == 2 and rep_v.vel_max == 100,
          (rep_v.vel_events, rep_v.vel_max))
    check("① 报告文字里能看到力度那一行",
          "逐事件力度" in rep_v.text(), rep_v.text().replace("\n", " | ")[:200])

    #   ★ ③ 和弦：三个音挤在同一个时间格 -> 用最大的那个 velocity
    _mkmid(tmp, [(0, [55], 1, 30), (0, [59], 1, 90), (0, [62], 1, 20)])
    fr_c, blob_c, rep_c = midi_to_frame(tmp)
    check("③ 和弦（3 个 note_on 同一格）取最大力度 90",
          len(fr_c.events) == 1 and fr_c.events[0].vel == 90,
          [(e.keys, e.vel) for e in fr_c.events])
    check("③ 和弦只发一个 0xC3（不是每个音一个）",
          blob_c.count(P.VEL_BYTE) == 1, blob_c.count(P.VEL_BYTE))
    check("③ 和弦的 0xC3 在 0xC0 前面（没塞进和弦里面）",
          blob_c.index(bytes([P.VEL_BYTE])) < blob_c.index(bytes([P.CHORD_ON])),
          blob_c.hex())

    #   ★ ② velocity == 0 是 note_off，不能当成"力度 0"
    import mido as _m
    _vz = os.path.join(tempfile.gettempdir(), "_fpga_midi_vel0.mid")
    _mz = _m.MidiFile(ticks_per_beat=480)
    _tz = _m.MidiTrack()
    _mz.tracks.append(_tz)
    _tz.append(_m.MetaMessage("set_tempo", tempo=500000, time=0))
    _tz.append(_m.Message("note_on", note=60, velocity=0, time=0))      # ★ = 抬手
    _tz.append(_m.Message("note_on", note=60, velocity=90, time=0))
    _tz.append(_m.Message("note_off", note=60, velocity=0, time=480))
    _mz.save(_vz)
    fr_z, blob_z, rep_z = midi_to_frame(_vz)
    check("② velocity=0 的 note_on 被当成 note_off（不是力度 0）",
          [e.vel for e in fr_z.events] == [90] and fr_z.events[0].keys == (13,),
          [(e.keys, e.vel) for e in fr_z.events])
    check("② 帧里没有 C3 00（没有「力度 0」这种东西）",
          bytes([P.VEL_BYTE, 0]) not in blob_z, blob_z.hex())
    check("② 力度全为 0 的文件不会产出 0xC3 力度 0", rep_z.vel_max == 90, rep_z.vel_max)

    #   ★ 力度正好 127：板子默认就是 127，所以不发 0xC3（结果一样，字节更省）
    _mkmid(tmp, [(0, [69], 1, 127), (1, [71], 1, 127)])
    _, blob_127, _ = midi_to_frame(tmp)
    check("★ 力度正好 127 -> 一个 0xC3 都不发（板子默认就是 127）",
          P.VEL_BYTE not in blob_127, blob_127.hex())

    #   ★★ ⑤ 关掉开关：一个 0xC3 都没有，而且要和"改造前"逐字节相同
    _mkmid(tmp, [(0, [69], 1, 100), (1, [71], 1, 40)])
    _fr_on, _blob_on, _ = midi_to_frame(tmp, use_velocity=True)
    _fr_off, _blob_off, _rep_off = midi_to_frame(tmp, use_velocity=False)
    check("★★ 关掉开关：帧里一个 0xC3 都没有",
          P.VEL_BYTE not in _blob_off, _blob_off.hex())
    check("★★ 关掉时所有事件都是默认 127（板子按满幅处理）",
          all(e.vel == P.VEL_DEFAULT for e in _fr_off.events),
          [e.vel for e in _fr_off.events])
    check("★★ 关掉时报告里说明了「一个 0xC3 都不发」",
          _rep_off.vel_disabled and "逐字节相同" in _rep_off.text(),
          _rep_off.text().replace("\n", " | ")[-160:])
    #   ★ 关掉后的帧 == 把同一份 Event 的力度全清成 127 再编码（= 改造前的行为）
    _blob_pre = P.encode_frame(P.Frame(
        events=[P.Event(keys=e.keys, ticks=e.ticks, oct=e.oct,
                        key_octs=e.key_octs) for e in _fr_on.events],
        tick_us=_fr_on.tick_us, rel_us=_fr_on.rel_us))
    check("★★ 关掉产出的帧 = 改造前那种帧（逐字节相同）",
          _blob_off == _blob_pre, (_blob_off.hex(), _blob_pre.hex()))
    check("开着和关掉的帧长度差 = 每个带力度的事件 2 字节",
          len(_blob_on) - len(_blob_off)
          == 2 * sum(1 for e in _fr_on.events if e.vel != P.VEL_DEFAULT),
          (len(_blob_on), len(_blob_off)))

    #   ★★ ④ 没有力度信息的来源：手工曲谱那一类（Event 根本不填 vel）
    _manual = P.Frame(events=[P.Event(keys=(13,), ticks=2),        # 手工谱：没有力度
                              P.Event(keys=(17, 20), ticks=1),
                              P.Event(keys=(), ticks=1)])
    check("★★ 手工曲谱（没有力度信息）：帧里一个 0xC3 都没有",
          P.VEL_BYTE not in P.encode_frame(_manual))
    #   ★ 这一条同时守住"MIDI 力度"功能不碰 score_import 那条路：
    #     score_import.arrange() 造的就是这种"不填 vel 的 Event"。
    check("★ 手工曲谱的帧 = 加力度功能之前的帧（逐字节相同）",
          P.encode_frame(_manual) == P.encode_frame(P.Frame(
              events=[P.Event(keys=(13,), ticks=2, vel=P.VEL_DEFAULT),
                      P.Event(keys=(17, 20), ticks=1, vel=P.VEL_DEFAULT),
                      P.Event(keys=(), ticks=1, vel=P.VEL_DEFAULT)])))

    #   ★ 和弦归并窗口没被 0xC3 搞乱：归并之后力度取的是并进来的那几个音的最大值
    #     （起音差 <= 25ms 且真重叠且音高不同 -> 并成同一格）
    _mkmid(tmp, [(0, [60], Fraction(1, 2), 30),
                 (Fraction(1, 100), [64], Fraction(1, 2), 110)])   # 差 5ms，真重叠
    _fr_mg, _blob_mg, _rep_mg = midi_to_frame(tmp)
    check("★ 和弦归并窗口照旧（两个音并到同一格）",
          _rep_mg.merged_notes >= 1 and len([e for e in _fr_mg.events if e.keys]) == 1,
          ([e.keys for e in _fr_mg.events], _rep_mg.merged_notes))
    check("★ 归并后的那一格力度 = 并进来的音里最大的（30 和 110 -> 110）",
          max((e.vel for e in _fr_mg.events), default=0) == 110,
          [e.vel for e in _fr_mg.events])

    for _p in (_vz,):
        try:
            os.remove(_p)
        except OSError:
            pass

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
