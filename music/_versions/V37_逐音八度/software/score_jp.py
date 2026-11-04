# -*- coding: utf-8 -*-
"""score_jp.py —— 结构化简谱（.jp.txt）解析器 + 校验器

格式规范见：…\\music\\tools\\协议说明_v2_和弦扩展.md 与同目录 README 说明。

本模块做四件事（全部纯逻辑、零第三方依赖）：
    1. parse()     文本 -> Score（调号/拍号/速度/小节/音符/和弦标记）
    2. validate()  强校验：每小节拍数、音域、和弦符号、反复配对、可疑时值写法
    3. repeat()    把反复记号展开成线性小节序列（用于算总时长）
    4. render()    回渲染成简谱样式，供人拿原图逐小节核对

命令行：
    python score_jp.py check 文件.txt        只校验
    python score_jp.py render 文件.txt       回渲染
    python score_jp.py selftest              自检
"""

from __future__ import annotations

import io
import os
import re
import sys
from dataclasses import dataclass, field
from fractions import Fraction
from typing import Dict, List, Optional, Sequence, Tuple

# ==========================================================================
# 板子音域与【键号 <-> 音高】换算
#
# ★★★ 键号是【接线顺序】，不是音高顺序！★★★
#
#   权威依据：top.v:1116-1220 的 delta[] 表，再用 top.v:1103 的注释
#   "每个黑键 = 它左边那个白键升半音（和钢琴一样）"交叉验证：
#
#     物理位(从左到右)  1    2    3    4    5    6    7    8    9   10   11   12
#     颜色              白   黑   白   黑   白   白   黑   白   黑   白   黑   白
#     模块丝印键号       6    1    7    2    8    9    3   10    4   11    5   12
#     板子键号           5    0    6    1    7    8    2    9    3   10    4   11
#     频率(Hz)         110  117  123  131  139  147  156  165  175  185  196  208
#     音名              A2  A#2   B2   C3  C#3   D3  D#3   E3   F3  F#3   G3  G#3
#
#   12 个键正好构成 A2->G#3 的完整半音阶；板子 36 键 = A2..G#5 = MIDI 45..80。
#   最低音 A2 在【键号 5】，不在键号 0。
#
#   ⚠️ tools\协议说明.md 里"键号 0 = A2、24 = A4"那张表是【错的】；
#      按那张表发字节音序会乱。以本文件的置换表为准。
# ==========================================================================
BOARD_MIDI_MIN = 45          # 键号 5（物理最左键）= A2 = MIDI 45
BOARD_MIDI_MAX = 80          # 键号 35             = G#5 = MIDI 80

# 片内半音序号(0=A) -> 片内板子键号(0..11)
KEY_OF_SEMITONE = [5, 0, 6, 1, 7, 8, 2, 9, 3, 10, 4, 11]
# 反查表，避免手抄出错
SEMITONE_OF_KEY = [KEY_OF_SEMITONE.index(i) for i in range(12)]

_PC_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]


def _oct_name(midi: int) -> str:
    """MIDI 45 -> 'A2'（按通行的 A4=440Hz 记法）"""
    return "%s%d" % (_PC_NAMES[midi % 12], midi // 12 - 1)


def midi_to_key(midi: int) -> int:
    """MIDI 音符号 -> 板子键号(0..35)；超出 A2..G#5 返回 -1。"""
    d = midi - BOARD_MIDI_MIN
    if not 0 <= d <= (BOARD_MIDI_MAX - BOARD_MIDI_MIN):
        return -1
    octv, semi = divmod(d, 12)
    return KEY_OF_SEMITONE[semi] + 12 * octv


def key_to_midi(key: int) -> int:
    """板子键号 -> MIDI 音符号；非法键号返回 -1。"""
    if not 0 <= key < 36:
        return -1
    octv, pos = divmod(key, 12)
    return BOARD_MIDI_MIN + 12 * octv + SEMITONE_OF_KEY[pos]


# 按键号索引的名字表（名字必须跟键号走，不能按音高排）
KEY_NAMES = [_oct_name(key_to_midi(k)) for k in range(36)]


def midi_to_keyno(m: int) -> int:
    """兼容旧名"""
    return midi_to_key(m)


def keyno_name(k: int) -> str:
    return KEY_NAMES[k] if 0 <= k < len(KEY_NAMES) else "越界"

PC = {"C": 0, "C#": 1, "DB": 1, "D": 2, "D#": 3, "EB": 3, "E": 4, "F": 5,
      "F#": 6, "GB": 6, "G": 7, "G#": 8, "AB": 8, "A": 9, "A#": 10, "BB": 10, "B": 11}
PC_NAME = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]


class ScoreError(ValueError):
    """解析阶段就失败（语法错误）。"""


# ==========================================================================
# 数据结构
# ==========================================================================
@dataclass
class Cell:
    """一个时间位置：旋律音（0 个=休止，>1 个=叠置和弦音） + 时值 + 可选和弦标记"""
    degrees: List[Tuple[int, int]] = field(default_factory=list)  # [(音级0-7, 变音-1/0/1)]
    octave: int = 0                    # 八度偏移（' = +1, , = -1），所有音共用
    dur: Fraction = Fraction(1)        # 拍数
    chord: Optional[str] = None        # 该位置起效的和弦符号
    tie: bool = False                  # 连音线（与后一同名音合并）
    src: str = ""                      # 原始记号文本，便于定位错误

    @property
    def is_rest(self) -> bool:
        return not self.degrees or all(d == 0 for d, _ in self.degrees)

    def notes_desc(self) -> str:
        if self.is_rest:
            return "0"
        out = []
        for d, a in self.degrees:
            s = ("#" if a > 0 else "b" if a < 0 else "") + str(d)
            s += "'" * self.octave if self.octave > 0 else "," * (-self.octave)
            out.append(s)
        return "[%s]" % " ".join(out) if len(out) > 1 else out[0]


@dataclass
class Measure:
    index: int
    cells: List[Cell] = field(default_factory=list)
    repeat_start: bool = False
    repeat_end: bool = False
    measure_repeat: bool = False        # % 记号

    @property
    def beats(self) -> Fraction:
        return sum((c.dur for c in self.cells), Fraction(0))

    def chords(self) -> List[str]:
        return [c.chord for c in self.cells if c.chord]


@dataclass
class Score:
    key_pc: int = 7                    # 1 对应的音高类（默认 G）
    key_text: str = "G"
    meter_num: int = 4
    meter_den: int = 4
    tempo: Optional[int] = None
    tempo_text: str = ""
    meters: List[Tuple[int, int]] = field(default_factory=list)   # 混合拍号（全部记下来）
    measures: List[Measure] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    # ---- 派生 ----
    @property
    def beat_title(self) -> Fraction:
        """一拍等于多少（以四分音符=1 为单位）。3/4 -> 1；6/8 -> 3/2（附点四分）"""
        return Fraction(4, self.meter_den)

    @property
    def measure_beats(self) -> Fraction:
        """一小节应有的拍数（以四分音符=1 为单位）"""
        return Fraction(self.meter_num) * self.beat_title


# ==========================================================================
# 音级 -> MIDI
# ==========================================================================
MAJOR_STEPS = [0, 2, 4, 5, 7, 9, 11]      # 大调音阶各音级相对主音的半音数


def degree_to_midi(degree: int, accidental: int, octave: int,
                   key_pc: int, center_octave: int = 4) -> int:
    """简谱音级 -> 标准 MIDI 音符号。

    约定（与中文简谱惯例一致）：
      · `1=C` 时中音 1 = C4 = MIDI 60；`1=G` 时中音 1 = G4 = MIDI 67
      · 音级 1..7 用【大调音阶】取音，且**相对主音累加半音**
        （★ 不能按音高类放到同一八度里 —— 1=G 的 5 是 D5 而不是 D4，
          因为 G 大调音阶 G4..F#5 本来就跨过了 C 的八度分界）
      · 变音 # / b 再额外升降半音
      · octave 是相对中音区的八度偏移（' = +1）
    """
    if degree == 0:
        return -1                                    # 休止
    tonic = 12 * (center_octave + 1) + key_pc        # 中音区 1 的位置
    return tonic + MAJOR_STEPS[(degree - 1) % 7] + accidental + 12 * octave


def midi_to_keyno(m: int) -> int:
    return m - BOARD_MIDI_MIN


def keyno_name(k: int) -> str:
    return KEY_NAMES[k] if 0 <= k < len(KEY_NAMES) else "越界"


# ==========================================================================
# 和弦符号 -> 音程
# ==========================================================================
# 支持两种写法：
#   级数式（推荐，简谱通用）： <1> <2m> <5> <4> <6m> <5 7> <2> <1M7>
#   字母式：                  <G> <Am> <D7> <C> <Em> <G/B>
_QUALITY_INTERVALS = {
    "":     [0, 4, 7],            # 大三
    "M":    [0, 4, 7],
    "maj":  [0, 4, 7],
    "m":    [0, 3, 7],            # 小三
    "min":  [0, 3, 7],
    "-":    [0, 3, 7],
    "dim":  [0, 3, 6],
    "aug":  [0, 4, 8],
    "+":    [0, 4, 8],
    "sus4": [0, 5, 7],
    "sus2": [0, 2, 7],
}

_CHORD_DEG_RE = re.compile(
    r"^\s*(?P<acc>[#b]?)(?P<deg>[1-7])\s*"
    r"(?P<qual>maj|min|dim|aug|sus[24]|m|M|M7|\+|-)?\s*"
    r"(?P<ext>[769]|7|9)?\s*$")

_CHORD_LETTER_RE = re.compile(
    r"^\s*(?P<root>[A-Ga-g][#b]?)\s*"
    r"(?P<qual>maj|min|dim|aug|sus[24]|m|M|\+|-)?\s*"
    r"(?P<ext>7|9|6)?\s*$")


def chord_root_midi(chord: str, key_pc: int, center_octave: int = 3) -> List[int]:
    """和弦符号 -> 该处同时发声的 MIDI 音列表（根音放在 center_octave 八度）。

    级数式按【大调音阶级数 + 三度叠置】展开；字母式按音名字母展开。
    """
    name = chord.strip()
    if not name:
        raise ScoreError("空的和弦符号")

    root_pc: Optional[int] = None
    quality = ""
    ext = ""

    m = _CHORD_DEG_RE.match(name)
    if m and m.group("deg"):
        deg = int(m.group("deg"))
        root_pc = (key_pc + MAJOR_STEPS[(deg - 1) % 7]) % 12
        if m.group("acc") == "#":
            root_pc = (root_pc + 1) % 12
        elif m.group("acc") == "b":
            root_pc = (root_pc - 1) % 12
        quality = m.group("qual") or ""
        ext = m.group("ext") or ""
    else:
        m2 = _CHORD_LETTER_RE.match(name)
        if not m2:
            raise ScoreError("看不懂的和弦符号: <%s>" % name)
        tok = m2.group("root").upper()
        root_pc = PC.get(tok)
        if root_pc is None:
            raise ScoreError("看不懂的音名: <%s>" % name)
        quality = m2.group("qual") or ""
        ext = m2.group("ext") or ""

    # 七和弦：默认属七（小七度）；M7/maj7 用大七度
    if ext in ("7", "9"):
        third = 3 if quality in ("m", "min", "-") else 4
        seventh = 11 if quality in ("M", "maj") else 10
        intervals = [0, third, 7, seventh]
        if ext == "9":
            intervals.append(14)
    elif ext == "6":
        intervals = [0, 4, 7, 9]
    else:
        intervals = list(_QUALITY_INTERVALS.get(quality, [0, 4, 7]))

    base = 12 * (center_octave + 1) + root_pc
    return [base + iv for iv in intervals]


# ==========================================================================
# 解析
# ==========================================================================
_META_RE = re.compile(r"^\s*(key|meter|tempo)\s*[:：]\s*(.*)$", re.I)
# 注释行：行首 '#' 后面【不是】0..7 —— '#4 5 6' 是音乐，'# 标题' 是注释
_COMMENT_RE = re.compile(r"^\s*#(?![0-7])")
_KEY_RE = re.compile(r"1\s*=\s*(?P<acc>[#b♯♭]?)\s*(?P<name>[A-Ga-g])")
_METER_RE = re.compile(r"(\d+)\s*/\s*(\d+)")

_DUR_CHARS = "-./~"


def _scan_cell(s: str, i: int) -> Tuple[Cell, int]:
    """从 s[i] 起扫一个音符记号，返回 (Cell, 新位置)"""
    n = len(s)
    start = i
    acc = 0
    if i < n and s[i] in "#b":
        acc = 1 if s[i] == "#" else -1
        i += 1
    if i >= n or not s[i].isdigit():
        raise ScoreError("位置 %d 处不是合法音符: %r" % (start, s[start:start + 12]))
    deg = int(s[i])
    i += 1
    if deg > 7:
        raise ScoreError("音级只能是 0..7，位置 %d 得到 %d" % (start, deg))

    octave = 0
    while i < n and s[i] in "',":
        octave += 1 if s[i] == "'" else -1
        i += 1

    marks = []
    # ★ 时值记号之间容许夹空格：简谱里 `6 - - -` 很常见（增时线一个个写开）。
    #   判据是"空格后面还得跟时值记号"，否则 `5 4/4/` 里的空格会被误吞 ——
    #   因为 `4` 不是时值记号，扫描会在空格处停下。
    while i < n:
        if s[i] in _DUR_CHARS:
            marks.append(s[i])
            i += 1
        elif s[i] == " ":
            j = i
            while j < n and s[j] == " ":
                j += 1
            if j < n and s[j] in _DUR_CHARS:
                i = j
            else:
                break
        else:
            break

    # 时值：下划线 / = 减半；附点 . = ×1.5；增时线 - = +1 拍
    k = marks.count("/")
    dots = marks.count(".")
    adds = marks.count("-")
    tie = "~" in marks
    dur = Fraction(1, 2 ** k)
    if dots:
        dur *= Fraction(3, 2) ** dots
    dur += adds

    cell = Cell(degrees=[(deg, acc)], octave=octave, dur=dur, tie=tie,
                src=s[start:i])
    return cell, i


def parse(text: str, verbose: bool = True) -> Score:
    """把 .jp.txt 文本解析成 Score。语法错误抛 ScoreError 并带位置提示。"""
    sc = Score()
    body_lines: List[str] = []

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        # 注释行：'//'、'【…】'、'##' 一律注释；
        # 单独一个 '#' 后面【不跟音级数字】也算注释（这样 '#4 5 6' 仍是音乐，
        # 而 '# 小步舞曲' 是注释）—— 见 _COMMENT_RE
        if line.startswith("//") or line.startswith("【") or line.startswith("##"):
            continue
        if _COMMENT_RE.match(line):
            continue
        m = _META_RE.match(line)
        if m:
            k, v = m.group(1).lower(), m.group(2).strip()
            if k == "key":
                km = _KEY_RE.search(v) or _KEY_RE.search(line)
                if not km:
                    sc.warnings.append("key 行看不懂: %r（例：key: 1=G）" % line)
                else:
                    a = km.group("acc")
                    # ★ 支持 Unicode 的 ♯ / ♭（简谱里常写成 1=♭B）
                    a = {"♯": "#", "♭": "b"}.get(a, a)
                    nm = (a + km.group("name")).upper() if a else km.group("name").upper()
                    pc = PC.get(nm)
                    if pc is None:
                        sc.warnings.append("调号音名看不懂: %r" % nm)
                    else:
                        sc.key_pc, sc.key_text = pc, nm
            elif k == "meter":
                # ★ 支持混合拍号（如 `6/8 9/8 12/8`）：全部记下来，
                #   小节校验用第一个；其余只做记录，不改变音符时值。
                pairs = re.findall(r"(\d+)\s*/\s*(\d+)", v)
                if not pairs:
                    sc.warnings.append("meter 行看不懂: %r（例：meter: 3/4）" % line)
                else:
                    sc.meters = [(int(x), int(y)) for x, y in pairs]
                    sc.meter_num, sc.meter_den = sc.meters[0]
                    for _n, _d in sc.meters:
                        if _d not in (2, 4, 8, 16):
                            sc.warnings.append("拍号分母 %d 不常见" % _d)
                    if len(sc.meters) > 1:
                        sc.warnings.append(
                            "混合拍号 %s：音符时值不受影响，但小节拍数校验只能按"
                            "第一个 %d/%d 做（其余拍号仅记录）"
                            % (" ".join("%d/%d" % t for t in sc.meters),
                               sc.meter_num, sc.meter_den))
            elif k == "tempo":
                sc.tempo_text = v
                tm = re.search(r"(\d+)", v)
                if tm:
                    sc.tempo = int(tm.group(1))
                elif v:
                    guess = {"慢速": 60, "中速": 100, "快速": 140,
                             "中速稍快": 120, "中速稍慢": 88}
                    sc.tempo = guess.get(v)
                    if sc.tempo is None:
                        sc.warnings.append("tempo 文本 %r 无法解析，按 100 处理" % v)
                        sc.tempo = 100
                else:
                    sc.tempo = 100
                    sc.warnings.append("tempo 为空，暂按 ♩=100 处理（可改）")
            continue
        body_lines.append(line)

    # ---- 逐行扫描成记号流 ----
    tokens: List[Tuple[str, object]] = []
    for line in body_lines:
        i, n = 0, len(line)
        while i < n:
            c = line[i]
            if c.isspace():
                i += 1
                continue
            if c == "<":
                j = line.find(">", i)
                if j < 0:
                    raise ScoreError("和弦标记 < 没有闭合: %r" % line[i:i + 20])
                tokens.append(("chord", line[i + 1:j].strip()))
                i = j + 1
                continue
            for pat, kind in (("||:", "rst"), (":||", "ren"), ("|:", "rst"),
                              (":|", "ren")):
                if line.startswith(pat, i):
                    tokens.append((kind, None))
                    i += len(pat)
                    break
            else:
                if c == "|":
                    tokens.append(("bar", None))
                    i += 1
                elif c == "%":
                    tokens.append(("mrep", None))
                    i += 1
                else:
                    cell, i = _scan_cell(line, i)
                    tokens.append(("cell", cell))
                continue
            continue

    # ---- 记号流 -> 小节 ----
    cur = Measure(index=1)
    pending_chord: Optional[str] = None
    started = False
    for kind, payload in tokens:
        if kind == "chord":
            pending_chord = str(payload)
            continue
        if kind == "rst":
            if cur.cells or cur.repeat_start or cur.repeat_end:
                sc.measures.append(cur)
                cur = Measure(index=len(sc.measures) + 1)
            cur.repeat_start = True
            started = True
            continue
        if kind == "ren":
            cur.repeat_end = True
            sc.measures.append(cur)
            cur = Measure(index=len(sc.measures) + 1)
            continue
        if kind == "bar":
            if cur.cells or cur.repeat_start or cur.repeat_end:
                sc.measures.append(cur)
                cur = Measure(index=len(sc.measures) + 1)
            continue
        if kind == "mrep":
            cur.measure_repeat = True
            sc.measures.append(cur)
            cur = Measure(index=len(sc.measures) + 1)
            continue
        # cell
        cell: Cell = payload                      # type: ignore[assignment]
        if pending_chord:
            cell.chord = pending_chord
            pending_chord = None
        cur.cells.append(cell)
        started = True

    if cur.cells or cur.repeat_start or cur.repeat_end:
        sc.measures.append(cur)

    # 去掉可能出现的空小节（行末多余的小节线造成）
    sc.measures = [m for m in sc.measures
                   if m.cells or m.repeat_start or m.repeat_end or m.measure_repeat]
    for k, m in enumerate(sc.measures, 1):
        m.index = k

    if not sc.measures:
        raise ScoreError("没有解析到任何小节（检查是否缺少 | 小节线）")

    if verbose:
        print("解析：调号 1=%s，拍号 %d/%d，tempo=%s，%d 个小节"
              % (sc.key_text, sc.meter_num, sc.meter_den, sc.tempo, len(sc.measures)))
    return sc


# ==========================================================================
# 校验
# ==========================================================================
@dataclass
class Problem:
    level: str        # "错误" / "警告" / "提示"
    where: str
    detail: str

    def __str__(self) -> str:
        return "[%s] %s：%s" % (self.level, self.where, self.detail)


def validate(sc: Score, strict: bool = True) -> List[Problem]:
    """全部校验。返回问题列表（空 = 全部通过）。

    strict=False：把"小节拍数不符"从【错误】降级为【提示】。
    用于小节线是手打、本来就不规整的谱子（手抄谱、混合拍号谱）——
    音符时值本身是自洽的，缺的只是小节线，不该因此拒绝生成。
    """
    ps: List[Problem] = []
    want = sc.measure_beats

    # ---- 1) 每小节拍数 ----
    for m in sc.measures:
        if m.measure_repeat:
            continue
        if not m.cells:
            continue
        got = m.beats
        if got != want:
            # 额外判断：是否可能只是"下划线漏写"
            hint = ""
            n_notes = len([c for c in m.cells if not c.is_rest])
            if got == Fraction(n_notes) and n_notes > 0 and Fraction(n_notes) != want:
                gap = Fraction(n_notes) - want
                need = "每音约 %s 拍" % (float(want / n_notes))
                hint = "（本小节 %d 个音，若都不是整拍请补下划线：%s）" % (n_notes, need)
            ps.append(Problem("错误" if strict else "提示", "第 %d 小节" % m.index,
                              "拍数 %s，应为 %s%s"
                              % (_f(got), _f(want), hint)))

    # ---- 2) 音域（按 key 换算，先看是否需要整体移八度）----
    all_midi: List[Tuple[int, int]] = []          # (midi, measure_index)
    for m in sc.measures:
        for c in m.cells:
            for d, a in c.degrees:
                if d == 0:
                    continue
                all_midi.append((degree_to_midi(d, a, c.octave, sc.key_pc), m.index))
    if all_midi:
        lo = min(x[0] for x in all_midi)
        hi = max(x[0] for x in all_midi)
        span = hi - lo
        ps.append(Problem("提示", "音域",
                          "旋律 MIDI %d..%d（%s..%s），跨度 %d 个半音；"
                          "板子可用 %d..%d（%s..%s）"
                          % (lo, hi, PC_NAME[lo % 12] + str(lo // 12 - 1),
                             PC_NAME[hi % 12] + str(hi // 12 - 1), span,
                             BOARD_MIDI_MIN, BOARD_MIDI_MAX,
                             keyno_name(0), keyno_name(35))))
        shift = None
        for s in (0, -12, 12, -24, 24):
            if lo + s >= BOARD_MIDI_MIN and hi + s <= BOARD_MIDI_MAX:
                shift = s
                break
        if shift is None:
            ps.append(Problem("错误", "音域",
                              "跨度 %d 个半音，超出板子 %d 个半音的能力，必须改编"
                              % (span, BOARD_MIDI_MAX - BOARD_MIDI_MIN)))
        elif shift != 0:
            ps.append(Problem("警告", "音域",
                              "需整体移八度 %+d 个半音（%s..%s -> %s..%s）才落进板子音域"
                              % (shift,
                                 PC_NAME[lo % 12] + str(lo // 12 - 1),
                                 PC_NAME[hi % 12] + str(hi // 12 - 1),
                                 PC_NAME[(lo + shift) % 12] + str((lo + shift) // 12 - 1),
                                 PC_NAME[(hi + shift) % 12] + str((hi + shift) // 12 - 1))))
        out = [(mi, mn) for mi, mn in all_midi
               if not (BOARD_MIDI_MIN <= mi <= BOARD_MIDI_MAX)]
        if out and shift == 0:
            by_m: Dict[int, int] = {}
            for _, mn in out:
                by_m[mn] = by_m.get(mn, 0) + 1
            ps.append(Problem("警告", "音域",
                              "有 %d 个音超出板子音域，分布在 %s"
                              % (len(out),
                                 "、".join("第%d小节(%d个)" % (k, v)
                                          for k, v in sorted(by_m.items())))))

    # ---- 3) 和弦符号 ----
    for m in sc.measures:
        for c in m.cells:
            if not c.chord:
                continue
            try:
                tones = chord_root_midi(c.chord, sc.key_pc)
                if not tones:
                    raise ScoreError("空")
            except ScoreError as exc:
                ps.append(Problem("错误", "第 %d 小节" % m.index,
                                  "和弦符号 %s 无法解析：%s" % (c.chord, exc)))

    # ---- 4) 反复记号配对 ----
    open_at: Optional[int] = None
    for m in sc.measures:
        if m.repeat_start:
            if open_at is not None:
                ps.append(Problem("警告", "第 %d 小节" % m.index,
                                  "重复的开始记号 ||: 嵌套（上一个在第 %d 小节）"
                                  % open_at))
            open_at = m.index
        if m.repeat_end:
            if open_at is None:
                ps.append(Problem("提示", "第 %d 小节" % m.index,
                                  "有 :|| 但没有配对的 ||:（表示从头反复）"))
            open_at = None
    if open_at is not None:
        ps.append(Problem("警告", "第 %d 小节" % open_at, "||: 没有配对的 :||"))

    # ---- 5) 可疑时值写法（. 与 / 同时出现，最常见混淆）----
    for m in sc.measures:
        for c in m.cells:
            if "." in c.src and "/" in c.src:
                ps.append(Problem("提示", "第 %d 小节" % m.index,
                                  "记号 %r 同时含附点和小划线（附点八分=0.75拍）。"
                                  "若原意是'附点四分 + 八分'，请写成两个记号：%s. %s/"
                                  % (c.src, c.src.split(".")[0],
                                     c.src.split("/")[-1] or "?")))

    # ---- 6) 元信息 ----
    if sc.tempo is None:
        ps.append(Problem("警告", "元信息", "缺少 tempo"))
    if not sc.measures[0].repeat_start and not sc.measures[0].cells:
        ps.append(Problem("提示", "元信息", "第一小节是空的"))

    return ps


def _f(x: Fraction) -> str:
    return str(x) if x.denominator != 1 else str(x.numerator)


# ==========================================================================
# 反复展开
# ==========================================================================
def expand_repeats(sc: Score) -> List[Tuple[Measure, int]]:
    """把反复记号展开成演奏顺序。返回 [(小节, 第几遍)]"""
    out: List[Tuple[Measure, int]] = []
    i = 0
    n = len(sc.measures)
    while i < n:
        m = sc.measures[i]
        if m.repeat_start:
            j = i
            while j < n and not sc.measures[j].repeat_end:
                j += 1
            block = sc.measures[i:(j + 1 if j < n else n)]
            for rep in (1, 2):
                for b in block:
                    out.append((b, rep))
            i = (j + 1) if j < n else n
            continue
        if m.repeat_end:
            out.append((m, 2))
            i += 1
            continue
        out.append((m, 1))
        i += 1
    return out


# ==========================================================================
# 回渲染（供人拿原图核对）
# ==========================================================================
def render(sc: Score, per_line: int = 4) -> str:
    lines = ["key: 1=%s" % sc.key_text,
             "meter: %d/%d" % (sc.meter_num, sc.meter_den),
             "tempo: %s" % (sc.tempo if sc.tempo is not None else "?"),
             ""]
    buf: List[str] = []
    for m in sc.measures:
        parts: List[str] = []
        if m.repeat_start:
            parts.append("||:")
        if m.measure_repeat:
            parts.append("%")
        for c in m.cells:
            txt = c.notes_desc()
            if c.tie:
                txt += "~"
            marks = _marks_of(c)
            parts.append(txt + marks)
        if m.repeat_end:
            parts.append(":||")
        piece = " ".join(parts)
        buf.append("%-46s   # 第%d小节 %s拍" % (piece, m.index, _f(m.beats)))
        if len(buf) >= per_line:
            lines.extend(buf)
            lines.append("")
            buf = []
    if buf:
        lines.extend(buf)
    return "\n".join(lines)


def _marks_of(c: Cell) -> str:
    """由 dur 反推时值记号（用于回渲染核对）。

    直接穷举候选写法再匹配，比"逆向拆解分母"可靠：
      · k 条下划线 -> 基准 1/2^k 拍
      · dots 个附点 -> ×(3/2)^dots
      · adds 条增时线 -> +adds 拍
    """
    dur = c.dur
    for k in range(5):
        base = Fraction(1, 2 ** k)
        for dots in range(3):
            v = base * Fraction(3, 2) ** dots
            for adds in range(0, 16):
                if v + adds == dur:
                    return "." * dots + "/" * k + "-" * adds
    return "?" + str(dur)


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

    print("[1] 记号扫描（含无空格连写）")
    cases = [
        ("5", (5, 0, 0, Fraction(1))),
        ("1'", (1, 0, 1, Fraction(1))),
        ("7,", (7, 0, -1, Fraction(1))),
        ("#4", (4, 1, 0, Fraction(1))),
        ("3/", (3, 0, 0, Fraction(1, 2))),
        ("5--", (5, 0, 0, Fraction(3))),
        ("1.", (1, 0, 0, Fraction(3, 2))),
        ("1./", (1, 0, 0, Fraction(3, 4))),
        ("5//", (5, 0, 0, Fraction(1, 4))),
        ("0", (0, 0, 0, Fraction(1))),
    ]
    for src, (d, a, o, du) in cases:
        c, end = _scan_cell(src, 0)
        got = (c.degrees[0][0], c.degrees[0][1], c.octave, c.dur)
        check("%-5s -> %s" % (src, got), got == (d, a, o, du) and end == len(src))

    print("[2] 无空格连写切分")
    toks = []
    i = 0
    s = "3/4/"
    while i < len(s):
        c, i = _scan_cell(s, i)
        toks.append(c.notes_desc() + _marks_of(c))
    check("3/4/ -> ['3/','4/']", toks == ["3/", "4/"])

    s = "1./7/1"
    toks, i = [], 0
    while i < len(s):
        c, i = _scan_cell(s, i)
        toks.append(c.notes_desc() + _marks_of(c))
    check("1./7/1 -> ['1./','7/','1']", toks == ["1./", "7/", "1"])

    print("[3b] 键号 <-> 音高（★ 键号是接线顺序，不是音高顺序）")
    check("MIDI 45(A2) -> 键号 5", midi_to_key(45) == 5)
    check("MIDI 46(A#2) -> 键号 0", midi_to_key(46) == 0)
    check("MIDI 47(B2) -> 键号 6", midi_to_key(47) == 6)
    check("MIDI 50(D3) -> 键号 8", midi_to_key(50) == 8)
    check("MIDI 57(A3) -> 键号 17", midi_to_key(57) == 17)
    check("MIDI 69(A4/440Hz) -> 键号 29", midi_to_key(69) == 29)
    check("MIDI 80(G#5) -> 键号 35", midi_to_key(80) == 35)
    check("低于音域返回 -1", midi_to_key(44) == -1)
    check("高于音域返回 -1", midi_to_key(81) == -1)
    check("36 个键号往返一致",
          all(key_to_midi(midi_to_key(m)) == m for m in range(45, 81)))
    check("片内 12 键按键号排出来是升半音序",
          sorted(range(12), key=lambda k: key_to_midi(k)) == KEY_OF_SEMITONE)
    check("键号 5 的名字是 A2", KEY_NAMES[5] == "A2")
    check("键号 0 的名字是 A#2", KEY_NAMES[0] == "A#2")
    check("键号 29 的名字是 A4", KEY_NAMES[29] == "A4")

    print("[3] 音级 -> MIDI")
    check("1=G 中音1 -> G4(67)", degree_to_midi(1, 0, 0, PC["G"]) == 67)
    check("1=G 中音5 -> D5(74)", degree_to_midi(5, 0, 0, PC["G"]) == 74)
    check("1=G 高音1 -> G5(79)", degree_to_midi(1, 0, 1, PC["G"]) == 79)
    check("1=G 低音7 -> F#4(66)", degree_to_midi(7, 0, -1, PC["G"]) == 66)
    check("1=G #4 -> C#5(73)", degree_to_midi(4, 1, 0, PC["G"]) == 73)

    print("[4] 和弦符号")
    g = chord_root_midi("1", PC["G"])       # I  = G B D
    check("1=G <1> -> G B D", [x % 12 for x in g] == [7, 11, 2])
    a = chord_root_midi("2m", PC["G"])      # ii = A C E
    check("1=G <2m> -> A C E", [x % 12 for x in a] == [9, 0, 4])
    d = chord_root_midi("5", PC["G"])       # V  = D F# A
    check("1=G <5> -> D F# A", [x % 12 for x in d] == [2, 6, 9])
    e = chord_root_midi("6m", PC["G"])      # vi = E G B
    check("1=G <6m> -> E G B", [x % 12 for x in e] == [4, 7, 11])
    a2 = chord_root_midi("2", PC["G"])      # II = A C# E
    check("1=G <2> -> A C# E", [x % 12 for x in a2] == [9, 1, 4])
    # ★ 注意：G 大调里 Am 是【ii 级】，不是 vi 级（vi 是 Em）
    check("字母式 <Am> 等于 <2m>（A C E）",
          [x % 12 for x in chord_root_midi("Am", PC["G"])] == [9, 0, 4])
    check("字母式 <Em> 等于 <6m>（E G B）",
          [x % 12 for x in chord_root_midi("Em", PC["G"])] == [4, 7, 11])
    check("<G7> 是四音", len(chord_root_midi("G7", PC["G"])) == 4)
    check("<Gmaj7> 七度是大七度(11)",
          [x % 12 for x in chord_root_midi("Gmaj7", PC["G"])][3] == 6)   # 11%12=11 -> F#=6

    print("[5] 解析 + 校验（好谱）")
    good = """key: 1=G
meter: 3/4
tempo: 120
<1> 5 1/ 2/ 3/4/ | <1> 5 1 1 | <4> 6 4/5/6/7/ | <1> 1' 1 1 |
"""
    sc = parse(good, verbose=False)
    ps = validate(sc)
    errs = [p for p in ps if p.level == "错误"]
    check("好谱没有错误", not errs)
    check("小节数 = 4", len(sc.measures) == 4)
    check("每小节 3 拍", all(m.beats == Fraction(3) for m in sc.measures))

    print("[6] 解析 + 校验（拍数错的谱）")
    bad = """key: 1=G
meter: 3/4
<1> 5 1 1 1 |
"""
    sc2 = parse(bad, verbose=False)
    ps2 = validate(sc2)
    check("抓到拍数错误", any(p.level == "错误" and "拍数" in p.detail for p in ps2))

    print("[7] 反复展开")
    rep = """key: 1=G
meter: 3/4
||: 5 5 5 | 6 6 6 :|| 7 7 7 |
"""
    sc3 = parse(rep, verbose=False)
    ex = expand_repeats(sc3)
    check("展开后小节数 = 5", len(ex) == 5)
    check("第一小节出现两遍", [m.index for m, _ in ex].count(1) == 2)

    print("[8] 回渲染")
    out = render(sc)
    check("回渲染含小节号", "第1小节" in out)

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

    if cmd in ("check", "render") and len(argv) >= 2:
        path = argv[1]
        with open(path, encoding="utf-8") as f:
            text = f.read()
        sc = parse(text)
        if cmd == "render":
            print(render(sc))
            return 0
        ps = validate(sc)
        errs = [p for p in ps if p.level == "错误"]
        warns = [p for p in ps if p.level == "警告"]
        infos = [p for p in ps if p.level == "提示"]
        for p in errs + warns + infos:
            print(p)
        print()
        print("合计：错误 %d，警告 %d，提示 %d" % (len(errs), len(warns), len(infos)))
        if errs:
            print("→ 有【错误】必须先修，否则节奏/音高一定不对。")
        elif warns:
            print("→ 没有错误，警告可以按需处理。")
        else:
            print("→ 通过。")
        return 1 if errs else 0

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
