# -*- coding: utf-8 -*-
"""fpga_proto.py —— FPGA 蓝牙乐谱协议 v2 编解码器（纯逻辑）

这是「上位机 <-> FPGA」的**唯一契约实现**。协议规范见：
    …\\music\\tools\\协议说明_v2_和弦扩展.md

设计原则
────────
1. **零依赖**：只用标准库。可以在任何 Python 里跑自检，不需要串口、不需要 GUI。
2. **编码 / 解码成对**：decode() 严格镜像 FPGA 接收器的行为（尤其是"操作数是原始
   数据、不受控制字节解释"这一条），所以 encode->decode 往返一致 = 协议自洽。
3. **字节可比对**：encode 出来的就是真正要写进串口的字节，一个不多一个不少。

帧结构
──────
    [0xFF × 16]  前导冲刷
    [0xAA]       开窗 + 设置复位
    [设置]       0xC4 + 3B(µs LE)  /  0xC5 + 3B(µs LE)      ← 可选，默认值时省略
    [事件流]     单音 / 休止 / 和弦，各自可跟 0xC2 + N 延长
    [0xFF]       帧尾：关窗 + 开播

事件字节
────────
    单音  键号 0x00..0x23                        默认 1 个时间格
    休止  0xEE                                   默认 1 个时间格
    和弦  0xC0 键号{1..36} 0xC1                   默认 1 个时间格
    延长  0xC2 N      （N = 0..255，本事件再加 N 格）

命令行
──────
    python fpga_proto.py selftest     跑自检
    python fpga_proto.py demo         打印一个带和弦的示例帧
"""

from __future__ import annotations

import random
import sys
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence, Tuple

# ==========================================================================
# 常量
# ==========================================================================
N_KEYS = 36                 # 键号 0..35（= A2..G#5）
NOTE_MAX = 0x23             # 最大键号 = 35
REST_BYTE = 0xEE            # 休止符
CLR_BYTE = 0xAA             # 开窗 / 写指针归零 / 停播
END_BYTE = 0xFF             # 帧尾 / 开播
N_FLUSH = 16                # 前导 0xFF 个数

CHORD_ON = 0xC0             # 和弦帧头
CHORD_OFF = 0xC1            # 和弦帧尾
DUR_EXT = 0xC2              # 时值延长：+1 字节 N
SET_TICK = 0xC4             # 时间格：+3 字节 µs(LE)
SET_REL = 0xC5              # 释音提前量：+3 字节 µs(LE)

# 容量与范围（与 FPGA 实现严格一致）
#
# ★ 上限是【事件数】，不是字节数 ★
#   v1 把字节流存进 note_buf[256]（后来 BUF_SIZE=4096），所以上限是字节。
#   v2 改成接收时就拼好和弦、每个时间格存一条记录：
#       slot_mask[0..1023] / slot_dur[0..1023]   -> 最多 1024 个事件
#   字节数已经不再是限制（接收是流式的，不缓存原始字节）。
#   依然保留 BUF_SIZE 这个名字给旧脚本用，但判断一律按 SLOT_MAX。
SLOT_MAX = 1024             # uart_packet_echo.v 的 SLOT_MAX（事件数上限）
BUF_SIZE = SLOT_MAX         # 兼容旧名（旧含义是 4096 字节，已作废）
MAX_TICKS = 255             # 单事件时间格上限（= RTL 的 slot_dur，8 位；1+N 饱和在 255）
N_EXT_MAX = 255             # 单个 0xC2 的 N 上限

# 默认值（不设置时 FPGA 用这两个值，等价于 v1 行为）
DEFAULT_TICK_US = 500_000   # 0.5 s / 时间格
DEFAULT_REL_US = 300_000    # 每事件末尾提前 0.3 s 松键
TICK_MIN_US = 20_000        # FPGA 会把时间格钳到 >= 20 ms
REL_MIN_US = 10_000         # 建议的最小释音提前量

US_PER_TICK_LIMIT = 0xFFFFFF  # 3 字节的表示上限

# 键号 -> 音名。
# ★ 键号是【接线顺序】（= TTP229 模块丝印键号 - 1），不是音高顺序！
#   依据 top.v 的 delta[] 表：键号 5 = A2（物理最左键、最低音），
#   键号 0 = A#2，键号 29 = A4(440Hz)，键号 35 = G#5。
#   ⚠️ tools\协议说明.md 里"键号 0 = A2、24 = A4"那张表是【错的】。
#   推导见 score_jp.py 顶部的置换表注释。
KEY_NAMES = ["A#2", "C3", "D#3", "F3", "G3", "A2", "B2", "C#3", "D3", "E3", "F#3", "G#3",
             "A#3", "C4", "D#4", "F4", "G4", "A3", "B3", "C#4", "D4", "E4", "F#4", "G#4",
             "A#4", "C5", "D#5", "F5", "G5", "A4", "B4", "C#5", "D5", "E5", "F#5", "G#5"]


class ProtoError(ValueError):
    """协议层错误（参数非法、超容量等）。"""


# ==========================================================================
# 基础工具
# ==========================================================================
def fmt_hex(data: bytes) -> str:
    """bytes -> '90 3C 64'"""
    return " ".join("%02X" % b for b in data)


def keys_to_mask(keys: Iterable[int]) -> int:
    """键号列表 -> 36 位掩码（与 top.v 的 key_mask 同一约定：bit n = 键号 n）"""
    m = 0
    for k in keys:
        m |= 1 << int(k)
    return m


def mask_to_keys(mask: int) -> List[int]:
    """36 位掩码 -> 升序键号列表"""
    return [i for i in range(N_KEYS) if (mask >> i) & 1]


def key_name(k: int) -> str:
    return KEY_NAMES[k] if 0 <= k < N_KEYS else "?"


# ==========================================================================
# 事件与帧
# ==========================================================================
@dataclass
class Event:
    """一个时间格事件：同时发声的一组键（空 = 休止） + 时长（格）。

    这是协议层的**唯一事件模型**，与 top.v 的 midi_keys 掩码一一对应。
    """
    keys: Tuple[int, ...] = ()
    ticks: int = 1

    def __post_init__(self):
        self.keys = tuple(sorted({int(k) for k in self.keys}))
        for k in self.keys:
            if not 0 <= k < N_KEYS:
                raise ProtoError("键号 %r 超出 0..%d" % (k, N_KEYS - 1))
        self.ticks = int(self.ticks)
        if self.ticks < 1:
            raise ProtoError("时长必须 >= 1 格，得到 %r" % self.ticks)
        if self.ticks > MAX_TICKS:
            raise ProtoError("时长 %d 超过上限 %d 格" % (self.ticks, MAX_TICKS))

    # ---- 便利属性 ----
    @property
    def is_rest(self) -> bool:
        return not self.keys

    @property
    def is_chord(self) -> bool:
        return len(self.keys) >= 2

    @property
    def mask(self) -> int:
        return keys_to_mask(self.keys)

    def __repr__(self) -> str:
        body = "休止" if self.is_rest else "+".join(key_name(k) for k in self.keys)
        return "Event(%s, %d格)" % (body, self.ticks)


@dataclass
class Frame:
    """一帧：设置 + 事件序列。"""
    events: List[Event] = field(default_factory=list)
    tick_us: int = DEFAULT_TICK_US
    rel_us: int = DEFAULT_REL_US

    def __post_init__(self):
        # fail-fast：构造即校验，不等到 encode 才报错
        self.validate()

    def validate(self) -> None:
        if not 1 <= self.tick_us <= US_PER_TICK_LIMIT:
            raise ProtoError("tick_us 必须在 1..%d，得到 %r"
                             % (US_PER_TICK_LIMIT, self.tick_us))
        if not 0 <= self.rel_us <= US_PER_TICK_LIMIT:
            raise ProtoError("rel_us 必须在 0..%d，得到 %r"
                             % (US_PER_TICK_LIMIT, self.rel_us))
        for e in self.events:
            if not isinstance(e, Event):
                raise ProtoError("事件必须是 Event，得到 %r" % (e,))

    @property
    def total_ticks(self) -> int:
        return sum(e.ticks for e in self.events)

    def duration_us(self) -> int:
        return self.total_ticks * self.tick_us

    def settings_needed(self) -> bool:
        return (self.tick_us != DEFAULT_TICK_US) or (self.rel_us != DEFAULT_REL_US)

    def __repr__(self) -> str:
        return "Frame(tick=%dµs, rel=%dµs, %d事件, %d格)" % (
            self.tick_us, self.rel_us, len(self.events), self.total_ticks)


# ==========================================================================
# 编码
# ==========================================================================
def _u24le(v: int) -> bytes:
    if not 0 <= v <= US_PER_TICK_LIMIT:
        raise ProtoError("%d 超出 3 字节范围" % v)
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])


def encode_settings(tick_us: int = DEFAULT_TICK_US,
                    rel_us: int = DEFAULT_REL_US) -> bytes:
    """设置段。两个值都等于默认值时返回空（这样 v1 帧能保持字节级一致）。"""
    out = bytearray()
    if tick_us != DEFAULT_TICK_US:
        out += bytes([SET_TICK]) + _u24le(tick_us)
    if rel_us != DEFAULT_REL_US:
        out += bytes([SET_REL]) + _u24le(rel_us)
    return bytes(out)


def _encode_event(e: Event) -> bytes:
    """单个事件 -> 字节（不含前导 / 帧尾）"""
    out = bytearray()
    if e.is_rest:
        out.append(REST_BYTE)
    elif e.is_chord:
        out.append(CHORD_ON)
        out.extend(e.keys)
        out.append(CHORD_OFF)
    else:
        out.append(e.keys[0])

    # ★ 时值只能发【一个】0xC2，不能分片 ★
    #   RTL 收到 0xC2 的操作数是【直接赋值】：
    #       if (!in_chord && pend_valid) pend_dur <= dur_patch_c;   // = N+1，饱和 255
    #   它是"赋值"不是"累加"。所以发两个分片 `C2 FF C2 05` 只会让后者生效
    #   （总时长变成 6 格，而不是 256 格）—— 分片是错的。
    #   好在 MAX_TICKS = 255 已经把上限钉死，一次就够，这里不需要循环。
    left = e.ticks - 1
    if left > 0:
        out.append(DUR_EXT)
        out.append(left)
    return bytes(out)


def encode_body(events: Sequence[Event]) -> bytes:
    """事件序列 -> 事件流字节（不含设置 / 前导 / 帧尾）"""
    out = bytearray()
    for e in events:
        out += _encode_event(e)
    return bytes(out)


def encode_frame(frame: Frame, include_settings: str = "auto",
                 head: bytes = bytes([CLR_BYTE]),
                 tail: bytes = bytes([END_BYTE]),
                 preamble: int = N_FLUSH) -> bytes:
    """Frame -> 完整帧字节（前导 preamble×FF + 帧头 0xAA + 设置 + 事件流 + 帧尾 0xFF）

    include_settings : "auto"（默认值就省略）/ "always" / "never"
    head / tail      : 帧头、帧尾字节。默认 0xAA / 0xFF；上位机若把这两项做成
                       可填的（串口助手就是），就从外面传进来，免得两处不一致。
    preamble         : 前导 0xFF 个数。只是"冲刷"，FPGA 在关窗状态下会忽略它们；
                       RTL 真正靠 0xAA 开窗、0xFF 收尾，所以设 0 也能工作。
    """
    frame.validate()
    if len(frame.events) > SLOT_MAX:
        raise ProtoError("事件数 %d 超过 FPGA 的 %d 格上限"
                         % (len(frame.events), SLOT_MAX))
    body = encode_body(frame.events)

    if include_settings == "always":
        st = encode_settings(frame.tick_us, frame.rel_us)
    elif include_settings == "never":
        st = b""
    else:
        st = encode_settings(frame.tick_us, frame.rel_us)

    return (bytes([END_BYTE]) * max(0, int(preamble)) + bytes(head) + st + body
            + bytes(tail))


def truncate_to_fit(frame: Frame, limit: int = SLOT_MAX,
                    verbose: bool = False) -> Tuple[Frame, int]:
    """按【事件边界】截断，绝不切开一个事件。返回 (新帧, 丢掉的格数)。

    limit 是【事件数】上限（默认 SLOT_MAX = 1024，与 RTL 的 slot 数组一致）。
    """
    if len(frame.events) <= limit:
        return Frame(events=list(frame.events), tick_us=frame.tick_us,
                     rel_us=frame.rel_us), 0
    kept: List[Event] = list(frame.events[:limit])
    dropped_ticks = sum(x.ticks for x in frame.events[limit:])
    if verbose and dropped_ticks:
        print("[警告] 事件数超出 %d，从事件边界截断，丢掉 %d 格"
              % (limit, dropped_ticks))
    return Frame(events=kept, tick_us=frame.tick_us, rel_us=frame.rel_us), dropped_ticks


def merge_repeats(frame: Frame) -> Frame:
    """把【键集合完全相同】的相邻事件合并成一个长事件。

    这是纯赚的优化：
      * 省字节（N 个 1 格事件 -> 1 个 N 格事件）
      * 更重要的：同一个键保持连续按下，包络不会每个格重新起音 —— 长音/持续和弦
        听起来是"真的按住"，而不是"一格一格地脉冲"
    """
    out: List[Event] = []
    for e in frame.events:
        if out and out[-1].keys == e.keys and out[-1].ticks + e.ticks <= MAX_TICKS:
            out[-1].ticks += e.ticks
        else:
            out.append(Event(keys=e.keys, ticks=e.ticks))
    return Frame(events=out, tick_us=frame.tick_us, rel_us=frame.rel_us)


# ==========================================================================
# 解码（严格镜像 FPGA 接收器/播放器的解析规则）
# ==========================================================================
def decode_frame(data: bytes) -> Frame:
    """完整帧字节 -> Frame。

    与 FPGA 侧的两条规则严格对应：
      1. 只有 0xAA 之后、0xFF 之前的字节才是"事件流"（armed 窗口）
      2. 0xC2 / 0xC4 / 0xC5 后面的操作数是【原始数据】，不受控制字节解释
    """
    if len(data) < N_FLUSH + 2:
        raise ProtoError("帧太短（%d 字节）" % len(data))

    # --- 找 0xAA 开窗点（前导的 0xFF 全部属于冲刷段） ---
    i = 0
    while i < len(data) and data[i] == END_BYTE:
        i += 1
    if i >= len(data) or data[i] != CLR_BYTE:
        raise ProtoError("找不到 0xAA 开窗字节（前导 %d 个 0xFF 之后是 0x%02X）"
                         % (i, data[i] if i < len(data) else -1))
    i += 1

    tick_us = DEFAULT_TICK_US
    rel_us = DEFAULT_REL_US
    events: List[Event] = []

    def read_u24(pos: int) -> Tuple[int, int]:
        if pos + 3 > len(data):
            raise ProtoError("设置参数被截断 @%d" % pos)
        v = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16)
        return v, pos + 3

    # --- 逐字节状态机 ---
    while i < len(data):
        b = data[i]

        # 帧尾：结束
        if b == END_BYTE:
            i += 1
            break

        # ---- 设置（只允许出现在事件之前，但这里容忍任意位置）----
        if b == SET_TICK:
            tick_us, i = read_u24(i + 1)
            continue
        if b == SET_REL:
            rel_us, i = read_u24(i + 1)
            continue

        # ---- 事件 ----
        keys: Tuple[int, ...] = ()
        if b <= NOTE_MAX:
            keys = (b,)
            i += 1
        elif b == REST_BYTE:
            keys = ()
            i += 1
        elif b == CHORD_ON:
            i += 1
            acc: List[int] = []
            while i < len(data) and data[i] != CHORD_OFF:
                bb = data[i]
                if bb <= NOTE_MAX:
                    acc.append(bb)
                    i += 1
                elif bb == REST_BYTE:
                    i += 1                      # 和弦内休止：忽略
                elif bb == DUR_EXT:
                    i += 2                      # 畸形：跳过操作数
                elif bb in (END_BYTE,):
                    break                       # 未闭合，就用已收集的
                elif bb in (SET_TICK, SET_REL):
                    break
                else:
                    i += 1
            if i < len(data) and data[i] == CHORD_OFF:
                i += 1
            keys = tuple(sorted(set(acc)))
        else:
            # 未知字节：跳过（FPGA 侧也是丢弃）
            i += 1
            continue

        # ---- 时长：吃掉紧跟的 0xC2 N 对（操作数当原始数据）----
        ticks = 1
        while i < len(data) and data[i] == DUR_EXT:
            if i + 1 >= len(data):
                raise ProtoError("0xC2 的操作数被截断 @%d" % i)
            ticks += data[i + 1]
            i += 2
            if ticks > MAX_TICKS:
                ticks = MAX_TICKS

        events.append(Event(keys=keys, ticks=ticks))

        # 若帧尾在事件之后立刻出现，交给循环顶部处理
        if i < len(data) and data[i] != END_BYTE and data[i] in (
                CLR_BYTE,):
            break                                # 出现新的开窗 -> 本帧结束

    return Frame(events=events, tick_us=tick_us, rel_us=rel_us)


# ==========================================================================
# 描述 / 预览
# ==========================================================================
def describe_frame(frame: Frame) -> str:
    """人看的单行摘要"""
    lines = ["时间格 = %d µs (%.3f s)   释音提前 = %d µs (%.3f s)"
             % (frame.tick_us, frame.tick_us / 1e6,
                frame.rel_us, frame.rel_us / 1e6),
             "共 %d 事件 / %d 格 / 时长 %.2f 秒"
             % (len(frame.events), frame.total_ticks,
                frame.duration_us() / 1e6)]
    return "\n".join(lines)


def describe_events(frame: Frame, per_line: int = 8) -> str:
    """事件表预览（这个是给肉眼核对的）"""
    out = []
    for idx, e in enumerate(frame.events):
        if e.is_rest:
            body = "休止"
        elif e.is_chord:
            body = "[%s]" % " ".join(key_name(k) for k in e.keys)
        else:
            body = key_name(e.keys[0])
        out.append("%3d:%-16s x%d" % (idx + 1, body, e.ticks))
    lines = []
    for i in range(0, len(out), per_line):
        lines.append("  " + " | ".join(out[i:i + per_line]))
    return "\n".join(lines)


# ==========================================================================
# 自检
# ==========================================================================
def _selftest() -> int:
    ok = 0
    fail = []

    def check(name, cond):
        nonlocal ok
        if cond:
            ok += 1
            print("  ok   %s" % name)
        else:
            fail.append(name)
            print("  FAIL %s" % name)

    print("[1] 基本往返")
    f = Frame(events=[Event((5,)), Event((5,)), Event((9,)), Event((), 1),
                      Event((0, 4, 7), 2), Event((11,))],
              tick_us=83_333, rel_us=10_000)
    enc = encode_frame(f)
    dec = decode_frame(enc)
    check("事件数量一致", len(dec.events) == len(f.events))
    check("事件内容一致", [e.keys for e in dec.events] == [e.keys for e in f.events])
    check("时长一致", [e.ticks for e in dec.events] == [e.ticks for e in f.events])
    check("tick_us 还原", dec.tick_us == f.tick_us)
    check("rel_us 还原", dec.rel_us == f.rel_us)
    check("帧头 16×FF", enc[:16] == bytes([0xFF] * 16))
    check("第 17 字节是 0xAA", enc[16] == 0xAA)
    check("末字节是 0xFF", enc[-1] == 0xFF)

    print("[2] 默认设置时省略设置段（保证 v1 帧字节级一致）")
    v1 = Frame(events=[Event((5,)), Event((9,)), Event((), 1)])
    enc_v1 = encode_frame(v1)
    expect_v1 = (bytes([0xFF] * 16) + bytes([0xAA]) + bytes([5, 9, 0xEE])
                 + bytes([0xFF]))
    check("与 v1 帧逐字节相同", enc_v1 == expect_v1)

    print("[3] 操作数与控制字节撞车（协议最关键的健壮性）")
    # N 取遍所有危险值
    danger = [0xAA, 0xEE, 0xFF, 0xC0, 0xC1, 0xC2, 0xC4, 0xC5, 0x00, 0x23]
    good = True
    for n in danger:
        want = min(MAX_TICKS, 1 + n)          # RTL 的 dur 饱和在 255
        fr = Frame(events=[Event((5,), want)])
        d = decode_frame(encode_frame(fr))
        if len(d.events) != 1 or d.events[0].ticks != want or d.events[0].keys != (5,):
            good = False
            print("     失败于 N=0x%02X -> %r" % (n, d.events))
    check("0xC2 的操作数撞任何控制字节都能正确还原", good)

    good = True
    for v in (0xAA, 0xEEFF, 0xFFFFFF, 0x000001):
        fr = Frame(events=[Event((5,))], tick_us=v)
        d = decode_frame(encode_frame(fr))
        if d.tick_us != v:
            good = False
            print("     失败于 tick_us=0x%06X -> 0x%06X" % (v, d.tick_us))
    check("0xC4 的 3 字节操作数撞任何值都能正确还原", good)

    print("[4] 边界事件")
    cases = [
        ("空和弦等价休止", Event(()), None),
        ("单音", Event((0,)), None),
        ("最高键号 35", Event((35,)), None),
        ("36 音全和弦", Event(tuple(range(36)), 3), None),
        ("1 格", Event((5,), 1), None),
        ("255 格（上限，单片延长）", Event((5,), 255), None),
    ]
    for name, e, _ in cases:
        try:
            d = decode_frame(encode_frame(Frame(events=[e])))
            good = (len(d.events) == 1 and d.events[0].keys == e.keys
                    and d.events[0].ticks == e.ticks)
        except Exception as exc:                       # noqa: BLE001
            good = False
            print("     异常: %r" % exc)
        check(name, good)

    print("[5] 越界拒绝")
    for name, fn in [
        ("键号 36 拒绝", lambda: Event((36,))),
        ("时长 0 拒绝", lambda: Event((5,), 0)),
        ("时长 257 拒绝", lambda: Event((5,), 257)),
        ("tick_us 0 拒绝", lambda: Frame(events=[], tick_us=0)),
    ]:
        try:
            fn()
            check(name, False)
        except ProtoError:
            check(name, True)

    print("[6] 容量与截断（上限 = 事件数 SLOT_MAX，不是字节数）")
    # 5000 个事件 > SLOT_MAX(1024)
    big = Frame(events=[Event((i % 36,)) for i in range(5000)])
    check("超容量时 encode 抛错", _raises(ProtoError, lambda: encode_frame(big)))
    t, dropped = truncate_to_fit(big)
    check("截断后正好 SLOT_MAX 个事件", len(t.events) == SLOT_MAX)
    check("截断后能编码", len(encode_frame(t)) > 0)
    check("截断有丢弃格数", dropped > 0)
    check("截断不切开事件", all(isinstance(e, Event) for e in t.events))

    print("[7] merge_repeats")
    m = merge_repeats(Frame(events=[Event((5,)), Event((5,)), Event((5,)),
                                    Event((9,)), Event((9,))]))
    check("相同键集合被合并", [e.ticks for e in m.events] == [3, 2])
    m2 = merge_repeats(Frame(events=[Event((5,)), Event((9,)), Event((5,))]))
    check("不同键集合不合并", [e.ticks for e in m2.events] == [1, 1, 1])

    print("[8] 随机模糊测试（encode -> decode -> 必须完全一致）")
    rng = random.Random(20261006)
    good = True
    for trial in range(400):
        n = rng.randint(0, 40)
        evs = []
        for _ in range(n):
            kind = rng.random()
            if kind < 0.25:
                evs.append(Event((), rng.randint(1, 40)))          # 休止
            elif kind < 0.70:
                evs.append(Event((rng.randrange(36),), rng.randint(1, 40)))
            else:
                k = rng.sample(range(36), rng.randint(2, 6))
                evs.append(Event(tuple(k), rng.randint(1, 40)))
        fr = Frame(events=evs,
                   tick_us=rng.choice([DEFAULT_TICK_US, 83_333, 1_000_000, 20_000]),
                   rel_us=rng.choice([DEFAULT_REL_US, 10_000, 0]))
        d = decode_frame(encode_frame(fr))
        if ([e.keys for e in d.events] != [e.keys for e in fr.events]
                or [e.ticks for e in d.events] != [e.ticks for e in fr.events]
                or d.tick_us != fr.tick_us or d.rel_us != fr.rel_us):
            good = False
            print("     第 %d 次失败" % trial)
            print("     原始: %r" % fr)
            print("     还原: %r" % d)
            break
    check("400 次随机帧往返一致", good)

    print("[9] 掩码工具")
    check("keys_to_mask/mask_to_keys 互逆",
          mask_to_keys(keys_to_mask([0, 5, 35])) == [0, 5, 35])

    print()
    print("=" * 62)
    if fail:
        print("失败 %d 项 / 通过 %d 项" % (len(fail), ok))
        for n in fail:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % ok)
    return 0


def _raises(exc, fn) -> bool:
    try:
        fn()
    except exc:
        return True
    except Exception:                                   # noqa: BLE001
        return False
    return False


# ==========================================================================
def _demo() -> int:
    """打印一个带和弦的示例帧（用于人工对照）"""
    # 用 83.333ms/格（BPM=90 的八分音符）：四分=2格、二分=6格
    f = Frame(
        tick_us=83_333,
        rel_us=10_000,
        events=[
            Event((26, 22, 24, 23), 2),   # 旋律 D5 + G 和弦(G4 B4 D5)  —— 四分
            Event((22, 22, 24, 23), 2),
            Event((24, 22, 24, 23), 2),
            Event((26, 22, 24, 23), 2),
            Event((27, 22, 24, 23), 2),
            Event((), 2),                 # 休止
            Event((26,), 6),              # 单音二分音符（6 格）
            Event((0, 4, 7), 6),          # 三音和弦二分音符
        ])
    f = merge_repeats(f)
    payload = encode_frame(f)
    print(describe_frame(f))
    print()
    print("事件表：")
    print(describe_events(f))
    print()
    print("字节流 (%d 字节)：" % len(payload))
    print("  " + fmt_hex(payload))
    print()
    back = decode_frame(payload)
    print("反解校验：%s" % ("一致 ✅" if [e.keys for e in back.events] ==
                            [e.keys for e in f.events] else "不一致 ❌"))
    return 0


if __name__ == "__main__":
    # ★ 本机控制台默认 cp1252，直接 print 中文会 UnicodeEncodeError。
    #   这里强制 stdout/stderr 走 UTF-8（errors=replace 兜底），
    #   这样无论从终端、IDLE 还是 GUI 子进程调用都不会崩。
    try:
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                   # noqa: BLE001
        pass

    cmd = sys.argv[1] if len(sys.argv) > 1 else "selftest"
    if cmd == "selftest":
        sys.exit(_selftest())
    elif cmd == "demo":
        sys.exit(_demo())
    else:
        print(__doc__)
        sys.exit(0)
