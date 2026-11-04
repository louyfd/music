# -*- coding: utf-8 -*-
"""sim_uart_packet_echo.py —— uart_packet_echo.v（和弦版）的 Python 行为模型

用途：**不上板**就把协议逻辑验一遍。它严格镜像 RTL 的接收分支顺序和播放状态，
所以改协议时先用它跑，能立刻发现"某个字节被吃错了""时值没生效"这类问题。

镜像对象：src\\uart_packet_echo.v（2026-10 和弦版）
    接收：0xAA 开窗 / 0xFF 帧尾 / 0xC4 0xC5 设置 / 0xC2 时值 / 0xC0 0xC1 和弦
          / 0x00~0x23 音 / 0xEE 休止；操作数一律当原始数据
    播放：每事件占 dur 个时间格，gate = 前 dur-1 格全程 + 最后一格留释音提前量

命令行：
    python sim_uart_packet_echo.py selftest
    python sim_uart_packet_echo.py run <谱子.jp.txt>      整条链路：谱子->帧->时间轴
"""

from __future__ import annotations

import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    sys.path.insert(0, r"D:\CODE\SPP_txandrx")
except Exception:                                       # noqa: BLE001
    pass

REST_BYTE = 0xEE
CLR_BYTE = 0xAA
END_BYTE = 0xFF
CHORD_ON = 0xC0
CHORD_OFF = 0xC1
DUR_EXT = 0xC2
SET_TICK = 0xC4
SET_REL = 0xC5
NOTE_MAX = 0x23
SLOT_MAX = 1024                      # 与 RTL 的 SLOT_MAX 一致
N_FLUSH = 16

TICK_DEF_US = 500_000
REL_DEF_US = 300_000
US_PER_CYC = 1.0 / 50e6          # clk_50m_ext 周期
TICK_MIN_US = 20_000
GAP_MIN_US = 10_000

# 键号 -> 音名（键号是【接线顺序】，不是音高顺序；见 score_jp.py 的置换表推导）
KEY_NAMES = ["A#2", "C3", "D#3", "F3", "G3", "A2", "B2", "C#3", "D3", "E3", "F#3", "G#3",
             "A#3", "C4", "D#4", "F4", "G4", "A3", "B3", "C#4", "D4", "E4", "F#4", "G#4",
             "A#4", "C5", "D#5", "F5", "G5", "A4", "B4", "C#5", "D5", "E5", "F#5", "G#5"]


def key_name(k):
    return KEY_NAMES[k] if 0 <= k < 36 else "?"


def mask_desc(m):
    ks = [i for i in range(36) if (m >> i) & 1]
    if not ks:
        return "休止"
    return "[%s]" % " ".join(key_name(k) for k in ks) if len(ks) > 1 \
        else key_name(ks[0])


# ==========================================================================
# 接收器（镜像 RTL 的 always 块）
# ==========================================================================
class Receiver(object):
    """把字节流解析成"每格一条记录"： (mask, dur_in_ticks)

    ★★ 逐拍照抄 RTL，不再"聪明地简化" ★★
    之前的版本写成"遇到 0xC1 就直接落盘"，和 RTL 的【延后一拍落盘】流水线
    不是一回事 —— 于是 RTL 里两个真 bug 它一个都没抓到：
      · rd_next 不绕回 -> 播放器一声不响
      · 0xC0 被当成落盘触发 -> 和弦前面那个事件被写两次
    现在这里严格镜像：pend_* 暂存 + "本拍 flush 写的是上一次挂上的事件"。
    """

    def __init__(self):
        self.slots = []              # 已落盘的事件 [(mask, dur)]
        self.tick_us = TICK_DEF_US
        self.rel_us = REL_DEF_US
        self.armed = False
        self.in_chord = False
        self.acc = 0
        self.cfg_left = 0            # 还要收几个设置参数字节
        self.cfg_which = 0           # 0=SET_TICK 1=SET_REL
        self.cfg_acc = 0
        self.dur_wait = False
        self.pend_mask = 0           # 待落盘事件（时值要等后面的 0xC2 才知道）
        self.pend_dur = 1
        self.pend_valid = False
        self.tx = []                 # 回显字节
        self.frames = []             # 每收到一个 0xFF（开窗状态）就存一帧快照
        self.dropped_outside = 0     # 窗口外丢弃的事件字节数
        self.warnings = []

    # ---- 主入口：喂一个字节（= RTL 的一个 rx_valid 拍）----
    def feed(self, b):
        # ---- 组合逻辑：本拍要不要落盘 / 要不要挂新事件 ----
        in_window = self.armed and self.cfg_left == 0 and not self.dur_wait
        chord_close = in_window and b == CHORD_OFF and self.in_chord
        plain_note = in_window and b <= NOTE_MAX and not self.in_chord
        plain_rest = in_window and b == REST_BYTE and not self.in_chord
        frame_end = in_window and b == END_BYTE
        # ★ 0xC0（chord_open）【不是】落盘触发 —— 见 RTL 里的说明
        flush_now = chord_close or plain_note or plain_rest or frame_end
        set_pend = chord_close or plain_note or plain_rest
        new_p_mask = self.acc if chord_close else ((1 << b) if plain_note else 0)
        flush_ok = flush_now and self.pend_valid and len(self.slots) < SLOT_MAX
        slots_next = len(self.slots) + (1 if flush_ok else 0)

        # ---- 时序逻辑 (0)：统一落盘 ----
        if flush_ok:
            self.slots.append((self.pend_mask, self.pend_dur))
        # ---- 时序逻辑 (0b)：挂起新的待落盘事件 ----
        if set_pend:
            self.pend_mask, self.pend_dur, self.pend_valid = new_p_mask, 1, True

        # ---- 时序逻辑：分支顺序必须和 RTL 的 else-if 链一致 ----
        if self.cfg_left:                                   # (a) 设置参数的操作数
            self.tx.append(b)
            if self.cfg_left == 1:
                val = (b << 16) | (self.cfg_acc & 0xFFFF)
                if self.cfg_which == 0:
                    self.tick_us = val
                else:
                    self.rel_us = val
                self.cfg_left = 0
            else:
                if self.cfg_left == 3:
                    self.cfg_acc = (self.cfg_acc & 0xFF00) | b
                else:
                    self.cfg_acc = (self.cfg_acc & 0x00FF) | (b << 8)
                self.cfg_left -= 1
            return

        if self.dur_wait:                                   # (b) 0xC2 的操作数 N
            self.tx.append(b)
            self.dur_wait = False
            if (not self.in_chord) and self.pend_valid:
                self.pend_dur = min(255, b + 1)             # RTL 饱和到 255
            return

        if b == CLR_BYTE:                                   # (c) 0xAA 开窗
            self.armed = True
            self.slots = []
            self.in_chord = False
            self.acc = 0
            self.dur_wait = False
            self.cfg_left = 0
            self.pend_valid = False
            self.tick_us = TICK_DEF_US
            self.rel_us = REL_DEF_US
            return                                          # 0xAA 不回显

        if b == END_BYTE:                                   # (d) 0xFF 帧尾
            self.tx.append(b)
            if self.armed:
                self.frames.append({
                    "slots": list(self.slots),
                    "tick_us": self.tick_us,
                    "rel_us": self.rel_us,
                    "n": slots_next,                        # = frame_slots
                })
            self.armed = False
            self.in_chord = False
            self.pend_valid = False
            return

        if b in (SET_TICK, SET_REL):                        # (e) 设置命令
            self.tx.append(b)
            self.cfg_left = 3
            self.cfg_which = 1 if b == SET_REL else 0
            return

        if b == DUR_EXT:                                    # (f) 时值延长
            self.tx.append(b)
            self.dur_wait = True
            return

        if b == CHORD_ON:                                   # (g) 和弦帧头 / 帧尾
            self.tx.append(b)
            self.in_chord = True
            self.acc = 0
            return
        if b == CHORD_OFF:
            self.tx.append(b)
            if not self.in_chord:
                return
            self.in_chord = False
            return                                          # 落盘由 (0) 统一做

        if b <= NOTE_MAX:                                   # (h) 音号 / 休止
            self.tx.append(b)
            if self.in_chord:
                self.acc |= (1 << b)
            elif not self.armed:
                self.dropped_outside += 1
            return
        if b == REST_BYTE:
            self.tx.append(b)
            if not self.armed and not self.in_chord:
                self.dropped_outside += 1
            return
        # 其它：丢弃

    def _write(self, mask):
        """兼容旧接口（旧测试用）—— 现在只是"挂一个待落盘事件"。"""
        self.pend_mask, self.pend_dur, self.pend_valid = (mask & ((1 << 36) - 1)), 1, True


def receive(byte_stream):
    rx = Receiver()
    for b in byte_stream:
        rx.feed(b)
    return rx


# ==========================================================================
# 播放器（镜像 RTL 的播放 always 块）
# ==========================================================================
def tick_cycles(tick_us):
    return int(tick_us * 50)


def play(frame):
    """返回 [(起格, mask, 格数, 末格发声格数占比)]，用于核对时间轴。"""
    slots = frame["slots"]
    tick_us, rel_us = frame["tick_us"], frame["rel_us"]
    if not slots:
        return []
    tick_cyc = max(tick_cycles(tick_us), tick_cycles(TICK_MIN_US))
    rel_cyc = min(tick_cycles(rel_us), tick_cyc - tick_cycles(GAP_MIN_US))
    gate_last_ratio = (tick_cyc - rel_cyc) / float(tick_cyc)

    out = []
    t = 0
    for mask, dur in slots:
        d = max(1, dur)
        out.append((t, mask, d, gate_last_ratio))
        t += d
    return out


def simulate_player(frame, legacy_stop=False):
    """逐拍照抄 RTL 的播放状态机，返回【真正会发声的事件序列】 [(格号, mask, 格数)]。

    为什么必须逐拍照抄：板子上"识别得到但一声不响"就是这条状态机的某一个条件
    写错了，而"事件表对不对"和"它到底会不会响"是两件事 —— 事件表全对、播放器
    一拍就停，听起来就是完全静音。

    ★ 关键点（都踩过）：
      · rd_slot 是 11 位，1023 + 1 = 1024 不会自己绕回 0；RTL 里显式写了绕回。
        旧代码用 `rd_next >= frame_slots` 当结束判据 -> 1024 对任何帧都成立 ->
        第一拍就被判成"放完" -> 一个音都不响。legacy_stop=True 就是复现这个。
      · 结束判据必须用"还剩几个事件"（ev_remain），不能用绕回的 rd_next。
    """
    slots = frame["slots"]
    n = frame.get("n", len(slots))
    if n <= 0 or not slots:
        return []

    rd_slot = 1023          # 哨兵：让第一次 rd_next 绕回 0
    ev_remain = n
    ev_left = 0
    loads = []
    guard = 2 * n + 16
    while guard > 0:
        guard -= 1
        if ev_left == 0:
            if legacy_stop:
                rd_next = rd_slot + 1                  # 旧写法：11 位不绕回
                if rd_next >= n:
                    break
            else:
                if ev_remain == 0:
                    break
                rd_next = 0 if rd_slot == 1023 else rd_slot + 1
            if rd_next >= len(slots):
                break
            dur = max(1, slots[rd_next][1])
            loads.append((rd_next, slots[rd_next][0], dur))
            rd_slot = rd_next
            ev_left = dur
            ev_remain = max(0, ev_remain - 1)
        else:
            ev_left = 0        # 这一事件的格数走完（tick 级推进，等价于逐周期数）
    return loads


def to_per_tick(frame):
    """展开成"每一格点亮哪些键"的列表（用于和期望值逐格比对）"""
    tc = tick_cycles(max(frame["tick_us"], TICK_MIN_US))
    rc = min(tick_cycles(frame["rel_us"]), tc - tick_cycles(GAP_MIN_US))
    gl = tc - rc
    rows = []
    for mask, dur in frame["slots"]:
        d = max(1, dur)
        for i in range(d):
            # gate：前 d-1 格全亮；最后一格亮 gl 个周期。这里按"格内是否有声"记 1/0
            on = 1 if (i < d - 1 or gl > 0) else 0
            rows.append((mask if on else 0, on))
    return rows


# ==========================================================================
# 自检
# ==========================================================================
def _frame(events, tick_us=TICK_DEF_US, rel_us=REL_DEF_US):
    """把 (mask, dur) 列表拼成 v2 帧字节（测试用，独立于 fpga_proto）"""
    out = bytearray([0xFF] * N_FLUSH + [CLR_BYTE])
    if tick_us != TICK_DEF_US:
        out += bytes([SET_TICK, tick_us & 0xFF, (tick_us >> 8) & 0xFF,
                      (tick_us >> 16) & 0xFF])
    if rel_us != REL_DEF_US:
        out += bytes([SET_REL, rel_us & 0xFF, (rel_us >> 8) & 0xFF,
                      (rel_us >> 16) & 0xFF])
    for mask, dur in events:
        ks = [i for i in range(36) if (mask >> i) & 1]
        if not ks:
            out.append(REST_BYTE)
        elif len(ks) == 1:
            out.append(ks[0])
        else:
            out += bytes([CHORD_ON] + ks + [CHORD_OFF])
        if dur > 1:
            out += bytes([DUR_EXT, dur - 1])
    out.append(END_BYTE)
    return bytes(out)


def _selftest():
    ok = 0
    fail = []

    def check(name, cond, extra=None):
        nonlocal ok
        if cond:
            ok += 1
            print("  ok   %s" % name)
        else:
            fail.append(name)
            print("  FAIL %s%s" % (name, ("  -> " + repr(extra)) if extra is not None else ""))

    print("[1] v1 老帧（无新字节）行为不变")
    v1 = bytes([0xFF] * 16 + [CLR_BYTE, 5, 5, 9, REST_BYTE, END_BYTE])
    r = receive(v1)
    check("一帧", len(r.frames) == 1)
    f = r.frames[0]
    check("4 格", len(f["slots"]) == 4)
    check("每格 1 个时间格", all(d == 1 for _, d in f["slots"]))
    check("时间格 = 默认 500ms", f["tick_us"] == TICK_DEF_US)
    check("释音提前 = 默认 300ms", f["rel_us"] == REL_DEF_US)
    tl = play(f)
    check("第 1 格是键 5", tl[0][1] == (1 << 5))
    check("第 4 格是休止（05 05 09 EE）", tl[3][1] == 0)
    check("每格发声占比 = 0.2/0.5 = 0.4",
          abs(tl[0][3] - 0.4) < 1e-9)

    print("[2] 和弦：一个时间格点亮多个键")
    fr = _frame([(0b1_0001_0000_0010_0000, 1)])   # 随便三个键
    r2 = receive(fr)
    f2 = r2.frames[0]
    check("帧里只有 1 格", len(f2["slots"]) == 1)
    check("掩码有 3 个位", bin(f2["slots"][0][0]).count("1") == 3)

    c = _frame([((1 << 5) | (1 << 9) | (1 << 12), 1)])
    r3 = receive(c)
    check("C 和弦 05/09/0C 同时点亮",
          r3.frames[0]["slots"][0][0] == ((1 << 5) | (1 << 9) | (1 << 12)))

    print("[3] 时值：0xC2 让一个事件占多个时间格")
    d = _frame([(1 << 5, 1), (1 << 9, 4), (1 << 11, 8)])
    r4 = receive(d)
    f4 = r4.frames[0]
    check("格数分别是 1/4/8", [x[1] for x in f4["slots"]] == [1, 4, 8])
    check("总格数 13", sum(x[1] for x in f4["slots"]) == 13)

    print("[4] 速度：0xC4 改时间格")
    s = _frame([(1 << 5, 1)], tick_us=250_000)
    r5 = receive(s)
    check("时间格 = 250ms", r5.frames[0]["tick_us"] == 250_000)

    print("[5] 发声占比：0xC5 改释音提前量（连奏）")
    a = _frame([(1 << 5, 4)], rel_us=10_000)
    r6 = receive(a)
    check("释音提前 = 10ms", r6.frames[0]["rel_us"] == 10_000)
    tl6 = play(r6.frames[0])
    check("末格发声占比 = 1 - 10ms/500ms = 98%", abs(tl6[0][3] - 0.98) < 1e-9)

    print("[6] 操作数撞控制字节（最容易埋雷的地方）")
    for n in (0xAA, 0xEE, 0xFF, 0xC0, 0xC1, 0xC2, 0x00, 0x23, 0xFE):
        want = min(255, 1 + n)          # RTL 里 dur 饱和在 255
        fr2 = _frame([(1 << 5, 1 + n)])
        rr = receive(fr2)
        got = rr.frames[0]["slots"]
        good = (len(got) == 1 and got[0][1] == want and got[0][0] == (1 << 5))
        if not good:
            print("     失败 N=0x%02X -> %r (期望 %d 格)" % (n, got, want))
        check("N=0x%02X 正确还原为 %d 格" % (n, want), good)

    print("[7] 0xAA 复位设置 + 停播")
    st = _frame([(1 << 5, 1)], tick_us=250_000, rel_us=10_000)
    r7 = receive(st)
    check("帧内设置生效", r7.frames[0]["tick_us"] == 250_000)
    r7.feed(CLR_BYTE)
    check("0xAA 后设置复位成默认",
          r7.tick_us == TICK_DEF_US and r7.rel_us == REL_DEF_US)

    print("[8] 窗口：开窗之前的字节一律丢弃")
    pre = bytes([0xFF] * 16 + [5, 9])                # 还没 0xAA
    r8 = receive(pre)
    check("窗口外的音不进缓冲、也不产生事件",
          len(r8.slots) == 0 and r8.dropped_outside == 2)
    r8b = receive(bytes([0xFF] * 16 + [CLR_BYTE, 5]))
    # ★ 落盘是【延后一拍】的：这一拍只是"挂起待落盘"，要等下一个触发才进数组
    check("开窗后挂起 1 个待落盘事件（还没进数组）",
          r8b.pend_valid and len(r8b.slots) == 0)
    r8b.feed(END_BYTE)
    check("帧尾一到才真正落盘", len(r8b.frames[0]["slots"]) == 1)

    print("[9] 空帧（前导冲刷用的 FF）不开播")
    r9 = receive(bytes([0xFF] * 16 + [END_BYTE]))
    check("没有产生帧", len(r9.frames) == 0)

    print("[10] 回显：除 0xAA 外都回显")
    r10 = receive(c)
    check("回显了 C0 与 C1", CHORD_ON in r10.tx and CHORD_OFF in r10.tx)
    check("0xAA 不回显", CLR_BYTE not in r10.tx)
    check("0xFF 回显", r10.tx[-1] == END_BYTE)

    print("[11] 空和弦 0xC0 0xC1 = 一个休止格")
    r11 = receive(bytes([0xFF] * 16 + [CLR_BYTE, CHORD_ON, CHORD_OFF, END_BYTE]))
    check("是 1 格且掩码为 0",
          len(r11.frames[0]["slots"]) == 1 and r11.frames[0]["slots"][0][0] == 0)

    # ==================================================================
    # [12] 播放状态机 —— ★ 这一节是为了"板子一声不响"那类 bug 专门加的 ★
    #      之前的模型只验"事件表对不对"，验不了"到底会不会响"，
    #      于是 RTL 里 rd_next 不绕回导致完全静音，模型全绿，上板没声。
    # ==================================================================
    print("[12] 播放状态机：事件表全对 != 会响")
    four = _frame([(1 << 5, 1), (1 << 9, 1), (1 << 11, 1), (1 << 12, 1)])
    f12 = receive(four).frames[0]
    loads = simulate_player(f12)
    check("4 个事件全部被装载", [k for k, _, _ in loads] == [0, 1, 2, 3])
    check("装载的掩码逐项一致",
          [m for _, m, _ in loads] == [1 << 5, 1 << 9, 1 << 11, 1 << 12])
    old = simulate_player(f12, legacy_stop=True)
    check("★ 旧代码（rd_next 不绕回）会一个音都不装载 -> 完全静音",
          old == [], old)

    print("[12b] 和弦前面那个事件不能被写两次")
    # 05 / 和弦(09,0C) / 0B / 和弦(04,12)  -> 正好 4 个事件
    raw = bytes([0xFF] * 16 + [CLR_BYTE, 0x05,
                               CHORD_ON, 0x09, 0x0C, CHORD_OFF,
                               0x0B,
                               CHORD_ON, 0x04, 0x12, CHORD_OFF, END_BYTE])
    f13 = receive(raw).frames[0]
    check("4 个事件（0xC0 不是落盘触发）", len(f13["slots"]) == 4, f13["slots"])
    check("顺序 = 单音/和弦/单音/和弦",
          [m for m, _ in f13["slots"]] ==
          [1 << 5, (1 << 9) | (1 << 0x0C), 1 << 0x0B, (1 << 4) | (1 << 0x12)],
          f13["slots"])
    check("frame_slots 和实际事件数一致", f13["n"] == 4, f13["n"])

    print("[12c] 满帧：1024 个事件不绕回、不重放")
    full = _frame([(1 << 5, 1)] * SLOT_MAX)
    f14 = receive(full).frames[0]
    check("存下 1024 个事件", len(f14["slots"]) == SLOT_MAX, len(f14["slots"]))
    l14 = simulate_player(f14)
    check("播放器正好装载 1024 次（不绕回重放）", len(l14) == SLOT_MAX, len(l14))
    check("最后一个事件是 1023 号", l14[-1][0] == SLOT_MAX - 1)

    print("[12d] 多格事件的装载与收尾")
    f15 = receive(_frame([(1 << 5, 4), (1 << 9, 8)])).frames[0]
    l15 = simulate_player(f15)
    check("装载 2 个事件、格数 4/8",
          [(k, d) for k, _, d in l15] == [(0, 4), (1, 8)], l15)

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
def run_score(path):
    """整条链路：谱子 -> 帧 -> 接收模型 -> 播放时间轴"""
    try:
        import score_jp as JP
        import score_import as SI
        import fpga_proto as P
    except ImportError as exc:
        print("[ERR] 需要同目录下的 fpga_proto.py / score_jp.py / score_import.py：%s" % exc)
        return 1

    with open(path, encoding="utf-8") as fh:
        sc = JP.parse(fh.read(), verbose=False)
    arr = SI.arrange(sc)
    payload = arr.payload

    print("帧：%d 字节；时间格 %d µs；释音提前 %d µs"
          % (len(payload), arr.frame.tick_us, arr.frame.rel_us))
    print()

    rx = receive(payload)
    check_tick = rx.frames[0]["tick_us"] == arr.frame.tick_us
    check_rel = rx.frames[0]["rel_us"] == arr.frame.rel_us
    print("接收模型还原：时间格 %s，释音提前 %s"
          % ("一致 ✅" if check_tick else "不一致 ❌",
             "一致 ✅" if check_rel else "不一致 ❌"))

    exp = [(e.mask, e.ticks) for e in arr.frame.events]
    got = rx.frames[0]["slots"]
    same = (len(exp) == len(got)
            and all(a[0] == b[0] and a[1] == b[1] for a, b in zip(exp, got)))
    print("事件序列还原：%s（%d 事件）"
          % ("完全一致 ✅" if same else "不一致 ❌", len(got)))
    if not same:
        for i, (a, b) in enumerate(zip(exp, got)):
            if a != b:
                print("  第 %d 个事件：发出 %s x%d，还原 %s x%d"
                      % (i, mask_desc(a[0]), a[1], mask_desc(b[0]), b[1]))
                break

    print()
    print("播放时间轴前 12 个事件（格号 / 点亮 / 占几格 / 末格发声占比）：")
    for t, mask, d, ratio in play(rx.frames[0])[:12]:
        print("  %5d  %-30s x%-3d  末格 %.0f%%"
              % (t, mask_desc(mask), d, ratio * 100))
    return 0 if (check_tick and check_rel and same) else 1


# ==========================================================================
# trace：把一个【真实的字节流】逐字节过一遍接收模型，说明每个字节的下场
#
#   用途（这就是它的存在理由）：
#     上位机发出去的东西"到底是什么"，光看代码看不出来 —— 把线上那串字节
#     原样喂进来，就能一句话说清"FPGA 到底认了几个字节、为什么一声不响"。
#
#   python sim_uart_packet_echo.py trace <file.mid>     # 会把 .mid 拆成裸 MIDI 消息
#   python sim_uart_packet_echo.py trace <file.bin>     # 或任何裸字节文件
# ==========================================================================
def load_stream(path):
    """返回 (字节流, 说明)。.mid/.midi/.kar 用 mido 拆成裸消息, 模拟上位机直接转发。"""
    low = path.lower()
    if low.endswith((".mid", ".midi", ".kar")):
        import mido
        mid = mido.MidiFile(path)
        out = bytearray()
        n = 0
        for msg in mid:
            if getattr(msg, "is_meta", False):
                continue
            out += bytes(msg.bytes())
            n += 1
        return bytes(out), "mido 拆出的 %d 条 MIDI 消息" % n
    with open(path, "rb") as fh:
        return fh.read(), "裸字节文件"


def classify(b, before):
    """按 RTL 的 else-if 分支顺序, 说清这一个字节会被怎么处理。"""
    armed, cfg_left, dur_wait = before[0], before[2], before[3]
    if cfg_left:
        return "0xC4/0xC5 的操作数（原始数据，收下）"
    if dur_wait:
        return "0xC2 的操作数（原始数据，收下）"
    if b == CLR_BYTE:
        return "★ 开窗 0xAA：从这里开始才收乐谱"
    if b == END_BYTE:
        return "帧尾 0xFF：" + ("开播 ✅" if armed else "★ 未开窗 → 不开播（不存不响）")
    if b in (SET_TICK, SET_REL):
        return "设置命令 0x%02X" % b
    if b == DUR_EXT:
        return "时值命令 0xC2"
    if b == CHORD_ON:
        return "和弦帧头 0xC0"
    if b == CHORD_OFF:
        return "和弦帧尾 0xC1"
    if b <= NOTE_MAX:
        return "键号 0x%02X(%d)：" % (b, b) + ("进缓冲 ✅" if armed else "★ 丢弃（未开窗）")
    if b == REST_BYTE:
        return "休止 0xEE：" + ("进缓冲 ✅" if armed else "★ 丢弃（未开窗）")
    return "★ 丢弃：不属于本协议任何字节（0x%02X）" % b


def trace(path, show=32):
    stream, why = load_stream(path)
    print("=" * 70)
    print("trace %s" % path)
    print("  内容：%s，共 %d 字节" % (why, len(stream)))
    print("=" * 70)

    rx = Receiver()
    rows = []
    for i, b in enumerate(stream):
        before = (rx.armed, rx.in_chord, rx.cfg_left, rx.dur_wait, len(rx.slots),
                  len(rx.frames), len(rx.tx), rx.dropped_outside)
        rx.feed(b)
        rows.append((i, b, before, classify(b, before)))

    print("前 %d 个字节：" % min(show, len(rows)))
    for i, b, before, act in rows[:show]:
        print("  #%-5d %02X   %s" % (i + 1, b, act))

    kept = sum(1 for r in rows if "✅" in r[3])
    print()
    print("---- 结论 ----")
    print("  真正进了缓冲的字节：%d" % kept)
    print("  窗口外被丢弃的事件字节：%d" % rx.dropped_outside)
    print("  模型收到的帧数：%d" % len(rx.frames))
    if not rx.frames:
        print("  ★ 一帧都没有 → 板子一个字都不会放。")
    else:
        f = rx.frames[0]
        print("  第一帧：%d 个事件，时间格 %d µs" % (len(f["slots"]), f["tick_us"]))
        # ★ 再走一遍播放状态机：事件表对 != 会响
        loads = simulate_player(f)
        print("  播放器实际装载：%d 个事件  %s"
              % (len(loads), "✅ 会响" if loads else "★ 一个都不装载 = 完全静音"))
        if loads:
            head = " ".join(mask_desc(m) for _, m, _ in loads[:6])
            print("  前几个音：%s%s" % (head, " …" if len(loads) > 6 else ""))
            total = sum(d for _, _, d in loads)
            print("  总时长：%.1f 秒（%d 格 x %.3f 秒）"
                  % (total * f["tick_us"] / 1e6, total, f["tick_us"] / 1e6))
        else:
            print("  ★ 能收下事件却装载不起来 —— 检查 rd_next 是否绕回、结束判据是否用 ev_remain")
    return 0 if rx.frames and simulate_player(rx.frames[0]) else 1


def main():
    argv = sys.argv[1:]
    if not argv or argv[0] == "selftest":
        return _selftest()
    if argv[0] == "run" and len(argv) > 1:
        return run_score(argv[1])
    if argv[0] == "trace" and len(argv) > 1:
        return trace(argv[1])
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
