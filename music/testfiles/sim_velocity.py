# -*- coding: utf-8 -*-
"""sim_velocity.py —— V47 MIDI 逐音力度的【行为验证】

用途：**不上板**就证明力度的协议语义和混音缩放都对。

镜像对象：
    src\\uart_packet_echo.v  —— 0xC3 的解析 + vel_hold/pend_vel/slot_vel
    src\\top.v               —— scale_tgt = (voice_scale × vel) >> 7

★ 关键设计（也是这个脚本要钉死的东西）：
    · 0xC3 只作用于【紧接着的下一个事件】，用完即清回满幅 127。
    · 力度用【两个寄存器】：vel_hold（等着的）-> pend_vel（挂到事件上的）。
      只有一个寄存器的话，事件字节那一拍会把刚设的力度清掉。
    · 没有 0xC3 时全程 127 = 【完全走原路】，老帧行为逐位不变。

命令行：
    python sim_velocity.py
"""

from __future__ import annotations

VEL_BYTE = 0xC3
VEL_FULL = 127
END_BYTE = 0xFF
REST_BYTE = 0xEE
CLR_BYTE = 0xAA
CHORD_ON = 0xC0
CHORD_OFF = 0xC1
NOTE_MAX = 0x23


# ==========================================================================
# 一、镜像 uart_packet_echo.v 里和力度有关的那条路径
# ==========================================================================

class VelModel(object):
    """只镜像"和力度有关"的那部分接收逻辑，别的字节一律跳过。"""

    def __init__(self):
        self.vel_wait = False
        self.vel_hold = VEL_FULL      # 刚收到 0xC3 的值
        self.pend_vel = VEL_FULL      # 挂到"待落盘事件"上的值
        self.pend_mask = 0
        self.pend_valid = False
        self.slots = []               # [(mask, vel)]，已落盘的事件
        self.armed = False
        self.in_chord = False
        self.acc = 0
        self.dur_wait = False

    def feed(self, b):
        # ---- 组合判据（严格照 RTL 的顺序）----
        in_window = self.armed and not self.dur_wait and not self.vel_wait
        chord_close = in_window and b == CHORD_OFF and self.in_chord
        plain_note = in_window and b <= NOTE_MAX and not self.in_chord
        plain_rest = in_window and b == REST_BYTE and not self.in_chord
        frame_end = in_window and b == END_BYTE
        # ★ 0xC0 不是落盘触发（RTL 里有说明）
        flush_now = chord_close or plain_note or plain_rest or frame_end
        set_pend = chord_close or plain_note or plain_rest
        new_mask = self.acc if chord_close else ((1 << b) if plain_note else 0)

        # ---- 时序逻辑 (0)：统一落盘。★ 写的是【上一次】挂上的事件 ----
        if flush_now and self.pend_valid:
            self.slots.append((self.pend_mask, self.pend_vel))

        # ---- 时序逻辑 (0b)：挂新事件，并【把 vel_hold 搬进 pend_vel】----
        if set_pend:
            self.pend_mask = new_mask
            self.pend_valid = True
            self.pend_vel = self.vel_hold
            self.vel_hold = VEL_FULL          # ★ 只作用一个事件
            self.acc = new_mask if self.in_chord else self.acc

        # ---- 时序逻辑：分支顺序必须和 RTL 的 else-if 链一致 ----
        if self.vel_wait:                     # (b0) 0xC3 的操作数
            self.vel_hold = VEL_FULL if b > VEL_FULL else b
            self.vel_wait = False
            return
        if self.dur_wait:                     # (b) 0xC2 的操作数
            self.dur_wait = False
            return
        # RTL 里还有一堆别的分支（映射/八度/节拍器…），这里只关心下面这些
        if b == CHORD_ON:
            self.in_chord = True
            self.acc = 0
        elif b == CHORD_OFF:
            self.in_chord = False
        elif b == VEL_BYTE:
            self.vel_wait = True
        elif b == CLR_BYTE:
            self.armed = True
        elif b <= NOTE_MAX and self.in_chord:
            self.acc |= (1 << b)
        elif b == END_BYTE:
            self.armed = False


def parse(stream):
    m = VelModel()
    for b in stream:
        m.feed(b)
    return m.slots


# ==========================================================================
# 二、镜像 top.v 的混音缩放
# ==========================================================================

def voice_scale(n):
    """★ 逐字照抄 top.v 里的 voice_scale() 函数。"""
    if n <= 1:
        return 256
    return {2: 192, 3: 154, 4: 128, 5: 115, 6: 102}.get(n, 90)


def scale_tgt(n_voice, vel):
    """★ 逐字照抄 top.v 里 V47 那一段。"""
    vs = voice_scale(n_voice)
    if vel == 127:
        return vs                      # 满幅 -> 旁路，逐位等于 V46
    return (vs * vel) >> 7


# ==========================================================================
# 三、检查
# ==========================================================================

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

    # ------------------------------------------------------------------
    section("1 协议：0xC3 只作用于【下一个】事件")
    st = [CLR_BYTE, VEL_BYTE, 64, 0x00, VEL_BYTE, 100, 0x02, END_BYTE]
    sl = parse(st)
    check("两个事件都落盘了（实际 %d 个）" % len(sl), len(sl) == 2, sl)
    check("第 1 个事件：键0、力度 64（实际 %s）" % (sl[0],), sl[0] == (1 << 0, 64), sl)
    check("第 2 个事件：键2、力度 100（实际 %s）" % (sl[1],), sl[1] == (1 << 2, 100), sl)

    section("2 ★ 力度用完即清：第 3 个事件回到满幅 127")
    st = [CLR_BYTE, VEL_BYTE, 40, 0x00, 0x02, 0x04, END_BYTE]
    sl = parse(st)
    check("3 个事件（实际 %d）" % len(sl), len(sl) == 3, sl)
    check("第 1 个 = 力度 40（实际 %s）" % (sl[0],), sl[0][1] == 40, sl)
    check("第 2 个 = 力度 127（没再发 0xC3）" , sl[1][1] == 127, sl)
    check("第 3 个 = 力度 127", sl[2][1] == 127, sl)

    section("3 ★ 完全没有 0xC3 时：全部满幅 127（= 老行为，逐位不变）")
    st = [CLR_BYTE, 0x00, 0x02, 0x04, END_BYTE]
    sl = parse(st)
    check("3 个事件全是 127（实际 %s）" % ([s[1] for s in sl],),
          all(s[1] == 127 for s in sl), sl)

    section("4 一个和弦 = 一个事件，共用一个力度（逐事件，不是逐音）")
    st = [CLR_BYTE, VEL_BYTE, 90, CHORD_ON, 0x00, 0x02, 0x04, CHORD_OFF, END_BYTE]
    sl = parse(st)
    check("和弦落成 1 个事件（实际 %d）" % len(sl), len(sl) == 1, sl)
    check("掩码 = 键 0/2/4（实际 %s）" % hex(sl[0][0]), sl[0][0] == 0x15, sl)
    check("力度 = 90（实际 %d）" % sl[0][1], sl[0][1] == 90, sl)

    section("5 力度夹紧：>127 夹到满幅，不是截低位")
    for raw, want in ((127, 127), (128, 127), (200, 127), (255, 127), (0, 0), (1, 1)):
        st = [CLR_BYTE, VEL_BYTE, raw, 0x00, 0x02, END_BYTE]
        sl = parse(st)
        check("收到 %3d -> 力度 %3d（期望 %d）" % (raw, sl[0][1], want),
              sl[0][1] == want, sl)

    section("6 0xC3 后面没有事件，力度不会漏给再后面的那个")
    st = [CLR_BYTE, 0x00, VEL_BYTE, 30, 0x02, END_BYTE]
    sl = parse(st)
    check("第 1 个 = 127（0xC3 在它后面才发）", sl[0][1] == 127, sl)
    check("第 2 个 = 30", sl[1][1] == 30, sl)

    # ------------------------------------------------------------------
    section("7 混音缩放：满幅【完全旁路】，逐位等于旧行为")
    for n in (1, 2, 3, 4, 5, 6, 8):
        check("n_voice=%d, vel=127 -> %d（= voice_scale，未乘）" % (n, scale_tgt(n, 127)),
              scale_tgt(n, 127) == voice_scale(n))

    section("8 混音缩放：力度线性下降")
    check("vel=64 ≈ 一半（%d，期望约 128）" % scale_tgt(1, 64),
          abs(scale_tgt(1, 64) - 128) <= 1, scale_tgt(1, 64))
    check("vel=32 ≈ 四分之一（%d，期望约 64）" % scale_tgt(1, 32),
          abs(scale_tgt(1, 32) - 64) <= 1, scale_tgt(1, 32))
    check("vel=0 -> 0（静音）", scale_tgt(1, 0) == 0, scale_tgt(1, 0))

    section("9 混音缩放：力度越大越响（单调）")
    ok = True
    for n in (1, 2, 4, 8):
        prev = -1
        for v in range(0, 128, 8):
            cur = scale_tgt(n, v)
            if cur < prev:
                ok = False
            prev = cur
    check("n=1/2/4/8 上都是单调不减", ok)

    section("10 力度和「声部数避让」是相乘关系，互不干扰")
    check("n=1 满幅 = 256（原行为）", scale_tgt(1, 127) == 256)
    check("n=4 满幅 = 128（原来的声部避让）", scale_tgt(4, 127) == 128)
    check("n=4 + vel=64 -> 64（两者相乘）", scale_tgt(4, 64) == 64, scale_tgt(4, 64))

    section("11 9x7 乘法不会溢出（15 位装得下）")
    check("256 × 127 = %d < 32768" % (256 * 127), 256 * 127 < 32768)

    section("12 ★ 力度最低不等于静音（避免「按了没声」的错觉）")
    check("vel=1 仍有声（%d > 0）" % scale_tgt(1, 1), scale_tgt(1, 1) > 0)
    check("vel=0 才是真静音", scale_tgt(1, 0) == 0)

    # ------------------------------------------------------------------
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
