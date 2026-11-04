# -*- coding: utf-8 -*-
"""sim_led_guide.py —— 逐拍仿真 src\\led_guide.v（V41：LED 演奏指引）

为什么要写这个
──────────────
`led_guide.v` 的相位机是【新写的】，而且它踩过的那类坑已经在独立工程里
真真切切发生过一次：

  独立工程 `LEDs\\src\\is31fl3236a_drv.v` 因为发起事务的判断少了一个 `!done`，
  多发了一笔事务，而那一笔写的是 `0x00 = 0x00`（SSD=0 软件关断）——
  实机现象就是 **"全部 LED 亮了一瞬间就灭了"**。
  根因：主机的 `done` 脉冲和 `busy<=0` 是【同一拍】生效的，所以 done 那一拍
  busy 已经是 0；而 `step` 要到那一拍结束才 +1。少了 !done 就会拿旧的 step
  再排一笔 go，下一拍 step 已经 +1 → 多发一笔。

所以这个脚本的核心就是把那些**看不出来的错**钉住：
  · 初始化恰好 76 笔（多了少了都是 bug）
  · `go` 与 `done` 从不同拍（就是上面那条不变量）
  · 初始化期间 PWM 全写 0（上电不许先闪一下全亮）
  · 图案没变时【一个字节都不发】
  · 图案变了发 36 笔 PWM + 1 笔 Update，地址/数据逐个对上
  · 手弹优先于乐谱
  · 接线顺序映射 ch2key() 真的被用上了（换成置换表也跟得上）
  · 时钟常数换算（12.5MHz 域 → SCL ≈ 100.8kHz）

分两部分：
  A~H   模型级：把 led_guide 的每个相位复刻出来跑（几十万拍，快）
  I     逐拍真仿真：led_guide 模型 + 真 `i2c_master.v` 模型 + IS31FL3236A 从机
        模型连起来，从【总线上】读回全部事务并逐笔核对
        （主机模型的 quarter 注入成 2，只影响快慢、不影响字节内容；
          真实值 31 由 [H] 单独验算）

跑法：
    cd "<工程根>\\testfiles"
    python sim_led_guide.py
"""

import os
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sim_i2c_master as base          # 复用已验过的 I2cMaster / Is31Slave 模型

_n_pass = 0
_n_fail = 0


def check(desc, cond, detail=None):
    global _n_pass, _n_fail
    if cond:
        _n_pass += 1
        print("  ok   %s" % desc)
    else:
        _n_fail += 1
        print("  FAIL %s%s" % (desc, "" if detail is None else "  -> %r" % (detail,)))


# ==========================================================================
# src/led_guide.v 的模型
# ==========================================================================
PH_PWR, PH_INIT, PH_WAIT, PH_SCAN, PH_UPD = 0, 1, 2, 3, 4
INIT_LAST = 75
NUM_CH = 36

# ---------------------------------------------------------------------------
# RTL 里 ch2key() 的那张表：物理位置 -> 键号（V42 按用户实测改）
#   用户原话：「琴键顺序是白黑白黑白白黑白黑白黑白，
#              对应第一个琴键应该是 05 00 06 01 07 08 02 09 03 0A 04 0B，
#              其它两个同理」
#   前 12 个就是 PERM，后两个八度各 +12 / +24。
# ---------------------------------------------------------------------------
PERM = [5, 0, 6, 1, 7, 8, 2, 9, 3, 10, 4, 11]
PHYS2KEY = [PERM[i % 12] + 12 * (i // 12) for i in range(NUM_CH)]


def reg_of_key(k):
    """键号 k 要亮，落在哪一路 LED 上（= 哪个 PWM 寄存器 0x01..0x24）"""
    return 0x01 + PHYS2KEY.index(k)


class LedGuide:
    """逐拍复刻 led_guide.v 的相位机。

    ★ 与 Verilog 一样：所有赋值先算进局部变量、最后统一应用（非阻塞语义）。
    ★ `go` / `decode()` 是本拍的【输出】（基于当前状态）；`step_once()` 推进一拍。
    ★ keymap 用来模拟 ch2key()：keymap[c] = 第 c 路 LED 对应的键号。
      默认恒等映射（就是 RTL 里现在的表）。改成置换表可以验证映射真的生效。
    """

    def __init__(self, duty_both=0xE6, duty_cur=0xBF, duty_hand=0xB3, duty_next=0x4D,
                 pwm_freq=0x01, pwr_up_cyc=100, scan_cyc=50, keymap=None):
        self.duty_both = duty_both
        self.duty_cur = duty_cur
        self.duty_hand = duty_hand
        self.duty_next = duty_next
        self.pwm_freq = pwm_freq
        self.pwr_up_cyc = pwr_up_cyc
        self.scan_cyc = scan_cyc
        self.keymap = keymap if keymap is not None else list(PHYS2KEY)

        self.ph = PH_PWR
        self.step = 0
        self.pwr_cnt = 0
        self.gap_cnt = 0
        self.snap_pat = 0
        self.go = 0

        # 输入（测试里直接改这三个）
        self.score_keys = 0
        self.next_keys = 0
        self.hand_keys = 0

    # ---- ch2key() ----
    def ch2key(self, c):
        c &= 0x3F
        return self.keymap[c] if c < NUM_CH else 0

    # ---- 组合：当前扫描这一路该写什么亮度 ----
    def scan_duty(self):
        """★ V43 四档优先级：90(现在+下一格) > 75(现在) > 30(下一格) > 70(手弹) > 灭

        ★ 手弹故意放最后：谱子要求的键保持自己的档位，
          不然一按对灯反而从 90% 掉到 70%。
        """
        k = self.ch2key(self.step & 0x3F)
        cur = (self.snap_pat >> k) & 1
        nxt = (self.snap_pat >> (36 + k)) & 1
        hnd = (self.snap_pat >> (72 + k)) & 1
        if cur and nxt:
            return self.duty_both
        if cur:
            return self.duty_cur
        if nxt:
            return self.duty_next
        if hnd:
            return self.duty_hand
        return 0x00

    # ---- 组合：(寄存器, 数据) ----
    def decode(self):
        if self.ph == PH_UPD:
            return (0x25, 0x00)
        if self.ph == PH_SCAN:
            return (0x01 + (self.step & 0x3F), self.scan_duty())
        s = self.step
        if s == 0:
            return (0x4F, 0x00)                       # 复位
        if s <= 36:
            return (0x26 + s - 1, 0x01)               # LED Control
        if s <= 72:
            return (0x01 + s - 37, 0x00)              # ★ 初始化时 PWM 全 0（全灭）
        if s == 73:
            return (0x25, 0x00)                       # ★ Update
        if s == 74:
            return (0x4B, self.pwm_freq)              # 输出频率
        return (0x00, 0x01)                           # ★ SSD=1（越界兜底也是 1）

    def live_pat(self):
        m = (1 << 36) - 1
        return (((self.hand_keys & m) << 72)
                | ((self.next_keys & m) << 36)
                | (self.score_keys & m))

    # ---- 一个 clk 上升沿 ----
    def step_once(self, busy, done, nack):
        n_go = 0
        ph, step = self.ph, self.step
        pwr, gap, snap = self.pwr_cnt, self.gap_cnt, self.snap_pat
        live = self.live_pat()
        pat_changed = (live != snap)

        # ★★★ 就是这一个条件决定了"会不会多发一笔事务" ★★★
        #   done 那一拍 busy 已经是 0、而 step 还没 +1。
        #   RTL 里写的是 (!busy && !go && !done)，这里必须一致。
        issue_ok = (not busy) and (not self.go) and (not done)

        if ph == PH_PWR:
            if pwr >= self.pwr_up_cyc:
                ph = PH_INIT
            else:
                pwr += 1
        elif ph == PH_INIT:
            if issue_ok:
                if step <= INIT_LAST:
                    n_go = 1
                else:
                    ph = PH_WAIT
                    gap = 0
            if done:
                step += 1
        elif ph == PH_WAIT:
            if pat_changed and gap >= self.scan_cyc:
                gap = 0
                snap = live
                step = 0
                ph = PH_SCAN
            elif gap < self.scan_cyc:
                gap += 1
        elif ph == PH_SCAN:
            if issue_ok:
                if step <= 35:
                    n_go = 1
                else:
                    ph = PH_UPD
            if done:
                step += 1
        elif ph == PH_UPD:
            if issue_ok:
                n_go = 1
            if done:
                ph = PH_WAIT
                gap = 0

        self.ph, self.step = ph, step
        self.pwr_cnt, self.gap_cnt, self.snap_pat = pwr, gap, snap
        self.go = n_go


class MasterStub:
    """只复刻 i2c_master 的【接口】：go 脉冲 → busy txn_cyc 拍 → busy=0 且 done 一拍。

    ★ done 那一拍 busy 已经是 0 —— 与真实主机一致，这一点正是上面那个坑的成因。
    """

    def __init__(self, txn_cyc=20):
        self.txn_cyc = txn_cyc
        self.busy = 0
        self.done = 0
        self.nack = 0
        self.left = 0
        self.issued = []

    def step(self, go, reg_a, dat):
        n_busy, n_done = self.busy, 0
        if self.busy:
            self.left -= 1
            if self.left == 0:
                n_busy = 0
                n_done = 1
        elif go:
            n_busy = 1
            self.left = self.txn_cyc
            self.issued.append((reg_a, dat))
        self.busy, self.done = n_busy, n_done


def expected_init(pwm_freq=0x01):
    """初始化 76 笔（顺序与独立工程/STM32 版一字不差，只有 PWM 的数据不同）"""
    seq = [(0x4F, 0x00)]
    seq += [(0x26 + i, 0x01) for i in range(36)]
    seq += [(0x01 + i, 0x00) for i in range(36)]     # ★ 全灭
    seq += [(0x25, 0x00), (0x4B, pwm_freq), (0x00, 0x01)]
    return seq


def run_stub(g, cycles, log=False):
    """用抽象主机跑 cycles 拍。返回 (stub, 每拍日志)"""
    mst = MasterStub()
    rec = []
    for c in range(cycles):
        go = g.go
        r, d = g.decode()
        if log:
            rec.append((c, g.ph, go, r, d, g.snap_pat, mst.busy, mst.done))
        mst.step(go, r, d)
        g.step_once(mst.busy, mst.done, mst.nack)
    return mst, rec


# ==========================================================================
def main():
    # ---------------------------------------------------------------------
    print("[A] 初始化序列：76 笔，逐笔对照（地址 + 数据 + 顺序）")
    g = LedGuide(pwr_up_cyc=5)
    mst, _ = run_stub(g, 20000)
    exp = expected_init()
    check("★ 恰好发出 76 笔初始化事务（多了少了都是 bug）", len(mst.issued) == 76,
          len(mst.issued))
    check("★ 76 笔逐笔完全一致", mst.issued[:76] == exp,
          [i for i in range(min(len(mst.issued), 76)) if mst.issued[i] != exp[i]][:6])
    check("第一笔是 0x4F 复位", mst.issued[0] == (0x4F, 0x00), mst.issued[0])
    check("最后一笔是 0x00 = 0x01（SSD=1 退出关断，不写它灯永远不亮）",
          mst.issued[75] == (0x00, 0x01), mst.issued[75])
    check("★ 初始化里没有任何一笔写 SSD=0（写了就是「亮一下又灭」）",
          not any(a == 0x00 and v == 0x00 for (a, v) in mst.issued))
    lc = [a for (a, _v) in mst.issued if 0x26 <= a <= 0x49]
    pw = [a for (a, _v) in mst.issued if 0x01 <= a <= 0x24]
    check("★ LED Control 恰好覆盖 0x26..0x49（36 路，不重不漏）",
          sorted(lc) == list(range(0x26, 0x4A)), (len(lc), len(set(lc))))
    check("★ PWM 恰好覆盖 0x01..0x24（36 路，不重不漏）",
          sorted(pw) == list(range(0x01, 0x25)), (len(pw), len(set(pw))))

    # ---------------------------------------------------------------------
    print("[B] 初始化写得是 0x00（全灭）—— 上电不许先闪一下全亮")
    check("★ 初始化里 36 笔 PWM 的数据全是 0x00",
          all(v == 0x00 for (a, v) in mst.issued if 0x01 <= a <= 0x24))
    check("初始化里没有一笔写任何 LED 亮度（没有 DUTY 值混进来）",
          not any(v in (0xE6, 0xBF, 0xB3, 0x4D) for (_a, v) in mst.issued))

    # ---------------------------------------------------------------------
    print("[C] 初始化完之后：图案没变 → 一个字节都不发")
    before = len(mst.issued)
    mst2, _ = run_stub(g, 5000)
    check("★ 静止 5000 拍，总线上一笔事务都没有", len(mst2.issued) == 0, len(mst2.issued))
    check("（初始化后 ph 停在 PH_WAIT）", g.ph == PH_WAIT, g.ph)

    # ---------------------------------------------------------------------
    print("[D] 图案变了 → 36 笔 PWM（0x01..0x24）+ 1 笔 Update")
    g.score_keys = (1 << 3) | (1 << 10)          # 乐谱要点亮键 3 和键 10
    g.hand_keys = 0
    mst3, _ = run_stub(g, 20000)
    check("★ 恰好 37 笔（36 路 PWM + 1 笔 Update）", len(mst3.issued) == 37,
          len(mst3.issued))
    check("★ 前 36 笔地址是 0x01..0x24 且按顺序",
          [a for (a, _v) in mst3.issued[:36]] == list(range(0x01, 0x25)))
    check("★ 第 37 笔是 0x25 = 0x00（Update：影子寄存器必须写它才生效）",
          mst3.issued[36] == (0x25, 0x00), mst3.issued[36])
    duty = {a: v for (a, v) in mst3.issued[:36]}
    r3, r10 = reg_of_key(3), reg_of_key(10)
    check("★ 键 3 按【物理顺序】落在寄存器 0x%02X，档位 = 75%%（只有现在要弹）" % r3,
          duty.get(r3) == 0xBF, hex(duty.get(r3, -1)))
    check("★ 键 10 同上（寄存器 0x%02X）" % r10,
          duty.get(r10) == 0xBF, hex(duty.get(r10, -1)))
    on = sorted(a for (a, v) in mst3.issued[:36] if v != 0)
    check("★ 只有那两路是亮的，其余 34 路写 0",
          on == sorted([r3, r10]), [hex(x) for x in on])

    # ---------------------------------------------------------------------
    print("[E] ★★ V43 四档亮度 90/75/30/70 各就各位，优先级正确")
    #   一次把四种情况摆齐：
    #     键 3  = 现在要弹 + 下一格也要弹 + 手也按着 -> 90%（谱子优先于手弹）
    #     键 10 = 只有现在要弹                        -> 75%
    #     键 20 = 只有下一格要弹                      -> 30%（提前预告）
    #     键 30 = 只有手弹按下                        -> 70%（不播放时按琴键点亮）
    g.score_keys = (1 << 3) | (1 << 10)
    g.next_keys = (1 << 3) | (1 << 20)
    g.hand_keys = (1 << 3) | (1 << 30)
    mst4, _ = run_stub(g, 20000)
    duty = {a: v for (a, v) in mst4.issued[:36]}
    check("★ 键 3（现在+下一格，且手也按着）→ 90% = 0xE6（手弹不覆盖谱子档位）",
          duty.get(reg_of_key(3)) == 0xE6, hex(duty.get(reg_of_key(3), -1)))
    check("★ 键 10（只有现在）→ 75% = 0xBF",
          duty.get(reg_of_key(10)) == 0xBF, hex(duty.get(reg_of_key(10), -1)))
    check("★ 键 20（只有下一格）→ 30% = 0x4D",
          duty.get(reg_of_key(20)) == 0x4D, hex(duty.get(reg_of_key(20), -1)))
    check("★ 键 30（只有手弹）→ 70% = 0xB3（按琴键点亮这个效果保留了）",
          duty.get(reg_of_key(30)) == 0xB3, hex(duty.get(reg_of_key(30), -1)))
    on = sorted(a for (a, v) in mst4.issued[:36] if v != 0)
    check("★ 恰好这 4 路亮，其余 32 路写 0",
          on == sorted([reg_of_key(3), reg_of_key(10),
                        reg_of_key(20), reg_of_key(30)]),
          [hex(x) for x in on])

    # ---------------------------------------------------------------------
    print("[F] 图案变回原样 → 也不再发（比较基准是「上一轮真发出去的」）")
    g.score_keys = 0
    g.next_keys = 0          # ★ 三个都要清 —— 漏一个图案就回不到全灭
    g.hand_keys = 0
    mst5, _ = run_stub(g, 20000)
    check("★ 一次 37 笔的家底写回去", len(mst5.issued) == 37, len(mst5.issued))
    check("★ 全灭：36 笔 PWM 数据全是 0",
          all(_v == 0 for (_a, _v) in mst5.issued[:36]))
    mst6, _ = run_stub(g, 5000)
    check("★ 之后再静止 5000 拍，一笔都不发", len(mst6.issued) == 0, len(mst6.issued))

    # ---------------------------------------------------------------------
    print("[G] ★ 不变量：go 与 done 从不同拍（独立工程「亮一下又灭」的根因）")
    g7 = LedGuide(pwr_up_cyc=5)
    mst7, rec = run_stub(g7, 20000, log=True)
    both = [e[:3] for e in rec if e[2] and e[7]]
    check("★★ go 与 done 从不同拍（少了这条就会多发一笔把灯关掉）",
          not both, both[:3])
    check("★ 因此初始化恰好 76 笔（不是 77）", len(mst7.issued) == 76, len(mst7.issued))
    check("★ 每一笔只发一次（没有重发/跳过）",
          [a for (a, _v) in mst7.issued] == [a for (a, _v) in expected_init()])

    # ---------------------------------------------------------------------
    print("[H] 电源/时序常数（12.5MHz 域）")
    CLK = 12_500_000
    quarter = CLK // (100_000 * 4)
    check("QUARTER = 12_500_000/(100_000×4) = 31", quarter == 31, quarter)
    scl = CLK / (quarter * 4)
    check("实际 SCL ≈ %.1fkHz（芯片上限 400kHz）" % (scl / 1000.0),
          95_000 <= scl <= 105_000, scl)
    txn_us = 116 * quarter / CLK * 1e6
    check("★ 单笔事务 ≈ %.0fµs（116 个 quarter）" % txn_us,
          270 <= txn_us <= 300, txn_us)
    refresh_ms = 37 * 116 * quarter / CLK * 1e3
    check("★ 一轮刷新（37 笔）≈ %.1fms" % refresh_ms, 9.0 <= refresh_ms <= 12.0,
          refresh_ms)
    init_ms = 76 * 116 * quarter / CLK * 1e3
    check("初始化 76 笔 ≈ %.1fms" % init_ms, 18.0 <= init_ms <= 25.0, init_ms)
    check("QUARTER ≥ 4（每个 quarter 至少 4 拍，占空比和采样点才够稳）",
          quarter >= 4, quarter)

    # ---------------------------------------------------------------------
    print("[I] 逐拍真仿真：led_guide + 真 i2c_master + 从机，从总线上读回全部事务")
    #   主机模型的 quarter 注入成 2（只影响快慢，不影响字节内容）；
    #   真实值 31 已由 [H] 验算。
    g9 = LedGuide(pwr_up_cyc=5, scan_cyc=10)
    mst9 = base.I2cMaster(quarter=2)
    slv9 = base.Is31Slave()
    for _ in range(120000):
        go = g9.go
        r, d = g9.decode()
        if go and mst9.st == base.S_IDLE and not mst9.busy:
            mst9.pending_reg = r
            mst9.pending_dat = d
            mst9.start_txn(0x3C, r, d)
        sda_bus = slv9.step(mst9.scl_r, mst9.sda_out if mst9.sda_oe else 1, mst9.sda_oe)
        mst9.step(sda_bus)
        g9.step_once(mst9.busy, mst9.done, mst9.nack)
        if len(slv9.txns) >= 76 and g9.ph == PH_WAIT:
            break
    wire = [tuple(t) for t in slv9.txns]
    check("★ 总线上读到 76 笔初始化", len(wire) == 76, len(wire))
    check("★ 全部是写事务：(0x78, reg, dat)，地址末位 R/W=0",
          all(w[0] == 0x78 for w in wire) and all((w[0] & 1) == 0 for w in wire))
    check("★ 总线内容与预期逐笔一致",
          wire == [(0x78, a, v) for (a, v) in expected_init()],
          [tuple(hex(x) for x in w) for w in wire[:4]])
    check("没有电平违规（SCL 高时数据没跳变）", not slv9.violations, slv9.violations[:2])
    check("没有 NACK", mst9.nack == 0)

    # 再来一轮刷新，看总线
    g9.score_keys = (1 << 0) | (1 << 35)
    n0 = len(slv9.txns)
    for _ in range(120000):
        go = g9.go
        r, d = g9.decode()
        if go and mst9.st == base.S_IDLE and not mst9.busy:
            mst9.pending_reg = r
            mst9.pending_dat = d
            mst9.start_txn(0x3C, r, d)
        sda_bus = slv9.step(mst9.scl_r, mst9.sda_out if mst9.sda_oe else 1, mst9.sda_oe)
        mst9.step(sda_bus)
        g9.step_once(mst9.busy, mst9.done, mst9.nack)
        if len(slv9.txns) >= n0 + 37:
            break
    rw = [tuple(t)[1:] for t in slv9.txns[n0:n0 + 37]]
    check("★ 刷新轮在总线上是 36 笔 PWM + 1 笔 Update",
          len(rw) == 37 and rw[36] == (0x25, 0x00), rw[36:])
    dmap = dict(rw[:36])
    R0, R35 = reg_of_key(0), reg_of_key(35)
    check("★ 键 0 亮（按物理顺序落在寄存器 0x%02X），档位 75%%" % R0,
          dmap.get(R0) == 0xBF, hex(dmap.get(R0, -1)))
    check("★ 键 35 亮（寄存器 0x%02X）" % R35,
          dmap.get(R35) == 0xBF, hex(dmap.get(R35, -1)))
    check("★ 其余 34 路写 0",
          sorted(a for (a, v) in rw[:36] if v != 0) == sorted([R0, R35]),
          [hex(a) for (a, v) in rw[:36] if v != 0])

    # ---------------------------------------------------------------------
    print("[J] ★ 物理顺序映射 ch2key()：必须是双射，且确实按物理顺序落位")
    check("★ 那张表是 0..35 的一个置换（不重不漏）",
          sorted(PHYS2KEY) == list(range(NUM_CH)), sorted(PHYS2KEY))
    check("★ 前 12 个就等于用户给的 05 00 06 01 07 08 02 09 03 0A 04 0B",
          PHYS2KEY[:12] == PERM, PHYS2KEY[:12])
    check("★ 后两个八度各 +12 / +24（用户说「其它两个同理」）",
          PHYS2KEY[12:24] == [k + 12 for k in PERM]
          and PHYS2KEY[24:36] == [k + 24 for k in PERM], PHYS2KEY[12:])
    # 物理第 1 颗灯 -> 键 5（A）；键 5 要亮就写寄存器 0x01
    gp = LedGuide(pwr_up_cyc=5)
    gp.score_keys = 1 << 5
    mstp, _ = run_stub(gp, 20000)
    on = [a for (a, v) in mstp.issued[76:112] if v != 0]
    check("★ 键 5（物理上第 1 个键）→ 写寄存器 0x01（第 1 颗灯）",
          on == [0x01], [hex(x) for x in on])
    gp2 = LedGuide(pwr_up_cyc=5)
    gp2.score_keys = 1 << 0
    mstq, _ = run_stub(gp2, 20000)
    on2 = [a for (a, v) in mstq.issued[76:112] if v != 0]
    check("★ 键 0（物理上第 2 个键）→ 写寄存器 0x02（第 2 颗灯）",
          on2 == [0x02], [hex(x) for x in on2])
    # 只有手弹（不播放时按琴键）-> 70%
    gp3 = LedGuide(pwr_up_cyc=5)
    gp3.hand_keys = 1 << 0
    mstr, _ = run_stub(gp3, 20000)
    d3 = dict(mstr.issued[76:112])
    check("★ 只有手弹 → 70% = 0xB3，落在物理第 2 颗灯（寄存器 0x02）",
          d3.get(0x02) == 0xB3 and d3.get(0x01) == 0x00,
          (hex(d3.get(0x02, -1)), hex(d3.get(0x01, -1))))
    # 换一张自定义置换表也要跟得上
    perm = [(c + 1) % NUM_CH for c in range(NUM_CH)]
    gp4 = LedGuide(pwr_up_cyc=5, keymap=perm)
    gp4.score_keys = 1 << 0
    msts, _ = run_stub(gp4, 20000)
    on4 = [a for (a, v) in msts.issued[76:112] if v != 0]
    check("★ 换一张置换表也跟得上：键 0 落到寄存器 0x24",
          on4 == [0x24], [hex(x) for x in on4])

    print()
    print("=" * 62)
    if _n_fail:
        print("失败 %d 项 / 通过 %d 项" % (_n_fail, _n_pass))
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % _n_pass)
    return 0


if __name__ == "__main__":
    sys.exit(main())
