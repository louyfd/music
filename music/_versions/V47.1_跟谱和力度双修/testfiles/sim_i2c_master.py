# -*- coding: utf-8 -*-
"""sim_i2c_master.py —— 逐拍仿真 src\\i2c_master.v，并用一个"只会回 ACK 的
IS31FL3236A 从机模型"监听总线，验证：

  1. 事务形状对不对（START → 地址+W → ACK → 寄存器 → ACK → 数据 → ACK → STOP）
  2. 字节内容对不对（位序、地址最后一位 R/W=0）
  3. START / STOP 的电平关系对不对（SCL 高时 SDA 才允许跳变）
  4. NACK 能不能被正确识别

为什么必须仿真
──────────────
I2C 的时序错误在代码里几乎看不出来（比如"ACK 采样点比 SCL 拉高早了一拍"），
但上板就是不通 —— 而"不通"在硬件上极难定位：分不清是从机地址错、上拉不够、
还是主机的时序本来就不对。这个脚本把主机的每一拍都复刻出来，
让从机模型替我们回答"我到底收到了什么"。

★ 改了 i2c_master.v 之后务必重跑本脚本。

跑法：
    python testfiles\\sim_i2c_master.py
"""

import sys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

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
# src/i2c_master.v 的逐拍模型
# ==========================================================================
CLK_HZ = 50_000_000
SCL_HZ = 100_000
QUARTER = CLK_HZ // (SCL_HZ * 4)          # = 125

S_IDLE, S_START, S_ADDR, S_ACK1, S_REG, S_ACK2, S_DATA, S_ACK3, S_STOP, S_DONE = range(10)


class I2cMaster:
    """逐拍复刻 i2c_master.v。每个 step() = 一个 clk 上升沿。

    ★ 所有赋值先算进 n{}、最后统一应用 —— Verilog 的非阻塞赋值语义。
    """

    def __init__(self, quarter=QUARTER):
        self.quarter = quarter
        self.st = S_IDLE
        self.q = 0
        self.qcnt = 0
        self.bitn = 0
        self.shreg = 0
        self.sda_oe = 1
        self.sda_out = 1
        self.scl_r = 1
        self.busy = 0
        self.done = 0
        self.nack = 0
        self.queued = None       # 待发起的事务 (dev, reg, dat)
        # 发到 S_REG / S_DATA 阶段时要从这两个里取下一个字节
        # （对应 Verilog 里 shreg <= reg_a / shreg <= dat）
        self.pending_reg = 0
        self.pending_dat = 0

    def start_txn(self, dev7, reg_a, dat):
        self.queued = (dev7, reg_a, dat)

    def step(self, sda_in):
        n = {}
        qend = (self.qcnt >= self.quarter - 1)
        go = 1 if (self.queued is not None and self.st == S_IDLE and not self.busy) else 0

        n["done"] = 0
        n["q"] = (0 if self.q == 3 else self.q + 1) if qend else self.q
        n["qcnt"] = 0 if qend else self.qcnt + 1
        n["st"] = self.st
        n["bitn"] = self.bitn
        n["shreg"] = self.shreg
        n["sda_oe"] = self.sda_oe
        n["sda_out"] = self.sda_out
        n["scl_r"] = self.scl_r
        n["busy"] = self.busy
        n["nack"] = self.nack

        st, q, qe = self.st, self.q, qend

        if st == S_IDLE:
            n["scl_r"] = 1
            n["sda_oe"] = 1
            n["sda_out"] = 1
            n["busy"] = 0
            if go:
                dev7, reg_a, dat = self.queued
                self.queued = None
                n["busy"] = 1
                n["nack"] = 0
                n["shreg"] = (dev7 << 1) & 0xFF       # 地址 + W=0
                n["bitn"] = 0
                n["q"] = 0
                n["qcnt"] = 0
                n["st"] = S_START
        elif st == S_START:
            n["scl_r"] = 1
            if q == 0:
                n["sda_oe"] = 1
                n["sda_out"] = 1
            elif q == 2 and qe:
                n["sda_out"] = 0                       # START：SCL 高时 SDA 下降
            if q == 3 and qe:
                n["scl_r"] = 0
                n["st"] = S_ADDR
        elif st in (S_ADDR, S_REG, S_DATA):
            if q == 0:
                n["scl_r"] = 0
                n["sda_oe"] = 1
                n["sda_out"] = (self.shreg >> 7) & 1   # MSB 先发
            elif q == 2 and qe:
                n["scl_r"] = 1
            elif q == 3 and qe:
                n["scl_r"] = 0
                n["shreg"] = ((self.shreg << 1) & 0xFF)
                if self.bitn == 7:
                    n["bitn"] = 0
                    n["st"] = S_ACK1 if st == S_ADDR else (S_ACK2 if st == S_REG else S_ACK3)
                else:
                    n["bitn"] = self.bitn + 1
        elif st in (S_ACK1, S_ACK2, S_ACK3):
            if q == 0:
                n["scl_r"] = 0
                n["sda_oe"] = 0                        # 释放 SDA
            elif q == 2 and qe:
                n["scl_r"] = 1
            elif q == 3 and qe:
                # ★★ 采样点：SCL 在 q2 末才被置 1，到了 q3 这一拍才真的是高。
                #    第一版把采样放在 q2 那一拍（SCL 还是低）—— 那是错的。
                if sda_in:
                    n["nack"] = 1
                n["scl_r"] = 0
                n["sda_oe"] = 1
                n["sda_out"] = 0
                if st == S_ACK1:
                    n["st"] = S_REG
                    n["shreg"] = self.pending_reg
                elif st == S_ACK2:
                    n["st"] = S_DATA
                    n["shreg"] = self.pending_dat
                else:
                    n["st"] = S_STOP
        elif st == S_STOP:
            # ★ SCL 必须全程保持高，SDA 才允许由低变高（STOP 的定义）。
            #   第一版这里写成"q1 拉高、其余拉低"，于是释放 SDA 时 SCL 是低的 ——
            #   不构成 STOP，从机不会结束事务（就是靠这个仿真抓出来的）。
            n["scl_r"] = 1
            if q == 0:
                n["scl_r"] = 0                     # 先拉低，把 SDA 建立成 0
                n["sda_oe"] = 1
                n["sda_out"] = 0
            elif q == 1:
                n["scl_r"] = 1
            elif q == 2 and qe:
                n["scl_r"] = 1                     # 保持高
                n["sda_oe"] = 0                    # ★ SCL 高时释放 SDA = STOP
            elif q == 3 and qe:
                n["scl_r"] = 1
                n["sda_oe"] = 1
                n["sda_out"] = 1
                n["st"] = S_DONE
        elif st == S_DONE:
            n["scl_r"] = 1
            n["sda_oe"] = 1
            n["sda_out"] = 1
            n["busy"] = 0
            n["done"] = 1
            n["st"] = S_IDLE

        for k, v in n.items():
            setattr(self, k, v)


# ==========================================================================
# 从机模型：观察总线、解事务、回 ACK
# ==========================================================================
class Is31Slave:
    """只会回 ACK 的 IS31FL3236A 模型。

    关心的是"主机到底发来了什么"，所以只做两件事：
      · 在 SCL 上升沿采 SDA 上的数据位
      · 收满 8 位后，在下一个 SCL 低电平期间把 SDA 拉低（ACK）
    """

    def __init__(self, addr7=0x3C, nack_on=b""):
        self.addr7 = addr7
        self.nack_on = set(nack_on)      # 在收到某字节时故意 NACK（用来测错误识别）
        self.prev_scl = 1
        self.prev_sda = 1
        self.bitbuf = []
        self.txn = []                    # 本次事务已收的字节
        self.txns = []                   # 收完的事务
        self.starts = 0
        self.stops = 0
        self.violations = []             # 电平跳变时机违规（SCL 高时数据不该变）
        self.drive = None                # 从机驱动 SDA 的值（None = 释放）
        self._ack_pending = False
        self._bits_this_byte = 0

    def sda_value(self):
        """从机交给总线的值（主机会读到这个）"""
        return self.drive

    def step(self, scl, sda_master, master_driving):
        """scl / sda_master 是主机这一拍输出的值；返回这一拍总线上的 SDA。

        ★ 总线是"线与"：只要有一方拉低，就是低。
        """
        # ---- 组合出总线电平（先算，供下一步采样用）----
        sda_bus = 0 if (self.drive == 0) else (sda_master if master_driving else 1)

        # ---- 检测 START / STOP（SCL 高时 SDA 跳变）----
        if self.prev_scl == 1 and scl == 1:
            if self.prev_sda == 1 and sda_bus == 0:
                self.starts += 1
                self.txn = []
                self.bitbuf = []
                self._bits_this_byte = 0
            elif self.prev_sda == 0 and sda_bus == 1:
                self.stops += 1
                if self.txn:
                    self.txns.append(tuple(self.txn))
                    self.txn = []

        # ---- SCL 上升沿：采一位数据 ----
        if self.prev_scl == 0 and scl == 1:
            if self._ack_pending:
                # 这一位是 ACK 位：主机在读，从机不动
                self._ack_pending = False
            else:
                self.bitbuf.append(1 if sda_bus else 0)
                if len(self.bitbuf) == 8:
                    val = 0
                    for b in self.bitbuf:          # MSB 先到
                        val = (val << 1) | b
                    self.txn.append(val)
                    self.bitbuf = []
                    self._ack_pending = True
                    # ★ 决定回 ACK 还是 NACK（在 SCL 变低后才驱动，
                    #   这里只是把"打算回什么"记下来）
                    self._next_ack = 0 if val not in self.nack_on else 1

        # ---- SCL 低电平期间驱动 ACK ----
        if scl == 0 and self._ack_pending:
            self.drive = self._next_ack          # 0 = ACK（拉低）, 1 = NACK（不拉低）
        elif scl == 0 and not self._ack_pending:
            self.drive = None

        # ---- 违规检查：SCL 高时，SDA 不该由主机改变 ----
        #   ★ START / STOP 本身就是"SCL 高时 SDA 跳变"，必须排除，
        #     否则每笔事务的 START 都会被误报成违规（第一版就报了这个假警）。
        if (self.prev_scl == 1 and scl == 1 and self.prev_sda != sda_bus
                and self.drive is None and master_driving):
            is_start = (self.prev_sda == 1 and sda_bus == 0)
            is_stop = (self.prev_sda == 0 and sda_bus == 1)
            if not (is_start or is_stop):
                self.violations.append(("数据位在 SCL 高时跳变", self.prev_sda, sda_bus))

        self.prev_scl = scl
        self.prev_sda = sda_bus
        return sda_bus


# ==========================================================================
# 跑一次事务
# ==========================================================================
def run_one(dev7, reg_a, dat, slave_nack_on=b"", slave_addr=0x3C, max_steps=200000):
    m = I2cMaster()
    s = Is31Slave(addr7=slave_addr, nack_on=slave_nack_on)
    m.pending_reg = reg_a
    m.pending_dat = dat
    m.start_txn(dev7, reg_a, dat)
    nack_seen = 0
    done = 0
    for _ in range(max_steps):
        sda_bus = s.step(m.scl_r, m.sda_out if m.sda_oe else 1, m.sda_oe)
        m.step(sda_bus)
        if m.nack:
            nack_seen = 1
        if m.done:
            done = 1
            break
    return m, s, done, nack_seen


def main():
    print("[1] 一次标准事务：dev=0x3C reg=0x25 data=0x00（PWM Update，初始化里最关键的一笔）")
    m, s, done, nack_seen = run_one(0x3C, 0x25, 0x00)
    check("事务跑完了（没有卡住）", done)
    check("恰好 1 个 START / 1 个 STOP", (s.starts, s.stops) == (1, 1), (s.starts, s.stops))
    check("收到 3 个字节", len(s.txns) == 1 and len(s.txns[0]) == 3,
          [tuple(hex(x) for x in t) for t in s.txns])
    if s.txns:
        got = s.txns[0]
        check("★ 字节内容 = (0x3C<<1|0, 0x25, 0x00) = (0x78, 0x25, 0x00)",
              got == (0x78, 0x25, 0x00), tuple(hex(x) for x in got))
        check("★ 地址字节最后一位 R/W = 0（本芯片只支持写）", (got[0] & 1) == 0, got[0])
    check("没有 NACK", nack_seen == 0)
    check("没有电平违规（SCL 高时数据没跳变）", not s.violations, s.violations[:2])

    print("[2] 位序：换几个'1/0 分布刁钻'的字节，确认 MSB 先发、没有错位")
    for reg_a, dat in ((0x01, 0xFF), (0x24, 0x80), (0x4F, 0x01), (0x26, 0x00)):
        _, s2, done2, _ = run_one(0x3C, reg_a, dat)
        got = s2.txns[0] if s2.txns else ()
        check("reg=0x%02X data=0x%02X 原样收到" % (reg_a, dat),
              done2 and got == (0x78, reg_a, dat), tuple(hex(x) for x in got))

    print("[3] 从机地址：四个候选都要能正确寻址（AD 引脚四种接法）")
    for a7 in (0x3C, 0x3D, 0x3E, 0x3F):
        _, s3, done3, _ = run_one(a7, 0x25, 0x00)
        got = s3.txns[0] if s3.txns else ()
        check("dev=0x%02X -> 地址字节 0x%02X" % (a7, (a7 << 1) & 0xFF),
              done3 and got and got[0] == ((a7 << 1) & 0xFF), tuple(hex(x) for x in got))

    print("[4] NACK 识别：从机对某个字节拉高不回应，主机必须把 nack 置起来")
    #   让从机对"数据字节"回 NACK —— 主机应该报 nack=1，但事务仍然要走完（发 STOP）
    m4, s4, done4, nack4 = run_one(0x3C, 0x25, 0x5A, slave_nack_on=(0x5A,))
    check("事务仍然跑完（不会卡死在 ACK 相位）", done4)
    check("★ 主机报出了 NACK", nack4 == 1, nack4)
    check("仍然只发 1 个 STOP（NACK 后也要正常收尾）", s4.stops == 1, s4.stops)

    print("[5] 连发多笔：连续事务之间必须各有独立的 START/STOP（模拟 76 笔初始化）")
    #   这里只连发 4 笔，验证"上一笔收尾干净、下一笔能重新起步"
    m5 = I2cMaster()
    s5 = Is31Slave()
    seq = [(0x3C, 0x4F, 0x00), (0x3C, 0x26, 0x01), (0x3C, 0x01, 0x40), (0x3C, 0x25, 0x00)]
    for (d, r, v) in seq:
        m5.pending_reg = r
        m5.pending_dat = v
        m5.start_txn(d, r, v)
        for _ in range(200000):
            sda_bus = s5.step(m5.scl_r, m5.sda_out if m5.sda_oe else 1, m5.sda_oe)
            m5.step(sda_bus)
            if m5.done:
                break
    check("4 笔都收全了", len(s5.txns) == 4, len(s5.txns))
    check("4 个 START / 4 个 STOP", (s5.starts, s5.stops) == (4, 4), (s5.starts, s5.stops))
    check("内容逐笔对上",
          [tuple(t) for t in s5.txns] == [(0x78, r, v) for (_d, r, v) in seq][:len(s5.txns)],
          [tuple(hex(x) for x in t) for t in s5.txns])

    print()
    print("=" * 62)
    if _n_fail:
        print("失败 %d 项 / 通过 %d 项" % (_n_fail, _n_pass))
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % _n_pass)
    return 0


if __name__ == "__main__":
    sys.exit(main())
