# -*- coding: utf-8 -*-
"""sim_disp_mode.py —— V48「LCD 显示模式接上位机 + 双向同步」的行为验证

镜像对象（逐拍照抄，不是"聪明地简化"）：
    src\\uart_packet_echo.v
        · 0xD0 的操作数接收（disp_wait）+ 夹紧规则
        · 0x5F 帧的组帧状态机（dp_*）与"值变了才发"的判据（dp_want）
        · 0xCB（请求回报一次状态）会强制补发一帧 0x5F
        · ★ 五路回传帧的【总线仲裁】：st_start / fx_start / mt_start /
          dp_start / rec_load 的互斥条件，以及 rec_busy
    src\\top.v
        · disp_mode 的写口（clk_50m_ext -> clk_50m，翻转沿 + 数据握手）
        · disp_mode 的回报口（clk_50m -> clk_50m_ext，同样握手）
        · 板载按键那条 5 档循环（一个字没改，这里要证明它还在）

★ 为什么单独一个文件：
  这一段的核心风险【不在】协议字节本身，而在两件工程性的事情上：
    ① 五路帧抢同一根 UART 发送 FIFO —— 两路同拍启动的话，
       后一路的字节会被【悄悄吞掉】（交接文档"总线仲裁"那段的原话）；
    ② 显示模式要跨两次时钟域，而 3 位在相邻档之间会同时翻两位 ——
       直接两级同步会采到【中间态】（1->2 采成 3 或 0），
       而 0x5F 是"变了就发"，发出去的假帧收不回来。
  这两件事用同一个文件钉死，比塞进 sim_uart_packet_echo.py 更清楚。

★ 端到端那一段（第 8 节）会把【本模型吐出的字节】喂给
  D:\\CODE\\SPP_txandrx\\fpga_rec.py 里的【真解析器】——
  也就是说"板子发什么"和"上位机怎么读"在这里真的对了一遍，
  而不是两边各写一份自以为是的期望值。

★ 一条容易写错的语义（本节测试夹具踩过一次）：
  RTL 里的 want 全是【电平】——"和上一次【发出去】的值不一样"。
  不是"变化的那一拍出一个脉冲"。所以立起 want 之后要一直保持，
  直到那一帧真的被服务；只立一拍的话，被高优先级的帧挡一下
  （比如 0x5B 先发）就永远补不上了。本模型按电平实现。

命令行：
    python sim_disp_mode.py            # 跑全部检查
"""

from __future__ import annotations

import os
import sys

try:
    sys.path.insert(0, r"D:\CODE\SPP_txandrx")
except Exception:                                       # noqa: BLE001
    pass

# ---- 协议常量（和 RTL / fpga_proto.py / fpga_rec.py 三处必须一致）----
DISP_SET_BYTE = 0xD0        # 上位机 -> 板子：+1 字节显示模式
DISP_STAT_BYTE = 0x5F       # 板子 -> 上位机：显示模式 + 录音开关
REC_ST_HEAD = 0x5B          # 开关帧（老帧，2 字节）
REC_FX_HEAD = 0x5D          # 效果帧（★ 必须仍然是 4 字节）
METRO_STAT_BYTE = 0x5E      # 节拍器帧（5 字节）
FX_REQ_BYTE = 0xCB          # 请求回报一次状态

DISP_MIN, DISP_MAX = 0, 2
NOTE_MAX = 0x23
CLR_BYTE = 0xAA
END_BYTE = 0xFF

# top.v 里 disp_mode 的 5 档（板子内部）
MODE_SPECTRUM, MODE_WAVE, MODE_LISS0, MODE_LISS1, MODE_LISS2 = 0, 1, 2, 3, 4
MODE_MAX = MODE_LISS2


def report_code(disp_mode):
    """镜像 top.v 的 `dsp_rep = (disp_mode >= 3'd2) ? 2'd2 : disp_mode[1:0]`。

    5 档 -> 协议 3 档：0->0 频谱、1->1 波形、2/3/4->2 李萨如。
    ★ 李萨如那三档在上位机看来本来就是同一档，所以粗粒度是正确的，不是错报。
    """
    return DISP_MAX if disp_mode >= MODE_LISS0 else (disp_mode & 0x3)


def clamp_disp(v):
    """镜像 RTL 的 `(rx_data > 8'd2) ? 2'd2 : rx_data[1:0]`。

    ★ 是【夹紧】不是【取模】：3 夹到 2，不绕回 0。
      和上位机 fpga_proto.clamp_disp 逐字同规则（第 8 节会真调用它比对）。
    """
    return DISP_MAX if v > DISP_MAX else (v & 0x3)


def wave_mode_of(disp_mode):
    """镜像 top.v：`wave_mode <= (disp_mode == 3'd1)`。"""
    return 1 if disp_mode == MODE_WAVE else 0


def liss_on_of(disp_mode):
    """镜像 top.v：`liss_on <= (disp_mode >= 3'd2)`。"""
    return 1 if disp_mode >= MODE_LISS0 else 0


def liss_src_of(disp_mode):
    """镜像 top.v：`liss_src <= (disp_mode >= 3'd2) ? disp_mode[1:0]-2 : 0`。"""
    return ((disp_mode & 0x3) - 2) & 0x3 if disp_mode >= MODE_LISS0 else 0


def next_disp_mode(disp_mode, btn_wave):
    """镜像 top.v 的 5 档循环（★ V48 没有改动它）。"""
    if btn_wave:
        return 0 if disp_mode >= MODE_MAX else disp_mode + 1
    return disp_mode


# ==========================================================================
# 一、uart_packet_echo.v：五路回传帧 + 总线仲裁 + 0x5F 组帧
# ==========================================================================
class BoardTx(object):
    """clk_50m_ext 域里那五路回传帧的状态机 —— 逐拍照抄 uart_packet_echo.v。

    ★ 每一路的 idx 语义和 RTL 完全一样：0 = 空闲，1..N = 正在发第几个字节。
    ★ "启动"和"推进"必须分成两段：RTL 里 `if (idx == 0) ... else if (!txf_full)`，
      启动那一拍只把 idx 置 1，【不推字节】—— 所以一个 3 字节的帧占 4 拍。
      写成"启动即推进"就会少算一拍，仲裁的互斥窗口也就跟着错。
    """

    def __init__(self):
        # ---- 下行（0xD0）----
        self.disp_wait = False
        self.disp_val = 0            # 已夹紧
        self.disp_tgl = 0            # 翻转沿
        # ---- 板子当前值（top.v 回送 + 本域的录音开关）----
        self.disp_mode_in = 0        # 2 位（top.v 保证 0..2；第 12 节会故意塞非法值）
        self.rec_on = 0
        self.btn_rec = 0             # 按键 10 的单拍脉冲
        self.rec_toggle = 0          # 收到 0x5C 的单拍脉冲
        # ---- 0x5B 开关帧 ----
        self.rec_on_d = 0
        self.rec_st_idx = 0
        # ---- 0x5D 效果帧（want = 和【上次发出去】的值比，电平型）----
        self.fx_idx = 0
        self.fx_state = 0
        self.tone_sel = 0
        self.wait_mode = 0
        self.fx_last = 0
        self.tone_last = 0
        self.mode_last = 0
        self.fx_byte_val = 0
        self.tone_val = 0
        self.mode_byte = 0
        self.fx_req = 0              # 收到 0xCB 之后置位，帧真发出去了才清
        # ---- 0x5E 节拍器帧（同样是电平型 want）----
        self.mt_idx = 0
        self.mt_bpm, self.mt_num, self.mt_den, self.mt_beat = 100, 4, 1, 0
        self.mt_bpm_l, self.mt_num_l, self.mt_den_l = 0, 0, 0
        self.mt_bpm_r, self.mt_num_r, self.mt_den_r, self.mt_beat_r = 0, 0, 0, 0
        self.mt_force = 1
        # ---- 0x5F 显示模式帧（★ V48 新增）----
        self.dp_disp_l = 0
        self.dp_rec_l = 0
        self.dp_disp_r = 0
        self.dp_rec_r = 0
        self.dp_idx = 0
        self.dp_force = 1            # 复位后强制发一次
        # ---- 0x5A 事件帧 ----
        self.rec_tx_idx = 0
        self.rec_shift = 0
        self.ev_want_in = False
        self.ev_mask = 0
        self.ev_dt = 0
        # ---- 发送 FIFO + 记录 ----
        self.tx = bytearray()
        self.pushed = []             # [(源, 字节)] —— 用来证明"一个都没丢"
        self.txf_full = False
        # ---- 0xCB 的单拍脉冲（收到那一拍置 1）----
        self.fx_req_set = 0
        # ---- 下行解析用（只做我们关心的那几条，用来钉"操作数不当音符"）----
        self.armed = False
        self.in_chord = False
        self.note_events = []        # 本拍落盘的音符（第 10 节用）

    # ---- 组合：dp_disp_w（RTL 里的防御性夹紧）----
    @property
    def dp_disp_w(self):
        return DISP_MAX if self.disp_mode_in > DISP_MAX else (self.disp_mode_in & 0x3)

    # ---- 组合：各路 want ----
    @property
    def dp_want(self):
        return bool(self.dp_force) or (self.dp_disp_w != self.dp_disp_l) \
            or (self.rec_on != self.dp_rec_l)

    @property
    def st_want(self):
        return self.rec_on != self.rec_on_d

    @property
    def fx_want(self):
        # ★ 三路来源：0xCB 主动请求 / 值变了 / 开始录音时强制报一次
        return bool(self.fx_req) or (self.fx_state != self.fx_last) \
            or (self.tone_sel != self.tone_last) \
            or (self.wait_mode != self.mode_last)

    @property
    def mt_want(self):
        return bool(self.mt_force) or (self.mt_bpm != self.mt_bpm_l) \
            or (self.mt_num != self.mt_num_l) or (self.mt_den != self.mt_den_l)

    # ---- 组合：总线仲裁（每个条件都和 RTL 逐字对应）----
    def _starts(self):
        st_start = (self.st_want and self.rec_tx_idx == 0 and self.fx_idx == 0
                    and self.mt_idx == 0 and self.dp_idx == 0)
        fx_start = (self.fx_want and self.rec_tx_idx == 0 and self.rec_st_idx == 0
                    and not st_start and self.mt_idx == 0 and self.dp_idx == 0)
        mt_start = (self.mt_want and self.mt_idx == 0 and self.rec_tx_idx == 0
                    and self.rec_st_idx == 0 and self.fx_idx == 0
                    and not st_start and not fx_start and self.dp_idx == 0)
        dp_start = (self.dp_want and self.dp_idx == 0 and self.rec_tx_idx == 0
                    and self.rec_st_idx == 0 and self.fx_idx == 0
                    and self.mt_idx == 0 and not st_start and not fx_start
                    and not mt_start)
        rec_load = (self.ev_want_in and self.rec_tx_idx == 0
                    and self.rec_st_idx == 0 and self.fx_idx == 0
                    and not st_start and not fx_start and self.mt_idx == 0
                    and self.dp_idx == 0)
        return st_start, fx_start, mt_start, dp_start, rec_load

    @property
    def busy(self):
        return (self.rec_tx_idx != 0 or self.rec_st_idx != 0 or self.fx_idx != 0
                or self.mt_idx != 0 or self.dp_idx != 0)

    # ---- 一拍 ----
    def step(self, txf_full=None):
        if txf_full is not None:
            self.txf_full = txf_full
        # ---- 录音开关（RTL：always @(posedge clk) if (btn_rec|rec_toggle) rec_on <= ~rec_on）----
        if self.btn_rec or self.rec_toggle:
            self.rec_on ^= 1

        # ---- 所有启动条件都用【本拍开始时】的值算 ----
        st_start, fx_start, mt_start, dp_start, rec_load = self._starts()

        # ---- 0xCB 的请求：置 dp_force / mt_force（RTL 也是本拍置位、下一拍才生效）----
        if self.fx_req_set:
            self.fx_req = 1
            self.dp_force = 1
            self.mt_force = 1
            self.fx_req_set = 0

        # ---- (1) 启动段：只置 idx，不推字节 ----
        if st_start:
            self.rec_on_d = self.rec_on
            self.rec_st_idx = 1
        if fx_start:
            self.fx_last = self.fx_state
            self.tone_last = self.tone_sel
            self.mode_last = self.wait_mode
            self.fx_byte_val = self.fx_state
            self.tone_val = self.tone_sel
            self.mode_byte = self.wait_mode
            self.fx_idx = 1
            self.fx_req = 0          # ★ 请求被服务了才清（RTL：else if (fx_start) fx_req <= 0）
        if mt_start:
            self.mt_bpm_l, self.mt_num_l, self.mt_den_l = \
                self.mt_bpm, self.mt_num, self.mt_den
            self.mt_bpm_r, self.mt_num_r, self.mt_den_r = \
                self.mt_bpm, self.mt_num, self.mt_den
            self.mt_beat_r = self.mt_beat
            self.mt_idx = 1
            self.mt_force = 0
        if dp_start:
            self.dp_disp_l = self.dp_disp_w
            self.dp_rec_l = self.rec_on
            self.dp_disp_r = self.dp_disp_w
            self.dp_rec_r = self.rec_on
            self.dp_idx = 1
            self.dp_force = 0
        if rec_load:
            # ★ RTL 的装载不等 txf_full（数据先搬进 rec_shift 就安全了）
            self.rec_shift = ((self.ev_dt & 0xFFFF) << 48) | (self.ev_mask << 8) \
                             | 0x5A
            self.rec_tx_idx = 1
            self.ev_want_in = False

        # ---- (2) 推进段：本拍最多一路在推（互斥由上面的启动条件保证）----
        pushing = []
        if self.rec_st_idx:
            pushing.append(("5B", self.rec_st_idx))
        if self.fx_idx:
            pushing.append(("5D", self.fx_idx))
        if self.mt_idx:
            pushing.append(("5E", self.mt_idx))
        if self.dp_idx:
            pushing.append(("5F", self.dp_idx))
        if self.rec_tx_idx:
            pushing.append(("5A", self.rec_tx_idx))

        if len(pushing) > 1:
            raise AssertionError("★ 两路帧在同一拍推进！仲裁漏了一个互斥条件：%r"
                                 % (pushing,))

        if pushing and not self.txf_full:
            src, idx = pushing[0]
            b = self._byte_of(src, idx)
            self.tx.append(b)
            self.pushed.append((src, b))
            if src == "5B":
                self.rec_st_idx = 2 if idx == 1 else 0
            elif src == "5D":
                self.fx_idx = 0 if idx >= 4 else idx + 1
            elif src == "5E":
                self.mt_idx = 0 if idx >= 5 else idx + 1
            elif src == "5F":
                self.dp_idx = 0 if idx >= 3 else idx + 1
            else:
                self.rec_shift = (self.rec_shift >> 8) & ((1 << 56) - 1)
                self.rec_tx_idx = 0 if idx >= 8 else idx + 1

    def _byte_of(self, src, idx):
        if src == "5B":
            return REC_ST_HEAD if idx == 1 else (self.rec_on & 1)
        if src == "5D":
            # ★ 0x5D 仍然是 4 字节：[头][效果][音色][跟谱模式]
            return {1: REC_FX_HEAD, 2: self.fx_byte_val,
                    3: self.tone_val, 4: self.mode_byte}[idx]
        if src == "5E":
            return {1: METRO_STAT_BYTE, 2: self.mt_bpm_r, 3: self.mt_num_r,
                    4: self.mt_den_r, 5: self.mt_beat_r}[idx]
        if src == "5F":
            # ★ V48 新帧：[0x5F][DISP_MODE][REC_ON]
            return {1: DISP_STAT_BYTE, 2: self.dp_disp_r, 3: self.dp_rec_r}[idx]
        return (self.rec_shift >> ((idx - 1) * 8)) & 0xFF

    # ---- 下行一个字节（只镜像和显示模式有关的那几步）----
    def rx_feed(self, b):
        """喂一个字节（= RTL 的一个 rx_valid 拍）。

        ★ 分支顺序必须和 RTL 的 else-if 链一致：
            (0) 等操作数的状态们  ->  (b9e) 命令字本身  ->  其它
          也就是 disp_wait 必须【挡在】命令解析前面，否则 0xD0 的操作数
          要是恰好等于 0xD0（有人连发两条），第二条会被当成操作数吃掉。
        ★ op_wait / in_window 也要把 disp_wait 算进去 —— 否则操作数会被
          当成音符落进谱子（交接文档坑 3）。
        """
        op_wait = self.disp_wait
        in_window = self.armed and not op_wait
        plain_note = in_window and (b <= NOTE_MAX) and not self.in_chord
        if plain_note:
            self.note_events.append(b)

        if self.disp_wait:
            self.disp_val = clamp_disp(b)
            self.disp_tgl ^= 1
            self.disp_wait = False
        elif b == DISP_SET_BYTE:
            self.disp_wait = True
        elif b == FX_REQ_BYTE:
            self.fx_req_set = 1
        elif b == CLR_BYTE:
            self.armed = True
            self.in_chord = False
        elif b == END_BYTE:
            self.armed = False
            self.in_chord = False


# ==========================================================================
# 二、top.v：显示模式的两处跨时钟域
# ==========================================================================
class TopCdc(object):
    """镜像 top.v 里 V48 新增的那四小段（写口同步、回报握手）。

    ★ 两个 always 块的赋值必须按【非阻塞】语义提交：所有右值都用
      "本拍开始时的值"算，最后一起落盘。写成"算一个改一个"就会差一拍，
      而这一拍之差正好是同步链的级数 —— 会让"采到没稳定的数据"这种
      问题在仿真里看不见（交接文档坑 18 是同一类错误）。
    """

    def __init__(self):
        # ---- clk_50m 域：写口 ----
        self.disp_mode = 0
        self.dsp_s1 = self.dsp_s2 = self.dsp_s3 = 0
        # ---- clk_50m 域：回报口 ----
        self.dsp_rep_lat = 0
        self.dsp_rep_tgl = 0
        self.dsp_rep_d = 0
        # ---- clk_50m_ext 域：回报口接收 ----
        self.dsp_t_s1 = self.dsp_t_s2 = self.dsp_t_s3 = 0
        self.disp_mode_rep = 0
        # ---- 边沿统计（测试用）----
        self.n_write_we = 0
        self.n_rep_tgl = 0

    def clk_50m_edge(self, disp_tgl, disp_val, btn_wave):
        # ---- 写口同步链（右值全部用本拍开始时的值）----
        we = self.dsp_s2 ^ self.dsp_s3
        new_mode = self.disp_mode
        if we:
            new_mode = disp_val & 0x3          # 0xD0 已在板内夹到 0~2
            self.n_write_we += 1
        elif btn_wave:
            new_mode = next_disp_mode(self.disp_mode, True)
        # ---- 回报口（★ 用的是【本拍开始时】的 disp_mode，不是 new_mode）----
        rep = report_code(self.disp_mode)
        new_rep_lat, new_rep_tgl = self.dsp_rep_lat, self.dsp_rep_tgl
        if rep != self.dsp_rep_d:
            new_rep_lat = rep
            new_rep_tgl = self.dsp_rep_tgl ^ 1
            self.n_rep_tgl += 1
        # ---- 统一落盘 ----
        self.dsp_s1, self.dsp_s2, self.dsp_s3 = disp_tgl, self.dsp_s1, self.dsp_s2
        self.disp_mode = new_mode
        self.dsp_rep_d = rep
        self.dsp_rep_lat = new_rep_lat
        self.dsp_rep_tgl = new_rep_tgl

    def clk_50m_ext_edge(self, board):
        # ---- 三级同步器 + 沿上锁存 ----
        edge = self.dsp_t_s2 ^ self.dsp_t_s3
        new_rep = self.dsp_rep_lat if edge else self.disp_mode_rep
        self.dsp_t_s1, self.dsp_t_s2, self.dsp_t_s3 = \
            self.dsp_rep_tgl, self.dsp_t_s1, self.dsp_t_s2
        self.disp_mode_rep = new_rep
        board.disp_mode_in = new_rep          # 送给 uart_packet_echo


class Chip(object):
    """板子：top.v 的两处 CDC + uart_packet_echo 的 0x5F，两个 50MHz 时钟各走各的。

    ★ 相位：clk_50m 和 clk_50m_ext 都是 PLL 出来的 50MHz，相位差【固定但未知】。
      这里用一个 phase 参数把两种极端相位都跑一遍 —— 同步链级数是 2 还是 3
      取决于相位，行为都必须一样。
    """

    def __init__(self, phase=0):
        self.phase = phase & 1
        self.tick = 0
        self.board = BoardTx()
        self.top = TopCdc()
        self.btn_wave_pending = []      # clk_50m 域的按键（按 tick 排队）
        self.btn_rec_pending = []       # clk_50m_ext 域的按键 10 脉冲
        self.rx_pending = []            # 下行字节队列
        self.trace = []                 # 每拍 (disp_mode, disp_mode_rep)

    def press_btn3(self, n=1):
        """按 TTP 键3（显示画面循环）—— clk_50m 域，单拍。"""
        self.btn_wave_pending.extend([1] * n)

    def press_btn10(self):
        """按 TTP 键10（录音开关）—— 已经同步到 clk_50m_ext 的单拍脉冲。"""
        self.btn_rec_pending.append(1)

    def feed(self, data):
        self.rx_pending.extend(data)

    def run(self, cycles=1):
        for _ in range(cycles):
            m_edge = (self.tick % 2 == 0)
            e_edge = ((self.tick + self.phase) % 2 == 0)
            if m_edge:
                bw = self.btn_wave_pending.pop(0) if self.btn_wave_pending else 0
                self.top.clk_50m_edge(self.board.disp_tgl, self.board.disp_val, bw)
            if e_edge:
                if self.rx_pending:
                    self.board.rx_feed(self.rx_pending.pop(0))
                self.board.btn_rec = self.btn_rec_pending.pop(0) \
                    if self.btn_rec_pending else 0
                self.top.clk_50m_ext_edge(self.board)
                self.board.step()
            self.tick += 1
            self.trace.append((self.top.disp_mode, self.top.disp_mode_rep))

    # ---- 查看产物 ----
    def take_bytes(self):
        b = bytes(self.board.tx)
        self.board.tx = bytearray()
        self.board.pushed = []
        return b

    def take_frames(self):
        """把已发出的字节切成帧（靠帧头识别，且校验长度）—— 测试用。"""
        data = self.take_bytes()
        frames = []
        i = 0
        lens = {0x5A: 8, REC_ST_HEAD: 2, REC_FX_HEAD: 4, METRO_STAT_BYTE: 5,
                DISP_STAT_BYTE: 3}
        while i < len(data):
            h = data[i]
            n = lens.get(h)
            if n is None:
                frames.append(("?", data[i:]))
                break
            frames.append((h, data[i:i + n]))
            i += n
        return frames


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
    section("1 0xD0 的夹紧规则（夹紧不是取模，和上位机逐字同规则）")
    check("0 -> 0（频谱）", clamp_disp(0) == 0)
    check("1 -> 1（波形）", clamp_disp(1) == 1)
    check("2 -> 2（李萨如）", clamp_disp(2) == 2)
    check("3 -> 2（★ 夹紧，不是绕回 0）", clamp_disp(3) == 2)
    check("4 -> 2", clamp_disp(4) == 2)
    check("5 -> 2", clamp_disp(5) == 2)
    check("255 -> 2", clamp_disp(255) == 2)

    section("2 0x5F 帧：布局 + 只在值变化时发")
    for phase in (0, 1):
        chip = Chip(phase=phase)
        chip.run(40)
        fr = chip.take_frames()
        # ★ 上电时板子会主动报两帧：
        #   · 0x5E 节拍器默认值（V45 就有的 mt_force，5 字节）
        #   · 0x5F 显示模式/录音（V48 新增的 dp_force，3 字节）
        check("相位%d：复位后强制报一次 0x5F 00 00" % phase,
              (DISP_STAT_BYTE, b"\x5f\x00\x00") in fr, fr)
        check("相位%d：复位那一批里 0x5F 只出现一次" % phase,
              [h for h, _ in fr].count(DISP_STAT_BYTE) == 1, fr)
        chip.run(200)
        check("相位%d：之后【不再周期上报】" % phase, chip.take_bytes() == b"",
              chip.board.tx)

    # ------------------------------------------------------------------
    section("3 上位机下行 0xD0 -> 板子真的换档（含派生位）")
    for phase in (0, 1):
        for want, mode in ((0, MODE_SPECTRUM), (1, MODE_WAVE), (2, MODE_LISS0)):
            chip = Chip(phase=phase)
            chip.run(40)
            chip.take_bytes()
            chip.feed(bytes([DISP_SET_BYTE, want]))
            chip.run(40)
            got = chip.top.disp_mode
            check("相位%d：0xD0 %02X -> disp_mode=%d" % (phase, want, mode),
                  got == mode, got)
            check("相位%d：wave_mode/liss_on/liss_src 跟着走（want=%d）" % (phase, want),
                  (wave_mode_of(got), liss_on_of(got), liss_src_of(got))
                  == (wave_mode_of(mode), liss_on_of(mode), liss_src_of(mode)),
                  (wave_mode_of(got), liss_on_of(got), liss_src_of(got)))
            fr = chip.take_frames()
            if want == MODE_SPECTRUM:
                # ★ 板子本来就在频谱：值【没变】-> 按协议"只在变化时发"，
                #   这一条不该有帧。上位机那边的确认由连接时自动发的
                #   0xCB 补上（下一节专门验）。
                check("相位%d：0xD0 到【当前档】不发帧（值没变，符合协议）" % phase,
                      fr == [], fr)
            else:
                check("相位%d：板子主动回一帧 0x5F %02X 00" % (phase, want),
                      fr == [(DISP_STAT_BYTE, bytes([DISP_STAT_BYTE, want, 0]))], fr)

    section("4 ★★ 0xCB（上位机一连接就自动发）会把状态强推一次")
    #   这是"上位机一打开就对得上"的关键：电脑版打开串口 600ms 后自动发 0xCB，
    #   板子顺势把显示模式/录音状态报上去，界面那行小字就从"未知"变成真实状态
    #   —— 不用等用户在板子上按一下。
    for phase in (0, 1):
        chip = Chip(phase=phase)
        chip.run(40)
        chip.take_bytes()
        chip.feed(bytes([DISP_SET_BYTE, 0]))      # 设成"当前档"：不会产生帧
        chip.run(40)
        check("相位%d：0xD0 到当前档 -> 确实没有帧" % phase,
              chip.take_frames() == [])
        chip.feed(bytes([FX_REQ_BYTE]))           # 上位机的自动查询
        chip.run(200)
        fr = chip.take_frames()
        check("相位%d：0xCB 之后 0x5F 被强制发一次（0x5D/0x5E 也各一帧）" % phase,
              sorted(h for h, _ in fr)
              == sorted([DISP_STAT_BYTE, REC_FX_HEAD, METRO_STAT_BYTE]), fr)
        check("相位%d：报的就是板子的真实状态 5F 00 00" % phase,
              [d for h, d in fr if h == DISP_STAT_BYTE] == [b"\x5f\x00\x00"],
              [d for h, d in fr if h == DISP_STAT_BYTE])

    section("5 ★ 板载按键 3（TTP）也驱动 0x5F —— 双向同步的核心")
    #   5 档 -> 3 档：0->1、1->2 各发一帧；2->3->4 是李萨如的子档，不发；
    #   4->0 又要发。这正是"粗粒度才一致"的证明。
    chip = Chip(phase=0)
    chip.run(40)
    chip.take_bytes()
    got = []
    for _ in range(5):
        chip.press_btn3()
        chip.run(60)
        got.extend(chip.take_frames())
    check("按 5 下按钮 3：恰好发 3 帧（2->3->4 的子档不发，4->0 要发）",
          len(got) == 3, got)
    check("前两帧 = 波形 / 李萨如",
          got[:2] == [(DISP_STAT_BYTE, b"\x5f\x01\x00"),
                      (DISP_STAT_BYTE, b"\x5f\x02\x00")], got[:2])
    check("第三帧 = 回到频谱（4->0）",
          got[2] == (DISP_STAT_BYTE, b"\x5f\x00\x00"), got[2:])
    check("5 下之后 disp_mode 回到 0（循环没被破坏）", chip.top.disp_mode == 0,
          chip.top.disp_mode)
    check("disp_mode_rep 和 disp_mode 一致（同步完成）",
          chip.top.disp_mode_rep == report_code(chip.top.disp_mode),
          (chip.top.disp_mode, chip.top.disp_mode_rep))

    section("6 ★ 板载按键 10（录音开关）也驱动 0x5F")
    chip = Chip(phase=0)
    chip.run(40)
    chip.take_bytes()
    chip.press_btn10()
    chip.run(80)
    fr = chip.take_frames()
    check("按一下按键 10：0x5B 和 0x5F 各发一帧（上位机两帧都认）",
          sorted(h for h, _ in fr) == sorted([REC_ST_HEAD, DISP_STAT_BYTE]), fr)
    f5f = [d for h, d in fr if h == DISP_STAT_BYTE]
    f5b = [d for h, d in fr if h == REC_ST_HEAD]
    check("0x5F 的 REC_ON = 1", f5f == [b"\x5f\x00\x01"], f5f)
    check("0x5B 的开关 = 1（老帧没被动过）", f5b == [b"\x5b\x01"], f5b)
    check("★ 0x5D 效果帧【一帧都没发】（没被牵连）",
          all(h != REC_FX_HEAD for h, _ in fr), fr)

    section("7 ★★ 总线仲裁：四路同时 want，一个字节都不能丢")
    #   这是本文件最重要的一节。RTL 里那段注释的原话是：
    #   "两路在同一拍同时启动：两边的移位寄存器都往前走了，
    #     但 FIFO 只收得下先写的那一个 —— 另一路的字节被悄悄吞掉"。
    #   这里把四路 want 在同一拍全部立起来，看发出来的字节流
    #   能不能还原成【四个完整的帧】。
    for phase in (0, 1):
        chip = Chip(phase=phase)
        chip.run(40)
        chip.take_bytes()
        b = chip.board
        b.rec_on = 1                # -> st_want 立刻立起来（电平型）
        b.fx_state = 0x0A           # -> 0x5D 的 want
        b.mt_bpm = 120              # -> 0x5E 的 want
        chip.top.disp_mode = 2      # -> 0x5F 的 want（跨时钟握手几拍后才到）
        chip.run(300)
        fr = chip.take_frames()
        heads = [h for h, _ in fr]
        check("相位%d：四路各出一帧、顺序 = 仲裁优先级（5B>5D>5E>5F）" % phase,
              heads == [REC_ST_HEAD, REC_FX_HEAD, METRO_STAT_BYTE, DISP_STAT_BYTE],
              heads)
        check("相位%d：0x5D 仍然是【4 字节】" % phase,
              [d for h, d in fr if h == REC_FX_HEAD]
              == [bytes([REC_FX_HEAD, b.fx_byte_val, b.tone_val, b.mode_byte])],
              [d for h, d in fr if h == REC_FX_HEAD])
        check("相位%d：0x5E 仍然是【5 字节】" % phase,
              [d for h, d in fr if h == METRO_STAT_BYTE]
              == [bytes([METRO_STAT_BYTE, b.mt_bpm_r, b.mt_num_r, b.mt_den_r,
                         b.mt_beat_r])],
              [d for h, d in fr if h == METRO_STAT_BYTE])
        check("相位%d：0x5F 是【3 字节】且内容正确" % phase,
              [d for h, d in fr if h == DISP_STAT_BYTE] == [b"\x5f\x02\x01"],
              [d for h, d in fr if h == DISP_STAT_BYTE])
        check("相位%d：0x5B 是【2 字节】" % phase,
              [len(d) for h, d in fr if h == REC_ST_HEAD] == [2],
              [len(d) for h, d in fr if h == REC_ST_HEAD])
        check("相位%d：一共只有这 4 帧，没有多也没有少" % phase, len(fr) == 4, fr)

    section("8 FIFO 背压（txf_full）：只会拖后，不会丢/重字节")
    for phase in (0, 1):
        chip = Chip(phase=phase)
        chip.run(40)
        chip.take_bytes()
        chip.top.disp_mode = 1
        chip.board.txf_full = True          # ★ 一路压着不放
        chip.run(40)
        check("相位%d：FIFO 满的时候一个字节都不发" % phase,
              chip.take_bytes() == b"")
        chip.board.txf_full = False
        chip.run(60)
        got = chip.take_bytes()
        check("相位%d：松开之后 0x5F 仍然是完整的一帧 5F 01 00" % phase,
              got == b"\x5f\x01\x00", got.hex())
        check("相位%d：字节一个不多一个不少" % phase, len(got) == 3, len(got))

    # ------------------------------------------------------------------
    section("9 端到端：板子模型吐的字节 -> 真的 fpga_rec.RecParser")
    try:
        import fpga_rec as R
        have_rec = True
    except Exception as exc:                             # noqa: BLE001
        have_rec = False
        print("  (skip) 没找到 fpga_rec：%s" % exc)
    if have_rec:
        # ★ 完整重现真机上的连接时序：上位机打开串口 -> 自动发 0xCB
        #   -> 板子回 0x5D + 0x5E + 0x5F（界面立刻从"未知"变成真实状态）
        #   -> 用户按板载键 10 录音（0x5B + 0x5F）-> 用户在软件上点"李萨如"
        chip = Chip(phase=0)
        chip.run(40)
        chip.take_bytes()
        chip.feed(bytes([FX_REQ_BYTE]))          # 上位机连接时的自动查询
        chip.run(200)
        chip.press_btn10()                       # 板载键 10
        chip.run(100)
        chip.feed(bytes([DISP_SET_BYTE, 2]))     # 软件上点"李萨如"
        chip.run(100)
        stream = chip.take_bytes()

        rx = R.RecParser()
        rx.feed(stream)
        check("上位机解析出 disp_mode=2（李萨如）", rx.disp_mode == 2, rx.disp_mode)
        check("上位机解析出 rec_on=True", rx.rec_on is True, rx.rec_on)
        check("0x5F 不算『丢弃字节』", rx.dropped == 0, rx.dropped)
        check("一帧假帧都不是（字节边界全对）", rx.bad_frames == 0, rx.bad_frames)
        check("0x5D 也没被冲掉（fx_state 收到了）", rx.fx_state == 0, rx.fx_state)
        check("0x5E 也没被冲掉（默认 100 BPM / 4/4）",
              (rx.metro_bpm, rx.metro_num, rx.metro_den) == (100, 4, 1),
              (rx.metro_bpm, rx.metro_num, rx.metro_den))

        # 反方向：上位机打包出来的字节，必须正好是板子等的那个
        check("fpga_rec.pack_disp_set(1) == D0 01",
              R.pack_disp_set(1) == b"\xd0\x01", R.pack_disp_set(1).hex())
        check("机器人造帧 == 板子会发的 3 个字节",
              R.pack_disp_frame(2, 1) == b"\x5f\x02\x01",
              R.pack_disp_frame(2, 1).hex())
        # ★ 板子在 3/4 档（李萨如子档）上报 2，上位机必须认，不能当假帧
        rx2 = R.RecParser()
        rx2.feed(b"\x5f\x02\x01")
        check("板子在李萨如子档（3/4）上报 2，上位机认得",
              rx2.disp_mode == 2 and rx2.bad_frames == 0,
              (rx2.disp_mode, rx2.bad_frames))

    # ------------------------------------------------------------------
    section("10 ★ 对照：直接两级同步 3 位会采到【中间态】（所以本版用握手）")
    #   模型：每一位的布线延迟不同，跨时钟被采到时可能"有的位已经翻、有的还没翻"。
    #   1(001) -> 2(010) 这一跳两位同时变，采到的可能是 000(频谱) 或 011(3)。
    #   ★ 0x5F 是"变了就发"，采到 011 的话上位机直接判成假帧、界面一抖；
    #     采到 000 更糟 —— 会真的报一帧"频谱"，上层就把控件切到频谱去了。
    def naive_sync_codes(v_from, v_to):
        """把两位同步器的采样点枚举出来（每一位要么新、要么旧）。"""
        out = set()
        for mask in range(4):
            out.add((v_from & ~mask) | (v_to & mask))
        return sorted(out)

    torn = naive_sync_codes(1, 2)
    check("1 -> 2 直接同步可能出现 0/1/2/3 四种读数", torn == [0, 1, 2, 3], torn)
    check("★ 其中 3 是非法码（上位机当假帧）", 3 in torn)
    check("★ 其中 0 会被【真的当成频谱报出去】", 0 in torn)
    #   握手版本：数据只在【沿的前一拍】稳定变化，接收侧永远锁存"配对"的那份
    seen = set()
    for phase in (0, 1):
        chip = Chip(phase=phase)
        chip.run(40)
        for w in (1, 2, 0, 2, 1, 0):
            chip.feed(bytes([DISP_SET_BYTE, w]))
            chip.run(30)
            seen.add(chip.top.disp_mode_rep)
    check("握手版本：报告码只出现 {0,1,2}，从没出现过中间态",
          seen <= {0, 1, 2}, seen)
    #   ★ 而且每一次【真的换档】都必须被回报到，一次都不能漏。
    #     期望值不写死 —— 直接按"报告码变了几次"推出来，
    #     这样连"粗粒度映射"这件事本身也一起验了。
    chip = Chip(phase=0)
    chip.run(40)
    chip.take_frames()          # ★ 先把上电那一批（0x5E 默认值 + 强制 0x5F 00）排空
    modes = []
    frames = []
    for _ in range(8):
        chip.press_btn3()
        chip.run(60)
        modes.append(chip.top.disp_mode)
        frames.extend(chip.take_frames())
    check("连按 8 下按钮 3：档位 = 1,2,3,4,0,1,2,3（5 档循环）",
          modes == [1, 2, 3, 4, 0, 1, 2, 3], modes)
    want_codes = [report_code(m) for m in modes]
    changed = [c for i, c in enumerate(want_codes)
               if i == 0 or c != want_codes[i - 1]]
    check("★ 帧数 == 报告码变化的次数（不多不少）",
          len(frames) == len(changed), (len(frames), len(changed), frames))
    check("★ 每一帧报的就是那一次的档位，录音位恒 0",
          [d[1] for _, d in frames] == changed
          and all(d[2] == 0 for _, d in frames),
          [d.hex() for _, d in frames])
    check("★ 2->3->4 这些李萨如子档【不发帧】（上位机眼里是同一档）",
          want_codes == [1, 2, 2, 2, 0, 1, 2, 2], want_codes)

    section("11 0xD0 的操作数不会被当成音符/命令（坑 3）")
    b = BoardTx()
    b.rx_feed(CLR_BYTE)          # 开窗
    b.rx_feed(0x05)              # 开窗期内一个正常音符（对照组）
    check("对照组：开窗期内 0x05 确实被当成音符", b.note_events == [5], b.note_events)
    b2 = BoardTx()
    b2.rx_feed(CLR_BYTE)
    b2.rx_feed(DISP_SET_BYTE)
    b2.rx_feed(0x05)             # 这是 0xD0 的操作数（= 音符号 5，最容易被吃错）
    check("★ 0xD0 的操作数 0x05【没有】被当成音符", b2.note_events == [],
          b2.note_events)
    check("★ 它被当成显示模式：夹到 2", b2.disp_val == 2, b2.disp_val)
    check("★ 而且翻转沿翻了（写口真的发出去了）", b2.disp_tgl == 1, b2.disp_tgl)
    b3 = BoardTx()
    b3.rx_feed(DISP_SET_BYTE)    # 没开窗也照样能设
    b3.rx_feed(0x01)
    check("★ 不受开窗限制（没 armed 也能设）", b3.disp_val == 1, b3.disp_val)
    b4 = BoardTx()
    b4.rx_feed(DISP_SET_BYTE)
    b4.rx_feed(DISP_SET_BYTE)    # ★ 连发两条：第二条是【操作数】，不是命令
    check("连发两条 0xD0：第二条当操作数（夹到 2），不会漏掉操作数",
          b4.disp_val == 2 and not b4.disp_wait, (b4.disp_val, b4.disp_wait))

    section("12 ★ 老路径逐位不变")
    chip = Chip(phase=0)
    chip.run(40)
    chip.take_bytes()
    chip.run(500)                     # 完全不碰显示模式、不碰录音
    check("没有 0xD0 / 没有按键 -> 一帧 0x5F 都不发", chip.take_bytes() == b"",
          chip.board.tx)
    chip2 = Chip(phase=0)
    seq = []
    for _ in range(6):
        seq.append(chip2.top.disp_mode)
        chip2.press_btn3()
        chip2.run(30)
    check("按键 3 的 5 档循环 = 0,1,2,3,4,0", seq == [0, 1, 2, 3, 4, 0], seq)
    check("★ 5 档各自的派生位映射没变",
          [(wave_mode_of(m), liss_on_of(m), liss_src_of(m)) for m in range(5)]
          == [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 1, 1), (0, 1, 2)],
          [(wave_mode_of(m), liss_on_of(m), liss_src_of(m)) for m in range(5)])

    section("13 disp_mode_in 的防御性夹紧（top.v 万一送错也不会发非法帧）")
    for bad, want in ((3, 2), (4, 2), (7, 2)):
        b = BoardTx()
        b.disp_mode_in = bad
        check("disp_mode_in=%d -> 帧里上报 %d" % (bad, want), b.dp_disp_w == want,
              b.dp_disp_w)

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
    try:
        sys.stdout.reconfigure(encoding="utf-8")        # py3.7+
    except Exception:                                   # noqa: BLE001
        pass
    raise SystemExit(_selftest())
