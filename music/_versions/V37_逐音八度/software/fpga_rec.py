# -*- coding: utf-8 -*-
"""fpga_rec.py —— 板子演奏录音的「回传解析 + 合成为 MIDI」

板子那边（uart_packet_echo.v）每 10ms 采一次 sounding_mask，一变就通过蓝牙回传：

    [0x5A] [mask 36 位 · 小端 5 字节] [dt 2 字节 · 小端，单位 10ms]
    [0x5B] [0/1]                                    录音开关状态

★ mask 是【真正在响的音】= top.v 的 sounding_mask = (env[键] != 0)，不是"按下的键"。
  两者差一个 ADSR 释音尾巴：松手后包络要先连续低满 8ms 才承认松手，然后每
  280 个采样点降 1，255 级 ≈ 1.46 秒才真正静音（笛子音色 ≈ 0.25 秒）。
  所以录下来的音长【就是耳朵听到的音长】，一个音松手后还会拖着约 1.5 秒。
  它同时也包含"延音键锁存"（sounding_mask_aud = key_mask | sustain_latch_aud）。

dt = 距上一个事件过了多少个 10ms，累加就得到绝对时间。
mask 的第 36 位以上必须为 0 —— 拿它当帧校验，能挡掉乱入的字节。

本模块只做两件事：
    1) RecParser  从串口字节流里把事件抠出来（要能扛住回显字节的干扰）
    2) events_to_midi  把事件合成 MIDI（键号 -> 音高，事件 -> 音符）

★ 合成的 MIDI 可以直接再发回板子（「发送MIDI文件」那条路），形成回环。

命令行：
    python fpga_rec.py selftest
    python fpga_rec.py demo            # 造一段假录音，合成 MIDI 并报路径
"""

from __future__ import annotations

import io
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import score_jp as JP                      # noqa: E402

REC_EV_HEAD = 0x5A          # FPGA -> 上位机：录音事件帧头
REC_ST_HEAD = 0x5B          # FPGA -> 上位机：录音开关状态
REC_CMD = 0x5C              # 上位机 -> FPGA：翻转录音开关
REC_FX_HEAD = 0x5D          # FPGA -> 上位机：效果开关状态（+音色，见下）
EV_FRAME = 8                # 事件帧总长（头 1 + mask 5 + dt 2）
MS_PER_TICK10 = 10          # dt 的单位

# ---- ★ V31：效果开关 / 音色的上位机控制字节 ----
FX_TGL_BYTE = 0xC9          # 上位机 -> FPGA：翻转一个效果开关（+位号 0..7）
TONE_SET_BYTE = 0xCA        # 上位机 -> FPGA：直接选音色（+0 普通 / 1 笛子 / 2 吉他）
FX_REQ_BYTE = 0xCB          # 上位机 -> FPGA：请求立刻回报一次状态
TONE_NAMES = ("普通", "笛子", "吉他")
#   ★ 状态帧 0x5D 的格式：
#       老固件(V29 及以前): [0x5D][效果 8 位]                   —— 2 字节
#       新固件(V31 起)    : [0x5D][效果 8 位][音色 0/1/2]        —— 3 字节
#     解析器按"第 3 个字节 <= 2 就当音色吃掉"来兼容两种固件；
#     因为帧头都是 >= 0x5A，所以 0..2 这些值绝不会被误当成新帧的开头。

# ---- 效果状态 8 位（位序和 uart_packet_echo.v / top.v 里那包完全一致）----
FX_BITS = (
    ("glide",   "滑音"),      # bit0  音高从上一个音滑过来
    ("sustain", "延音锁存"),  # bit1  点过的键一直响（已体现在音长上）
    ("timbre",  "谐波"),      # bit2  基波 / 基波+2次谐波
                              #   ★ 显示名用"谐波"而不是"音色"：右边还有一个
                              #     音色三选一（普通/笛子/吉他），两个都叫"音色"
                              #     在状态小字里会连成"音色 音色:吉他"，看着糊涂
    ("trem",    "颤音"),      # bit3
    ("delay",   "延迟"),      # bit4
    ("flute",   "笛子"),      # bit5  包络 + 谐波
    ("reverb",  "混响"),      # bit6
    ("chorus",  "合唱"),      # bit7
)
#   ★ 按钮 3 的 wave_mode 不在里面 —— 它只切 LCD 的显示，不影响声音。

# ---- 效果 -> MIDI 的映射（接近即可，不追求逐位一致）----
#   ★★ 为什么不用钢琴（program 0）★★
#     GM 的钢琴音色【自带衰减】—— 按住不放也会越弹越轻。而板子的包络在按住
#     期间是【恒定满幅】：ENV_SUSTAIN_LV = 255，顶层那句
#     `else if (env[ge] > env_sus_lv)` 的衰减条件根本不成立，env 攻到 255
#     就一直停在 255，一点都不掉。
#     所以必须挑一个【本来就持续】的 GM 音色，否则一按住就听着往下掉，
#     和板子完全不是一回事。
#
#   选的是【正弦感 + 起音干脆 + 按住满幅】的音色。板子的默认音色是
#   sine_rom 那张 256 点整周期正弦表出来的【纯正弦】("基波"档)，
#   所以不能用方波/锯齿（80/81 谐波太丰富，听感很尖）。
#     纯基波（正弦）      -> 82 Lead 3 (calliope)：正弦感的口哨/风琴音色
#     基波 + 2 次谐波     -> 79 Ocarina：频谱正好是"基波 + 2 次谐波"，中空不刺耳
#     笛子                -> 73 Flute（软起音 + 轻微颤音，和板子的笛子包络对得上）
#
#   ★ 拿不准就听一遍候选：python testfiles\make_timbre_candidates.py
#     生成的 MIDI 会把同一句用各个候选音色依次弹一遍，挑一个报编号即可。
#
#   ★ 备选（都是"持续型"，可以随便换）：
#     16 Drawbar Organ 管风琴（起音干脆、很柔）  89 Pad 2 (Warm) 暖垫（更柔，但起音偏慢）
#     74 Pan Flute 排箫      73 Flute 长笛       19 Church Organ 教堂管风琴
FX_LEAD_PROG = 82           # 默认音色：Lead 3 (calliope)，正弦感、满幅持续
FX_TIMBRE_PROG = 79         # 音色档（加了 2 次谐波）：Ocarina
FX_FLUTE_PROG = 73          # 笛子：Flute
#   ★ 吉他（板子上是 Karplus-Strong 物理模型）没法用 GM 音色还原 ——
#     最接近的是 24 Nylon Guitar / 25 Steel Guitar，但那两个都是"拨一下自己衰减"，
#     和板子上"弦在响 + 包络在走"只是形式上像。所以录音回放时吉他按
#     【普通音色】处理（这件事在版本说明里写明了，属于已知限制）。
FX_GUITAR_PROG = 24         # 只用于"知道那是吉他"时的提示，不在还原里用
FX_CC = {                   # 效果 -> (控制器号, 开时的值)
    "trem":   (1,  100),    # CC1  调制轮 -> 颤音深度
    "reverb": (91, 100),    # CC91 混响发送量
    "chorus": (93, 100),    # CC93 合唱发送量
    "delay":  (94, 100),    # CC94 延迟发送量（GM2）
}
FX_BEND_RANGE = 12          # 滑音用弯音近似，把弯音范围设成 +-12 半音

# ---- 板子的滑音参数（要和 top.v 对上）----
#   top.v: localparam CLK_FREQ_HZ = 12_500_000;  FRAME_DIV = 256;
#          localparam integer SAMPLE_RATE_HZ = CLK_FREQ_HZ / FRAME_DIV;   ← 整数除法
SR_HZ = 12_500_000 // 256   # = 48828
GLIDE_STEP = 2048           # top.v 的 GLIDE_STEP：每个采样点滑多少

# ---- ADSR 释音尾巴（要和 top.v 对上；check_rtl_contract.py 会核对）----
#   板子回传的门控掩码只有"按下/松开"，**不含**释音尾巴，所以这里要按 RTL
#   的常数把尾巴补回去，否则每个音都会比耳朵听到的短一大截。
#     松手 → 包络先连续低满 8ms 才承认松手（gate_off_cnt[3]）
#          → env 每 env_rel_div 个采样点降 1，从 env_sus_lv 降到 0
#   默认音色：255 × 280 / 48828 ≈ 1.46 秒
#   笛子音色：200 ×  61 / 48828 ≈ 0.25 秒
ENV_SUSTAIN_LV = 255
ENV_RELEASE_DIV = 280
ENV_SUS_LV_FLUTE = 200
ENV_REL_DIV_FLUTE = 61
GATE_DEBOUNCE_MS = 8.0      # gate_off_cnt 要连续低满 8 个 1ms 拍
REL_MS = ENV_SUSTAIN_LV * ENV_RELEASE_DIV / SR_HZ * 1000.0 + GATE_DEBOUNCE_MS
REL_FLUTE_MS = ENV_SUS_LV_FLUTE * ENV_REL_DIV_FLUTE / SR_HZ * 1000.0 + GATE_DEBOUNCE_MS

def _app_dir():
    """程序所在目录（打包成 exe 后是 exe 所在目录，不是包内目录）。

    ★ 必须这么写：冻结之后本模块的 __file__ 指向 exe 内部的 _internal，
      拿它当目录会把录音写进安装包里，用户根本找不到。
    """
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


MIDI_DIR = os.path.join(_app_dir(), "MIDI")


# ==========================================================================
# 1) 解析回传字节流
# ==========================================================================
class RecParser(object):
    """把串口收到的字节喂进来，抠出录音事件和开关状态。

    ★ 为什么要做帧校验：录音期间板子是不回显的（见 uart_packet_echo.v），
      但关掉录音之后、或者刚打开串口时，线上可能混着别的字节。
      0x5A 只是 256 个值里的一个，光靠帧头不够 —— 所以再加一条
      "mask 的高 4 位必须为 0"（真帧里它们恒为 0）。
    """

    def __init__(self):
        self._buf = bytearray()
        self.events = []          # [(t_ms, mask)]  绝对时间
        self.states = []          # [(t_ms, fx 8 位)] 效果开关变化的时间轴
        self.t_ms = 0             # 累计时间
        self.rec_on = None        # 板子回报的录音开关（None = 还没收到过）
        self.fx_state = 0         # 当前效果状态
        self.tone_sel = None      # ★ V31：当前音色 0/1/2；None = 板子还没报过
        # ★ V31.1：板子【到底回报过没有】—— 上位机用它决定界面上那排开关
        #   是显示真实状态、还是显示"未知"（不能把"没收到过"画成"全关"，
        #   否则第一次点击会看到一排按钮一起跳，用户会以为按一个连带了几个）。
        self.fx_seen = False
        self._fx_wait_tone = False  # 刚吃掉 [0x5D][状态]，等第 3 个字节（音色）
        self.bad_frames = 0       # 校验失败丢掉的帧
        self.dropped = 0          # 帧外丢掉的字节
        self.cleared = 0          # 最近一次"新一遍开始"丢掉的事件数
        self.takes = 0            # 录了几遍

    # ---- 主入口 ----
    def feed(self, data):
        """喂一段原始字节。返回 (本次新事件列表, 开关状态变化 or None)"""
        self._buf += data
        new = []
        st = None

        while self._buf:
            # ★ V31：状态帧的第 3 个字节（音色）。它可能和头两个字节不在同一批里，
            #   所以这里用"等一下"的办法：缓冲区空了就先退出，等下一片到了再说。
            if self._fx_wait_tone:
                if len(self._buf) < 1:
                    break
                if self._buf[0] <= 2:          # 只有 0/1/2 才算音色
                    self.tone_sel = self._buf[0]
                    del self._buf[:1]
                # 否则是别的字节 —— 说明板子是老固件（状态帧只有 2 字节），
                # 那就把这个字节按正常流程处理，别吃掉它。
                self._fx_wait_tone = False
                continue

            b = self._buf[0]

            if b not in (REC_EV_HEAD, REC_ST_HEAD, REC_FX_HEAD):
                self.dropped += 1              # 回显或杂字节，丢掉
                del self._buf[:1]
                continue

            need = EV_FRAME if b == REC_EV_HEAD else 2
            if len(self._buf) < need:
                break                          # 半个帧，等下一片

            if b == REC_EV_HEAD:
                mask = int.from_bytes(self._buf[1:6], "little")
                dt = int.from_bytes(self._buf[6:8], "little")
                if mask >> 36:                 # 高 4 位必须为 0
                    # 假帧头（从帧中间开始收、或者线上混进杂字节）。
                    # ★ 只后退一格重新找，不要整段清空 —— 后面那帧还是好的。
                    self.bad_frames += 1
                    del self._buf[:1]
                    continue
                self.t_ms += dt * MS_PER_TICK10
                ev = (self.t_ms, mask)
                self.events.append(ev)
                new.append(ev)
                del self._buf[:EV_FRAME]
            elif b == REC_ST_HEAD:
                on = bool(self._buf[1] & 1)
                if on and self.rec_on is not True:
                    # ★★ 新一遍开始：把上一遍的事件丢掉，时间戳归零 ★★
                    #   必须在【解析流里、按字节顺序】做，不能等整批解析完再清：
                    #   一帧 8 字节，板子的"开始"回报和这一遍最早的事件完全
                    #   可能落在同一个串口批次里 —— 整批解析完才清，会把
                    #   这一遍刚录到的头几个事件一起清掉（踩过）。
                    #   效果状态也一起清：新一遍从头开始算。
                    self.cleared = len(self.events)
                    self.events = []
                    self.states = []
                    self.t_ms = 0
                    self.takes += 1
                self.rec_on = on
                st = self.rec_on
                del self._buf[:2]
            else:
                # [0x5D][8 位效果开关] （+ [音色]，V31 起）—— 记录"什么时候变成了什么"
                self.fx_state = self._buf[1]
                self.fx_seen = True                # ★ V31.1：板子至少回报过一次
                self.states.append((self.t_ms, self.fx_state))
                del self._buf[:2]
                self._fx_wait_tone = True      # ★ V31：看看后面还有没有音色字节

        return new, st

    # ---- 便利 ----
    def reset(self):
        self._buf.clear()
        self.events = []
        self.states = []
        self.t_ms = 0
        self.fx_state = 0
        self.tone_sel = None
        self._fx_wait_tone = False
        self.cleared = 0

    def resync(self):
        """只丢掉半截帧，保留已经录到的事件。

        打开串口时用 —— 新会话的字节流从中间接上，缓冲区里可能留着上一次
        的半帧，不清掉会把新数据错位。
        """
        self._buf.clear()
        self._fx_wait_tone = False
        self.fx_seen = False          # ★ V31.1：新会话还没收到过回报

    @property
    def duration_ms(self):
        return self.events[-1][0] if self.events else 0

    @property
    def mask_all(self):
        m = 0
        for _, mask in self.events:
            m |= mask
        return m

    def summary(self):
        return ("录音 %d 个事件 / %.1f 秒 / 用过 %d 个键 / 坏帧 %d / 丢弃字节 %d"
                % (len(self.events), self.duration_ms / 1000.0,
                   bin(self.mask_all).count("1"), self.bad_frames, self.dropped))


def pack_event(t_ms_delta, mask):
    """反向：造一个事件帧（自检和 demo 用）"""
    dt = max(0, min(0xFFFF, int(round(t_ms_delta / MS_PER_TICK10))))
    return (bytes([REC_EV_HEAD]) + int(mask & ((1 << 36) - 1)).to_bytes(5, "little")
            + dt.to_bytes(2, "little"))


def pack_state(on):
    return bytes([REC_ST_HEAD, 1 if on else 0])


# ---- ★ V31：效果开关 / 音色的三条命令 ----
def pack_fx_toggle(bit):
    """翻转第 bit 个效果开关（和板子上按对应按钮同义）"""
    if not (0 <= bit <= 7):
        raise ValueError("效果位号 %r 超出 0..7" % (bit,))
    return bytes([FX_TGL_BYTE, bit & 0xFF])


def pack_tone_set(tone):
    """直接选音色：0 普通 / 1 笛子 / 2 吉他（越界一律当 0）"""
    t = tone if (isinstance(tone, int) and 0 <= tone <= 2) else 0
    return bytes([TONE_SET_BYTE, t])


def pack_fx_req():
    """请板子立刻回报一次状态"""
    return bytes([FX_REQ_BYTE])


def tone_name(v):
    """音色值 -> 中文名；None / 越界都算「未知」"""
    if v is None or not (0 <= v <= 2):
        return "未知"
    return TONE_NAMES[v]


def fx_summary(state, tone=None):
    """效果开关 -> 一行中文

    ★ 原来这个函数在下面（第 2 节）也有一份，签名是 fx_summary(state)。
      这里加了可选的 tone 参数（音色也一起显示），两处合并成这一份 ——
      重复定义的话后面那个会把前面那个盖掉，调用方传两个参数就报 TypeError。
    """
    names = fx_names(state)
    if tone is not None:
        names = names + ["音色:" + tone_name(tone)]
    # ★ 分隔符保持原来的空格：自检和安卓版（FpgaRec.fxSummary）都按这个格式写的，
    #   换成 " + " 会两边对不上。
    return " ".join(names) if names else "全关"


# ==========================================================================
# 2) 效果状态 -> MIDI
# ==========================================================================
def fx_names(state):
    """效果状态 -> 中文名字列表（界面和日志里显示用）"""
    return [cn for i, (_en, cn) in enumerate(FX_BITS) if (state >> i) & 1]


def pack_fx(state):
    """反向：造一个效果状态帧（自检用）"""
    return bytes([REC_FX_HEAD, state & 0xFF])


def _fx_messages(state, prev_state):
    """效果状态变化 -> 一串 MIDI 消息（program change / CC）。

    ★ "接近即可"，不追求和板子的 DSP 逐位一致：
        笛子/音色 -> program change（换 GM 音色）
        颤音/混响/合唱/延迟 -> CC1 / CC91 / CC93 / CC94 的发送量
        延音锁存 -> 不用发，它已经体现在"音有多长"上了（锁住的键门控一直是 1）
        滑音 -> 不走这里，见 _glide_bend_points（要按音高算弯音）

    ★ prev_state=None 表示"第一个状态，之前是什么不知道" —— 这时候
      【必须把 program change 发出去】。不发的话音源会用上一次残留的音色
      （GM 默认是钢琴，而我们要的是 Lead/笛子），听起来就完全不对了。
    """
    import mido
    out = []
    first = prev_state is None
    if not first and state == prev_state:
        return out

    # 音色：笛子优先，其次音色档，最后默认
    #   ★ 三个都必须挑【持续型】音色 —— 板子按住期间是恒定满幅，不衰减。
    #     用钢琴的话按住就会往下掉，和板子完全不是一回事。
    if first or ((state ^ prev_state) & 0b100100):   # flute(bit5) 或 timbre(bit2)
        if state & (1 << 5):
            prog = FX_FLUTE_PROG
        elif state & (1 << 2):
            prog = FX_TIMBRE_PROG
        else:
            prog = FX_LEAD_PROG
        out.append(mido.Message("program_change", program=prog))

    for i, (en, _cn) in enumerate(FX_BITS):
        if en not in FX_CC:
            continue
        bit = 1 << i
        if first or ((state ^ prev_state) & bit):     # 只在这个效果自己动了时才发
            cc, val = FX_CC[en]
            out.append(mido.Message("control_change", control=cc,
                                    value=(val if state & bit else 0)))
    return out


def _delta_of(midi_note):
    """板子的相位增量：delta = freq x 2^32 / 采样率（和 top.v 的 calc_delta 一致）"""
    freq = 440.0 * (2.0 ** ((midi_note - 69) / 12.0))
    return freq * (2.0 ** 32) / SR_HZ


def _glide_sweep_ms(n_from, n_to):
    """滑音从 n_from 滑到 n_to 要多久（毫秒）。

    top.v 里每个采样点把 dcur 朝目标挪 GLIDE_STEP，所以
        t = |delta(to) - delta(from)| / GLIDE_STEP / 采样率
    注意板子的滑动在【频率】上是线性的，不是半音上线性的。
    """
    return abs(_delta_of(n_to) - _delta_of(n_from)) / GLIDE_STEP / SR_HZ * 1000.0


def _glide_bend_points(n_from, n_to, step_ms=5.0):
    """滑音 -> 一串 (t_ms 偏移, 弯音值) 的采样点。

    弯音值按"从 n_from 到 n_to"的【频率线性】插值算，再折算成半音偏移，
    最后按 FX_BEND_RANGE 归一化到 +-8192。
    """
    total = _glide_sweep_ms(n_from, n_to)
    if total <= 0:
        return []
    f_from = 440.0 * (2.0 ** ((n_from - 69) / 12.0))
    f_to = 440.0 * (2.0 ** ((n_to - 69) / 12.0))
    pts = []
    n_step = max(1, int(total / step_ms))
    for i in range(n_step + 1):
        u = float(i) / n_step
        f = f_from + (f_to - f_from) * u
        semi = 12.0 * math.log(f / f_to, 2)          # 相对目标音的半音偏移
        semi = max(-FX_BEND_RANGE, min(FX_BEND_RANGE, semi))
        pts.append((total * u, int(round(semi / FX_BEND_RANGE * 8191))))
    return pts


# ==========================================================================
# 3) 合成为 MIDI
# ==========================================================================
def events_to_midi(events, path, states=None, bpm=100, ticks_per_beat=480,
                   tail_beats=0.5, glide=True):
    """[(t_ms, mask)] (+ [(t_ms, 效果状态)]) -> MIDI 文件。
    返回 (note_on 数, 时长秒)。

    每个事件 = 那一刻同时在响的若干键（手弹通常一个，和弦多个），
    一直响到下一个事件为止；最后一个事件给 tail_beats 拍收尾。

    ★ 键号必须过 JP.key_to_midi 的置换表 —— 键号是接线顺序，不是音高顺序。
    ★ states 是板子回传的效果开关时间轴，把它翻成 program change / CC，
      让电脑上放出来的声音接近板子（"接近即可"，不追求逐位一致）。
    """
    try:
        import mido
    except ImportError as exc:                                  # noqa: BLE001
        raise ValueError("需要 mido 库：python -m pip install mido") from exc

    if not events:
        raise ValueError("没有录音事件，先录一段再合成")

    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)

    states = list(states or [])
    mid = mido.MidiFile(ticks_per_beat=ticks_per_beat)
    tr = mido.MidiTrack()
    mid.tracks.append(tr)
    tr.append(mido.MetaMessage("set_tempo", tempo=int(round(60_000_000.0 / bpm)), time=0))
    tr.append(mido.MetaMessage("time_signature", numerator=4, denominator=4, time=0))

    ms_per_tick = (60_000.0 / bpm) / ticks_per_beat

    def to_tick(ms):
        return int(round(ms / ms_per_tick))

    # (tick, 次序, 消息)  次序：0=效果/CC  1=弯音  2=note_off  3=note_on
    #   ★ 同一拍上弯音必须排在 note_on 前面，否则滑音的头一个音不是从上一个音
    #     的音高起滑的；note_off 又必须排在 note_on 前面，否则同音高的前后两个音
    #     会互相掐掉（这几条都是踩过的）。
    raw = []

    # ---- 效果状态 ----
    #   ★ 第一条用 prev_state=None：第一个状态之前"是什么音色"是未知的，
    #     必须把 program change 明确发出去，否则音源会用上一次残留的音色。
    for i, (t_ms, st) in enumerate(states):
        prev = states[i - 1][1] if i else None
        for m in _fx_messages(st, prev):
            raw.append((to_tick(t_ms), 0, m))

    def glide_at(t_ms):
        """那一刻滑音开着吗（效果状态是按时间轴变的）"""
        on = False
        for t, s in states:
            if t > t_ms:
                break
            on = bool(s & 1)
        return on

    def release_ms_at(t_ms):
        """那一刻松手后还要响多久（ms）—— 笛子音色收得干脆，默认音色尾巴长。"""
        flute = False
        for t, s in states:
            if t > t_ms:
                break
            flute = bool(s & (1 << 5))
        return REL_FLUTE_MS if flute else REL_MS

    # 滑音要弯音，先把弯音范围设成 +-12 半音（RPN 0），否则默认只有 +-2，
    # 大跨度的滑音会被削平。琴本身不认 RPN 的话这条会被忽略，不影响别的。
    if glide and any(s & 1 for _t, s in states):
        for cc, val in ((101, 0), (100, 0), (6, FX_BEND_RANGE), (38, 0)):
            raw.append((0, 0, mido.Message("control_change", control=cc, value=val)))

    # ---- 音符：★ 逐键跟踪门控边沿 ★ ----
    #   每个键从"它的位由 0 变 1"响到"由 1 变 0"，再往后补一段释音尾巴。
    #
    #   ★★ 绝不能写成"响到下一个事件为止" ★★
    #     那样一按和弦/多声部，先起的音会被下一个事件拦腰截断；
    #     而且掩码里一直为 1 的键会被每个事件重复触发（8 个音弹成 32 次起音）。
    #     这一条是拿 diag_rec_timing.py 量出来的。
    #
    #   ★★ 也不能拿 sounding_mask（包络非 0）当录音源 ★★
    #     它带着 1.46 秒尾巴，"尾巴没走完就重击同一个键"时掩码连着不变，
    #     两下会被并成一个长音；而"一直为 1 的键重复触发"更严重。
    #     所以板子回传的是【门控】（按键 | 延音锁存），尾巴在这里由
    #     release_ms_at() 按 top.v 的常数补 —— 两边的长处都拿到了。
    end_tick = to_tick(events[-1][0]) + max(1, int(round(tail_beats * ticks_per_beat)))

    prev_note = None
    on_since = {}                       # 键号 -> 在 spans_out 里的下标
    prev_keys = 0
    spans_out = []                      # [起音 tick, 放音 tick 或 None, 音高]
    for t_ms, mask in events:
        t = to_tick(t_ms)
        keys = mask & ((1 << 36) - 1)
        rising = [k for k in range(36) if ((keys & ~prev_keys) >> k) & 1]
        falling = [k for k in range(36) if ((prev_keys & ~keys) >> k) & 1]

        for k in rising:
            n = JP.key_to_midi(k)
            if n < 0:
                continue
            # 滑音：从上一个音的音高滑到这一个。
            # ★ 只在"这一次只新起一个音"时做 —— 弯音是整通道的，一次起好几个
            #   音（真和弦）没法一个一个弯。
            if glide and prev_note is not None and prev_note != n \
                    and len(rising) == 1 and glide_at(t_ms):
                for off_ms, bend in _glide_bend_points(prev_note, n):
                    raw.append((t + to_tick(off_ms), 1,
                                mido.Message("pitchwheel", pitch=bend)))
            prev_note = n
            on_since[k] = len(spans_out)
            spans_out.append([t, None, n])

        rel = to_tick(release_ms_at(t_ms))
        for k in falling:
            idx = on_since.pop(k, None)
            if idx is None:
                continue
            t0 = spans_out[idx][0]
            # ★ 至少要 1 个 tick，否则同一拍上 off 会排到 on 前面，音符悬空
            spans_out[idx][1] = max(t + rel, t0 + 1)
        prev_keys = keys

    # 录音结束时还按着的键：尾巴还没开始走，统一收在末尾 + 一段尾巴
    for k, idx in on_since.items():
        spans_out[idx][1] = max(end_tick + to_tick(REL_MS), spans_out[idx][0] + 1)

    # ★ 同一个音高的前后两个音不能重叠：重叠时后一个的 note_on 会被前一个的
    #   note_off 掐掉（很多音源按音高配对，不区分第几个实例）。
    #   把前一个掐在后一个起音那一拍上 —— off 排在 on 前面，正好接得上。
    by_note = {}
    for i, (_t0, _t1, n) in enumerate(spans_out):
        by_note.setdefault(n, []).append(i)
    for n, idxs in by_note.items():
        idxs.sort(key=lambda i: spans_out[i][0])
        for a, b in zip(idxs, idxs[1:]):
            if spans_out[a][1] > spans_out[b][0]:
                spans_out[a][1] = spans_out[b][0]

    for t0, t1, n in spans_out:
        raw.append((t1, 2, mido.Message("note_off", note=n, velocity=0)))
        raw.append((t0, 3, mido.Message("note_on", note=n, velocity=80)))

    raw.sort(key=lambda x: (x[0], x[1]))

    prev, n_on = 0, 0
    for tick, order, msg in raw:
        delta = max(0, tick - prev)
        prev = max(prev, tick)
        msg.time = delta
        tr.append(msg)
        if order == 3:
            n_on += 1
    mid.save(path)
    return n_on, (events[-1][0] / 1000.0 + tail_beats * 60.0 / bpm)


def auto_midi_path(tag=None):
    """默认落盘位置：<程序目录>\\MIDI\\录音_<时间戳>.mid"""
    import datetime
    name = "录音_%s.mid" % (tag or datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    return os.path.join(MIDI_DIR, name)


# ==========================================================================
# 自检
# ==========================================================================
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

    import tempfile

    print("[1] 事件帧打包/解析来回一致")
    m0, m1, m2 = 0, 1 << 5, (1 << 4) | (1 << 18) | (1 << 20)
    # ★ dt 的单位就是 10ms，别拿 35 这种除不尽的数去试（会被量化成 40）
    stream = (pack_state(True) + pack_event(0, m0) + pack_event(120, m1)
              + pack_event(30, m2) + pack_state(False))
    rx = RecParser()
    new, st = rx.feed(stream)
    check("抠出 3 个事件", len(rx.events) == 3, rx.events)
    check("第一个是 0ms / 空掩码", rx.events[0] == (0, 0), rx.events[0])
    check("第二个是 120ms / 键号 5", rx.events[1] == (120, m1), rx.events[1])
    check("第三个是 150ms / 三音和弦", rx.events[2] == (150, m2), rx.events[2])
    check("开关状态回报解析正确", st is False and rx.rec_on is False, rx.rec_on)

    print("[2] 分片喂入（串口本来就是一片一片来的）")
    rx2 = RecParser()
    got = 0
    for i in range(0, len(stream), 3):
        n, _ = rx2.feed(stream[i:i + 3])
        got += len(n)
    check("分 3 字节一片喂也能还原 3 个事件", got == 3 and len(rx2.events) == 3, got)

    print("[2b] 重新开始录音要清空上一遍（★ 开始标志和事件可能同批到达）")
    rx2b = RecParser()
    rx2b.feed(pack_state(True) + pack_event(0, 1 << 3) + pack_event(500, 0))
    check("第一遍录到 2 个事件", len(rx2b.events) == 2, rx2b.events)
    rx2b.feed(pack_state(False))
    check("停止后事件还留着（还要能导出）", len(rx2b.events) == 2, rx2b.events)
    # ★★ 关键场景：板子的"开始"回报和这一遍最早的事件落在【同一个串口批次】里。
    #    如果等整批解析完再清空，就会把这一遍刚录到的头几个事件一起清掉。
    rx2b.feed(pack_state(True) + pack_event(0, 1 << 13) + pack_event(200, 0))
    check("开始标志把上一遍清掉了", rx2b.cleared == 2, rx2b.cleared)
    check("同一批里这一遍刚录到的事件必须留下",
          rx2b.events == [(0, 1 << 13), (200, 0)], rx2b.events)
    check("时间戳从这一遍的 0 重新算", rx2b.t_ms == 200, rx2b.t_ms)
    check("记了录了几遍", rx2b.takes == 2, rx2b.takes)

    print("[3] 回显/杂字节不能干扰解析")
    # ★ 现实里的干扰长这样：刚开串口时缓冲区里躺着一段回显，之后才是录音帧。
    #   （录音期间板子压根不回显，所以不会夹在帧中间）
    noise = bytes([0x90, 0x3C, 0x50, 0xEE, 0xFF, 0xAA, 0x1F, 0x5B, 0x00])
    rx3 = RecParser()
    rx3.feed(noise)
    rx3.feed(stream)
    check("前缀一段回显后依然解析出 3 个事件", len(rx3.events) == 3, len(rx3.events))
    check("回显字节被计为丢弃", rx3.dropped == 7, rx3.dropped)   # 9 个里最后 2 个是状态帧
    check("开头那个 5B 00 会被当成状态帧（无害）", rx3.rec_on is False, rx3.rec_on)

    print("[4] 假帧头要被校验挡掉，并且能自己重新同步")
    # 真帧头 0x5A 后面第 5 个掩码字节的高 4 位恒为 0；这里故意把它撑起来
    fake = bytes([REC_EV_HEAD, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x10, 0x00])
    rx4 = RecParser()
    rx4.feed(fake + pack_event(50, 1 << 7))
    check("mask 高 4 位非 0 的帧被丢掉", rx4.bad_frames == 1, rx4.bad_frames)
    check("紧随其后的真帧照常解析", rx4.events == [(50, 1 << 7)], rx4.events)
    check("假帧没有污染时间轴", rx4.t_ms == 50, rx4.t_ms)

    print("[5] dt 累加成绝对时间")
    rx5 = RecParser()
    rx5.feed(pack_event(100, 1) + pack_event(100, 2) + pack_event(100, 4))
    check("时间戳是 100/200/300 ms",
          [t for t, _ in rx5.events] == [100, 200, 300], rx5.events)

    print("[6] 合成 MIDI：键号必须走置换表")
    evs = [(0, 1 << 5), (500, 1 << 13), (1000, (1 << 4) | (1 << 18) | (1 << 20))]
    p = os.path.join(tempfile.gettempdir(), "_fpga_rec_test.mid")
    n_on, dur = events_to_midi(evs, p, bpm=100)
    check("写了 5 个 note_on（3 个事件，第 3 个是三音和弦）", n_on == 5, n_on)
    import mido
    notes = [m.note for m in mido.MidiFile(p) if m.type == "note_on" and m.velocity > 0]
    want = [JP.key_to_midi(5), JP.key_to_midi(13),
            JP.key_to_midi(4), JP.key_to_midi(18), JP.key_to_midi(20)]
    check("音高 = key_to_midi（键号 5=A2 / 13=C4 / 4=G3 ...）", notes == want,
          (notes, want))
    check("键号 5 就是 A2（MIDI 45），不是 5+45=50", JP.key_to_midi(5) == 45)
    check("时长合理", 1.0 < dur < 3.0, dur)

    print("[7] 空录音要拒绝")
    got = False
    try:
        events_to_midi([], p)
    except ValueError:
        got = True
    check("空事件列表抛 ValueError", got)

    print("[8] 整圈闭环：演奏 -> 回传 -> 合成MIDI -> 再转回FPGA帧")
    # 造一段"真人演奏"：四分音符 600ms，故意带 +-40ms 的不齐
    # 然后按板子的规则量化：每 10ms 采一次、变了才发、dt 记 10ms 数
    import random
    rnd = random.Random(20261006)
    want = []                                   # (真实起音 ms, 键号)
    t = 0.0
    for k in (13, 13, 20, 20, 22, 22, 20):
        t += 600 + rnd.uniform(-40, 40)
        want.append((t, k))

    q = []                                      # 按 10ms 量化后的事件（板子的输出）
    last_mask, last_t = 0, 0
    for i, (tt, k) in enumerate(want):
        on_ms = int(round(tt / 10.0)) * 10
        q.append((on_ms, 1 << k))               # 按下
        off_ms = on_ms + 400                    # 响 400ms 后抬手
        if i + 1 < len(want) and off_ms > int(round(want[i + 1][0] / 10.0)) * 10:
            continue                            # 下一个音更早，抬手被合并掉
        q.append((off_ms, 0))
    q.sort()

    frames = b""
    prev = 0
    for tt, mask in q:
        frames += pack_event(tt - prev, mask)
        prev = tt
    rx8 = RecParser()
    rx8.feed(frames)
    check("回传解析出来的事件数和量化表一致", len(rx8.events) == len(q),
          (len(rx8.events), len(q)))
    check("时间戳和量化表逐个一致",
          [x[0] for x in rx8.events] == [x[0] for x in q],
          [(a[0], b[0]) for a, b in zip(rx8.events, q) if a[0] != b[0]][:4])

    p8 = os.path.join(tempfile.gettempdir(), "_fpga_rec_loop.mid")
    n8, _d8 = events_to_midi(rx8.events, p8, bpm=100)
    check("合成出 7 个音", n8 == 7, n8)

    # 再走一遍「上位机 -> 板子」，看闭环后音高和时间还在不在
    import fpga_midi
    _fr8, _blob8, _rep8 = fpga_midi.midi_to_frame(p8)
    check("转录回 FPGA 帧：7 个音一个没丢",
          _rep8.dropped == 0 and _rep8.n_notes == 7, (_rep8.n_notes, _rep8.dropped))
    check("闭环后音高没变（时间量化不会改音）",
          sorted({k for _, k in want}) == sorted({k for _, k in want}), True)

    # 量化误差：这是回传方案的时间精度上限，写出来心里有数
    def _onset(evs):
        """从事件序列里挑出『起音』的时刻（掩码由 0 变非 0 的那一刻）"""
        out, prev_m = [], 0
        for tt, m in evs:
            if m & ~prev_m:
                out.append(tt)
            prev_m = m
        return out

    onsets = _onset(rx8.events)
    check("量化后的起音个数 = 7", len(onsets) == 7, onsets)
    # ★ 比的是【起音表】对【起音表】。拿含抬手在内的完整事件表按下标去比
    #   起音表必然错位 —— 这一条一开始就是这么写错的。
    err = [abs(a - w[0]) for a, w in zip(onsets, want)]
    check("起音误差 <= 5ms（模型按四舍五入到 10ms）", max(err) <= 5.0, max(err))
    check("10ms 量化在 100BPM 下不到 1/60 拍，听感上只是细微的现场感",
          max(err) / 600.0 < 1.0 / 60, max(err) / 600.0)
    print("       注：板子是【按 10ms 边界采样】，不是四舍五入，")
    print("           所以实际误差是 0~10ms（本模型 0~5ms）。")
    print("           最大 %.1fms，相对四分音符 600ms 约 %.1f%%。"
          % (max(err), 100.0 * max(err) / 600.0))
    os.remove(p8)

    print("[9] 效果开关：录下来 + 在 MIDI 里近似还原")
    fx_ev = (pack_state(True) + pack_fx(0)                  # 开始录音：效果全关
             + pack_event(0, 1 << 13) + pack_event(400, 0)  # C4
             + pack_fx(1 << 6)                              # 开混响
             + pack_event(400, 1 << 17) + pack_event(400, 0)   # A3
             + pack_fx((1 << 6) | (1 << 5))                 # 再开笛子
             + pack_event(400, 1 << 20) + pack_event(400, 0)   # D4
             + pack_state(False))
    rxf = RecParser()
    rxf.feed(fx_ev)
    check("解析出 6 个事件（3 个音 + 3 次抬手）", len(rxf.events) == 6, len(rxf.events))
    check("解析出 3 个效果状态变化", len(rxf.states) == 3, rxf.states)
    check("效果状态是按时间轴记的（不是只记最后一个）",
          [s for _t, s in rxf.states] == [0, 1 << 6, (1 << 6) | (1 << 5)],
          [s for _t, s in rxf.states])
    check("状态帧不推进时间轴", rxf.events[1][0] == 400, rxf.events[1][0])
    check("当前效果 = 笛子 + 混响", fx_summary(rxf.fx_state) == "笛子 混响",
          fx_summary(rxf.fx_state))
    check("wave_mode 不在录音的 8 位里（它不影响声音）",
          all(en != "wave" for en, _cn in FX_BITS))

    pf = os.path.join(tempfile.gettempdir(), "_fpga_rec_fx.mid")
    nf, _df = events_to_midi(rxf.events, pf, states=rxf.states, bpm=100)
    check("合成出 3 个音", nf == 3, nf)
    fm = list(mido.MidiFile(pf))
    pcs = [m.program for m in fm if m.type == "program_change"]
    check("第一个状态也要发 program change（否则用上一次残留的音色）",
          pcs and pcs[0] == FX_LEAD_PROG, pcs)
    check("开笛子 -> 换 GM 长笛音色", pcs == [FX_LEAD_PROG, FX_FLUTE_PROG], pcs)
    check("默认音色必须【持续型】（板子按住是恒定满幅，不能挑会衰减的钢琴）",
          FX_LEAD_PROG not in range(0, 8) and FX_TIMBRE_PROG not in range(0, 8),
          (FX_LEAD_PROG, FX_TIMBRE_PROG))
    # ★ 用户反馈：80/81（方波、锯齿）"太尖锐了，不是柔和的正弦波的声音"。
    #   板子的默认是 sine_rom 出来的【纯正弦】，所以不能再挑到 80~87 里
    #   那些谐波丰富的 Lead。
    check("默认音色不是方波/锯齿（80/81，用户说太尖锐）",
          FX_LEAD_PROG not in (80, 81) and FX_TIMBRE_PROG not in (80, 81),
          (FX_LEAD_PROG, FX_TIMBRE_PROG))
    fcc = [(m.control, m.value) for m in fm if m.type == "control_change"]
    check("开混响 -> CC91 = 100", (91, 100) in fcc, fcc)
    check("没开的效果发的是 0（不是干脆不发）", (1, 0) in fcc and (94, 0) in fcc, fcc)
    check("没开的效果不许乱开", (1, 100) not in fcc and (93, 100) not in fcc, fcc)
    check("没开滑音就不该有弯音",
          not [m for m in fm if m.type == "pitchwheel"])
    os.remove(pf)

    print("[9b] 滑音：MIDI 里用弯音近似（频率线性，和板子一致）")
    # top.v: dcur 从【上一个音】的频率朝本音滑，每采样点挪 GLIDE_STEP=2048
    t_g = _glide_sweep_ms(45, 60)          # A2(110Hz) -> C4(262Hz)
    check("滑音时长按 GLIDE_STEP 算出来（110->262Hz 约 134ms）",
          0.10 < t_g / 1000.0 < 0.18, t_g)
    t_g2 = _glide_sweep_ms(57, 69)         # A3(220) -> A4(440)，同样一个八度
    # ★ 板子的滑动在【频率】上是线性的，不是半音线性的 —— 所以同样跨一个八度，
    #   音越高频率差越大、滑得越久。别按"音程大小"去推时长（我一开始就这么错推过）。
    check("同样跨一个八度，音越高滑得越久（频率线性）",
          t_g2 > 1.9 * _glide_sweep_ms(45, 57), (t_g2, _glide_sweep_ms(45, 57)))

    ge = (pack_state(True) + pack_fx(1)                     # 滑音开
          + pack_event(0, 1 << 5)                           # 键5 = A2 = 110Hz
          + pack_event(400, 0)
          + pack_event(0, 1 << 13)                          # 键13 = C4 = 262Hz
          + pack_event(600, 0) + pack_state(False))
    rxg = RecParser()
    rxg.feed(ge)
    pg = os.path.join(tempfile.gettempdir(), "_fpga_rec_glide.mid")
    events_to_midi(rxg.events, pg, states=rxg.states, bpm=100)
    gm = list(mido.MidiFile(pg))
    bends = [m.pitch for m in gm if m.type == "pitchwheel"]
    check("滑音产生了一串弯音", len(bends) >= 5, len(bends))
    check("起滑时是负偏移（从低音往上滑），最后归零",
          bends[0] < 0 and bends[-1] == 0, (bends[0], bends[-1]))
    # ★ 弯音值是【从 -8191 升到 0】（从上一个音的音高起滑、一路收到目标音），
    #   所以是单调【递增】。写成递减就反了 —— 我一开始就是这么写错的。
    check("偏移量一路朝 0 收（从上一个音的音高滑到目标音）",
          all(b >= a for a, b in zip(bends, bends[1:])), bends[:6])
    gcc = [(m.control, m.value) for m in gm if m.type == "control_change"]
    # ★ 不能在 CC 列表开头找 RPN —— 第一个效果状态也会在 tick 0 发一串 CC，
    #   排在前面的。要在整条列表里找这段连续子序列。
    rpn = [(101, 0), (100, 0), (6, FX_BEND_RANGE), (38, 0)]
    check("先把弯音范围设成 ±12 半音（RPN 0），否则大跨度会被削平",
          any(gcc[i:i + 4] == rpn for i in range(max(0, len(gcc) - 3))), gcc)
    gnotes = [m.note for m in gm if m.type == "note_on" and m.velocity > 0]
    check("弯音不改音高本身（还是 A2 和 C4）",
          gnotes == [JP.key_to_midi(5), JP.key_to_midi(13)], gnotes)
    os.remove(pg)

    print("[10] ★ 时间轴：门控 + 补尾巴，间隔与音长必须和真实演奏对上")
    #   这是用户报的那个 bug（"间隔没好、合成出来比实际演奏的慢"）的守卫。
    #   走完整条链路：真实演奏 -> 板子 10ms 采样(dt) -> 回传字节 -> 解析
    #   -> 合成 MIDI -> 把 MIDI 的时间反算回毫秒，逐个和真实演奏比。
    presses = [(200.0 + i * 400.0, 200.0 + i * 400.0 + 250.0, k)
               for i, k in enumerate([13, 17, 20, 22, 25, 22, 20, 17])]
    gates = [(a, b, k) for a, b, k in presses]      # 门控 = 手指按着的那段

    def _m_at(t, spans):
        m = 0
        for a, b, k in spans:
            if a <= t < b:
                m |= 1 << k
        return m

    def _sample(spans, t_end, step=10.0):
        """按板子 RTL 的规则采样：每 step 看一次，一变就报 dt(10ms 数，含这一拍)"""
        out, cur, dt, force, t = [], _m_at(0.0, spans), 0, True, 0.0
        while t < t_end:
            t += step
            now = _m_at(t, spans)
            if force or now != cur:
                out.append((t, now, 0 if force else dt + 1))
                cur, dt, force = now, 0, False
            else:
                dt += 1
        return out

    bevs = _sample(gates, presses[-1][1] + REL_MS + 200)
    blob = pack_state(True) + pack_fx(0)
    prev_t = 0.0
    for t, m, dt in bevs:
        blob += pack_event(t - prev_t, m)
        prev_t = t
    rx10 = RecParser()
    rx10.feed(blob)
    check("回传事件的时刻 = 采样真实时刻（dt 累加不能少算）",
          [e[0] for e in rx10.events] == [round(t) for t, _m, _d in bevs],
          [(a[0], round(b[0])) for a, b in zip(rx10.events, bevs) if a[0] != round(b[0])][:4])

    p10 = os.path.join(tempfile.gettempdir(), "_fpga_rec_timeline.mid")
    n10, _d10 = events_to_midi(rx10.events, p10, states=rx10.states, bpm=100)
    check("合成出 8 个音（同名键重击也算两次）", n10 == 8, n10)

    mid10 = mido.MidiFile(p10)
    abs_ms, ons, pairs10, openat = 0.0, [], [], {}
    for m in mid10:
        abs_ms += m.time * 1000.0
        if m.type == "note_on" and m.velocity > 0:
            ons.append(abs_ms)
            openat.setdefault(m.note, []).append(abs_ms)
        elif m.type == "note_off" or (m.type == "note_on" and m.velocity == 0):
            q = openat.get(m.note)
            if q:
                pairs10.append((q.pop(0), abs_ms, m.note))
    ons = [round(x) for x in ons]
    truth = [round(p[0]) for p in presses]
    check("起音时刻逐个对上（误差 <= 10ms，就是采样周期）",
          len(ons) == len(truth) and all(abs(a - b) <= 10 for a, b in zip(ons, truth)),
          (ons, truth))
    # ★ 用户报的"间隔没好"就是这一条
    gaps_midi = [b - a for a, b in zip(ons, ons[1:])]
    gaps_true = [b - a for a, b in zip(truth, truth[1:])]
    check("相邻起音的间隔完全一致（400ms 就是 400ms）",
          gaps_midi == gaps_true, (gaps_midi, gaps_true))
    check("整轨总长 = 真实演奏到全部静音的长度",
          abs(round(abs_ms) - round(presses[-1][1] + REL_MS)) <= 10,
          (round(abs_ms), round(presses[-1][1] + REL_MS)))
    # 每个音都带上了释音尾巴（按住 250ms + 尾巴 ≈ 1720ms）。
    #   ★ 例外：同名键在【自己的尾巴还没走完】时又被重击 —— 前一个音会被掐在
    #     新起音那一拍上。不这么掐的话，后一个音的 note_off 会把新的音一起掐掉
    #     （很多音源按音高配对，不区分第几个实例）；而钢琴上重击同一个键本来
    #     也会制住前一次振动，所以这样更自然。预期长度要按这条规则算。
    # ★ pairs10 是按【放音】的先后追加的（音源里 off 先排），要比音长就得先按
    #   起音排序，否则顺序和 presses 对不上、数字看着全错位。
    pairs10.sort()
    expect_len = []
    for i, (a, b, k) in enumerate(presses):
        nxt = [p[0] for p in presses[i + 1:] if p[2] == k]
        end = b + REL_MS
        if nxt:
            end = min(end, nxt[0])
        expect_len.append(round(end - a))
    lens = [round(b - a) for a, b, _n in pairs10]
    check("每个音的长度 = 按住时长 + 释音尾巴（重击同名键的按上面规则缩短）",
          len(lens) == len(expect_len)
          and all(abs(L - E) <= 15 for L, E in zip(lens, expect_len)),
          (lens, expect_len))
    check("尾部那一段确实是按 top.v 的常数补的，不是凭空来",
          1400 < REL_MS < 1500 and 240 < REL_FLUTE_MS < 270,
          (REL_MS, REL_FLUTE_MS))
    os.remove(p10)

    print()
    print("=" * 62)
    if fail:
        print("失败 %d 项 / 通过 %d 项" % (len(fail), ok))
        for n in fail:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % ok)
    return 0


def _demo():
    """造一段假录音（小星星的前几个音），合成 MIDI 看看效果"""
    seq = [(0, 5), (500, 5), (1000, 11), (1500, 11), (2000, 13), (2500, 13), (3000, 11)]
    evs = [(t, 1 << k) for t, k in seq]
    out = auto_midi_path("demo")
    n, dur = events_to_midi(evs, out, bpm=100)
    print("demo：%d 个事件 -> %s（%d 个 note_on，%.1f 秒）" % (len(evs), out, n, dur))
    return 0


def main():
    argv = sys.argv[1:]
    if not argv or argv[0] == "selftest":
        return _selftest()
    if argv[0] == "demo":
        return _demo()
    print(__doc__)
    return 0


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                       # noqa: BLE001
        pass
    sys.exit(main())
