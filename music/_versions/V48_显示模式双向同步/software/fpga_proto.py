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
    力度  0xC3 VEL    （0..127，只顾【紧接着的下一个事件】；不发 = 127）

命令行
──────
    python fpga_proto.py selftest     跑自检
    python fpga_proto.py demo         打印一个带和弦的示例帧
"""

from __future__ import annotations

import random
import sys
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Sequence, Tuple

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
# ---- ★ V32：事件八度偏移（拓宽蓝牙播放音域）----
#   [0xCC][有符号 3 位] —— 只作用于【它后面那一格事件】。
#   0..3 = +0..+3 个八度；4..7 = -4..-1 个八度（两补数）。
#   ★ 0（= 原调）时上位机【不发这个字节】，所以老谱子编出来逐字节不变。
OCT_BYTE = 0xCC
OCT_MIN, OCT_MAX = -4, 3

# ---- ★ V37：和弦内【逐音八度】前缀 ----
#   [0xC0] ... [0xCD][有符号 3 位] 音号 ... [0xC1]
#   0xCD 只作用于【紧跟在它后面的那一个音号】；它和 0xCC 的区别是：
#     0xCC  → 一整格共用一个八度（超出范围的音只能折返，旋律会断）
#     0xCD  → 这一个音自己的八度（"低音 + 高音跨 4 个八度"的和弦也能按原音高发）
#   ★ 没有 0xCD 的音号继续用本事件的 oct（= 现在的行为），
#     一个 0xCD 都不发的谱子编出来【逐字节不变】。
#   ★ 0xCD 不会和音号撞车：音号最大 0x23，0xCD 在命令区且未分配。
KEY_OCT_BYTE = 0xCD

# ★★ V43：跟谱演奏模式命令 ★★
#   [0xCE] [1 字节]   0 = 正常播放（默认），非 0 = 跟谱演奏
#   跟谱演奏的含义：当前时间格要求的键【全部按住】才前进到下一格
#   （要松开再按才算下一格；休止格自动通过）。
#
#   ★ 和 0xC9/0xCA/0xCB 同类：不受开窗限制、随时可用、可插队。
#   ★ 板子上的 TTP 按键13 也能翻转同一个状态；
#     模式状态由 0x5D 状态帧的【第 4 个字节】回传（见 fpga_rec.py）。
#   ★ 0xCE 不会撞车：现有命令用到 0xC0~0xCD，0xCE/0xCF 是空的。
MODE_SET_BYTE = 0xCE
#   0x5D 状态帧的第 4 个字节就是模式值（0/1），所以这两个值必须一致
MODE_OFF = 0
MODE_ON = 1

# ★★ V44：节拍器（板载 READY 灯 U12 按 BPM 匀速闪）★★
#   [0xCF] [BPM] [NUM] [DEN]      上位机 -> 板子：设置节拍器（操作数 3 字节）
#     BPM : 1 字节，30~240；★ 0 = 关闭节拍器
#           （1~29 板子会夹到 30，>240 夹到 240；上位机发之前就先夹好，见
#             clamp_metro_bpm —— 界面显示的值 = 发出去的值 = 板子回报的值）
#     NUM : 1 字节，拍号分子 1~8（超出夹紧）
#     DEN : 1 字节，拍号分母的【编码】：0 -> 2，1 -> 4，2 -> 8（超出夹紧到 0~2）
#           ★ 线上传的是【编码】不是分母本身：分母只有 2/4/8 三种可能，
#             编码更省字节，也免得"分母 = 0"这种非法值出现在线上。
#
#   ★ 节拍器【不参与任何其它模块】，纯粹是外部参考：它不碰乐谱、不碰效果、
#     不碰演奏状态 —— 所以它和 0xC9/0xCA/0xCB/0xCE 同类：不受开窗限制、
#     随时可用、可以插队发（夹在谱子字节中间也不会被板子当成音符）。
#   ★ 0xCF 不会撞车：现有命令用到 0xC0~0xCE，0xCF 是最后一个空位。
METRO_SET_BYTE = 0xCF
#   板子 -> 上位机：[0x5E] [BPM] [NUM] [DEN] [BEAT]（整帧 5 字节）
#     BPM/NUM/DEN = 板子当前【实际生效】的值（夹紧之后的，不是我们发过去的）
#     BEAT        = 小节内第几拍，0 起算（0 .. NUM-1）
#   ★ 板子在【设置发生变化时】主动发这一帧，不是周期发送 —— 收到它就说明
#     刚才那次设置已经生效，界面可以拿它把控件同步成板子的真实状态。
#   ★ 这是一个【全新独立】的状态帧，和 0x5D（效果/音色/模式，4 字节）无关：
#     0x5D 一个字节都没动，两端各解析各的。
METRO_STAT_BYTE = 0x5E

# 节拍器取值范围 / 出厂默认（板子出厂：BPM=100、4/4、节拍器开着）
METRO_BPM_MIN = 30
METRO_BPM_MAX = 240
METRO_BPM_OFF = 0            # ★ BPM = 0 是【关闭】节拍器，不是"最慢"
METRO_NUM_MIN = 1
METRO_NUM_MAX = 8
METRO_DEN_MIN = 0            # 分母编码 0 -> 2 分音符
METRO_DEN_MAX = 2            # 分母编码 2 -> 8 分音符
METRO_DEN_CODES = (2, 4, 8)  # ★ 下标 = 编码，值 = 真实分母
DEFAULT_METRO_BPM = 100
DEFAULT_METRO_NUM = 4
DEFAULT_METRO_DEN = 1        # ★ 是【编码】：1 -> 4 分音符，所以默认就是 4/4

# ★★ V46：LCD 显示模式（双向同步）★★
#   [0xD0] [DISP_MODE]   上位机 -> 板子：直接【设定】LCD 显示模式（操作数 1 字节）
#     DISP_MODE : 0 = 频谱，1 = 波形，2 = 李萨如
#                 ★ 超出范围【夹到 0~2】（不是循环、也不是取模）：
#                   取模的话发 3 会变成 0（看着像"按了没反应"），
#                   而夹紧的语义是"你要的那个没有，给你最接近的一头"。
#
#   ★ 语义是【直接设】不是"循环切换"（和音色 0xCA 同类）：
#     板子上按 TTP 按钮 3 是循环切换，上位机这边是明确的三选一，
#     直接设才能保证"我选李萨如 = 板子就到李萨如"，不会因为两端
#     一时不同步而设反。
#
#   板子 -> 上位机：[0x5F] [DISP_MODE] [REC_ON]（整帧 3 字节）
#     DISP_MODE = 板子当前【实际生效】的显示模式（0/1/2）
#     REC_ON    = 板子当前的录音开关状态（0 = 关，★ 非 0 = 开）
#   ★★ 板子在【这两个值变化时】主动发这一帧 —— 包括用户在板子上按
#      TTP 按钮 3（切显示）或按钮 10（录音开关）的时候。
#      这就是"双向同步"的关键：在板子上按按钮，上位机也能知道。
#   ★ 0xD0 不会撞车：现有命令用到 0x5C/0xAA/0xC0~0xC3/0xC4~0xC8/
#     0xCA~0xCF/0xEE/0xFF，0xD0 是空的。
#   ★ 0x5F 不会撞车：现有回传帧头是 0x5A/0x5B/0x5D/0x5E，0x5F 是空的。
DISP_SET_BYTE = 0xD0
DISP_STAT_BYTE = 0x5F
DISP_MIN = 0                # ★ 频谱
DISP_MAX = 2                # ★ 李萨如
DISP_NAMES = ("频谱", "波形", "李萨如")

# ★★ V45：逐事件力度（velocity）★★
#   [0xC3] [VEL]      上位机 -> 板子：设置【紧接着的下一个事件】的力度（操作数 1 字节）
#     VEL : 1 字节，0~127；★ 127 = 满幅（= 出厂默认）
#
#   ★ 语义和 0xC2 时值延长同一类：都是"挂着等下一个事件"的 pending 机制 ——
#     0xC3 本身不是一个事件，它只是给后面那一个事件挂一个参数。
#   ★★ 没有 0xC3 前缀时，板子按 127 处理 ★★
#     所以【不发 0xC3 的帧和加这个功能之前逐字节相同】—— 手工曲谱（.jp.txt）
#     那种没有力度信息的来源就一个字节都不发，行为一点不变。
#   ★ 力度是【逐事件】的，不是逐音的：一个和弦（同一格的多个音）只带一个 VEL。
#     取法见 fpga_midi.midi_to_frame —— 同一格里取各音 velocity 的【最大值】。
#   ★ 0xC3 不会撞车：现有命令用到 0x5C/0xAA/0xC0~0xC2/0xC4~0xC8/0xCA~0xCF/
#     0xEE/0xFF，0xC3 是空的（正好夹在 0xC2 和 0xC4 中间）。
VEL_BYTE = 0xC3
VEL_MIN = 0
VEL_MAX = 127
VEL_DEFAULT = 127            # ★ 满幅；也是"没有力度信息"时板子的行为

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


# ==========================================================================
# ★ V39.1：按【协议单元边界】切块（给"整帧分块投递"用）
# ==========================================================================
#   为什么需要它
#   ────────────
#   上位机把整帧按固定块长（256 字节）丢进发送队列，而控制命令
#   （0xC9/0xCA/0xCB）是【插队】发出去的 —— 插到"下一块"之前。
#   如果块的边界正好落在【一个命令字和它的操作数之间】：
#
#        ... C4 | (插队) C9 04 | 00 E8 01 ...
#
#   板子收到 C4 之后进入"等 3 个参数"，接着把插队的 C9、04 和下一块开头的
#   00 当成三个参数【吃掉】—— 参数被污染（时间格被设成错值），而 E8 01
#   变成游离字节，整帧从那里开始错位。
#   ★ 这比"插队插在帧中间"更糟：那个只是错位，这个还会污染参数。
#
#   ★★ 反过来：只要切点落在【单元边界】上，插队就完全无害 ——
#      板子收到的是 "…单元A | C9 04 | 单元B…"，两个都是完整单元，
#      命令被正常执行，帧一个字节都不丢。
#   ★ 这也是为什么这个毛病一直是"偶发"的：设置类命令只在帧头出现一次，
#     其余操作数大多只有 1 字节，固定块长切中"命令+操作数"之间
#     每帧大约只有 1% 的机会。
# ==========================================================================

#   每个命令字的【操作数长度】。不在表里的一律当"无操作数"（1 字节单元）。
_OP_LEN = {
    0x5C: 0,     # 录音开关
    0xAA: 0,     # 开窗 + 写指针归零 + 停播
    CHORD_ON: 0,     # 0xC0 和弦头（单独处理，见下）
    CHORD_OFF: 0,    # 0xC1 和弦尾
    DUR_EXT: 1,      # 0xC2 时值延长 + N
    # ★ V45：力度 + 1 字节 VEL
    #   ★ 必须登记在这里 —— 否则 safe_chunk_ranges 分块时可能把
    #     "0xC3 + 它的操作数"从中间切开，插队命令正好落进那一对中间，
    #     板子就会把插队字节当成力度吃掉（坑 21：参数被污染 + 整帧错位）。
    VEL_BYTE: 1,
    SET_TICK: 3,     # 0xC4 时间格 + 3 字节 µs(LE)
    SET_REL: 3,      # 0xC5 释音提前量 + 3 字节 µs(LE)
    0xC6: 2,     # 按键映射 + 键号 + 音符
    0xC7: 0,     # 停止演奏
    0xC8: 0,     # 重播
    OCT_BYTE: 1,     # 0xCC 整格八度 + 偏移
    KEY_OCT_BYTE: 1, # 0xCD 逐音八度 + 偏移
    0xCA: 1,     # 音色 + 档位
    0xCB: 0,     # 查询状态
    # ★ V43：跟谱演奏模式 + 1 字节（0 = 正常，非 0 = 跟谱）
    #   ★ 必须登记在这里 —— 否则 safe_chunk_ranges 分块时可能把
    #     "0xCE + 它的操作数"从中间切开，插队命令正好落进那一对中间，
    #     板子就会把插队字节当成操作数吃掉（坑 21，污染参数 + 整帧错位）。
    MODE_SET_BYTE: 1,
    # ★ V44：节拍器设置 + 它的 3 个字节操作数（BPM/NUM/DEN）
    #   ★ 同样【必须登记】—— 理由和上面 0xCE 那条一模一样（坑 21）：
    #     节拍器命令也是插队发的，不登记的话 safe_chunk_ranges 可能把
    #     "0xCF + BPM NUM DEN"从中间切开，插队命令落进那一对中间，
    #     板子就会把插队字节当成操作数吃掉（参数被污染 + 整帧错位）。
    METRO_SET_BYTE: 3,
    # ★ V46：LCD 显示模式 + 1 字节 DISP_MODE（0/1/2）
    #   ★ 必须登记在这里 —— 否则 safe_chunk_ranges 分块时可能把
    #     "0xD0 + 它的操作数"从中间切开，插队命令正好落进那一对中间，
    #     板子就会把插队字节当成显示模式吃掉（坑 21：参数被污染 + 整帧错位）。
    #     显示模式命令和节拍器/音色一样是【插队发】的，所以这条不是"以防万一"。
    DISP_SET_BYTE: 1,
    REST_BYTE: 0,    # 0xEE 休止
    END_BYTE: 0,     # 0xFF 帧尾
}


def unit_spans(blob: bytes):
    """扫出协议单元的 [start, end) 边界（生成器）。

    ★ 一个"单元"= 板子接收状态机里【不可分割】的一小段：
        单音 / 休止 / 整个和弦（0xC0..0xC1）/ 命令字 + 它全部的字节操作数。
      切块时只能切在单元之间 —— 切进单元内部就会让板子的
      "等操作数"计数器与字节流错位。
    """
    i = 0
    n = len(blob)
    while i < n:
        b = blob[i]
        if b == CHORD_ON:
            # 和弦：整段（含结尾的 0xC1）算一个单元
            j = i + 1
            while j < n and blob[j] != CHORD_OFF:
                j += 1
            if j < n:
                j += 1                      # 吃掉 0xC1
            yield (i, j)
            i = j
        else:
            step = 1 + _OP_LEN.get(b, 0)
            yield (i, min(i + step, n))
            i += step


def safe_chunk_ranges(blob: bytes, chunk: int):
    """把 blob 切成不超过 chunk 的块，但【切点只落在协议单元边界上】。

    返回 [(start, end), ...]；blob 为空返回 []。
    ★ 单个单元如果本身就超过 chunk，会被整块放出去（宁可超一点，
      也不能把它切开）—— 实际最大单元是 36 音的和弦 = 38 字节，远小于 256。
    """
    if chunk <= 0 or not blob:
        return []
    out = []
    start = 0
    acc = 0
    for (s, e) in unit_spans(blob):
        size = e - s
        if acc and (acc + size > chunk):
            out.append((start, s))
            start = s
            acc = 0
        acc += size
    if start < len(blob):
        out.append((start, len(blob)))
    return out


def key_name(k: int) -> str:
    return KEY_NAMES[k] if 0 <= k < N_KEYS else "?"


# ==========================================================================
# ★ V44：节拍器 —— 参数夹紧 / 组包 / 名字
# ==========================================================================
#   板子的 READY 灯（引脚 U12）原来是"音色指示"灯，现在改成节拍器：
#   按 BPM 匀速闪，每拍闪得一样长（★ 不区分强拍弱拍 —— 用户明确要求"每拍
#   闪得一样长"，所以这里没有任何"第一拍闪久一点"的参数）。
#
#   ★★ 为什么夹紧一定要放在【上位机】这一侧 ★★
#     板子收到非法的 BPM/NUM/DEN 会自己夹紧，但它回报的 0x5E 里放的是
#     【夹紧之后】的值。上位机要是不先夹，就会出现这样的来回拉扯：
#       用户填 999 -> 界面显示 999 -> 板子回报 240 -> 界面被刷成 240
#       -> 用户以为控件坏了，再改一次又发 999……
#     所以发之前先夹好：界面显示的值 = 真正发出去的值 = 板子回报的值。
#     （这些函数同时也是 fpga_rec.pack_metro_set 的底座，只有一份实现。）
def clamp_metro_bpm(bpm) -> int:
    """BPM 夹到【0（关）】或者【30~240】。

    ★ 0 是"关闭"而不是"最慢"，所以不能简单地对 30 取下界：
      写成 max(30, min(240, v)) 的话，关闭请求（发 0）会被夹成 30 = 又开起来了
      —— 这正是"关了关不掉"的那类 bug，单独在这里钉一句。
    """
    try:
        v = int(bpm)
    except (TypeError, ValueError):
        return DEFAULT_METRO_BPM       # 空/非数字 -> 出厂默认，别往线上发脏值
    if v <= METRO_BPM_OFF:
        return METRO_BPM_OFF           # 0 和负数都算"关闭节拍器"
    return max(METRO_BPM_MIN, min(METRO_BPM_MAX, v))


def clamp_metro_num(num) -> int:
    """拍号分子夹到 1~8。"""
    try:
        v = int(num)
    except (TypeError, ValueError):
        return DEFAULT_METRO_NUM
    return max(METRO_NUM_MIN, min(METRO_NUM_MAX, v))


def clamp_metro_den(den_code) -> int:
    """拍号分母的【编码】夹到 0~2（= 分母 2/4/8）。

    ★ 参数是编码不是分母本身：传 4 会得到 2（= 8 分音符），因为编码 2 才对应
      分母 8。要拿"人写的分母"来调用，先过 metro_den_code()。
    """
    try:
        v = int(den_code)
    except (TypeError, ValueError):
        return DEFAULT_METRO_DEN
    return max(METRO_DEN_MIN, min(METRO_DEN_MAX, v))


def metro_den_value(den_code) -> int:
    """分母【编码】 -> 真实分母 2 / 4 / 8。"""
    return METRO_DEN_CODES[clamp_metro_den(den_code)]


def metro_den_code(den_value) -> int:
    """真实分母 2 / 4 / 8 -> 编码 0 / 1 / 2。

    ★ 界面上是给用户选"2 / 4 / 8"的，而线上要发编码，所以这个反向换算
      放在协议层（而不是在界面代码里各写一份）。
      不是这三个值时取【最接近】的一个，并且夹在合法范围内：
      None / 非法值 -> 默认编码（= 4 分音符），不会抛异常打断界面。
    """
    try:
        v = int(den_value)
    except (TypeError, ValueError):
        return DEFAULT_METRO_DEN
    best = 0
    for i, dv in enumerate(METRO_DEN_CODES):
        if abs(dv - v) < abs(METRO_DEN_CODES[best] - v):
            best = i
    return best


def pack_metro_set(bpm, num, den_code) -> bytes:
    """上位机 -> 板子：设置节拍器（[0xCF] [BPM] [NUM] [DEN]）。

    ★ 返回值就是真正要写进串口的 4 个字节，一个不多一个不少。
    ★ 三个参数都会先夹紧，所以调用方可以放心地把控件里的原始值直接丢进来：
        pack_metro_set(100, 4, 1) == b'\\xcf\\x64\\x04\\x01'   （= 100 BPM，4/4）
        pack_metro_set(0, 4, 1)   == b'\\xcf\\x00\\x04\\x01'   （= 关闭节拍器）
    ★★ V48.1：BPM 也要过 safe_operand ★★
      这一格特别容易中招：BPM 的合法范围 30~240 里就含 5 个帧头字节
      （90/91/93/94/95），而且板子【紧接着就会回一帧 0x5E】——
      假帧会被那一帧"确认"下来，于是界面显示出一个板子根本没开的效果。
      选 93 会按 92 发出去（差 1 BPM，节拍器上完全听不出来），
      板子回报的也是 92 —— 界面以板子为准，这是刻意的。
    """
    return bytes([METRO_SET_BYTE,
                  # ★ 下界取 0（不是 METRO_BPM_MIN=30）：0 = 关，是合法值，
                  #   而且 0 不是帧头，safe_operand 会原样放行。
                  safe_operand(clamp_metro_bpm(bpm), 0, METRO_BPM_MAX),
                  clamp_metro_num(num), clamp_metro_den(den_code)])


def metro_name(bpm, num, den_code) -> str:
    """节拍器设置 -> 一行中文（界面和日志里显示用）。"""
    b = clamp_metro_bpm(bpm)
    if b == METRO_BPM_OFF:
        return "节拍器 关"
    return "节拍器 %d BPM %d/%d" % (b, clamp_metro_num(num),
                                   metro_den_value(den_code))


# ==========================================================================
# ★ V45：逐事件力度 —— 参数夹紧 / 组包 / 名字
# ==========================================================================
#   ★ 和节拍器一样，夹紧放在【上位机】这一侧：界面显示的值 = 发出去的值
#     = 板子实际用的值，不会出现"填 200 -> 板子按 127 响"这种两头对不上。
def clamp_vel(v) -> int:
    """力度夹到 0~127。

    ★ 这里【没有 BPM 那种"0 = 关闭"的特例】：0 就是"最小声"，
      而"不发 0xC3"才表示"用默认 127" —— 两件事不要混。
    ★ 空值 / 非数字 -> VEL_DEFAULT(127)：力度是锦上添花的参数，拿不到就当
      "和以前一样"，绝不为了一个脏值把整首曲子的转换打断。
    """
    try:
        n = int(v)
    except (TypeError, ValueError):
        return VEL_DEFAULT
    return max(VEL_MIN, min(VEL_MAX, n))


def pack_vel(v) -> bytes:
    """上位机 -> 板子：设置【下一个事件】的力度（[0xC3] [VEL]）。

    ★ 返回值就是真正要写进串口的 2 个字节，一个不多一个不少。
    ★ 参数先夹紧，所以调用方可以把 MIDI 里的原始 velocity 直接丢进来：
        pack_vel(100) == b'\\xc3\\x64'
        pack_vel(200) == b'\\xc3\\x7f'      （夹到 127）
    ★★ V48.1：力度还要过一道 safe_operand —— 见下面"回显安全"那一大段。
      93(0x5D) 这种值会被微调成 92，听感上完全一样，但不会再把上位机骗了。
    """
    return bytes([VEL_BYTE, safe_operand(clamp_vel(v), VEL_MIN, VEL_MAX)])


# ==========================================================================
# ★★ V48.1：回显安全 —— 操作数里不许出现【会被当成状态帧头】的字节 ★★
# ==========================================================================
#   板子会把收到的字节【原样回显】给上位机（协议 v1 就有的行为，只有 0xAA 不回显）。
#   而上位机要在这条混合流里找【板子主动发的状态帧】—— 帧头就是这 5 个。
#
#   于是有一个天生的危险：**只要上位机【发出去】的操作数里出现了帧头字节，
#   它的回显就会被上位机当成一帧真状态。** 而谱子里有几类操作数是【任意值】：
#       · 0xC3 的力度      0~127   ← 每格事件一个，最常见
#       · 0xC2 的时值      0~255
#       · 0xC4/0xC5 的时间参数 3 字节
#
#   ★★ 真实事故（用户 2026-10-11 实测报的）：
#      "每次发谱，软件自己就把【混响和合唱】显示成打开，音色有时还变成【笛子】；
#       点一下「刷新状态」就正常了。"
#      根因：力度 0x5D(=93) 正好撞上帧头，紧接着是和弦头 0xC0：
#          C3 5D C0 01 ...   ->  fx=0xC0（混响+合唱）、tone=0x01（笛子）
#      本机 MIDI 目录 12 首里有 10 首会命中（93 是很常见的力度）。
#
#   ★ 治法（两道一起上，缺一不可）：
#     ① **发送侧**：safe_operand / safe_u24 把操作数微调成不含帧头的值 ——
#        力度差 1/127、时间参数差几 µs、时值差 1 格，都是听不出来的量级；
#     ② **接收侧**：fpga_rec 里 0x5D 改成"候选帧"，要两个字节都合规才提交
#        （防的是【老固件】和【别的主机】发的帧，那一侧我们改不了）。
#   ★ 这两道是【互补】的：① 管我们自己发出去的，② 管别人发进来、被回显的。
#   ★ 判据见 `test_echo_collision.py`（穷举全部操作数取值 + 全部真实谱子）。
FRAME_HEADS = (0x5A, 0x5B, 0x5D, 0x5E, 0x5F)     # 与 fpga_rec 的 5 个帧头一一对应


def safe_operand(v: int, lo: int = 0, hi: int = 255) -> int:
    """把操作数微调成【不是帧头字节】的值（在 lo..hi 里尽量少动）。

    ★ 先往下调、下不去再往上：这样子"看着更像原来的值"
      （93 -> 92）。
    ★★ 必须【一圈一圈往外找】，±1 是不够的 —— 帧头是连在一起的：
          90(0x5A) 91(0x5B) 93(0x5D) 94(0x5E) 95(0x5F)
      所以 94 的左右邻居 93/95 也全是帧头，只试 ±1 会原地返回 94（踩过）。
      （这正是 `test_echo_collision.py` 里"0~127 全扫"那一条抓出来的。）
    """
    v = max(lo, min(hi, int(v)))
    if v not in FRAME_HEADS:
        return v
    for d in range(1, 16):
        for cand in (v - d, v + d):
            if lo <= cand <= hi and cand not in FRAME_HEADS:
                return cand
    return v                      # 理论上到不了这儿


def safe_u24(value: int) -> bytes:
    """3 字节小端，且【三个字节里一个帧头都没有】。

    ★ 时间参数的单位是 µs（一整格几百 ms），差几 µs 完全无感。
    """
    v = max(0, min(0xFFFFFF, int(value)))
    for d in range(0, 256):
        cands = (v,) if d == 0 else (v - d, v + d)
        for cand in cands:
            if not (0 <= cand <= 0xFFFFFF):
                continue
            b = _u24le(cand)
            if all(x not in FRAME_HEADS for x in b):
                return b
    return _u24le(v)              # 理论上到不了这儿


def vel_name(v) -> str:
    """力度 -> 一行中文（日志/报告里显示用）。★ 127 说明"没发 0xC3"。"""
    n = clamp_vel(v)
    if n == VEL_DEFAULT:
        return "力度 %d（满幅，不发 0xC3）" % n
    return "力度 %d" % n


# ==========================================================================
# ★ V46：LCD 显示模式 —— 参数夹紧 / 组包 / 名字
# ==========================================================================
#   ★ 和节拍器/力度一样，夹紧放在【上位机】这一侧：界面显示的值 = 发出去的值
#     = 板子回报的值，不会出现"选了李萨如、板子却在波形"这种两头对不上。
#   ★ 这里【没有 BPM 那种"0 = 关闭"的特例】：0 就是"频谱"这个真实档位，
#     三个档位地位一样（见 DISP_MIN/DISP_MAX）。
def clamp_disp(v) -> int:
    """显示模式夹到 0~2（★ 夹紧，不是取模）。

    ★ 为什么不是取模：发 3 取模会变成 0（频谱）—— 用户看到的是"选了没反应、
      还跳回频谱"；夹紧则是"3 -> 2（李萨如）"，语义是"没有第 4 档，给你最
      接近的一头"。协议里明确定了是【夹到 0~2】。
    ★ 空值 / 非数字 -> DISP_MIN(0)：显示模式取不到就当"频谱"（出厂默认），
      绝不为了一个脏值抛异常打断界面。
    """
    try:
        n = int(v)
    except (TypeError, ValueError):
        return DISP_MIN
    return max(DISP_MIN, min(DISP_MAX, n))


def pack_disp_set(v) -> bytes:
    """上位机 -> 板子：直接设定 LCD 显示模式（[0xD0] [DISP_MODE]）。

    ★ 返回值就是真正要写进串口的 2 个字节，一个不多一个不少。
    ★ 参数先夹紧，所以调用方可以直接把控件里的原始值丢进来：
        pack_disp_set(0) == b'\\xd0\\x00'      （频谱）
        pack_disp_set(1) == b'\\xd0\\x01'      （波形）
        pack_disp_set(2) == b'\\xd0\\x02'      （李萨如）
        pack_disp_set(5) == b'\\xd0\\x02'      （夹到 2）
    """
    return bytes([DISP_SET_BYTE, clamp_disp(v)])


def disp_name(v) -> str:
    """显示模式 -> 中文名（界面和日志里显示用）。

    ★ None / 越界都算「未知」—— 不能把"板子还没报过"画成"频谱"。
    """
    if v is None:
        return "未知"
    try:
        n = int(v)
    except (TypeError, ValueError):
        return "未知"
    if not (DISP_MIN <= n <= DISP_MAX):
        return "未知"
    return DISP_NAMES[n]


# ==========================================================================
# 事件与帧
# ==========================================================================
@dataclass
class Event:
    """一个时间格事件：同时发声的一组键（空 = 休止） + 时长（格）+ 八度偏移。

    这是协议层的**唯一事件模型**，与 top.v 的 midi_keys 掩码一一对应。

    ★ oct（V32）：这一格整体的八度偏移，有符号，-4..+3。0 = 原调。
      板子只有 36 个声部（A2..G#5），超出这个范围的音原来一律折八度进来；
      有了它，上位机可以让某一格【整体升/降 n 个八度】，
      于是"高两个八度的那句旋律"能按原八度弹出来。
      编码成 [0xCC][off & 0x07]（两补数），放在这一格的音符前面。
      ★ oct == 0 时【一个字节都不发】—— 所以原来的谱子编出来逐字节不变。
    """
    keys: Tuple[int, ...] = ()
    ticks: int = 1
    oct: int = 0
    # ★ V37：逐键八度覆盖 {键号: 八度}。不在里面的键用上面的 oct（整格八度）。
    #   归一化规则（__post_init__ 末尾做）：
    #     · 单音事件：逐键八度等价于整格八度，直接提升成 oct（编码更短）
    #     · 与本事件 oct 相同的项没有信息量，删掉 —— 保证"无覆盖"只有一种写法
    key_octs: Dict[int, int] = field(default_factory=dict)
    # ★ V45：这一格的力度（0~127，超出夹紧）。
    #   ★ 默认 VEL_DEFAULT(127) = 【不发 0xC3】—— 板子没有 0xC3 就是按 127 处理，
    #     所以不填这个字段的旧代码 / 手工曲谱编出来【逐字节不变】。
    vel: int = VEL_DEFAULT

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
        self.oct = int(self.oct)
        if not OCT_MIN <= self.oct <= OCT_MAX:
            raise ProtoError("八度偏移 %r 超出 %d..%+d" % (self.oct, OCT_MIN, OCT_MAX))
        # ★ V45：力度一律夹到 0~127（夹紧规则只有 fpga_proto.clamp_vel 一份实现）
        self.vel = clamp_vel(self.vel)
        # ---- ★ V37：逐键八度归一化 ----
        ko: Dict[int, int] = {}
        for k, v in dict(self.key_octs or {}).items():
            k, v = int(k), int(v)
            if k not in self.keys:
                raise ProtoError("逐键八度指定的键号 %d 不在本事件的 keys %r 里"
                                 % (k, self.keys))
            if not OCT_MIN <= v <= OCT_MAX:
                raise ProtoError("键 %d 的八度 %r 超出 %d..%+d"
                                 % (k, v, OCT_MIN, OCT_MAX))
            ko[k] = v
        if len(self.keys) == 1 and ko:
            # 单音：没有"逐键 vs 整格"的区别，合并成一个 oct
            self.oct = ko[self.keys[0]]
            ko = {}
        self.key_octs = {k: v for k, v in sorted(ko.items()) if v != self.oct}

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
        tail = "" if self.oct == 0 else ", 八度%+d" % self.oct
        # ★ V45：只有"真的要发 0xC3"的时候才把力度显示出来（默认 127 不显示，
        #   这样老代码打印出来的 Event 一个字都不变）
        if self.vel != VEL_DEFAULT:
            tail += ", 力度%d" % self.vel
        # ★ V37：逐键八度显示成 "键名±n"，只列与整格八度不同的那些
        if self.key_octs:
            tail += " [" + " ".join("%s%+d" % (key_name(k), v)
                                    for k, v in sorted(self.key_octs.items())) + "]"
        return "Event(%s, %d格%s)" % (body, self.ticks, tail)


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
        # ★ V48.1：时间参数也不能带帧头字节（见"回显安全"那一大段）
        out += bytes([SET_TICK]) + safe_u24(tick_us)
    if rel_us != DEFAULT_REL_US:
        out += bytes([SET_REL]) + safe_u24(rel_us)
    return bytes(out)


def _encode_event(e: Event) -> bytes:
    """单个事件 -> 字节（不含前导 / 帧尾）"""
    out = bytearray()
    # ★ V45：力度必须发在【这一格事件的最前面】—— 和 0xCC/0xCD 一样，板子是
    #   "收到 0xC3 就挂到下一个事件上"，发在音符后面就挂到下一格去了。
    #   ★★ vel == VEL_DEFAULT(127) 时【一个字节都不发】★★：板子没有 0xC3 就是
    #     按 127 处理，所以"没有力度信息的来源"（手工曲谱）和改造前逐字节相同。
    if e.vel != VEL_DEFAULT:
        out += pack_vel(e.vel)
    # ★ V32：八度偏移必须发在【这一格的音符前面】—— 板子是"收到 0xCC 就挂到
    #   下一个事件上"，音符后面再发就挂到下一格去了。
    #   ★ oct == 0 时一个字节都不发，所以原调谱子逐字节不变。
    if e.oct:
        out.append(OCT_BYTE)
        out.append(e.oct & 0x07)
    if e.is_rest:
        out.append(REST_BYTE)
    elif e.is_chord:
        out.append(CHORD_ON)
        # ★ V37：带逐键八度的音，在它【前面】发一个 [0xCD][八度] ——
        #   板子是"收到 0xCD 就把它挂给紧跟着的那一个音号"。
        #   ★ 一个逐键八度都没有时，这里与 V36 逐字节相同。
        for k in e.keys:
            if k in e.key_octs:
                out.append(KEY_OCT_BYTE)
                out.append(e.key_octs[k] & 0x07)
            out.append(k)
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
        # ★ V48.1：时值操作数同样不许是帧头字节（差 1 格完全无感；
        #   而且真会撞上的只有 92~96 格那种超长事件，本来就极罕见）
        out.append(safe_operand(left, 0, 255))
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

    ★ V45：键集合相同但【力度不同】的相邻事件绝不合并 —— 并了就等于把后面
      那一格的力度改掉了（力度变化正是要靠这些格子表达的）。
    """
    out: List[Event] = []
    for e in frame.events:
        if (out and out[-1].keys == e.keys and out[-1].vel == e.vel
                and out[-1].ticks + e.ticks <= MAX_TICKS):
            out[-1].ticks += e.ticks
        else:
            # ★ V45：新事件带上本次的力度（别的字段照旧，一个都没动）
            out.append(Event(keys=e.keys, ticks=e.ticks, vel=e.vel))
    return Frame(events=out, tick_us=frame.tick_us, rel_us=frame.rel_us)


# ==========================================================================
# 解码（严格镜像 FPGA 接收器/播放器的解析规则）
# ==========================================================================
def decode_frame(data: bytes) -> Frame:
    """完整帧字节 -> Frame。

    与 FPGA 侧的两条规则严格对应：
      1. 只有 0xAA 之后、0xFF 之前的字节才是"事件流"（armed 窗口）
      2. 0xC2 / 0xC3 / 0xC4 / 0xC5 后面的操作数是【原始数据】，不受控制字节解释
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
    pend_oct = 0                      # ★ V32：0xCC 给的八度偏移，作用于下一格
    pend_vel = VEL_DEFAULT            # ★ V45：0xC3 给的力度，作用于下一格

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

        # ---- ★ V32：0xCC 八度偏移（作用于它后面那一格事件）----
        if b == OCT_BYTE:
            if i + 1 >= len(data):
                raise ProtoError("0xCC 的操作数被截断 @%d" % i)
            raw = data[i + 1] & 0x07
            pend_oct = raw - 8 if raw >= 4 else raw          # 两补数 -> 有符号
            i += 2
            continue

        # ---- ★ V45：0xC3 力度（作用于它后面那一格事件）----
        #   ★ 操作数是【原始数据】：它可能是任何 0..127 的值，包括 0x00..0x23
        #     （看着像键号）、0x5C 之类 —— 所以必须按"命令字 + 1 字节"整对吃掉，
        #     绝不能让它落到下面的"事件"分支里去被当成音符。
        if b == VEL_BYTE:
            if i + 1 >= len(data):
                raise ProtoError("0xC3 的操作数被截断 @%d" % i)
            pend_vel = clamp_vel(data[i + 1])
            i += 2
            continue

        # ---- 事件 ----
        keys: Tuple[int, ...] = ()
        key_octs_ev: Dict[int, int] = {}          # ★ V37：逐键八度
        if b <= NOTE_MAX:
            keys = (b,)
            i += 1
        elif b == REST_BYTE:
            keys = ()
            i += 1
        elif b == CHORD_ON:
            i += 1
            acc: List[int] = []
            acc_ko: Dict[int, int] = {}      # ★ V37：和弦内逐键八度
            pend_key_oct = None              # ★ V37：0xCD 给的，等下一个音号
            while i < len(data) and data[i] != CHORD_OFF:
                bb = data[i]
                if bb <= NOTE_MAX:
                    acc.append(bb)
                    if pend_key_oct is not None:
                        acc_ko[bb] = pend_key_oct
                        pend_key_oct = None
                    i += 1
                elif bb == KEY_OCT_BYTE and i + 1 < len(data):
                    raw = data[i + 1] & 0x07     # ★ V37：只作用一个音号
                    pend_key_oct = raw - 8 if raw >= 4 else raw
                    i += 2
                elif bb == REST_BYTE:
                    i += 1                      # 和弦内休止：忽略
                elif bb == OCT_BYTE and i + 1 < len(data):
                    raw = data[i + 1] & 0x07    # ★ V32：和弦里也能带八度偏移
                    pend_oct = raw - 8 if raw >= 4 else raw
                    i += 2
                elif bb == DUR_EXT:
                    i += 2                      # 畸形：跳过操作数
                elif bb == VEL_BYTE:
                    i += 2                      # ★ V45：畸形（力度写在和弦里）：跳过操作数
                elif bb in (END_BYTE,):
                    break                       # 未闭合，就用已收集的
                elif bb in (SET_TICK, SET_REL):
                    break
                else:
                    i += 1
            if i < len(data) and data[i] == CHORD_OFF:
                i += 1
            keys = tuple(sorted(set(acc)))
            key_octs_ev = {k: v for k, v in acc_ko.items() if k in keys}
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

        events.append(Event(keys=keys, ticks=ticks, oct=pend_oct,
                            key_octs=key_octs_ev, vel=pend_vel))
        pend_oct = 0                  # ★ V32：只作用一格
        pend_vel = VEL_DEFAULT        # ★ V45：只作用一格（下一格回到默认 127）

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
    #   ★★ V48.1：有【唯一一个】例外 —— 落进帧头字节的那几个操作数值，
    #      编码器会【刻意 ±1】避开"回显撞帧头"（见 safe_operand 那一大段）：
    #        力度 90/91/93/94/95、时值 N 92~96、时间参数里含这些字节的 µs 值。
    #      所以往返比对要按【消毒后的值】比。这是设计，不是 bug。
    def _sanitized(fr):
        evs = []
        for e in fr.events:
            ticks = e.ticks
            if ticks > 1:
                ticks = safe_operand(ticks - 1, 0, 255) + 1
            evs.append(Event(e.keys, ticks,
                             vel=safe_operand(e.vel, VEL_MIN, VEL_MAX)))
        return (evs,
                int.from_bytes(safe_u24(fr.tick_us), "little"),
                int.from_bytes(safe_u24(fr.rel_us), "little"))

    rng = random.Random(20261006)
    good = True
    for trial in range(400):
        n = rng.randint(0, 40)
        evs = []
        for _ in range(n):
            kind = rng.random()
            # ★ V45：力度也随机（大部分是默认 127 = 不发 0xC3，少数是别的值）
            vel = rng.choice([VEL_DEFAULT, VEL_DEFAULT, VEL_DEFAULT,
                              rng.randrange(0, 128)])
            if kind < 0.25:
                evs.append(Event((), rng.randint(1, 40)))          # 休止
            elif kind < 0.70:
                evs.append(Event((rng.randrange(36),), rng.randint(1, 40), vel=vel))
            else:
                k = rng.sample(range(36), rng.randint(2, 6))
                evs.append(Event(tuple(k), rng.randint(1, 40), vel=vel))
        fr = Frame(events=evs,
                   tick_us=rng.choice([DEFAULT_TICK_US, 83_333, 1_000_000, 20_000]),
                   rel_us=rng.choice([DEFAULT_REL_US, 10_000, 0]))
        want_evs, want_tick, want_rel = _sanitized(fr)
        d = decode_frame(encode_frame(fr))
        if ([e.keys for e in d.events] != [e.keys for e in want_evs]
                or [e.ticks for e in d.events] != [e.ticks for e in want_evs]
                or [e.vel for e in d.events] != [e.vel for e in want_evs]
                or d.tick_us != want_tick or d.rel_us != want_rel):
            good = False
            print("     第 %d 次失败" % trial)
            print("     原始: %r" % fr)
            print("     期望: %r" % want_evs)
            print("     还原: %r" % d)
            break
    check("400 次随机帧往返一致（含随机力度；按消毒后的值比）", good)
    #   ★ 单独钉一条：消毒只动"撞帧头"的那几个值，别的一个都不许动
    _same = all(safe_operand(v, VEL_MIN, VEL_MAX) == v
                for v in range(VEL_MIN, VEL_MAX + 1) if v not in FRAME_HEADS)
    check("★ 消毒只改撞帧头的值，其它力度值一个都不动", _same)
    check("★ 力度 93(0x5D) 会被改成 92（不是 94 —— 94 也是帧头）"
          "（实际 %d）" % safe_operand(93, VEL_MIN, VEL_MAX),
          safe_operand(93, VEL_MIN, VEL_MAX) == 92)
    check("★ 消毒后的力度仍在 0~127 内（不会越界）",
          all(0 <= safe_operand(v, VEL_MIN, VEL_MAX) <= VEL_MAX
              for v in range(0, 256)))

    print("[9] 掩码工具")
    check("keys_to_mask/mask_to_keys 互逆",
          mask_to_keys(keys_to_mask([0, 5, 35])) == [0, 5, 35])

    print("[10] 按协议单元边界切块（safe_chunk_ranges）")
    #   V39.1：上位机把整帧按块丢进发送队列，而控制命令是【插队】发的
    #   （插到下一块之前）。如果块边界落在一个"命令字+操作数"内部，
    #   插队命令就会插进那一对中间，被板子当成操作数吃掉 ——
    #   参数被污染、后面全部错位。所以切点必须只落在单元边界上。

    def _bounds(b):
        s = {0, len(b)}
        for (x, y) in unit_spans(b):
            s.add(x)
            s.add(y)
        return s

    _blob = encode_frame(Frame(events=[
        Event(keys=(5,), ticks=2),                                # 单音 + 0xC2
        Event(keys=(13, 21, 25), ticks=1),                        # 和弦
        Event(keys=(), ticks=3),                                  # 休止
        Event(keys=(29,), ticks=1, oct=1),                        # 带 0xCC 的单音
        Event(keys=(1, 25), ticks=1, key_octs={1: -1, 25: 1}),    # 带 0xCD 的和弦
    ]))
    _bd = _bounds(_blob)
    bad = []
    for ch in (1, 2, 3, 5, 7, 13, 64, 256, 4096):
        rs = safe_chunk_ranges(_blob, ch)
        if not rs or rs[0][0] != 0 or rs[-1][1] != len(_blob):
            bad.append((ch, "范围没覆盖全", rs[:2]))
            continue
        for i in range(1, len(rs)):
            if rs[i][0] != rs[i - 1][1]:
                bad.append((ch, "有缝或重叠", (rs[i - 1], rs[i])))
        for (s, e) in rs:
            if s not in _bd or e not in _bd:
                bad.append((ch, "切进了单元内部", (s, e)))
        if b"".join(_blob[s:e] for (s, e) in rs) != _blob:
            bad.append((ch, "拼不回原字节流"))
    check("九种块长(1..4096)：覆盖完整、无缝无重叠、切点全在单元边界、拼回原样"
          + ("" if not bad else "  首个反例=%r" % (bad[:3],)),
          not bad)

    #   ★ 定向用例：0xC4/0xC5 是 4 字节的长单元，固定块长最容易在这里切坏
    _b2 = encode_settings(123456, 65432) + bytes([5]) + bytes([DUR_EXT, 3])
    _b2bd = _bounds(_b2)
    hard = []
    for ch in (1, 2, 3, 4, 5, 6, 7, 8):
        for (s, e) in safe_chunk_ranges(_b2, ch):
            if s not in _b2bd or e not in _b2bd:
                hard.append((ch, s, e))
    check("★ 设置命令(0xC4+3B / 0xC5+3B)这种长单元：任何块长都不会被切开"
          + ("" if not hard else "  首个反例=%r" % (hard[:3],)),
          not hard)

    check("空 blob -> 空结果", safe_chunk_ranges(b"", 256) == [])
    check("chunk<=0 -> 空结果（不崩）", safe_chunk_ranges(_blob, 0) == [])
    check("单字节 blob -> 恰好一个块",
          safe_chunk_ranges(bytes([0xAA]), 1) == [(0, 1)])
    _mx = max(e - s for (s, e) in unit_spans(_blob))
    check("最大单元远小于 256（块长 256 一定切得开），实测 %d 字节" % _mx, _mx < 64)

    print("[11] ★ V44：节拍器（0xCF 设置 / 参数夹紧 / 分块不切开）")
    #   这里钉死三件事：
    #     ① 字节布局（和 FPGA 侧对好的那 4 个字节，一个都不能变）
    #     ② 夹紧规则（★ BPM=0 是"关"而不是"最慢"，这条最容易写错）
    #     ③ 0xCF 必须登记在 _OP_LEN 里、长度 = 3（否则分块会把"命令 + 操作数"
    #        切开，插队命令落进中间被板子当操作数吃掉 —— 坑 21）
    check("pack_metro_set(100,4,1) == CF 64 04 01",
          pack_metro_set(100, 4, 1) == b"\xcf\x64\x04\x01")
    check("★ BPM=0 就是关（不是被夹成 30 又开起来）",
          pack_metro_set(0, 4, 1) == b"\xcf\x00\x04\x01")
    check("BPM 1~29 夹到 30", clamp_metro_bpm(1) == 30 and clamp_metro_bpm(29) == 30)
    check("BPM >240 夹到 240", clamp_metro_bpm(241) == 240 and clamp_metro_bpm(9999) == 240)
    check("BPM 30 / 240 边界原样保留",
          clamp_metro_bpm(30) == 30 and clamp_metro_bpm(240) == 240)
    check("BPM 空/非数字 -> 出厂默认 100",
          clamp_metro_bpm(None) == 100 and clamp_metro_bpm("") == 100)
    check("分子夹到 1~8",
          (clamp_metro_num(0), clamp_metro_num(1), clamp_metro_num(8),
           clamp_metro_num(9)) == (1, 1, 8, 8))
    check("分母编码夹到 0~2",
          (clamp_metro_den(-1), clamp_metro_den(0), clamp_metro_den(2),
           clamp_metro_den(3)) == (0, 0, 2, 2))
    check("编码 -> 真实分母 2 / 4 / 8",
          tuple(metro_den_value(c) for c in (0, 1, 2)) == (2, 4, 8))
    check("真实分母 2/4/8 -> 编码 0/1/2（★ 界面选的是分母，线上发的是编码）",
          tuple(metro_den_code(v) for v in (2, 4, 8)) == (0, 1, 2)
          and metro_den_code(None) == DEFAULT_METRO_DEN)
    check("出厂默认 = 100 BPM / 4 / 分母编码 1（= 4 分音符，即 4/4）",
          (DEFAULT_METRO_BPM, DEFAULT_METRO_NUM, metro_den_value(DEFAULT_METRO_DEN))
          == (100, 4, 4))
    check("★ 0xCF 登记进操作数长度表，长度 = 3（坑 21）",
          _OP_LEN.get(METRO_SET_BYTE) == 3)
    check("0xCF 不和已有命令撞车（0xC0~0xCE 都还占着）",
          METRO_SET_BYTE not in (0xC0, 0xC1, 0xC2, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8,
                                 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE))
    check("0x5E 不和已有状态帧头撞车（0x5A / 0x5B / 0x5D 都还占着，0x5D 一个字节没动）",
          METRO_STAT_BYTE == 0x5E and METRO_STAT_BYTE not in (0x5A, 0x5B, 0x5D))

    #   ★ 分块定向用例：0xCF + 3 字节 = 4 字节的长单元，
    #     固定块长 1..7 都必须把它整个放出去，绝不能切在中间。
    _mb = bytes([0xAA, METRO_SET_BYTE, 100, 4, 1, REST_BYTE, END_BYTE])
    _mbd = _bounds(_mb)
    _mbad = []
    for ch in (1, 2, 3, 4, 5, 6, 7):
        for (s, e) in safe_chunk_ranges(_mb, ch):
            if s not in _mbd or e not in _mbd:
                _mbad.append((ch, s, e))
    check("★ 节拍器命令(0xCF + 3B)任何块长都不会被切开"
          + ("" if not _mbad else "  首个反例=%r" % (_mbad[:3],)),
          not _mbad)
    check("节拍器命令整个算一个 4 字节单元",
          list(unit_spans(_mb))[1] == (1, 5))
    check("metro_name 说人话（关的时候不报 BPM）",
          metro_name(0, 4, 1) == "节拍器 关" and metro_name(120, 3, 1) == "节拍器 120 BPM 3/4")

    print("[12] ★ V45：逐事件力度（0xC3 / 夹紧 / 只作用一格 / 分块不切开 / 默认不变）")
    #   这里钉死四件事：
    #     ① 字节布局（和 FPGA 侧对好的 C3 64，一个都不能变）
    #     ② 夹紧规则（0 = 最小声；"不发 0xC3"才是默认 127 —— 两件事不能混）
    #     ③ 0xC3 必须登记在 _OP_LEN 里、长度 = 1（否则分块会把"命令 + 操作数"
    #        切开，插队命令落进中间被板子当成力度吃掉 —— 坑 21）
    #     ④ ★ 不发 0xC3 的帧必须和加这个功能之前【逐字节相同】
    check("pack_vel(100) == C3 64", pack_vel(100) == b"\xc3\x64")
    check("pack_vel(200) 夹到 127 -> C3 7F", pack_vel(200) == b"\xc3\x7f")
    check("clamp_vel: -5->0, 0->0, 64->64, 127->127, 200->127",
          (clamp_vel(-5), clamp_vel(0), clamp_vel(64), clamp_vel(127),
           clamp_vel(200)) == (0, 0, 64, 127, 127))
    check("★ clamp_vel 空/非数字 -> 127（退回「和以前一样」，不抛异常打断转换）",
          clamp_vel(None) == VEL_DEFAULT and clamp_vel("") == VEL_DEFAULT
          and clamp_vel("abc") == VEL_DEFAULT)
    check("vel_name 说人话（127 说明不发 0xC3）",
          vel_name(100) == "力度 100" and "不发 0xC3" in vel_name(127))
    check("★ 0xC3 登记进操作数长度表，长度 = 1（坑 21）",
          _OP_LEN.get(VEL_BYTE) == 1)
    check("0xC3 不和已有命令撞车（0x5C/0xAA/0xC0~0xC2/0xC4~0xC8/0xCA~0xCF/0xEE/0xFF 都还占着）",
          VEL_BYTE not in (0x5C, 0xAA, 0xC0, 0xC1, 0xC2, 0xC4, 0xC5, 0xC6, 0xC7,
                           0xC8, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xEE, 0xFF))
    check("VEL_MAX = 127（7 位），VEL_DEFAULT = 127 = 满幅",
          VEL_MAX == 127 and VEL_DEFAULT == VEL_MAX and VEL_MIN == 0)

    #   ★★ ④ 一条不发力度的帧 = 改造前的帧（这是"老功能不受影响"的硬证据）★★
    _nv = Frame(events=[Event((5,), 2), Event((0, 4, 7), 1), Event((), 3),
                        Event((29,), 1, oct=1)])
    check("★ 不填 vel 的事件：帧里一个 0xC3 都没有",
          VEL_BYTE not in encode_frame(_nv))
    check("★ 与「全部按默认 127」逐字节相同（即改造前的字节流）",
          encode_frame(_nv) == encode_frame(Frame(events=[
              Event((5,), 2, vel=VEL_DEFAULT), Event((0, 4, 7), 1, vel=127),
              Event((), 3), Event((29,), 1, oct=1, vel=127)])))

    #   ★ 编码位置：0xC3 必须在【这一格事件的最前面】（和 0xCC 一样是 pending）
    _vb1 = encode_body([Event((5,), vel=100)])
    check("带力度的单音：C3 64 紧挨在音符前面"
          + ("" if _vb1 == b"\xc3\x64\x05" else "  %r" % (_vb1,)),
          _vb1 == b"\xc3\x64\x05")
    _vb2 = encode_body([Event((0, 4, 7), vel=100)])
    check("带力度的和弦：C3 64 在 0xC0 前面（不能塞进和弦里面）"
          + ("" if _vb2 == b"\xc3\x64\xc0\x00\x04\x07\xc1" else "  %r" % (_vb2,)),
          _vb2 == b"\xc3\x64\xc0\x00\x04\x07\xc1")
    _vb3 = encode_body([Event((5,), 3, vel=100)])
    check("带力度 + 延长：C3 64 / 音符 / C2 02 的顺序"
          + ("" if _vb3 == b"\xc3\x64\x05\xc2\x02" else "  %r" % (_vb3,)),
          _vb3 == b"\xc3\x64\x05\xc2\x02")
    _vb4 = encode_body([Event((5,), vel=100, oct=1)])
    check("力度和八度同时有：C3 在前，两者都在音符前面"
          + ("" if _vb4 == b"\xc3\x64\xcc\x01\x05" else "  %r" % (_vb4,)),
          _vb4 == b"\xc3\x64\xcc\x01\x05")
    check("vel 越界在事件里就夹好（-1->0，999->127）",
          (Event((5,), vel=-1).vel, Event((5,), vel=999).vel) == (0, 127))

    #   ★ 只作用一格：下一格回到默认 127（板子的 pending 就是这个语义）
    _vf = Frame(events=[Event((5,), vel=100), Event((9,))])
    _vd = decode_frame(encode_frame(_vf))
    check("encode -> decode：力度还原"
          + ("" if [e.vel for e in _vd.events] == [100, VEL_DEFAULT]
             else "  %r" % ([e.vel for e in _vd.events],)),
          [e.vel for e in _vd.events] == [100, VEL_DEFAULT])
    check("★ 力度只作用一格（第二个事件回到 127）",
          _vd.events[1].vel == VEL_DEFAULT)

    #   ★ 力度是【原始数据】：任何 0..127 的值都要能还原 ——
    #     包括 0x00..0x23（长得像键号）、0x5C（长得像录音命令）那些值。
    _vvals = (0, 1, 0x23, 0x5C, 0x7E, 0x7F)
    _vbad = [(v, decode_frame(encode_frame(Frame(events=[Event((5,), vel=v)])))
              .events[0].vel) for v in _vvals]
    check("力度撞任何值（含 0x00/0x23/0x5C/0x7F）都能正确还原"
          + ("" if _vbad == [(v, v) for v in _vvals] else "  %r" % (_vbad,)),
          _vbad == [(v, v) for v in _vvals])

    #   ★ 分块定向用例：0xC3 + 1 字节 = 2 字节单元，
    #     固定块长 1..7 都必须把它整个放出去，绝不能切在中间。
    _vb = bytes([0xAA, VEL_BYTE, 100, 5, REST_BYTE, END_BYTE])
    _vbd = _bounds(_vb)
    _vbad2 = []
    for ch in (1, 2, 3, 4, 5, 6, 7):
        for (s, e) in safe_chunk_ranges(_vb, ch):
            if s not in _vbd or e not in _vbd:
                _vbad2.append((ch, s, e))
    check("★ 力度命令(0xC3 + 1B)任何块长都不会被切开"
          + ("" if not _vbad2 else "  首个反例=%r" % (_vbad2[:3],)),
          not _vbad2)
    check("力度命令整个算一个 2 字节单元", list(unit_spans(_vb))[1] == (1, 3))
    check("★ 整帧分块后拼回来 = 原字节流（带力度的帧也不丢字节）",
          b"".join(encode_frame(_vf)[s:e]
                   for (s, e) in safe_chunk_ranges(encode_frame(_vf), 7))
          == encode_frame(_vf))

    print("[13] ★ V45：merge_repeats 不跨力度合并")
    _m = merge_repeats(Frame(events=[Event((5,), vel=100), Event((5,), vel=100),
                                    Event((5,), vel=40), Event((5,), vel=100)]))
    _mtv = [(e.ticks, e.vel) for e in _m.events]
    check("★ 键相同但力度不同 -> 不合并（并了就把力度改掉了）"
          + ("" if _mtv == [(2, 100), (1, 40), (1, 100)] else "  %r" % (_mtv,)),
          _mtv == [(2, 100), (1, 40), (1, 100)])
    check("键相同、力度也相同 -> 照样合并成一条",
          [e.ticks for e in merge_repeats(
              Frame(events=[Event((5,), vel=40), Event((5,), vel=40)])).events] == [2])

    print("[14] ★ V46：LCD 显示模式（0xD0 设定 / 0x5F 回报 / 夹到 0~2 / 分块不切开）")
    #   这里钉死四件事：
    #     ① 字节布局（和 FPGA 侧对好的那 2 个字节，一个都不能变）
    #     ② 夹紧规则（★ 是【夹到 0~2】不是取模：发 5 得 2，不是得 0）
    #     ③ 0xD0 必须登记在 _OP_LEN 里、长度 = 1（否则分块会把"命令 + 操作数"
    #        切开，插队命令落进中间被板子当操作数吃掉 —— 坑 21）
    #     ④ 三个档位名说人话（界面上就是这三个字）
    check("pack_disp_set(1) == D0 01（波形）",
          pack_disp_set(1) == b"\xd0\x01")
    check("pack_disp_set(0/2) == D0 00 / D0 02（频谱 / 李萨如）",
          pack_disp_set(0) == b"\xd0\x00" and pack_disp_set(2) == b"\xd0\x02")
    _dc = (clamp_disp(-1), clamp_disp(0), clamp_disp(1), clamp_disp(2),
           clamp_disp(3), clamp_disp(5))
    check("clamp_disp: -1->0, 0->0, 1->1, 2->2, 3->2, 5->2"
          + ("" if _dc == (0, 0, 1, 2, 2, 2) else "  %r" % (_dc,)),
          _dc == (0, 0, 1, 2, 2, 2))
    check("★ 夹紧不是取模（3 夹到 2 而不是绕回 0）", clamp_disp(3) == 2)
    check("★ clamp_disp 空/非数字 -> 0（当作出厂默认「频谱」，不抛异常打断界面）",
          clamp_disp(None) == DISP_MIN and clamp_disp("") == DISP_MIN
          and clamp_disp("abc") == DISP_MIN)
    check("越界的值发出去之前就夹好（5 -> D0 02）",
          pack_disp_set(5) == b"\xd0\x02")
    _dn = (disp_name(0), disp_name(1), disp_name(2))
    check("disp_name：0/1/2 -> 频谱/波形/李萨如"
          + ("" if _dn == ("频谱", "波形", "李萨如") else "  %r" % (_dn,)),
          _dn == ("频谱", "波形", "李萨如"))
    check("disp_name：None / 越界 -> 未知（不把「板子还没报过」画成频谱）",
          (disp_name(None), disp_name(3), disp_name(99)) == ("未知", "未知", "未知"))
    check("★ 0xD0 登记进操作数长度表，长度 = 1（坑 21）  %r"
          % (_OP_LEN.get(DISP_SET_BYTE),),
          _OP_LEN.get(DISP_SET_BYTE) == 1)
    check("0xD0 不和已有命令撞车（0x5C/0xAA/0xC0~0xC3/0xC4~0xC8/0xCA~0xCF/0xEE/0xFF 都还占着）",
          DISP_SET_BYTE not in (0x5C, 0xAA, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
                                0xC6, 0xC7, 0xC8, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE,
                                0xCF, 0xEE, 0xFF))
    check("0x5F 不和已有回传帧头撞车（0x5A/0x5B/0x5D/0x5E 都还占着）",
          DISP_STAT_BYTE == 0x5F
          and DISP_STAT_BYTE not in (0x5A, 0x5B, 0x5D, 0x5E))
    check("DISP_MIN=0 / DISP_MAX=2，正好三个档位名",
          (DISP_MIN, DISP_MAX) == (0, 2) and len(DISP_NAMES) == DISP_MAX - DISP_MIN + 1)

    #   ★ 分块定向用例：0xD0 + 1 字节 = 2 字节单元，
    #     固定块长 1..7 都必须把它整个放出去，绝不能切在中间。
    _db = bytes([0xAA, DISP_SET_BYTE, 1, 5, REST_BYTE, END_BYTE])
    _dbd = _bounds(_db)
    _dbad = []
    for ch in (1, 2, 3, 4, 5, 6, 7):
        for (s, e) in safe_chunk_ranges(_db, ch):
            if s not in _dbd or e not in _dbd:
                _dbad.append((ch, s, e))
    check("★ 显示模式命令(0xD0 + 1B)任何块长都不会被切开"
          + ("" if not _dbad else "  首个反例=%r" % (_dbad[:3],)),
          not _dbad)
    check("显示模式命令整个算一个 2 字节单元", list(unit_spans(_db))[1] == (1, 3))

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
