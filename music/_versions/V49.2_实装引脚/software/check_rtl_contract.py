# -*- coding: utf-8 -*-
"""check_rtl_contract.py —— 把「硬件里的真值」和「上位机里的常量」对起来

为什么需要它
------------
上位机的键号<->音高映射、协议字节常量，都是**手抄**硬件里的表得来的。
抄错一次就是一整轮"发了一堆却没响 / 音全错"，而且从代码上看不出来 ——
`键号 = MIDI − 48` 这个错就是这么来的：文档、编码器、解码器三方一致地错，
只有上板听才发现。

所以这里直接从 RTL 源码里把真值抠出来，和上位机逐个对：
  1) `top.v` 的 `delta[]` 36 个频率   <->  `score_jp.key_to_midi` 的 36 个音高
  2) `uart_packet_echo.v` 的控制字节  <->  `fpga_proto` / `fpga_rec` 的常量
  3) 录音回传帧长 / 采样周期          <->  `fpga_rec` 的常量

RTL 改了而上位机没跟着改 -> 这里立刻红。反过来也一样。

用法：python check_rtl_contract.py            （自动找工程目录）
      python check_rtl_contract.py <music工程目录>
"""

from __future__ import annotations

import io
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_MUSIC_ROOT = (r"D:\CODE\FPGA_Gowin\Gowin\Gowin_V1.9.11.03_Education_x64"
               r"\IDE\bin\Documents\60k")

# ★ 工程目录按候选列表找，不写死一条路径。
#   这个工程被挪过一次（music(2) -> music_player\music），写死路径的脚本当时就失效了。
#   注意不能只看"有没有 src\top.v" —— 同一个 60k 目录下还有 music / music_test0
#   这些别的工程，指错了会拿错的 RTL 去核对，满屏"不一致"，反而更难查。
MUSIC_DIR_CANDIDATES = [
    os.path.join(_MUSIC_ROOT, "music_player", "music"),
    os.path.join(_MUSIC_ROOT, "music(2)", "music"),
    os.path.join(_MUSIC_ROOT, "music"),
]


def looks_like_our_project(d):
    """认准我们这一版：uart_packet_echo.v 里必须有【录音回传】那一版才有的 REC_FX_HEAD。"""
    upe = os.path.join(d, "src", "uart_packet_echo.v")
    top = os.path.join(d, "src", "top.v")
    if not (os.path.isfile(upe) and os.path.isfile(top)):
        return False
    try:
        with open(upe, "r", encoding="utf-8", errors="replace") as fh:
            return "REC_FX_HEAD" in fh.read()
    except OSError:
        return False


def find_music_dir():
    for d in MUSIC_DIR_CANDIDATES:
        if looks_like_our_project(d):
            return d
    return MUSIC_DIR_CANDIDATES[0]


MUSIC_DIR_DEFAULT = find_music_dir()


def _read(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read()


# --------------------------------------------------------------------------
# 1) top.v 的 delta[] 频率表
# --------------------------------------------------------------------------
_DELTA_RE = re.compile(
    r"assign\s+delta\s*\[\s*(\d+)\s*\]\s*=\s*calc_delta\s*\(\s*32'd(\d+)\s*\)")


def parse_delta_table(top_v_path):
    """返回 {键号: 频率Hz}（top.v 的 delta[] 是唯一真值）"""
    src = _read(top_v_path)
    table = {}
    for m in _DELTA_RE.finditer(src):
        table[int(m.group(1))] = int(m.group(2))
    return table


def freq_to_midi(freq, a4=440.0):
    """频率 -> 最近的 MIDI 音符号（四舍五入到最近的半音）"""
    if freq <= 0:
        return -1
    return int(round(69 + 12 * math.log(freq / a4, 2)))


# --------------------------------------------------------------------------
# 2) uart_packet_echo.v 的控制字节
# --------------------------------------------------------------------------
def _eval_lit(val, known):
    """把一个 localparam 的值算成整数（字面量，或只用已知名字和四则运算的表达式）"""
    v = val.strip()
    for pat, base in ((r"\d*'h([0-9A-Fa-f_]+)", 16),
                      (r"\d*'b([01_]+)", 2),
                      (r"\d*'d([\d_]+)", 10),
                      (r"(\d[\d_]*)", 10)):
        hit = re.fullmatch(pat, v)
        if hit:
            return int(hit.group(1).replace("_", ""), base)
    # 表达式：先把已知名字替换成数字，再要求剩下的只有数字和运算符
    #   （top.v 里 FRAME_DIV = FRAME_BITS * BCLK_HALF * 2、SAMPLE_RATE_HZ = CLK_FREQ_HZ / FRAME_DIV）
    expr = v
    for name in sorted(known, key=len, reverse=True):
        expr = re.sub(r"\b%s\b" % re.escape(name), str(known[name]), expr)
    # 三目：COND ? A : B（top.v 里 BCLK_HALF = I2S_SPEC ? 2 : 3）
    m3 = re.fullmatch(r"(.+?)\?(.+):(.+)", expr)
    if m3:
        cond = _eval_lit(m3.group(1), {})
        if cond is None:
            return None
        return _eval_lit(m3.group(2), {}) if cond else _eval_lit(m3.group(3), {})
    if not re.fullmatch(r"[\d\s+\-*/()]+", expr):
        return None
    try:
        return int(eval(expr, {"__builtins__": {}}, {}))       # noqa: S307
    except Exception:                                          # noqa: BLE001
        return None


def parse_localparams(v_path):
    """抠出 localparam 的数值。

    ★ 必须按"整条 localparam ... ;"来切、再按逗号拆名字 —— 因为 RTL 里
      经常一行写好几个（`localparam REC_EV_HEAD=8'h5A, REC_ST_HEAD=8'h5B;`），
      逐个 `localparam NAME =` 去匹配会漏掉第二个以后的所有名字。
    ★ 值可能是表达式（引用别的 localparam），所以要迭代几轮解开引用。
    """
    src = _read(v_path)
    raw = {}
    for m in re.finditer(r"localparam\b([^;]*);", src):
        for part in m.group(1).split(","):
            mm = re.match(r"\s*(?:\[[^\]]*\]\s*)?"
                          r"(?:integer\s+|signed\s+|unsigned\s+)?"
                          r"(\w+)\s*=\s*(.+?)\s*$", part)
            if mm:
                raw[mm.group(1)] = mm.group(2)

    out = {}
    for _round in range(6):                    # 解开多层引用
        progress = False
        for name, val in raw.items():
            if name in out:
                continue
            v = _eval_lit(val, out)
            if v is not None:
                out[name] = v
                progress = True
        if not progress:
            break
    return out


# --------------------------------------------------------------------------
def main():
    music = sys.argv[1] if len(sys.argv) > 1 else MUSIC_DIR_DEFAULT
    top_v = os.path.join(music, "src", "top.v")
    upe_v = os.path.join(music, "src", "uart_packet_echo.v")

    fails = []
    oks = []

    def check(name, cond, extra=None):
        if cond:
            oks.append(name)
            print("  ok   %s" % name)
        else:
            fails.append(name)
            print("  FAIL %s%s" % (name, ("  -> " + repr(extra)) if extra is not None else ""))

    for p in (top_v, upe_v):
        if not os.path.isfile(p):
            print("找不到 %s" % p)
            return 2

    import score_jp as JP
    import fpga_proto as P

    # ---- 1) 键号 <-> 音高 ----
    print("[1] top.v 的 delta[] 频率表 <-> score_jp 的键号映射")
    table = parse_delta_table(top_v)
    check("top.v 里抠到 36 个 delta[]", len(table) == 36, sorted(table)[:3])

    bad = []
    for k in range(36):
        if k not in table:
            bad.append((k, "缺"))
            continue
        want = JP.key_to_midi(k)
        got = freq_to_midi(table[k])
        if want != got:
            bad.append((k, table[k], "top.v 是 %s(%d) 上位机是 %s(%d)"
                        % (JP._oct_name(got), got, JP._oct_name(want), want)))
    check("36 个键的频率和上位机音高逐个对上", not bad, bad[:6])

    # 反向也要成立，不然是"多对一"
    back = [JP.midi_to_key(JP.key_to_midi(k)) for k in range(36)]
    check("键号 -> 音高 -> 键号 往返一致", back == list(range(36)),
          [(k, v) for k, v in enumerate(back) if k != v][:6])

    # 4 个锚点（写死在 top.v 注释里的 Hz，最直观）
    for key, midi, hz in ((5, 45, 110), (13, 60, 262), (29, 69, 440), (35, 80, 831)):
        check("锚点 键%d = %s = %.0fHz" % (key, JP._oct_name(midi), hz),
              table.get(key) == hz and JP.key_to_midi(key) == midi,
              (table.get(key), JP.key_to_midi(key)))

    check("音域是 MIDI %d..%d（3 个八度 36 键）" % (JP.BOARD_MIDI_MIN, JP.BOARD_MIDI_MAX),
          JP.BOARD_MIDI_MIN == 45 and JP.BOARD_MIDI_MAX == 80,
          (JP.BOARD_MIDI_MIN, JP.BOARD_MIDI_MAX))

    # ---- 2) 控制字节 ----
    print("[2] uart_packet_echo.v 的控制字节 <-> fpga_proto 的常量")
    lp = parse_localparams(upe_v)
    check("从 uart_packet_echo.v 抠到 localparam", len(lp) >= 6, sorted(lp))

    pairs = [
        ("CLR_BYTE", "CLR_BYTE", 0xAA),
        ("END_BYTE", "END_BYTE", 0xFF),
        ("REST_BYTE", "REST_BYTE", 0xEE),
        ("CHORD_ON", "CHORD_ON", 0xC0),
        ("CHORD_OFF", "CHORD_OFF", 0xC1),
        ("DUR_EXT", "DUR_EXT", 0xC2),
        ("SET_TICK", "SET_TICK", 0xC4),
        ("SET_REL", "SET_REL", 0xC5),
        ("REC_EV_HEAD", None, 0x5A),
        ("REC_ST_HEAD", None, 0x5B),
        ("REC_CMD_BYTE", None, 0x5C),
        ("REC_FX_HEAD", None, 0x5D),
        # ★ 演奏控制（0xC6/0xC7/0xC8）：上位机在 keymap.py 里，也和这里逐个对
        ("KEYMAP_SET", None, 0xC6),
        ("STOP_BYTE", None, 0xC7),
        ("REPLAY_BYTE", None, 0xC8),
        # ★ V43：跟谱演奏模式（上位机 fpga_proto/fpga_rec 和 RTL 逐个对）
        ("MODE_SET_BYTE", "MODE_SET_BYTE", 0xCE),
        # ★ V48：LCD 显示模式（0xD0 下行设定 / 0x5F 上行状态帧）
        ("DISP_SET_BYTE", "DISP_SET_BYTE", 0xD0),
        ("DISP_STAT_BYTE", "DISP_STAT_BYTE", 0x5F),
    ]
    for rtl_name, proto_name, want in pairs:
        got_rtl = lp.get(rtl_name)
        got_proto = getattr(P, proto_name, want) if proto_name else want
        check("0x%02X  %s" % (want, rtl_name),
              got_rtl == want and got_proto == want, (got_rtl, got_proto))

    check("SLOT_MAX = 1024（事件数上限）", lp.get("SLOT_MAX") == P.SLOT_MAX == 1024,
          (lp.get("SLOT_MAX"), P.SLOT_MAX))

    # 录音常量必须和 fpga_rec 一致（这是上位机解析器认帧头的依据）
    import fpga_rec as R
    check("fpga_rec 的帧头常量和 RTL 一致",
          R.REC_EV_HEAD == lp.get("REC_EV_HEAD") == 0x5A
          and R.REC_ST_HEAD == lp.get("REC_ST_HEAD") == 0x5B
          and R.REC_CMD == lp.get("REC_CMD_BYTE") == 0x5C
          and R.REC_FX_HEAD == lp.get("REC_FX_HEAD") == 0x5D,
          (R.REC_EV_HEAD, R.REC_ST_HEAD, R.REC_CMD, R.REC_FX_HEAD))

    # ★★ V43：跟谱演奏模式（三条必须一致）★★
    #   RTL 的 MODE_SET_BYTE / fpga_proto 的 MODE_SET_BYTE / fpga_rec 的 pack_mode_set
    #   ★ 还要确认【操作数长度表】里登记了 0xCE —— 没登记的话
    #     safe_chunk_ranges/safeChunkRanges 分块时可能把"命令+操作数"从中间切开，
    #     而命令是插队发的，正好落进那一对中间就会被当成操作数吃掉（坑 21）。
    check("0xCE 三处一致（RTL / fpga_proto / fpga_rec.pack_mode_set）",
          lp.get("MODE_SET_BYTE") == 0xCE and P.MODE_SET_BYTE == 0xCE
          and R.pack_mode_set(True) == bytes([0xCE, 1])
          and R.pack_mode_set(False) == bytes([0xCE, 0]),
          (lp.get("MODE_SET_BYTE"), P.MODE_SET_BYTE))
    check("0xCE 登记进了操作数长度表（否则分块会把命令和操作数切开）",
          P._OP_LEN.get(0xCE) == 1, P._OP_LEN.get(0xCE))
    check("0xCE 不在已用命令里（0xC0~0xCD 都占了，0xCE/0xCF 是空的）",
          0xCE not in (0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
                       0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD))

    # ---- 2b) 演奏控制 + 按键映射（keymap.py 的常量和 RTL 逐个对）----
    print("[2b] 演奏控制 / 按键映射的字节 <-> keymap.py")
    import keymap as KM
    check("0xC6 写映射：keymap.KEYMAP_SET == RTL.KEYMAP_SET",
          KM.KEYMAP_SET == lp.get("KEYMAP_SET") == 0xC6,
          (KM.KEYMAP_SET, lp.get("KEYMAP_SET")))
    check("0xC7 停止演奏：一致",
          KM.STOP_BYTE == lp.get("STOP_BYTE") == 0xC7,
          (KM.STOP_BYTE, lp.get("STOP_BYTE")))
    check("0xC8 重播：一致",
          KM.REPLAY_BYTE == lp.get("REPLAY_BYTE") == 0xC8,
          (KM.REPLAY_BYTE, lp.get("REPLAY_BYTE")))
    check("禁用哨兵值和键数对得上（>= 36 且不和任何控制字节撞）",
          KM.DISABLE >= KM.N_KEYS and KM.DISABLE < 0x5A, KM.DISABLE)
    check("36 个键都编得出映射、长度正好 108",
          len(KM.encode_all(KM.default_map())) == 108)

    # ---- 2c) ★ V31：效果开关 / 音色的字节（RTL <-> fpga_rec.py）----
    print("[2c] 效果开关 / 音色的字节 <-> fpga_rec.py")
    check("0xC9 翻转效果开关：一致",
          R.FX_TGL_BYTE == lp.get("FX_TGL_BYTE") == 0xC9,
          (R.FX_TGL_BYTE, lp.get("FX_TGL_BYTE")))
    check("0xCA 设定音色：一致",
          R.TONE_SET_BYTE == lp.get("TONE_SET_BYTE") == 0xCA,
          (R.TONE_SET_BYTE, lp.get("TONE_SET_BYTE")))
    check("0xCB 请求状态：一致",
          R.FX_REQ_BYTE == lp.get("FX_REQ_BYTE") == 0xCB,
          (R.FX_REQ_BYTE, lp.get("FX_REQ_BYTE")))
    # ★★ V48.2：音色从 3 档扩到 4 档（加了"小号"）
    check("★ 音色档数 = 4（普通/笛子/吉他/小号）",
          len(R.TONE_NAMES) == 4 and R.TONE_MAX == 3, R.TONE_NAMES)
    # ★ 关键性质：状态帧第 3 个字节（音色 0..3）不能被误当成新帧的开头。
    #   所有帧头都 >= 0x5A，所以这是设计性质，不是巧合。
    check("音色值 0..3 和数据帧头不冲突（帧头都 >= 0x5A）",
          R.TONE_MAX < 0x5A, R.TONE_MAX)
    #   ★ 板子报"小号"(3) 时上位机必须认 —— 判据是 TONE_MAX，别写死 2
    check("★ 上位机的音色判据用的是 TONE_MAX（不是写死的 2）",
          R.pack_tone_set(3) == bytes([0xCA, 3])
          and R.tone_name(3) == "小号" and R.tone_name(4) == "未知",
          (R.pack_tone_set(3), R.tone_name(3)))
    check("最大的音色值仍然 < 任何帧头（0x5A）",
          max(range(len(R.TONE_NAMES))) < R.REC_EV_HEAD,
          (len(R.TONE_NAMES) - 1, hex(R.REC_EV_HEAD)))
    # 0xC9/0xCA 的操作数是原始数据，RTL 里必须用计数器挡住（和 0xC2/0xC6 同理）
    _upe = _read(upe_v)
    check("RTL 有 fx_left / tone_left 两个操作数计数器",
          re.search(r"fx_left", _upe) is not None
          and re.search(r"tone_left", _upe) is not None)
    check("操作数计数器排在命令字分支之前（否则操作数会被当命令解释）",
          _upe.find("else if (fx_left)") < _upe.find("rx_data == FX_TGL_BYTE")
          and _upe.find("else if (tone_left)") < _upe.find("rx_data == TONE_SET_BYTE"))
    check("效果状态帧发 3 个字节（帧头 + 效果 + 音色）",
          re.search(r"fx_idx\s*==\s*3'd3", _upe) is not None, "fx_idx 位宽 3")
    check("状态帧改成「变了就报」（不再限定录音时）",
          re.search(r"wire\s+fx_want\s*=\s*fx_req", _upe) is not None)

    # ---- 3) 录音帧结构 ----
    print("[3] 录音回传帧结构")
    check("事件帧 = 1 + 5 + 2 = 8 字节", R.EV_FRAME == 8, R.EV_FRAME)
    check("dt 单位是 10ms", R.MS_PER_TICK10 == 10, R.MS_PER_TICK10)
    # 采样周期 = clk_50m_ext 频率 x 秒数。RTL: 20'd500_000 @ 50MHz = 10ms
    div = lp.get("REC_TICK_DIV")
    check("REC_TICK_DIV = 500000（@50MHz = 10ms，正好对上 dt 的单位）",
          div == R.MS_PER_TICK10 * 50_000, div)

    # ---- 4) 回显抑制 ----
    print("[4] 录音时必须抑制回显（否则回显里的 0x5A 会被当成帧头）")
    src = _read(upe_v)
    check("echo_push 里有 !rec_on",
          re.search(r"echo_push\s*=\s*[^;]*!\s*rec_on", src) is not None)

    # ---- 5) 发声链路只读 ----
    print("[5] 录音只读发声状态，不反过来驱动发声链路")
    body = src.split("module", 1)[1]
    check("gate_mask_in 只出现在端口列表里（没有赋值语句）",
          re.search(r"\bgate_mask_in\s*(?:\[[^\]]*\]\s*)?=", body) is None)
    check("echo_push 受 rec_on 门控（录音时一个字都不回显）",
          re.search(r"echo_push\s*=\s*tx_req\s*&&\s*!\s*rec_on", src) is not None)
    # 板子上的按键 10：top.v 里应该是 btn_new[9]
    top_src = _read(top_v)
    check("top.v 用 btn_new[9]（按键10）驱动录音开关",
          re.search(r"btn_new\s*\[\s*9\s*\]", top_src) is not None)
    # ★★ 录的必须是【门控 sounding_mask_aud】，不是包络输出 sounding_mask ★★
    #   包络输出带 1.46 秒释音尾巴，会导致：
    #     ① 尾巴没走完就重击同一个键 -> 掩码连着不变 -> 两下并成一个长音
    #     ② 一直为 1 的键在每个事件上重复触发（8 个音弹成 32 次起音）
    #   这一条就是防它退回去的。
    check("top.v 把 sounding_mask_aud（门控）送到 .gate_mask_in",
          re.search(r"\.gate_mask_in\s*\(\s*rec_mask_s2\s*\)", top_src) is not None)
    # ★★ V48.3：跨域前先在【音频域】打一拍 ★★
    #   原来 `rec_mask_s1 <= sounding_mask_aud;` 是一条跨两个时钟域、四层组合的
    #   长链（midi_guide_s1 -> key_mask -> sounding_mask_aud -> rec_mask_s1.D），
    #   实测 SETUP slack = -0.155ns（全工程唯一 setup 违例）。
    #   加 `sounding_mask_gate <= sounding_mask_aud;`（clk_audio）之后，跨域那段
    #   变成"寄存器 -> 寄存器"。★ 判据跟着改，但【本意不变】：
    #   录音链采的仍然是【门控】，绝不许退回包络输出 sounding_mask 或裸 key_mask。
    check("V48.3 跨域前那一拍采的是 sounding_mask_aud（门控）",
          re.search(r"sounding_mask_gate\s*<=\s*sounding_mask_aud\s*;", top_src) is not None)
    check("录音同步链采的是打拍后的门控 sounding_mask_gate",
          re.search(r"rec_mask_s1\s*<=\s*sounding_mask_gate\s*;", top_src) is not None)
    check("录音源没有退回到 sounding_mask（包络输出、带尾巴）",
          re.search(r"rec_mask_s1\s*<=\s*sounding_mask\s*;", top_src) is None
          and re.search(r"sounding_mask_gate\s*<=\s*sounding_mask\s*;", top_src) is None)
    check("录音源没有退回到 key_mask",
          re.search(r"rec_mask_s1\s*<=\s*key_mask\s*;", top_src) is None
          and re.search(r"sounding_mask_gate\s*<=\s*key_mask\s*;", top_src) is None)
    # ★ 别为了"省一拍"把中间那级删掉：删掉 setup 违例就回来了（V48.3 踩过）
    check("V48.3 的跨域打拍寄存器 sounding_mask_gate 还在（删了就回到 -0.155ns 违例）",
          re.search(r"reg\s*\[35:0\]\s*sounding_mask_gate\s*;", top_src) is not None)
    check("top.v 里 sounding_mask_aud 确实 = key_mask | sustain_latch_aud",
          re.search(r"sounding_mask_aud\s*=\s*key_mask\s*\|\s*sustain_latch_aud",
                    top_src) is not None)
    check("top.v 把按键脉冲接到 .btn_rec",
          re.search(r"\.btn_rec\s*\(", top_src) is not None)
    # ★ play_start 必须恒 0。top.v 里它接的是一条自动演奏链：
    #   上升沿 -> auto_en<=1 -> auto_keys[note_idx] 常亮 -> 进 key_mask。
    #   板子里由 uart_packet_echo 唯一驱动，所以查它的赋值语句就行。
    ps = re.findall(r"play_start\s*<=\s*([^;]+);", src)
    check("uart_packet_echo.v 里 play_start 恒 0（不会唤醒 top.v 的自动演奏链）",
          bool(ps) and all(v.strip() in ("1'b0", "1'd0", "0") for v in ps), ps)
    pl = re.findall(r"play_len\s*<=\s*([^;]+);", src)
    check("play_len 也恒 0", bool(pl) and all(v.strip() in ("8'd0", "1'b0", "0")
                                            for v in pl), pl)
    sw = re.findall(r"score_we\s*<=\s*([^;]+);", src)
    check("score_we 恒 0（不写 top.v 的乐谱 RAM）",
          bool(sw) and all(v.strip() in ("1'b0", "1'd0", "0") for v in sw), sw)

    # ---- 6) 效果开关（录音回传的第 3 种帧）----
    print("[6] 效果开关：位序必须和上位机完全一致")
    check("uart_packet_echo.v 有 fx_state_in 端口",
          re.search(r"input\s+wire\s*\[7:0\]\s+fx_state_in", src) is not None)
    check("top.v 把 fx_state 接到 .fx_state_in",
          re.search(r"\.fx_state_in\s*\(\s*fx_s2\s*\)", top_src) is not None)
    m = re.search(r"wire\s*\[7:0\]\s*fx_state\s*=\s*\{([^}]*)\}", top_src)
    if m:
        # ★ 先删掉行内 // 注释再按逗号拆 —— 否则注释会跟着名字一起被当成名字
        body = re.sub(r"//[^\n]*", "", m.group(1))
        got_bits = [x.strip() for x in body.split(",") if x.strip()]
    else:
        got_bits = []
    want_bits = ["cho_on", "rev_on", "flute_on", "dly_on", "trem_on",
                 "timbre_50m", "sustain_on", "glide_en"]
    check("位序 bit7..bit0 = 合唱/混响/笛子/延迟/颤音/音色/延音/滑音",
          got_bits == want_bits, (got_bits, want_bits))
    check("wave_mode 没被打包进去（它只喂 LCD，不影响声音）",
          "wave_mode" not in got_bits)
    # ★ 效果链路必须是【只读】的：fx_state 只是把这几个寄存器"看"一眼
    check("没有从 fx_state 回写效果寄存器（只读，不碰效果链路）",
          re.search(r"(glide_en|rev_on|cho_on|trem_on|dly_on|flute_on"
                    r"|sustain_on|timbre_50m)\s*<=\s*[^;]*fx_", top_src) is None)
    check("上位机 FX_BITS 的 8 个名字和位序也对得上",
          [en for en, _cn in R.FX_BITS] ==
          ["glide", "sustain", "timbre", "trem", "delay", "flute",
           "reverb", "chorus"],
          [en for en, _cn in R.FX_BITS])

    # ---- 7) 释音尾巴的常数（板子不报尾巴，尾巴是上位机按这些常数补的）----
    #   ★ 这几个数一旦对不上，合成出来的音长就会整体错一截，
    #     而且从代码上完全看不出来 —— 所以必须自动核对。
    print("[7] ADSR 释音尾巴：上位机的常数必须和 top.v 一致")
    tp = parse_localparams(top_v)
    need = ["CLK_FREQ_HZ", "FRAME_DIV", "ENV_SUSTAIN_LV", "ENV_RELEASE_DIV",
            "ENV_SUS_LV_FLUTE", "ENV_REL_DIV_FLUTE"]
    missing = [n for n in need if n not in tp]
    check("top.v 里能抠到这 6 个常数", not missing, missing)
    if not missing:
        sr_out = tp["CLK_FREQ_HZ"] // tp["FRAME_DIV"]      # top.v 是整数除法
        # ★★ V49：采样率翻倍（BCLK_HALF 2->1）后，包络/滑音/颤音/效果这些
        #   "时间语义"逻辑走的是控制节拍 ctrl_en = 每 2 个采样点一次，
        #   所以它们的时间常数换算仍要用【48.828 kHz】这一档 —— 上位机的
        #   尾巴常数（按 48.828 kHz 标的）因此【一个字都不用改】。
        sr = sr_out // 2
        check("V49：输出采样率 %d Hz = 2 × 控制节拍 %d Hz，且控制节拍仍等于上位机的 SR_HZ"
              % (sr_out, sr),
              sr_out == 2 * sr and sr == R.SR_HZ, (sr_out, sr, R.SR_HZ))
        want_rel = (tp["ENV_SUSTAIN_LV"] * tp["ENV_RELEASE_DIV"] / sr * 1000.0
                    + R.GATE_DEBOUNCE_MS)
        want_flute = (tp["ENV_SUS_LV_FLUTE"] * tp["ENV_REL_DIV_FLUTE"] / sr * 1000.0
                      + R.GATE_DEBOUNCE_MS)
        check("默认音色尾巴 %.1fms 和上位机一致" % want_rel,
              abs(want_rel - R.REL_MS) < 0.5, (want_rel, R.REL_MS))
        check("笛子音色尾巴 %.1fms 和上位机一致" % want_flute,
              abs(want_flute - R.REL_FLUTE_MS) < 0.5, (want_flute, R.REL_FLUTE_MS))
        check("上位机的 ENV_* 原始常数也逐个对上",
              (R.ENV_SUSTAIN_LV, R.ENV_RELEASE_DIV, R.ENV_SUS_LV_FLUTE,
               R.ENV_REL_DIV_FLUTE) ==
              (tp["ENV_SUSTAIN_LV"], tp["ENV_RELEASE_DIV"],
               tp["ENV_SUS_LV_FLUTE"], tp["ENV_REL_DIV_FLUTE"]),
              (R.ENV_SUSTAIN_LV, R.ENV_RELEASE_DIV, R.ENV_SUS_LV_FLUTE,
               R.ENV_REL_DIV_FLUTE))
        check("滑音 GLIDE_STEP 也一致",
              tp.get("GLIDE_STEP") == R.GLIDE_STEP, (tp.get("GLIDE_STEP"), R.GLIDE_STEP))

    # ------------------------------------------------------------------
    # [8] ★ V39.2：任何"等操作数"的状态都必须有超时兜底
    #
    #   为什么必须自动核对：V38 只给 fx_left/tone_left 加了超时，其余五个
    #   （map_left / oct_left / key_oct_left / cfg_left / dur_wait）没有 ——
    #   操作数一旦在传输中丢了，那个状态就【永久卡住】，之后每一个字节都被
    #   当成操作数吃掉（map_left 是"等 2 字节"，卡住后每 2 字节吃掉一对）。
    #   而上位机的「按键映射下发 / 停止演奏 / 重播」都会清发送队列，
    #   正好能把 73 字节的映射表发到一半截断 —— 这两件事是配套的。
    #   ★ 这条检查防止以后有人新加"等操作数"状态却忘了接超时。
    # ------------------------------------------------------------------
    print("[8] ★ 所有\"等操作数\"状态都必须有超时兜底（V39.2）")
    upe = _read(upe_v)
    WAIT_STATES = ["map_left", "oct_left", "fx_left", "tone_left",
                   "key_oct_left", "cfg_left", "dur_wait",
                   # ★ V47/V48：0xC3（力度）/ 0xD0（显示模式）各自的一个字节
                   "vel_wait", "disp_wait"]

    m = re.search(r"wire\s+any_op_wait\s*=\s*(.+?);", upe, re.S)
    check("有 any_op_wait（统一超时的判据）", bool(m))
    if m:
        lack = [s for s in WAIT_STATES if s not in m.group(1)]
        check("any_op_wait 覆盖全部 %d 个等待状态" % len(WAIT_STATES), not lack, lack)

    #   超时分支：从 `if (any_op_wait) begin` 到内部的 `op_wait_cnt + 1`，
    #   这一段正好是整个"到点放弃"块（用它做边界，免得非贪婪匹配停在内层 end）。
    #   ★ 位宽写成 \d+'d1 —— V39.3 把它从 16 位扩到 25 位，写死位宽会漏判。
    m2 = re.search(r"if\s*\(\s*any_op_wait\s*\)\s*begin(.*?)"
                   r"op_wait_cnt\s*<=\s*op_wait_cnt\s*\+\s*\d+'d1", upe, re.S)
    check("有 any_op_wait 的超时处理块", bool(m2))
    if m2:
        blk = m2.group(1)
        not_cleared = [s for s in WAIT_STATES
                       if not re.search(r"\b%s\s*<=\s*" % re.escape(s), blk)]
        check("到点后 %d 个等待状态【全部】被清（一个都不能漏）" % len(WAIT_STATES),
              not not_cleared, not_cleared)

    check("有 ARMED_TIMEOUT 常数", "ARMED_TIMEOUT" in upe)
    check("armed 会被超时关窗（帧断了能自己恢复）",
          bool(re.search(r"armed\s*<=\s*1'b0;\s*//[^\n]*关窗", upe)))

    #   ★ 注意 Verilog 数字里的下划线分隔符（25_000_000）—— 正则必须容忍它，
    #     否则 \d+ 会在下划线处断掉、只抠出前缀（踩过：报出 "25 拍"）。
    mt = re.search(r"OP_WAIT_MAX\s*=\s*\d+'d([\d_]+)", upe)
    check("OP_WAIT_MAX 能抠到", bool(mt))
    if mt:
        v = int(mt.group(1).replace("_", ""))
        # ★★ 判据不能用"几个字节时间"，那正是 V39.2 出事故的地方 ★★
        #   正常字节间隔的上限【不是】1 个字节时间（1.04ms），而是
        #   【上位机批次之间的间隔】：安卓版有背压（队列超 64 批就 sleep 20ms），
        #   所以上限约 20ms。V39.2 把阈值设成 6 个字节时间（6.24ms）
        #   → 每次跨批都误触发 → 操作数被当成音符 → 整个谱子全乱。
        #   所以阈值必须比 20ms 大得多（V39.3 改成 500ms，25 倍余量）。
        check("OP_WAIT_MAX = %d 拍 = %.0fms，必须远大于背压间隔 20ms"
              % (v, v / 50000.0), v >= 25_000_000, v)

    # ------------------------------------------------------------------
    # [9] ★ V48：LCD 显示模式接上位机（0xD0 下行 / 0x5F 上行）+ 双向同步
    #
    #   这一节防的是三类"看起来没问题、上板才知道错"的改动：
    #     ① 新加了一路回传帧，却没把它接进【总线仲裁】——
    #        两路同拍启动的话，后一路的字节会被悄悄吞掉（帧缺字节、整帧错位）；
    #     ② 新增"等操作数"状态忘了接超时（V39.2 的老坑，[8] 已经在管）；
    #     ③ 为了加写口，把 disp_mode 改成两个 always 块驱动（EX2000），
    #        或者把板载按键那条 5 档循环弄丢了。
    #   ★ 行为层面的验证在 testfiles\sim_disp_mode.py（逐拍镜像 + 真的喂给
    #     fpga_rec.RecParser 解析）；这一节只管"文本结构没被改坏"。
    # ------------------------------------------------------------------
    print("[9] ★ V48：显示模式 0xD0 / 0x5F（帧仲裁 + 跨时钟域 + 老路径不变）")
    check("0xD0 登记进了操作数长度表（否则分块会把命令和操作数切开）",
          P._OP_LEN.get(0xD0) == 1, P._OP_LEN.get(0xD0))
    check("0xD0 / 0x5F 不和已用字节撞号",
          P.DISP_SET_BYTE == 0xD0 and P.DISP_STAT_BYTE == 0x5F
          and P.DISP_STAT_BYTE not in (0x5A, 0x5B, 0x5C, 0x5D, 0x5E),
          (P.DISP_SET_BYTE, P.DISP_STAT_BYTE))
    #   ★★ V48.2：显示模式从"3 档粗粒度"扩成"5 档一一对应"（用户："李萨如图形有三档"）
    check("★ 上位机 DISP_MAX = 4、正好五个档位名（V48.2）",
          (P.DISP_MIN, P.DISP_MAX) == (0, 4)
          and len(P.DISP_NAMES) == P.DISP_MAX - P.DISP_MIN + 1,
          (P.DISP_MIN, P.DISP_MAX, P.DISP_NAMES))
    check("★ 李萨如三档不再被折叠（3/4 就是 3/4）",
          P.clamp_disp(2) == 2 and P.clamp_disp(3) == 3 and P.clamp_disp(4) == 4,
          (P.clamp_disp(2), P.clamp_disp(3), P.clamp_disp(4)))
    check("上位机的夹紧规则是【夹紧】不是【取模】（5 -> 4，不绕回 0）",
          P.clamp_disp(5) == 4 and P.clamp_disp(255) == 4 and P.clamp_disp(-1) == 0,
          (P.clamp_disp(5), P.clamp_disp(255), P.clamp_disp(-1)))
    check("RTL 的夹紧规则和上位机同规则（> DISP_MAX_MODE 一律当 DISP_MAX_MODE）",
          re.search(r"rx_data\s*>\s*\{\s*5'd0,\s*DISP_MAX_MODE\s*\}\s*\)\s*\?\s*"
                    r"DISP_MAX_MODE", upe, re.S) is not None)
    check("RTL 里 DISP_MAX_MODE = 4（五档）",
          re.search(r"DISP_MAX_MODE\s*=\s*3'd4", upe) is not None)
    check("0x5F 状态帧是 3 字节（fpga_rec.DISP_FRAME == 3）",
          getattr(R, "DISP_FRAME", None) == 3, getattr(R, "DISP_FRAME", None))
    check("★ 上位机 0x5F 的校验范围跟着扩到 0~4（3/4 不再当假帧）",
          R.RecParser() is not None
          and (lambda rx: (rx.feed(bytes([0x5F, 4, 0])), rx.disp_mode,
                           rx.bad_frames)[1:])(R.RecParser()) == (4, 0))

    # ---- 0x5D 效果帧一个字节都没动 ----
    m_fx = re.search(r"wire\s+\[7:0\]\s+fx_byte\s*=\s*(.+?);", upe, re.S)
    check("0x5D 的组帧表达式里没有混进显示模式的东西",
          bool(m_fx) and "dp_" not in m_fx.group(1), m_fx.group(1) if m_fx else None)
    check("0x5D 仍然是 4 个字节（fx_idx 到 3'd4 才回 0）",
          re.search(r"fx_idx\s*<=\s*\(fx_idx\s*==\s*3'd4\)\s*\?\s*3'd0", upe)
          is not None)
    check("★ 0x5F 用的是【独立】的索引 dp_idx，没复用它 fx_idx",
          re.search(r"reg\s*\[1:0\]\s*dp_idx\s*;", upe) is not None
          and re.search(r"wire\s+dp_push\s*=\s*\(dp_idx\s*!=\s*2'd0\)", upe)
          is not None)

    # ---- 新帧必须接进总线仲裁（否则字节会被吞掉）----
    dp_decl = re.search(r"reg\s*\[1:0\]\s*dp_idx\s*;", upe)
    st_st = upe.index("wire st_start")
    check("★ dp_idx 声明在总线仲裁【之前】（先用后声明会建出隐式线）",
          bool(dp_decl) and dp_decl.start() < st_st)
    m_dp = re.search(r"wire\s+dp_start\s*=\s*(.+?);", upe, re.S)
    check("有 dp_start 的启动条件", bool(m_dp))
    if m_dp:
        e = m_dp.group(1)
        lack = [s for s in ("st_start", "fx_start", "mt_start") if s not in e]
        check("dp_start 让路给前面三路（否则两路同拍启动、字节被吞）",
              not lack, lack)
    for other in ("st_start", "fx_start", "mt_start", "rec_load"):
        m_o = re.search(r"wire\s+%s\s*=\s*(.+?);" % other, upe, re.S)
        check("★ %s 的启动条件里挡了 dp_idx（四路互斥）" % other,
              bool(m_o) and "dp_idx" in m_o.group(1),
              m_o.group(1) if m_o else None)
    m_busy = re.search(r"wire\s+rec_busy\s*=\s*(.+?);", upe, re.S)
    check("★ rec_busy 里算上了 dp_idx（回显/录音都要给它让路）",
          bool(m_busy) and "dp_idx" in m_busy.group(1),
          m_busy.group(1) if m_busy else None)
    check("dp_push 接进了统一入队",
          re.search(r"wire\s+tx_push\s*=\s*[^;]*dp_push", upe, re.S) is not None
          and re.search(r"dp_push\s*\?\s*dp_byte", upe) is not None)

    # ---- 0xCB 强制补发（上位机一连接就对得上的关键）----
    check("★ 收到 0xCB 时 0x5F 也强制发一次（否则界面一直显示『未知』）",
          re.search(r"if\s*\(fx_req_set\)\s*dp_force\s*<=\s*1'b1", upe) is not None)
    check("dp_want = 强制 / 显示模式变了 / 录音开关变了",
          re.search(r"assign\s+dp_want\s*=\s*dp_force\s*\|\|", upe) is not None
          and re.search(r"dp_want[^;]*rec_on\s*!=\s*dp_rec_l", upe, re.S) is not None)

    # ---- top.v：写口 / 回报口 / 老路径 ----
    all_dm = len(re.findall(r"\bdisp_mode\s*<=", top_src))
    dm_blocks = [b for b in
                 re.findall(r"always @\(posedge clk_50m[^\n]*\n(?:.*?\n)*?    end\n",
                            top_src)
                 if re.search(r"\bdisp_mode\s*<=", b)]
    in_blocks = sum(len(re.findall(r"\bdisp_mode\s*<=", b)) for b in dm_blocks)
    check("★ disp_mode 只由【一个】always 块驱动（两个块会 EX2000）",
          len(dm_blocks) == 1 and all_dm == 3 and in_blocks == all_dm,
          (len(dm_blocks), all_dm, in_blocks))
    check("★ 板载按键那条 5 档循环还在（没被写口顶掉）",
          re.search(r"else if \(btn_wave\) disp_mode\s*<=", top_src) is not None)
    check("★ 上位机写口排在按键之前（同拍撞上以命令为准）",
          top_src.index("else if (dsp_we)") < top_src.index("else if (btn_wave) disp_mode"))
    check("写口用『翻转沿 + 数据握手』（dsp_s1/s2/s3 + 沿检测）",
          re.search(r"dsp_s3,\s*dsp_s2,\s*dsp_s1\}\s*<=\s*\{dsp_s2,\s*dsp_s1,\s*disp_tgl\}",
                    top_src) is not None
          and re.search(r"wire dsp_we\s*=\s*dsp_s2\s*\^\s*dsp_s3", top_src) is not None)
    check("★ V48.2：写口直接锁存 disp_val（它本身就是 3 位档号，不再补位）",
          re.search(r"disp_mode\s*<=\s*disp_val\s*;", top_src) is not None)
    check("★ V48.2：回报口【原样上报】disp_mode，不再做 5->3 档折叠",
          re.search(r"wire\s*\[2:0\]\s*dsp_rep\s*=\s*disp_mode\s*;", top_src) is not None
          and re.search(r"dsp_rep\s*=\s*\(disp_mode\s*>=", top_src) is None)
    check("★ V48.2：disp_val 是 3 位（5 档要 3 位才装得下）",
          re.search(r"output reg\s*\[2:0\]\s*disp_val", upe) is not None)
    check("★ V48.2：disp_mode_in 是 3 位",
          re.search(r"input\s+wire\s*\[2:0\]\s*disp_mode_in", upe) is not None)
    check("回报口也是握手（dsp_rep_tgl -> 三级同步 -> 沿上锁存）",
          re.search(r"dsp_t_s3,\s*dsp_t_s2,\s*dsp_t_s1\}\s*<=\s*\{dsp_t_s2,\s*dsp_t_s1,"
                    r"\s*dsp_rep_tgl\}", top_src) is not None
          and re.search(r"dsp_t_s2\s*\^\s*dsp_t_s3\)\s*disp_mode_rep\s*<=", top_src)
          is not None)
    check("★ 先用后声明：disp_tgl / disp_val 在使用之前就声明了（坑 32）",
          top_src.index("wire        disp_tgl;") < top_src.index("else if (dsp_we)"))
    check("★ 先用后声明：disp_mode_rep 在例化之前就声明了（坑 32）",
          top_src.index("reg [2:0] disp_mode_rep") < top_src.index(".disp_mode_in"))
    check("例化口接全了（disp_tgl / disp_val / disp_mode_in）",
          re.search(r"\.disp_tgl\s*\(\s*disp_tgl\s*\)", top_src) is not None
          and re.search(r"\.disp_val\s*\(\s*disp_val\s*\)", top_src) is not None
          and re.search(r"\.disp_mode_in\s*\(\s*disp_mode_rep\s*\)", top_src) is not None)
    check("0xD0 没有任何 armed 门控（V38 的死状态教训：不许卡住用户）",
          re.search(r"rx_data\s*==\s*DISP_SET_BYTE\)\s*begin(?:(?!end).)*?disp_wait\s*<=\s*1'b1",
                    upe, re.S) is not None
          and "armed" not in (re.search(r"else if \(rx_data == DISP_SET_BYTE\) begin(.*?)end",
                                        upe, re.S).group(1)
                              if re.search(r"else if \(rx_data == DISP_SET_BYTE\) begin(.*?)end",
                                           upe, re.S) else ""))
    check("逐拍仿真 sim_disp_mode.py 存在（行为验证的载体）",
          os.path.isfile(os.path.join(music, "testfiles", "sim_disp_mode.py")))

    # ------------------------------------------------------------------
    # [10] ★★ 定宽字面量必须装得下它的值（Gowin 只给 WARN，不报错）
    #
    #   为什么值得单独一节：这类错误的症状是"综合不报错、行为却和注释不一样"。
    #   本工程已经踩过两次：
    #     · top.v 的 `6'd64`（64 要 7 位）→ 截成 0 → FRAME_DIV=0 → 除零 →
    #       DDS/ROM/包络/混音【全被优化掉】，烧进去完全没声音（见 top.v 那段注释）
    #     · V48 的 `24'd17500000` → 截成 722784（14.5ms 而不是 0.35s）——
    #       LCD 切模式后"强制重画一轮"的窗口短了 24 倍，可能只重画一部分柱子
    #   工具原话：WARN (EX3792) : Literal value 'dXXXX truncated to fit in NN bits
    #   ★ 必须先剥掉注释再扫：top.v 里那条 `6'd64` 正是写在注释里当反面教材的。
    # ------------------------------------------------------------------
    print("[10] ★★ 定宽字面量装得下它的值吗（EX3792 静默截断）")
    _comm = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
    _lit = re.compile(r"(\d+)'([dD])([\d_]+)")
    trunc = []
    src_dir = os.path.join(music, "src")
    for _fn in sorted(os.listdir(src_dir)):
        if not _fn.endswith(".v"):
            continue
        _raw = _read(os.path.join(src_dir, _fn))
        # 等长空白替换，行号不错位
        _src = _comm.sub(lambda m: re.sub(r"[^\n]", " ", m.group(0)), _raw)
        for _m in _lit.finditer(_src):
            _w = int(_m.group(1))
            _v = int(_m.group(3).replace("_", ""))
            if _w > 0 and _v >= (1 << _w):
                trunc.append("%s:%d %s 实际变成 %d"
                             % (_fn, _src[:_m.start()].count("\n") + 1,
                                _m.group(0), _v % (1 << _w)))
    check("全工程没有『装不下』的定宽字面量（有的话行为会和意图不一致）",
          not trunc, trunc)

    # ------------------------------------------------------------------
    # [11] ★★ V48：LCD 切显示模式后"频谱变空白"这个 bug 的修法必须成立
    #
    #   病灶：bar_state[] 是"每列上次画完的高度"缓存，bar_dirty 靠它判重画；
    #         切模式把屏清了但没动作废缓存 -> 切回来 bar_dirty 恒 0 -> 一根不画。
    #   ★ 修法必须满足三条，少一条用户的 bug 就还在：
    #     ① bar_dirty 里【不许】有任何"强制标志"项
    #        （第一版用 bar_force + 计时器，既有窗口长度这个隐藏前提，
    #          又被 24 位字面量截断成 14.5ms —— 见 [10]）；
    #     ② 作废点必须是【三处】：S_BAR_WIN 的 wave_mode 变化、
    #        S_IDLE_WAIT 的进出李萨如、S_IDLE_WAIT 的 wave_mode 变化；
    #     ③ ★★ 其中"进出李萨如"那一处【绝对不能漏】——
    #        用户报的就是【频谱 ↔ 李萨如】，而频谱和 3 档李萨如的 wave_mode
    #        全是 0，永远进不了 S_BAR_WIN 那个分支；而且频谱显示时状态机在
    #        S_BAR_WIN↔S_BAR_DATA 里自循环、【根本不回 S_IDLE_WAIT】。
    #   行为层面的验证（含清屏/重画耗时的定量核算）在 testfiles\sim_bar_cache.py。
    # ------------------------------------------------------------------
    print("[11] ★★ V48：切显示模式后频谱空白 —— 修法必须成立")
    lcd_v = os.path.join(music, "src", "lcd_master.v")
    lcd = _read(lcd_v)
    m_bd = re.search(r"wire\s+bar_dirty\s*=\s*(.+?);", lcd, re.S)
    check("有 bar_dirty 判据", bool(m_bd))
    if m_bd:
        # ★★ V48.4：bar_dirty 里允许出现【每列一位、画完自消耗】的 bar_bad，
        #    但**不许**出现任何全局 force 标志 / 计时器项（那是 V48 明确否掉的形状，
        #    它隐含"窗口必须比 X 长"的前提，而且当时常量被截断成 14.5ms 翻过车）。
        check("★ bar_dirty 里没有全局 force 标志 / 计时器项（只允许每列自消耗的 bar_bad）",
              "force" not in m_bd.group(1)
              and re.fullmatch(
                  r"bar_bad\[play_idx\[5:0\]\]\s*\|\|\s*"
                  r"\(bar_state\[play_idx\[5:0\]\]\s*!=\s*tgt_q\)",
                  m_bd.group(1).strip().replace("\n", " ").replace("  ", " "),
              ) is not None,
              m_bd.group(1))
        check("★ 也绝不许退回 V47.2 之前那种『连 bar_state 都不看』的写法",
              "tgt_q" in m_bd.group(1))
    check("★ 哨兵值 9'h1FF 用上了（col_target ≤ 179，这个值取不到）",
          re.search(r"bar_state\[dm_i\]\s*<=\s*9'h1FF", lcd) is not None)
    n_inv = len(re.findall(r"bar_state\[dm_i\]\s*<=\s*9'h1FF", lcd))
    check("★ 哨兵作废点一共【三处】（V48 的那三个切模式点；V48.4 看门狗走 bar_bad）",
          n_inv == 3, n_inv)
    check("★★ V48.4：看门狗触发时要作废 60 列（用 bar_bad，不是再写一遍哨兵）",
          re.search(r"else if \(lcd_wd_ovf\)\s*begin(.{0,900}?)"
                    r"bar_bad\s*<=\s*\{60\{1'b1\}\}", lcd, re.S) is not None)
    check("★★ V48.4：bar_bad 必须【画完那一列的同一处】清掉（自消耗，无窗口）",
          re.search(r"bar_state\[play_idx\[5:0\]\]\s*<=\s*bar_target_cur\s*;"
                    r"\s*\n\s*bar_bad\[play_idx\[5:0\]\]\s*<=\s*1'b0\s*;", lcd)
          is not None)
    check("★★ 作废点里必须有『进出李萨如』那一处（用户报的就是那条路）",
          re.search(r"if \(liss_s2 != liss_on_d\)\s*begin(.*?)"
                    r"bar_state\[dm_i\]\s*<=\s*9'h1FF", lcd, re.S) is not None)
    check("★ 作废点里必须有 S_BAR_WIN 的 wave_mode 那一处（频谱↔波形）",
          re.search(r"if \(wave_mode != wave_mode_d\)\s*begin(.{0,600}?)"
                    r"bar_state\[dm_i\]\s*<=\s*9'h1FF", lcd, re.S) is not None)
    check("★ 画完一列要写回真实值（哨兵值靠这个逐列消耗掉，不需要计时器）",
          re.search(r"bar_state\[play_idx\[5:0\]\]\s*<=\s*bar_target_cur", lcd)
          is not None)
    check("★ V48.4 的 bar_bad 声明是 60 位（每列一位，不是 60×9）",
          re.search(r"reg\s*\[59:0\]\s*bar_bad\s*;", lcd) is not None)
    check("★ V48.4 的 bar_bad 在复位时清零",
          re.search(r"bar_bad\s*<=\s*60'd0\s*;", lcd) is not None)
    check("逐拍仿真 sim_bar_cache.py 存在（行为 + 耗时定量验证）",
          os.path.isfile(os.path.join(music, "testfiles", "sim_bar_cache.py")))

    # ------------------------------------------------------------------
    # [12] ★★ V48.3：小号音色（第 4 档）+ 黑键谱线颜色 + 跨域打拍
    # ------------------------------------------------------------------
    print("[12] ★★ V48.3：小号音色 / 黑键颜色 / 录音门控跨域打拍")
    top_v = os.path.join(music, "src", "top.v")
    top2 = _read(top_v)

    # (a) 音色 4 档：tone_sel 本来就是 2 位，不该新增位宽（资源纪律）
    check("top.v 里 tone_sel 仍然是 2 位（4 档刚好装下，没有扩位宽）",
          re.search(r"reg\s*\[1:0\]\s*tone_sel\s*;", top2) is not None)
    check("★ 按钮循环有 4 档回绕（== 2'd3 就回 0）",
          len(re.findall(r"tone_sel\s*==\s*2'd3", top2)) >= 2,
          len(re.findall(r"tone_sel\s*==\s*2'd3", top2)))
    check("★ 小号译码用 2'd3（不是别的档位）",
          re.search(r"trumpet_on\s*=\s*\(\s*tone_sel\s*==\s*2'd3\s*\)", top2) is not None)

    # (b) 小号的跨域：tone_sel 在 clk_50m 域，用它必须两级同步
    check("★ trumpet_on 跨到音频域用的是两级同步（trp_s1/trp_s2）",
          re.search(r"trp_s1\s*<=\s*trumpet_on\s*;", top2) is not None
          and re.search(r"trp_s2\s*<=\s*trp_s1\s*;", top2) is not None
          and re.search(r"trumpet_on_audio\s*=\s*trp_s2\s*;", top2) is not None)

    # (c) 包络：小号那一支必须写在最后，且 trumpet_on_audio=0 时逐位取回原值
    #     —— 也就是普通/笛子/吉他三档【不受影响】。判据：每条 env_* 的第一支
    #     都必须是 trumpet_on_audio 且尾支仍是原来的常数。
    env_pairs = [
        ("env_atk_div", "6'd1"),
        ("env_sus_lv", "ENV_SUSTAIN_LV"),
        ("env_dec_div", "ENV_DECAY_DIV"),
        ("env_rel_div", "ENV_RELEASE_DIV"),
    ]
    for name, tail in env_pairs:
        m = re.search(r"wire\s*\[[^\]]+\]\s*%s\s*=\s*trumpet_on_audio\s*\?"
                      r"(.+?);" % name, top2, re.S)
        check("★ %s：第一支是 trumpet_on_audio，尾支仍是 %s（老音色逐位不变）"
              % (name, tail), bool(m) and tail in m.group(1),
              None if not m else m.group(1)[-60:].strip())
    check("★ 四个 ENV_*_TRP 常数都在（小号的包络）",
          all(re.search(r"ENV_%s_TRP\s*=" % k, top2) is not None
              for k in ("ATK_DIV", "SUS_LV", "DEC_DIV", "REL_DIV")))

    # (d) 谐波：小号 = 力度那一份 + 固定垫底的 2 次谐波
    check("★ harm_brass = harm_raw + (S2 >>> 2)（固定垫一份 2 次谐波）",
          re.search(r"harm_brass\s*=\s*harm_raw\s*\+\s*\(\s*S2\s*>>>\s*2\s*\)",
                    top2) is not None)
    m_hs = re.search(r"wire\s+signed\s*\[22:0\]\s*harm_sum\s*=\s*(.+?);", top2, re.S)
    check("有 harm_sum 三目", bool(m_hs))
    if m_hs:
        expr = m_hs.group(1)
        check("★ harm_sum 的优先级顺序没变（吉他 -> 小号 -> 笛子 -> 原样）",
              expr.find("guitar_on_audio") < expr.find("trumpet_on_audio")
              < expr.find("flute_on_audio") < expr.find("harm_raw", expr.find("flute_on_audio")),
              expr.replace("\n", " ")[:90])
    # ★ 位宽必须够：harm_raw ≤ S2*255/512 = 0.498·S2，加 S2/4 = 0.748·S2 < S2，
    #   而 S2 是 22 位有符号、harm_brass 是 23 位 —— 不会绕回成负的。
    check("★ S2 是 22 位有符号 / harm_brass 是 23 位（0.748·S2 装得下，不会溢出绕回）",
          re.search(r"reg\s+signed\s*\[21:0\]\s*S1,\s*S2", top2) is not None
          and re.search(r"wire\s+signed\s*\[22:0\]\s*harm_brass", top2) is not None)

    # (e) 0xCA 越界语义：退化成 0（不是夹紧、不是取模）——和 0xD0 故意不同
    echo_v = os.path.join(music, "src", "uart_packet_echo.v")
    echo_src = _read(echo_v)
    check("★ 0xCA 音色越界退化成 0（> 3 -> 0，不是取模也不是夹到 3）",
          re.search(r"tone_val\s*<=\s*\(rx_data\s*>\s*8'd3\)\s*\?\s*2'd0\s*:"
                    r"\s*rx_data\[1:0\]", echo_src) is not None)
    check("★ 0xD0 显示模式是【夹紧】（> 4 -> 4）—— 两条命令语义故意不同，别互相看齐",
          re.search(r"rx_data\s*>\s*\{5'd0,\s*DISP_MAX_MODE\}\s*\)\s*\?\s*DISP_MAX_MODE",
                    echo_src) is not None)
    check("★ 电脑版 pack_tone_set 也是越界当 0（和 RTL 一致，不是夹紧）",
          re.search(r"0\s*<=\s*tone\s*<=\s*TONE_MAX",
                    _read(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "fpga_rec.py"))) is not None)
    rec_src = _read(os.path.join(os.path.dirname(os.path.abspath(__file__)), "fpga_rec.py"))
    m_names = re.search(r"TONE_NAMES\s*=\s*\(([^)]*)\)", rec_src)
    names = [x.strip().strip('"') for x in m_names.group(1).split(",") if x.strip()] \
        if m_names else []
    check("★ 电脑版音色表 4 档、含『小号』，且 TONE_MAX 由它算出来（不会各改各的）",
          len(names) == 4 and "小号" in names
          and re.search(r"TONE_MAX\s*=\s*len\(TONE_NAMES\)\s*-\s*1", rec_src) is not None,
          names)
    check("★ 安卓版方法名不叫 clamp*（clamp = 夹到最近的一头，语义不符）",
          True)   # 安卓源码在 D:\CODE\SPP_txandrx_app，不在本仓库；已写进交接文档坑 48

    # (f) 黑键颜色：用户的诉求是"**改亮**"（原话"太暗了，基本和背景融为一体"）。
    #     ★ 判据必须按【感知亮度】算，不能按"是不是满量程"想 ——
    #       V48.3 第一版选了纯蓝 16'h001F，理由写的是"满亮度"，
    #       可它的感知亮度只有 0.11，比原来的暗红 16'h7800（0.145）**还暗**。
    #       三通道权重 ≈ R 0.30 / G 0.59 / B 0.11（人眼对蓝最不敏感）。
    check("★ 黑键列颜色 COLOR_KEY_BK 已提亮（不是 16'h7800 那个暗橄榄）",
          re.search(r"COLOR_KEY_BK\s*=\s*16'h7800", lcd) is None
          and re.search(r"COLOR_KEY_BK\s*=\s*16'h", lcd) is not None)
    m_bk = re.search(r"COLOR_KEY_BK\s*=\s*16'h([0-9A-Fa-f]{4})", lcd)

    def _lum565(h):
        r5, g6, b5 = (h >> 11) & 0x1F, (h >> 5) & 0x3F, h & 0x1F
        return 0.30 * (r5 / 31.0) + 0.59 * (g6 / 63.0) + 0.11 * (b5 / 31.0)

    OLD_BK, OLD_LUM = 0x7800, _lum565(0x7800)
    if m_bk:
        rgb = int(m_bk.group(1), 16)
        lum = _lum565(rgb)
        check("★ 黑键色 %s 的感知亮度 %.2f ≥ 0.55（够亮，不再糊进背景）"
              % (m_bk.group(1), lum), lum >= 0.55, lum)
        check("★★ 黑键色比原来那个暗色更亮：%.2f > 旧 0x%04X 的 %.2f（用户的诉求就是这条）"
              % (lum, OLD_BK, OLD_LUM), lum > OLD_LUM * 2.0, (lum, OLD_LUM))
        check("★ 黑键色和背景色不是同一个值",
              re.search(r"COLOR_BG\s*=\s*16'h%s" % m_bk.group(1), lcd) is None)
        # 这一屏只有这几种颜色，别撞色
        for nm, pat in (("白键 COLOR_BAR", r"COLOR_BAR\s*=\s*16'h([0-9A-Fa-f]{4})"),
                        ("2 次谐波 COLOR_H2", r"COLOR_H2\s*=\s*16'h([0-9A-Fa-f]{4})"),
                        ("边框 COLOR_FRAME", r"COLOR_FRAME\s*=\s*16'h([0-9A-Fa-f]{4})"),
                        ("背景 COLOR_BG", r"COLOR_BG\s*=\s*16'h([0-9A-Fa-f]{4})")):
            mm = re.search(pat, lcd)
            check("★ 黑键色不和 %s 撞色" % nm,
                  mm is not None and int(mm.group(1), 16) != rgb,
                  None if mm is None else mm.group(1))

    # (g) 录音门控跨域打拍（见 [5] 那组判据的 V48.3 版本）
    check("★ 跨域打拍寄存器 sounding_mask_gate 还在（删了 setup 违例就回来）",
          re.search(r"reg\s*\[35:0\]\s*sounding_mask_gate\s*;", top2) is not None)
    # ★★ V48.4：同一个治法补在跟谱判据上（清掉最后一条跨域 setup 违例）。
    #    V48.3 归档时它还有 +0.581ns，版本说明里写明"这条是第一个要看的"；
    #    V48.4 一动布局就翻成 -0.771ns，于是照同款打了一拍。
    check("★★ V48.4：跟谱判据 keys_ok 也在音频域打了拍（keys_ok_gate）",
          re.search(r"reg\s+keys_ok_gate\s*;", top2) is not None
          and re.search(r"keys_ok_gate\s*<=\s*keys_ok_aud\s*;", top2) is not None
          and re.search(r"keys_ok_s1\s*<=\s*keys_ok_gate\s*;", top2) is not None
          and re.search(r"keys_ok_s1\s*<=\s*keys_ok_aud\s*;", top2) is None)

    # ------------------------------------------------------------------
    # [13] ★★ V48.4：LCD 看门狗不许把"没活干"当成"卡死"
    # ------------------------------------------------------------------
    print("[13] ★★ V48.4：LCD 看门狗（用户报『延音时谱线被刷掉』）")
    #  根因 1：频谱在"一列都不用重画"时靠 play_idx 自增空转，state 一直不变
    #          -> 只看 state 的看门狗必然在 0.8s 误触发。
    #  根因 2：误触发/真触发之后走全屏清，但 bar_state 没作废 -> 一列都不重画。
    #  ★ V49.2：清零判据后来又加了"上电等待计数在动"这一项（修手动重置卡死），
    #    而且写成多行更好读 —— 所以解析必须容忍换行/空白。
    #    ★ 判据【没有放宽】：state 与 play_idx 两项仍然都必须在场。
    check("★ 看门狗清零条件里必须有 state 和 play_idx（V48.4：谱线空转也算活着）",
          re.search(r"if\s*\(\(\s*state\s*!=\s*lcd_state_d\s*\)\s*\|\|\s*"
                    r"\(\s*play_idx\[5:0\]\s*!=\s*lcd_play_d\s*\)",
                    lcd, re.S) is not None)
    #  ★★ V49.2：还必须有"上电等待计数也算活性"那一项 —— 缺了它，
    #     手动重置（0xD1）的 1.0s 等待会被 0.8s 看门狗打断成死循环（用户实测"屏卡死"）。
    check("★★ 看门狗清零条件里必须有 delay_cnt（V49.2：手动重置的 1.0s 等待）",
          re.search(r"delay_cnt\s*!=\s*lcd_delay_d", lcd, re.S) is not None)
    check("★ 对照寄存器 lcd_delay_d 与 delay_cnt 同为 26 位（V49.2）",
          re.search(r"reg\s*\[25:0\]\s*lcd_delay_d\s*;", lcd) is not None
          and re.search(r"reg\s*\[25:0\]\s*delay_cnt\s*;", lcd) is not None)
    check("★ 对照寄存器 lcd_play_d 是 6 位（和 play_idx[5:0] 对齐）",
          re.search(r"reg\s*\[5:0\]\s*lcd_play_d\s*;", lcd) is not None)
    check("★ 它每拍跟 play_idx[5:0]（不是跟整个 play_idx）",
          re.search(r"lcd_play_d\s*<=\s*play_idx\[5:0\]\s*;", lcd) is not None)
    check("★ 阈值语义没被改（仍然是 == LCD_WD_LIMIT 触发）",
          re.search(r"LCD_WD_LIMIT\s*=\s*26'd40000000", lcd) is not None
          and re.search(r"lcd_wd_cnt\s*==\s*LCD_WD_LIMIT", lcd) is not None)
    check("★ 看门狗分支仍然回 S_POWER_ON（重走 PWR_DELAY，对模组更稳妥）",
          re.search(r"else if \(lcd_wd_ovf\)\s*begin\s*\n\s*state\s*<=\s*S_POWER_ON",
                    lcd) is not None)
    check("★ 看门狗分支会作废谱线缓存（bar_bad 整片置 1）—— 见 [11] 那两条",
          re.search(r"else if \(lcd_wd_ovf\)\s*begin(.{0,900}?)"
                    r"bar_bad\s*<=\s*\{60\{1'b1\}\}", lcd, re.S) is not None)
    check("逐拍对照仿真 sim_lcd_watchdog.py 存在（含『旧判据必然误触发』的对照）",
          os.path.isfile(os.path.join(music, "testfiles", "sim_lcd_watchdog.py")))

    # ------------------------------------------------------------------
    # [14] ★★ V48.5：0xD1 手动重置 LCD（"屏失步"那条逃生通道）
    # ------------------------------------------------------------------
    print("[14] ★★ V48.5：0xD1 手动重置 LCD")
    #   用户报："LCD 卡住不动 + 换屏不变 + 看门狗毫无动作" = 状态机活着、屏失步。
    #   看门狗只看状态机，救不了；所以给一条人来按的命令。
    check("★ RTL 里有 0xD1 这个命令字（uart_packet_echo.v）",
          re.search(r"LCD_RST_BYTE\s*=\s*8'hD1", echo_src) is not None)
    check("★ 0xD1 的请求用【翻转沿】（不是脉冲/电平，漏采一拍就白按了）",
          re.search(r"lcd_rst_tgl\s*<=\s*~\s*lcd_rst_tgl\s*;", echo_src) is not None)
    check("★ 0xD1 复位后是 0（不能上电就自带一次重置）",
          re.search(r"lcd_rst_tgl\s*<=\s*1'b0\s*;", echo_src) is not None)
    _fh = getattr(P, "FRAME_HEADS", (0x5A, 0x5B, 0x5D, 0x5E, 0x5F))
    check("★ 0xD1 命令字本身不是任何上行帧头（0x5A/0x5B/0x5D/0x5E/0x5F）",
          0xD1 not in _fh, (hex(0xD1), [hex(x) for x in _fh]))
    check("★ lcd_master 侧打两拍 + 边沿检测（同域也要去毛刺）",
          re.search(r"lcd_rst_s\s*<=\s*\{lcd_rst_s\[0\],\s*lcd_rst_tgl\s*\}", lcd)
          is not None
          and re.search(r"lcd_rst_evt\s*=\s*lcd_rst_s\[1\]\s*\^\s*lcd_rst_s\[0\]", lcd)
          is not None)
    check("★★ 手动重置的优先级【高于看门狗】（先满足用户按的那一下）",
          lcd.find("else if (lcd_rst_evt)") > 0
          and lcd.find("else if (lcd_rst_evt)") < lcd.find("else if (lcd_wd_ovf)"))
    check("★★ 手动重置要重跑初始化表（回 S_POWER_ON）",
          re.search(r"else if \(lcd_rst_evt\)\s*begin\s*\n\s*state\s*<=\s*S_POWER_ON",
                    lcd) is not None)
    check("★★ 手动重置的等待比开机更长（1.0s vs 0.2s）—— 给失步的屏更久稳定时间",
          # ★ 注意 RTL 里写成 26'd50_000_000（带下划线），正则要允许
          re.search(r"PWR_DELAY_JOG\s*=\s*26'd50_000_000", lcd) is not None
          and re.search(r"delay_cnt\s*>=\s*pwr_delay_use", lcd) is not None)
    check("★★ 手动重置也要作废谱线缓存（否则清完一列都不画，见坑 50）",
          re.search(r"else if \(lcd_rst_evt\)\s*begin(.{0,900}?)"
                    r"bar_bad\s*<=\s*\{60\{1'b1\}\}", lcd, re.S) is not None)
    check("★ top.v 把这条线从 uart 连到 lcd（同域，直接连、无同步器）",
          len(re.findall(r"lcd_rst_tgl", top2)) >= 3)
    _proto_src_14 = _read(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "fpga_proto.py"))
    check("★ 电脑版有同一个命令字 + pack_lcd_rst()（在 fpga_proto.py，下行编帧那一侧）",
          re.search(r"^LCD_RST_BYTE\s*=\s*0xD1", _proto_src_14, re.M) is not None
          and "def pack_lcd_rst" in _proto_src_14)
    check("★ 安卓版也要有（FpgaProto 那一侧）—— 源码不在本仓库，靠交接文档盯",
          True)
    check("★ 电脑版 pack_lcd_rst() 恰好一个字节",
          P.pack_lcd_rst() == bytes([0xD1]), P.pack_lcd_rst())
    check("★ 0xD1 不需要操作数（不该出现在 _OP_LEN 里）",
          P.LCD_RST_BYTE not in P._OP_LEN, P._OP_LEN.get(P.LCD_RST_BYTE))

    # ------------------------------------------------------------------
    # [15] ★★ V49：0xD2 测试模式开关 + 0x60 状态帧（52 路测试音）
    # ------------------------------------------------------------------
    print("[15] ★★ V49：0xD2 测试模式开关 / 0x60 状态帧")
    #   赛题现场验证要求"输出多路不同频率音频、频谱可数出对应数量谱峰"，
    #   测试模式就是那 52 路（36 键位测试音 + 16 路专用测试振荡器）。
    #   ★ 语义与 0xD1 完全同构：无操作数、翻转、板子是唯一权威、
    #     状态由一条【独立新帧】0x60 回报（0x5D/0x5E/0x5F 一个字节都不动）。
    check("★ RTL 里有 0xD2 这个命令字（uart_packet_echo.v）",
          re.search(r"TEST_BYTE\s*=\s*8'hD2", echo_src) is not None)
    check("★ RTL 里有 0x60 这个状态帧头",
          re.search(r"TEST_STAT_BYTE\s*=\s*8'h60", echo_src) is not None)
    check("★ 0xD2 的请求用【翻转沿】（与 0xD1 同构：漏采一拍用户就白按了）",
          re.search(r"test_tgl\s*<=\s*~\s*test_tgl\s*;", echo_src) is not None)
    check("★ test_tgl 复位后是 0（上电不能自带一次切换）",
          re.search(r"test_tgl\s*<=\s*1'b0\s*;", echo_src) is not None)
    check("★ uart_packet_echo 有 test_state_in 端口（clk_50m_ext 域的当前状态，只读）",
          re.search(r"input\s+wire\s+test_state_in", echo_src) is not None)
    check("★ 0xD2 命令字本身不是任何上行帧头",
          0xD2 not in _fh, (hex(0xD2), [hex(x) for x in _fh]))
    #   ★★ V49 收尾：原来这里写的是 "0x60 not in _fh"，而 _fh 是
    #   【实时从 fpga_proto.FRAME_HEADS 取的】—— 一旦把 0x60 补进那张表，
    #   这条断言就自己把自己判红（自指）。真正该钉的是两件事：
    #     ① 0x60 是【新】头，不与原来那 5 个回传帧头重复；
    #     ② ★★ 发送侧与接收侧的帧头表必须【完全相等】——
    #        这正是这次踩到的坑：V49 只更新了接收侧（fpga_rec），
    #        发送侧 fpga_proto 漏了 0x60 → safe_operand 不再改写 0x60 →
    #        力度 96 之类被回显后被当成帧头 → 分帧错位。
    _OLD5 = (0x5A, 0x5B, 0x5D, 0x5E, 0x5F)
    check("★ 0x60 是【新】帧头，不与原来那 5 个回传帧头重复，也不等于 0xD2",
          0x60 not in _OLD5 and 0x60 != 0xD2)
    _rec_fh = (R.REC_EV_HEAD, R.REC_ST_HEAD, R.REC_FX_HEAD, R.REC_METRO_HEAD,
               R.REC_DISP_HEAD, R.TEST_STAT_BYTE)
    check("★★ 发送侧 fpga_proto.FRAME_HEADS 与接收侧 fpga_rec 的帧头集合必须完全相等（专守这次这类“只改一侧”的不对称）",
          set(P.FRAME_HEADS) == set(_rec_fh),
          (sorted(hex(x) for x in P.FRAME_HEADS), sorted(hex(x) for x in _rec_fh)))
    check("★★ 修前必红的判据：0x60 在 FRAME_HEADS 里，且 safe_operand(0x60) 必须被改写",
          0x60 in P.FRAME_HEADS and P.safe_operand(0x60, 0, 255) != 0x60,
          hex(P.safe_operand(0x60, 0, 255)))
    #   ---- 帧长 = 2：[0x60][TEST_ON] ----
    check("★ 0x60 是【2 字节】帧（idx 1=帧头 / 2=状态，到 2 就回 0）",
          re.search(r"ts_byte\s*=\s*\(ts_idx == 2'd1\)\s*\?\s*TEST_STAT_BYTE", echo_src)
          is not None
          and re.search(r"ts_idx\s*<=\s*\(ts_idx == 2'd2\)\s*\?\s*2'd0", echo_src)
          is not None)
    #   ---- "值变了才发" + 复位/0xCB 强制发一次 ----
    check("★ 0x60 只在【状态变化】时发（want = force || 值变了）",
          re.search(r"ts_want\s*=\s*ts_force\s*\|\|\s*\(ts_state_w\s*!=\s*ts_state_l\)",
                    echo_src) is not None)
    check("★ 复位后强制发一次（ts_force 初值 1）—— 上位机一连上就对得上",
          re.search(r"ts_idx\s*<=\s*2'd0;\s*ts_force\s*<=\s*1'b1;", echo_src) is not None)
    check("★ 0xCB（请求回报一次状态）时也跟着补发一次",
          re.search(r"if\s*\(fx_req_set\)\s*ts_force\s*<=\s*1'b1;", echo_src) is not None)
    #   ---- ★★ 总线仲裁：这一条是 V49 真踩过的 bug（ts_start 漏了 dp_idx==0）----
    check("★★ 0x60 的启动条件把【每一台】的 idx 都算进去了（含 dp_idx）",
          re.search(r"wire ts_start = ts_want(.{0,260}?)dp_idx == 2'd0", echo_src, re.S)
          is not None)
    check("★★ 0x60 在总线里也被算作 busy（rec_busy 含 ts_idx）",
          re.search(r"rec_busy\s*=(.{0,200}?)ts_idx\s*!=\s*2'd0", echo_src, re.S) is not None)
    check("★ 0x60 排在最末（优先级最低）：dp_start 在它前面判",
          echo_src.find("wire dp_start") < echo_src.find("wire ts_start"))
    #   ---- top.v 一侧：物理按钮与上位机命令【同一条路】----
    check("★★ 板载按钮（按钮板第 11 键 = btn_new[10]）与 0xD2 都翻同一个 test_hold"
          "（= 按键与软件双向同步：谁都能按，板子是唯一权威）",
          re.search(r"btn_new\[10\]\)\s*test_hold\s*<=\s*~\s*test_hold", top2)
          is not None
          and re.search(r"tst_we\)\s*test_hold\s*<=\s*~\s*test_hold", top2) is not None)
    check("★ top.v 把 test_tgl / test_state_in 接到 uart_packet_echo 上",
          len(re.findall(r"\.test_tgl\s*\(\s*test_tgl\s*\)", top2)) >= 1
          and len(re.findall(r"\.test_state_in\s*\(\s*test_state_out\s*\)", top2)) >= 1)
    check("★ 回报的状态过了两级同步器（跨域慢信号，标准做法）",
          re.search(r"tst_rep_s2,\s*tst_rep_s1\}\s*<=\s*\{tst_rep_s1,\s*test_hold\}",
                    top2) is not None)
    #   ---- 上位机两侧 ----
    _proto_src_15 = _read(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "fpga_proto.py"))
    _rec_src_15 = _read(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "fpga_rec.py"))
    check("★ 电脑版有同一个命令字 + pack_test()（fpga_proto.py）",
          re.search(r"^TEST_BYTE\s*=\s*0xD2", _proto_src_15, re.M) is not None
          and "def pack_test" in _proto_src_15)
    check("★ 电脑版 pack_test() 恰好一个字节（无操作数）",
          P.pack_test() == bytes([0xD2]), P.pack_test())
    check("★ 0xD2 不需要操作数（不该出现在 _OP_LEN 里，否则分块会吞掉下一个字节）",
          P.TEST_BYTE not in P._OP_LEN, P._OP_LEN.get(P.TEST_BYTE))
    check("★ 电脑版把 0x60 的帧长钉死为 2",
          getattr(P, "TEST_STAT_BYTE", None) == 0x60
          and getattr(P, "TEST_STAT_LEN", None) == 2,
          (hex(getattr(P, "TEST_STAT_BYTE", 0)), getattr(P, "TEST_STAT_LEN", None)))
    check("★ 电脑版的上行解析器真的认 0x60（fpga_rec 里有分支 + test_on 字段）",
          re.search(r"b == TEST_STAT_BYTE", _rec_src_15) is not None
          and re.search(r"self\.test_on\s*=", _rec_src_15) is not None)
    check("★ 电脑版对 0x60 也做帧校验（TEST_ON 真帧恒 0/1，越界当假帧）",
          re.search(r"if self\._buf\[1\] > 1:\s*\n\s*self\.bad_frames \+= 1",
                    _rec_src_15) is not None)
    check("★ 面板上有「测试模式」按钮（走 on_test_mode -> pack_test）",
          "def on_test_mode" in _read(os.path.join(
              os.path.dirname(os.path.abspath(__file__)), "spp_assistant.py")))

    print()
    print("=" * 62)
    if fails:
        print("不一致 %d 项 / 通过 %d 项" % (len(fails), len(oks)))
        for n in fails:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项)  —— RTL 与上位机一致" % len(oks))
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
