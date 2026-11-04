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
    check("音色档数 = 3（普通/笛子/吉他）", len(R.TONE_NAMES) == 3, R.TONE_NAMES)
    # ★ 关键性质：状态帧第 3 个字节（音色 0..2）不能被误当成新帧的开头。
    #   所有帧头都 >= 0x5A，所以这是设计性质，不是巧合。
    check("音色值 0..2 和数据帧头不冲突（帧头都 >= 0x5A）",
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
    check("录音同步链采的是 sounding_mask_aud（门控）",
          re.search(r"rec_mask_s1\s*<=\s*sounding_mask_aud\s*;", top_src) is not None)
    check("录音源没有退回到 sounding_mask（包络输出、带尾巴）",
          re.search(r"rec_mask_s1\s*<=\s*sounding_mask\s*;", top_src) is None)
    check("录音源没有退回到 key_mask",
          re.search(r"rec_mask_s1\s*<=\s*key_mask\s*;", top_src) is None)
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
        sr = tp["CLK_FREQ_HZ"] // tp["FRAME_DIV"]          # top.v 是整数除法
        check("采样率 %d Hz 和上位机一致" % sr, sr == R.SR_HZ, (sr, R.SR_HZ))
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
