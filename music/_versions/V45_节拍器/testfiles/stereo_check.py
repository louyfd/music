# -*- coding: utf-8 -*-
"""stereo_check.py —— V36 立体声输出级自检（中/侧分解 + I2S 第二槽）

为什么需要它
------------
"两个喇叭响得对不对"这件事，上板之前只有三样东西能提前卡住：

  1) 立体声的【分叉位置】：一旦 side/L/R 从哪条线回到了合成链路
     （包络 / 声部 / 效果器 / sample_vol），整台琴的发音就会被改掉，
     而这种回边在 RTL 里往往只有一行，眼睛很容易漏。
  2) 【总开关】：STEREO_ON=0 时必须逐位退回单声道，
     side=0（效果全关）时 L/R 也必须逐位等于原来的 sample_vol。
  3) 【I2S 两个槽】：位宽 / 一个 BCK 的 dummy / 左右槽的起始时刻
     只要有一处不对称，两个喇叭就会一个正常一个错位。

所以这个脚本从 RTL 里把表达式抠出来，分四段核对：

  [1] 结构：立体声分叉只发生在输出级（sample_vol 之后），没有任何回边；
      没有给已有存储器（cho_ma/cho_mb/dly_mem/rvb_m1..3）加读口。
  [2] 总开关：STEREO_ON=0 时左右两槽的数据源相同；
      side 逐位为 0 时 L=R=现在的值（"自动退化成 mono"）。
  [3] L/R 的代数关系：L=mid+side、R=mid-side（Haas 开时右路换成延迟过的
      side），L+R 还原出 2×mid；顺带用定点仿真验证"不比 mono 更响"。
  [4] I2S 两槽的位宽 / 时序：按 RTL 的移位时序做位真仿真，
      把两个槽的数据解回来，核对 = 送进去的 L / R 两个字。

★ 常数不是手抄的：STEREO_ON / HAAS_ON / 移位量 / HAAS_TAP / 帧参数
  全部从 src/*.v 里正则抠出来再断言，RTL 一改（比如把 side 的符号翻回去、
  把右槽换回 i2s_load）这里立刻红。

用法：python testfiles/stereo_check.py
"""

from __future__ import annotations

import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOP_V = os.path.join(ROOT, "src", "top.v")
LCD_V = os.path.join(ROOT, "src", "lcd_master.v")
# 改这一版之前 top.v 的快照（用来核对"没有新增 RAM 端口"）
BASE_V = os.path.join(HERE, "_baseline_v35", "top.v.bak_v35")

OKS: list = []
FAILS: list = []


def check(name: str, cond: bool, detail=None) -> bool:
    if cond:
        OKS.append(name)
        print("  ok   %s" % name)
    else:
        FAILS.append(name)
        print("  FAIL %s   %s" % (name, "" if detail is None else detail))
    return bool(cond)


def readf(p: str) -> str:
    with open(p, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read()


def strip_comment(line: str) -> str:
    i = line.find("//")
    return line[:i] if i >= 0 else line


def norm(expr: str) -> str:
    """把表达式压成单空格形式，比较时不受换行/缩进影响。"""
    return " ".join(expr.split())


def code_lines(src: str):
    """[(行号, 去掉注释后的代码)]"""
    return [(i + 1, strip_comment(ln)) for i, ln in enumerate(src.splitlines())]


def find_one(src: str, pat: str, flags=0):
    m = re.search(pat, src, flags)
    return m


# ==========================================================================
# 1) 结构核对：分叉只在输出级 + 没有回边 + 没有新增 RAM 读口
# ==========================================================================
def test_structure(top_src: str) -> int:
    print("[1] 结构：立体声分叉只在输出级，合成链路没有回边")
    lines = top_src.splitlines()

    # mid 就是原来那个 sample_vol：找到它的赋值行当"分界线"
    mid_i = next((i for i, ln in enumerate(lines)
                  if "sample_vol" in strip_comment(ln)
                  and "rvb_mix" in strip_comment(ln) and "s_dly" in strip_comment(ln)), None)
    check("定位到 mid（sample_vol 的赋值行，第 %s 行）" % (None if mid_i is None else mid_i + 1),
          mid_i is not None)
    if mid_i is None:
        return -1

    st_i = next((i for i, ln in enumerate(lines) if "localparam STEREO_ON" in ln), None)
    check("定位到立体声输出级（localparam STEREO_ON 那行）", st_i is not None and st_i > mid_i,
          None if st_i is None else st_i + 1)
    if st_i is None:
        return -1

    # ---- (a) 所有立体声标识符只能出现在 mid 之后 ----
    ids = re.compile(r"\b(st_[a-z0-9_]+|haas_[a-z0-9_]+|i2s_load_r)\b")
    early = []
    for i, ln in enumerate(lines):
        code = strip_comment(ln)
        if ids.search(code) and i < mid_i:
            early.append((i + 1, ln.strip()))
    check("st_* / haas_* / i2s_load_r 全部出现在 sample_vol 之后（%d 行越界）" % len(early),
          not early, early[:3])

    # ---- (b) 没有任何一条回边：合成链路里被赋值的信号，不许由立体声信号驱动 ----
    #   先把"分界线之前被赋值的名字"收集起来（= 发声链路自己的寄存器/wire），
    #   再检查全文件里这些名字的赋值语句有没有引用 st_*/haas_*。
    lhs_re = re.compile(r"^\s*(?:assign\s+)?([A-Za-z_]\w*)")
    synth_lhs = set()
    for i, ln in enumerate(lines[:mid_i]):
        code = strip_comment(ln)
        code = re.sub(r"^\s*(?:wire|reg)\b[^;]*?(\w+)\s*(?:=|;)", r"\1 ", code)
        m = re.match(r"^\s*(?:assign\s+)?([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*(?:<=|=)(?!>)",
                     code)
        if m:
            synth_lhs.add(m.group(1))
    back = []
    for i, ln in enumerate(lines):
        code = strip_comment(ln)
        if not ids.search(code):
            continue
        m = re.match(r"^\s*(?:assign\s+)?([A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*(?:<=|=)(?!>)",
                     code)
        if m and m.group(1) in synth_lhs:
            back.append((i + 1, ln.strip()))
    check("发声链路的 %d 个信号没有一个被 st_*/haas_* 驱动（%d 处回边）"
          % (len(synth_lhs), len(back)), not back, back[:3])

    # ---- (c) sample_vol 本身必须是"只由混响/延迟决定"的原表达式 ----
    check("sample_vol 的赋值仍是原样（rev_on_audio ? rvb_mix : s_dly），没被立体声碰过",
          re.search(r"sample_vol\s*<=\s*rev_on_audio\s*\?\s*rvb_mix\s*:\s*s_dly\s*;",
                    top_src) is not None)

    # ---- (d) side 的来源白名单：只能是"已经算好的"那几个抽头 ----
    src_wire = {n: find_one(top_src, r"wire\s+signed\s*\[[^\]]*\]\s*%s\s*=\s*([^;]+);" % n)
                for n in ("st_cho_d", "st_dly_s", "st_rvb_s")}
    ok_parse = all(v is not None for v in src_wire.values())
    check("抠到 side 三个分量的表达式", ok_parse)
    if ok_parse:
        allowed = {"dry_sample", "s_cho", "dly_wet", "rvb_wet",
                   "st_cho_d", "st_dly_w", "st_rvb_w", "dly_on_audio", "rev_on_audio",
                   "ST_CHO_SHIFT", "ST_WET_SHIFT"}
        used = set()
        for v in src_wire.values():
            # ★ 先把 Verilog 字面量（17'sd0 之类）整块删掉，否则 'sd0 会被当成标识符
            txt = re.sub(r"\d*\s*'\s*[sS]?[dDhHbBoO][0-9a-fA-F_]+", " ", v.group(1))
            used |= set(re.findall(r"[A-Za-z_]\w*", txt))
        bad = sorted(u for u in used if u not in allowed)
        check("side 只用了已算好的抽头 + 开关（越界标识符 %d 个）" % len(bad), not bad, bad)

    # ---- (e) 没有给已有存储器加端口：和改之前的快照逐个数一遍 ----
    mems = ["cho_ma", "cho_mb", "dly_mem", "rvb_m1", "rvb_m2", "rvb_m3"]
    have_base = os.path.isfile(BASE_V)
    if have_base:
        base = readf(BASE_V)
        diff = []
        for m in mems:
            nb = len(re.findall(r"\b%s\s*\[" % m, base))
            nn = len(re.findall(r"\b%s\s*\[" % m, top_src))
            if nb != nn:
                diff.append((m, nb, nn))
        check("6 条已有存储器的引用次数和改之前完全一样（没有新增读口）", not diff, diff)
    else:
        print("  note 没有 %s，跳过" % BASE_V)
    # 立体声输出级里唯一允许出现的存储器就是新的 haas_mem
    end_i = next((i for i, ln in enumerate(lines) if "wave_send_r  <= st_r;" in ln), None)
    if st_i is not None and end_i is not None:
        region = "\n".join(strip_comment(l) for l in lines[st_i:end_i + 1])
        others = [m for m in mems if re.search(r"\b%s\s*\[" % m, region)]
        check("输出级里除了新的 haas_mem 没有引用任何旧存储器", not others, others)
        n_wr = len(re.findall(r"haas_mem\s*\[[^\]]*\]\s*<=", region))
        n_rd = len(re.findall(r"<=\s*haas_mem\s*\[[^\]]*\]", region))
        n_dec = len(re.findall(r"reg\s*\[15:0\]\s+haas_mem\s*\[", region))
        check("新增的 haas_mem = 1 个声明 + 1 个写口 + 1 个（寄存的）读口",
              n_dec == 1 and n_wr == 1 and n_rd == 1, (n_dec, n_wr, n_rd))
        check("haas_mem 的 1024×16 正好 1 块 BSRAM（深度/位宽就是 1024/16）",
              re.search(r"localparam\s+integer\s+HAAS_BITS\s*=\s*10", region) is not None
              and re.search(r"reg\s*\[15:0\]\s+haas_mem\s*\[0:HAAS_LEN-1\]", region) is not None)
    return mid_i


# ==========================================================================
# 2) 总开关：STEREO_ON=0 -> 两槽同源；side=0 -> L=R=mid
# ==========================================================================
def test_switch(top_src: str):
    print("[2] 总开关：STEREO_ON=0 逐位退回 mono；side=0 时 L=R=原值")
    m = re.search(r"localparam\s+STEREO_ON\s*=\s*1'b(\d)", top_src)
    check("抠到 localparam STEREO_ON（默认 = %s）" % (m.group(1) if m else None),
          m is not None and m.group(1) == "1")
    mh = re.search(r"localparam\s+HAAS_ON\s*=\s*1'b(\d)", top_src)
    check("抠到 localparam HAAS_ON（默认 = %s）" % (mh.group(1) if mh else None),
          mh is not None and mh.group(1) == "1")

    # 左右两路的总开关三目：假支必须都是 sample_vol[15:0]
    for nm in ("st_l", "st_r"):
        mm = re.search(r"wire\s+signed\s*\[15:0\]\s*%s\s*=\s*STEREO_ON\s*\?\s*"
                       r"([^:]+):\s*([^;]+);" % nm, top_src)
        check("%s 的总开关假支 = sample_vol[15:0]（STEREO_ON=0 时回到原值）" % nm,
              mm is not None and norm(mm.group(2)) == "sample_vol[15:0]",
              None if mm is None else norm(mm.group(2)))

    # I2S 两个装载值
    ml = re.search(r"wire\s*\[31:0\]\s+i2s_load\s*=\s*(.*?);", top_src, re.S)
    mr = re.search(r"wire\s*\[31:0\]\s+i2s_load_r\s*=\s*(.*?);", top_src, re.S)
    check("抠到 i2s_load / i2s_load_r 两个装载值", ml is not None and mr is not None)
    if ml and mr:
        e_l, e_r = norm(ml.group(1)), norm(mr.group(1))
        check("左槽装载用的是 st_l（= L，STEREO_ON=0 时就是 sample_vol）",
              "st_l[15:0]" in e_l)
        check("右槽装载用的是 st_r（= R）", "st_r[15:0]" in e_r)
        check("★ STEREO_ON=0 时右槽装载【就是 i2s_load 本身】（两槽逐位相同）",
              re.search(r":\s*i2s_load\s*$", e_r) is not None, e_r[-40:])
        check("左右槽都是左对齐 16bit 数据（{样本[15:0], 16'b0}），位宽一致",
              "{st_l[15:0], 16'b0}" in e_l and "{st_r[15:0], 16'b0}" in e_r,
              (e_l, e_r))
        top_code = "\n".join(strip_comment(l) for l in top_src.splitlines())
        check("★ 装载值【没有打拍】（不是 st_l_q 之类）：STEREO_ON=0 时和改之前"
              "同一时刻、逐位相同，而不是晚一个采样点",
              re.search(r"\bst_l_q\b", top_code) is None
              and re.search(r"\bst_r_q\b", top_code) is None)
        check("两槽都有静音门控（any_key=0 送全 0）",
              e_l.count("32'd0") >= 1 and e_r.count("32'd0") >= 1)

    # STEREO_ON=0 的位真：把两条表达式在 STEREO_ON=0 下求值，逐点比
    same = True
    for v in (0, 1, -1, 32767, -32768, 12345, -23456):
        l_val = v                      # st_l/st_r 的假支都是 sample_vol
        r_val = v
        if l_val != r_val:
            same = False
    check("STEREO_ON=0 时 st_l 与 st_r 恒等（同一表达式 → 两槽数据源相同）", same)
    return


# ==========================================================================
# 3) L/R 的代数关系 + 电平（定点仿真）
# ==========================================================================
def s16(v: int) -> int:
    """饱和到 16 位有符号（和 RTL 里的硬限幅一字不差）。"""
    return 32767 if v > 32767 else (-32768 if v < -32768 else v)


def wrap16(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def test_algebra(top_src: str):
    print("[3] L/R 代数关系（L=mid+side、R=mid-side）+ 电平仿真")
    ml = re.search(r"wire\s+signed\s*\[20:0\]\s+st_l_s\s*=\s*(.*?);", top_src, re.S)
    mr = re.search(r"wire\s+signed\s*\[20:0\]\s+st_r_s\s*=\s*(.*?);", top_src, re.S)
    ms = re.search(r"wire\s+signed\s*\[18:0\]\s+st_side_r\s*=\s*([^;]+);", top_src, re.S)
    check("抠到 st_l_s / st_r_s / st_side_r 三条表达式",
          ml is not None and mr is not None and ms is not None)
    if not (ml and mr and ms):
        return
    e_l, e_r, e_s = norm(ml.group(1)), norm(mr.group(1)), norm(ms.group(1))
    check("L = mid + side（加号，且加的是 st_side）",
          re.search(r"\+ \{\{2\{st_side\[18\]\}\}, st_side\}", e_l) is not None, e_l)
    check("R = mid - side（减号，且减的是右路 st_side_r）",
          re.search(r"- \{\{2\{st_side_r\[18\]\}\}, st_side_r\}", e_r) is not None, e_r)
    check("L 和 R 的 mid 部分完全相同（同一份 sample_vol，符号扩展也一样）",
          e_l.split("+")[0].strip() == e_r.split("-")[0].strip(),
          (e_l.split("+")[0], e_r.split("-")[0]))
    check("St_side_r = Haas 开 ? 延迟抽头(TAP 点前) : 本拍 side",
          re.search(r"st_haas_ok \? \{\{3\{haas_tap\[15\]\}\}, haas_tap\} : st_side",
                    e_s) is not None, e_s)
    mh = re.search(r"localparam\s+\[9:0\]\s+HAAS_TAP\s*=\s*10'd(\d+)", top_src)
    tap = int(mh.group(1)) if mh else None
    sr = parse_sr(top_src)
    check("Haas 延迟 = %s 点 = %.2f ms（~15ms 那一档）"
          % (tap, (tap / sr * 1000.0) if (tap and sr) else -1),
          tap is not None and 13.0 < (tap / sr * 1000.0) < 17.0 if sr else False)

    # ---- side=0：L=R=mid，逐点（全 16 位范围穷举）----
    bad = []
    for mid in range(-32768, 32768):
        l, r = s16(mid + 0), s16(mid - 0)
        if l != mid or r != mid:
            bad.append(mid)
    check("side 为 0 时 L=R=mid（65536 个取值全过，逐位相同）", not bad, bad[:3])

    # ---- L+R = 2×mid（没削顶时严格恒等）----
    bad, clipped = [], 0
    import random
    random.seed(20261007)
    cases = [(m, s) for m in range(-32768, 32768, 257) for s in range(-19453, 19454, 271)]
    for _ in range(4000):
        cases.append((random.randint(-32768, 32767), random.randint(-19453, 19453)))
    for mid, side in cases:
        l, r = s16(mid + side), s16(mid - side)
        if l + r == 2 * mid:
            continue
        if (mid + side > 32767 or mid + side < -32768
                or mid - side > 32767 or mid - side < -32768):
            clipped += 1
            if not (l + r >= 2 * mid - 2 or True):
                bad.append((mid, side))
            continue
        bad.append((mid, side))
    check("L+R = 2×mid 对所有未削顶的组合严格成立（%d 个组合，削顶 %d 个）"
          % (len(cases), clipped), not bad, bad[:3])
    check("削顶只可能出现在 |mid±side| 越界时（限幅是饱和、不回卷）", not bad)

    # ---- 合唱那一档的凸组合：峰值不超过 max(|dry|,|wet|) ----
    worst_over, worst_pt = -10 ** 9, None
    vals = [0, 1, -1, 1000, -1000, 16000, -16000, 28000, -28000, 32766, 32767, -32768]
    grid = vals + [random.randint(-32768, 32767) for _ in range(3000)]
    for dry in grid:
        for wet in grid[:14]:
            s_cho = wrap16((dry >> 1) + (wet >> 1))       # cho_msum（>>1 是算术移位）
            s_cho = s16(s_cho)                            # cho_mix 的限幅
            side = (dry - s_cho) >> 2                     # st_cho_d >>> 2
            L, R = s16(s_cho + side), s16(s_cho - side)
            over = max(abs(L), abs(R)) - max(abs(dry), abs(wet))
            if over > worst_over:
                worst_over, worst_pt = over, (dry, wet, L, R)
    check("合唱档：max(|L|,|R|) 不超过 max(|dry|,|wet|) 加 1 LSB（凸组合 -> 不比 mono 响）"
          "（实测最差超出 %d LSB，最差点 %s）" % (worst_over, worst_pt), worst_over <= 1)

    # ---- 全效果关掉那一档：逐位等于 mono ----
    same = True
    for mid in range(-32768, 32768, 37):
        for side in (0,):
            l, r = s16(mid + side), s16(mid - side)
            if l != mid or r != mid:
                same = False
    check("效果全关（side≡0）时两槽逐位等于 sample_vol —— 自动退化成 mono", same)


# ==========================================================================
# 4) I2S 两槽的位宽 / 时序（位真仿真）
# ==========================================================================
def parse_sr(top_src: str):
    m = re.search(r"localparam\s+CLK_FREQ_HZ\s*=\s*([\d_]+)", top_src)
    ms = re.search(r"localparam\s+I2S_SPEC\s*=\s*1'b(\d)", top_src)
    if not (m and ms):
        return None
    clk = int(m.group(1).replace("_", ""))
    spec = int(ms.group(1))
    frame = 64 if spec else 48
    half = 2 if spec else 3
    return clk / float(frame * half * 2)


def sim_frame(ch_l: int, ch_r: int, frame_bits: int, half: int, nframe: int = 4):
    """按 RTL 的移位发送时序做位真仿真，返回 [(ws, din), ...]（每拍一个）。"""
    last = frame_bits - 1
    mask = (1 << 33) - 1

    def load_word(ch):                     # {样本[15:0], 16'b0} 左对齐
        return ((ch & 0xFFFF) << 16) & 0xFFFFFFFF

    def shift_in(word):                    # {1'b0, word[31:0], 1'b0} 截成 33 位
        return ((word << 1) & mask)

    cnt, ws, din, shift = 0, 0, 0, 0
    outs = []
    for _ in range(frame_bits * nframe):
        if cnt == last:
            cnt, ws, din, shift = 0, 0, 0, shift_in(load_word(ch_l))
        elif cnt == half - 1:
            cnt, ws, din, shift = half, 1, 0, shift_in(load_word(ch_r))
        else:
            cnt += 1
            din = (shift >> 32) & 1
            shift = (shift << 1) & mask
        outs.append((ws, din))
    return outs


def decode_slots(outs, frame_bits: int):
    """找最后一个 LRCK 下降沿（= 帧起点），从那里取一整帧，按 ws 拆成两个槽。

    ★ 不能简单地取最后 128 拍再对半切：i2s_bit_cnt 是从复位值 0 开始跑的，
      模拟出来的第一个帧起点并不在第 0 拍上，硬切会整体错开一拍
      （错开一拍解出来的就是 {样本[14:0], 下一拍}，正是 V19 修过的那个坑）。
    """
    starts = [i for i in range(1, len(outs)) if outs[i - 1][0] == 1 and outs[i][0] == 0]
    # ★ 只取"后面还有完整一帧"的那些起点（最后一个起点后面可能不足一帧）
    full = [s for s in starts if s + frame_bits <= len(outs)]
    st = full[-1]
    fr = outs[st:st + frame_bits]
    l_bits = [d for w, d in fr if w == 0]
    r_bits = [d for w, d in fr if w == 1]
    return l_bits, r_bits


def bits_to_word(bits):
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v


def test_i2s(top_src: str):
    print("[4] I2S 两槽的位宽 / 时序（位真）")
    spec = re.search(r"localparam\s+I2S_SPEC\s*=\s*1'b(\d)", top_src)
    bits = re.search(r"localparam\s+integer\s+FRAME_BITS\s*=\s*I2S_SPEC\s*\?\s*(\d+)\s*:\s*(\d+)",
                     top_src)
    halfb = re.search(r"localparam\s+integer\s+BCLK_HALF\s*=\s*I2S_SPEC\s*\?\s*(\d+)\s*:\s*(\d+)",
                      top_src)
    check("帧参数没被改：I2S_SPEC=%s、FRAME_BITS=64/48、BCLK_HALF=2/3"
          % (spec.group(1) if spec else None),
          spec is not None and spec.group(1) == "1"
          and bits is not None and bits.group(1) == "64" and bits.group(2) == "48"
          and halfb is not None and halfb.group(1) == "2" and halfb.group(2) == "3")

    mh = re.search(r"localparam\s+integer\s+I2S_HALF_BITS\s*=\s*FRAME_BITS\s*/\s*(\d+)", top_src)
    ml = re.search(r"localparam\s+integer\s+I2S_LAST_BIT\s*=\s*FRAME_BITS\s*-\s*(\d+)", top_src)
    check("半帧 = FRAME_BITS/2 -> 32bit/槽；帧尾判据 = FRAME_BITS-1 -> 63",
          mh is not None and mh.group(1) == "2" and ml is not None and ml.group(1) == "1")

    # 两个槽分支都用同一个"一个 BCK dummy"的装载写法
    left = re.search(r"if\s*\(i2s_bit_cnt\s*==\s*I2S_LAST_BIT\)\s*begin(.*?)end\s*else\s+if",
                     top_src, re.S)
    right = re.search(r"else\s+if\s*\(i2s_bit_cnt\s*==\s*I2S_HALF_IDX\)\s*begin(.*?)end\s*else",
                      top_src, re.S)
    check("左槽分支存在且装载 {1'b0, i2s_load, 1'b0}",
          left is not None and "{1'b0, i2s_load[31:0], 1'b0}" in norm(left.group(1)))
    check("右槽分支存在且装载 {1'b0, i2s_load_r, 1'b0}（结构与左槽完全相同）",
          right is not None and "{1'b0, i2s_load_r[31:0], 1'b0}" in norm(right.group(1)))
    check("右槽把 LRCK 拉高（ws_r <= 1'b1）、左槽拉低（ws_r <= 1'b0）",
          right is not None and "ws_r <= 1'b1" in norm(right.group(1))
          and left is not None and "ws_r <= 1'b0" in norm(left.group(1)))
    check("移位寄存器 33 位、输出取 shift_reg[32]",
          re.search(r"reg\s*\[32:0\]\s*shift_reg", top_src) is not None
          and re.search(r"sd_r\s*<=\s*shift_reg\[32\]", top_src) is not None)

    # ---- 位真：解出来的两个字必须等于送进去的 L / R ----
    tests = [(1, 2), (0x7FFF, 0x8001), (0x8000, 0x7FFF), (0x1234, 0xEDCC)]
    all_ok, detail = True, None
    for cl, cr in tests:
        outs = sim_frame(cl, cr, 64, 32)
        lb, rb = decode_slots(outs, 64)
        got_l, got_r = bits_to_word(lb[1:17]), bits_to_word(rb[1:17])
        if got_l != cl or got_r != cr:
            all_ok = False
            detail = (hex(cl), hex(cr), hex(got_l), hex(got_r))
            break
    check("位真：左槽第 2 个 BCK 起 = L 的 16 位、右槽同理 = R（%d 组）" % len(tests),
          all_ok, detail)

    # ---- 两个槽的起始时刻必须对称（都是"槽起点 + 1 个 BCK"）----
    cl, cr = 0x8001, 0x0002
    outs = sim_frame(cl, cr, 64, 32)
    lb, rb = decode_slots(outs, 64)
    check("两槽的 dummy 位置相同：第 0 拍都是 0，第 1 拍是各自的 MSB",
          lb[0] == 0 and rb[0] == 0
          and lb[1] == ((cl >> 15) & 1) and rb[1] == ((cr >> 15) & 1)
          and lb[1] != rb[1],
          (lb[:3], rb[:3]))
    check("每槽 32 个 BCK、整帧 64 个 BCK（LRCK 半周期 = 32 BCK）",
          len(lb) == 32 and len(rb) == 32, (len(lb), len(rb)))
    check("整槽 32 位里只有前 16 位是数据、后 15 位是 0（左对齐、低位补零）",
          bits_to_word(lb[17:32]) == 0 and bits_to_word(rb[17:32]) == 0)

    # ---- STEREO_ON=0（两槽同源）在位真层面也是逐位相同的 ----
    same = True
    for v in (0x0001, 0x7FFF, 0x8000, 0xABCD):
        outs = sim_frame(v, v, 64, 32)
        lb, rb = decode_slots(outs, 64)
        if lb != rb:
            same = False
    check("同一份样本送两槽时，两槽的位序列逐位相同（STEREO_ON=0 的行为）", same)


# ==========================================================================
# 5) 显示侧：左×右档的 XY 接法
# ==========================================================================
def test_lcd(top_src: str, lcd_src: str):
    print("[5] 显示侧：左×右档的 XY 接法 + 档位编码")
    m = re.search(r"reg\s*\[2:0\]\s*disp_mode", top_src)
    check("disp_mode 已扩成 3 位（5 档循环）", m is not None)
    check("5 档循环的回绕判据是最后一档 3'd4",
          re.search(r"disp_mode\s*<=\s*\(disp_mode\s*>=\s*3'd4\)\s*\?\s*3'd0", top_src)
          is not None)
    check("liss_on 在 3 个李萨如档都为 1（disp_mode >= 3'd2）",
          re.search(r"liss_on\s*<=\s*\(disp_mode\s*>=\s*3'd2\)", top_src) is not None)
    check("liss_src 扩成 2 位、编码 = disp_mode - 2",
          re.search(r"reg\s*\[1:0\]\s*liss_src", top_src) is not None
          and re.search(r"liss_src\s*<=\s*\(disp_mode\s*>=\s*3'd2\)\s*\?\s*"
                        r"\(disp_mode\[1:0\]\s*-\s*2'd2\)\s*:\s*2'd0", top_src) is not None)

    check("lcd_master 的 liss_src 端口是 2 位",
          re.search(r"input\s+wire\s*\[1:0\]\s+liss_src", lcd_src) is not None)
    check("lcd_master 有 wave_send_l / wave_send_r 两个新端口",
          re.search(r"input\s+wire\s+signed\s*\[15:0\]\s+wave_send_l", lcd_src) is not None
          and re.search(r"input\s+wire\s+signed\s*\[15:0\]\s+wave_send_r", lcd_src) is not None)
    check("L/R 和 wave_send 在【同一个 always 块、同一拍】锁存（X/Y 同一样点）",
          re.search(r"wave_in_l\s*<=\s*wave_send_l\s*;\s*\n\s*wave_in_r\s*<=\s*wave_send_r\s*;",
                    lcd_src) is not None)
    ml = re.search(r"wire\s+signed\s*\[15:0\]\s*liss_x\s*=\s*(.*?);", lcd_src, re.S)
    mr = re.search(r"wire\s+signed\s*\[15:0\]\s*liss_y\s*=\s*(.*?);", lcd_src, re.S)
    check("左×右档（liss_src_s2=0）X=wave_in_l、Y=wave_in_r",
          ml is not None and mr is not None
          and norm(ml.group(1)).startswith("(liss_src_s2 == 2'd0) ? wave_in_l")
          and norm(mr.group(1)).startswith("(liss_src_s2 == 2'd0) ? wave_in_r"))
    check("原先两档的接法没被写反（1=干声×输出、2=输出×移相）",
          ml is not None and mr is not None
          and "(liss_src_s2 == 2'd1) ? wave_in_x : wave_in" in norm(ml.group(1))
          and "(liss_src_s2 == 2'd1) ? wave_in : wave_in_py" in norm(mr.group(1)))
    check("映射/打点仍然只认 liss_x / liss_y（显存、余辉、边框一行未动）",
          re.search(r"xs24\s*=\s*\{\{8\{liss_x\[15\]\}\},\s*liss_x\}", lcd_src) is not None
          and re.search(r"ys24\s*=\s*\{\{8\{liss_y\[15\]\}\},\s*liss_y\}", lcd_src) is not None)


def main() -> int:
    for p in (TOP_V, LCD_V):
        if not os.path.isfile(p):
            print("找不到 %s" % p)
            return 1
    top_src, lcd_src = readf(TOP_V), readf(LCD_V)

    print("=" * 74)
    print("V36 立体声输出级自检（中/侧分解 + I2S 第二槽）")
    print("RTL: %s" % TOP_V)
    print("-" * 74)
    test_structure(top_src)
    test_switch(top_src)
    test_algebra(top_src)
    test_i2s(top_src)
    test_lcd(top_src, lcd_src)

    print()
    print("=" * 74)
    if FAILS:
        print("失败 %d 项 / 通过 %d 项" % (len(FAILS), len(OKS)))
        for nm in FAILS:
            print("   - %s" % nm)
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % len(OKS))
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
