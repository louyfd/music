# -*- coding: utf-8 -*-
"""liss_geom_check.py —— 李萨如显示 (X,Y) -> (col,row) 映射的几何自检

为什么需要它
------------
李萨如这一屏的"对不对"平时只能上板子看：图形画歪了、只画了半屏、
满量程跑到窗口外面，从 RTL 上完全看不出来（和 check_rtl_contract.py
里说的"三方一致地错"是同一类问题）。

所以这里把 lcd_master.v 里的映射【按 RTL 的算法原样】在 Python 里走一遍
（移位相加、算术右移、夹取，一步都不省），然后验三个理想输入：

    1) 同相同幅   X = Y          -> 一条过中心的直线（正好 45° 的对角线）
    2) 相位差 90° X⊥Y（等幅）    -> 【正圆】（V35.1 起 X/Y 像素刻度相同）
    3) 频率比 2:1 X=f, Y=2f      -> 8 字（∞），两瓣 + 中心自交一次

再把映射的边界（满量程 ±32767/±32768、0、以及越界夹取）列出来。

★ 常数不是手抄的：全部从 src/lcd_master.v 里正则抠出来再断言。
★★ 但这里踩过一个坑，写下来给后面的人（也是本脚本重构的原因）：
   V35.1 把 X 系数从 ×140 改成 ×120、Y 从 ×100 改成 ×120，wire 也改名成
   x120/y120，而这个脚本的正则还写死着 x140/y100 —— 于是它【既没红也没报错，
   而是崩在后面的 %d 格式化上】。批量跑回归时，一条"崩掉的检查"最容易被当成
   环境噪音放过，等于这个几何检查白挂了好几版。
   现在改成：按形状抓 `x<数字>`/`y<数字>`（不写死数字）+ wire 名字与移位和互证
   + 抠不到就 check 判失败并提前收工，绝不带着 None 往下算。

用法：python testfiles/liss_geom_check.py
"""

from __future__ import annotations

import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LCD_V = os.path.join(ROOT, "src", "lcd_master.v")

OKS: list[str] = []
FAILS: list[str] = []


def check(name: str, cond: bool, detail=None) -> bool:
    if cond:
        OKS.append(name)
        print("  ok   %s" % name)
    else:
        FAILS.append(name)
        print("  FAIL %s   %s" % (name, "" if detail is None else detail))
    return bool(cond)


# --------------------------------------------------------------------------
# 0) 从 RTL 里把映射常数抠出来
# --------------------------------------------------------------------------
def read_rtl(path: str) -> str:
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read()


def _coef_from(src: str, axis: str):
    """从 `wire signed [23:0] x120 = (xs24 <<< 6) + (xs24 <<< 5) + ...;` 抠系数。

    ★ 双取互证：wire【名字里的数字】是给人看的，【移位和】才是综合器真正用的，
      两者不一致就直接判失败 —— 不许只信一个。
    ★★ 为什么改成按 `x<数字>` 泛化而不是写死 x140：
       V35.1 为了让正圆是圆，把 X 从 ×140 改成 ×120、Y 从 ×100 改成 ×120，
       wire 也跟着改名成 x120/y120 —— 而这个脚本的正则还写着 x140/y100，
       **于是它既没红、也没报错，是直接崩在后面的 %d 格式化上**（None）。
       一个"崩掉的检查"比"红掉的检查"危险得多：批量跑回归时很容易被当成噪音放过。
       所以现在：① 名字按形状抓，不写死数字；② 名字和表达式必须互证；
       ③ 抓不到就 check 判失败（红），再由 main 提前收工，绝不带着 None 往下算。
    """
    m = re.search(r"wire\s+signed\s*\[23:0\]\s+%s(\d+)\s*=\s*([^;]+);" % axis, src)
    if not m:
        return None, None, None
    name_c = int(m.group(1))
    shifts = [int(g) for g in re.findall(r"<<<\s*(\d+)", m.group(2))]
    expr_c = sum(1 << s for s in shifts)
    return name_c, expr_c, shifts


def _slice_shift(src: str, name: str, vecbase: str):
    """从 `wire signed [8:0] liss_dx = x120[23:15];` 反推【实际移了几位】。

    返回 (shift, 目标位宽)。shift = -1 表示"拼出来的不是干净的算术右移"。

    ★ 这一条是踩出来的（教训原样保留）：RTL 曾经写成
          wire signed [8:0] liss_dx = {x140[23], x140[23:15]};
      {..} 是 10 位（符号位重复了一次），赋给 9 位 wire 时【丢的是低位】，
      结果实际变成右移 16 位 —— 图形幅度直接小一半，而代码看着还挺像对的。
      综合日志里只有一句 "Expression size 10 truncated to fit in target size 9"。
      所以两种写法都认，并且拼接写法按 Verilog 的位宽规则算出真实移位量。
    """
    m = re.search(r"wire\s+signed\s*\[(\d+):(\d+)\]\s*%s\s*=\s*([^;]+);" % name, src)
    if not m:
        return None, None
    tgt_w = int(m.group(1)) - int(m.group(2)) + 1
    expr = m.group(3).strip()
    # 形式 A：纯位选 `x120[23:15]`（V35.1 之后的写法）—— 干净，直接算
    #   ★ 移位量 = 切片的【低位】（b），不是 a-b：源向量是 24 位有符号，
    #     取 bit23..bit15 表示的就是 x >> 15。别写成 23-15=8（那是宽度-1）。
    #   ★ 同时要求切片顶上就是源向量的符号位（a == 23），否则不是算术右移。
    ma = re.fullmatch(r"%s\d*\s*\[\s*(\d+)\s*:\s*(\d+)\s*\]" % vecbase, expr)
    if ma:
        hi, lo = int(ma.group(1)), int(ma.group(2))
        m3 = re.search(r"wire\s+signed\s*\[(\d+):\d+\]\s*%s\d*\s*=" % vecbase, src)
        msb = int(m3.group(1)) if m3 else None
        if msb is not None and hi != msb:
            return -1, tgt_w               # 没带上符号位 -> 语义不对
        return lo, tgt_w
    # 形式 B：拼接 `{x140[23], x140[23:15]}` —— 展开成"用了源向量的哪几位"，再截到目标位宽
    bits: list[int] = []
    for a, b in re.findall(r"%s\d*\[(\d+)(?::(\d+))?\]" % vecbase, expr):
        if b == "":
            bits.append(int(a))
        else:
            bits += list(range(int(a), int(b) - 1, -1))
    if not bits:
        return None, tgt_w
    eff = bits[-tgt_w:]                     # Verilog：截断保留低位
    m2 = re.search(r"wire\s+signed\s*\[(\d+):\d+\]\s*%s\s*=" % vecbase, src)
    if not m2:
        return None, tgt_w
    want = list(range(int(m2.group(1)), int(m2.group(1)) - tgt_w, -1))
    if eff != want:
        # 拼出来的低位不等于"源向量的连续高 width 位" -> 语义不是干净的算术右移
        return -1, tgt_w
    return eff[-1], tgt_w                   # 移位量 = 切片的最低那位


def parse_rtl(src: str) -> dict:
    """把映射常数全部从源码里抠出来（不手抄任何一个数字）。"""
    xn, xcoef, xsh = _coef_from(src, "x")
    yn, ycoef, ysh = _coef_from(src, "y")
    check("X 系数：wire 名字 x%s 与移位和 %s 互证（移位 %s）"
          % (xn, xcoef, xsh), xn is not None and xn == xcoef, (xn, xcoef, xsh))
    check("Y 系数：wire 名字 y%s 与移位和 %s 互证（移位 %s）"
          % (yn, ycoef, ysh), yn is not None and yn == ycoef, (yn, ycoef, ysh))
    m = re.search(r"liss_col_s\s*=\s*10'sd(\d+)\s*\+", src)
    col_mid = int(m.group(1)) if m else None
    m = re.search(r"liss_row_s\s*=\s*10'sd(\d+)\s*-", src)
    row_mid = int(m.group(1)) if m else None
    m = re.search(r">\s*10'sd(\d+)\)\s*\?\s*9'd(\d+)", src)
    xmax = int(m.group(2)) if m else None
    m = re.search(r">\s*10'sd(\d+)\)\s*\?\s*8'd(\d+)", src)
    ymax = int(m.group(2)) if m else None
    shift, dx_w = _slice_shift(src, "liss_dx", "x")
    _, dy_w = _slice_shift(src, "liss_dy", "y")
    return dict(xcoef=xcoef, ycoef=ycoef, shift=shift, col_mid=col_mid,
                row_mid=row_mid, xmax=xmax, ymax=ymax,
                xname=xn, yname=yn, dx_w=dx_w, dy_w=dy_w)


# --------------------------------------------------------------------------
# 1) 映射：和 RTL 一模一样的算法
# --------------------------------------------------------------------------
class Map:
    def __init__(self, xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax):
        self.xcoef, self.ycoef, self.shift = xcoef, ycoef, shift
        self.col_mid, self.row_mid = col_mid, row_mid
        self.xmax, self.ymax = xmax, ymax

    def _sar(self, v: int) -> int:
        """算术右移 = floor(v / 2**shift)。RTL 是取符号位+高 8 位，等价于此。"""
        return v >> self.shift          # Python 的 >> 对负数就是算术右移

    def xy(self, x: int, y: int):
        x140 = x * self.xcoef           # RTL: (xs24<<7)+(xs24<<3)+(xs24<<2)
        y100 = y * self.ycoef
        dx = self._sar(x140)
        dy = self._sar(y100)
        col = self.col_mid + dx
        row = self.row_mid - dy
        # 夹取（RTL 里是两级三目比较）
        col = 0 if col < 0 else (self.xmax if col > self.xmax else col)
        row = 0 if row < 0 else (self.ymax if row > self.ymax else row)
        return col, row


FULL = 32767            # 16 位有符号满量程
FULL_NEG = -32768


def ideal(mp: Map, x: float, y: float):
    """RTL 想做的那条公式，用浮点算（不带取整），用来量量化误差。"""
    return (mp.col_mid + x * mp.xcoef / float(1 << mp.shift),
            mp.row_mid - y * mp.ycoef / float(1 << mp.shift))


def sweep(mp: Map, fx, fy, n=1024):
    """按绝对时刻采样：先把连续信号量化成 16 位整数，再走 RTL 的映射。"""
    pts, sam = [], []
    for i in range(n + 1):                       # n+1 个点：首尾是同相位，可以验闭合
        th = 2.0 * math.pi * i / n
        x = int(round(FULL * fx(th)))
        y = int(round(FULL * fy(th)))
        x = max(FULL_NEG, min(FULL, x))
        y = max(FULL_NEG, min(FULL, y))
        sam.append((x, y))
        pts.append(mp.xy(x, y))
    return pts, sam


def test_quantize(mp: Map) -> None:
    print("[0] 映射实现 = 公式本身（floor + 取整，逐点误差 ≤ 1 像素）")
    worst_c = worst_r = 0.0
    for i in range(2048):
        th = 2.0 * math.pi * i / 2048
        x = int(round(FULL * math.sin(3 * th)))
        y = int(round(FULL * math.cos(2 * th)))
        c, r = mp.xy(x, y)
        ci, ri = ideal(mp, x, y)
        worst_c = max(worst_c, abs(c - ci))
        worst_r = max(worst_r, abs(r - ri))
    check("col 与浮点公式最多差 %.2f 像素（向下取整，<1）" % worst_c, worst_c < 1.0)
    check("row 与浮点公式最多差 %.2f 像素（向下取整，<1）" % worst_r, worst_r < 1.0)


# --------------------------------------------------------------------------
# 三个几何判据
# --------------------------------------------------------------------------
def test_diagonal(mp: Map) -> None:
    print("[1] 同相同幅 X = Y  ->  过中心的直线（对角线）")
    pts, _ = sweep(mp, lambda t: math.sin(t), lambda t: math.sin(t))
    cols = [c for c, _ in pts]
    rows = [r for _, r in pts]
    # ★ 边界不是对称的 ±120：x=+32767 时 32767×120/32768 = 119.999…（向下取整）-> 119，
    #   只有 -32768 才正好到 -120。所以 col 实际落 40..279。
    #   row 的理论下端是 120+120 = 240，出屏了，被夹到 239（见 main 里的说明）。
    check("col 落 %d..%d（满量程 ±%d 像素，正端被 floor 吃掉 1）"
          % (mp.col_mid - mp.xcoef, mp.col_mid + mp.xcoef - 1, mp.xcoef),
          min(cols) == mp.col_mid - mp.xcoef
          and max(cols) == mp.col_mid + mp.xcoef - 1, (min(cols), max(cols)))
    check("row 落 %d..%d（正端同样 floor；负端理论 %d 出屏 -> 夹到 %d）"
          % (mp.row_mid - mp.ycoef + 1, mp.ymax, mp.row_mid + mp.ycoef, mp.ymax),
          min(rows) == mp.row_mid - mp.ycoef + 1 and max(rows) == mp.ymax,
          (min(rows), max(rows)))
    check("图形穿过中心 (160,120)", (mp.col_mid, mp.row_mid) in pts)
    # 共线性：两轴系数相等 + 各自向下取整 ⇒ Δcol + Δrow 恒为 0
    # （只有最底下那一点被夹取，会差 1）
    resid = max(abs((c - mp.col_mid) + (r - mp.row_mid)) for c, r in pts)
    check("全部点共线且正好 45°（Δcol + Δrow == 0，残差 %d ≤ 1 = 底部那次夹取）"
          % resid, resid <= 1)
    check("两轴同增同减（斜率符号唯一，不是两条线）",
          len({(c >= mp.col_mid) == (r <= mp.row_mid) for c, r in pts}) == 1)
    print("       斜率 = Δrow/Δcol = -1，即 -45°（V35.1 起 X/Y 像素刻度相同 %d:%d）"
          % (mp.xcoef, mp.ycoef))


def test_circle(mp: Map) -> None:
    print("[2] 相位差 90°  X = cos, Y = sin")
    pts, _ = sweep(mp, math.cos, math.sin)
    # ★ V35.1 之后 X/Y 刻度相同，所以等幅输入【就是正圆】，不再是椭圆。
    scale = FULL / float(1 << mp.shift)          # 幅度 32767 -> 120 × 0.99997 px
    rx = mp.xcoef * scale
    ry = mp.ycoef * scale
    worst = 0.0
    for c, r in pts:
        u = (c - mp.col_mid) / rx
        v = (mp.row_mid - r) / ry
        worst = max(worst, abs(math.hypot(u, v) - 1.0))
    check("等幅时是【正圆】，半径 %.2f px（刻度 %d:%d 相同，(Δcol/r)²+(Δrow/r)²=1，"
          "误差 %.4f）" % (rx, mp.xcoef, mp.ycoef, worst), worst < 0.05)
    check("闭合（首尾同相位的两个点画出同一个像素）", pts[0] == pts[-1])
    # 反过来再验一次"刻度真的等比例"：Y 幅度减半就该画出 2:1 的椭圆
    ax, ay = FULL, FULL // 2
    us, vs = [], []
    for i in range(721):
        th = 2.0 * math.pi * i / 720
        c, r = mp.xy(int(round(ax * math.cos(th))), int(round(ay * math.sin(th))))
        us.append(abs(c - mp.col_mid))
        vs.append(abs(mp.row_mid - r))
    ratio = max(us) / float(max(vs))
    check("Y 幅度减半 -> 椭圆，长短轴比 %.3f ≈ 2（刻度一致 ⇒ 映射是等比例缩放）" % ratio,
          abs(ratio - 2.0) < 0.03, ratio)
    print("       ★ 结论写清楚：映射的像素刻度 X:Y = %d:%d，所以【等幅输入画出来就是"
          "正圆】；上一版是 140:100，等幅画出来是 0.714 的椭圆（V35.1 已修）。"
          % (mp.xcoef, mp.ycoef))


def test_figure8(mp: Map) -> None:
    print("[3] 频率比 2:1  X = sin(t), Y = sin(2t)  ->  8 字")
    pts, sam = sweep(mp, lambda t: math.sin(t), lambda t: math.sin(2 * t))
    # (a) 代数关系放在【理想曲线】上验（映射后的整数点有 1 像素量化误差，
    #     在 x->±1 处曲线几乎是竖直的，1 像素能放大成很大的 y 误差）
    worst = 0.0
    for i in range(len(pts)):
        x = sam[i][0] / float(FULL)
        y = sam[i][1] / float(FULL)
        worst = max(worst, abs(abs(y) - 2.0 * abs(x) * math.sqrt(max(0.0, 1 - x * x))))
    check("采样序列本身满足 8 字方程 |y| = 2|x|√(1-x²)（误差 %.5f）" % worst, worst < 0.01)
    # (b) 画到屏上仍然贴着这条曲线（理想曲线 vs 整数点，逐点 ≤ 1.5 像素）
    worst_px = 0.0
    for i in range(len(pts)):
        x = sam[i][0] / float(1 << 15) * 1.0     # 归一化到 ±1
        y = sam[i][1] / float(1 << 15) * 1.0
        ci = mp.col_mid + x * mp.xcoef
        ri = mp.row_mid - y * mp.ycoef
        worst_px = max(worst_px, abs(pts[i][0] - ci), abs(pts[i][1] - ri))
    check("屏上每个点都在理想曲线 1 像素内（最大 %.2f px）" % worst_px, worst_px < 1.0)
    up = [p for p in pts if p[1] < mp.row_mid]     # row 小 = y 正
    dn = [p for p in pts if p[1] > mp.row_mid]
    check("两瓣：上半 %d 点 / 下半 %d 点，都不为空" % (len(up), len(dn)),
          len(up) > 0 and len(dn) > 0)
    span_up = max(c for c, _ in up) - min(c for c, _ in up)
    span_dn = max(c for c, _ in dn) - min(c for c, _ in dn)
    check("两瓣各自横跨整个横轴（%d / %d 像素，满幅 %d）"
          % (span_up, span_dn, 2 * mp.xcoef - 1),
          span_up >= 2 * mp.xcoef - 3 and span_dn >= 2 * mp.xcoef - 3)
    check("中心 (160,120) 是自交点（两瓣在此相接，且被经过两次以上）",
          pts.count((mp.col_mid, mp.row_mid)) >= 2)
    para, _ = sweep(mp, math.cos, lambda t: math.cos(2 * t))
    check("和 cos/cos 的抛物线区分开（那是另一个图形，不过中心）",
          para.count((mp.col_mid, mp.row_mid)) == 0)


def test_bounds(mp: Map) -> None:
    print("[4] 映射边界（满量程 / 半个量程 / 越界夹取）")
    print("       X        Y     ->  (col,row)")
    rows = [(0, 0), (FULL, FULL), (FULL_NEG, FULL_NEG), (FULL, FULL_NEG),
            (FULL_NEG, FULL), (16384, -16384), (1, -1), (-1, 1),
            (FULL, 0), (0, FULL), (FULL_NEG, 0), (0, FULL_NEG)]
    for x, y in rows:
        c, r = mp.xy(x, y)
        print("    %7d %7d  ->  (%3d,%3d)" % (x, y, c, r))
    check("X = +32767 -> col = %d（floor，不越界）" % (mp.col_mid + mp.xcoef - 1),
          mp.xy(FULL, 0)[0] == mp.col_mid + mp.xcoef - 1)
    check("X = -32768 -> col = %d（不越界）" % (mp.col_mid - mp.xcoef),
          mp.xy(FULL_NEG, 0)[0] == mp.col_mid - mp.xcoef)
    check("Y = +32767 -> row = %d（floor，不越界）" % (mp.row_mid - mp.ycoef + 1),
          mp.xy(0, FULL)[1] == mp.row_mid - mp.ycoef + 1)
    # ★ 这里现在会碰到夹取：理论值 120+120 = 240 出屏，被夹到 239
    check("Y = -32768 -> row = %d（理论 %d 出屏，被夹到 %d —— 圆底 1 px 平台）"
          % (mp.ymax, mp.row_mid + mp.ycoef, mp.ymax),
          mp.xy(0, FULL_NEG)[1] == mp.ymax)
    # 夹取：直接塞超出 16 位的数进去，看夹取有没有生效
    #   ★ 输入得给够大：dx 要超过 159 才会顶到 319，×120 时 40000 只到 146（不够）
    check("越界输入被夹到 0/319（col）",
          mp.xy(100000, 0)[0] == 319 and mp.xy(-100000, 0)[0] == 0,
          (mp.xy(100000, 0)[0], mp.xy(-100000, 0)[0]))
    #   ★ row 和 col 的符号是反的（row = 中心 - dy），别照抄 col 那两行
    check("越界输入被夹到 0/239（row）",
          mp.xy(0, 100000)[1] == 0 and mp.xy(0, -100000)[1] == 239,
          (mp.xy(0, 100000)[1], mp.xy(0, -100000)[1]))
    print("       ★ 数学上 |dx|<=%d、|dy|<=%d 是硬上界，16 位输入只有 -32768 这一个点"
          "能正好顶到负端；夹取是留给'以后改采样位宽'的保险。" % (mp.xcoef, mp.ycoef))
    print("       ★ 图形实际占 col %d..%d / row %d..%d，边框画在 0/319、0/239，"
          % (mp.col_mid - mp.xcoef, mp.col_mid + mp.xcoef - 1,
             mp.row_mid - mp.ycoef + 1, mp.row_mid + mp.ycoef))
    print("         中心十字画在 160 列 / 120 行 —— 都在图形外面或正中，不会糊在一起。")


def main() -> int:
    if not os.path.isfile(LCD_V):
        print("找不到 %s" % LCD_V)
        return 1
    src = read_rtl(LCD_V)
    p = parse_rtl(src)
    xcoef, ycoef, shift = p["xcoef"], p["ycoef"], p["shift"]
    col_mid, row_mid, xmax, ymax = p["col_mid"], p["row_mid"], p["xmax"], p["ymax"]
    print("=" * 66)
    print("李萨如几何自检   (X,Y) -> (col,row)")
    print("RTL: %s" % LCD_V)
    print("-" * 66)
    # ★ 抠不到就先收工：带着 None 往下算只会崩在某个 %d 上，
    #   而"崩"看起来像环境问题、"红"才看得出是 RTL 和脚本对不上了。
    if None in (xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax):
        print("失败：映射常数抠不全 -> %r" % (p,))
        print("  现在 RTL 里 X/Y 系数那两行长这样：")
        for line in src.splitlines():
            if "wire signed [23:0]" in line or "liss_dx" in line or "liss_dy" in line:
                print("    %s" % line.strip())
        return 1
    print("      col = %d + (X * %d) >> %d        (夹到 0..%d)"
          % (col_mid, xcoef, shift, xmax))
    print("      row = %d - (Y * %d) >> %d        (夹到 0..%d)"
          % (row_mid, ycoef, shift, ymax))
    check("★ X、Y 系数【必须相等】（V35.1：刻度不等的话，等幅输入会画成椭圆）",
          xcoef == ycoef, (xcoef, ycoef))
    check("X 系数 = 120（= 64+32+16+8，全移位相加，不占 DSP）", xcoef == 120, xcoef)
    check("Y 系数 = 120（同上）", ycoef == 120, ycoef)
    check("右移 15 位（满量程 ±32767 -> ±120 像素）", shift == 15,
          "实际移位量 %r（-1 = 位宽截断把语义改掉了，见 _slice_shift 的说明）" % shift)
    check("切片宽度 = 24 - 移位量（liss_dx 应是 9 位：含符号位）",
          p["dx_w"] == 24 - shift and p["dy_w"] == 24 - shift,
          (p["dx_w"], p["dy_w"], shift))
    check("中心 (160,120)、边界 319/239 与 320×240 屏一致",
          (col_mid, row_mid, xmax, ymax) == (160, 120, 319, 239),
          (col_mid, row_mid, xmax, ymax))
    # 溢出检查：24 位有符号上下文够不够
    check("24 位有符号不溢出（32768 × %d = %d < 2^23 = %d）"
          % (xcoef, 32768 * xcoef, 1 << 23), 32768 * xcoef < (1 << 23))
    # ★ 图形占屏（±120）：上下正好顶边，左右留 40 像素
    check("图形占 col %d..%d / row %d..%d（屏 0..%d / 0..%d）"
          % (col_mid - xcoef, col_mid + xcoef - 1,
             row_mid - ycoef + 1, row_mid + ycoef,
             xmax, ymax),
          col_mid - xcoef == 40 and col_mid + xcoef - 1 == 279
          and row_mid - ycoef + 1 == 1)
    print("       ★ 满量程正端被 floor 吃掉 1 像素（32767×120/32768 = 119.99 -> 119），")
    print("         所以 col 实际是 40..279、row 是 1..239；row 的正下方那 1 像素")
    print("         （理论值 240）落在屏外，被夹到 239 —— 圆的底部有个 1 像素平台，")
    print("         肉眼看不出来（要消掉得动 RTL，不在本次范围内）。")

    mp = Map(xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax)
    test_quantize(mp)
    test_diagonal(mp)
    test_circle(mp)
    test_figure8(mp)
    test_bounds(mp)

    print()
    print("=" * 66)
    if FAILS:
        print("失败 %d 项 / 通过 %d 项" % (len(FAILS), len(OKS)))
        for n in FAILS:
            print("   - %s" % n)
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
