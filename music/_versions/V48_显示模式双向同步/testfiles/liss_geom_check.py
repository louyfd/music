# -*- coding: utf-8 -*-
"""liss_geom_check.py —— 李萨如显示 (X,Y) -> (col,row) 映射的几何自检

为什么需要它
------------
李萨如这一屏的"对不对"平时只能上板子看：图形画歪了、只画了半屏、
满量程跑到窗口外面，从 RTL 上完全看不出来（和 check_rtl_contract.py
里说的"三方一致地错"是同一类问题）。

所以这里把 lcd_master.v 里的映射【按 RTL 的算法原样】在 Python 里走一遍
（移位相加、算术右移、夹取，一步都不省），然后验三个理想输入：

    1) 同相同幅   X = Y          -> 一条过中心的直线（对角线）
    2) 相位差 90° X⊥Y（等幅）    -> 正椭圆；(Y 幅度 = 1.4×X) -> 正圆
    3) 频率比 2:1 X=f, Y=2f      -> 8 字（∞），两瓣 + 中心自交一次

再把映射的边界（满量程 ±32767/±32768、0、以及越界夹取）列出来。

★ 常数不是手抄的：全部从 src/lcd_master.v 里正则抠出来再断言，
  RTL 改了（比如把 ×140 改成 ×150）这里立刻红。

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


def parse_rtl(src: str):
    """返回 (xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax)，全部来自源码。"""
    # x140 = (xs24 <<< 7) + (xs24 <<< 3) + (xs24 <<< 2)      -> 128+8+4
    m = re.search(r"x140\s*=\s*\(xs24\s*<<<\s*(\d+)\)\s*\+\s*\(xs24\s*<<<\s*(\d+)\)"
                  r"\s*\+\s*\(xs24\s*<<<\s*(\d+)\)", src)
    xcoef = sum(1 << int(g) for g in m.groups()) if m else None
    m = re.search(r"y100\s*=\s*\(ys24\s*<<<\s*(\d+)\)\s*\+\s*\(ys24\s*<<<\s*(\d+)\)"
                  r"\s*\+\s*\(ys24\s*<<<\s*(\d+)\)", src)
    ycoef = sum(1 << int(g) for g in m.groups()) if m else None
    m = re.search(r"liss_col_s\s*=\s*10'sd(\d+)\s*\+", src)
    col_mid = int(m.group(1)) if m else None
    m = re.search(r"liss_row_s\s*=\s*10'sd(\d+)\s*-", src)
    row_mid = int(m.group(1)) if m else None
    m = re.search(r">\s*10'sd(\d+)\)\s*\?\s*9'd(\d+)", src)
    xmax = int(m.group(2)) if m else None
    m = re.search(r">\s*10'sd(\d+)\)\s*\?\s*8'd(\d+)", src)
    ymax = int(m.group(2)) if m else None
    shift = _sar_shift(src, "liss_dx", "x140")
    return xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax


def _sar_shift(src: str, name: str, srcvec: str):
    """把"算术右移"那行按 Verilog 的位宽规则算成【实际移了几位】。

    ★ 这一条是踩出来的：RTL 原来写成
          wire signed [8:0] liss_dx = {x140[23], x140[23:15]};
      {..} 是 10 位（符号位重复了一次），赋给 9 位 wire 时【丢的是低位】，
      结果实际变成右移 16 位 —— 图形幅度直接小一半，而代码看着还挺像对的。
      综合日志里只有一句 "Expression size 10 truncated to fit in target size 9"。
      所以这里老老实实把拼接展开成"用到了源向量的哪几位"，再截到目标位宽。
    """
    m = re.search(r"wire\s+signed\s*\[(\d+):(\d+)\]\s*%s\s*=\s*([^;]+);" % name, src)
    if not m:
        return None
    tgt_msb, tgt_lsb = int(m.group(1)), int(m.group(2))
    width = tgt_msb - tgt_lsb + 1
    expr = m.group(3)
    bits: list[int] = []
    for a, b in re.findall(r"%s\[(\d+)(?::(\d+))?\]" % srcvec, expr):
        if b == "":
            bits.append(int(a))
        else:
            bits += list(range(int(a), int(b) - 1, -1))
    if not bits:
        return None
    eff = bits[-width:]                     # Verilog：截断保留低位
    m2 = re.search(r"wire\s+signed\s*\[(\d+):\d+\]\s*%s\s*=" % srcvec, src)
    want = list(range(int(m2.group(1)), int(m2.group(1)) - width, -1))
    if eff != want:
        # 拼出来的低位不等于"源向量的连续高 width 位" -> 语义不是干净的算术右移
        return -1
    return eff[-1]                          # 移位量 = 切片的最低那位


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
    # ★ 边界不是对称的 ±140：x=+32767 时 32767×140/32768 = 139.999…（向下取整）-> 139，
    #   只有 -32768 才正好到 -140。所以 col 实际落 20..299、row 落 21..220。
    check("col 落 20..299（满量程 ±140 像素，正端被 floor 吃掉 1）",
          min(cols) == 20 and max(cols) == 299, (min(cols), max(cols)))
    check("row 落 21..220（满量程 ±100 像素，正端同样 floor）",
          min(rows) == 21 and max(rows) == 220, (min(rows), max(rows)))
    check("图形穿过中心 (160,120)", (mp.col_mid, mp.row_mid) in pts)
    # 共线性：RTL 里 col/row 各自向下取整，所以允许 ±(140+100)/2 的截距残差
    resid = max(abs(100 * (c - mp.col_mid) - 140 * (mp.row_mid - r)) for c, r in pts)
    check("全部点共线（100·Δcol 与 140·Δrow 一致，残差 %d ≤ 240 = 两个取整误差）"
          % resid, resid <= 240)
    check("两轴同增同减（斜率符号唯一，不是两条线）",
          len({(c >= mp.col_mid) == (r <= mp.row_mid) for c, r in pts}) == 1)
    print("       斜率 = Δrow/Δcol = -100/140 = -0.714（约 -35.5°，因为 X/Y 像素刻度"
          " 140:100 不同，所以不是 45°）")


def test_circle(mp: Map) -> None:
    print("[2] 相位差 90°  X = cos, Y = sin")
    pts, _ = sweep(mp, math.cos, math.sin)
    # (a) 等幅：正椭圆，半轴 = 140 / 100
    worst = 0.0
    for c, r in pts:
        u = (c - mp.col_mid) / 140.0
        v = (mp.row_mid - r) / 100.0
        worst = max(worst, abs(math.hypot(u, v) - 1.0))
    check("等幅时是【正椭圆】，半轴 140/100（(Δcol/140)²+(Δrow/100)²=1，误差 %.4f）"
          % worst, worst < 0.05)
    check("闭合（首尾同相位的两个点画出同一个像素）", pts[0] == pts[-1])
    # (b) Y 幅度 = 1.4 × X 幅度 -> 正圆
    ax = 23000
    ay = int(round(ax * 140.0 / 100.0))          # 32200，未超满量程
    rs = []
    for i in range(721):
        th = 2.0 * math.pi * i / 720
        rs.append(math.hypot(*[a - b for a, b in
                               zip(mp.xy(int(round(ax * math.cos(th))),
                                         int(round(ay * math.sin(th)))),
                                   (mp.col_mid, mp.row_mid))]))
    check("Y 幅度取 1.4×X（= 32200 ≤ 32767）时是【正圆】，半径 %.1f ± %.1f px"
          % (sum(rs) / len(rs), max(rs) - min(rs)), max(rs) - min(rs) <= 4.0)
    print("       ★ 结论写清楚：映射的像素刻度 X:Y = 140:100，所以【等幅输入画出来"
          "是椭圆】，想要正圆得让 Y 的幅度是 X 的 1.4 倍。")


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
        ci = mp.col_mid + x * 140.0
        ri = mp.row_mid - y * 100.0
        worst_px = max(worst_px, abs(pts[i][0] - ci), abs(pts[i][1] - ri))
    check("屏上每个点都在理想曲线 1 像素内（最大 %.2f px）" % worst_px, worst_px < 1.0)
    up = [p for p in pts if p[1] < mp.row_mid]     # row 小 = y 正
    dn = [p for p in pts if p[1] > mp.row_mid]
    check("两瓣：上半 %d 点 / 下半 %d 点，都不为空" % (len(up), len(dn)),
          len(up) > 0 and len(dn) > 0)
    span_up = max(c for c, _ in up) - min(c for c, _ in up)
    span_dn = max(c for c, _ in dn) - min(c for c, _ in dn)
    check("两瓣各自横跨整个横轴（%d / %d 像素，满幅 279）" % (span_up, span_dn),
          span_up >= 277 and span_dn >= 277)
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
    check("X = +32767 -> col = 299（floor，不越界）", mp.xy(FULL, 0)[0] == 299)
    check("X = -32768 -> col = 20（不越界）", mp.xy(FULL_NEG, 0)[0] == 20)
    check("Y = +32767 -> row = 21（floor，不越界）", mp.xy(0, FULL)[1] == 21)
    check("Y = -32768 -> row = 220（不越界）", mp.xy(0, FULL_NEG)[1] == 220)
    # 夹取：直接塞超出 16 位的数进去，看夹取有没有生效
    check("越界输入被夹到 0/319（col）", mp.xy(40000, 0)[0] == 319 and mp.xy(-40000, 0)[0] == 0)
    check("越界输入被夹到 0/239（row）", mp.xy(0, 40000)[1] == 0 and mp.xy(0, -40000)[1] == 239)
    print("       ★ 数学上 |dx|<=140、|dy|<=100 是硬上界，16 位输入永远撞不到夹取；"
          "夹取是留给'以后改采样位宽'的保险。")
    print("       ★ 图形实际占 col 20..299 / row 21..220，边框画在 0/319、0/239，")
    print("         中心十字画在 160 列 / 120 行 —— 都在图形外面或正中，不会糊在一起。")


def main() -> int:
    if not os.path.isfile(LCD_V):
        print("找不到 %s" % LCD_V)
        return 1
    src = read_rtl(LCD_V)
    xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax = parse_rtl(src)
    print("=" * 66)
    print("李萨如几何自检   (X,Y) -> (col,row)")
    print("RTL: %s" % LCD_V)
    print("-" * 66)
    check("从 lcd_master.v 抠到映射常数", None not in
          (xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax),
          (xcoef, ycoef, shift, col_mid, row_mid, xmax, ymax))
    print("      col = %d + (X * %d) >> %d        (夹到 0..%d)"
          % (col_mid, xcoef, shift, xmax))
    print("      row = %d - (Y * %d) >> %d        (夹到 0..%d)"
          % (row_mid, ycoef, shift, ymax))
    check("X 系数 = 140（= 128+8+4，全移位相加，不占 DSP）", xcoef == 140, xcoef)
    check("Y 系数 = 100（= 64+32+4，全移位相加，不占 DSP）", ycoef == 100, ycoef)
    check("右移 15 位（满量程 ±32767 -> ±140 / ±100 像素）", shift == 15,
          "实际移位量 %r（-1 = 位宽截断把语义改掉了，见 _sar_shift 的说明）" % shift)
    check("中心 (160,120)、边界 319/239 与 320×240 屏一致",
          (col_mid, row_mid, xmax, ymax) == (160, 120, 319, 239),
          (col_mid, row_mid, xmax, ymax))
    # 溢出检查：24 位有符号上下文够不够
    check("24 位有符号不溢出（32768 × 140 = %d < 2^23 = %d）"
          % (32768 * xcoef, 1 << 23), 32768 * xcoef < (1 << 23))

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
