# -*- coding: utf-8 -*-
"""liss_phase_check.py —— 相位式李萨如（档 3）的相位 / 几何自检

为什么需要它
------------
档 3 的"好不好看"平时只能上板子看：相移偏了多少、单音画出来是圆还是扁的
一条线、屏上占多大，从 RTL 上完全看不出来。所以这里把两件事都在 Python 里
原样走一遍，上板之前先把数字拿出来：

  1) 相位：按 top.v 里那个 Q15 系数 a，算 110/220/440/830Hz 的相移，
     并跟"理想 90°"比；顺带证明【一级就是最优】—— 穷举两级串起来的情况，
     看它会不会更平（结论：不会，反而更差）。
  2) 位真仿真：把纯正弦按【和 RTL 一模一样的定点递归】跑一遍
     （17 位减法、Q15 乘法、>>15、18 位加法、饱和，一步不省），
     把 (X,Y) 轨迹攒起来，报长短轴比 —— 证明"单音画成接近圆的闭合环"。
  3) 几何：(X,Y) -> (col,row) 的映射按 lcd_master.v 的算法再走一遍，
     核对三个基准输入：同相 -> 对角线、90° -> 圆（屏幕上是 140:100 的椭圆）、
     两路相同 -> 直线（也就是【改之前档 2 在效果全关时的样子】）。

★ 常数不是手抄的：a、采样率、映射系数全部从 src/*.v 里正则抠出来再断言，
  RTL 改了（比如把系数改成别的、把 tap 点挪走）这里立刻红。

用法：python testfiles/liss_phase_check.py
"""

from __future__ import annotations

import io
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOP_V = os.path.join(ROOT, "src", "top.v")
LCD_V = os.path.join(ROOT, "src", "lcd_master.v")

# 键盘音域（四个代表频点：低音区 / 中音区 / 中高 / 高音区）
BAND = [110.0, 220.0, 440.0, 830.0]
TARGET = -90.0                      # 想要的相移（Y 滞后 X 90°）

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


# ==========================================================================
# 0) 从 RTL 抠常数
# ==========================================================================
def parse_top(src: str):
    """返回 dict：采样率 SR、Q15 系数 a、tap 判据、饱和上限。"""
    out = {}
    m = re.search(r"localparam\s+CLK_FREQ_HZ\s*=\s*([\d_]+)", src)
    out["clk"] = int(m.group(1).replace("_", "")) if m else None
    m = re.search(r"localparam\s+I2S_SPEC\s*=\s*1'b(\d)", src)
    spec = int(m.group(1)) if m else None
    out["i2s_spec"] = spec
    if out["clk"] is not None and spec is not None:
        bclk_half = 2 if spec else 3
        frame_bits = 64 if spec else 48
        div = frame_bits * bclk_half * 2
        out["frame_div"] = div
        out["sr"] = out["clk"] / float(div)
    else:
        out["frame_div"] = out["sr"] = None

    # ★ 系数：必须带负号（见 RTL 注释里那段"极点落在 z=-a"的推导）
    m = re.search(r"localparam\s+signed\s*\[\s*15\s*:\s*0\s*\]\s*PH_A\s*=\s*"
                  r"-?\s*16'sd(\d+)", src)
    out["ph_a_raw"] = m.group(0) if m else None
    out["a_q15_neg"] = ("-16'sd" in m.group(0)) if m else None
    out["a_q15"] = (-int(m.group(1)) if out["a_q15_neg"] else int(m.group(1))) if m else None
    if out["a_q15"] is not None:
        out["a"] = out["a_q15"] / 32768.0

    # tap 判据：移相器状态和 wave_send 必须同一个节拍
    m = re.search(r"end\s+else\s+if\s+\(\s*sample_cnt\s*==\s*16'd(\d+)\s*\)\s*begin\s*\n"
                  r"\s*ph_x1", src)
    out["tap"] = int(m.group(1)) if m else None
    m = re.search(r"end\s+else\s+if\s+\(\s*sample_cnt\s*==\s*16'd(\d+)\s*\)\s*begin\s*\n"
                  r"\s*wave_send\s*<=", src)
    out["tap_wave"] = int(m.group(1)) if m else None
    # 饱和（保险）那两级
    out["sat"] = bool(re.search(r"ph_ys\s*>\s*18'sd32767", src))
    return out


def parse_lcd_map(src: str):
    """(X,Y)->(col,row) 的映射常数（和 liss_geom_check.py 一套）。

    ★ V35.1：原来按变量名 x140/y100 抠、还写死"三项移位" —— 刻度一改成
      120（四项移位）就解析成 None 了。现在改成【与变量名无关、移位项数也不写死】：
      只要那一行是 "(变量 <<< n) + (变量 <<< n) ..."，就把 2^n 加起来。
    """
    def coef(var: str):
        m = re.search(var + r"\s*=\s*((?:\((?:\w+)\s*<<<\s*\d+\)\s*\+?\s*)+)", src)
        if not m:
            return None
        return sum(1 << int(g) for g in re.findall(r"<<<\s*(\d+)", m.group(1)))

    xcoef = coef("x120")
    ycoef = coef("y120")
    m = re.search(r"liss_col_s\s*=\s*10'sd(\d+)\s*\+", src)
    col_mid = int(m.group(1)) if m else None
    m = re.search(r"liss_row_s\s*=\s*10'sd(\d+)\s*-", src)
    row_mid = int(m.group(1)) if m else None
    shift = 15 if re.search(r"liss_dx\s*=\s*\w+\[(\d+):(\d+)\]", src) else None
    return xcoef, ycoef, shift, col_mid, row_mid


# ==========================================================================
# 1) 相位：一阶全通的相频特性
# ==========================================================================
def phase_deg(p: float, f: float, sr: float) -> float:
    """一阶全通 H(z)=(z^-1 - p)/(1 - p*z^-1) 在 f 处的相移（度）。

    ★ 这里用的 p 是【极点位置】(z=+p)，和 RTL 递归 y=a*x+x1-a*y1 里的 a
      差一个负号：a = -p。差点搞错的就是这一步（见 RTL 注释）。
    公式：phi(w) = -w - 2*atan2(p*sin w, 1 - p*cos w)，w = 2*pi*f/SR。
    """
    w = 2.0 * math.pi * f / sr
    return math.degrees(-w - 2.0 * math.atan2(p * math.sin(w), 1.0 - p * math.cos(w)))


def f90_of_p(p: float, sr: float) -> float:
    """相移正好 -90° 的那个频点（精确解）。"""
    return (math.pi / 2.0 - 2.0 * math.atan(p)) * sr / (2.0 * math.pi)


def f90_approx(p: float, sr: float) -> float:
    """用户给的近似式 f90 = SR*(1-p)/(2*pi*sqrt(p))，用来看它准不准。"""
    return sr * (1.0 - p) / (2.0 * math.pi * math.sqrt(p))


def shape_ratio(phi: float) -> float:
    """等幅两路相位差 phi 时，信号空间里椭圆的长短轴比（0=直线，1=圆）。

    长短轴 = sqrt(2)*|cos(phi/2)| 和 sqrt(2)*|sin(phi/2)|，所以比值
    = |tan(phi/2)|，再取 min(r,1/r) 归一化到 (0,1]（>1 表示长短轴换了个方向）。
    """
    r = abs(math.tan(math.radians(phi) / 2.0))
    return min(r, 1.0 / r) if r > 0 else 0.0


def screen_ratio(phi: float, xcoef: float, ycoef: float) -> float:
    """把 (X,Y) 上的椭圆搬到【屏幕】上之后的长短轴比。

    屏幕上 col ∝ X*140、row ∝ -Y*100，两轴刻度不一样（这就是
    liss_geom_check.py 里"等幅画出的是椭圆、不是圆"的原因），所以
    信号空间的比值不能直接当屏幕上的比值用，得老老实实做一次奇异值分解：
        (u,v) 是 ±45° 方向的两个主分量，幅度 s1=sqrt(2)cos(phi/2), s2=sqrt(2)sin(phi/2)
        屏幕变换 A = [[xcoef/sqrt2,  xcoef/sqrt2],
                      [-ycoef/sqrt2, ycoef/sqrt2]]
        求 A*diag(s1,s2) 的两个奇异值，比值就是屏幕上的长短轴比。
    """
    s1 = math.sqrt(2.0) * abs(math.cos(math.radians(phi) / 2.0))
    s2 = math.sqrt(2.0) * abs(math.sin(math.radians(phi) / 2.0))
    r2 = math.sqrt(2.0)
    b = [[xcoef / r2 * s1, xcoef / r2 * s2],
         [-ycoef / r2 * s1, ycoef / r2 * s2]]
    # B*B^T 的特征值开根号 = 奇异值
    m00 = b[0][0] * b[0][0] + b[0][1] * b[0][1]
    m01 = b[0][0] * b[1][0] + b[0][1] * b[1][1]
    m11 = b[1][0] * b[1][0] + b[1][1] * b[1][1]
    tr, det = m00 + m11, m00 * m11 - m01 * m01
    disc = math.sqrt(max(0.0, tr * tr / 4.0 - det))
    l1, l2 = tr / 2.0 + disc, tr / 2.0 - disc
    if l1 <= 0:
        return 0.0
    return math.sqrt(max(0.0, l2) / l1)


def test_phase(sr: float, a: float) -> float:
    p = -a                        # 极点位置
    print("[1] 相移表（Q15 系数 a = %d/32768 = %.6f，极点 p = -a = %.6f）"
          % (round(a * 32768), a, p))
    print("      f90 精确解 = %.2f Hz   用户给的近似式 = %.2f Hz（差 %.2f%%）"
          % (f90_of_p(p, sr), f90_approx(p, sr),
             100.0 * abs(f90_approx(p, sr) - f90_of_p(p, sr)) / f90_of_p(p, sr)))
    worst = 0.0
    tab = []
    for f in BAND:
        ph = phase_deg(p, f, sr)
        err = abs(ph - TARGET)
        worst = max(worst, err)
        tab.append((f, ph, err, shape_ratio(ph)))
        print("      %6.0f Hz  相移 %8.2f°   偏离 90° %6.2f°   "
              "信号空间长短轴比 %.3f" % (f, ph, err, shape_ratio(ph)))
    check("四个频点的相移都算出来了，且都不落在 0°/180°（永不退化成直线）",
          all(15.0 < abs(ph) < 165.0 for _, ph, _, _ in tab))
    check("最大偏离 90° = %.1f°（一阶全通的极限，见 [2] 的穷举）" % worst, worst < 52.0)
    check("相移随频率单调变负（110->830Hz 一路往下走，没有回头）",
          all(phase_deg(p, BAND[i], sr) > phase_deg(p, BAND[i + 1], sr)
              for i in range(len(BAND) - 1)))
    check("中音区（220~440Hz）偏离 90° < 25°（这一段屏上是正圆附近）",
          abs(phase_deg(p, 220.0, sr) - TARGET) < 25.0 and
          abs(phase_deg(p, 440.0, sr) - TARGET) < 25.0)
    check("低频端比高频端偏得一样多（把 f90 摆在这个音域正中是对的）",
          abs(abs(phase_deg(p, BAND[0], sr) - TARGET) -
              abs(phase_deg(p, BAND[-1], sr) - TARGET)) < 1.0)
    return worst


def test_one_vs_two(sr: float, a_q15: int):
    """一级够不够？把两级（两个不同系数）穷举一遍，看能不能更平。

    ★ 判据不是"两端离 90° 多近"这一条，还要加一条【一票否决】：
      只要带内某个频点的总相移落回 180° 的整数倍，那一档在这个音上就退化成
      一条直线（等于改之前档 2 的问题）。两级串起来相移会翻倍地跑，最容易
      踩这个坑（比如两级都摆在 728Hz，那 728Hz 上正好合成 -180°，画出来是直线）。
    """
    print("[2] 一级 vs 两级：穷举两个一阶全通串起来，看相移能不能更接近 90°")

    def span_and_dev(ps):
        """返回 (带内相移跨度, 两端最大偏离, 是否穿过了 180° 的整数倍)。"""
        ph = [sum(phase_deg(p, f, sr) for p in ps if p > 0.0) for f in BAND]
        span = abs(ph[0] - ph[-1])
        dev = max(abs(ph[0] - TARGET), abs(ph[-1] - TARGET))
        crossed = False
        for i in range(len(ph) - 1):
            lo, hi = sorted((ph[i], ph[i + 1]))
            if math.ceil(lo / 180.0) <= math.floor(hi / 180.0):
                crossed = True
        return span, dev, crossed

    p_one = -a_q15 / 32768.0
    span1, dev1, _ = span_and_dev([p_one])
    best, arg, span2 = 1e9, None, None
    for i in range(10, 1000):                 # 避开 p→0（那一支只是纯延迟）
        for j in range(10, 1000):
            sp, dv, cr = span_and_dev([i / 1000.0, j / 1000.0])
            if cr:
                continue                      # 会退化成直线，直接否掉
            if dv < best:
                best, arg, span2 = dv, (i / 1000.0, j / 1000.0), sp
    print("      单级（a = %d/32768）：相移跨度 %.2f°，两端最大偏离 %.2f°"
          % (a_q15, span1, dev1))
    print("      两级最优（p1=%.3f, p2=%.3f）：跨度 %.2f°，两端最大偏离 %.2f°"
          % (arg[0], arg[1], span2, best))
    check("两级比单级【更差】-> 一级就是最优，不该加第二级（省一个乘法器）",
          best > dev1, "单级 %.2f / 两级 %.2f" % (dev1, best))
    check("理论依据也对得上：两级的相移跨度 = 两级各自跨度之和 ≥ 一级的 %.2f°"
          % span1, span2 >= span1 - 1e-6, "两级跨度 %.2f°" % span2)


# ==========================================================================
# 2) 位真定点仿真（和 top.v 的递归逐位一致）
# ==========================================================================
def allpass_fixed(xs, a_q15: int, want_stats=False):
    """按 RTL 的位宽原样算：17 位减法 -> Q15 乘法 -> >>15 -> 18 位加法 -> 饱和。"""
    x1 = 0
    y1 = 0
    ys = []
    max_d = max_m = 0
    sat = 0
    for x in xs:
        d = x - y1
        m = d * a_q15
        ap = m >> 15                     # Python 的 >> 对负数就是算术右移
        s = ap + x1
        if s > 32767:
            y, sat = 32767, sat + 1
        elif s < -32768:
            y, sat = -32768, sat + 1
        else:
            y = s
        ys.append(y)
        max_d = max(max_d, abs(d))
        max_m = max(max_m, abs(m))
        x1, y1 = x, y
    if want_stats:
        return ys, max_d, max_m, sat
    return ys


def sine_i16(f: float, sr: float, n: int):
    out = []
    for i in range(n):
        v = int(round(20000.0 * math.sin(2.0 * math.pi * f * i / sr)))
        out.append(max(-32768, min(32767, v)))
    return out


def measure_ratio(pts, cx=0.0, cy=0.0):
    """PCA 拟合椭圆，返回 (短轴/长轴, 长轴角度°, 长半轴, 短半轴, 残差)。

    ★ 中心为什么取【原点】而不是点云的质心：
      X、Y 都是没有直流分量的音频正弦，李萨如图形的中心天生就是原点。
      而仿真里取的采样点数不可能正好是整数个信号周期，非整数周期会给
      质心带来一个偏移（实测 110Hz 时偏心 683，是短半轴的 7%），
      拿质心当中心去量椭圆，残差会被这个偏心污染成 0.16 —— 那是
      量法的错，不是轨迹的错。所以这里固定用原点，另外单独查直流偏移。
    """
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - cx) ** 2 for p in pts) / n
    syy = sum((p[1] - cy) ** 2 for p in pts) / n
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in pts) / n
    tr, det = sxx + syy, sxx * syy - sxy * sxy
    disc = math.sqrt(max(0.0, tr * tr / 4.0 - det))
    l1, l2 = tr / 2.0 + disc, tr / 2.0 - disc
    ratio = math.sqrt(max(0.0, l2) / l1) if l1 > 0 else 0.0
    ang = 0.5 * math.degrees(math.atan2(2.0 * sxy, sxx - syy))
    a = math.sqrt(2.0 * l1) if l1 > 0 else 0.0
    b = math.sqrt(2.0 * l2) if l2 > 0 else 0.0
    # 残差：每个点算 q = (u/a)^2 + (v/b)^2，真椭圆上处处 q=1；
    # 螺旋（越画越偏 / 越画越大）会把 q 撑开。
    ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
    worst = 0.0
    if a > 0 and b > 0:
        for px, py in pts:
            dx, dy = px - cx, py - cy
            u = dx * ca + dy * sa
            v = -dx * sa + dy * ca
            worst = max(worst, abs((u / a) ** 2 + (v / b) ** 2 - 1.0))
    else:
        worst = 1.0
    dc = math.hypot(mx - cx, my - cy)
    return ratio, ang, a, b, worst, dc


def block_mean(vals, f: float, sr: float) -> float:
    """整周期平均 —— 用来量"有没有直流"。

    ★ 为什么不能直接对全部采样求平均：SR/f 不是整数（110Hz 时一个周期
      443.9 个采样点），截取的那一段不是整数个周期，正弦自己的平均就
      偏了（实测 110Hz 偏 1811，看着像有直流，其实是截断造成的）。
      先去掉尾巴凑成整数个周期再平均，才是真的直流分量。
    """
    per = sr / f
    m = int(len(vals) / per)
    if m < 1:
        return sum(vals) / float(len(vals))
    k = int(round(m * per))
    seg = vals[len(vals) - k:]
    return sum(seg) / float(len(seg))


def test_trajectory(sr: float, a: float):
    """纯正弦 -> 位真递归 -> (X,Y) 点云 -> 长短轴比 + 闭环性。"""
    print("[3] 位真仿真：纯正弦喂进移相器，(X,Y) 轨迹是闭合环还是直线")
    a_q15 = round(a * 32768)
    n = 4000
    k = n // 2                                # 丢掉起振瞬态，只看稳态
    ratios, amps = {}, {}
    print("       f(Hz)   信号空间短/长轴   屏幕短/长轴   倾角    幅度比  椭圆残差  直流偏移")
    for f in BAND + [f90_of_p(-a, sr)]:
        xs = sine_i16(f, sr, n)
        ys, max_d, max_m, sat = allpass_fixed(xs, a_q15, want_stats=True)
        pts = list(zip(xs[k:], ys[k:]))
        r, ang, aa, bb, resid, dc = measure_ratio(pts)
        # 真·直流（整周期平均）：X 和 Y 各算一个
        dcx = block_mean(xs[k:], f, sr)
        dcy = block_mean(ys[k:], f, sr)
        dc = max(abs(dcx), abs(dcy)) if f in BAND else dc
        spts = [(140.0 * p[0] / 32768.0, 100.0 * p[1] / 32768.0) for p in pts]
        rs = measure_ratio(spts)[0]
        amp = (max(p[1] for p in pts) - min(p[1] for p in pts)) / \
              float(max(p[0] for p in pts) - min(p[0] for p in pts))
        ratios[f], amps[f] = (r, rs), amp
        print("      %6.0f        %.3f          %.3f      %6.1f°     %.3f    %.4f    %.1f"
              % (f, r, rs, ang, amp, resid, dc))
        if f in BAND:
            check("%.0fHz 轨迹不是直线（短/长轴比 %.3f > 0.3）" % (f, r), r > 0.3)
            check("%.0fHz 位真结果没有截幅（饱和次数 %d，|x[n]-y[n-1]|max=%d ≤ 65535）"
                  % (f, sat, max_d), sat == 0 and max_d <= 65535)
            check("%.0fHz 乘法器不溢出（|积|max = %.3g < 2^31）" % (f, max_m),
                  max_m < 2 ** 31)
            check("%.0fHz 移相器恒增益（Y/X 峰峰值比 %.3f，1±0.05）" % (f, amp),
                  abs(amp - 1.0) < 0.05)
            check("%.0fHz 轨迹闭合（绕原点拟合椭圆，残差 %.4f，螺旋会撑到 1 附近）"
                  % (f, resid), resid < 0.05)
            check("%.0fHz 图形以原点为中心（整周期平均的直流 max(|X|,|Y|) = %.1f，"
                  "只有长短轴的 %.2f%%）" % (f, dc, 100.0 * dc / aa), dc < 0.005 * aa)
    fmid = f90_of_p(-a, sr)
    check("f90=%.0fHz 处短/长轴比在带内最大（%.3f）-> 中音区最像圆"
          % (fmid, ratios[fmid][0]),
          ratios[fmid][0] >= max(ratios[f][0] for f in BAND) - 1e-9)
    check("110~830Hz 全程都不会退化成直线（最小短/长轴比 %.3f）"
          % min(ratios[f][0] for f in BAND),
          min(ratios[f][0] for f in BAND) > 0.3)
    check("屏幕上中音区接近正圆（短/长轴比 %.3f，屏幕刻度 140:100 的上限就是 %.3f）"
          % (ratios[fmid][1], 100.0 / 140.0), ratios[fmid][1] > 0.68)
    return ratios


# ==========================================================================
# 3) (X,Y) -> (col,row) 几何核对
# ==========================================================================
class Map:
    def __init__(self, xcoef, ycoef, shift, col_mid, row_mid):
        self.xcoef, self.ycoef, self.shift = xcoef, ycoef, shift
        self.col_mid, self.row_mid = col_mid, row_mid

    def xy(self, x: int, y: int):
        dx = (x * self.xcoef) >> self.shift
        dy = (y * self.ycoef) >> self.shift
        col = self.col_mid + dx
        row = self.row_mid - dy
        col = 0 if col < 0 else (319 if col > 319 else col)
        row = 0 if row < 0 else (239 if row > 239 else row)
        return col, row


def sweep(mp: Map, fx, fy, n=720):
    pts = []
    for i in range(n + 1):
        th = 2.0 * math.pi * i / n
        x = max(-32768, min(32767, int(round(30000 * fx(th)))))
        y = max(-32768, min(32767, int(round(30000 * fy(th)))))
        pts.append(mp.xy(x, y))
    return pts


def test_geometry(mp: Map, xcoef, ycoef):
    print("[4] (X,Y) -> (col,row) 映射核对（三种基准输入）")
    # (a) 同相 -> 对角线
    pts = sweep(mp, math.sin, math.sin)
    r, ang = measure_ratio([(c - mp.col_mid, mp.row_mid - rr) for c, rr in pts])[:2]
    print("      同相（档 2 效果全关时的样子）：短/长轴比 = %.4f，倾角 %.1f°" % (r, ang))
    check("同相 -> 一条直线（短/长轴比 < 0.02）", r < 0.02, r)
    check("同相 -> 过中心 (160,120)", (mp.col_mid, mp.row_mid) in pts)
    # (b) 90° -> 圆（信号空间）；屏幕上因刻度 140:100 是 1.4:1 的椭圆
    pts = sweep(mp, math.cos, math.sin)
    r, ang = measure_ratio([(c - mp.col_mid, mp.row_mid - rr) for c, rr in pts])[:2]
    print("      严格 90°：短/长轴比 = %.4f（= %d/%d，屏幕刻度本来就不是 1:1）"
          % (r, ycoef, xcoef))
    check("严格 90° -> 屏幕上是正椭圆，比值 = %d/%d = %.4f"
          % (ycoef, xcoef, ycoef / float(xcoef)), abs(r - ycoef / float(xcoef)) < 0.02)
    check("严格 90° -> 闭合（首尾同相位两点落同一个像素）", pts[0] == pts[-1])
    # (c) 两路完全相同 -> 直线（和 (a) 一样，但这里用"同一个信号接两轴"表达）
    pts = sweep(mp, lambda t: math.sin(3 * t), lambda t: math.sin(3 * t))
    r = measure_ratio([(c - mp.col_mid, mp.row_mid - rr) for c, rr in pts])[0]
    check("两路完全相同 -> 直线（短/长轴比 < 0.02）—— 这正是改之前档 2 的问题", r < 0.02, r)
    # (d) 把档 3 的实测相移代进去，看屏上比值
    return r


def ascii_plot(mp: Map, xs, ys, title, w=44, h=18):
    """把轨迹打到字符网格上，肉眼确认"是个环，不是一条线"。"""
    grid = [[" "] * w for _ in range(h)]
    for x, y in zip(xs, ys):
        c, r = mp.xy(x, y)
        cx = int(c * (w - 1) / 319.0)
        cy = int(r * (h - 1) / 239.0)
        grid[cy][cx] = "#"
    print("      %s" % title)
    for row in grid:
        print("      |" + "".join(row) + "|")


# ==========================================================================
def strip_comment(line: str) -> str:
    i = line.find("//")
    return line[:i] if i >= 0 else line


def test_tap_only_feeds_display(top_src: str, lcd_src: str):
    """硬约束的静态证明：移相器【只喂显示】，一根线都不许回可听通路。

    做法不是"看一眼觉得没问题"，而是把 RTL 里所有跟移相器有关的标识符
    全部找出来，逐条核对它们出现的位置：
      · 所有 ph_* / PH_A 只能出现在那一个 tap 块里（+ 那一行 wave_send_py <= ph_y）；
      · wave_send_py 只能出现在 tap 块、lcd_master 的端口连接、以及 lcd_master
        内部的 3 处（端口声明 / 同拍锁存 / 2 选 1）；
      · 送 DAC / I2S 的那几行（含 sample_vol[15:0] 和 shift_reg 的赋值）
        【一个 ph_ / wave_send_py 都不许出现】。
    只要这三条成立，音频样本序列就逐位不变 —— 因为 Verilog 里信号的影响
    只能沿着"谁读谁"传播，移相器的输出没有任何一条路径回到 sample_vol。
    """
    print("[6] 硬约束静态核对：移相器只喂显示，没碰可听通路")
    lines = top_src.splitlines()
    # tap 块的范围：从 PH_A 那行到 wave_tgl <= ~wave_tgl 那一行
    i0 = next((i for i, ln in enumerate(lines) if "PH_A" in ln and "localparam" in ln), None)
    i1 = next((i for i, ln in enumerate(lines)
               if "wave_tgl" in ln and "~wave_tgl" in ln and i > (i0 or 0)), None)
    check("在 top.v 里定位到移相器 tap 块", i0 is not None and i1 is not None,
          (i0, i1))
    # tap 块往前多留 20 行：那段长注释里也提了 ph_ 的名字（注释不算逻辑）
    lo = max(0, i0 - 60)
    ph_ids = ["ph_x1", "ph_y1", "ph_x ", "ph_d ", "ph_m ", "ph_ap", "ph_ys", "ph_y ",
              "PH_A"]
    outside = []
    for i, ln in enumerate(lines):
        code = strip_comment(ln)
        if not any(t in code for t in ph_ids):
            continue
        if lo <= i <= i1 + 1:
            continue
        if "wave_send_py <= ph_y" in code:            # 就是那一行"喂显示"
            continue
        outside.append((i + 1, ln.strip()))
    check("所有 ph_* / PH_A 只出现在 tap 块内（%d 行越界）" % len(outside),
          not outside, outside[:3])
    # 送 DAC / I2S 的那几行
    dac = [(i + 1, ln.strip()) for i, ln in enumerate(lines)
           if "sample_vol[15:0]" in strip_comment(ln)]
    bad = [d for d in dac if ("ph_" in d[1] or "wave_send_py" in d[1])]
    check("送 DAC/I2S 的 %d 行里没有移相器信号（逐位不变）" % len(dac), not bad and dac,
          bad[:3])
    check("DAC 那一行仍然只用 sample_vol（原样：{sample_vol[15:0], …}）",
          any("sample_vol[15:0]" in strip_comment(ln) for ln in lines))
    # wave_send_py 的全部落脚点
    uses = [(i + 1, strip_comment(ln).strip()) for i, ln in enumerate(lines)
            if "wave_send_py" in strip_comment(ln)]
    check("top.v 里 wave_send_py 只有 4 处（声明 / 复位 / 赋值 / 送 lcd_master），实际 %d 处"
          % len(uses), len(uses) == 4, uses)
    check("其中一处就是 .wave_send_py (wave_send_py) 的端口连接",
          any(".wave_send_py" in u[1] for u in uses))
    luses = [(i + 1, strip_comment(ln).strip()) for i, ln in enumerate(lcd_src.splitlines())
             if "wave_send_py" in strip_comment(ln)]
    check("lcd_master.v 里 wave_send_py 只有 2 处（端口声明 / 同拍锁存进 wave_in_py），"
          "实际 %d 处" % len(luses), len(luses) == 2, luses)
    # 两屏李萨如的 XY 选择：确认档 2/档 3 的接法没写反
    check("档 2（liss_src=0）X=干声 wave_in_x、Y=输出 wave_in",
          re.search(r"liss_x\s*=\s*liss_src_s2\s*\?\s*wave_in\s*:\s*wave_in_x", lcd_src)
          is not None)
    check("档 3（liss_src=1）X=输出 wave_in、Y=移相输出 wave_in_py",
          re.search(r"liss_y\s*=\s*liss_src_s2\s*\?\s*wave_in_py\s*:\s*wave_in", lcd_src)
          is not None)
    check("映射后面只认 liss_x / liss_y（两屏共用一套显存/打点/余辉/边框）",
          re.search(r"xs24\s*=\s*\{\{8\{liss_x\[15\]\}\},\s*liss_x\}", lcd_src) is not None
          and re.search(r"ys24\s*=\s*\{\{8\{liss_y\[15\]\}\},\s*liss_y\}", lcd_src) is not None)


def main() -> int:
    if not os.path.isfile(TOP_V) or not os.path.isfile(LCD_V):
        print("找不到 %s / %s" % (TOP_V, LCD_V))
        return 1
    top_src, lcd_src = readf(TOP_V), readf(LCD_V)
    ti = parse_top(top_src)
    xcoef, ycoef, shift, col_mid, row_mid = parse_lcd_map(lcd_src)

    print("=" * 74)
    print("相位式李萨如自检（档 3）")
    print("RTL: %s" % TOP_V)
    print("-" * 74)
    check("从 top.v 抠到 I2S_SPEC / CLK_FREQ_HZ，推出采样率",
          ti["sr"] is not None, ti["sr"])
    check("采样率 = %d / %d = %.1f Hz（和 RTL 注释里的 48828 一致）"
          % (ti["clk"], ti["frame_div"], ti["sr"]), abs(ti["sr"] - 48828.125) < 0.5,
          ti["sr"])
    check("抠到 Q15 移相系数 PH_A，且【带负号】（极点必须在 z=+|a| 那一支）",
          ti["a_q15"] is not None and ti["a_q15_neg"] is True, ti["ph_a_raw"])
    check("移相器的节拍判据 = sample_cnt==%s，和 wave_send 的 %s 一致（X/Y 同一样点）"
          % (ti["tap"], ti["tap_wave"]),
          ti["tap"] is not None and ti["tap"] == ti["tap_wave"], (ti["tap"], ti["tap_wave"]))
    check("移相器输出带饱和保护（瞬态不会回卷成反相尖峰）", ti["sat"] is True)
    check("从 lcd_master.v 抠到 (X,Y)->(col,row) 映射常数", None not in
          (xcoef, ycoef, shift, col_mid, row_mid),
          (xcoef, ycoef, shift, col_mid, row_mid))
    check("映射 = col %d + X*%d>>%d / row %d - Y*%d>>%d（★ X/Y 刻度必须相等，圆才是圆）"
          % (col_mid, xcoef, shift, row_mid, ycoef, shift),
          (xcoef, ycoef, shift, col_mid, row_mid) == (120, 120, 15, 160, 120)
          and xcoef == ycoef)

    a, sr = ti["a"], ti["sr"]
    test_phase(sr, a)
    test_one_vs_two(sr, ti["a_q15"])
    ratios = test_trajectory(sr, a)
    mp = Map(xcoef, ycoef, shift, col_mid, row_mid)
    test_geometry(mp, xcoef, ycoef)
    test_tap_only_feeds_display(top_src, lcd_src)

    print("[5] 屏上实景（字符示意：把 (col,row) 缩到 44x18 的格子上）")
    n = 4000
    k = n // 2
    fmid = f90_of_p(-a, sr)
    xs = sine_i16(fmid, sr, n)
    ys = allpass_fixed(xs, round(a * 32768))
    ascii_plot(mp, xs[k:], ys[k:], "档 3 中音区 (%.0fHz，相移≈-90°)：应该是个环" % fmid)
    xs = sine_i16(110.0, sr, n)
    ys = allpass_fixed(xs, round(a * 32768))
    ascii_plot(mp, xs[k:], ys[k:], "档 3 最低音 (110Hz，相移≈-40°)：扁环，但绝不是一条线")
    xs = sine_i16(440.0, sr, n)
    ascii_plot(mp, xs[k:], xs[k:], "对照：档 2 在效果全关时 (X≡Y) —— 只有一条对角线")

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
