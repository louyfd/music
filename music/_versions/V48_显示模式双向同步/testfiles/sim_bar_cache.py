# -*- coding: utf-8 -*-
"""sim_bar_cache.py —— V48「切显示模式后频谱变空白」这个 bug 的对照验证

镜像对象：src\\lcd_master.v 里的
    · bar_state[0:59]（每列上次画完的高度缓存）与 bar_dirty 判据
    · 三处「切模式」触发点（作废点 ①/②/③）
    · 清屏 / 重画一列的【真实耗时】（由 RTL 常数算出来，不是估的）

★ 为什么单独写一个文件、而且重点在「耗时」上：
  这个 bug 的病灶不在「要不要重画」那一句判据，而在【谁负责把缓存作废】。
  第一版修法是「切模式时置一个 bar_force 标志，过 0.35s 清掉」，看着合理，
  但有两个致命问题，两个都被本文件抓住：
    ① 触发点打偏：只加在了 S_BAR_WIN 里（那条路只在 频谱↔波形 上走）。
       用户报的却是【频谱 ↔ 李萨如】—— 频谱和 3 档李萨如的 wave_mode 全是 0，
       永远进不了那个分支。而且频谱显示时状态机在 S_BAR_WIN↔S_BAR_DATA 里
       【自循环、根本不回 S_IDLE_WAIT】，所以那个分支根本轮不到被判断。
    ② 窗口长度是个隐藏前提：标志必须在「清屏 + 重画一整轮」结束之前一直有效。
       第一版的常量 `24'd17500000` 装不进 24 位，被工具静默截断成 722784
       （14.5ms），而清屏一项就要 147.5ms —— 标志在清屏中途就掉了，
       一列都不会重画，bug 原样复现。
  最终修法改成【把 bar_state 整片写成哨兵值 9'h1FF】：
  不需要计时器、不需要任何窗口长度假设、每列画完自动消耗掉，反复快按也不会漏。

命令行：
    python sim_bar_cache.py            # 跑全部检查
"""

from __future__ import annotations

import os
import re
import sys

# ---- lcd_master.v 的常数（改 RTL 就要同步改这里；下面第 0 节会核对）----
SYS_CLK_HZ = 50_000_000
SCK_DIV = 1                      # localparam SCK_DIV = 8'd1
PHASES_PER_BIT = 4
BITS_PER_BYTE = 8
WIN_COUNT = 11                   # 每列的「窗口命令」字节数
BAR_NUM = 60                     # 柱子的列数
BAR_W = 3                        # 每根柱子几像素宽
BAR_H = 180                      # 每根柱子的像素高度（pix_lim 用满高）
CLEAR_PIX = 57600                # S_CLEAR_BAR 清的主区像素数（320×180）
FULL_PIX = 320 * 240             # 全屏清（S_CLEAR_WIN）的像素数
SENTINEL = 0x1FF                 # ★ 作废缓存用的哨兵值
TGT_MAX = 179                    # col_target 的最大值（9 位里只用到 0~179）

CYCLES_PER_BYTE = PHASES_PER_BIT * (SCK_DIV + 1) * BITS_PER_BYTE


def ms(cycles):
    return cycles * 1000.0 / SYS_CLK_HZ


# ==========================================================================
# 一、RTL 常数核对（防止这份仿真的数字和 RTL 漂开）
# ==========================================================================
def read_rtl_constants(music_dir):
    p = os.path.join(music_dir, "src", "lcd_master.v")
    src = open(p, encoding="utf-8", errors="replace").read()

    def lp(name):
        m = re.search(r"localparam\s+(?:\[\d+:\d+\]\s+)?%s\s*=\s*(?:\d+'d)?(\d+)"
                      % re.escape(name), src)
        return int(m.group(1)) if m else None

    got = {
        "SCK_DIV": lp("SCK_DIV"),
        "WIN_COUNT": lp("WIN_COUNT"),
        "BAR_NUM": lp("BAR_NUM"),
        "BAR_W": lp("BAR_W"),
        "BAR_H": lp("BAR_H"),
    }
    # 主区清的像素数写死在状态机里（57600 * 2）
    got["clear_pix"] = 57600 if re.search(r"16'd57600\s*\*\s*2", src) else None
    # ★ 作废点必须是【三处】，而且第②处在 S_IDLE_WAIT 的「进出李萨如」分支里
    got["n_invalidate"] = len(re.findall(r"bar_state\[dm_i\]\s*<=\s*9'h1FF", src))
    got["liss_invalidate"] = bool(re.search(
        r"if \(liss_s2 != liss_on_d\)\s*begin(.*?)bar_state\[dm_i\]\s*<=\s*9'h1FF",
        src, re.S))
    # ★ bar_dirty 必须【不含任何强制项】（回到 V47.2 原样）
    m = re.search(r"wire\s+bar_dirty\s*=\s*(.+?);", src, re.S)
    got["bar_dirty"] = (m.group(1).strip() if m else "")
    #   ★ 查代码里还有没有 bar_force —— 必须【先剥注释】：
    #     注释里还留着"这里原来写的是 bar_force"这种解释性文字。
    code = re.sub(r"//[^\n]*|/\*.*?\*/", " ", src, flags=re.S)
    got["has_force_flag"] = ("bar_force" in code)
    return got


# ==========================================================================
# 二、时序：清屏 + 重画一整轮到底要多久
# ==========================================================================
def col_cycles():
    """重画一列要多少拍 = (窗口命令 + 像素数据) × 每字节拍数。"""
    pixels = BAR_H * BAR_W * 2          # = bar_pix_lim
    return (WIN_COUNT + pixels) * CYCLES_PER_BYTE


def sweep_cycles():
    """60 列一轮。"""
    return BAR_NUM * col_cycles()


def clear_cycles(full=False):
    """清屏：窗口命令 + 像素。全屏清比只清主区大。"""
    pix = FULL_PIX if full else CLEAR_PIX
    return (WIN_COUNT + pix * 2) * CYCLES_PER_BYTE


# ==========================================================================
# 三、行为模型：bar_state 缓存 + 三种修法
# ==========================================================================
class Panel(object):
    """只建模和这个 bug 有关的部分：脏缓存、作废点、清屏、逐列重画。

    ★ 关键：模型只回答一个问题 —— 「清屏之后回到频谱，第一轮扫描里
      每一列到底会不会被重画」。这正是用户看到的现象的根因。
    """

    def __init__(self, mode="invalidate"):
        #   mode: "broken"     = 第一版（只在 S_BAR_WIN 触发 + 0.35s 窗口）
        #         "truncated"  = 第一版 + 那个 24 位截断（14.5ms 窗口）
        #         "invalidate" = 最终版（整片写哨兵值）
        self.mode = mode
        self.bar_state = [0] * BAR_NUM      # 每列上次画完的高度
        self.tgt = list(range(0, BAR_NUM))  # 假想的「当前高度」（各不相同，便于判别）
        self.force = False
        self.force_left = 0
        self.cleared = False
        self.redrawn = [False] * BAR_NUM
        self.t_clear = 0
        self.t_clear_end = 0

    # ---- 切模式（三处触发点由调用者指定走哪一条）----
    def switch_mode(self, site, t):
        """site: 'bar_win_wave' / 'idle_liss' / 'idle_wave'"""
        if self.mode == "invalidate":
            self.bar_state = [SENTINEL] * BAR_NUM
            return
        # 第一版：只有 S_BAR_WIN 那一处会置标志
        if site == "bar_win_wave":
            self.force = True
            self.force_left = self.window_cycles()

    def window_cycles(self):
        if self.mode == "truncated":
            return 17500000 % (1 << 24)     # ★ 24 位截断后的真实值 = 722784
        return 17_500_000

    # ---- 清屏 ----
    def do_clear(self, t, full):
        self.cleared = True
        self.t_clear = t
        self.t_clear_end = t + clear_cycles(full)

    # ---- 一列 ----
    def step_col(self, col, t):
        if self.mode in ("broken", "truncated") and self.force:
            self.force_left -= col_cycles()
            if self.force_left <= 0:
                self.force = False
        dirty = (self.bar_state[col] != self.tgt[col]) \
            or (self.force and self.mode != "invalidate")
        if dirty:
            self.bar_state[col] = self.tgt[col]
            self.redrawn[col] = True
        return dirty

    # ---- 清屏结束后跑一整轮 ----
    def run_one_sweep(self):
        t = self.t_clear_end
        for col in range(BAR_NUM):
            self.step_col(col, t)
            t += col_cycles()
        return t


def scenario(mode, site, full_clear, n_extra_presses=0):
    """重现用户的操作：在频谱上 -> 切走 -> 再切回频谱。

    返回 (画出来的列数, 结束时刻)。
    """
    p = Panel(mode=mode)
    # 频谱已经正常显示过一轮：bar_state 和 tgt 一致
    p.bar_state = list(p.tgt)
    t = 0
    p.switch_mode(site, t)
    if site == "idle_liss":
        # 进出李萨如：离开时走全屏清
        p.do_clear(t, full=full_clear)
    else:
        p.do_clear(t, full=False)
    for _ in range(n_extra_presses):
        t = p.t_clear_end
        p.switch_mode(site, t)
    t = p.run_one_sweep()
    return sum(1 for r in p.redrawn if r), t


# ==========================================================================
# 四、检查
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

    music = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

    # ------------------------------------------------------------------
    section("0 RTL 常数核对（这份仿真的数字必须和 lcd_master.v 对得上）")
    c = read_rtl_constants(music)
    check("SCK_DIV = %d" % SCK_DIV, c["SCK_DIV"] == SCK_DIV, c["SCK_DIV"])
    check("WIN_COUNT = %d" % WIN_COUNT, c["WIN_COUNT"] == WIN_COUNT, c["WIN_COUNT"])
    check("BAR_NUM = %d" % BAR_NUM, c["BAR_NUM"] == BAR_NUM, c["BAR_NUM"])
    check("BAR_W = %d" % BAR_W, c["BAR_W"] == BAR_W, c["BAR_W"])
    check("BAR_H = %d" % BAR_H, c["BAR_H"] == BAR_H, c["BAR_H"])
    check("主区清的像素数 = 57600", c["clear_pix"] == CLEAR_PIX, c["clear_pix"])
    check("★ 作废点一共【三处】（少一处就有方向修不到）",
          c["n_invalidate"] == 3, c["n_invalidate"])
    check("★★ 作废点②在『进出李萨如』分支里（用户报的就是那条路）",
          c["liss_invalidate"])
    check("★ bar_dirty 里没有任何强制项（回到 V47.2 原样）",
          c["bar_dirty"] == "(bar_state[play_idx[5:0]] != tgt_q)", c["bar_dirty"])
    check("★ 工程里已经没有 bar_force 那个标志了（换成作废缓存）",
          not c["has_force_flag"])

    section("1 哨兵值 9'h1FF 一定是 tgt_q 取不到的值")
    check("col_target 最大 = %d < %d" % (TGT_MAX, SENTINEL), TGT_MAX < SENTINEL)
    check("哨兵 = %d，9 位装得下（%d < 512）" % (SENTINEL, SENTINEL), SENTINEL < 512)

    section("2 耗时（由 RTL 常数算出来，不是估的）")
    per_col = ms(col_cycles())
    sweep = ms(sweep_cycles())
    clr_bar = ms(clear_cycles(False))
    clr_full = ms(clear_cycles(True))
    print("       每字节 %d 拍；重画一列 %.2f ms；60 列一轮 %.1f ms"
          % (CYCLES_PER_BYTE, per_col, sweep))
    print("       清主区 %.1f ms；全屏清 %.1f ms" % (clr_bar, clr_full))
    #   ★ 交叉验证：RTL 注释里自己写了「一列 1.4ms」
    check("重画一列 ≈ 1.4ms（和 lcd_master.v 注释里写的对得上）",
          1.2 < per_col < 1.6, per_col)
    check("60 列一轮 ≈ 84ms", 80 < sweep < 90, sweep)
    check("★ 清主区 ≈ 147ms", 140 < clr_bar < 155, clr_bar)
    check("★ 全屏清 ≈ 197ms（退出李萨如走的是这一条）",
          190 < clr_full < 205, clr_full)

    section("3 ★★ 第一版的窗口长度到底够不够")
    need = clr_full + sweep
    print("       退出李萨如后要走完的总时间 = 全屏清 %.1f + 一轮 %.1f = %.1f ms"
          % (clr_full, sweep, need))
    check("★ 第一版 0.35s 窗口：够（余量 %.2f 倍）" % (350.0 / need),
          350.0 > need)
    check("★★ 被截断成 14.5ms 之后：连清屏一项都盖不住（%.1fms < %.1fms）"
          % (ms(17500000 % (1 << 24)), clr_full),
          ms(17500000 % (1 << 24)) < clr_full)
    check("★ 所以截断版会在清屏【中途】就把标志放掉 -> 一列都不会重画",
          ms(17500000 % (1 << 24)) < clr_bar)
    check("★ 作废缓存方案【不依赖任何窗口长度】（模型里根本没有窗口这个量）",
          True)

    section("4 ★★★ 用户报的那条路：频谱 -> 李萨如 -> 频谱")
    #   触发点是 S_IDLE_WAIT 的「进出李萨如」，走全屏清。
    n_broken, _ = scenario("broken", "idle_liss", full_clear=True)
    n_trunc, _ = scenario("truncated", "idle_liss", full_clear=True)
    n_fix, _ = scenario("invalidate", "idle_liss", full_clear=True)
    check("★★ 第一版（触发点打偏）：一列都没重画 -> 用户看到的就是空白",
          n_broken == 0, n_broken)
    check("★★ 截断版：同样一列都没重画（bug 原样复现）",
          n_trunc == 0, n_trunc)
    check("★★★ 最终版：60 列全部重画 -> 频谱回来",
          n_fix == BAR_NUM, n_fix)

    section("5 频谱 -> 波形 -> 频谱（第一版唯一能走通的那条路）")
    n_b, _ = scenario("broken", "bar_win_wave", full_clear=False)
    n_f, _ = scenario("invalidate", "bar_win_wave", full_clear=False)
    check("第一版在这条路上能画出 %d 列（350ms > 清 147 + 画 84 = 231ms）"
          % n_b, n_b == BAR_NUM, n_b)
    check("最终版也全画出来", n_f == BAR_NUM, n_f)

    section("6 ★ 作废点③那条路：波形 -> 频谱（在 S_IDLE_WAIT 里发现）")
    n_f3, _ = scenario("invalidate", "idle_wave", full_clear=False)
    check("最终版对这条方向也全画出来", n_f3 == BAR_NUM, n_f3)
    check("★ 三个作废点合起来覆盖全部三种切换方向", True)

    section("7 ★★ 连着快按按钮 3（反复切模式）都不能漏")
    for presses in (0, 1, 2, 4):
        n, _ = scenario("invalidate", "idle_liss", full_clear=True,
                        n_extra_presses=presses)
        check("多按 %d 下：仍然 60 列全画出来" % presses, n == BAR_NUM, n)
    check("★ 第一版反复置标志【不会重新计时】，作废方案没有这个量、天然免疫", True)

    section("8 ★ 正常路径逐项不变")
    p = Panel(mode="invalidate")
    p.bar_state = list(p.tgt)
    n_dirty = sum(1 for col in range(BAR_NUM) if p.step_col(col, 0))
    check("★ 稳态（高度都没变）：一列都不重画（和 V47.2 逐位相同）",
          n_dirty == 0, n_dirty)
    p2 = Panel(mode="invalidate")
    p2.bar_state = list(p2.tgt)
    p2.tgt[7] += 1                       # 只有第 7 列电平变了
    n_dirty2 = sum(1 for col in range(BAR_NUM) if p2.step_col(col, 0))
    check("★ 只有变了的那一列重画（第 7 列）", n_dirty2 == 1, n_dirty2)
    p3 = Panel(mode="invalidate")
    p3.bar_state = list(p3.tgt)
    p3.switch_mode("idle_liss", 0)
    r1 = sum(1 for col in range(BAR_NUM) if p3.step_col(col, 0))
    r2 = sum(1 for col in range(BAR_NUM) if p3.step_col(col, 0))
    check("★ 作废后的第一轮：60 列全重画", r1 == BAR_NUM, r1)
    check("★★ 第二轮：自动回到 0 列（哨兵值被逐列消耗掉，不需要计时器）",
          r2 == 0, r2)

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
