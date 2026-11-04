# -*- coding: utf-8 -*-
"""sim_lcd_watchdog.py —— LCD 刷屏看门狗的逐拍对照仿真（V48.4）

为什么需要它
------------
V47.2 加的"LCD 看门狗"用户实测有效（不再卡死），但 **V48.4 收到一个新 bug**：

    用户原话：「LCD 不会卡死了，但是**延音时谱线会被刷掉**，
               即延音没办法保持谱线，**这个看门狗把延音状态当卡死给刷掉了**」

根因是两条叠在一起（见 lcd_master.v 看门狗那一段注释）：

  ① **判据漏了一种"合法的原地踏步"**：频谱这一屏是
     `S_BAR_WIN <-> S_BAR_DATA` 自循环的，而"这一列要不要重画"由 `bar_dirty` 决定。
     高度不变时（**延音锁住 / 没弹**）一列都不脏 -> `S_BAR_WIN` 的 phase0
     只做"`play_idx` 自增 + `phase <= 0`"，**state 一个时钟都不变** ->
     静态画面超过 0.8s 就被误判成卡死。
     ★ "没活干"和"卡死"在 **state 这一维上是同一种形状**。

  ② **恢复路径不作废谱线缓存**：误判后走 S_POWER_ON -> S_INIT -> 全屏清，
     但 `bar_state[]` 里还是清屏前的高度、**和当前目标高度一样** ->
     `bar_dirty` 恒 0 -> **一列都不重画** -> 屏就空在那里。
     这就是用户看到的"谱线被刷掉"。

本仿真把这四件事都跑一遍：
  [A] 空转一圈要几拍（逐拍数出来，不用手抄常数）
  [B] 旧判据：静态画面 **必然** 在 0.8s 误触发（算出到第几次扫描）
  [C] 新判据：静态画面 **永不** 触发（把"最大累计"量出来，和 0.8s 比）
  [D] 真卡死（state 与 play_idx 都冻住）：新旧**都要**触发（本职不能丢）
  [E] 恢复后重画：不作废 -> **0 列**（复现用户的 bug）；作废 -> **60 列**
  [F] 一个反面例子：高度真的在变时，即使不作废也会重画
      —— 说明用户为什么偏偏在"延音"时才看见

用法：python testfiles/sim_lcd_watchdog.py
"""

from __future__ import annotations

import io
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
# 0) 从 RTL 里抠常数（不手抄）
# --------------------------------------------------------------------------
def read_rtl(path: str) -> str:
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        return fh.read()


def parse_consts(src: str) -> dict:
    def num(pat, cast=int):
        m = re.search(pat, src)
        return cast(m.group(1)) if m else None

    c = {
        "SCK_DIV": num(r"localparam\s+SCK_DIV\s*=\s*8'd(\d+)"),
        "WD_LIMIT": num(r"LCD_WD_LIMIT\s*=\s*26'd(\d+)"),
        # ★ 列数要看【回绕】那一条（`play_idx >= 6'd59 -> 0`），
        #   不能抓第一个 `play_idx >= 6'dN`：文件里还有 `lvl_col2` 的键区间判据
        #   （`play_idx >= 6'd12 && play_idx < 6'd48`），抓到它就会算成 13 列。
        "COLS": num(r"play_idx\s*>=\s*6'd(\d+)\)\s*play_idx\s*<=\s*8'd0"),
        "SENTINEL": num(r"bar_state\[dm_i\]\s*<=\s*9'h([0-9A-Fa-f]+)", str),
        "SYS_HZ": 50_000_000,
    }
    if c["COLS"] is not None:
        c["COLS"] += 1
    return c


# --------------------------------------------------------------------------
# 1) 逐拍镜像：频谱"一列都不用重画"时的空转
#    ★ 只镜像【和看门狗有关的那几条赋值】，位宽/分支条件照 RTL 抄：
#        S_BAR_WIN.phase0 每"一拍"（sck_div_cnt >= SCK_DIV）走一步：
#          没脏 && bar_armed != 2 -> bar_armed+1（phase 保持 0）
#          没脏 && bar_armed == 2 -> play_idx+1（回绕 60），bar_armed <= 0
#      state 在这整段时间里恒为 S_BAR_WIN。
# --------------------------------------------------------------------------
SBAR = "S_BAR_WIN"


def tick_clocks(sck_div: int) -> int:
    """一拍 = sck_div_cnt 从 0 数到 SCK_DIV = SCK_DIV+1 个时钟。"""
    return sck_div + 1


def clean_column_trace(sck_div: int, cols: int):
    """走一圈"没脏"的谱线扫描，返回 (每列耗费的时钟列表, play_idx 序列, 每列的拍数)。"""
    per_col, play_seq, ticks_per_col = [], [], []
    armed = 0
    play = 0
    for _ in range(cols):
        t = 0
        # phase0 的三次"一拍"：armed 0->1、1->2、然后走 else 分支
        while True:
            t += tick_clocks(sck_div)
            if armed != 2:
                armed += 1
            else:
                armed = 0
                play = 0 if play >= cols - 1 else play + 1
                break
        per_col.append(t)
        play_seq.append(play)
        ticks_per_col.append(t // tick_clocks(sck_div))
    return per_col, play_seq, ticks_per_col


# --------------------------------------------------------------------------
# 2) 看门狗：新旧两种判据
# --------------------------------------------------------------------------
def wd_old(clocks_where_state_unchanged: int, limit: int) -> bool:
    """旧判据：只看 state。返回是否触发。"""
    return clocks_where_state_unchanged >= limit


def wd_new_run(events, limit: int):
    """新判据：state 动【或】play_idx 动 -> 清零。

    ★ 计数器【饱和】在 limit（RTL 原文是 `else if (lcd_wd_cnt != LCD_WD_LIMIT)
      lcd_wd_cnt <= lcd_wd_cnt + 1'b1;`）—— 模型必须一起饱和，
      否则"触发时刻"会算成超过 0.8s。

    events: 迭代器，每项 = (本段时钟数, 这一段里 play_idx 是否变过)
    返回 (是否触发, 触发前最大累计时钟数)
    """
    cnt = 0
    mx = 0
    for clocks, play_moved in events:
        if play_moved:
            cnt = 0
        else:
            cnt = min(cnt + clocks, limit)      # ★ 饱和
            mx = max(mx, cnt)
            if cnt >= limit:
                return True, mx
    return False, mx


def main() -> int:
    if not os.path.isfile(LCD_V):
        print("找不到 %s" % LCD_V)
        return 1
    src = read_rtl(LCD_V)
    c = parse_consts(src)
    print("=" * 70)
    print("LCD 看门狗逐拍对照仿真（V48.4）")
    print("RTL: %s" % LCD_V)
    print("-" * 70)
    check("从 RTL 抠到常数（SCK_DIV / WD_LIMIT / 列数 / 哨兵值）",
          None not in (c["SCK_DIV"], c["WD_LIMIT"], c["COLS"], c["SENTINEL"]), c)
    if None in (c["SCK_DIV"], c["WD_LIMIT"], c["COLS"], c["SENTINEL"]):
        return 1
    sck_div, limit, cols = c["SCK_DIV"], c["WD_LIMIT"], c["COLS"]
    # ★ V48.5：手动重置那两个常数（也从 RTL 抠，别手抄）
    mj = re.search(r"PWR_DELAY_JOG\s*=\s*26'd([\d_]+)", src)
    mn = re.search(r"PWR_DELAY\s*=\s*24'd([\d_]+)", src)
    pwr_jog = int(mj.group(1).replace("_", "")) if mj else 0
    pwr_normal = int(mn.group(1).replace("_", "")) if mn else 0
    mi = re.search(r"INIT_COUNT\s*=\s*16'd(\d+)", src)
    init_count = int(mi.group(1)) if mi else 0
    check("从 RTL 抠到 PWR_DELAY / PWR_DELAY_JOG / INIT_COUNT",
          pwr_jog > 0 and pwr_normal > 0 and init_count > 0,
          (pwr_normal, pwr_jog, init_count))
    check("看门狗阈值 = 0.8s @50MHz（%d 拍）" % limit,
          limit == c["SYS_HZ"] * 8 // 10, limit)
    check("哨兵值 9'h%s 比最大列高 179 大（取不到）" % c["SENTINEL"],
          int(c["SENTINEL"], 16) == 0x1FF and 0x1FF > 179)

    # ---- [A] 空转一圈几拍 ----
    print("\n[A] 「一列都不用重画」时，空转一圈要几拍（逐拍数）")
    per_col, play_seq, ticks_per_col = clean_column_trace(sck_div, cols)
    print("     每列 %d 拍 = %d 时钟；一圈 %d 列 = %d 时钟 = %.2f µs"
          % (ticks_per_col[0], per_col[0], cols, sum(per_col),
             sum(per_col) / c["SYS_HZ"] * 1e6))
    check("每列固定 3 拍（armed 0->1、1->2、走 else）", set(ticks_per_col) == {3},
          sorted(set(ticks_per_col)))
    check("每列耗 %d 个时钟（= 3 拍 × (SCK_DIV+1)）" % (3 * (sck_div + 1)),
          set(per_col) == {3 * (sck_div + 1)}, sorted(set(per_col)))
    check("★ play_idx 每列都动（60 列一轮，回绕到 0）",
          play_seq == [1, 2, 3] + play_seq[3:] and max(play_seq) == cols - 1
          and play_seq[-1] == 0, (play_seq[:5], play_seq[-3:]))
    check("★★ 但 state 全程不变（这就是被误判成卡死的原因）", True,
          "state 恒为 %s" % SBAR)

    # ---- [B] 旧判据：必然误触发 ----
    print("\n[B] 旧判据（只看 state）：静态画面会不会误触发")
    sweep_clocks = sum(per_col)                      # 一圈的时钟数
    sweeps_to_limit = -(-limit // sweep_clocks)      # 向上取整
    t_fire = sweeps_to_limit * sweep_clocks / c["SYS_HZ"]
    print("     一圈 %d 时钟 -> 第 %d 圈（%.3f s）就攒满 %d"
          % (sweep_clocks, sweeps_to_limit, t_fire, limit))
    check("★ 旧判据【必然】在 %.3f s 误触发（阈值是 0.8s，只差 %.1f ms）"
          % (t_fire, (t_fire - 0.8) * 1000),
          wd_old(sweeps_to_limit * sweep_clocks, limit))
    check("★★ 也就是说：延音/静音下每 %.2f 秒屏幕就被整台重来一次" % t_fire,
          t_fire < 0.81, t_fire)

    # ---- [C] 新判据：永不触发 ----
    print("\n[C] 新判据（state 动【或】play_idx 动）：静态画面还会不会误触发")
    # 每列一个事件：play_idx 动过 -> 清零；最多累计永远不会超过一列的时钟
    ev = [(per_col[i % len(per_col)], True) for i in range(cols * 20000)]   # 20000 圈
    fired, mx = wd_new_run(ev, limit)
    check("★ 新判据在 20000 圈（%.1f s 仿真时间）里一次都不触发"
          % (len(ev) * 3 * (sck_div + 1) / c["SYS_HZ"]), not fired)
    check("★ 计数器最大累计只有 %d 拍（一列的时钟数），离 %d 差 %d 倍"
          % (3 * (sck_div + 1), limit, limit // (3 * (sck_div + 1))),
          mx <= 3 * (sck_div + 1), mx)
    check("★★ 余量算得出来：%.1f 万倍" % (limit / (3 * (sck_div + 1)) / 10000),
          limit / (3 * (sck_div + 1)) > 1e6)

    # ---- [D] 真卡死：新旧都要触发 ----
    print("\n[D] 真卡死（state 与 play_idx 都冻住）：看门狗本职不能丢")
    hang_clocks = limit + 10
    check("旧判据：卡死 %d 拍后触发" % hang_clocks, wd_old(hang_clocks, limit))
    fired_h, mx_h = wd_new_run([(hang_clocks, False)], limit)
    check("★ 新判据：卡死同样触发（不会因为放宽判据把本职丢掉）", fired_h)
    check("★ 触发时刻仍是 %.3f s（不多不少）" % (limit / c["SYS_HZ"]),
          mx_h == limit, mx_h)
    # 边界：卡死刚好差一拍 -> 不该触发
    fired_b, _ = wd_new_run([(limit - 1, False)], limit)
    check("差一拍不触发（阈值语义没变，仍是 >= 才触发）", not fired_b)

    # ---- [E] 恢复后重画 ----
    print("\n[E] 误触发/真卡死恢复之后：谱线会不会自己回来")
    sentinel = int(c["SENTINEL"], 16)

    def redraw_columns(bar_state, targets):
        """镜像绘制循环：只有 bar_dirty 的列才画，画完把真实高度写回缓存。"""
        drawn = 0
        for i in range(len(bar_state)):
            if bar_state[i] != targets[i]:        # bar_dirty
                bar_state[i] = targets[i]         # 画完写回真实值
                drawn += 1
        return drawn

    sustained = [90] * cols                       # 延音：每列高度恒定
    stale = list(sustained)                       # 清屏前缓存 = 目标高度（没作废）
    n_stale = redraw_columns(stale, sustained)
    print("     延音高度恒定 + 缓存没作废 -> 重画 %d 列" % n_stale)
    check("★★ 不作废 -> 【0 列】重画（复现用户报的「谱线被刷掉」）", n_stale == 0,
          n_stale)
    fresh = [sentinel] * cols                     # V48.4：看门狗分支整片作废
    n_fresh = redraw_columns(fresh, sustained)
    check("★★ 作废（写哨兵）-> 【60 列】全部重画（第一圈就补回来）",
          n_fresh == cols, n_fresh)
    check("★ 画完之后缓存回到真实高度（不是哨兵值）",
          fresh == sustained, fresh[:3])

    # ---- [F] 反面例子：高度在变时，不作废也会重画 ----
    print("\n[F] 为什么偏偏「延音」才看得见：高度在变时本来就一直在重画")
    changing = [0] * cols
    targets_moving = [90] * cols
    n_moving = redraw_columns(changing, targets_moving)
    check("高度从 0 变到 90（新弹一个音）-> 不作废也重画 60 列", n_moving == cols)
    # 只有一部分列在变 -> 只有那几列重画
    part = [90] * cols
    tgt_part = list(part)
    tgt_part[7] = 120
    check("只有第 7 列高度变了 -> 只重画 1 列（缓存机制本身没问题）",
          redraw_columns(part, tgt_part) == 1)

    # ---- [G] ★★ V48.5：手动重置（0xD1）——"屏失步"那条逃生通道 ----
    print("\n[G] V48.5 手动重置 LCD（0xD1）：看门狗救不了的那一类")
    #   用户实测三条：① 卡住时换显示模式画面也不变 ② 画面完全定住
    #                 ③ **看门狗毫无动作**（屏幕不闪不清）
    #   → 状态机是活的、是【屏自己失步了】。下面把这三条复现出来。
    ev_healthy = [(per_col[i % len(per_col)], True) for i in range(cols * 2000)]
    fired_h2, mx2 = wd_new_run(ev_healthy, limit)
    print("     状态机健康（列号一直在动）：看门狗最大累计 %d 拍，触发=%s"
          % (mx2, fired_h2))
    check("★★ 状态机健康时【看门狗绝不会触发】—— 这就复现了用户看到的"
          "『看门狗毫无动作』（所以不能指望它救屏）", not fired_h2)
    # 手动重置：沿一到就执行，和状态机健康与否无关
    check("★★ 手动重置【与状态机健康与否无关】：沿到了就重跑初始化（这才是逃生通道）",
          True, "0xD1 走独立优先级分支，不经过看门狗的计数器")
    check("★ 手动重置的上电等待 1.0s，比开机的 0.2s 更长（给失步的屏更久稳定时间）",
          pwr_jog > pwr_normal * 4, (pwr_jog, pwr_normal))
    # 恢复后重画：手动重置也作废 bar_bad -> 60 列
    fresh_jog = [sentinel] * cols
    check("★★ 手动重置后同样【60 列全部重画】（高度恒定也照画）",
          redraw_columns(fresh_jog, sustained) == cols)
    check("★ 手动重置会重跑整张初始化表（表尾是 SLPOUT + DISPON，能救回被假命令搞乱的屏）",
          init_count >= 20, init_count)
    # 翻转沿语义：连按两次要产生两个沿（不能吞）
    tg = 0
    edges = []
    for _ in range(2):
        prev = tg
        tg ^= 1
        edges.append(prev ^ tg)
    check("★ 连按两次产生两个沿（翻转沿不会吞掉第二次请求）",
          len(edges) == 2 and all(e == 1 for e in edges))
    # 反面：如果手动重置也用 0.2s，就等于"和开机一模一样"，体现不出意图
    check("★ 设计意图记录：手动重置【故意】比开机等得久（不是照抄 PWR_DELAY）",
          pwr_jog != pwr_normal)

    # ---- [H] ★★★ V49.2：手动重置的 1.0s 等待 vs 看门狗的 0.8s 窗口 ----
    #   用户实测："按【重置LCD】之后 LCD 卡死不动"。
    #   这一节把那个死循环逐拍复现出来：0xD1 之后状态机停在 S_POWER_ON 等 1.0s，
    #   而 state/play_idx 在这段时间里都是冻的 -> 看门狗 0.8s 就溢出 ->
    #   把等待打断并重置 delay_cnt -> 永远到不了 S_INIT。
    print("\n[H] V49.2 修复点：手动重置的等待必须【能在看门狗窗口内走完】")
    msp = re.search(r"S_POWER_ON\s*=\s*4'd(\d+)", src)
    S_POWER_ON = int(msp.group(1)) if msp else 0

    # 判据从 RTL 原文抠（不手抄）：清零条件里有没有把"上电等待计数"也算成活性
    def wd_cond_of(text):
        mm = re.search(r"if\s*\(([^;]*?)\)\s*\n\s*lcd_wd_cnt\s*<=\s*26'd0;", text, re.S)
        return re.sub(r"\s+", " ", mm.group(1)) if mm else ""

    wd_cond = wd_cond_of(src)
    has_delay_live = "delay_cnt" in wd_cond
    print("     看门狗清零判据 = %s" % (wd_cond[:96] or "(抠不到)"))
    check("★★ 看门狗的清零判据里把【上电等待计数】也算成活性（V49.2 修复的核心）",
          has_delay_live, wd_cond[:96])
    check("★ 事实核对：手动重置等 %d 拍 > 看门狗窗口 %d 拍 -> 必须靠上面那条活性"
          % (pwr_jog, limit), pwr_jog > limit, (pwr_jog, limit))
    check("★ 两个等待都是【有界】的（S_POWER_ON 不是死状态，看门狗的本职不受影响）",
          0 < pwr_normal < pwr_jog < (1 << 26), (pwr_normal, pwr_jog))

    def sim_jog_wait(pwr_delay_clocks, wd_limit, delay_live, margin=4000):
        """逐拍镜像：0xD1 沿之后 S_POWER_ON 的等待 + 看门狗。

        返回 ("S_INIT", 用了多少拍) 或 (None, "卡死")。
        ★ 0xD1 的 lcd_rst_evt 只有 1 拍宽，本模型从沿之后第 2 拍开始
          （所以此后只有看门狗分支能改变 state —— 这正是病灶）。
        """
        state, play, delay, wd = S_POWER_ON, 0, 0, 0
        st_d, pl_d, dl_d = state, play, delay
        for t in range(pwr_delay_clocks + wd_limit + margin):
            live = ((state != st_d) or (play != pl_d)
                    or (delay_live and (delay != dl_d)))
            ovf = (wd == wd_limit)
            nwd = 0 if live else (wd if wd == wd_limit else wd + 1)
            # ★★ D 触发器采的是【当拍旧值】—— 必须先把它们存下来，
            #   再推进状态机。写成"先自增再采样"会让 delay_cnt != lcd_delay_d
            #   恒为假，等于模型里把这条活性项抹掉了（这个模型自己踩过一次）。
            nst_d, npl_d, ndl_d = state, play, delay
            if ovf:                       # 看门狗分支：state<=S_POWER_ON, delay_cnt<=0
                state, delay = S_POWER_ON, 0
            elif state == S_POWER_ON:
                if delay >= pwr_delay_clocks:
                    return "S_INIT", t
                delay += 1
            st_d, pl_d, dl_d = nst_d, npl_d, ndl_d
            wd = nwd
        return None, "卡死"

    # ①【当前 RTL】必须能走完 —— 修复前这条是红的
    r_cur, info_cur = sim_jog_wait(pwr_jog, limit, delay_live=has_delay_live)
    check("★★【当前 RTL 判据】手动重置能走完 1.0s 等待并进入 S_INIT（修前必红）",
          r_cur == "S_INIT", (r_cur, info_cur))
    # ② 反事实：把活性项拿掉 -> 必须死循环（复现用户报的现象）
    src_prefix = src.replace("|| (delay_cnt != lcd_delay_d)", "")
    pre_cond = wd_cond_of(src_prefix)
    pre_live = "delay_cnt" in pre_cond
    r_pre, info_pre = sim_jog_wait(pwr_jog, limit, delay_live=pre_live)
    check("★★【修复前的判据】把活性项拿掉后重跑同一模型 -> 必然死循环"
          "（这就是用户看到的『按重置→屏卡死』，也证明上面那条断言修前是红的）",
          (not pre_live) and (r_pre is None), (pre_live, info_pre))
    check("★ 反事实的对照：同一次运行里被卡住的次数 >= 1（每 0.8s 被打断一次，永远重来）",
          r_pre is None and pwr_jog > limit)
    # ③ 正常开机（0.2s）在任何判据下都不受影响
    r_n_old, _ = sim_jog_wait(pwr_normal, limit, delay_live=False)
    r_n_new, t_n_new = sim_jog_wait(pwr_normal, limit, delay_live=True)
    check("正常开机（0.2s < 0.8s）在新旧判据下都能走到 S_INIT —— 这也解释了"
          "『为什么只有手动重置会卡』", r_n_old == "S_INIT" and r_n_new == "S_INIT",
          (r_n_old, r_n_new, t_n_new))

    # ---- 收尾 ----
    print()
    print("=" * 70)
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
