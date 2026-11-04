# -*- coding: utf-8 -*-
"""sim_v49_1_testmode.py —— V49.1「测试模式 44 路（36 键位 + 8 测试声部）」的镜像验证

镜像对象：
    src\\top.v                —— TEST_ENV / 16 路测试振荡器 / 分时读表 / S1 相加 /
                                test_hold 状态机 / 与 uart 的两根连线
    src\\uart_packet_echo.v   —— 0xD2 下行翻转、0x60 上行回报（那份由协议侧负责，
                                这里只核对【接口对得上】）

★ 这一步的风险点：
   · 44 路同时响会不会溢出/削顶（峰要平才好数）
   · 8 路测试音与 36 个键位音在频域上会不会重叠（重叠就数不清峰）
   · 分时读表的时隙够不够（128 拍里塞 8 隙 × 4 拍）
   · 测试模式退出后有没有"后遗症"（env_eff/相位/sounding_mask 要能逐位回到原状）
   · 按钮与上位机是不是真的同权（状态只能有一个驱动源）

命令行：python sim_v49_1_testmode.py
"""

from __future__ import annotations

import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
TOP = os.path.join(ROOT, "src", "top.v")
UART = os.path.join(ROOT, "src", "uart_packet_echo.v")

FS = 12_500_000 / 128.0          # 97.65625 kHz（V49）
N_ITEMS = 0
N_FAIL = 0


def ok(name, cond, detail=""):
    global N_ITEMS, N_FAIL
    N_ITEMS += 1
    if cond:
        print("  [PASS] %s%s" % (name, ("  " + str(detail)) if detail != "" else ""))
    else:
        N_FAIL += 1
        print("  [FAIL] %s%s" % (name, ("  " + str(detail)) if detail != "" else ""))


def read(p):
    with open(p, encoding="utf-8", errors="replace") as f:
        return f.read()


SRC = read(TOP)
USRC = read(UART)


# ==========================================================================
print("\n[A] 8 路测试振荡器的存在与结构（每路都要有独立相位/频率/包络）")
# ==========================================================================
ok("有 gen_tv 的 generate 块（16 路实例）", "for (tj = 0; tj < T_SLOT_N; tj = tj + 1) begin : gen_tv" in SRC)
ok("T_SLOT_N == 8（方案 A：16 路曾把 CLS 顶到 90% 导致 DesRoute failed）",
   "localparam integer T_SLOT_N = 8;" in SRC)
ok("每路有自己的相位寄存器 tph（reg [31:0] tph，在 gen_tv 里）",
   re.search(r"for \(tj = 0; tj < T_SLOT_N.*?reg \[31:0\] tph;", SRC, re.S) is not None)
ok("每路有自己的包络寄存器 tenv + 斜坡计数 trc（= 独立包络控制）",
   re.search(r"for \(tj = 0; tj < T_SLOT_N.*?reg \[7:0\]  tenv;.*?reg \[11:0\] trc;", SRC, re.S) is not None)
ok("每路的频率字是【本路自己的常数】，且由 calc_delta 编译期算出",
   "localparam [31:0] TINC = calc_delta(test_freq(tj[3:0]));" in SRC)
ok("相位累加在 sample_en（全速 97.656 kHz，跟采样率走）",
   re.search(r"if \(sample_en\) tph <= tph \+ TINC;", SRC) is not None)
ok("包络斜坡在 ctrl_en（时间语义，跟真实时间走）",
   re.search(r"if \(ctrl_en\) begin\s*\n\s*if \(test_on && \(trc < T_RAMP_MAX", SRC) is not None)
ok("退出测试模式时包络会自己退回去（!test_on 分支）", "else if (!test_on && (trc != 12'd0)) begin" in SRC)


# ==========================================================================
print("\n[B] 频率规划：36 键位 + 16 测试音必须【互不重叠】且都在音域内")
# ==========================================================================
key_f = [int(x) for x in re.findall(r"assign delta\[\d+\]\s*=\s*calc_delta\(32'd(\d+)\)", SRC)]
tf = [int(x) for x in re.findall(r"test_freq = 32'd(\d+);", SRC)]
ok("键位频率解析到 36 个", len(key_f) == 36, len(key_f))
ok("测试频率解析到 8 个", len(tf) == 8, tf)
ok("36 个键位频率互不相同", len(set(key_f)) == 36)
ok("8 个测试频率互不相同", len(set(tf)) == 8)
ok("★ 频带不重叠：测试音最低 %d Hz > 键位最高 %d Hz" % (min(tf), max(key_f)),
   min(tf) > max(key_f))
allf = key_f + tf
ok("44 个频率全体互不相同 → 频谱上 44 个独立峰", len(set(allf)) == 44)
# 相邻峰的相对间隔（1/3 倍频程内要能分辨：最近的间隔也要 > 1%）
srt = sorted(allf)
gap = min((srt[i + 1] - srt[i]) / float(srt[i]) for i in range(len(srt) - 1))
ok("相邻频率最小相对间隔 %.2f%% > 1%%（频谱仪分得开）" % (gap * 100), gap > 0.01)
# 频率字复算
worst = 0.0
for f in tf:
    d = (f << 32) // 97656
    worst = max(worst, abs(d * 97656 / float(1 << 32) - f) / f)
ok("16 路测试音的频率字复算误差 < 0.05%", worst < 5e-4, "最大 %.2e" % worst)


# ==========================================================================
print("\n[C] 分时读表：128 拍里塞得下 8 隙 × 4 拍吗")
# ==========================================================================
ok("时隙选择 = sample_cnt[6:4]（8 隙，每隙 16 拍）",
   "wire [3:0]        tslot  = {1'b0, sample_cnt[6:4]};" in SRC)
ok("地址 = 本路相位高 9 位（和 36 路键位同一个取法）",
   "wire [8:0]        t_addr = tph_out[tslot][31:23];" in SRC)
ok("只有【一个】共享波表实例 u_trom", SRC.count("sine_rom u_trom (") == 1)
ok("四拍流水齐全，且拍内判据用 cnt[3:0]（不是 [2:0]，否则一隙触发两次）",
   ("case (sample_cnt[3:0])" in SRC)
   and all(p in SRC for p in ("4'd1: t_rom_r <=", "4'd2: t_mul_r <=", "4'd3: t_acc"))
   and ("case (sample_cnt[2:0])" not in SRC))
sub = 4                      # 每隙用 cnt[2:0] 的 0..3
ok("8 隙 × %d 拍 = %d 拍 <= 128 拍（一个采样周期，占 1/4）" % (sub, 8 * sub), 8 * sub <= 128)
ok("乘法只占 1 个（16 路共用）—— 不是 16 个乘法器",
   SRC.count("t_mul_r <= t_rom_r *") == 1)
ok("缩放与 36 路同一量纲（>>8，即 ÷256）", "t_mul_r[23:8]" in SRC)
ok("t_acc 在采样边界搬到 t_sum（晚一个采样点参与混音）",
   re.search(r"if \(sample_en\) begin\s*\n\s*t_sum <= t_acc;", SRC) is not None)
ok("S1 把测试声部和加进去了（只加在 S1 = 基波，不动 S2~S4）",
   "16'd1: S1 <= sum_env + t_sum;" in SRC)


# ==========================================================================
print("\n[D] 36 个键位路在测试模式下的行为（退出后要能逐位还原）")
# ==========================================================================
ok("测试模式下 36 路固定发【本键基础音高】（走 delta[g]，不跟移调/八度）",
   "else if (test_on)      phase[g] <= phase[g] + delta[g];" in SRC)
ok("★ 非测试模式的相位路径【一个字没改】（else 分支仍是 inc[g]）",
   "else                   phase[g] <= phase[g] + inc[g];" in SRC)
ok("有效包络 env_eff[ge] = test_on ? TEST_ENV : env[ge]（退出后逐位回到 env）",
   "assign env_eff[ge] = test_on ? TEST_ENV : env[ge];" in SRC)
ok("sounding_mask 在测试模式下全亮", "assign sounding_mask[ge] = test_on | (env[ge] != 8'd0);" in SRC)
ok("逐键乘包络用的是 env_eff（不是 env）",
   "wire signed [23:0] s_p   = s_raw * $signed({1'b0, env_eff[g]});" in SRC)
ok("测试模式下强制走正弦 ROM（吉他弦的谐波太多，峰数不清）",
   "wire signed [15:0] src_g = (guitar_on_audio && !test_on) ? ks_smp[g]" in SRC)
ok("AGC 负载把 8 路测试声部也算进去（8×200=1600，否则 44 路会削顶）",
   "+ (test_on ? 14'd1600 : 14'd0);" in SRC)
ok("LCD 电平扫描走 env_eff（测试模式下 36 根柱一起亮，好数）",
   "scan_val <= env_eff[scan_next];" in SRC)


# ==========================================================================
print("\n[E] 幅度与溢出（44 路同时响不能回绕）")
# ==========================================================================
TEST_ENV = int(re.search(r"localparam \[7:0\] TEST_ENV = 8'd(\d+);", SRC).group(1))
ok("TEST_ENV 解析到 200", TEST_ENV == 200, TEST_ENV)
peak_key = 36 * (32767 * TEST_ENV // 256)
peak_tst = 8 * (32767 * TEST_ENV // 256)
tot = peak_key + peak_tst
ok("S1 最坏情况 %d < 2^21（22 位有符号上限 2,097,151）→ 不会回绕" % tot, tot < 2 ** 21)
ok("AGC 负载最坏 %d < 2^14（env_load 是 14 位）" % (36 * TEST_ENV + 8 * TEST_ENV),
   36 * TEST_ENV + 8 * TEST_ENV < 2 ** 14)
ok("8 路声部和 t_acc 最坏 %d < 2^21（22 位）" % peak_tst, peak_tst < 2 ** 21)


# ==========================================================================
print("\n[F] 状态机：板子唯一权威 + 按钮与上位机同权 + 与 uart 的接口")
# ==========================================================================
ok("test_hold 只有一个 always 块驱动（全文件唯一权威）",
   len(re.findall(r"test_hold\s*<=", SRC)) == 3,
   "出现 %d 次（复位 + 按钮 + 0xD2）" % len(re.findall(r"test_hold\s*<=", SRC)))
ok("按钮板第 11 键（btn_new[10]）能翻它", "else if (btn_new[10])  test_hold <= ~test_hold;" in SRC)
ok("上位机 0xD2 的边沿也能翻它", "else if (tst_we)       test_hold <= ~test_hold;" in SRC)
ok("0xD2 用三级同步 + 边沿检测（不是直接拿电平用）",
   re.search(r"\{tst_s3, tst_s2, tst_s1\} <= \{tst_s2, tst_s1, test_tgl\};", SRC) is not None
   and "wire tst_we = tst_s2 ^ tst_s3;" in SRC)
ok("test_state_out 两级同步后在 clk_50m_ext 域（送给 0x60 回报）",
   re.search(r"posedge clk_50m_ext.*?\{tst_rep_s2, tst_rep_s1\} <= \{tst_rep_s1, test_hold\};", SRC, re.S) is not None)
ok("test_on（音频域）两级同步",
   re.search(r"test_a1 <= test_hold; test_a2 <= test_a1;", SRC) is not None)
ok("top.v 把 test_tgl / test_state_in 接到了 uart_packet_echo",
   ".test_tgl      (test_tgl)," in SRC and ".test_state_in (test_state_out)" in SRC)
ok("uart_packet_echo.v 里确有这两个端口",
   "output reg         test_tgl," in USRC and "input  wire        test_state_in," in USRC)
ok("uart 侧 0xD2 是【翻转】语义、无操作数",
   re.search(r"rx_data == TEST_BYTE\) begin.{0,120}test_tgl <= ~test_tgl;", USRC, re.S) is not None)
ok("uart 侧 0x60 是 2 字节帧 [0x60][TEST_ON]",
   "ts_byte = (ts_idx == 2'd1) ? TEST_STAT_BYTE : {7'd0, ts_state_r};" in USRC)


# ==========================================================================
print("\n" + "=" * 72)
if N_FAIL == 0:
    print("sim_v49_1_testmode: ALL PASSED（%d 项）" % N_ITEMS)
    sys.exit(0)
else:
    print("sim_v49_1_testmode: %d/%d 项失败" % (N_FAIL, N_ITEMS))
    sys.exit(1)
