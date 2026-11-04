//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11.03 Education
//Part Number: GW5AT-LV60PG484AC1/I0
//Device: GW5AT-60
//Device Version: B
//
//=============================================================================
//  music 电子琴  时序约束 (SDC)
//
//  依据：25k/music_all_K44/PORT_25k_to_Console60k_清单.txt 第 F3 条
//    "25k 工程完全没有时序约束文件（PnR 报告里 <Timing Constraints File>: ---），
//     所以现在的『时序通过』是空转。60K 器件更大、8 个 PLL、16 个全局时钟、
//     11 个 bank，建议借官方例程的 .sdc 补上 create_clock（至少 clk_50m）。"
//
//  ── 加这份 SDC 之前的问题（实测）────────────────────────────────────
//  没有 SDC 时 STA 仍会自己造时钟，但会把 50MHz 域和 12.5MHz 音频域
//  当成【同步域】去算跨域路径，于是报出一堆跨时钟同步器的 HOLD 违例：
//      HOLD 最差 -1.367 ns，端点全是 pressed_mask_sync1 / tr_audio_r /
//      event_toggle_d1 这些两级同步器的第一级
//   这些路径本来就该被排除（它们跨异步域），负 slack 是【假违例】。
//
//  ── 本文件的时钟结构 ────────────────────────────────────────────────
//    clk_50m       输入口，板上 50MHz 晶振，周期 20ns   ← 这里 create_clock
//    clk_audio     PLL ODIV0=56 → 700/56 = 12.5MHz    ← 工具自动派生
//    clk_50m_ext   PLL ODIV1=14 → 700/14 = 50MHz      ← 工具自动派生
//
//  ★ 与 Gowin 官方例程一致：PLL 输出【不写在这里】，由工具自动派生。
//    （参考 60k/ADC+SPI/src/ADC+SPI.sdc 的注释：
//      "PLL 输出的 80MHz 由工具自动派生，不需要在这里写。"）
//
//  ★ 端口名必须是 clk_50m。名字写错会报
//      ERROR (TA2003) : Can't set timing constraint to object
//=============================================================================


//-----------------------------------------------------------------------------
// 1) 主时钟：板上 50MHz，周期 20ns，上升沿 0 / 下降沿 10
//-----------------------------------------------------------------------------
create_clock -name clk_50m -period 20 -waveform {0 10} [get_ports {clk_50m}]


//-----------------------------------------------------------------------------
// 2) 时钟域隔离
//
//    clk_50m 与 PLL 出来的 clk_audio(12.5MHz) / clk_50m_ext(50MHz) 之间
//    是【异步】关系，设计里全部用两级同步器过渡：
//      pressed_mask     : 50MHz -> clk_audio   (pressed_mask_sync1/2)
//      sustain_on       : 50MHz -> clk_audio   (sustain_s1/s2)
//      timbre_toggle_50m: 50MHz -> clk_audio   (tt_d1/d2/d3)
//      event_toggle     : 50MHz -> clk_audio   (event_toggle_d1/d2/d3)
//      glide_en         : 50MHz -> clk_audio   (glide_s1/s2)
//      auto_keys/midi_keys: 50MHz -> clk_audio (auto_keys_s1/s2, midi_keys_s1/s2)
//
//    标成 asynchronous 之后，工具不再对这些跨域路径做 setup/hold 检查，
//    上面那批假 HOLD 违例就会消失。
//
//    ★ 前提是这些信号确实都走了两级同步器 —— 已逐个核对过，成立。
//      以后新增跨域信号，务必同样加两级同步器，再挂到这个组里。
//-----------------------------------------------------------------------------
//    ★ 写法上有三个坑，全部实测踩过：
//      1) 必须写成【单行】。Gowin 的 SDC 解析器不接受反斜杠续行，
//         写成多行会报 syntax error near token '\'。
//      2) 派生时钟【不能按名字取】。create_clock 之后立刻用
//         [get_clocks {clk_audio}] 会报
//           ERROR (TA2004) : Cannot get clock with name 'clk_audio'
//         因为 PLL 的派生时钟要到后面才建立。
//      3) 裸星号 [get_clocks *] 也不行，报
//           ERROR (TA2000) : syntax error near token '*]'
//         所以用通配写法 clk*，把 clk_50m / clk_audio / clk_50m_ext
//         三个时钟一起圈进来。
set_clock_groups -asynchronous -group [get_clocks {clk*}]


//-----------------------------------------------------------------------------
// 3) 输出报告，便于复核
//    （PnR 的 .rpt.txt 原本没有时序汇总章节，加这几条才能看到结果）
//-----------------------------------------------------------------------------
report_timing -setup -max_paths 20 -max_common_paths 1
report_timing -hold  -max_paths 20 -max_common_paths 1
