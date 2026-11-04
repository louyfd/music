# ★ 原路径 D:/CODE/FPGA_Gowin/... 在本机不存在，实际安装位置是：
#     D:\gaoyun\Gowin\Gowin_V1.9.11.03_Education_x64\IDE\bin\gw_sh.exe
set pj {D:/FPGA_60K/music/music.gprj}
puts "=== OPENING PROJECT ==="
if {[catch {open_project $pj} msg]} {
    puts "OPEN_ERR: $msg"
    exit 1
}
puts "OPEN_OK"
if {[catch {set_option -top_module top} msg]} {
    puts "SETOPT_ERR: $msg"
    exit 3
}
puts "SETOPT_OK: top_module = top"

#=============================================================================
# 双用途配置脚设置（依据 PORT_25k_to_Console60k_清单.txt 第 27-30 行）
#
# ★ Console 60K 的 READY / DONE / RECONFIG_N 在板上接了配置指示电路
#   （原理图 FPGA_CFG 页：Q7A/Q7B + RGB LED + RECFG_KEY）。
#   清单明确要求【保持专用脚 regular_io=false】。
#   G11 = DONE，U12 = READY。
#
#   ★ V17 更正：当前工作区的 src/music.cst 里 led_glide 就是 G11(DONE)、
#     led 就是 U12(READY)，必须【释放成普通 IO】，否则 PnR 直接报
#         ERROR (PR2017) 'led_glide' cannot be placed ... dedicated pin (DONE)
#         ERROR (PR2017) 'led'       cannot be placed ... dedicated pin (READY)
#     （下面原来写的"显式写 0"对应的是另一版把 LED 挪到 W21/N14 的 cst，
#       那份 cst 不在当前工作区，照着写 0 会编译失败。）
#     这两个开关存在 impl/music_process_config.json 里，会被沿用到下一次构建，
#     所以必须显式写清楚，不能靠"不写"。这里显式写 1。
#
# CPU 那组保持释放（Mega/基石板阶段为了引脚分配做的，对 Console 无害）。
#   IDE 里对应选项：Project -> Configuration -> Dual-Purpose Pin
#=============================================================================
set_option -use_done_as_gpio  1
set_option -use_ready_as_gpio 1
set_option -use_cpu_as_gpio   1

# 时序约束文件（清单 F3）：没有它，STA 会把 50MHz 域和 12.5MHz 音频域
# 当同步域算，报出一堆跨域同步器的假 HOLD 违例。
add_file src/music.sdc

set_option -output_base_name music
run all
