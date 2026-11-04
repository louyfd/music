# =============================================================================
# build_chord.tcl —— 蓝牙和弦版重建脚本
#
# ★ 路径从【脚本自己的位置】推导，不写死绝对路径。
#   为什么：这个工程被挪过一次目录（music(2) -> music_player\music），
#   写死路径的脚本当时就全失效了，每次挪窝都得回来改一遍。
#   本脚本在 <工程根>\testfiles\ 下，所以工程根就是它的上一级。
#
# 用法：
#   & "<Gowin>\IDE\bin\gw_sh.exe" "<工程根>\testfiles\build_chord.tcl"
#
# 注意：music.cst 把 led=U12(=READY) / led_glide=G11(=DONE) 约束在专用脚上，
#       所以 DONE / READY 必须【显式释放成普通 IO】，否则 PnR 报
#       ERROR (PR2017) ... dedicated pin (DONE/READY)。
# =============================================================================

set here [file normalize [file dirname [info script]]]
set root [file normalize [file join $here ..]]
set pj   [file join $root music.gprj]

puts "工程根目录: $root"
if {![file exists $pj]} {
    puts "OPEN_ERR: 找不到 $pj"
    exit 1
}

puts "=== OPEN PROJECT ==="
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

set_option -use_done_as_gpio  1
set_option -use_ready_as_gpio 1
set_option -use_cpu_as_gpio   1
set_option -output_base_name music

puts "=== RUN ALL (synthesis + place&route) ==="
if {[catch {run all} msg]} {
    puts "RUN_ERR: $msg"
    exit 2
}
puts "RUN_OK"
exit 0
