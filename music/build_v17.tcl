# V17 摇杆灵敏度修复：重编译脚本（路径已修正到实际安装的 Gowin）
set pj {D:/FPGA_60K/music/music.gprj}
puts "=== OPENING PROJECT ==="
if {[catch {open_project $pj} msg]} {
    puts "OPEN_ERR: $msg"
    exit 1
}
puts "OPEN_OK"
set_option -top_module top

# 当前 music.cst 把 led_glide/led 约束在 G11(DONE) / U12(READY)，
# 所以 DONE/READY 必须【释放成普通 IO】，否则 PR2017 报专用脚冲突。
#   （rebuild.tcl 里写的 0 对应的是另一版把 LED 挪到 W21/N14 的 cst，不适用这里）
set_option -use_done_as_gpio  1
set_option -use_ready_as_gpio 1
set_option -use_cpu_as_gpio   1

set_option -output_base_name music
puts "=== RUN ALL (syn + pnr) ==="
if {[catch {run all} msg]} {
    puts "RUN_ERR: $msg"
    exit 2
}
puts "RUN_OK"
exit 0
