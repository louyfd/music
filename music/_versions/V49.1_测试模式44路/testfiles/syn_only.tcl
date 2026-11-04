# =============================================================================
# syn_only.tcl —— 只跑【综合】，不做布局布线
#
# 用途：诊断综合阶段的问题（尤其是 Tech-Mapping 卡顿、以及数组有没有进 BSRAM）。
#   完整构建（build_chord.tcl 的 `run all`）要 20~30 分钟，综合只占前 2~4 分钟。
#   要对照"改了某处综合是不是变快了 / 数组有没有进 BSRAM"，用这个脚本迭代最快。
#
# 用法：
#   & "<Gowin>\IDE\bin\gw_sh.exe" "<工程根>\testfiles\syn_only.tcl"
# 判定：出现 SYN_OK —— 综合成功（后面的 RUN_ALL 才算完整验证）
#
# ★ 它【不产出 bitstream】，所以不能替代 build_chord.tcl，只用于诊断。
#
# ★★ V43.2 修正：原来这里写的是 run synthesis，gw_sh 会报
#      unknown process: synthesis
#    —— 也就是说这个脚本【从写下来就是坏的】，一直没人真正跑过。
#    本版改成先试 run syn，失败了再退回 run all（宁可贵一点也别报错停住）。
#    ★ 还没实测过 run syn 在这个 gw_sh 版本上是否合法，第一次跑请看输出。
# =============================================================================

set here [file normalize [file dirname [info script]]]
set root [file normalize [file join $here ..]]
set pj   [file join $root music.gprj]

puts "工程根目录: $root"
if {![file exists $pj]} {
    puts "OPEN_ERR: 找不到 $pj"
    exit 1
}

if {[catch {open_project $pj} msg]} {
    puts "OPEN_ERR: $msg"
    exit 1
}

set_option -top_module top
set_option -use_done_as_gpio  1
set_option -use_ready_as_gpio 1
set_option -use_cpu_as_gpio   1
set_option -output_base_name music

puts "=== RUN SYNTHESIS ONLY ==="
if {[catch {run syn} msg]} {
    puts "SYN_CMD_ERR: run syn 不认 —— 退回 run all"
    if {[catch {run all} msg2]} {
        puts "SYN_ERR: $msg2"
        exit 2
    }
}
puts "SYN_OK"
exit 0
