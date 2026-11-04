# ★★★ 2026-10-09 修正 ★★★
#   原来这里写的是 .../Documents/60k/music/music.gprj —— 【那是另一个工程】！
#   Documents/60k/ 下面同时存在两个目录：
#       ～/60k/music/                  <- 10/3 的旧工程，src/top.v 只有 161KB，已废弃
#       ～/60k/music_player/music/     <- 【本工程】，src/top.v 263KB
#   后果：用错路径构建会「RUN_OK 一切正常」，但生成的 music.fs 是那个旧工程的，
#         而且本目录下 impl/pnr/ 的文件【时间戳根本不会变】—— 非常隐蔽。
#   判据：构建结束后本目录 impl/pnr/music.fs 的时间戳必须变新。
set pj {D:/CODE/FPGA_Gowin/Gowin/Gowin_V1.9.11.03_Education_x64/IDE/bin/Documents/60k/music_player/music/music.gprj}
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
# 释放双用途配置脚为普通 IO：
#   G11 = DONE, U12 = READY —— Console 60K 把这两个脚当板载 LED 用，
#   不释放就会报 ERROR (PR2017) dedicated pin (DONE/READY)。
#   CPU 那组原来就已经释放过（P19 等）。
foreach opt {-use_cpu_as_gpio -use_done_as_gpio -use_ready_as_gpio} {
    if {[catch {set_option $opt 1} msg]} {
        puts "SETOPT_WARN $opt : $msg"
    } else {
        puts "SETOPT_OK $opt = 1"
    }
}
puts "=== RUN ALL (syn + pnr) ==="
if {[catch {run all} msg]} {
    puts "RUN_ERR: $msg"
    exit 2
}
puts "RUN_OK"
puts "=== DONE ==="
exit 0
