$ErrorActionPreference = 'Stop'
$path = 'D:\FPGA_60K\music\src\top.v'
$enc  = New-Object System.Text.UTF8Encoding($false)

if (-not (Test-Path "$path.nosw.bak")) { Copy-Item $path "$path.nosw.bak"; Write-Host "backup -> $path.nosw.bak" }

$L = [System.Collections.ArrayList]@($enc.GetString([System.IO.File]::ReadAllBytes($path)) -split "`n")
$log = New-Object System.Collections.ArrayList

function Chk([int]$idx, [string]$anchor) {
    if ($idx -lt 1 -or $idx -gt $script:L.Count) { throw "idx $idx out of range" }
    if ($script:L[$idx-1] -notlike "*$anchor*") { throw "anchor '$anchor' not found on line $idx : " + $script:L[$idx-1] }
}
# 用「替换」代替一段行；$start..$end 为 1-based 闭区间
function Repl([int]$start, [int]$end, [string[]]$newLines, [string]$tag) {
    $script:L.RemoveRange($start-1, $end-$start+1)
    $script:L.InsertRange($start-1, [string[]]$newLines)
    [void]$script:log.Add("ok  $tag")
}
function DelR([int]$start, [int]$end, [string]$tag) { Repl $start $end ([string[]]@()) $tag }

# ---- 从下往上做，行号不会被前面的改动带偏 ----

Chk 2081 'SW'
Repl 2081 2081 @('    //   ★ 按一下【按钮4】在 0/1 之间来回切。') '2081 音色切换按钮注释'

Chk 402 'SW'
Repl 402 402 @('    // ★ 音色只由【按钮4】短按循环决定（与 25K K44 一致）。') '402 音色注释'

Chk 211 '按钮5'; Chk 216 'key_sw'
Repl 211 216 @(
    '    //   ★ 按钮分配（16 键触摸模块）：',
    '    //       按钮1(bit0)=滑音   按钮2(bit1)=延音',
    '    //       按钮3(bit2)=波形   按钮4(bit3)=音色（实测无反应的那颗）',
    '    //       按钮5(bit4)=颤音  ← 本次新增'
) '211-216 按钮分配说明（去掉 SW/key_sw）'

Chk 122 '键3'; Chk 123 '键4~16'
Repl 122 123 @(
    '    //   键3 -> 波形 / 频谱显示切换',
    '    //   键4 -> 音色切换',
    '    //   键5 -> 颤音（Tremolo）开关',
    '    //   键6~16 暂时不用'
) '122-123 按钮功能表'

Chk 94 'SW'; Chk 103 'SW_ACTIVE_LOW'
DelR 94 103 '94-103 删除 SW_ACTIVE_LOW 及其说明（死参数，全工程无人引用）'

Chk 69 'SW'
DelR 69 69 '69 删除 "SW -> J5 -> ..." 说明'

Chk 65 'ADS1115'
Repl 65 65 @('    // 摇杆模块（ADS1115 双通道：X 轴 / Y 轴）') '65 摇杆模块标题'

Chk 27 'KEY0/KEY1/KEY2'; Chk 30 'music.cst'
DelR 27 30 '27-30 删除 key_sw / key_sustain 说明（.cst 里根本没有这两个端口）'

Chk 24 'LED'
Repl 24 24 @('    // ---- LED ----') '24 SW+LED 标题'

[System.IO.File]::WriteAllText($path, ($L -join "`n"), $enc)
$log | ForEach-Object { Write-Host $_ }
$t = $enc.GetString([System.IO.File]::ReadAllBytes($path))
Write-Host ("剩余 SW 引用： " + ([regex]::Matches($t, '\bSW\b|key_sw|key_sustain|SW_ACTIVE_LOW')).Count)
