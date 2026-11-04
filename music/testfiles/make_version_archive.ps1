# make_version_archive.ps1 —— 把一个版本完整归档到 _versions\<版本名>\
#
# 用法（在 music 工程根目录或任意位置）：
#   powershell -ExecutionPolicy Bypass -File make_version_archive.ps1 -Name "V24_演奏录音回传"
#
# 归档内容（和 V21~V23 的结构保持一致）：
#   music.gprj                     工程文件
#   src\                           15 个 RTL 源文件
#   bitstream\                     烧录文件 + 资源/时序报告
#   testfiles\                     测试脚本、谱子、说明
#   tools\                         协议文档
#   bluetooth\                     旧的蓝牙脚本
#   software\                      D:\CODE\SPP_txandrx 里的上位机程序
#   版本说明.txt                    由调用方先写好，本脚本不动它
#
# ★ 只在器件布局布线跑完之后调用 —— bitstream 是这一版最关键的产物。

param(
    [Parameter(Mandatory = $true)][string]$Name,
    # ★ 默认从脚本自己的位置推导（本脚本在 <工程根>\testfiles\ 下），
    #   不写死绝对路径 —— 工程挪过目录（music(2) -> music_player\music），
    #   写死的那次就失效了。
    [string]$Music = "",   # 留空，下面用 $MyInvocation 推导（$PSScriptRoot 在 param 默认值里取不到）
    [string]$Software = "D:\CODE\SPP_txandrx"
)

$ErrorActionPreference = "Stop"

# ★ 工程根 = 本脚本所在目录的上一级（本脚本在 <工程根>\testfiles\ 下）。
#   ★ 不能用 $PSScriptRoot 写在 param() 的默认值里 —— 这个环境上那里是空的，
#     会报 Join-Path 参数为空。放函数体里用 $MyInvocation.MyCommand.Path 取。
#   为什么非要推导：工程挪过一次目录（music(2) -> music_player\music），
#   当时写死绝对路径的脚本全失效了。
if (-not $Music) {
    $here = Split-Path -Parent $MyInvocation.MyCommand.Path
    $Music = (Resolve-Path (Join-Path $here "..")).Path
}

$dst = Join-Path $Music "_versions\$Name"
Write-Host "归档到: $dst"

# 先检查关键产物在不在，别归档一个半成品
$fs = Join-Path $Music "impl\pnr\music.fs"
if (-not (Test-Path $fs)) {
    Write-Error "找不到 $fs —— 布局布线还没跑完，别急着归档。"
    exit 1
}
$rpt = Join-Path $Music "impl\pnr\music.rpt.html"
if (-not (Test-Path $rpt)) {
    Write-Host "警告: 没有 music.rpt.html（资源/时序报告）"
}

New-Item -ItemType Directory -Force -Path $dst | Out-Null

function Copy-Tree($from, $to, $filter) {
    if (-not (Test-Path $from)) { Write-Host "  跳过（不存在）: $from"; return }
    New-Item -ItemType Directory -Force -Path $to | Out-Null
    Get-ChildItem $from -Recurse -File -Filter $filter -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -notmatch '\\__pycache__\\' } |
        ForEach-Object {
            $rel = $_.FullName.Substring($from.Length).TrimStart('\')
            $target = Join-Path $to $rel
            New-Item -ItemType Directory -Force -Path (Split-Path $target) | Out-Null
            Copy-Item $_.FullName $target -Force
        }
}

Write-Host "  src\ ..."
Copy-Tree (Join-Path $Music "src") (Join-Path $dst "src") "*"
Copy-Item (Join-Path $Music "music.gprj") $dst -Force

Write-Host "  bitstream\ ..."
$bd = Join-Path $dst "bitstream"
New-Item -ItemType Directory -Force -Path $bd | Out-Null
# ★★ V48.3：把音乐_tr_content.html 也带上 ★★
#   为什么非带不可：**Fmax / TNS / 违例端点数的汇总表只在这个文件里**，
#   别的报告（music.rpt.html/txt）只有资源表和引脚表。
#   上一版归档时没带它，于是"V48.2 时序全 0"这句话**事后无法复核** ——
#   而 V48.3 恰好就是在这些数字上翻过车（TNS 表显示全 0，实际有 1 个 setup
#   和 6 个 hold 违例端点，只有"Numbers of ... Violated Endpoints"和路径表里
#   才看得见）。所以时序汇总必须跟位流一起归档。
foreach ($f in "music.fs", "music.bin", "music.rpt.html", "music.rpt.txt",
               "music.timing_paths", "music.log",
               "music_tr_content.html", "music_tr_cata.html", "music.tr.html") {
    $src = Join-Path $Music "impl\pnr\$f"
    if (Test-Path $src) { Copy-Item $src (Join-Path $bd $f) -Force }
}

Write-Host "  testfiles\ tools\ bluetooth\ ..."
Copy-Tree (Join-Path $Music "testfiles")  (Join-Path $dst "testfiles")  "*"
Copy-Tree (Join-Path $Music "tools")      (Join-Path $dst "tools")      "*"
Copy-Tree (Join-Path $Music "bluetooth")  (Join-Path $dst "bluetooth")  "*.py"

Write-Host "  software\ ..."
Copy-Tree $Software (Join-Path $dst "software") "*.py"
# 上位机目录里的文档/说明也带上
Get-ChildItem $Software -File -Include *.md, *.txt -ErrorAction SilentlyContinue |
    ForEach-Object { Copy-Item $_.FullName (Join-Path $dst "software") -Force }
# 打包/安装脚本和图标也要带上，否则以后没法复现这一版的 exe
foreach ($pat in "*.ps1", "*.ico", "*.cmd", "*.bat") {
    Get-ChildItem $Software -File -Filter $pat -ErrorAction SilentlyContinue |
        ForEach-Object { Copy-Item $_.FullName (Join-Path $dst "software") -Force }
}

# 上位机的测试 MIDI（体积小，方便以后复现）
$midi = Join-Path $Software "MIDI"
if (Test-Path $midi) {
    New-Item -ItemType Directory -Force -Path (Join-Path $dst "software\MIDI") | Out-Null
    Get-ChildItem $midi -File -Filter "*.mid" -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -notlike "录音_*" } |
        ForEach-Object { Copy-Item $_.FullName (Join-Path $dst "software\MIDI") -Force }
}

$n = (Get-ChildItem $dst -Recurse -File | Measure-Object).Count
$sz = (Get-ChildItem $dst -Recurse -File | Measure-Object -Property Length -Sum).Sum
Write-Host ("完成: {0} 个文件, {1:N1} MB" -f $n, ($sz / 1MB))
if (-not (Test-Path (Join-Path $dst "版本说明.txt"))) {
    Write-Host "★ 还没写 版本说明.txt —— 记得补上。"
}
