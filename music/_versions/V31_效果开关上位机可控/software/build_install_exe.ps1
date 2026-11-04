# build_install_exe.ps1 —— 重打包上位机并装到位
#
# 桌面快捷方式指向的是【冻结后的 exe】，不是 .py：
#     D:\desktop\蓝牙串口助手.lnk  ->  D:\CODE\SPP_txandrx\SPP_Assistant\SPP_Assistant.exe
# 所以改完 spp_assistant.py 必须重打包，否则界面上什么都看不到（踩过好几次）。
#
# 用法： powershell -ExecutionPolicy Bypass -File build_install_exe.ps1
#
# 装两个：
#   SPP_Assistant\SPP_Assistant.exe        目录版（onedir，启动快，快捷方式用这个）
#   SPP_Assistant_onefile.exe              单文件版（方便拷给别人）
#
# ★ 编码：本文件必须存成【UTF-8 带 BOM】。
#   Windows PowerShell 5.1 读无 BOM 的脚本时按 ANSI(1252) 解码，
#   中文注释会变乱码，乱码里的引号会把字符串截断 → 整个脚本语法错误（踩过）。
#
# ★ 打包时程序不能开着，否则替换 exe 会失败（文件被占用）。

# 故意不用 Stop：PyInstaller 把 INFO 写到 stderr，
# 配上 Stop 会让 PowerShell 把普通日志当成终止性错误，脚本直接断在第一步（踩过）。
$ErrorActionPreference = "Continue"

$root = "D:\CODE\SPP_txandrx"
Set-Location $root

$icon  = Join-Path $root "spp.ico"
$entry = Join-Path $root "spp_assistant.py"
$inst  = Join-Path $root "SPP_Assistant"
$common = @(
    "--noconfirm", "--clean",
    "--name", "SPP_Assistant",
    # ★★ --windowed 绝对不能漏 ★★
    #   漏了就会编成【控制台子系统】的程序，双击后一直挂着一个黑色命令行窗口。
    #   我重写这个脚本（修 stderr 那个问题时）漏过一次，从 V25 到 V27 的包
    #   全是带黑窗口的 —— 所以下面加了 PE 子系统校验，漏了就报错停手。
    "--windowed",
    "--icon", $icon,                                  # ★ 必须绝对路径：
                                                      #   相对路径会相对 --specpath 解析（踩过）
    "--collect-submodules", "mido",
    "--hidden-import", "rtmidi",                      # 电脑上放 MIDI 要用
    "--hidden-import", "fpga_rec",                    # ★ 录音回传解析
    "--hidden-import", "score_jp",
    "--hidden-import", "fpga_midi",
    "--hidden-import", "fpga_proto"
)

# 读 PE 头的 Subsystem：2 = GUI（没有控制台窗口），3 = 控制台（会有黑窗口）
function Get-PESubsystem($path) {
    try {
        $fs = [System.IO.File]::OpenRead($path)
        try {
            $br = New-Object System.IO.BinaryReader($fs)
            $fs.Position = 0x3C
            $peOff = $br.ReadInt32()
            $fs.Position = $peOff
            if ($br.ReadUInt32() -ne 0x00004550) { return -1 }   # "PE\0\0"
            $fs.Position = $peOff + 24 + 68                       # OptionalHeader.Subsystem
            return $br.ReadUInt16()
        } finally { $fs.Close() }
    } catch { return -1 }
}

# ---- 0) 关掉正在运行的程序 ----
# ★ 只关【确实是这个安装目录里那个 exe】的进程：
#   按进程名盲杀很危险（本机踩过用模式匹配把自己杀掉的事故）。
#
# ★ 为什么要等它真的退出：Stop-Process 返回不等于进程没了、更不等于
#   文件句柄放了。不等就 Remove-Item，会拿到 IOException，
#   而 $ErrorActionPreference 是 Continue —— 脚本会继续往下 Copy-Item，
#   最后留下一个【旧 exe + 新 _internal】的混合安装目录（踩过）。
$running = @(Get-CimInstance Win32_Process -Filter "Name='SPP_Assistant.exe'" -ErrorAction SilentlyContinue |
             Where-Object { $_.ExecutablePath -and
                            $_.ExecutablePath.StartsWith($inst, [StringComparison]::OrdinalIgnoreCase) })
if ($running.Count -gt 0) {
    Write-Host "先关掉正在运行的程序（已核对路径）:"
    $running | ForEach-Object { Write-Host ("  PID={0}  {1}" -f $_.ProcessId, $_.ExecutablePath) }
    foreach ($p in $running) {
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
    $t0 = Get-Date
    while ($true) {
        $left = @(Get-CimInstance Win32_Process -Filter "Name='SPP_Assistant.exe'" -ErrorAction SilentlyContinue |
                  Where-Object { $_.ExecutablePath -and
                                 $_.ExecutablePath.StartsWith($inst, [StringComparison]::OrdinalIgnoreCase) })
        if ($left.Count -eq 0) { break }
        if (((Get-Date) - $t0).TotalSeconds -gt 15) {
            Write-Host "★ 程序关不掉（还活着）:"
            $left | ForEach-Object { Write-Host ("   PID={0}" -f $_.ProcessId) }
            Write-Host "  请手动关掉它再跑一次这个脚本 —— 现在停手，不动安装目录。"
            exit 1
        }
        Start-Sleep -Milliseconds 300
    }
    Write-Host "  已退出。"
    Start-Sleep -Milliseconds 500          # 再给系统一点时间放文件句柄
} else {
    Write-Host "程序没在运行，直接打包。"
}

function Invoke-PyInstaller($extra, $label) {
    Write-Host $label
    $log = & py -3.9 -m PyInstaller @common @extra $entry 2>&1 | Out-String
    if ($log -notmatch "Build complete") {
        Write-Host "  打包失败，末尾输出："
        ($log -split "`r?`n" | Select-Object -Last 15) | ForEach-Object { Write-Host "    $_" }
        return $false
    }
    Write-Host "  成功"
    return $true
}

# ---- 1) 目录版 ----
$null = Invoke-PyInstaller @("--onedir", "--distpath", "_out_onedir",
                             "--workpath", "_build_onedir",
                             "--specpath", "_build_onedir") "[1/4] 打包目录版..."

# ---- 2) 单文件版 ----
$null = Invoke-PyInstaller @("--onefile", "--distpath", "_out_onefile",
                             "--workpath", "_build_onefile",
                             "--specpath", "_build_onefile") "[2/4] 打包单文件版..."

# ---- 3) 装目录版（保留用户的 spp_config.json）----
Write-Host "[3/4] 安装目录版..."
$newExe = Join-Path $root "_out_onedir\SPP_Assistant\SPP_Assistant.exe"
if (-not (Test-Path $newExe)) {
    Write-Host "★ 目录版没打出来，停手 —— 绝不把能用的安装覆盖成空目录。"
    exit 1
}
$cfg = Join-Path $inst "spp_config.json"
$cfgBak = $null
if (Test-Path $cfg) { $cfgBak = Get-Content $cfg -Raw }

# ★★ 删除必须【删干净了才往下走】★★
#   删不掉还继续 Copy-Item 的话，会得到"旧 exe + 新 _internal"的混合目录，
#   从外面完全看不出来（踩过：exe 没换成 GUI 的，黑窗口还在）。
$removed = $false
for ($try = 1; $try -le 5; $try++) {
    if (-not (Test-Path $inst)) { $removed = $true; break }
    try {
        Remove-Item $inst -Recurse -Force -ErrorAction Stop
        $removed = $true
        break
    } catch {
        Write-Host ("  第 {0} 次删除失败：{1}" -f $try, $_.Exception.Message)
        Start-Sleep -Milliseconds 800
    }
}
if (-not $removed) {
    Write-Host "★ 旧安装目录删不掉（八成程序还开着，或者有别的程序占着里面的文件）。"
    Write-Host "  现在【停手，不动安装目录】—— 请关掉程序后重跑本脚本。"
    Write-Host "  新版本已经打好在 _out_onedir\ 里，不会被浪费。"
    exit 1
}
Copy-Item (Join-Path $root "_out_onedir\SPP_Assistant") $inst -Recurse -Force
if ($cfgBak) { Set-Content -Path (Join-Path $inst "spp_config.json") -Value $cfgBak -NoNewline }

# ---- 4) 装单文件版 ----
Write-Host "[4/4] 安装单文件版..."
$one = Join-Path $root "_out_onefile\SPP_Assistant.exe"
if (Test-Path $one) {
    Copy-Item $one (Join-Path $root "SPP_Assistant_onefile.exe") -Force
} else {
    Write-Host "★ 单文件版没打出来，保持原样。"
}

Write-Host ""
Write-Host "已安装："
foreach ($p in @((Join-Path $inst "SPP_Assistant.exe"),
                 (Join-Path $root "SPP_Assistant_onefile.exe"))) {
    if (Test-Path $p) {
        $f = Get-Item $p
        Write-Host ("  {0,12:N0} bytes  {1:HH:mm:ss}  {2}" -f $f.Length, $f.LastWriteTime, $f.FullName)
    }
}
# 最后一次自检：exe 大小像不像话（307KB 那种是坏掉的 stub，踩过）
$f = Get-Item (Join-Path $inst "SPP_Assistant.exe") -ErrorAction SilentlyContinue
if (-not $f -or $f.Length -lt 500000) {
    Write-Host "★ 目录版 exe 大小不对（<500KB），很可能是坏的 stub，别用。"
    exit 1
}

# ★★ PE 子系统必须是 GUI(2)：是控制台(3) 的话双击后会挂一个黑色命令行窗口 ★★
#    这一条就是防"--windowed 漏写"的，漏了直接报错停手。
$bad = @()
foreach ($p in @((Join-Path $inst "SPP_Assistant.exe"),
                 (Join-Path $root "SPP_Assistant_onefile.exe"))) {
    if (-not (Test-Path $p)) { continue }
    $sub = Get-PESubsystem $p
    $tag = if ($sub -eq 2) { "GUI，无控制台窗口 ✅" }
           elseif ($sub -eq 3) { "控制台，会有黑窗口 ❌" }
           else { "读不出来（$sub）" }
    Write-Host ("  子系统 {0}  {1}  {2}" -f $sub, (Split-Path $p -Leaf), $tag)
    if ($sub -ne 2) { $bad += (Split-Path $p -Leaf) }
}
if ($bad.Count -gt 0) {
    Write-Host ("★ 这些 exe 不是 GUI 子系统，双击会挂黑窗口: " + ($bad -join ", "))
    Write-Host "  检查 --windowed 是不是漏了。"
    exit 1
}
Write-Host "完成。"
