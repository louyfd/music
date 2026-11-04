# =============================================================================
#  build_with_cst.ps1 —— 选一份引脚约束，构建，并【自动核对构建真的生效了】
# =============================================================================
#  背景：本工程有两份并列的引脚约束，同一时刻只能启用一份：
#      src\music.cst          —— 调试版（默认，当前 enable="1"）
#      src\music_release.cst  —— 实装版（交付用）
#  本脚本负责「切换 -> 构建 -> 核对」，不用手改 music.gprj。
#
#  用法（在 music_player\music 目录下）：
#      pwsh -File tools\build_with_cst.ps1 -Mode release
#      pwsh -File tools\build_with_cst.ps1 -Mode debug
#
#  ★ 为什么必须核对：见工作交接.md 第 5 节坑 29 ——
#    gw_sh 即使编的是【另一个工程】也会照样打印 RUN_OK 和
#    "Generate file ... completed"，所以「只看 RUN_OK」完全不可靠。
#    唯一可信的判据是：本工程 impl\pnr\music.fs 的时间戳必须变新。
#    本脚本把这条判据做成了硬检查，不通过就以非 0 退出。
#
#  ★ 只改 music.gprj 里那两个 type="file.cst" 的 enable 标志，
#    其余字节原样保留（用 .NET 写回，不带 BOM）。
# =============================================================================

param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("debug", "release")]
    [string]$Mode
)

$ErrorActionPreference = "Stop"

# ---- 路径 ----
$Root    = Split-Path -Parent $PSScriptRoot          # ...\music_player\music
$Gprj    = Join-Path $Root "music.gprj"
$Tcl     = Join-Path $Root "build_console.tcl"
$GwSh    = "D:\CODE\FPGA_Gowin\Gowin\Gowin_V1.9.11.03_Education_X64\IDE\bin\gw_sh.exe"
$PnrOut  = Join-Path $Root "impl\pnr\music.fs"
$LogFile = Join-Path $Root ("build_{0}.log" -f $Mode)

foreach ($p in @($Gprj, $Tcl)) {
    if (-not (Test-Path $p)) { throw "找不到：$p" }
}
if (-not (Test-Path $GwSh)) { throw "找不到 gw_sh.exe：$GwSh" }

# ---- 1. 切换 enable 标志 ----
$wantDebug   = if ($Mode -eq "debug") { "1" } else { "0" }
$wantRelease = if ($Mode -eq "release") { "1" } else { "0" }

$text = [System.IO.File]::ReadAllText($Gprj, [System.Text.Encoding]::UTF8)

$patDebug   = '(<File\s+path="src/music\.cst"\s+type="file\.cst"\s+enable=")[01](")'
$patRelease = '(<File\s+path="src/music_release\.cst"\s+type="file\.cst"\s+enable=")[01](")'

$n1 = ([regex]::Matches($text, $patDebug)).Count
$n2 = ([regex]::Matches($text, $patRelease)).Count
if ($n1 -ne 1) { throw "music.gprj 里 src/music.cst 的 File 行找到 $n1 处（应为 1），不改动，退出。" }
if ($n2 -ne 1) { throw "music.gprj 里 src/music_release.cst 的 File 行找到 $n2 处（应为 1），不改动，退出。`n（如果刚加进来还没保存，请先在 music.gprj 的 FileList 里补上这一行）" }

$new = [regex]::Replace($text, $patDebug,   { param($m) $m.Groups[1].Value + $wantDebug   + $m.Groups[2].Value })
$new = [regex]::Replace($new,  $patRelease, { param($m) $m.Groups[1].Value + $wantRelease + $m.Groups[2].Value })

if ($new -ne $text) {
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Gprj, $new, $utf8NoBom)
    Write-Host "[切换] music.gprj 已更新：music.cst enable=$wantDebug, music_release.cst enable=$wantRelease"
} else {
    Write-Host "[切换] music.gprj 已经是目标状态（music.cst=$wantDebug, music_release.cst=$wantRelease），未改动"
}

# ---- 2. 复核：必须恰好启用一份 ----
$chk = [System.IO.File]::ReadAllText($Gprj, [System.Text.Encoding]::UTF8)
$mD = [regex]::Match($chk, $patDebug)
$mR = [regex]::Match($chk, $patRelease)
# ★ 必须用 .Value（【整个】匹配，含末尾那个 0/1），不能用 Groups[1] ——
#   第 1 组只到 enable=" 为止、不含数字，拿它去匹配 'enable="1"' 永远是 False，
#   会误报"两份 cst 启用状态一样"然后退出。（这个坑本脚本踩过一次）
$enDebug   = $mD.Success -and ($mD.Value -match 'enable="1"')
$enRelease = $mR.Success -and ($mR.Value -match 'enable="1"')
if ($enDebug -eq $enRelease) {
    throw "严格检查失败：两份 cst 的启用状态一样（debug=$enDebug, release=$enRelease）。必须恰好启用一份。"
}
$active = if ($enDebug) { "src\music.cst（调试版）" } else { "src\music_release.cst（实装版）" }
Write-Host "[复核] 当前生效的约束：$active"

# ---- 3. 记下构建前 music.fs 的时间戳（坑 29 的判据） ----
$before = $null
if (Test-Path $PnrOut) { $before = (Get-Item $PnrOut).LastWriteTime }
Write-Host ("[构建前] impl\pnr\music.fs 时间 = " + $(if ($before) { $before } else { "（不存在）" }))

# ---- 4. 构建（纯重定向，绝不用 Tee-Object） ----
Write-Host "[构建] 开始，约 18 分钟…… 日志：$LogFile"
$sw = [System.Diagnostics.Stopwatch]::StartNew()
& $GwSh $Tcl *> $LogFile
$code = $LASTEXITCODE
$sw.Stop()
Write-Host ("[构建] gw_sh 退出码 = {0}，耗时 {1:N1} 分钟" -f $code, $sw.Elapsed.TotalMinutes)

# ---- 5. ★ 硬核对：RUN_OK + music.fs 时间戳必须变新 ----
$logTail = Get-Content $LogFile -Tail 40
$runOk = ($logTail | Select-String -Pattern "RUN_OK").Count -gt 0
if (-not $runOk) {
    Write-Host "[失败] 日志里没有 RUN_OK，构建没成功。请看 $LogFile"
    exit 2
}

if (-not (Test-Path $PnrOut)) {
    Write-Host "[失败] impl\pnr\music.fs 不存在。"
    exit 3
}
$after = (Get-Item $PnrOut).LastWriteTime
if ($before -and ($after -le $before)) {
    Write-Host "[★失败★] music.fs 的时间戳【没有变新】（$before -> $after）。"
    Write-Host "         这几乎就是坑 29：构建脚本打开的是【另一个工程】，"
    Write-Host "         所以本目录的 impl\pnr 根本没被写。请检查 build_console.tcl 第 1 行的项目路径。"
    exit 4
}

$hash = (Get-FileHash $PnrOut -Algorithm SHA256).Hash
Write-Host ""
Write-Host "==================== 构建成功 ===================="
Write-Host ("生效约束 : {0}" -f $active)
Write-Host ("位流     : {0}" -f $PnrOut)
Write-Host ("时间     : {0}   （旧 {1}）" -f $after, $before)
Write-Host ("大小     : {0} 字节" -f (Get-Item $PnrOut).Length)
Write-Host ("SHA256   : {0}" -f $hash)
Write-Host "================================================="
exit 0
