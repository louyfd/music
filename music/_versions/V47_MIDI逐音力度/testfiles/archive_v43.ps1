# archive_v43.ps1
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$name = "V43_跟谱演奏和四档亮度"
Write-Host "版本名: $name"
& (Join-Path $here "make_version_archive.ps1") -Name $name