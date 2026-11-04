# archive_v42.ps1 —— 归档 V42（版本名写在【文件里】，不走命令行 —— 见交接文档坑 15）
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$name = "V42_LED物理顺序映射"
Write-Host "版本名: $name"
& (Join-Path $here "make_version_archive.ps1") -Name $name