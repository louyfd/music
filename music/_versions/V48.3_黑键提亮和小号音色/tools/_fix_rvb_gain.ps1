$ErrorActionPreference = 'Stop'
$u = New-Object System.Text.UTF8Encoding($false)
$p = 'D:\FPGA_60K\music\src\top.v'
$t = [System.IO.File]::ReadAllText($p, $u)
$L = New-Object System.Collections.ArrayList
$L.AddRange([string[]]($t -split "`r`n"))

$fixed = 0
$notes = @(
    '    // ★★ 增益取值要点（第一版就栽在这）★★',
    '    //   这个"共享反馈"结构里，一个采样点要绕满一圈 2048 点才会被再次读出，',
    '    //   而一圈里它会依次经过三个抽头 => 等效环路增益 = G1×G2×G3（乘积！）。',
    '    //   一开始按"并联梳状"的直觉取 0.35/0.28/0.22，乘积只有 0.021，',
    '    //   实测尾巴 84ms 就没了。要 ~1.6 秒尾巴，乘积得 ~0.86：',
    '    //     0.977 × 0.953 × 0.922 = 0.858  ->  -0.66dB/周期(42ms) -> T60 ≈ 1.6s'
)

for ($i = 0; $i -lt $L.Count; $i++) {
    $line = $L[$i]
    if ($line.StartsWith("`$18'd")) {
        if ($line.StartsWith("`$18'd250")) {
            $L.RemoveAt($i); $L.Insert($i, "{0}" -f "    localparam [7:0]   RVB_G1  = 8'd250;              // 0.977")
            foreach ($x in ($notes[6..0])) { $L.Insert($i, $x) }
            $fixed++
        } elseif ($line.StartsWith("`$18'd244")) {
            $L.RemoveAt($i); $L.Insert($i, "{0}" -f "    localparam [7:0]   RVB_G2  = 8'd244;              // 0.953")
            $fixed++
        } elseif ($line.StartsWith("`$18'd236")) {
            $L.RemoveAt($i); $L.Insert($i, "{0}" -f "    localparam [7:0]   RVB_G3  = 8'd236;              // 0.922")
            $fixed++
        }
    }
}
Write-Host "修复 $fixed 行"

$bad = "    //   ★ 反馈增益合计 0.352+0.281+0.215 = 0.85 < 1，稳定不啸叫。"
$good = "    //   ★ 环路增益 = G1×G2×G3（见下面 localparam 处的说明），0.858 < 1，稳定不啸叫。"
$t2 = ($L -join "`r`n")
if ($t2.Contains($bad)) { $t2 = $t2.Replace($bad, $good); Write-Host "增益说明注释已更正" }

[System.IO.File]::WriteAllText($p, $t2, $u)
$s = $u.GetString([System.IO.File]::ReadAllBytes($p))
Write-Host ("CRLF=" + ([regex]::Matches($s,"`r`n")).Count + " 单独LF=" + ([regex]::Matches($s,"(?<!`r)`n")).Count)
Write-Host ("utf8合法=" + (($u.GetBytes($s) -join ',') -eq (([System.IO.File]::ReadAllBytes($p)) -join ',')))
Write-Host ("残留 `$18 = " + ([regex]::Matches($s,[regex]::Escape("`$18"))).Count)
$LL = $s -split "`r`n"
for ($n = 0; $n -lt $LL.Count; $n++) { if ($LL[$n] -match 'RVB_G[123]\s+=') { "{0,5}|{1}" -f ($n+1), $LL[$n] } }
