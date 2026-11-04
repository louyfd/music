$ErrorActionPreference = 'Stop'
$path = 'D:\FPGA_60K\music\src\top.v'
$enc  = New-Object System.Text.UTF8Encoding($false)   # top.v is UTF-8 without BOM (verified byte-exact round-trip)

if (-not (Test-Path "$path.bak")) {
    Copy-Item $path "$path.bak"
    Write-Host "backup -> $path.bak"
}

$s = [System.IO.File]::ReadAllText($path, $enc)
# unify line endings (file was a mix of 2143 LF + 20 CRLF)
$s = [regex]::Replace($s, "`r`n", "`n")
$s = [regex]::Replace($s, "`r(?!`n)", "`n")
$orig = $s
$report = New-Object System.Collections.ArrayList

function Apply([string]$name, [string]$pattern, [string]$replacement, [bool]$isRegex) {
    if (-not $isRegex) { $pattern = [regex]::Replace($pattern, "`r`n", "`n") }
    $replacement = [regex]::Replace($replacement, "`r`n", "`n")
    $cnt = [regex]::Matches($script:s, $(if ($isRegex) { $pattern } else { [regex]::Escape($pattern) })).Count
    if ($cnt -ne 1) { [void]$script:report.Add("FAIL  $name  (matches=$cnt)"); return }
    if ($isRegex) { $script:s = [regex]::Replace($script:s, $pattern, $replacement, 1) }
    else          { $script:s = $script:s.Replace($pattern, $replacement) }
    [void]$script:report.Add("ok    $name")
}

# ------------------------------------------------------------------ 1) sum_env
$l1 = New-Object System.Collections.ArrayList
foreach ($x in @(
    '// ==================================================================',
    '    // ★★ 优化（数学上完全等价，不改任何功能）★★',
    '    //',
    '    //   env_a[g] = (正弦值 × env[g]) >> 8，而 sounding_mask[g] 的定义',
    '    //   恰好就是 (env[g] != 0)。也就是说：',
    '    //       env[g] == 0  ->  sounding_mask[g] == 0  且  env_a[g] == 0',
    '    //   所以原来的每一项',
    "    //       sounding_mask[g] ? env_a[g] : 22'sd0",
    '    //   恒等于 env_a[g] 本身 —— 这 36 个 22 位选择器是纯冗余。',
    '    //   去掉之后：省 36×22 ≈ 790 个 LUT，加法树少一级扇出，',
    '    //   sounding_mask 的扇出也大幅下降（它只再供 any_key / 屏幕频谱用）。',
    '    //',
    '    //   ★ 仍然是【单个表达式】，综合器照样自动搭 log2(N) 级平衡加法树，',
    '    //     千万不要改成 for 循环串行累加。',
    '    //   ★ 22 位足够：36 × 32767 = 1,179,612 < 2^21',
    '    // ==================================================================',
    '    wire signed [21:0] sum_env ='
)) { [void]$l1.Add($x) }
for ($i = 0; $i -lt 36; $i += 6) {
    $line = '        '
    for ($j = $i; $j -lt [Math]::Min($i + 6, 36); $j++) {
        $pad = if ($j -lt 10) { ' ' } else { '' }
        $line += "env_a[$j]$pad + "
    }
    $line = $line.Substring(0, $line.Length - 3)
    if ($i + 6 -lt 36) { $line += ' +' } else { $line += ';' }
    [void]$l1.Add($line)
}
$newSum = $l1 -join "`n"

Apply '1) sum_env 去掉 36 个冗余选择器' '(?s)wire signed \[21:0\] sum_env =.*?;' $newSum $true

# ----------------------------------------------------- 2) gain_lut index clamp
$old2 = @'
    reg [9:0] gain_k;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) gain_k <= 10'd256;
        else                gain_k <= gain_lut[env_load[13:3]];
    end
'@
$new2 = @'
    reg [9:0] gain_k;

    // ------------------------------------------------------------------
    // ★★ 越界索引修正（唯一一处会改变行为的地方，其实是修 bug）★★
    //
    //   env_load 最大 = 36 键 × 255 = 9180，env_load[13:3] 最大 = 1147，
    //   而 gain_lut 只有 512 项（0~511）—— 11 位的索引会越界，
    //   综合器丢掉高位后等于"地址绕回表头"，也就是 env_load > 4095 时
    //   增益会从 16 突然跳回几十甚至 256，多键同时满包络
    //   （比如延音锁住十几路）时会突然削顶。
    //   这里把索引钳在 511，与"表尾恒为 16"的设计意图一致。
    //
    //   ★ 不想要这一改：把下面 gain_idx 那行的比较去掉，
    //     直接写 gain_lut[env_load[11:3]] 就行（小和弦下两者完全一样）。
    // ------------------------------------------------------------------
    wire [8:0] gain_idx = (env_load[13:3] > 9'd511) ? 9'd511 : env_load[11:3];

    always @(posedge clk_audio) begin
        if (rst_audio_sync) gain_k <= 10'd256;
        else                gain_k <= gain_lut[gain_idx];
    end
'@
Apply '2) gain_lut 索引钳位' $old2 $new2 $false

# --------------------------------------------------------- 3) declare dg_wready
$old3 = @'
    reg [4:0] dg_idx;
'@
$new3 = @'
    wire        dg_wready;      // ★ 显式声明：原来靠隐式网络，综合日志报 Undeclared symbol
    reg [4:0] dg_idx;
'@
Apply '3) dg_wready 显式声明' $old3 $new3 $false

# ---------------------------------------------------------------- 4) tr_result
$new4 = @'
    // ★ 等价改写：原来是「34 位结果赋给 32 位」被静默截掉高 2 位（EX3791 告警）。
    //   直接写位选 tr_prod[33:2]，取值完全一样，告警消失。
    wire [31:0] tr_result = tr_prod[33:2];     // ≡ dcur × ratio / 65536
'@
Apply '4) tr_result 显式位选' '(?m)^    wire \[31:0\] tr_result = tr_prod >> 2;.*$' $new4 $true

# ---------------------------------------------------------------- 5) harm_sum
$old5 = @'
    wire signed [29:0] harm_mul  = S2 * $signed({1'b0, vel_audio});
    wire signed [22:0] harm_sum  = (timbre[0] ? (harm_mul >>> 9) : 23'sd0);
'@
$new5 = @'
    wire signed [29:0] harm_mul   = S2 * $signed({1'b0, vel_audio});
    // ★ 等价改写：右移不改变表达式位宽（仍是 30 位），原来赋给 23 位变量时
    //   取的是低 23 位；这里先落到 30 位 wire 再显式取 [22:0]，
    //   取值完全一致，只是消掉 EX3791 截断告警。
    wire signed [29:0] harm_shift = harm_mul >>> 9;
    wire signed [22:0] harm_sum   = timbre[0] ? harm_shift[22:0] : 23'sd0;
'@
Apply '5) harm_sum 显式位选' $old5 $new5 $false

# --------------------------------------------------- 6) 16-point average bit slice
$old6 = @'
    wire js_tap  = (js_tdiv == JS_TAP_DIV - 1'b1);
    wire js_done = js_tap && (js_acnt == JS_AVG_N-1);
'@
$new6 = @'
    wire js_tap  = (js_tdiv == JS_TAP_DIV - 1'b1);
    wire js_done = js_tap && (js_acnt == JS_AVG_N-1);

    // ★ 优化：把「16 点平均」写成显式位选。
    //   原来是 (xacc + ain0_data) >> JS_AVG_LOG2 再截成 16 位，
    //   等价于取加法结果的位段 [JS_AVG_LOG2+15 : JS_AVG_LOG2]，
    //   数值完全一致，只是把静默截断（EX3791 告警）变成写明的位段。
    wire [20:0] js_xsum = xacc + ain0_data;
    wire [20:0] js_ysum = yacc + ain1_data;
'@
Apply '6a) js 平均加法结果提出来' $old6 $new6 $false

$old7 = @'
                    js_x    <= (xacc + ain0_data) >> JS_AVG_LOG2;
                    js_y    <= (yacc + ain1_data) >> JS_AVG_LOG2;
'@
$new7 = @'
                    js_x    <= js_xsum[JS_AVG_LOG2+15 : JS_AVG_LOG2];
                    js_y    <= js_ysum[JS_AVG_LOG2+15 : JS_AVG_LOG2];
'@
Apply '6b) js_x / js_y 显式位选' $old7 $new7 $false

if ($s -eq $orig) { throw 'nothing changed' }
[System.IO.File]::WriteAllText($path, $s, $enc)
$report | ForEach-Object { Write-Host $_ }
Write-Host ("new chars = " + $s.Length + "   old chars = " + $orig.Length)
