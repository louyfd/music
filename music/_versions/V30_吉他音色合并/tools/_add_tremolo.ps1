$ErrorActionPreference = 'Stop'
$path = 'D:\FPGA_60K\music\src\top.v'
$enc  = New-Object System.Text.UTF8Encoding($false)

if (-not (Test-Path "$path.trem.bak")) {
    Copy-Item $path "$path.trem.bak"
    Write-Host "backup -> $path.trem.bak"
}

$s = [System.IO.File]::ReadAllText($path, $enc)
$s = [regex]::Replace($s, "`r`n", "`n")
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

# ============================================================ A) 50MHz 域：按钮5
$oldA = @'
        else if (btn_wave) wave_mode <= ~wave_mode;
    end
'@
$newA = @'
        else if (btn_wave) wave_mode <= ~wave_mode;
    end

    //==================================================================
    // ★★ 颤音（Tremolo）效果器开关：按【按钮5】= btn_keys[4] ★★
    //
    //   ★ 按钮5 现在是 16 键触摸模块上唯一空着的按钮：
    //       按钮1(bit0)=滑音   按钮2(bit1)=延音
    //       按钮3(bit2)=波形   按钮4(bit3)=音色（实测无反应的那颗）
    //       按钮5(bit4)=颤音  ← 本次新增
    //   ★ 摇杆模块上那个 SW 在 music.cst 里【没有端口】，本工程根本读不到，
    //     要用它必须先加端口 + .cst 约束（见文件里 key_sw 的说明）。
    //
    //   这里只做"按一下翻转一次"；效果器本体在音频域，
    //   开关状态用两级同步器送过去（慢信号，2 级足够）。
    //==================================================================
    wire btn_trem = btn_new[4];              // 按钮 5

    reg trem_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        trem_on <= 1'b0;          // 上电默认【关】
        else if (btn_trem) trem_on <= ~trem_on;      // 按一下开，再按一下关
    end
'@
Apply 'A) 按钮5 颤音开关（50MHz 域）' $oldA $newA $false

# ================================================== B) 50MHz -> 12.5MHz 两级同步
$oldB = @'
    wire sustain_on_audio = sustain_s2;
'@
$newB = @'
    wire sustain_on_audio = sustain_s2;

    // ---- 颤音开关跨到音频域（慢信号，两级同步足够）----
    reg trem_s1, trem_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            trem_s1 <= 1'b0;
            trem_s2 <= 1'b0;
        end else begin
            trem_s1 <= trem_on;
            trem_s2 <= trem_s1;
        end
    end
    wire trem_on_audio = trem_s2;
'@
Apply 'B) 颤音开关 50MHz->12.5MHz 同步' $oldB $newB $false

# ============================================================ C) 音频域：效果器本体
$oldC = @'
    localparam [7:0] VOL_FIXED = 8'd220;

    wire signed [31:0] vol_prod = vol_norm * $signed({1'b0, VOL_FIXED});
'@
$newC = @'
    localparam [7:0] VOL_FIXED = 8'd220;

    //==================================================================
    // ★★★ 颤音效果器（Tremolo）：慢速幅度调制 AM ★★★
    //
    //   加在【自动增益 + 归一化之后】，所以任何音色、任何力度下听感都是
    //   稳定的"忽明忽暗"，不会和前面的 AGC 打架。
    //
    //   ★ 不额外占乘法器：调制量直接扣在原来的 VOL_FIXED 上，
    //     复用的还是 vol_norm × vol_dyn 那一个乘级。
    //   ★ 关掉时 trem_amt 稳在 0 -> vol_dyn = VOL_FIXED，
    //     输出与"没加颤音"时【逐位相同】（不影响原来的声音）。
    //
    //   LFO：32 位相位累加器取三角波
    //        freq = TREM_INC × fs / 2^32 = 439800 × 48828 / 2^32 ≈ 5.0 Hz
    //        三角波算法：相位最高位选升/降，再看 [30:25] 六位当取值
    //        （前半周 [30:25] 单调 0→63，后半周取反 63→0，一个来回正好一个周期）
    //   深度：最大衰减 110 / 220 = 1/2 → -6dB（谷底不会变成完全静音）
    //   斜坡：trem_amt 每采样点最多动 1，开关时不"咔"（约 2.3ms 淡入淡出）
    //==================================================================
    localparam [31:0] TREM_INC   = 32'd439800;   // LFO 约 5.0 Hz
    localparam [7:0]  TREM_DEPTH = 8'd110;       // 最大衰减量（相对 VOL_FIXED=220）

    reg [31:0] trem_acc;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) trem_acc <= 32'd0;
        else if (sample_en) trem_acc <= trem_acc + TREM_INC;
    end

    // 三角波 0~63~0
    wire [5:0] trem_tri = trem_acc[31] ? ~trem_acc[30:25] : trem_acc[30:25];

    // 目标衰减量 = tri/64 × DEPTH（tri 只有 6 位 × 常数，很便宜）
    wire [7:0] trem_tgt = (trem_tri * TREM_DEPTH) >> 6;

    // 衰减量斜坡：关掉时回 0；打开时追目标值
    reg [7:0] trem_amt;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) trem_amt <= 8'd0;
        else if (sample_en) begin
            if (!trem_on_audio) begin
                if (trem_amt != 8'd0)         trem_amt <= trem_amt - 1'b1;
            end else if (trem_amt < trem_tgt) trem_amt <= trem_amt + 1'b1;
            else if     (trem_amt > trem_tgt) trem_amt <= trem_amt - 1'b1;
        end
    end

    // 颤音后的等效音量：VOL_FIXED 扣掉衰减量（钳住，不出现负数）
    //   ★ 想让颤音更深：把 TREM_DEPTH 从 110 调大（最大 220 = 谷底静音）
    //   ★ 想换速度：改 TREM_INC，freq ≈ TREM_INC × 48828 / 2^32
    wire [7:0] vol_dyn = (trem_amt >= VOL_FIXED) ? 8'd0 : (VOL_FIXED - trem_amt);

    wire signed [31:0] vol_prod = vol_norm * $signed({1'b0, vol_dyn});
'@
Apply 'C) 颤音效果器本体（音频域）' $oldC $newC $false

[System.IO.File]::WriteAllText($path, $s, $enc)
$report | ForEach-Object { Write-Host $_ }
