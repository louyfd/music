$ErrorActionPreference = 'Stop'
$enc = New-Object System.Text.UTF8Encoding($false)
$top = 'D:\FPGA_60K\music\src\top.v'
$log = New-Object System.Collections.ArrayList

function Edit([string]$name, [string]$old, [string]$new, [bool]$isRegex) {
    $t = [System.IO.File]::ReadAllText($script:top, $enc)
    if (($enc.GetBytes($t) -join ',') -ne (([System.IO.File]::ReadAllBytes($script:top)) -join ',')) { throw "not clean UTF-8" }
    if ($isRegex) { $pat = $old } else { $pat = [regex]::Escape($old) }
    $pat = [regex]::Replace($pat, "`r`n", "`n"); $new = [regex]::Replace($new, "`r`n", "`n")
    $n = ([regex]::Matches($t, $pat)).Count
    if ($n -ne 1) { [void]$script:log.Add("FAIL  $name (matches=$n)"); return }
    if ($isRegex) { $r = [regex]::Replace($t, $pat, $new, 1) } else { $r = $t.Replace($old, $new) }
    [System.IO.File]::WriteAllText($script:top, $r, $enc)
    [void]$script:log.Add("ok    $name")
}

# ===== 1) 按钮功能表补上按钮6 =====
Edit '1) 按钮功能表' @'
    //   键5 -> 颤音（Tremolo）开关
    //   键6~16 暂时不用
'@ @'
    //   键5 -> 颤音（Tremolo）开关
    //   键6 -> 延迟（Delay / Echo）开关
    //   键7~16 暂时不用
'@ $false

# ===== 2) 收窄成按钮5，并加上按钮6 =====
Edit '2) btn_trem 收窄 + btn_dly + dly_on' @'
    // ★ 容错：bit4~bit15 全是【没用到的】按钮，所以任意一颗空按钮都能开关颤音。
    //   这样做是因为 16 键模块丝印上的"第几个触摸区"和 DATA 的哪一位
    //   并不一定有把握一一对上；等确认了按钮5 到底是哪一位，
    //   把这行收窄成 btn_new[4] 即可（功能不变，只是别的空按钮不再触发）。
    wire btn_trem = |btn_new[15:4];

    reg trem_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        trem_on <= 1'b0;          // 上电默认【关】
        else if (btn_trem) trem_on <= ~trem_on;      // 按一下开，再按一下关
    end
'@ @'
    //   ★ 按钮5/6 各管一个效果器，所以这里必须【一位对一位】，
    //     不能再像上一版那样用 |btn_new[15:4] 兜底（那样按6 会开成颤音）。
    wire btn_trem = btn_new[4];              // 按钮 5 -> 颤音
    wire btn_dly  = btn_new[5];              // 按钮 6 -> 延迟

    reg trem_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        trem_on <= 1'b0;          // 上电默认【关】
        else if (btn_trem) trem_on <= ~trem_on;      // 按一下开，再按一下关
    end

    reg dly_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       dly_on <= 1'b0;            // 上电默认【关】
        else if (btn_dly) dly_on <= ~dly_on;         // 按一下开，再按一下关
    end
'@ $false

# ===== 3) LED 指示把延迟也并进去 =====
Edit '3) led_glide 加 dly_on' @'
    assign led_glide = LED_ACTIVE_LOW ? ~(glide_en | sustain_on | trem_on)
                                      :  (glide_en | sustain_on | trem_on);
'@ @'
    assign led_glide = LED_ACTIVE_LOW ? ~(glide_en | sustain_on | trem_on | dly_on)
                                      :  (glide_en | sustain_on | trem_on | dly_on);
'@ $false

# ===== 4) 延迟开关跨到音频域 =====
Edit '4) dly_on 两级同步' @'
    wire trem_on_audio = trem_s2;
'@ @'
    wire trem_on_audio = trem_s2;

    // ---- 延迟开关跨到音频域（慢信号，两级同步足够）----
    reg dly_s1, dly_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            dly_s1 <= 1'b0;
            dly_s2 <= 1'b0;
        end else begin
            dly_s1 <= dly_on;
            dly_s2 <= dly_s1;
        end
    end
    wire dly_on_audio = dly_s2;
'@ $false

# ===== 5) 延迟效果器本体 + 干湿混合 =====
$new5 = @'
    // ★ 干声 = 原来直接写进 sample_vol 的那个值（钳位到 16 位有符号）
    wire signed [15:0] dry_sample =
        (vol_out >  23'sd32767) ?  16'sd32767 :
        (vol_out < -23'sd32768) ? -16'sd32768 :
                                  vol_out[15:0];

    //==================================================================
    // ★★★ 延迟效果器（Delay / Echo）：环形缓冲 + 反馈 + 干湿混合 ★★★
    //   按钮6 开关。最经典的一路回声：
    //
    //       干声 ──┬────────────────────────► 输出
    //              └─►(+)─►[延迟线 L 点]─┬─► ×WET ──┘
    //                   ▲                │
    //                   └─── ×FB ◄───────┘
    //
    //   ★ 延迟线用 BSRAM 推断（独立写指针 + 读指针寄存器）：
    //       DLY_LEN = 2^DLY_BITS = 16384 点，16384 / 48828Hz ≈ 335 ms
    //       16384×16bit = 256Kb → 约 16 块 BSRAM（本设计 118 块，还够）
    //     ★ 绝对不能写成变址的寄存器数组让综合器当分布式 RAM 展开，
    //       那样会变成几万个 FF。
    //   ★ 读地址就取写地址：环形缓冲里最"老"的那一格正是它，
    //     所以读出来刚好是 L 个采样点之前写进去的值，一个地址就够。
    //   ★ 上电时 BSRAM 内容未知，所以前 L 个采样点【只写 0】并把
    //     反馈/湿声全部静音，避免开机把垃圾当回声放出来。
    //   ★ 反馈只在效果器打开时累积：关掉时缓冲里始终是"纯干声的历史"，
    //     所以按一下按钮立刻就能听到刚才 335ms 的回声，不用等。
    //   ★ 关掉时 sample_vol 直接取 dry_sample，与加延迟之前【逐位相同】。
    //
    //   想调：
    //     DLY_BITS 13 -> 8192 点 ≈ 168ms（回声更密），12 -> 84ms
    //     DLY_FB   大 -> 拖尾更长（96 ≈ 0.375；160 ≈ 0.63 拖很久）
    //     DLY_WET  大 -> 回声更响（110 ≈ 0.43）
    //==================================================================
    localparam integer DLY_BITS = 14;
    localparam integer DLY_LEN  = 1 << DLY_BITS;
    localparam [7:0]   DLY_FB   = 8'd96;          // 反馈量 ≈ 0.375
    localparam [7:0]   DLY_WET  = 8'd110;         // 湿声量 ≈ 0.43

    reg  [15:0]         dly_mem [0:DLY_LEN-1];
    reg  [DLY_BITS-1:0] dly_wp, dly_rp;
    reg  signed [15:0]  dly_tap;

    // 上电填充计数：写满一圈之前不算数
    reg  [DLY_BITS-1:0] dly_fill;
    wire dly_ready = (dly_fill == {DLY_BITS{1'b1}});

    always @(posedge clk_audio) begin
        if (rst_audio_sync)              dly_fill <= {DLY_BITS{1'b0}};
        else if (sample_en && !dly_ready) dly_fill <= dly_fill + 1'b1;
    end

    wire signed [15:0] dly_use = dly_ready ? dly_tap : 16'sd0;

    // ★★ 位宽提醒：乘法结果宽度 = max(操作数宽度)，会被左边卡住。
    //    这里所有乘法的结果都必须先落到 24 位 wire 上（颤音就栽在这上面）。
    wire signed [23:0] dly_fb_p = dly_use * $signed({1'b0, DLY_FB});   // 24 位上下文
    wire signed [15:0] dly_fb   = dly_fb_p >>> 8;                       // ≈ tap × 0.375
    wire signed [16:0] dly_wsum = dry_sample + (dly_on_audio ? dly_fb : 16'sd0);
    wire signed [15:0] dly_wr   = (dly_wsum >  17'sd32767) ?  16'sd32767 :
                                  (dly_wsum < -17'sd32768) ? -16'sd32768 :
                                                              dly_wsum[15:0];

    always @(posedge clk_audio) begin
        if (sample_en) begin
            dly_mem[dly_wp] <= dly_ready ? dly_wr : 16'sd0;   // 写端口（含反馈）
            dly_tap         <= dly_mem[dly_rp];               // 读端口（= L 点之前）
            dly_wp          <= dly_wp + 1'b1;
            dly_rp          <= dly_rp + 1'b1;
        end
    end

    wire signed [23:0] dly_wet_p = dly_use * $signed({1'b0, DLY_WET});  // 24 位上下文
    wire signed [15:0] dly_wet   = dly_wet_p >>> 8;                     // ≈ tap × 0.43
    wire signed [16:0] dly_msum  = dry_sample + dly_wet;
    wire signed [15:0] dly_mix   = (dly_msum >  17'sd32767) ?  16'sd32767 :
                                   (dly_msum < -17'sd32768) ? -16'sd32768 :
                                                               dly_msum[15:0];

    reg signed [15:0] sample_vol;

    always @(posedge clk_audio) begin
        if (rst_audio_sync) sample_vol <= 16'sd0;
        else if (TEST_SINE) sample_vol <= test_sine_sample;
        else if (sample_cnt == 16'd10)
            sample_vol <= dly_on_audio ? dly_mix : dry_sample;
    end
'@
$pat5 = '(?s)    reg signed \[15:0\] sample_vol;\n\n    always @\(posedge clk_audio\) begin\n.*?\n    end\n'
Edit '5) 延迟效果器本体 + 干湿混合' $pat5 ($new5 + "`n") $true

$log | ForEach-Object { Write-Host $_ }
