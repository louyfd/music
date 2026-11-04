$ErrorActionPreference = 'Stop'
$enc = New-Object System.Text.UTF8Encoding($false)
$top = 'D:\FPGA_60K\music\src\top.v'
$log = New-Object System.Collections.ArrayList
$edits = @()

$edits += ,@('F1 按钮8/9 开关', $false, @'
    reg dly_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       dly_on <= 1'b0;            // 上电默认【关】
        else if (btn_dly) dly_on <= ~dly_on;         // 按一下开，再按一下关
    end
'@, @'
    reg dly_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       dly_on <= 1'b0;            // 上电默认【关】
        else if (btn_dly) dly_on <= ~dly_on;         // 按一下开，再按一下关
    end

    //==================================================================
    // ★★ 按钮8 = 混响（Reverb）  按钮9 = 合唱（Chorus）★★
    //   两个都是【纯新增】：关掉时整条音频链路逐位等于现在，其它功能一律不动。
    //   ★ 都【不需要双声道】，理由写在各效果器本体那一段里。
    //==================================================================
    wire btn_rev = btn_new[7];               // 按钮 8 -> 混响
    wire btn_cho = btn_new[8];               // 按钮 9 -> 合唱

    reg rev_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       rev_on <= 1'b0;            // 上电默认【关】
        else if (btn_rev) rev_on <= ~rev_on;
    end

    reg cho_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       cho_on <= 1'b0;            // 上电默认【关】
        else if (btn_cho) cho_on <= ~cho_on;
    end
'@)

# F2 取消：led_glide 已经被改成颤音 PWM 亮度指示，不去动它

$edits += ,@('F3 混响/合唱开关跨到音频域', $false, @'
    wire flute_on_audio = flt_s2;
'@, @'
    wire flute_on_audio = flt_s2;

    // ---- 混响 / 合唱开关跨到音频域（慢信号，两级同步足够）----
    reg rev_s1, rev_s2, cho_s1, cho_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            rev_s1 <= 1'b0; rev_s2 <= 1'b0;
            cho_s1 <= 1'b0; cho_s2 <= 1'b0;
        end else begin
            rev_s1 <= rev_on; rev_s2 <= rev_s1;
            cho_s1 <= cho_on; cho_s2 <= cho_s1;
        end
    end
    wire rev_on_audio = rev_s2;
    wire cho_on_audio = cho_s2;
'@)

# ---------------- 合唱块：插在 dry_sample 之后 ----------------
$edits += ,@('F4 合唱效果器', $false, @'
    wire signed [15:0] dry_sample =
        (vol_out_g >  24'sd32767) ?  16'sd32767 :
        (vol_out_g < -24'sd32768) ? -16'sd32768 :
                                     vol_out_g[15:0];
'@, @'
    wire signed [15:0] dry_sample =
        (vol_out_g >  24'sd32767) ?  16'sd32767 :
        (vol_out_g < -24'sd32768) ? -16'sd32768 :
                                     vol_out_g[15:0];

    //==================================================================
    // ★★★ 合唱（Chorus）★★★  按钮9
    //
    //   原理：干声 + 一个【延迟量被慢速 LFO 来回拉动】的副本。
    //   延迟量一变，副本的音高就跟着微微升降，和干声一叠加就产生拍频，
    //   听感"变厚、像两个人一起唱"——这就是最经典的合唱做法。
    //
    //   ★★ 不需要双声道 ★★
    //     单声道合唱就是这一条被调制的延迟线，所有吉他合唱单块都是单声道做的。
    //     立体声合唱只是"左右两路用反相 LFO、拉得更宽"——那是"更好听"，
    //     不是"必需"。
    //
    //   ★ 延迟量必须带【小数位】并做线性插值：只挪整数采样点的话，
    //     每挪一下就是一次硬跳变，会听到"咔咔"的阶梯噪声。
    //     所以用了两个存储体：A 读 n-D，B 读 n-D+1，再按小数部分插值。
    //
    //   参数：中心延迟 15.0ms，LFO ±2.0ms @0.9Hz，干湿各 0.5
    //         （干湿各半 => 峰值最多 0.5+0.5，不会削顶）
    //==================================================================
    localparam integer CHO_BITS = 10;                 // 1024 点 ≈ 21ms 缓冲
    localparam integer CHO_LEN  = 1 << CHO_BITS;
    localparam [31:0]  CHO_INC      = 32'd79164;      // LFO ≈ 0.9Hz @48828Hz
    localparam [17:0]  CHO_BASE_Q8  = 18'd187392;     // 732 点 << 8 = 15.0ms
    localparam [17:0]  CHO_DEPTH_Q8 = 18'd25088;      //  98 点 << 8 = ±2.0ms

    // ---- 上电填充：合唱 1024 + 混响 2048 两条线都写满一遍之前不出声 ----
    //   （BSRAM 上电内容未知，不填就是一声垃圾噪声）
    reg [10:0] fx_fill;
    wire fx_ready = (fx_fill == 11'h7FF);
    always @(posedge clk_audio) begin
        if (rst_audio_sync)               fx_fill <= 11'd0;
        else if (sample_en && !fx_ready)  fx_fill <= fx_fill + 1'b1;
    end

    // ---- LFO：32 位相位累加器取三角波（和颤音同一套做法）----
    reg [31:0] cho_acc;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) cho_acc <= 32'd0;
        else if (sample_en) cho_acc <= cho_acc + CHO_INC;
    end
    wire [7:0] cho_tri = cho_acc[31] ? ~cho_acc[30:23] : cho_acc[30:23];  // 0~255 三角波
    wire signed [8:0] cho_lfo = $signed({1'b0, cho_tri}) - 9'sd128;        // -128 ~ +127

    // 调制量（Q8）：±(128 × DEPTH_Q8)/128
    wire signed [23:0] cho_dmod = cho_lfo * $signed({1'b0, CHO_DEPTH_Q8});   // 24 位上下文
    // 最终延迟量（Q8，19 位有符号）：162304 ~ 212480 即 634 ~ 830 点
    wire signed [18:0] cho_dly_q8 = $signed({1'b0, CHO_BASE_Q8}) + (cho_dmod >>> 7);

    wire [9:0] cho_int  = cho_dly_q8[17:8];      // 整数部分
    wire [7:0] cho_frac = cho_dly_q8[7:0];       // 小数部分（插值权重）

    // ---- 两个存储体：内容相同，读的偏移差 1 ----
    reg  [15:0]        cho_ma [0:CHO_LEN-1];
    reg  [15:0]        cho_mb [0:CHO_LEN-1];
    reg  [9:0]         cho_wp, cho_ra, cho_rb;
    reg  signed [15:0] cho_a, cho_b;

    wire signed [16:0] cho_dd  = cho_b - cho_a;                       // 新 - 旧
    wire signed [24:0] cho_ip  = cho_dd * $signed({1'b0, cho_frac});  // 25 位上下文
    wire signed [16:0] cho_tap = cho_a + (cho_ip >>> 8);              // 线性插值结果
    // 干湿各半；上电填充期不出湿声
    wire signed [16:0] cho_msum = (dry_sample >>> 1) + (fx_ready ? (cho_tap >>> 1) : 17'sd0);
    wire signed [15:0] cho_mix  = (cho_msum >  17'sd32767) ?  16'sd32767 :
                                  (cho_msum < -17'sd32768) ? -16'sd32768 :
                                                              cho_msum[15:0];

    always @(posedge clk_audio) begin
        if (sample_en) begin
            cho_ma[cho_wp] <= dry_sample;          // 合唱没有反馈，写进去的就是干声
            cho_mb[cho_wp] <= dry_sample;
            cho_a  <= cho_ma[cho_ra];              // 读端口用【已寄存的】地址
            cho_b  <= cho_mb[cho_rb];
            cho_ra <= cho_wp - cho_int;
            cho_rb <= cho_wp - cho_int + 10'd1;
            cho_wp <= cho_wp + 1'b1;
        end
    end

    // 合唱关掉时 s_cho 就是干声，逐位不变
    wire signed [15:0] s_cho = cho_on_audio ? cho_mix : dry_sample;
'@)

$edits += ,@('F5 延迟输入改接合唱输出', $false, @'
    wire signed [16:0] dly_wsum = dry_sample + (dly_on_audio ? dly_fb : 16'sd0);
'@, @'
    wire signed [16:0] dly_wsum = s_cho + (dly_on_audio ? dly_fb : 16'sd0);
'@)

$edits += ,@('F6 延迟干声改接合唱输出', $false, @'
    wire signed [16:0] dly_msum  = dry_sample + dly_wet;
'@, @'
    wire signed [16:0] dly_msum  = s_cho + dly_wet;
'@)

# ---------------- 混响块 + 输出改写 ----------------
$edits += ,@('F7 混响效果器', $false, @'
                                                               dly_msum[15:0];
'@, @'
                                                               dly_msum[15:0];

    // 延迟关掉时就是合唱输出；逐位不变
    wire signed [15:0] s_dly = dly_on_audio ? dly_mix : s_cho;

    //==================================================================
    // ★★★ 混响（Reverb）★★★  按钮8
    //
    //   结构：三条不同长度的延迟线并联成"多抽头反馈梳状滤波器"，
    //   反馈回路里带一阶低通（阻尼），尾巴才不会金属味太重：
    //
    //        in ─►(+)─►[21ms]──┬─► t1 ─► ×G1 ─┐
    //             ▲  [29ms]──┼─► t2 ─► ×G2 ─┼─► 阻尼低通 ─┐
    //             │  [34ms]──┴─► t3 ─► ×G3 ─┘            │
    //             └───────────────────────────────────────┘
    //        out = in/2 + (t1+t2+t3)/4 × WET
    //
    //   ★★ 不需要双声道 ★★
    //     单声道混响就是几条延迟线的反馈网络。立体声混响只是"左右各一套
    //     不同参数"，听起来更宽，但单声道混响完全成立、也是常见做法。
    //
    //   ★ 三条线用三个独立存储体（都写同一份数据、读的偏移不同）：
    //     一个 BSRAM 只有一个读端口，要在同一拍拿到 3 个抽头就得复制存储体。
    //     本设计 BSRAM 富余（118 块），这样最省事也最不容易出错。
    //   ★ 反馈增益合计 0.352+0.281+0.215 = 0.85 < 1，稳定不啸叫。
    //   ★ 干声压一半再叠加，配合 8 倍输出增益也不会削顶。
    //==================================================================
    localparam integer RVB_BITS = 11;                 // 2048 点 ≈ 42ms
    localparam integer RVB_LEN  = 1 << RVB_BITS;
    localparam [10:0]  RVB_D1  = 11'd1024;            // 21.0ms
    localparam [10:0]  RVB_D2  = 11'd1429;            // 29.3ms
    localparam [10:0]  RVB_D3  = 11'd1663;            // 34.1ms
    localparam [7:0]   RVB_G1  = 8'd90;               // 0.352
    localparam [7:0]   RVB_G2  = 8'd72;               // 0.281
    localparam [7:0]   RVB_G3  = 8'd55;               // 0.215
    localparam [7:0]   RVB_WET = 8'd110;              // 湿声 0.43

    reg  [15:0]        rvb_m1 [0:RVB_LEN-1];
    reg  [15:0]        rvb_m2 [0:RVB_LEN-1];
    reg  [15:0]        rvb_m3 [0:RVB_LEN-1];
    reg  [10:0]        rvb_wp, rvb_r1, rvb_r2, rvb_r3;
    reg  signed [15:0] rvb_t1, rvb_t2, rvb_t3;
    reg  signed [16:0] rvb_lp;

    // 填充期内三路抽头全部按 0 算
    wire signed [16:0] rvb_u1 = fx_ready ? rvb_t1 : 17'sd0;
    wire signed [16:0] rvb_u2 = fx_ready ? rvb_t2 : 17'sd0;
    wire signed [16:0] rvb_u3 = fx_ready ? rvb_t3 : 17'sd0;

    // ★ 位宽：三个乘法都先落到 24 位 wire，不能直接赋给窄变量
    wire signed [23:0] rvb_p1 = rvb_u1 * $signed({1'b0, RVB_G1});
    wire signed [23:0] rvb_p2 = rvb_u2 * $signed({1'b0, RVB_G2});
    wire signed [23:0] rvb_p3 = rvb_u3 * $signed({1'b0, RVB_G3});
    wire signed [17:0] rvb_fb = (rvb_p1 >>> 8) + (rvb_p2 >>> 8) + (rvb_p3 >>> 8);

    wire signed [19:0] rvb_ws = s_dly + rvb_lp;
    wire signed [15:0] rvb_wr = (rvb_ws >  20'sd32767) ?  16'sd32767 :
                                (rvb_ws < -20'sd32768) ? -16'sd32768 :
                                                          rvb_ws[15:0];

    always @(posedge clk_audio) begin
        if (rst_audio_sync) rvb_lp <= 17'sd0;
        else if (sample_en) begin
            rvb_m1[rvb_wp] <= fx_ready ? rvb_wr : 16'sd0;
            rvb_m2[rvb_wp] <= fx_ready ? rvb_wr : 16'sd0;
            rvb_m3[rvb_wp] <= fx_ready ? rvb_wr : 16'sd0;
            rvb_t1 <= rvb_m1[rvb_r1];
            rvb_t2 <= rvb_m2[rvb_r2];
            rvb_t3 <= rvb_m3[rvb_r3];
            rvb_r1 <= rvb_wp - RVB_D1;
            rvb_r2 <= rvb_wp - RVB_D2;
            rvb_r3 <= rvb_wp - RVB_D3;
            rvb_wp <= rvb_wp + 1'b1;
            // 一阶低通阻尼（>>2 约 2kHz 转折）：让高频先衰减，尾巴更自然
            rvb_lp <= rvb_lp + ((rvb_fb - rvb_lp) >>> 2);
        end
    end

    wire signed [18:0] rvb_avg  = (rvb_u1 + rvb_u2 + rvb_u3) >>> 2;         // 三路平均
    wire signed [26:0] rvb_wmul = rvb_avg * $signed({1'b0, RVB_WET});       // 27 位上下文
    wire signed [18:0] rvb_wet  = rvb_wmul >>> 8;
    wire signed [19:0] rvb_msum = (s_dly >>> 1) + rvb_wet;                  // 干声压一半防削顶
    wire signed [15:0] rvb_mix  = (rvb_msum >  20'sd32767) ?  16'sd32767 :
                                  (rvb_msum < -20'sd32768) ? -16'sd32768 :
                                                              rvb_msum[15:0];
'@)

$edits += ,@('F8 输出接混响', $false, @'
            sample_vol <= dly_on_audio ? dly_mix : dry_sample;
'@, @'
            sample_vol <= rev_on_audio ? rvb_mix : s_dly;
'@)

$edits += ,@('F9 按钮表补8/9', $false, @'
    //     按钮7(bit6)=笛子音色
'@, @'
    //     按钮7(bit6)=笛子音色  按钮8(bit7)=混响  按钮9(bit8)=合唱
'@)

# ---------- 预检 ----------
$raw = [System.IO.File]::ReadAllText($top, $enc)
$eol = if ([regex]::Matches($raw, "`r`n").Count -gt 0 -and [regex]::Matches($raw, "(?<!`r)`n").Count -eq 0) { "`r`n" } else { "`n" }
Write-Host ("行尾 = " + $(if ($eol -eq "`r`n") { 'CRLF' } else { 'LF' }))
$t = [regex]::Replace($raw, "`r`n", "`n")
$bad = 0
foreach ($e in $edits) {
    $pat = if ($e[1]) { $e[2] } else { [regex]::Escape(([regex]::Replace($e[2], "`r`n", "`n"))) }
    $n = ([regex]::Matches($t, $pat)).Count
    if ($n -ne 1) { $bad++; [void]$log.Add("PRE-FAIL $($e[0])  matches=$n") }
}
if ($bad -gt 0) { $log | ForEach-Object { Write-Host $_ }; throw "锚点不唯一，一个字都没写" }
Write-Host "预检通过：$($edits.Count) 个锚点全部唯一"

# ---------- 应用 ----------
foreach ($e in $edits) {
    $new = [regex]::Replace($e[3], "`r`n", "`n")
    if ($e[1]) { $t = [regex]::Replace($t, $e[2], $new, 1) }
    else       { $t = $t.Replace(([regex]::Replace($e[2], "`r`n", "`n")), $new) }
    [void]$log.Add("ok    $($e[0])")
}
$out = if ($eol -eq "`r`n") { [regex]::Replace($t, "(?<!`r)`n", "`r`n") } else { $t }
[System.IO.File]::WriteAllText($top, $out, $enc)
$log | ForEach-Object { Write-Host $_ }
