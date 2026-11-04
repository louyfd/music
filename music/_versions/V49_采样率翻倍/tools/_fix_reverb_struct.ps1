$ErrorActionPreference = 'Stop'
$u = New-Object System.Text.UTF8Encoding($false)
$p = 'D:\FPGA_60K\music\src\top.v'
$t = [System.IO.File]::ReadAllText($p, $u)
$L = New-Object System.Collections.ArrayList
$L.AddRange([string[]]($t -split "`r`n"))

$i0 = -1; $i1 = -1
for ($n = 0; $n -lt $L.Count; $n++) {
    if ($L[$n].Contains('延迟关掉时就是合唱输出')) { $i0 = $n }
    if ($L[$n].Contains('rvb_msum[15:0];'))       { $i1 = $n; break }
}
if ($i0 -lt 0 -or $i1 -lt 0 -or $i1 -le $i0) { throw "找不到混响块边界 i0=$i0 i1=$i1" }
Write-Host "替换行范围：$($i0+1) .. $($i1+1)（共 $($i1-$i0+1) 行）"

$new = @'
    // 延迟关掉时就是合唱输出；逐位不变
    wire signed [15:0] s_dly = dly_on_audio ? dly_mix : s_cho;

    //==================================================================
    // ★★★ 混响（Reverb）★★★  按钮8
    //
    //   3 个【并联梳状滤波器】+ 反馈阻尼低通，输出取三路抽头的平均：
    //
    //        ┌─[21ms 梳状 g=0.918]─┐
    //   in ──┼─[29ms 梳状 g=0.891]─┼──► (t1+t2+t3)/4 ×WET ──┐
    //        └─[34ms 梳状 g=0.875]─┘                        ├─► out
    //   in/2 ──────────────────────────────────────────────┘
    //
    //   每个梳状：w_i[n] = in[n] + ( lp_i / 256 )，lp_i 是 g_i·w_i[n-D_i] 的一阶低通
    //
    //   ★★ 不需要双声道 ★★
    //     单声道混响就是几条延迟线的反馈网络，完全成立、也是常见做法。
    //     立体声混响只是"左右各一套不同参数"，听起来更宽，但不是必需的。
    //
    //   ★★ 两个必须知道的坑（都实测踩过）★★
    //     1) 结构一定要【并联】。第一版写成"三条线共享一个反馈"的多抽头结构，
    //        那种结构里一个脉冲每绕一圈会分裂成 3 个，等效环路增益是三个增益
    //        之【和】（0.977+0.953+0.922 = 2.85）——直接自激，实测内部削顶。
    //        并联则每个梳状独立反馈，判据很干净：g_i < 1 就稳。
    //     2) 精度：阻尼低通会把每个回声抹开成一小段，之后每圈的整数截断会
    //        累积成很大损耗（实测多掉 18dB/0.2s，尾巴 0.6 秒就没）。
    //        所以反馈路径全程用 Q8 定点（lp 里存的是 ×256 的值），
    //        截断损失降到 1/256 个采样 LSB，可以忽略。
    //
    //   实测（10000 幅度单点冲激，阻尼 >>2）：
    //     0.2s 约 0dB   0.4s -10dB   0.6s -28dB
    //     （真实演奏电平比这高 3 倍左右，实际尾巴还要更长，听感约 1 秒）
    //   想更长：把 G1/G2/G3 往上加。实测 248(=0.969) 时 T60 ≈ 4 秒且仍不发散。
    //==================================================================
    localparam integer RVB_BITS = 11;                 // 2048 点 ≈ 42ms（最长 34ms 够用）
    localparam integer RVB_LEN  = 1 << RVB_BITS;
    localparam [10:0]  RVB_D1  = 11'd1024;            // 21.0ms
    localparam [10:0]  RVB_D2  = 11'd1429;            // 29.3ms
    localparam [10:0]  RVB_D3  = 11'd1663;            // 34.1ms
    localparam [7:0]   RVB_G1  = 8'd235;              // 0.918
    localparam [7:0]   RVB_G2  = 8'd228;              // 0.891
    localparam [7:0]   RVB_G3  = 8'd224;              // 0.875
    localparam [7:0]   RVB_WET = 8'd110;              // 湿声 0.43

    reg  [15:0]        rvb_m1 [0:RVB_LEN-1];
    reg  [15:0]        rvb_m2 [0:RVB_LEN-1];
    reg  [15:0]        rvb_m3 [0:RVB_LEN-1];
    reg  [10:0]        rvb_wp, rvb_r1, rvb_r2, rvb_r3;
    reg  signed [15:0] rvb_t1, rvb_t2, rvb_t3;
    reg  signed [24:0] rvb_l1, rvb_l2, rvb_l3;        // Q8 阻尼低通状态（值 = 采样 × 256）

    // 填充期内三路抽头全部按 0 算（BSRAM 上电内容未知）
    wire signed [16:0] rvb_u1 = fx_ready ? rvb_t1 : 17'sd0;
    wire signed [16:0] rvb_u2 = fx_ready ? rvb_t2 : 17'sd0;
    wire signed [16:0] rvb_u3 = fx_ready ? rvb_t3 : 17'sd0;

    // ★ 位宽：乘法结果必须落到够宽的中间 wire（乘法宽度 = max(操作数宽度)）
    wire signed [23:0] rvb_p1 = rvb_u1 * $signed({1'b0, RVB_G1});   // Q8 反馈量
    wire signed [23:0] rvb_p2 = rvb_u2 * $signed({1'b0, RVB_G2});
    wire signed [23:0] rvb_p3 = rvb_u3 * $signed({1'b0, RVB_G3});
    wire signed [24:0] rvb_d1 = rvb_p1 - rvb_l1;                    // Q8 差值
    wire signed [24:0] rvb_d2 = rvb_p2 - rvb_l2;
    wire signed [24:0] rvb_d3 = rvb_p3 - rvb_l3;
    wire signed [16:0] rvb_f1 = rvb_l1 >>> 8;                       // 回到整数采样
    wire signed [16:0] rvb_f2 = rvb_l2 >>> 8;
    wire signed [16:0] rvb_f3 = rvb_l3 >>> 8;

    // 三个梳状的写数据：in + 阻尼后的反馈
    wire signed [17:0] rvb_a1 = s_dly + rvb_f1;
    wire signed [17:0] rvb_a2 = s_dly + rvb_f2;
    wire signed [17:0] rvb_a3 = s_dly + rvb_f3;
    wire signed [15:0] rvb_w1 = (rvb_a1 >  18'sd32767) ?  16'sd32767 :
                                (rvb_a1 < -18'sd32768) ? -16'sd32768 : rvb_a1[15:0];
    wire signed [15:0] rvb_w2 = (rvb_a2 >  18'sd32767) ?  16'sd32767 :
                                (rvb_a2 < -18'sd32768) ? -16'sd32768 : rvb_a2[15:0];
    wire signed [15:0] rvb_w3 = (rvb_a3 >  18'sd32767) ?  16'sd32767 :
                                (rvb_a3 < -18'sd32768) ? -16'sd32768 : rvb_a3[15:0];

    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            rvb_l1 <= 25'sd0; rvb_l2 <= 25'sd0; rvb_l3 <= 25'sd0;
        end else if (sample_en) begin
            rvb_m1[rvb_wp] <= fx_ready ? rvb_w1 : 16'sd0;
            rvb_m2[rvb_wp] <= fx_ready ? rvb_w2 : 16'sd0;
            rvb_m3[rvb_wp] <= fx_ready ? rvb_w3 : 16'sd0;
            rvb_t1 <= rvb_m1[rvb_r1];
            rvb_t2 <= rvb_m2[rvb_r2];
            rvb_t3 <= rvb_m3[rvb_r3];
            rvb_r1 <= rvb_wp - RVB_D1;
            rvb_r2 <= rvb_wp - RVB_D2;
            rvb_r3 <= rvb_wp - RVB_D3;
            rvb_wp <= rvb_wp + 1'b1;
            // 一阶低通阻尼（>>2 约 2kHz 转折）：让高频先衰减，尾巴更自然
            rvb_l1 <= rvb_l1 + (rvb_d1 >>> 2);
            rvb_l2 <= rvb_l2 + (rvb_d2 >>> 2);
            rvb_l3 <= rvb_l3 + (rvb_d3 >>> 2);
        end
    end

    wire signed [18:0] rvb_avg  = (rvb_u1 + rvb_u2 + rvb_u3) >>> 2;      // 三路抽头平均
    wire signed [26:0] rvb_wmul = rvb_avg * $signed({1'b0, RVB_WET});    // 27 位上下文
    wire signed [18:0] rvb_wet  = rvb_wmul >>> 8;
    wire signed [19:0] rvb_msum = (s_dly >>> 1) + rvb_wet;               // 干声压一半防削顶
    wire signed [15:0] rvb_mix  = (rvb_msum >  20'sd32767) ?  16'sd32767 :
                                  (rvb_msum < -20'sd32768) ? -16'sd32768 :
                                                              rvb_msum[15:0];
'@ -replace "`r`n", "`n"

$L.RemoveRange($i0, $i1 - $i0 + 1)
$L.InsertRange($i0, [string[]]($new -split "`n"))
[System.IO.File]::WriteAllText($p, ($L -join "`r`n"), $u)

$s = $u.GetString([System.IO.File]::ReadAllBytes($p))
Write-Host ("CRLF=" + ([regex]::Matches($s,"`r`n")).Count + " 单独LF=" + ([regex]::Matches($s,"(?<!`r)`n")).Count)
Write-Host ("utf8合法=" + (($u.GetBytes($s) -join ',') -eq (([System.IO.File]::ReadAllBytes($p)) -join ',')))
Write-Host ("begin=" + ([regex]::Matches($s,'\bbegin\b')).Count + " end=" + ([regex]::Matches($s,'\bend\b')).Count + " endmodule=" + ([regex]::Matches($s,'\bendmodule\b')).Count)
Write-Host ("旧的多抽头残留 rvb_fb = " + ([regex]::Matches($s,'rvb_fb')).Count + " / rvb_wr = " + ([regex]::Matches($s,'rvb_wr')).Count)
