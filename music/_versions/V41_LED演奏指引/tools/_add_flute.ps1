$ErrorActionPreference = 'Stop'
$enc = New-Object System.Text.UTF8Encoding($false)
$top = 'D:\FPGA_60K\music\src\top.v'
$log = New-Object System.Collections.ArrayList

# ---------- 先做一遍「全部锚点唯一性」预检，避免改一半 ----------
$edits = @()

$edits += ,@('E1 按钮7 + flute_on 开关', $false, @'
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
    // ★★ 按钮7 = 笛子音色开关（纯新增，其它功能一律不动）★★
    //
    //   关掉时全设计【逐位等于现在】：被它切换的每一个量都写成
    //       flute_on_audio ? 笛子值 : 原值
    //   的三目选择，flute_on=0 时取到的就是原值，逻辑与原来完全一致。
    //
    //   笛子音色靠包络 + 谐波两处做出来（对比"现在"这一档）：
    //     起音   0.65ms  -> 约 30ms   （气息渐强，不再"啪"一下起来）
    //     S 段   255     -> 200       （按住保持 -2dB，略有衰减感）
    //     释音   1.46s   -> 约 250ms  （笛子收得干脆，不留长尾巴）
    //     2 次谐波        ->  减半     （音色更纯、更"空心"）
    //==================================================================
    wire btn_flute = btn_new[6];             // 按钮 7 -> 笛子音色

    reg flute_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)         flute_on <= 1'b0;        // 上电默认【现有音色】
        else if (btn_flute) flute_on <= ~flute_on;   // 按一下换笛子，再按换回来
    end
'@)

$edits += ,@('E2 LED 音色指示补两档', $false, @'
    reg led_out;
    always @(*) begin
        case (timbre_50m)
            1'b0: led_out = led_cnt[23];          // 慢闪 = 纯基波
            1'b1: led_out = led_cnt[21];          // 快闪 = 基波 + 2 次
            default: led_out = led_cnt[23];
        endcase
    end
'@, @'
    reg led_out;
    // ★ 笛子模式补两档花样（原本注释里就写着"慢闪/快闪/双闪/常亮"四档，
    //   这里正好补齐）。flute_on=0 时取的是 2'b00 / 2'b01 两支，
    //   与原来的 case(timbre_50m) 逐位相同 —— 不动原有指示。
    always @(*) begin
        case ({flute_on, timbre_50m})
            2'b00:   led_out = led_cnt[23];       // 慢闪   = 现有音色 / 纯基波
            2'b01:   led_out = led_cnt[21];       // 快闪   = 现有音色 / 基波+2次
            2'b10:   led_out = led_cnt[22];       // 中速闪 = 笛子 / 纯基波
            2'b11:   led_out = 1'b1;              // 常亮   = 笛子 / 基波+2次
            default: led_out = led_cnt[23];
        endcase
    end
'@)

$edits += ,@('E3 笛子开关跨到音频域', $false, @'
    wire trem_on_audio = trem_s2;
'@, @'
    wire trem_on_audio = trem_s2;

    // ---- 笛子音色开关跨到音频域（慢信号，两级同步足够）----
    reg flt_s1, flt_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            flt_s1 <= 1'b0;
            flt_s2 <= 1'b0;
        end else begin
            flt_s1 <= flute_on;
            flt_s2 <= flt_s1;
        end
    end
    wire flute_on_audio = flt_s2;
'@)

$edits += ,@('E4a 笛子包络参数', $false, @'
    localparam [15:0] ENV_RELEASE_DIV = 16'd280;
'@, @'
    localparam [15:0] ENV_RELEASE_DIV = 16'd280;

    //==================================================================
    // ★ 笛子音色（按钮7）的包络参数 —— 只在这里加常数，上面三个
    //   ENV_* 一个都没改，所以【现有音色完全不受影响】。
    //
    //   采样率 48828Hz 下的实际时长：
    //     起音 255/8 = 32 拍 × 46 点 ≈ 1472 点 ≈ 30.1ms
    //     衰减 255→200 = 55 级 × 280 点 ≈ 15400 点 ≈ 315ms
    //     释音 200→0  = 200 级 × 61 点 ≈ 12200 点 ≈ 250ms
    //==================================================================
    localparam [5:0]  ENV_ATK_DIV_FLUTE = 6'd46;    // 起音节拍分频（放慢起音）
    localparam [7:0]  ENV_SUS_LV_FLUTE  = 8'd200;   // 按住保持电平(-2dB)
    localparam [15:0] ENV_DEC_DIV_FLUTE = 16'd280;  // 衰减速度（沿用现值的感）
    localparam [15:0] ENV_REL_DIV_FLUTE = 16'd61;   // 释音节拍分频（收得干脆）

    // 三目选择：flute_on_audio=0 时每个都取"原值"，逐位不变
    wire [5:0]  env_atk_div = flute_on_audio ? ENV_ATK_DIV_FLUTE : 6'd1;
    wire [7:0]  env_sus_lv  = flute_on_audio ? ENV_SUS_LV_FLUTE  : ENV_SUSTAIN_LV;
    wire [15:0] env_dec_div = flute_on_audio ? ENV_DEC_DIV_FLUTE : ENV_DECAY_DIV;
    wire [15:0] env_rel_div = flute_on_audio ? ENV_REL_DIV_FLUTE : ENV_RELEASE_DIV;
'@)

$edits += ,@('E4b dec/rel tick 用模式值', $false, @'
    wire dec_tick = (dec_cnt >= ENV_DECAY_DIV   - 1);
    wire rel_tick = (rel_cnt >= ENV_RELEASE_DIV - 1);
'@, @'
    wire dec_tick = (dec_cnt >= env_dec_div - 1'b1);
    wire rel_tick = (rel_cnt >= env_rel_div - 1'b1);

    // ★ 起音节拍：现有音色 env_atk_div=1 -> att_tick 恒为 1，等于没有这一级
    //   （att_cnt 一直停在 0），所以现有音色的起音【逐位不变】。
    reg  [5:0] att_cnt;
    wire       att_tick = (att_cnt >= env_atk_div - 1'b1);
    always @(posedge clk_audio) begin
        if (rst_audio_sync) att_cnt <= 6'd0;
        else if (sample_en) att_cnt <= att_tick ? 6'd0 : (att_cnt + 1'b1);
    end
'@)

$edits += ,@('E4c 起音按模式放慢', $true, '(?s)                        end else if \(!attacking\) begin\n.*?\n                        end else if \(env\[ge\] > ENV_SUSTAIN_LV\) begin\n', @'
                        end else if (!attacking) begin
                            // Attack：冲到顶，到顶后置标志
                            // ★ 笛子模式用 att_tick 放慢起音；现有音色下
                            //   env_atk_div=1 -> att_tick 恒为 1 -> 逐位不变
                            if (att_tick) begin
                                if (env[ge] >= 8'd255 - ENV_ATTACK_STEP) begin
                                    env[ge]   <= 8'd255;
                                    attacking <= 1'b1;
                                end else
                                    env[ge]   <= env[ge] + ENV_ATTACK_STEP;
                            end
                        end else if (env[ge] > env_sus_lv) begin
'@)

$edits += ,@('E5 笛子谐波减半', $false, @'
    wire signed [22:0] harm_sum   = timbre[0] ? harm_shift[22:0] : 23'sd0;
'@, @'
    wire signed [22:0] harm_raw   = timbre[0] ? harm_shift[22:0] : 23'sd0;
    // ★ 笛子音色：2 次谐波压一半，音色更纯更"空心"
    //   （摇杆 Y 的"力度 -> 谐波"效果照样有效，只是幅度减半）
    //   flute_on_audio=0 时 harm_sum 取的就是 harm_raw，逐位不变。
    wire signed [22:0] harm_sum   = flute_on_audio ? (harm_raw >>> 1) : harm_raw;
'@)

$edits += ,@('E6a 按钮表补按钮7', $false, @'
    //     按钮4(bit3)=音色   按钮5(bit4)=颤音   按钮6(bit5)=延迟
'@, @'
    //     按钮4(bit3)=音色   按钮5(bit4)=颤音   按钮6(bit5)=延迟
    //     按钮7(bit6)=笛子音色
'@)

$edits += ,@('E6b 修正上方写错的按钮分配注释', $false, @'
    //   ★ 当前按钮分配（16 键触摸模块）：
    //       按钮1(bit0)=滑音    按钮2(bit1)=延音
    //       按钮3(bit2)=波形    按钮4(bit3)=颤音
    //       按钮5(bit4)=音色    按钮6(bit5)=延迟
'@, @'
    //   ★ 当前按钮分配（16 键触摸模块）—— 与下面代码逐位对齐：
    //       按钮1(bit0)=滑音    按钮2(bit1)=延音
    //       按钮3(bit2)=波形    按钮4(bit3)=音色
    //       按钮5(bit4)=颤音    按钮6(bit5)=延迟    按钮7(bit6)=笛子音色
'@)

$edits += ,@('E6c 收窄说明改字', $false, @'
    //   ★ 按钮6/7 各管一个效果器，所以必须【一位对一位】，
'@, @'
    //   ★ 按钮5/6/7 各管一个开关，所以必须【一位对一位】，
'@)

# ---------- 文件现在是纯 CRLF，统一转成 LF 处理，最后再转回去 ----------
$raw = [System.IO.File]::ReadAllText($top, $enc)
$eol = if ([regex]::Matches($raw, "`r`n").Count -gt 0 -and [regex]::Matches($raw, "(?<!`r)`n").Count -eq 0) { "`r`n" } else { "`n" }
Write-Host ("文件行尾 = " + $(if ($eol -eq "`r`n") { 'CRLF' } else { 'LF' }))
$t = [regex]::Replace($raw, "`r`n", "`n")

# ---------- 预检 ----------
$bad = 0
foreach ($e in $edits) {
    $pat = if ($e[1]) { $e[2] } else { [regex]::Escape($e[2]) }
    $n = ([regex]::Matches($t, $pat)).Count
    if ($n -ne 1) { $bad++; [void]$log.Add("PRE-FAIL $($e[0])  matches=$n") }
}
if ($bad -gt 0) { $log | ForEach-Object { Write-Host $_ }; throw "anchors not unique, nothing written" }
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
