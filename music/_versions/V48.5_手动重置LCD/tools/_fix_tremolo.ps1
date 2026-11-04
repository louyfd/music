$ErrorActionPreference = 'Stop'
$enc = New-Object System.Text.UTF8Encoding($false)
$log = New-Object System.Collections.ArrayList

function Edit([string]$path, [string]$name, [string]$old, [string]$new) {
    $t = [System.IO.File]::ReadAllText($path, $enc)
    if ($enc.GetBytes($t) -join ',' -ne ([System.IO.File]::ReadAllBytes($path) -join ',')) { throw "encoding not clean UTF-8: $path" }
    $old = [regex]::Replace($old, "`r`n", "`n"); $new = [regex]::Replace($new, "`r`n", "`n")
    $n = ([regex]::Matches($t, [regex]::Escape($old))).Count
    if ($n -ne 1) { [void]$log.Add("FAIL  $name (matches=$n)"); return }
    [System.IO.File]::WriteAllText($path, $t.Replace($old, $new), $enc)
    [void]$log.Add("ok    $name")
}

$top = 'D:\FPGA_60K\music\src\top.v'

# ===== 1) 根因：乘法位宽被截断（颤音因此恒等于没开）=====
Edit $top '1) trem_tgt 位宽（根因）' @'
    // 目标衰减量 = tri/64 × DEPTH（tri 只有 6 位 × 常数，很便宜）
    wire [7:0] trem_tgt = (trem_tri * TREM_DEPTH) >> 6;
'@ @'
    // 目标衰减量 = tri/64 × DEPTH
    //  ★★ 这里原来有个位宽陷阱（颤音听不出来的根因）★★
    //     Verilog 里 a*b 的结果宽度 = max(操作数宽度) 再被赋值左边卡住：
    //     trem_tri(6位) × 110 最大 6930（要 13 位），但直接写
    //         wire [7:0] trem_tgt = (trem_tri * TREM_DEPTH) >> 6;
    //     乘法是在 8 位上下文里做的 → 6930 被截成 18 → >>6 恒为 0/1
    //     → vol_dyn 只会在 220/219 之间抖 0.5% → 完全听不出来。
    //     必须先把乘积落到够宽的中间 wire 上再移位。
    wire [13:0] trem_prod = trem_tri * TREM_DEPTH;   // 14 位上下文，乘积不丢
    wire [7:0]  trem_tgt  = trem_prod >> 6;
'@

# ===== 2) 按钮容错：按钮5~16 任意一个空按钮都算（丝印和 bit 的对应关系先不管）=====
Edit $top '2) 按钮容错 5~16' @'
    wire btn_trem = btn_new[4];              // 按钮 5
'@ @'
    // ★ 容错：bit4~bit15 全是【没用到的】按钮，所以任意一颗空按钮都能开关颤音。
    //   这样做是因为 16 键模块丝印上的"第几个触摸区"和 DATA 的哪一位
    //   并不一定有把握一一对上；等确认了按钮5 到底是哪一位，
    //   把这行收窄成 btn_new[4] 即可（功能不变，只是别的空按钮不再触发）。
    wire btn_trem = |btn_new[15:4];
'@

# ===== 3) LED 直接指示颤音状态，按一下就能看出按钮有没有被读到 =====
Edit $top '3) led_glide 指示颤音' @'
    assign led_glide = LED_ACTIVE_LOW ? ~(glide_en | sustain_on)
                                      :  (glide_en | sustain_on);
'@ @'
    //   ★ 顺手把颤音状态并进这颗灯：按一下【按钮5】灯就变，
    //     用来确认"按钮到底读到没有"（不想要就把 | trem_on 去掉）。
    assign led_glide = LED_ACTIVE_LOW ? ~(glide_en | sustain_on | trem_on)
                                      :  (glide_en | sustain_on | trem_on);
'@

# ===== 4) ttp229_reader：raw_d 写死 12 位 → 16 键那片的 bit12~15 永远读不到 =====
Edit 'D:\FPGA_60K\music\src\ttp229_reader.v' '4) raw_d 位宽参数化' @'
    reg  [11:0] raw_d;          // ★ 上一轮读数的原始值（抗干扰滤波用）
'@ @'
    // ★ 位宽必须跟着 KEY_BITS 走：原来写死 12 位，16 键那片实例的
    //   raw_d 只有 12 位（综合日志 WARN EX3791: 16 截成 12），
    //   于是 bit12~15 的"两轮一致"滤波里 raw_d 恒为 0，
    //   key_stable 的 bit12~15 永远不可能变成 1 —— 按钮13~16 直接是死的。
    reg  [KEY_BITS-1:0] raw_d;  // ★ 上一轮读数的原始值（抗干扰滤波用）
'@

$log | ForEach-Object { Write-Host $_ }
