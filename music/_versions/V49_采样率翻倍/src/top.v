`timescale 1ns / 1ps
//=============================================================================
// top.v  (Gowin GW5A-25：多片 TTP229 电子琴 + ADS1115 四通道采集)
module top (
    input  wire        clk_50m,
    input  wire        rst_n,

    // ---- TTP229：每片一组 scl/sdo ----
    output wire        scl,        // 琴键1
    inout  wire        sdo,
    output wire        scl2,       // 琴键2
    inout  wire        sdo2,
    // ★PAD3-1  第 3 片键盘（低八度）：Console 60K 上 SCL = A9，SDO = A11
    output wire        scl3,       // 琴键3
    inout  wire        sdo3,
    // ---- TTP229（16键，当按钮用）----
    output wire        scl_btn,
    inout  wire        sdo_btn,
    // ---- I2S ----
    output wire        bck,
    output wire        lck,
    output wire        din,

    // ---- LED ----
    output wire        led,          // U12(=READY ball, 复用为 GPIO)：音色指示
    output wire        led_glide,    // G11(=DONE  ball, 复用为 GPIO)：滑音/延音指示

    // ---- ★★ V41：LED 演奏指引（IS31FL3236A，走【第二条】I2C）★★ ----
    //   ★ 和下面 ADS1115 的 i2c_scl/i2c_sda 是两条【完全独立】的总线，别搞混：
    //       ADS1115（摇杆）: i2c_scl = G22 / i2c_sda = V19
    //       LED 指引        : led_scl = V17 / led_sda = W17  ← 本次新增
    //   ★ 灯怎么亮、为什么放在音频域，见 src\led_guide.v 的文件头。
    output wire        led_scl,
    inout  wire        led_sda,

    // ---- ADS1115 (I2C) ----
    //   摇杆 X/Y 就接在它的 A0/A1 上
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    // ---- Bluetooth HC-05/06 (UART) ----
    input  wire        uart_rxd,     // BT TXD
    output wire        uart_txd,     // BT RXD

    // ---- ★★ V44：延迟测量测试脚（外部示波器用）★★ ----
    //   CH1 -> lat_key (C19)：琴键端 —— FPGA 检测到"有键刚按下"的那一拍
    //   CH2 -> lat_pcm (C18)：PCM 端 —— 本帧要送 DAC 的样本【由 0 变非 0】那一拍
    //   CH3 -> 喇叭/DAC 模拟输出（不用加脚，探头夹上去就行）
    //   ★ 用法三通道一起看：CH1↑ → CH2↑ → CH3 起波形，
    //     一张图就把"触摸扫描 / FPGA 内部 / 串帧+DAC"三段分开。
    //   ★ 两者都是【上升沿准确、宽度约 1µs 的脉冲】——
    //     原始事件只有一拍（20ns/80ns），很多示波器带宽不够会看成毛刺甚至抓不到，
    //     所以拉宽到 1µs；★ 拉宽只延长【后沿】，上升沿仍在事件发生那一拍。
    output wire        lat_key,
    output wire        lat_pcm,

    // ---- SPI LCD (ST7789 320x240) ----
    output wire        lcd_sck,
    output wire        lcd_mosi,
    output wire        lcd_dc,
    output wire        lcd_cs,
    output wire        lcd_blk
);

    //==================================================================
    // ★PAD3-2  键数：3 片 = 36（第 3 片已加入，低八度）
    //==================================================================
    localparam NUM_KEYS = 36;

    //==================================================================
    // 摇杆模块（ADS1115 双通道：X 轴 / Y 轴）
    //
    //   A0 / AIN0 → X 轴 → 移调：整台琴升降半音，每边最多 JS_STEPS 个
    //   A1 / AIN1 → Y 轴 → 总音量：拉到底静音，推到头 = 原来的正常响度
    //   ★ 原压力传感器（力度）已整体移除，谐波比例改回由音色档位决定。
    //==================================================================
    localparam [15:0] JS_CENTER = 16'd13200;   // 摇杆居中时的读数
    // ★ V17：死区 2600 -> 1200，行程 11000 -> 12500。
    //
    //   V16 把死区加到 2600 是为了压住"静止降半音"，但代价是【太钝】：
    //   有效无移调窗口 = ±(2600 + 32768/JS_XMUL) ≈ ±3000 counts，
    //   占半行程 13200 的 23% —— 摇杆要推掉近 1/4 才开始换档，
    //   而且行程 11000 只有物理半行程的 83%，推到头一大段是浪费的。
    //
    //   实测（ADS1117 工程挂 VOFA+ 看原始读数）：静止时 I0=13179、I1=13002，
    //   相对 JS_CENTER=13200 只偏 21 / 198 个 count，根本不需要 2600 的死区。
    //
    //   现在：有效无移调窗口 = ±(1200 + 32768/70) ≈ ±1668 counts（13%），
    //         满 12 个半音发生在 |偏移| ≈ 12435 counts（物理半行程的 94%）。
    //   ★ 如果换摇杆后静止读数偏得多（>1600），先改 JS_CENTER，别再加死区。
    localparam [15:0] JS_DEAD   = 16'd1200;    // 中心死区（counts），防静止跑调
    localparam [15:0] JS_TRAVEL = 16'd12500;   // 从中位推到极端的行程
    localparam integer JS_STEPS = 12;          // 每边最多移几个半音

    localparam [31:0] JS_XSPAN = JS_TRAVEL - JS_DEAD;                             // 11300
    localparam [31:0] JS_XMUL  = (JS_STEPS * 32'd65536 + JS_XSPAN/2) / JS_XSPAN;  // 70
    localparam [31:0] JS_VMUL  = (32'd255 * 32'd65536 + 32'd13200) / 32'd26400;   // 633


    //==================================================================
    // Gowin PLLï¼š50MHz -> 12.5MHz
    //==================================================================
    wire clk_audio;
    wire clk_50m_ext;      // PLL CLKOUT1 = 50MHz : Bluetooth UART + SPI LCD

    Gowin_PLL u_pll (
        .clkin   (clk_50m),
        .clkout0 (clk_audio),      // 12.5MHz : audio domain (unchanged)
        .clkout1 (clk_50m_ext),    // 50MHz   : BT + LCD
        .mdclk   (1'b0),
        .reset   (1'b0)            // 60k 版 IP 多了这个端口，接 0（= 原工程 I_RST 接地）
    );
    //==================================================================
    // ★ TTP229 当按钮：第 4 片（16 键模式）
    //   键1 -> 滑音开关（代替 AA13）
    //   键2 -> 延音开关（代替 AB13）
    //   键3 -> 显示画面循环：频谱 -> 波形 -> 李萨如(X-Y) -> 相位李萨如 -> 频谱
    //   键4 -> 音色切换
    //   键5 -> 颤音（Tremolo）开关
    //   键6 -> 延迟（Delay / Echo）开关
    //   键7~16 暂时不用（原来留了键12 给李萨如单开，后来改成并进键3 了）
    //
    //   ★ 这片用 16 位；三片琴键继续用默认 12 位，互不影响。
    //   ★ key_stable 是【正逻辑】：1 = 按下。
    //==================================================================
    wire [15:0] btn_keys;

    ttp229_reader #(.KEY_BITS(16)) u_btn (
        .clk         (clk_50m),
        .rst_n       (rst_n),
        .scl         (scl_btn),
        .sdo         (sdo_btn),
        .key_stable  (btn_keys),
        .rdy_toggle  ()
    );

    // 按下沿检测（key_stable 内部已有 2/2 去抖，这里只做边沿）
    reg [15:0] btn_keys_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) btn_keys_d <= 16'd0;
        else        btn_keys_d <= btn_keys;
    end
    wire [15:0] btn_new = btn_keys & ~btn_keys_d;   // 单拍脉冲

    wire btn_glide   = btn_new[0];   // 键 1
    wire btn_sustain = btn_new[1];   // 键 2
    // ★ 键 3 -> 波形切换
    wire btn_wave    = btn_new[2];
    //=========================================================================
    // ★★ V19 更正：按钮分配【保持原样】，不再改动 ★★
    //
    //   原来这里有一段注释说"按钮4(bit3) 实测无反应，改用按钮5"，
    //   那条注释【是错的，已删除】：
    //     · 音色一直挂在 btn_new[3]（按钮4）上，你实际能用 -> 按钮4 正常
    //     · 你说按钮全部灵敏且功能正确 -> 不用换键
    //   所以按位分配维持原样，只是把误导性的注释清掉了。
    //
    //     按钮1(bit0)=滑音   按钮2(bit1)=延音   按钮3(bit2)=波形
    //     按钮4(bit3)=音色   按钮5(bit4)=颤音   按钮6(bit5)=延迟
    //     按钮7(bit6)=笛子音色  按钮8(bit7)=混响  按钮9(bit8)=合唱
    //=========================================================================
    wire btn_timbre  = btn_new[3];   // 按钮 4 -> 音色切换（原有，未动）
    wire btn_trem    = btn_new[4];   // 按钮 5 -> 颤音（原有，未动）
    //==================================================================

    //==================================================================
    // ★★ V31：效果开关 / 音色的【上位机写口】★★
    //
    //   要求：软件（蓝牙串口助手）里也能切这 8 个效果和音色，
    //         板子上的 TTP 按钮【继续保留】—— 两边谁都能动，状态互相跟着走。
    //
    //   做法和 V29 的按键映射写口完全一样：UART 侧(clk_50m_ext)出一个
    //   "翻转信号 + 稳定数据"，这边(clk_50m)用边沿检测接住：
    //       fx_tgl 翻转  ->  fx_bit 指出翻转 fx_state 的第几位
    //       tone_we_tgl 翻转 -> tone_val 是目标音色 0/1/2
    //   ★ 数据在翻转沿之前就已经稳定（UART 侧先写数据后翻沿），
    //     所以这里两级同步数据、再用沿去采，采到的一定是配套的那份。
    //
    //   ★ 为什么效果开关用"翻转"而不是"直接设为 1/0"：
    //     翻转的语义和按物理按钮一模一样，板子永远是唯一权威 ——
    //     上位机不需要知道当前状态就能安全地切，也就不会出现
    //     "软件以为它是关的、其实是开的，一设反而设反了"。
    //   ★ 唯一的例外是音色：三档没法用一位翻转表达，所以 0xCA 是"直接选档"。
    //==================================================================
    //   ★ 这四个是 uart_packet_echo 的输出（写口），先声明再用
    wire        fx_tgl;
    wire [2:0]  fx_bit;
    wire        tone_we_tgl;
    wire [1:0]  tone_val;
    //   ★ V32：当前正在播放那一格的八度偏移（有符号 3 位，来自 uart_packet_echo）
    wire [2:0]  oct_cur;
    //   ★★ V48：LCD 显示模式的上位机写口（0xD0）—— 也是 uart_packet_echo 的输出，
    //      用在下面 disp_mode 那个 always 块里，所以必须【在这里】先声明。
    //      （先用后声明会建出隐式 1 位线，综合不报错、行为却是错的 —— 坑 32。）
    //      ★ V48.2：从 2 位加宽到 3 位（显示模式现在是 5 档 0~4，不再折叠）。
    wire        disp_tgl;
    wire [2:0]  disp_val;
    // ★★ V48.5：软件"手动重置 LCD"（0xD1）的请求沿。
    //   ★ 这条**不是跨时钟域**：uart_packet_echo 和 lcd_master 都在
    //     clk_50m_ext 域（见下面两处例化的 .clk/.sys_clk），所以直接连一根线，
    //     不需要握手/同步器那一套。lcd_master 内部打两拍 + 边沿检测。
    wire        lcd_rst_tgl;

    reg        fxw_s1, fxw_s2;
    reg [2:0]  fxw_bit_s1, fxw_bit_s2;
    reg        tonw_s1, tonw_s2;
    reg [1:0]  tonw_val_s1, tonw_val_s2;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            fxw_s1 <= 1'b0; fxw_s2 <= 1'b0;
            fxw_bit_s1 <= 3'd0; fxw_bit_s2 <= 3'd0;
            tonw_s1 <= 1'b0; tonw_s2 <= 1'b0;
            tonw_val_s1 <= 2'd0; tonw_val_s2 <= 2'd0;
        end else begin
            fxw_s1 <= fx_tgl;      fxw_s2 <= fxw_s1;
            fxw_bit_s1 <= fx_bit;  fxw_bit_s2 <= fxw_bit_s1;
            tonw_s1 <= tone_we_tgl; tonw_s2 <= tonw_s1;
            tonw_val_s1 <= tone_val; tonw_val_s2 <= tonw_val_s1;
        end
    end
    wire fxw_we   = fxw_s1  ^ fxw_s2;     // 翻转沿 = 收到一条 0xC9
    wire tonw_we  = tonw_s1 ^ tonw_s2;    // 翻转沿 = 收到一条 0xCA

    //   位号 -> 每一位是不是"软件要翻它"（和 fx_state 的位定义一一对应）
    wire fxw_glide   = fxw_we && (fxw_bit_s2 == 3'd0);
    wire fxw_sustain = fxw_we && (fxw_bit_s2 == 3'd1);
    wire fxw_timbre  = fxw_we && (fxw_bit_s2 == 3'd2);
    wire fxw_trem    = fxw_we && (fxw_bit_s2 == 3'd3);
    wire fxw_dly     = fxw_we && (fxw_bit_s2 == 3'd4);
    wire fxw_tonecyc = fxw_we && (fxw_bit_s2 == 3'd5);   // 音色循环（= 按按钮 7）
    wire fxw_rev     = fxw_we && (fxw_bit_s2 == 3'd6);
    wire fxw_cho     = fxw_we && (fxw_bit_s2 == 3'd7);

    //==================================================================
    // ★ 只认 TTP229 键2 + 上位机的第 1 位（延音）
    reg sustain_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                          sustain_on <= 1'b0;
        else if (btn_sustain | fxw_sustain)  sustain_on <= ~sustain_on;
    end

    // ★ 只认 TTP229 键3 + 上位机的第 3 位（音色档）
    wire timbre_cycle = btn_timbre | fxw_timbre;

    //   板载按键是【瞬时按键】，按一下翻转一次，不再需要长按 2 秒。
    //   ★ 极性：AA13 / AB13 是【板载按键】：bank9 / 1.5V / 内部上拉 / 按下 = 低电平。
    //   led_glide 直接指示 glide_en（亮 = 滑音开）。
    // ★ 只认 TTP229 键1 + 上位机的第 0 位（滑音）
    wire glide_toggle = btn_glide | fxw_glide;

    reg glide_en;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)            glide_en <= 1'b0;       // 上电默认【滑音关】，按 AA13 打开
        else if (glide_toggle) glide_en <= ~glide_en;
    end

    reg [0:0] timbre_50m;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            timbre_50m <= 1'b0;
        end else if (timbre_cycle) begin
            timbre_50m <= ~timbre_50m;      // 0 <-> 1
        end
    end
    //==================================================================
    // ★ TTP229 键4：频谱 / 波形 切换
    //
    //   那块 16 键按钮板是【宏维微电 HW-136 / TTP229-BSF 16路触摸模块】，
    //   它自己的 Arduino 例程 key_16out.ino 里是这么编号的：
    //        DATA |= digitalRead(SDO) << i;         // 先出 bit0
    //        if ((h & (1 << i)) == 0) 按键 (i+1)    // 注意例程把这模块的读数取反用
    //   本工程 ttp229_reader 里已经写成 key = ~DATA，所以
    //        btn_keys[n] = 模块丝印上的第 (n+1) 个触摸区
    //   即 bit0 = 按钮1，bit1 = 按钮2，bit2 = 按钮3，bit3 = 按钮4。
    //   ★ 模块必须在 16 键模式：板上 P1 那排针里【数字 3 处的两个排针要短接】
    //     （见 D:\电容按键16\A1-6-3L\例程源码\arduino例程\接线说明.txt），
    //     不短接默认是 8 键模式，只有 bit0~bit7 有效。
    //     现在颤音/音色已对调，分配见上面 btn_trem / btn_timbre 那一段。

    //==================================================================
    // ★★ 按钮 3 = 显示画面循环 ★★
    //
    //   V35 是 4 档（频谱 / 波形 / 李萨如·干声×效果后 / 李萨如·相位），
    //   V36 因为输出级真的变成双声道了，在【第二位】插进一档"左×右"，
    //   现在是 5 档循环，按一下换一个画面：
    //
    //       频谱 → 波形 → 李萨如(左×右) → 李萨如(干声×效果后) → 李萨如(相位) → 循环
    //
    //   ★ 关键是【wave_mode 的原语义一个字没变】：它仍然只表示
    //       0 = 频谱、1 = 波形
    //     李萨如是另一根独立的线 liss_on，两个都送进 lcd_master，
    //     由 lcd_master 决定"liss_on 优先"。这样频谱/波形那两屏的状态机、
    //     常数、刷新节奏全都不用碰。
    //   ★ 三屏李萨如共用 lcd_master 里的同一套显存/打点/余辉/边框/十字，
    //     只是 XY 来源不同，用 liss_src 选（V36 从 1 位扩成 2 位）：
    //       liss_src = 0 -> X = L（左声道输出）、Y = R（右声道输出）
    //       liss_src = 1 -> X = 干声 dry_sample、Y = 输出 sample_vol（原档 2）
    //       liss_src = 2 -> X = 输出 sample_vol、Y = 输出经一阶全通移相 90°（原档 3）
    //     ★ liss_src 的编码就取"disp_mode 减 2"，三档天然是 0/1/2，不会错位。
    //   ★ 为什么"左×右"档要存在：板子上 DAC 两个声道都接了喇叭，这一档是
    //     唯一能【直接看见立体声分离度】的画面：开合唱/延迟/混响后图形张
    //     开成环/花，全关时 L≡R 所以画的是一条对角线 —— ★ 那是【正确】的，
    //     说明此刻确实没有立体声信息（mono 内容 = 左右完全相同），不是 bug。
    //   ★ 为什么原"档 2"要另做一屏：效果全关时 dry_sample 和 sample_vol
    //     逐位相同，那一档画的也是一条对角线，不像教科书上的李萨如；
    //     相位档让两轴差 90°，单音才画成圆/椭圆那样的闭合环。
    //   ★ 派生位各自打一拍再送出去：跨到 LCD 域 (clk_50m_ext) 的是寄存器
    //     输出，不是比较器的组合输出（组合输出在切换那一拍可能有窄毛刺，
    //     被 50MHz 那侧采到就成了"两屏同时被选中"）。
    //==================================================================
    reg [2:0] disp_mode;      // 0=频谱 1=波形 2=左×右 3=干声×输出 4=相位

    //==================================================================
    // ★★ V48：显示模式的【上位机写口】（clk_50m_ext -> clk_50m）★★
    //
    //   用户要求："所有 TTP 上的可调按键全部在上位上也可调，并且状态保持同步"。
    //   TTP 键3 = 显示画面循环，原来只有板子这一条路，现在上位机发 0xD0
    //   也能【直接设定】到某一档。
    //
    //   ★ 为什么用"翻转沿 + 数据握手"而不是直接两级同步 disp_val：
    //     disp_mode 是 3 位、而且 1->2 这种相邻档会同时翻两位 ——
    //     直接同步有可能采到"中间态"（1->2 可能瞬时读成 3），
    //     那会让上位机看到一帧假的显示模式（0x5F 只在变化时发，
    //     发出去就收不回来了）。握手能保证采到的一定是稳定值。
    //   ★ disp_val 已经在 uart_packet_echo 里夹到 0~4 了（和上位机 clamp_disp
    //     同一套规则），所以它本身就是合法的 3 位档号。
    //   ★★ 关键：`disp_mode` 全文件仍然只有【这一个】always 块驱动 ★★
    //     （两个块驱动同一个 reg 会报 EX2000 —— 坑 1）。
    //     按键那条路一个比特都没改，只是多了一条优先级更高的写入口。
    //==================================================================
    reg dsp_s1, dsp_s2, dsp_s3;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) {dsp_s3, dsp_s2, dsp_s1} <= 3'b000;
        else        {dsp_s3, dsp_s2, dsp_s1} <= {dsp_s2, dsp_s1, disp_tgl};
    end
    wire dsp_we = dsp_s2 ^ dsp_s3;      // 翻转沿 = 收到一条 0xD0

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        disp_mode <= 3'd0;       // 上电默认频谱
        // ★ V48：上位机的设定优先（同拍撞上按键时以命令为准，人手那一下
        //   下一拍还能再按）。数据在沿到来前早就稳定了，直接锁存。
        //   ★ V48.2：disp_val 现在就是 3 位档号本身（5 档 0~4），不用再补位。
        else if (dsp_we)   disp_mode <= disp_val;
        // ★ 5 档循环：判据必须是 ">= 3'd4"（最后一档）而不是 "== 3'd3"，
        //   漏改这一处的话最后一档会一直加下去，计数器自己回绕成 0 ——
        //   表面看也是"循环"，实际是计数器溢出，中间会闪一下别的档。
        else if (btn_wave) disp_mode <= (disp_mode >= 3'd4) ? 3'd0 : (disp_mode + 3'd1);
    end

    reg [1:0] liss_src;       // 0=左×右 1=干声×输出 2=相位（见上面那张表）
    reg wave_mode;            // 给 lcd_master 的原来那一位：0=频谱 1=波形
    reg liss_on;              // 给 lcd_master 的新一位：1=画李萨如（档 2/3/4 都为 1）

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            wave_mode <= 1'b0;
            liss_on   <= 1'b0;
            liss_src  <= 2'd0;
        end else begin
            wave_mode <= (disp_mode == 3'd1);
            liss_on   <= (disp_mode >= 3'd2);
            // ★ liss_src = disp_mode - 2（2->0、3->1、4->2）。
            //   频谱/波形那两档 liss_on=0，liss_src 取什么都不参与显示，给 0。
            liss_src  <= (disp_mode >= 3'd2) ? (disp_mode[1:0] - 2'd2) : 2'd0;
        end
    end

    //==================================================================
    // ★★ V48：显示模式的【状态回报口】（clk_50m -> clk_50m_ext）★★
    //
    //   uart_packet_echo 要用它发 0x5F 帧（"板子现在是哪一档"）。
    //   ★★★ V48.2：**不再做 5 档 -> 3 档的折叠**，直接原样报 0~4 ★★★
    //       0 频谱 / 1 波形 / 2 李萨如·左×右 / 3 李萨如·干声×效果后 / 4 李萨如·相位
    //     用户指出"李萨如图形有三档" —— 上位机要能直接选这三档、也要能看出
    //     板子现在在哪一档。所以上面那两句曾经的映射（2/3/4 一律报 2）已经删掉：
    //     `dsp_rep` 现在就是 disp_mode 本身。
    //   ★ 为什么不用"两级同步器直接过 3 位"：和上面写口同一个理由 ——
    //     相邻档会同时翻多位，直接同步可能采到中间态（1->2 采成 3），
    //     而 0x5F 是"变了就发"，发出去就撤不回来了。
    //     这里用同样的"翻转沿 + 数据握手"：数据先稳、沿后到，
    //     接收侧在沿上锁存，采到的一定是配对的稳定值（坑 2 的纪律）。
    //     ★ 5 档有 3 位，握手反而更重要（1->2 是 001->010，两位同时翻）。
    //==================================================================
    wire [2:0] dsp_rep = disp_mode;      // ★ V48.2：原样上报，不折叠
    reg [2:0]  dsp_rep_lat;      // 要送到 UART 域的档号（锁存值）
    reg        dsp_rep_tgl;      // 它的"翻转沿"
    reg [2:0]  dsp_rep_d;        // 上一次已经握手过的档号（比它来判"变了没"）

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            dsp_rep_lat <= 3'd0;
            dsp_rep_tgl <= 1'b0;
            dsp_rep_d   <= 3'd0;
        end else begin
            dsp_rep_d <= dsp_rep;
            // ★ 只在【真的变了】时握手一次（0x5F 只在值变化时发，不能每拍都发）。
            //   ★ V48.2：5 档现在各不相同，所以 2->3->4 之间的切换
            //     **也会各发一帧** —— 这正是用户要的。
            if (dsp_rep != dsp_rep_d) begin
                dsp_rep_lat <= dsp_rep;
                dsp_rep_tgl <= ~dsp_rep_tgl;
            end
        end
    end

    //   clk_50m_ext 侧：三级同步器看到沿之后，锁存早就在手边的稳定数据
    reg dsp_t_s1, dsp_t_s2, dsp_t_s3;
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) {dsp_t_s3, dsp_t_s2, dsp_t_s1} <= 3'b000;
        else        {dsp_t_s3, dsp_t_s2, dsp_t_s1} <= {dsp_t_s2, dsp_t_s1, dsp_rep_tgl};
    end

    reg [2:0] disp_mode_rep;     // 送给 uart_packet_echo 的"当前显示模式"（0~4）
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n)                        disp_mode_rep <= 3'd0;
        else if (dsp_t_s2 ^ dsp_t_s3)      disp_mode_rep <= dsp_rep_lat;
    end

    //==================================================================
    // ★★ 颤音（Tremolo）效果器开关 ★★
    //
    //   ★ 当前按钮分配（16 键触摸模块）—— 与下面代码逐位对齐：
    //       按钮1(bit0)=滑音    按钮2(bit1)=sustain
    //       按钮3(bit2)=显示画面循环(频谱/波形/左×右/干声×效果后/相位)  按钮4(bit3)=音色
    //       按钮5(bit4)=颤音    按钮6(bit5)=relay    按钮7(bit6)=笛子音色      按钮8(bit7)=混响    按钮9(bit8)=合唱
    //
    //   这里只做"按一下翻转一次"；效果器本体在音频域，
    //   开关状态用两级同步器送过去（慢信号，2 级足够）。
    //==================================================================
    //   ★ 按钮5/6/7 各管一个开关，所以必须【一位对一位】，
    //     不能像更早那版用 |btn_new[15:4] 兜底（那样按6 会开成颤音）。
    wire btn_dly  = btn_new[5];              // 按钮 6 -> 延迟

    reg trem_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                       trem_on <= 1'b0;    // 上电默认【关】
        else if (btn_trem | fxw_trem)     trem_on <= ~trem_on; // 按一下开，再按一下关
    end

    reg dly_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                      dly_on <= 1'b0;      // 上电默认【关】
        else if (btn_dly | fxw_dly)      dly_on <= ~dly_on;   // 按一下开，再按一下关
    end

    //==================================================================
    // ★★ 按钮8 = 混响（Reverb）  按钮9 = 合唱（Chorus）★★
    //   两个都是【纯新增】：关掉时整条音频链路逐位等于现在，其它功能一律不动。
    //==================================================================
    wire btn_rev = btn_new[7];               // 按钮 8 -> 混响
    wire btn_cho = btn_new[8];               // 按钮 9 -> 合唱

    reg rev_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                  rev_on <= 1'b0;         // 上电默认【关】
        else if (btn_rev | fxw_rev)  rev_on <= ~rev_on;
    end

    reg cho_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                  cho_on <= 1'b0;         // 上电默认【关】
        else if (btn_cho | fxw_cho)  cho_on <= ~cho_on;
    end

    //==================================================================
    // ★★ 按钮7 = 笛子音色开关（纯新增，其它功能一律不动）★★
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
    wire btn_flute = btn_new[6];             // 按钮 7 -> 音色循环：普通 -> 笛子 -> 吉他 -> 普通

    //==================================================================
    // ★★ V30：音色从"两档"改成"三档"；★★ V48.2：再扩成【四档】★★
    //   原来 flute_on 是一位开关（普通 / 笛子），V30 换成 2 位 tone_sel：
    //       0 = 普通（纯正弦 + 少量 2 次谐波，原来的默认）
    //       1 = 笛子（起音放慢、保持电平低一点、收得干脆、2 次谐波减半）
    //       2 = 吉他（Karplus-Strong 拨弦模型，见文件末尾的 ks_string 模块）
    //       3 = ★ 小号（V48.2 新增：起音快、**保持电平满**、收得比普通干脆，
    //                  而且【固定】垫一份 2 次谐波 —— 铜管那种一直亮着的质感）
    //   ★ 2 位 tone_sel 本来就装得下 4 档，所以【没有新增任何位宽】，
    //     协议、0x5D 状态帧的音色字节、LED/LCD 指示全都只是值域 0~2 -> 0~3。
    //   ★ flute_on / guitar_on / trumpet_on 由它译码出来，所以后面所有
    //     "看音色办事"的地方（包络、谐波）写法都不用改，只是多了一支。
    //   ★ 按钮还是原来那个按钮（TTP 按钮 7），按一下往下循环一格。
    //==================================================================
    reg [1:0] tone_sel;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)          tone_sel <= 2'd0;       // 上电默认【普通音色】
        // ★ V48.2：循环上限从 2 改成 3（最后一档）—— 和 disp_mode 那条同一个坑：
        //   判据必须是"== 最后一档"，写成 2 的话第 4 档会一直加下去、自己回绕
        else if (fxw_tonecyc) tone_sel <= (tone_sel == 2'd3) ? 2'd0 : (tone_sel + 2'd1);
        else if (btn_flute)  tone_sel <= (tone_sel == 2'd3) ? 2'd0 : (tone_sel + 2'd1);
        // ★ V31：上位机 0xCA 直接选档（多档没法用一位翻转表达）
        else if (tonw_we)    tone_sel <= tonw_val_s2;
    end
    wire flute_on   = (tone_sel == 2'd1);
    wire guitar_on  = (tone_sel == 2'd2);
    wire trumpet_on = (tone_sel == 2'd3);      // ★ V48.2
    //------------------------------------------------------------------
    // ★★ V45：原来这里的"LED 指示当前音色（慢闪/快闪/中速闪/常亮）"
    //   已经【整块删掉】★★
    //   · 用户要求把 READY 灯(U12) 改成【节拍器】，音色指示明确不要了
    //     （音色档位在 LCD 上本来就看得到）。
    //   · 删掉的是 led_cnt[23:0] 计数器和那个 case(tone_sel) 的 led_out，
    //     两者全文【只在这里用】（已 grep 确认），删掉不影响任何别的逻辑。
    //   · 节拍器本体见下面"V45 节拍器"那一段（clk_50m 域）。
    //------------------------------------------------------------------

    //------------------------------------------------------------------
    // ★★ V16：led / led_glide 直接复用板上 DONE / READY 两个球 ★★
    //   用户明确要求：不用管板上那套"配置状态"指示电路，直接把这两个球当 LED 用。
    //     G11 = DONE   -> led_glide（滑音/延音指示）
    //     U12 = READY  -> led
    //   代价与前提：
    //     1) 必须让工具把 DONE / READY 放开成普通 IO，见 rebuild/build 脚本：
    //          set_option -use_done_as_gpio  1
    //          set_option -use_ready_as_gpio 1
    //        写 0（保持专用脚）会直接报
    //          ERROR (PR2017) ... the location is a dedicated pin (DONE/READY)
    //     2) 放开之后，板上那颗由 DONE/READY 驱动的配置指示就不再反映配置状态了。
    //   极性：板上是 LED + 限流电阻接法，若实测"亮灭反了"，把下面
    //         LED_ACTIVE_LOW 取反即可（改动只在这一处）。
    //        （25k 原板的 LED 是高电平点亮。）
    //------------------------------------------------------------------
    localparam LED_ACTIVE_LOW = 1'b1;      // Console 60K = 1（低电平点亮）

    // ★★ V45：LED(U12/READY) 从"音色指示"改成【节拍器】★★
    //   用户要求：这颗灯以后按设定的 BPM 匀速闪（每拍闪得一样长，
    //   不区分强拍弱拍）。音色指示【取消】—— 音色在 LCD 上本来就看得到。
    //   ★ metro_led 由下面"V45 节拍器"那一段（clk_50m 域）驱动；
    //     这里先声明，是为了不让这个 assign 变成"先用后声明"。
    //   ★ 极性只在这一个 assign 里处理。
    wire metro_led;
    // ★ V45：节拍器相关的线网，统一在这里先声明 ——
    //   下面 assign led / uart_packet_echo 例化 / 节拍器本体 三处都要用，
    //   不提前声明就会出现"先用后声明"（工具会建出隐式 1 位线）。
    wire [7:0] metro_bpm, metro_num, metro_den;
    wire       metro_tgl;          // clk_50m_ext -> clk_50m 的"翻转沿 + 数据握手"
    wire [2:0] metro_beat_50m;     // clk_50m 域内部的小节内拍号
    wire [2:0] metro_beat_ext;     // 过两级同步器后，给 uart_packet_echo 回报用
    // ★★ V47：逐事件力度 ★★
    //   ★ 这两条必须【在这里】先声明：midi_vel_aud 在下面"混音缩放"那一段
    //     （voice_scale 附近）就要用，而驱动它的同步器在很后面。
    //     不提前声明就是"先用后声明"，工具会建出隐式 1 位线。
    wire [6:0] midi_vel;           // uart_packet_echo 的输出（clk_50m_ext 域）
    wire [6:0] midi_vel_aud;       // 同步到 clk_audio 之后

    assign led = LED_ACTIVE_LOW ? ~metro_led : metro_led;

    //=========================================================================
    // ★★★ V19 调试用：led_glide 显示【摇杆2 给出的颤音深度】（PWM 亮度）★★★
    //
    //   目的：把"摇杆2 到底有没有被 FPGA 读到"变成肉眼可见。
    //         ★ 不需要按任何按钮。
    //
    //   判读（亮度 ≈ 深度/255）：
    //     · 摇杆2 静止   -> 微亮（约 17%）
    //     · 慢慢推       -> 逐渐变亮
    //     · 推到底       -> 很亮（约 78%）
    //     · 怎么推都不变 -> AIN2 没读到
    //
    //   ★ 深度值 trem_depth_dyn 在 audio 域产生，这里用两级同步器取到
    //     50MHz 域显示（慢信号，2 级足够）。
    //   ★ 调试完想恢复成"颤音开关指示"，把最后那个 assign 换回：
    //       assign led_glide = LED_ACTIVE_LOW ? ~(glide_en|sustain_on|trem_on|dly_on)
    //                                         :  (glide_en|sustain_on|trem_on|dly_on);
    //=========================================================================
    reg [7:0] trem_dep_s1, trem_dep_s2;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            trem_dep_s1 <= 8'd0;
            trem_dep_s2 <= 8'd0;
        end else begin
            trem_dep_s1 <= trem_depth_dyn;
            trem_dep_s2 <= trem_dep_s1;
        end
    end

    // 50MHz 域 PWM：10 位分辨率，PWM 频率 = 50M/1024 ≈ 48.8kHz（无闪烁）
    reg [9:0] trem_pwm_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                    trem_pwm_cnt <= 10'd0;
        else if (trem_pwm_cnt >= 10'd1023) trem_pwm_cnt <= 10'd0;
        else                           trem_pwm_cnt <= trem_pwm_cnt + 10'd1;
    end

    // 8 位深度 -> 10 位占空比（左移 2 位，正好铺满 0~1023）
    wire [9:0] trem_duty = {trem_dep_s2, 2'b00};

    assign led_glide = LED_ACTIVE_LOW ? ~(trem_pwm_cnt < trem_duty)
                                      :  (trem_pwm_cnt < trem_duty);

    //------------------------------------------------------------------
    // 音色切换脉冲（翻转沿），跨时钟到音频域
    //------------------------------------------------------------------
    reg timbre_toggle_50m;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) timbre_toggle_50m <= 1'b0;
        else if (timbre_cycle) timbre_toggle_50m <= ~timbre_toggle_50m;
    end

    //==================================================================
    // 50MHz 域：TTP229 读取（每片一个实例，并行独立运行）
    //==================================================================
    wire [11:0] kp1, kp2, kp3;
    wire        rdy1, rdy2, rdy3;

    ttp229_reader u_kp1 (
        .clk         (clk_50m),
        .rst_n       (rst_n),
        .scl         (scl),
        .sdo         (sdo),
        .key_stable  (kp1),
        .rdy_toggle  (rdy1)
    );

    // ★ Console 60K 版：键盘2 已修好，三片全部启用。
    //   万一哪天某一片又坏掉，把 KB2_DISABLED 改回 1'b1 就能整段静默隔离：
    //     · scl2 恒输出 0 —— TTP229 的 SCL 空闲本来就是低，不会顶牛
    //     · sdo2 不再由读取器驱动，只当一个输入用
    //     · kp2 恒 0 —— 该片永远"没有键按下"，不会再出现 16 个幽灵音
    localparam KB2_DISABLED = 1'b0;

    generate
        if (KB2_DISABLED) begin : g_kb2_off
            assign scl2 = 1'b0;
            assign kp2  = 12'd0;
            assign rdy2 = 1'b0;
            wire sdo2_dummy = sdo2;      // 保留输入缓冲（引脚按 .cst 的 PULL_MODE 设置）
        end else begin : g_kb2_on
            ttp229_reader u_kp2 (
                .clk         (clk_50m),
                .rst_n       (rst_n),
                .scl         (scl2),
                .sdo         (sdo2),
                .key_stable  (kp2),
                .rdy_toggle  (rdy2)
            );
        end
    endgenerate

    // ★PAD3-3  第 3 片键盘（低八度）：SCL = L11，SDO = E10
    ttp229_reader u_kp3 (
        .clk         (clk_50m),
        .rst_n       (rst_n),
        .scl         (scl3),
        .sdo         (sdo3),
        .key_stable  (kp3),
        .rdy_toggle  (rdy3)
    );

    //==================================================================
    // 按键掩码：低位 = 琴键1，中间 = 琴键2，高位 = 琴键3
    //==================================================================
    wire [NUM_KEYS-1:0] pressed_mask = {kp3, kp2, kp1};

    //==================================================================
    // NOTE: the old "new-key event" generator (rdy edge detect + event_toggle)
    // was removed together with the key_mask rewrite above.  It existed only to
    // make the audio domain "load key_mask on a new key event"; now the audio
    // domain simply 2-stage-syncs the round-stable snapshot (pressed_mask),
    // which is both simpler and race-free.
    //
    // rdy1/rdy2/rdy3 are still produced by the ttp229_reader instances but are
    // no longer needed by the logic; they stay unconnected/harmless.

    //==================================================================
    // 跨时钟：按键事件 50MHz -> 12.5MHz
    //==================================================================
    // (event_toggle_d1/d2/d3 已随 key_mask 重写一并删除)

    //==================================================================
    // ★★ 按键跨时钟域：直接两级同步"整轮扫描内恒定"的稳定键值快照 ★★
    //
    //   （照 music_test0 的修复结论重写，这是"按一次响两次/响一片"的主因）
    //
    //   〖原来错在哪〗
    //   原实现：
    //       pressed_mask = {kp3,kp2,kp1}，而 kp 来自 ttp229_reader 里
    //       随轮的 ~DATA（DATA 在 IDLE 态被清零 → 那 0.25ms 里全 1）；
    //       50MHz 域再靠 rdy 边沿产生一个 event_toggle 脉冲，
    //       脉冲位置正好压在 DATA 被清零的窗口上；
    //       音频域用 key_event_audio 去 load key_mask。
    //   一旦脉冲压在清零之后，key_mask 就被 load 成【全 1】，
    //   36 路包络一起 Attack 一下 —— 听感是一记闷响/和弦咔声，
    //   紧跟在正常音头后面（就是所谓"第二声"）。
    //
    //   〖现在怎么改〗
    //   ttp229_reader 已经改成只在 WAIT_2MS 第一拍锁存快照 key_stable
    //   （整轮扫描期内恒定，且做过 2/2 去抖），所以这里【直接两级同步】
    //   就够：既不会加进毛刺，松手也能立即跟上。
    //
    //   ★ 顺手修掉一个隐藏 bug：原来同一轮里两个键同时新按下时，
    //     event_toggle 连翻两次、净变化为 0 → 音频域收不到脉冲 →
    //     那两个音直接漏响。改成快照直通后这个问题自然消失。
    //
    //   ★ 自动演奏 / MIDI 音符不再并进 key_mask 的载入，
    //     改为在音频域直接 OR 进来（见下面的 key_mask 声明）。
    //==================================================================

    //------------------------------------------------------------------
    // 跨时钟：音色翻转沿 50MHz -> 12.5MHz
    //------------------------------------------------------------------
    reg tt_d1, tt_d2, tt_d3;
    reg       timbre_audio;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            tt_d1        <= 1'b0;
            tt_d2        <= 1'b0;
            tt_d3        <= 1'b0;
            timbre_audio <= 1'b0;
        end else begin
            tt_d1 <= timbre_toggle_50m;
            tt_d2 <= tt_d1;
            tt_d3 <= tt_d2;
            if (tt_d2 ^ tt_d3)
                timbre_audio <= ~timbre_audio;   // 翻转，不再 +1
        end
    end

    // ★ 音色只由【按钮4】短按循环决定（与 25K K44 一致）。
    //   摇杆 X 轴在 25K 上是【移调】，不是音色 —— 不要改这个语义。

    //==================================================================
    reg [NUM_KEYS-1:0] pressed_mask_sync1, pressed_mask_sync2;

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            pressed_mask_sync1 <= {NUM_KEYS{1'b0}};
            pressed_mask_sync2 <= {NUM_KEYS{1'b0}};
        end else begin
            pressed_mask_sync1 <= pressed_mask;
            pressed_mask_sync2 <= pressed_mask_sync1;
        end
    end

    // X: replace event_toggle-based key_mask with direct 2-stage sync of the
    //    stable snapshot.  pressed_mask = {kp3,kp2,kp1}, and each kp is now
    //    the round-stable key_stable snapshot from ttp229_reader.
    //
    //    The old event_toggle scheme latched key_mask exactly on the window
    //    where DATA is cleared in IDLE state (~0.25ms of all-ones), which
    //    loaded a whole slab of keys at once -- the "one press, two sounds"
    //    / chord-thump root cause.  It also silently dropped notes when two
    //    keys in the same round were newly pressed (toggle flipped twice).
    //
    //    The snapshot only changes once per 2.26ms scan, so two sync flops
    //    are plenty; the ~8ms release debounce in the envelope is the backstop.
    //
    //    key_mask reflects physical keys only; auto-play / MIDI notes are OR'd
    //    in directly below.
    // ★ 蓝牙按键掩码跨到音频域（两级同步）
    reg [35:0] bt_keys_s1, bt_keys_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            bt_keys_s1 <= 36'd0;
            bt_keys_s2 <= 36'd0;
        end else begin
            bt_keys_s1 <= bt_keys;
            bt_keys_s2 <= bt_keys_s1;
        end
    end
    wire [35:0] bt_keys_aud = bt_keys_s2;

    //==================================================================
    // ★★ 演奏按键自定义映射：物理键 -> 声部（音符）★★
    //
    //   需求：演奏者能把某个按键重新指定成另一个音（默认恒等：键 n 发音符 n）。
    //
    //   ★★★ 为什么把映射做在【"物理键 -> 声部"这一步】，而不是去改发音目标 ★★★
    //
    //   最直觉的做法是加一张 key_map[]，让发音取 delta[key_map[g]]。
    //   但下面滑音那段有一句实测注释（见 gen_phase 里）：
    //       目标必须是【常数】delta[g]；一旦变成运行时寄存器，
    //       每个键都要一套真的 32 位加减法器，实测 24 键就多烧 2300 个 ALU。
    //   36 个键按这个比例要 3400+，而且全都压在音频关键路径上。
    //
    //   所以改成：**声部一个都不动**（还是常数 delta[g]），只把"哪个物理键
    //   按下"这件事搬到对应的声部位上。按下物理键 p -> 点亮声部 key_map[p]。
    //
    //   这个位置换来三个额外好处（都不是巧合，是选它的理由）：
    //     ① 滑音 / 移调 / 包络 / 相位那一整套一个字都不用改；
    //     ② 【乐谱播放不受映射影响】—— midi_keys 本来就是声部号，
    //        所以同一份谱子不管映射怎么改，弹出来还是谱子上写的音。
    //        电脑发谱时【不需要】按映射反查（本来以为要，现在不用了）；
    //     ③ 录音回传本来采的就是声部掩码，电脑合成 MIDI 时也不用套表。
    //   也就是说：**映射只影响手弹，不影响放谱**，这正好是想要的语义。
    //
    //   key_map[p] = 6'd63（或任何 >= 36 的值）表示【禁用该键】，按了不发声。
    //   两个物理键映到同一个声部也可以（或在一起）；反过来，一个声部如果没有
    //   任何物理键映过去，那就只能由乐谱来弹。
    //
    //   代价：36 x 36 的 one-hot 选择网络（每个物理键做一次 36 位译码，
    //   再按"这个键按下了吗"选通，最后 36 路或起来）。组合逻辑，
    //   在 12.5MHz 的音频域里深度完全够用。
    //==================================================================
    // 来自 uart_packet_echo 的映射写口（下面例化处驱动）
    wire       keymap_we_tgl;
    wire [5:0] keymap_key;
    wire [5:0] keymap_note;

    reg [5:0] key_map [0:NUM_KEYS-1];      // 默认恒等，见下面复位那段

    wire [NUM_KEYS-1:0] kmap_term [0:NUM_KEYS-1];
    genvar kp;
    generate
        for (kp = 0; kp < NUM_KEYS; kp = kp + 1) begin : gen_kmap
            // 这个物理键的映射值译成 one-hot；映射值越界（=禁用）时全 0
            assign kmap_term[kp] =
                (!pressed_mask_sync2[kp] || (key_map[kp] >= NUM_KEYS))
                    ? {NUM_KEYS{1'b0}}
                    : ({{(NUM_KEYS-1){1'b0}}, 1'b1} << key_map[kp]);
        end
    endgenerate

    wire [NUM_KEYS-1:0] hand_mask =
        kmap_term[0]  | kmap_term[1]  | kmap_term[2]  | kmap_term[3]  |
        kmap_term[4]  | kmap_term[5]  | kmap_term[6]  | kmap_term[7]  |
        kmap_term[8]  | kmap_term[9]  | kmap_term[10] | kmap_term[11] |
        kmap_term[12] | kmap_term[13] | kmap_term[14] | kmap_term[15] |
        kmap_term[16] | kmap_term[17] | kmap_term[18] | kmap_term[19] |
        kmap_term[20] | kmap_term[21] | kmap_term[22] | kmap_term[23] |
        kmap_term[24] | kmap_term[25] | kmap_term[26] | kmap_term[27] |
        kmap_term[28] | kmap_term[29] | kmap_term[30] | kmap_term[31] |
        kmap_term[32] | kmap_term[33] | kmap_term[34] | kmap_term[35];

    // ---- 映射表本体：上电恒等；之后由 UART 逐条改写 ----
    //   ★ 写口在 clk_50m_ext（UART 那边），这里要跨回 clk_audio：
    //     用"翻转信号 + 边沿检测"（和录音按钮同一套做法），
    //     key/note 两个字节在翻转沿到来之前早就稳定了，两级同步足够。
    reg [2:0] map_tgl_s;
    reg [5:0] map_key_s1, map_key_s2;
    reg [5:0] map_note_s1, map_note_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            map_tgl_s  <= 3'b000;
            map_key_s1 <= 6'd0; map_key_s2 <= 6'd0;
            map_note_s1<= 6'd0; map_note_s2<= 6'd0;
        end else begin
            map_tgl_s   <= {map_tgl_s[1:0], keymap_we_tgl};
            map_key_s1  <= keymap_key;   map_key_s2  <= map_key_s1;
            map_note_s1 <= keymap_note;  map_note_s2 <= map_note_s1;
        end
    end
    wire keymap_we_aud = map_tgl_s[2] ^ map_tgl_s[1];   // 还原成 1 拍脉冲

    integer kmi;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            // ★ 默认【恒等】：键号 n 就发声符 n —— 和加这个功能之前逐位相同
            for (kmi = 0; kmi < NUM_KEYS; kmi = kmi + 1)
                key_map[kmi] <= kmi[5:0];
        end else if (keymap_we_aud && (map_key_s2 < NUM_KEYS)) begin
            key_map[map_key_s2] <= map_note_s2;
        end
    end

    wire [NUM_KEYS-1:0] key_mask = hand_mask | auto_keys_aud | midi_keys_aud | bt_keys_aud;

    // 延音开关跨到音频域（慢信号，2 级同步足够）
    reg sustain_s1, sustain_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            sustain_s1 <= 1'b0;
            sustain_s2 <= 1'b0;
        end else begin
            sustain_s1 <= sustain_on;
            sustain_s2 <= sustain_s1;
        end
    end
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

    // ---- 吉他音色开关跨到音频域（慢信号，两级同步足够）----
    //   ★ 弦模型跑在 clk_audio 上，所以"是不是吉他"必须在音频域里是一根干净的线。
    reg gtr_s1, gtr_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            gtr_s1 <= 1'b0;
            gtr_s2 <= 1'b0;
        end else begin
            gtr_s1 <= guitar_on;
            gtr_s2 <= gtr_s1;
        end
    end
    wire guitar_on_audio = gtr_s2;

    // ---- ★ V48.2：小号音色开关跨到音频域（和上面两个完全同一套做法）----
    //   ★ 慢信号（人按按钮才变），两级同步足够。
    reg trp_s1, trp_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            trp_s1 <= 1'b0;
            trp_s2 <= 1'b0;
        end else begin
            trp_s1 <= trumpet_on;
            trp_s2 <= trp_s1;
        end
    end
    wire trumpet_on_audio = trp_s2;

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

    //==================================================================
    // 摇杆：ADS1115 A0 = X 轴（移调），A1 = Y 轴（总音量）
    //
    //   ★ 原压力传感器（力度 + 峰值锁存）已整体移除。
    //   ADC 是单端输入、PGA=±4.096V，32767 对应 4.096V；
    //   摇杆 3.3V 供电 → 满幅读数约 26400，不会饱和。
    //   A0/A1 的原始读数原样上报到 VOFA+ 第 1、2 路，第一次上板可以
    //   直接看波形来校准 JS_CENTER / JS_TRAVEL。
    //==================================================================
    wire [15:0] ain0_data, ain1_data, ain2_data, ain3_data;

    // ★ 暂时没接摇杆时，把这个改成 1'b1：
    //   忽略摇杆输入，固定"原调 + 满音量"，先验证琴本身正常。
    //   （因为 Y 轴读数如果飘到 0，音量就是 0，会表现为完全没声）
    // ★★ V16 音高修正（根因）★★
    //
    //   现象：琴键1 最左边那个白键（应该是 A2 = 110 Hz）被调音器读成 G#2 = 103.83 Hz。
    //   证据链：
    //     110 x trans_ratio[11] = 110 x 61858/65536 = 103.83 Hz  ← 与实测【分毫不差】
    //     也就是【正好低一个半音】= 摇杆移调档位停在 11 而不是 12（原调）。
    //   为什么停在 11：X 轴静止时读数不在中心。原来真正的"无移调窗口"只有 ±1610
    //   counts（见 JS_DEAD 处的推导），而摇杆静止偏差只要落进 1610~2428 这一带，
    //   算出来就是 js_xn = 1 -> 降一个半音。
    //   修法：把 JS_DEAD 加宽到 2600，实际无移调窗口变成 ±2948 counts，
    //         覆盖了 1610~2428 全带 -> 静止时必定回【原调】，音阶恢复正常。
    //   ★ 所以这里保持 1'b0（摇杆可用）。真要让摇杆彻底不参与，才改成 1'b1。
    localparam JOY_BYPASS = 1'b0;

    //------------------------------------------------------------------
    // 1) 50MHz 域：对 ADS1115 原始读数做【多点平均】降噪
    //
    //   ★ 为什么要换掉原来的一阶 IIR：
    //     ADS1115 是 860SPS 四通道轮询，同一个通道大约每 4.65ms 才更新
    //     一次。而原来的 IIR 每 20.5us 就走一步、时间常数只有 0.66ms ——
    //     它在两次 ADC 更新之间早就走到位了，等于只把台阶抹圆了一点，
    //     对 ADC 自身的噪声几乎没有任何抑制。
    //     平均窗口必须跨过至少两个 ADC 周期才真正起作用。
    //------------------------------------------------------------------
    // ★ V17：16 点 -> 4 点。
    //   ADS1115 是 128SPS（见 ads1115_ctrl.v 的 CFG_L=0x83），
    //   一个通道约 20ms 才出一个【新】读数；而 16 点窗口只有 10.5ms，
    //   等于把同一个样本重复平均，噪声没压下去，只白白加了 10.5ms 延迟。
    //   128SPS 本身就足够安静，改成 4 点（2.6ms）当毛刺滤波即可。
    localparam JS_AVG_LOG2 = 2;                       // 2^2 = 4 点
    localparam JS_AVG_N    = (1 << JS_AVG_LOG2);      // 4
    localparam [15:0] JS_TAP_DIV = 16'd32768;         // 每 655us 采一点

    reg [15:0] js_tdiv;
    reg [4:0]  js_acnt;
    reg [20:0] xacc, yacc, zacc;
    reg [15:0] js_x, js_y, js_z;

    wire js_tap  = (js_tdiv == JS_TAP_DIV - 1'b1);
    wire js_done = js_tap && (js_acnt == JS_AVG_N-1);

    // ★ 优化：把「16 点平均」写成显式位选。
    //   原来是 (xacc + ain0_data) >> JS_AVG_LOG2 再截成 16 位，
    //   等价于取加法结果的位段 [JS_AVG_LOG2+15 : JS_AVG_LOG2]，
    //   数值完全一致，只是把静默截断（EX3791 告警）变成写明的位段。
    wire [20:0] js_xsum = xacc + ain0_data;
    wire [20:0] js_ysum = yacc + ain1_data;
    wire [20:0] js_zsum = zacc + ain2_data;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            js_tdiv <= 16'd0;
            js_acnt <= 5'd0;
            xacc    <= 21'd0;
            yacc    <= 21'd0;
            zacc    <= 21'd0;
            js_x    <= JS_CENTER;
            js_y    <= JS_CENTER;
            js_z    <= JS_CENTER;
        end else begin
            js_tdiv <= js_tap ? 16'd0 : (js_tdiv + 1'b1);

            if (js_tap) begin
                xacc <= xacc + ain0_data;
                yacc <= yacc + ain1_data;
                zacc <= zacc + ain2_data;

                if (js_done) begin
                    // 最后一点这一拍还没进累加器，所以这里要手动补上
                    js_x    <= js_xsum[JS_AVG_LOG2+15 : JS_AVG_LOG2];
                    js_y    <= js_ysum[JS_AVG_LOG2+15 : JS_AVG_LOG2];
                    js_z    <= js_zsum[JS_AVG_LOG2+15 : JS_AVG_LOG2];
                    xacc    <= 21'd0;
                    yacc    <= 21'd0;
                    zacc    <= 21'd0;
                    js_acnt <= 5'd0;
                end else begin
                    js_acnt <= js_acnt + 1'b1;
                end
            end
        end
    end

    // 每秒 1024 拍（≈20.5us）一次的快节拍，给下面的换档迟滞用
    reg [9:0] js_tick;
    wire      js_upd = (js_tick == 10'd1023);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) js_tick <= 10'd0;
        else        js_tick <= js_upd ? 10'd0 : (js_tick + 1'b1);
    end

    //------------------------------------------------------------------
    // 2) X 轴 → 移调档位 0~24（12 = 原调）
    //    |x - 中心| 先扣掉死区，再按行程线性映射成 0~JS_STEPS 个半音。
    //    死区必须留：摇杆静止时读数会在几百个 count 内抖，
    //    不扣死区会一直在相邻半音之间跳。
    //------------------------------------------------------------------
    wire signed [16:0] js_xd = $signed({1'b0, js_x}) - $signed({1'b0, JS_CENTER});
    wire        [15:0] js_xa = js_xd[16] ? (~js_xd[15:0] + 1'b1) : js_xd[15:0];  // |偏移|
    wire        [15:0] js_xe = (js_xa > JS_DEAD) ? (js_xa - JS_DEAD) : 16'd0;    // 扣死区

    //==================================================================
    // ★★ V48：这条组合链插【两级流水】★★
    //==================================================================
    //   病灶（V48 构建实测，PnR 时序报告原话）：
    //       js_x_7_s0/Q  →  trans_cand_3_s0/D    数据延迟 22.541ns（约束 20ns）
    //     也就是 js_x → 减中线 → 取绝对值 → 扣死区 → ×70 → 四舍五入 → 限幅
    //     → 12±n 这一长串全是组合逻辑，一共 5 条端点负 slack，
    //     clk_50m 的 TNS = **-7.899ns**（V47.2 时三个时钟域都是 0）。
    //     ★ 逻辑一个字没改，是 V48 新增的那点逻辑把布局挤了 ——
    //       正是第 5 节坑 25 那条："我的改动不在那个域 ≠ 那个域的时序不会变差"。
    //   治法：和 V41 治 LCD 那条 21 级链、V17c 给 peak_msb 打拍【同一个套路】——
    //     从中间插寄存器，把一条长链切成三段短的。
    //   ★★ 为什么插在这里【一定安全】（三条论证，缺一不可）：
    //     ① **时间上绰绰有余**：js_x 每 (4 × 655us) ≈ **2.6ms** 才更新一次
    //        （`js_done`），而采样点 `js_upd` 是 **20.5us** 一次 ——
    //        中间隔了 100 多倍。迟 2 拍 = 40ns，在这个尺度上完全看不见。
    //     ② **没有别的消费者**：`js_xa` / `js_xd[16]` / `js_xn` 这三个中间量
    //        全文件【只在这里用】（已 grep 核对），不存在"改了一处、
    //        别处还在用旧组合值"的风险。
    //     ③ **不会撕裂**：第 2 级那几个寄存器是【同一拍一起打】的，
    //        所以任何一个采样点看到的永远是"同一份 js_x 算出来"的一组值，
    //        不会出现"幅值是新的、符号还是旧的"这种中间态。
    //   ★ 代价：25 个触发器。和 7.9ns 的负 slack 比不值一提。
    //   ★ 行为等价性：`js_xe_q/js_dead_q/js_neg_q` 就是原来那三个组合量的
    //     打一拍版本，`js_xn_q` 同理 —— 数值一个比特都没变，只是晚 2 拍。
    reg [15:0] js_xe_q;
    reg        js_dead_q;      // 对应原来的 (js_xa <= JS_DEAD)
    reg        js_neg_q;       // 对应原来的 js_xd[16]（往低移）
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            js_xe_q   <= 16'd0;
            js_dead_q <= 1'b1;      // 复位后当"在死区里" -> 原调，和原来一致
            js_neg_q  <= 1'b0;
        end else begin
            js_xe_q   <= js_xe;
            js_dead_q <= (js_xa <= JS_DEAD);
            js_neg_q  <= js_xd[16];
        end
    end

    wire [31:0] js_xn32 = ((js_xe_q * JS_XMUL) + 32'd32768) >> 16;               // 四舍五入
    wire [4:0]  js_xn   = (js_xn32 > JS_STEPS) ? JS_STEPS[4:0] : js_xn32[4:0];

    //   第二级：乘法结果和两个标志再打一拍（合起来给 trans_cand 用）
    reg [4:0]  js_xn_q;
    reg        js_dead_q2, js_neg_q2;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            js_xn_q    <= 5'd0;
            js_dead_q2 <= 1'b1;
            js_neg_q2  <= 1'b0;
        end else begin
            js_xn_q    <= js_xn;
            js_dead_q2 <= js_dead_q;
            js_neg_q2  <= js_neg_q;
        end
    end

    //------------------------------------------------------------------
    // ★ 迟滞：候选档位必须连续 8 次都不同才真正换档（8×20.5us ≈ 164us）。
    //
    //   死区只能挡住"中心附近"的抖动，挡不住【格子边界】上的抖动：
    //   摇杆静止时读数正好压在两个半音的临界点上，档位就会两格之间
    //   来回跳，每跳一次 inc[] 就要重算 → dcur 跟着来回滑 →
    //   听感就是一层粗糙的沙沙声。这段迟滞专门治它。
    //------------------------------------------------------------------
    reg [4:0] trans_cand, trans_idx;    // 0~24，12 = 原调
    reg [3:0] trans_hcnt;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            trans_cand <= 5'd12;
            trans_idx  <= 5'd12;
            trans_hcnt <= 4'd0;
        end else if (js_upd) begin
            // 1) 算出候选档位
            //   ★ V48：用【打完两拍】的那一份（见上面流水线那一大段）——
            //     三个量是一起打拍的，所以永远是同一份 js_x 算出来的一组值。
            if (js_dead_q2)      trans_cand <= 5'd12;
            else if (js_neg_q2)  trans_cand <= 5'd12 - js_xn_q;   // 往低移
            else                 trans_cand <= 5'd12 + js_xn_q;   // 往高移

            // 2) 连续稳定才提交
            if (trans_cand == trans_idx) begin
                trans_hcnt <= 4'd0;
            end else if (trans_hcnt == 4'd7) begin
                trans_idx  <= trans_cand;
                trans_hcnt <= 4'd0;
            end else begin
                trans_hcnt <= trans_hcnt + 1'b1;
            end
        end
    end

    //------------------------------------------------------------------
    // 3) Y 轴 → 总音量 0~255（0 = 静音，255 = 改造前的正常响度）
    //    ★ 255 档实际是 ×255/256，所以推到头不会比以前更早削顶。
    //------------------------------------------------------------------
    //------------------------------------------------------------------
    // ★ V19：AIN1 -> 力度 也改成【查表】，省掉一个 16×10 乘法器
    //
    //   原来是  js_v32 = js_y * JS_VMUL(633) >> 16，再钳到 255。
    //   这里把结果预先算好放进 64 项表（索引 = js_y[15:10]，每档 1024 counts），
    //   数值与原公式在档位中点处完全一致，且严格单调。
    //   省一个宽乘法，对综合时间有帮助。
    //   ★ 若哪天要改摇杆灵敏度，改这张表即可（或改回乘法版）。
    //------------------------------------------------------------------
    reg [7:0] v_lut [0:63];

    initial begin
        v_lut[ 0] = 8'd5;    v_lut[ 1] = 8'd15;   v_lut[ 2] = 8'd25;   v_lut[ 3] = 8'd35;
        v_lut[ 4] = 8'd45;   v_lut[ 5] = 8'd54;   v_lut[ 6] = 8'd64;   v_lut[ 7] = 8'd74;
        v_lut[ 8] = 8'd84;   v_lut[ 9] = 8'd94;   v_lut[10] = 8'd104;  v_lut[11] = 8'd114;
        v_lut[12] = 8'd124;  v_lut[13] = 8'd134;  v_lut[14] = 8'd143;  v_lut[15] = 8'd153;
        v_lut[16] = 8'd163;  v_lut[17] = 8'd173;  v_lut[18] = 8'd183;  v_lut[19] = 8'd193;
        v_lut[20] = 8'd203;  v_lut[21] = 8'd213;  v_lut[22] = 8'd223;  v_lut[23] = 8'd232;
        v_lut[24] = 8'd242;  v_lut[25] = 8'd252;  v_lut[26] = 8'd255;  v_lut[27] = 8'd255;
        v_lut[28] = 8'd255;  v_lut[29] = 8'd255;  v_lut[30] = 8'd255;  v_lut[31] = 8'd255;
        v_lut[32] = 8'd255;  v_lut[33] = 8'd255;  v_lut[34] = 8'd255;  v_lut[35] = 8'd255;
        v_lut[36] = 8'd255;  v_lut[37] = 8'd255;  v_lut[38] = 8'd255;  v_lut[39] = 8'd255;
        v_lut[40] = 8'd255;  v_lut[41] = 8'd255;  v_lut[42] = 8'd255;  v_lut[43] = 8'd255;
        v_lut[44] = 8'd255;  v_lut[45] = 8'd255;  v_lut[46] = 8'd255;  v_lut[47] = 8'd255;
        v_lut[48] = 8'd255;  v_lut[49] = 8'd255;  v_lut[50] = 8'd255;  v_lut[51] = 8'd255;
        v_lut[52] = 8'd255;  v_lut[53] = 8'd255;  v_lut[54] = 8'd255;  v_lut[55] = 8'd255;
        v_lut[56] = 8'd255;  v_lut[57] = 8'd255;  v_lut[58] = 8'd255;  v_lut[59] = 8'd255;
        v_lut[60] = 8'd255;  v_lut[61] = 8'd255;  v_lut[62] = 8'd255;  v_lut[63] = 8'd255;
    end

    wire [7:0] js_vol = v_lut[js_y[15:10]];

    //------------------------------------------------------------------
    // 3b) ADC2（ADS1115 的 AIN2）-> 颤音深度
    //
    //   ★ 参照 D:\FPGA_60K\ADS11172 那个【已经跑通】的工程：
    //       它的 led_pwm 约束在 G11，而本工程 G11 就是 led_glide。
    //       同一个球、同一路 I2C 引脚，它能亮 -> AIN2 硬件完全正常。
    //       它用的办法是【纯查表、不做任何乘除】，这里照抄同一套思路，
    //       所以这一段没有任何乘法器。
    //
    //   ★★ V19 反转（按你的要求）：摇杆【中位 = 颤音最深】，
    //      往两边推越来越浅，推到约 78% 行程后几乎无颤音。
    //         偏移 0      -> 深度 200（最深）
    //         偏移 4096   -> 深度 116
    //         偏移 8192   -> 深度 32
    //         偏移 >=10240-> 深度 10（几乎无）
    //
    //   取法：以 ADC 中位为 0，往两边推都算偏移（取绝对值），
    //         所以推左推右效果一样，摇杆不回中也没关系。
    //   索引 = |js_z - JS_CENTER| 的高 6 位（64 档，每档 1024 counts）
    //------------------------------------------------------------------
    wire signed [16:0] js_zd = $signed({1'b0, js_z}) - $signed({1'b0, JS_CENTER});
    wire        [15:0] js_za = js_zd[16] ? (~js_zd[15:0] + 1'b1) : js_zd[15:0];  // |偏移|

    //==================================================================
    // ★★ V48.2：这条链也插【两级流水】★★
    //==================================================================
    //   病灶（V48.2 构建实测，PnR 时序报告原话）：
    //       js_z_6_s0/Q  →  tdep_send_7_s0/CE    数据延迟 19.248ns（约束 20ns）
    //     也就是 js_z → 减中线 → 取绝对值 → 取高 6 位 → ×21 → 比较 → 那个
    //     "三个值有没有变"的 OR → 21 个触发器的 CE。一共 14 条端点，
    //     clk_50m 因此掉到 51.127MHz（余量只剩 2.3%）。
    //     ★ 和坑 25 一样：这一版只改了显示模式的位宽，**摇杆逻辑一个字没动**，
    //       纯粹是布局被挤了（V48 时同一条链还有 71.47MHz 的余量）。
    //   治法：和上面 js_x 那条链【完全同一个套路】——从中间插寄存器，
    //     把一条长链切成三段短的。
    //   ★★ 为什么插在这里一定安全（和 js_x 的三条论证同源）：
    //     ① **时间上绰绰有余**：js_z 每 (4 × 655us) ≈ **2.6ms** 才更新一次
    //        （`js_done`），迟 2 拍 = 40ns 完全看不见；
    //     ② **没有别的消费者**：`js_za` 这个中间量全文件只喂下面这一处
    //        （`tk` -> `js_tdep`），`js_tdep` 也只喂那个"值变了就发"的比较；
    //     ③ **不会撕裂**：两级各自整体打拍，取用点看到的永远是同一份 js_z
    //        算出来的一组值（`js_za_q` 与 `js_tdep_q` 都是整字寄存）。
    //   ★ 代价：16 + 8 = 24 个触发器，换回二十多 MHz 的余量。
    reg [15:0] js_za_q;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) js_za_q <= 16'd0;
        else        js_za_q <= js_za;
    end

    // ★★ 索引用【高 6 位】js_za[15:10]（每档 1024 counts）★★
    //   ★ V19 反转：摇杆【中位 = 最深 200】，往两边推越来越浅，
    //     推到 10240 counts（约 78% 行程）之后几乎无颤音（深度 10）。
    //     也就是"中间最明显、两边几乎没有"。
    //
    // ★★★ V19 速度优化：64 项查表 -> 算术式（零功能改动）★★★
    //   原来的 tdep_lut[0:63] 是 reg 数组 + initial，综合器要把它
    //   展开成 64:1 的 LUT MUX 树。
    //   实测表值【完全等于】下面这条公式（逐项核对 64/64 一致）：
    //       k<=8 : 200 - 21k        k=0->200  k=1->179 ... k=8->32
    //       k>=9 : 10
    //   换成算术式后只占一个小乘法 + 一个比较器，不再展开 MUX 树。
    //   ★ 只动摇杆这一路，ROM / gain_lut / sine_rom 一律没碰。
    //------------------------------------------------------------------
    wire [5:0]  tk      = js_za_q[15:10];      // ★ V48.2：用打完一拍的那份
    wire [7:0]  tk_mul  = {2'b0, tk} * 8'd21;
    wire [7:0]  js_tdep = (tk <= 6'd8) ? (8'd200 - tk_mul) : 8'd10;

    //   ★★ V48.2：第二级流水 —— 深度值再打一拍才交给"值变了就发"那段。
    //     这样 CE 那一侧的组合深度只剩"8 位比较 + 或 + 扇出"。
    reg [7:0] js_tdep_q;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) js_tdep_q <= 8'd10;        // 和 js_tdep 的静止值一致
        else        js_tdep_q <= js_tdep;
    end


    //------------------------------------------------------------------
    // 4) 跨到 12.5MHz 音频域
    //
    //   ★★ 多位总线【必须】用翻转握手，不能用普通两级同步器！
    //
    //     普通两级同步器只保证"每一位"不亚稳态，不保证各位在同一次
    //     采样里被判成同一个时刻的值。js_vol 从 127 变 128 时 8 位同时
    //     在翻，音频域可能采到"高位是新的、低位是旧的"这种不合法的
    //     组合（比如直接采成 255）→ 输出瞬间跳一个大台阶 →
    //     听感就是"劈里啪啦"的电流声/爆音。
    //
    //     而且越是频繁改越容易撞上 —— 调音量时 js_vol 持续在变，
    //     命中概率暴涨，正好对应"调音量时尤为频繁"。
    //     trans_idx 虽然只有 5 位，同样会中招，采错就瞬间跳调。
    //
    //   做法：数据先在 50MHz 域寄存稳定，再用一个翻转位通知音频域
    //   "可以取了"；音频域检出翻转沿才去采，此时数据已稳定很久。
    //------------------------------------------------------------------
    reg [4:0] tr_send;
    reg [7:0] vel_send;
    reg [7:0] tdep_send;
    reg       cdc_tgl;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tr_send   <= 5'd12;
            vel_send  <= 8'd220;
            tdep_send <= 8'd110;      // 默认 = 原来的 TREM_DEPTH
            cdc_tgl   <= 1'b0;
        end else if ((tr_send != trans_idx) || (vel_send != js_vol) ||
                     (tdep_send != js_tdep_q)) begin
            tr_send   <= trans_idx;
            vel_send  <= js_vol;
            tdep_send <= js_tdep_q;    // ★ V48.2：和上面判据用同一份（打完两拍的值）
            cdc_tgl   <= ~cdc_tgl;
        end
    end

    reg       cdc_t1, cdc_t2, cdc_t3;
    reg [4:0] tr_audio_r;
    reg [7:0] vel_audio_r;
    reg [7:0] tdep_audio_r;

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            cdc_t1      <= 1'b0;
            cdc_t2      <= 1'b0;
            cdc_t3      <= 1'b0;
            tr_audio_r  <= 5'd12;
            vel_audio_r <= 8'd220;
            tdep_audio_r <= 8'd110;
        end else begin
            cdc_t1 <= cdc_tgl;
            cdc_t2 <= cdc_t1;
            cdc_t3 <= cdc_t2;
            // 检出翻转沿才采：此时 tr_send/vel_send/tdep_send 已稳定至少 2 拍
            if (cdc_t2 ^ cdc_t3) begin
                tr_audio_r   <= tr_send;
                vel_audio_r  <= vel_send;
                tdep_audio_r <= tdep_send;
            end
        end
    end

    wire [4:0] trans_audio = JOY_BYPASS ? 5'd12  : tr_audio_r;
    wire [7:0] vel_audio = JOY_BYPASS ? 8'd220 : vel_audio_r;
    wire [7:0] tdep_audio = JOY_BYPASS ? 8'd110 : tdep_audio_r;

    //==================================================================
    // 12.5MHz 域：NUM_KEYS 路 DDS + 混音 + I2S
    //==================================================================
    localparam CLK_FREQ_HZ = 12_500_000;

    //=========================================================================
    // ★★ I2S 参数（唯一开关：I2S_SPEC）★★
    //
    //  为什么要有这个开关 —— PCM5102A 数据手册 ZHCS323B 的硬要求：
    //     "System clock frequency = 64, 128, 192, 256, 384, 512, 768,
    //      1024, 1152, 1536, 2048, or 3072 Ã— fS"
    //     "SCK rates that are not common to standard audio clocks, between
    //      1MHz and 50MHz, are only supported in software mode, available
    //      only in the PCM512x and PCM514x devices"
    //  PCM5102A 没有软件模式。而本板 PCM5102 模块的 SCK 接 GND，
    //  DAC 拿 BCK 当系统时钟，所以 BCK/LRCK 就是这个比值。
    //
    //     I2S_SPEC = 1'b0（历史原样，25k K44 同款）：
    //         BCK 半周期 3 拍 → BCK = 12.5M/6 = 2.0833 MHz
    //         一帧 48 个 BCK  → fs  = 12.5M/(48*6) = 43.40 kHz
    //         比值 48×  ← ★不在允许表里，超规格运行
    //
    //     I2S_SPEC = 1'b1（V18 起改对；V49 起 BCLK_HALF 由 2 改成 1）：
    //         V48 及以前：BCK 半周期 2 拍 → BCK = 12.5M/4 = 3.1250 MHz
    //                     一帧 64 个 BCK   → fs  = 12.5M/(64*4) = 48.83 kHz
    //         V49 起    ：BCK 半周期 1 拍 → BCK = 12.5M/2 = 6.2500 MHz
    //                     一帧 64 个 BCK   → fs  = 12.5M/(64*2) = 97.66 kHz
    //         比值都是 64×  ← 允许表里的最小值，合法（比值没变，只是整体快了）
    //         左/右各 32bit 槽，16bit 样本左对齐放在每槽高 16 位
    //
    //  ★ 采样率、SAMPLE_CNT_MAX 全部由下面这组常数推导，
    //    保证「DDS 每产生一个样本，I2S 正好发一个帧」，不会丢样本。
    //=========================================================================
    // =========================================================================
    // ★★ V18 结论：I2S_SPEC 改回 1'b0（48bit 帧）★★
    //
    //   证据链（用能出完美正弦的 D:\FPGA_60K\i2s 工程做对照）：
    //
    //     项目            BCLK_HALF  FRAME_BITS  每帧 BCK   数据放置
    //     i2s（完美正弦）      3          48         48      {sample,8'b0}
    //     music（有直线）      2          64         64      {sample,16'b0}
    //
    //   i2s 工程用的就是【48bit 帧 = 24bit 槽】那一套，
    //   而 music 从 TEST_C_I2S改合规_64x 起换成了 64bit 帧 = 32bit 槽。
    //
    //   版本时间线也印证：V7/V8/V9（无 I2S_SPEC，沿用 K44 原样）→ 好；
    //   TEST_C 起 I2S_SPEC=1 → 一直坏到 V17。
    //   历史里还有一个版本叫 V6_I2S也回退到K44 —— 说明 K44 原样的
    //   48bit 规格才是这块板子上验证过能出声的版本。
    //
    //   为什么 64bit 会出直线：每帧多出来的 16 个位被 PCM5102A 当成
    //   有效数据继续采样，DAC 每个采样周期多收了一截，输出波形被拉平。
    //   48bit（左对齐 16bit 数据 + 8bit 补零）才和这颗 DAC 的时序对上。
    //
    //   ★ 所以这里改回 1'b0。下面 SAMPLE_RATE_HZ 会自动跟着变成 43.40kHz，
    //     与 i2s 工程声明的 48000 标称值是同一档（12.5M/288）。
    //   ★ 若哪天换成真正支持 64× 的 DAC，再改回 1'b1。
    // =========================================================================
    // ★★ V18 更正：上面那段"改回 1'b0 对齐 i2s"的结论是【错的】，已改回 1'b1 ★★
    //   实测改成 1'b0（48bit 帧）后：音量小得可怕（约 -48dB）、多键爆音。
    //   说明 DAC 是按【32bit 槽】取数的，帧缩到 48bit 后它仍按 32bit 切，
    //   24bit 槽里的数据被又右移 8 位 -> 音量掉 256 倍；左右声道也错位 -> 爆音。
    //
    //   ★ 关键教训：D:\FPGA_60K\i2s 那个"完美正弦"工程与本工程的差别
    //     【不在 I2S 帧规格】，而在【音频链】：
    //         i2s  ：无包络（无 ADSR，长按一直响）、无谐波、无 AGC、12 键
    //         music：有包络（会衰减到静音）、有谐波、有 AGC、36 键
    //     拿它对照 I2S 帧长是我判断失误，两边 DAC 都工作在 64×/32bit 槽。
    //
    //   ★ 查"波形上的直线"要对照【音频链】而不是 I2S：
    //     i2s 无包络，按住时永远有信号 -> 不会有直线；
    //     music 的包络会衰减到 0，env=0 输出恒 0 -> 那就是直线。
    localparam I2S_SPEC = 1'b1;

    //=========================================================================
    // ★★★ V19 结论：真正的病灶是【缺了 I2S 的 1 个 BCK 延迟】★★★
    //
    //   前面 V18 一直在"48bit 帧 / 64bit 帧"之间来回改，方向是错的 ——
    //   帧长和槽宽只决定 DAC 一个槽里收多少位，不影响【位对齐】。
    //
    //   真正的错误在发送时序：原来是把数据【紧贴 LRCK 跳变】送出去
    //   （left-justified），LRCK 跳变后第 1 个 BCK 就送 sample[15]。
    //   而 PCM5102A 在硬件 I2S 模式下按【标准 I2S】取数：
    //   第 1 个 BCK 是空位，真正的 MSB 在第 2 个 BCK 上。
    //
    //   于是 DAC 实际锁存到的是整体左移 1 位的样值：
    //       DAC_word = 2 × (真实样值)   —— 还丢掉了低位、只剩 ~8 位有效精度
    //   用 iverilog 把 bck/lck/din 逐位解出来量过：
    //       修之前：DAC_word = 8 × sample_vol   （幅度错 8 倍、台阶极粗）
    //       修之后：DAC_word =     sample_vol   （逐位精确，145/146 帧完全相等）
    //
    //   示波器上"一段正弦 + 一条拉平的直线"就是这个原因：
    //   强信号段被顶成平顶/台阶，弱信号段台阶塌成 0，看上去就是直线。
    //
    //   修法见下面 I2S 发送段：每个声道前面补 1 个 BCK 空位，
    //   数据右移一位（LSB 补 0），MSB 正好落在 LRCK 跳变后的第 2 个 BCK。
    //   ★ 改 I2S_SPEC / FRAME_BITS 时必须一起看那一段，别把延迟再丢掉。
    //=========================================================================

    // I2S_ALWAYS: BCK/LRCK 常开，静音靠 DIN 送 0 实现。
    //   music_test0 实测结论：PCM5102A 检测到 BCK/LRCK 停止会进
    //   Standby/Power Save，每个音的起停都重走一次内部流程，
    //   必须把这个门控去掉。
    localparam integer I2S_ALWAYS = 1;

    // ★★ 位宽教训（踩过一次，导致整块音频域被综合掉 → 完全没声音）★★
    //   绝对不能写成  localparam [5:0] FRAME_BITS = 6'd64;
    //   因为 64 需要 7 位（6 位只能表示 0~63），会被截断成 0，于是
    //       FRAME_DIV = FRAME_BITS * BCLK_HALF * 2 = 0
    //       SAMPLE_RATE_HZ = CLK_FREQ_HZ / 0   â†’ x
    //   综合日志里的证据：
    //       WARN (EX3792): Literal value 'd64 truncated to fit in 6 bits
    //       WARN (EX3810): Attempt to divide by 0, returning x
    //   后果不是报错，而是【静默地把 DDS/ROM/包络/混音全部优化掉】，
    //   网表资源从 REG 4319 掉到 124，烧进去完全没声音。
    //   所以这里一律用【无位宽的整数】做常数算术，避免任何截断。
    // ★★★ V49：BCLK_HALF 2 → 1，采样率翻倍到 97.656 kHz ★★★
    //   为什么只改这一个数就能翻倍：
    //     fs = clk_audio / (FRAME_BITS × BCLK_HALF × 2)
    //        = 12.5MHz / (64 × 1 × 2) = 12.5MHz / 128 = 97656.25 Hz
    //   ★ BCK/fs 比值【没变，还是 64×】—— 变快的是 BCK 本身
    //     （3.125MHz → 6.25MHz），所以 PCM5102A 仍然工作在规格内的
    //     标准倍率档（手册：fSCK 倍率表含 64；fs 支持 8k~384k；
    //     系统时钟最高 50MHz）→ 合法，不需要动 PLL。
    //   ★ 音高不用手工重算：下面 SAMPLE_RATE_HZ 和 calc_delta() 都是
    //     由这组常数【推导】出来的，36 个频率字自动跟着变。
    localparam integer BCLK_HALF  = I2S_SPEC ? 1 : 3;
    localparam integer FRAME_BITS = I2S_SPEC ? 64 : 48;

    // 一帧 = FRAME_BITS 个 BCK = FRAME_BITS × 2 × BCLK_HALF 个音频时钟
    //   ★ V49（当前）：64bit 帧 × 2 × 1 = 128 → fs = 12.5M/128 = 97.66kHz, 64×
    //   V48 及以前   ：64bit 帧 × 2 × 2 = 256 → fs = 12.5M/256 = 48.83kHz, 64×
    //   48bit 帧 × 2 × 3 = 288  → fs = 12.5M/288 = 43.40kHz, BCK/fs = 48×
    localparam integer FRAME_DIV = FRAME_BITS * BCLK_HALF * 2;

    //=========================================================================
    // ★ 采样周期 = I2S 帧周期（这一条本来就是对的，这里明确写下来）
    //
    //   FRAME_DIV = 64 × 1 × 2 = 128 个音频时钟（V49）
    //     → DAC 端 fs = 12.5MHz / 128 = 97.66 kHz
    //   SAMPLE_CNT_MAX = FRAME_DIV - 1 = 127
    //     → 音频域每 128 拍产生一个新采样，与帧边界一一对齐
    //
    //   两者必须相等：DAC 每帧取一次数，音频域每帧也给一个新数，
    //   这样"一个采样正好占一帧"，不会被连发两遍或漏掉；
    //   同时 SAMPLE_RATE_HZ 必须等于真实交付给 DAC 的采样率
    //   （12.5MHz/128 = 97656Hz），否则 calc_delta() 算出的频率字
    //   整体偏低一半，所有音都会闷掉一个八度。
    //
    //   ★ 注意：不要把它写成 48828 之类的旧值 —— 那个数字对应的是
    //     48bit 帧/BLCK_HALF=2 的老配置（256 拍）。改 I2S_SPEC 时
    //     这三个常数必须一起跟着推，不能单独写死。
    //=========================================================================
    localparam integer SAMPLE_RATE_HZ = CLK_FREQ_HZ / FRAME_DIV;
    localparam integer SAMPLE_CNT_MAX = FRAME_DIV - 1;

    // 音频域复位同步
    reg rst_a1, rst_a2;
    always @(posedge clk_audio) begin
        rst_a1 <= ~rst_n;
        rst_a2 <= rst_a1;
    end
    wire rst_audio_sync = rst_a2;

    //------------------------------------------------------------------
    // 音符频率 → 频率字
    //   delta = freq * 2^32 / 48000
    //   片内序号：0~4 = 黑键 C# D# F# G# A#；5~11 = 白键 C D E F G A B
    //   第 n 片 = 第 0 片 << n（正好高 n 个八度，2:1 精确八度）
    //------------------------------------------------------------------
    function [31:0] calc_delta;
        input [31:0] freq;
        reg   [63:0] tmp;
        begin
            tmp        = {32'd0, freq} << 32;
            calc_delta = tmp / SAMPLE_RATE_HZ;
        end
    endfunction

    wire [31:0] delta [0:NUM_KEYS-1];   // 每键基础频率字（常数）
    reg  [31:0] dcur  [0:NUM_KEYS-1];   // 每键滑音中的当前频率字（未加移调）
    reg  [31:0] inc   [0:NUM_KEYS-1];   // 送进相位累加器的增量（已加移调 + 八度偏移）
    // ★ V32：每个声部自己的八度偏移（有符号 3 位）。声明在这里是因为
    //   下面"共享移调路径"要读它（赋值那段在后面，跟着 score_new / key_new 走）。
    reg  [2:0]  voice_oct [0:NUM_KEYS-1];
    reg  [4:0]  tr_now;                 // 已经生效的档位
    reg  [4:0]  tr_gi;                  // 轮询到第几个键
    reg  [17:0] tr_ratio_r;

    localparam [4:0] TRANS_DEF = 5'd12;        // 原调档位

    //------------------------------------------------------------------
    // 2^((i-12)/12) 的 Q16 定点（i=12 -> 1.0）
    //------------------------------------------------------------------
    function [17:0] trans_ratio;
        input [4:0] i;
        begin
            case (i)
                5'd0 : trans_ratio = 18'd32768;    // -12 半音 = 0.5
                5'd1 : trans_ratio = 18'd34719;
                5'd2 : trans_ratio = 18'd36785;
                5'd3 : trans_ratio = 18'd38968;
                5'd4 : trans_ratio = 18'd41285;
                5'd5 : trans_ratio = 18'd43740;
                5'd6 : trans_ratio = 18'd46341;
                5'd7 : trans_ratio = 18'd49097;
                5'd8 : trans_ratio = 18'd52016;
                5'd9 : trans_ratio = 18'd55109;
                5'd10: trans_ratio = 18'd58386;
                5'd11: trans_ratio = 18'd61858;
                5'd12: trans_ratio = 18'd65536;    // 原调 = 1.0
                5'd13: trans_ratio = 18'd69433;
                5'd14: trans_ratio = 18'd73562;
                5'd15: trans_ratio = 18'd77936;
                5'd16: trans_ratio = 18'd82570;
                5'd17: trans_ratio = 18'd87480;
                5'd18: trans_ratio = 18'd92682;
                5'd19: trans_ratio = 18'd98193;
                5'd20: trans_ratio = 18'd104032;
                5'd21: trans_ratio = 18'd110218;
                5'd22: trans_ratio = 18'd116772;
                5'd23: trans_ratio = 18'd123716;
                5'd24: trans_ratio = 18'd131072;   // +12 半音 = 2.0
                default: trans_ratio = 18'd65536;
            endcase
        end
    endfunction

    //==================================================================
    // 音阶表：三片键盘【连续覆盖 A2 ~ G#4】（36 键 = 3 个八度）
    //
    //   物理排布（每片从左到右）：白 黑 白 黑 白 白 黑 白 黑 白 黑 白
    //     白键位 1、3、5、6、8、10、12      黑键位 2、4、7、9、11
    //     ★ 每个黑键 = 它【左边那个白键】升半音（和钢琴一样）
    //
    //   TTP229 模块丝印键号与物理位置是错开的：
    //     模块键 1~5  = 黑键 -> 物理位 2、4、7、9、11
    //     模块键 6~12 = 白键 -> 物理位 1、3、5、6、8、10、12
    //   所以每条后面都标了【模块键号】，接线对照时看这个。
    //
    //   三片关系：琴键2 = 琴键1 +1 个八度，琴键3 = +2 个八度（×2 / ×4）
    //   音名 -> 频率：A4 = 440Hz 基准，freq = 110 × 2^(n/12)
    //   delta = freq × 2^32 / SAMPLE_RATE_HZ
    //==================================================================

    // ---- 琴键1（键 1~12）：A2 ~ G#2 ----
    assign delta[0]  = calc_delta(32'd117); // 键1  模块键1  黑    A#2   117 Hz
    assign delta[1]  = calc_delta(32'd131); // 键2  模块键2  黑    C3   131 Hz
    assign delta[2]  = calc_delta(32'd156); // 键3  模块键3  黑    D#3   156 Hz
    assign delta[3]  = calc_delta(32'd175); // 键4  模块键4  黑    F3   175 Hz
    assign delta[4]  = calc_delta(32'd196); // 键5  模块键5  黑    G3   196 Hz
    assign delta[5]  = calc_delta(32'd110); // 键6  模块键6  白    A2   110 Hz
    assign delta[6]  = calc_delta(32'd123); // 键7  模块键7  白    B2   123 Hz
    assign delta[7]  = calc_delta(32'd139); // 键8  模块键8  白    C#3   139 Hz
    assign delta[8]  = calc_delta(32'd147); // 键9  模块键9  白    D3   147 Hz
    assign delta[9]  = calc_delta(32'd165); // 键10 模块键10 白    E3   165 Hz
    assign delta[10] = calc_delta(32'd185); // 键11 模块键11 白    F#3   185 Hz
    assign delta[11] = calc_delta(32'd208); // 键12 模块键12 白    G#3   208 Hz

    // ---- 琴键2（键 13~24）：A3 ~ G#3 = 琴键1 的 2 倍 ----
    assign delta[12] = calc_delta(32'd233); // 键13 模块键1  黑    A#3   233 Hz
    assign delta[13] = calc_delta(32'd262); // 键14 模块键2  黑    C4   262 Hz
    assign delta[14] = calc_delta(32'd311); // 键15 模块键3  黑    D#4   311 Hz
    assign delta[15] = calc_delta(32'd349); // 键16 模块键4  黑    F4   349 Hz
    assign delta[16] = calc_delta(32'd392); // 键17 模块键5  黑    G4   392 Hz
    assign delta[17] = calc_delta(32'd220); // 键18 模块键6  白    A3   220 Hz
    assign delta[18] = calc_delta(32'd247); // 键19 模块键7  白    B3   247 Hz
    assign delta[19] = calc_delta(32'd277); // 键20 模块键8  白    C#4   277 Hz
    assign delta[20] = calc_delta(32'd294); // 键21 模块键9  白    D4   294 Hz
    assign delta[21] = calc_delta(32'd330); // 键22 模块键10 白    E4   330 Hz
    assign delta[22] = calc_delta(32'd370); // 键23 模块键11 白    F#4   370 Hz
    assign delta[23] = calc_delta(32'd415); // 键24 模块键12 白    G#4   415 Hz

    //------------------------------------------------------------------
    // ★ 摇杆移调：inc[g] = dcur[g] × 2^((trans-12)/12)
    //
    //   trans_audio = 0~24，12 = 原调，每一格 = 1 个半音。
    //   ★ 分工：滑音只在 dcur[] 里做，目标永远是【常数】delta[g]；
    //     移调单独在 inc[] 里做，只用一个乘法器轮询算。
    //     两者拼起来 = 边滑边移调，功能一个不少。
    //
    //   ★ 只多用了一个乘法器，而且是分时复用的：
    //     档位不变时状态机完全停着，摇杆动一下才跑一轮刷新
    //     （36 个键各占 1 拍 = 36 拍）。所以它既不占时序也不费功耗。
    //
    //   精度：delta 先截成高 18 位再乘，等效频率误差约 2^-18
    //         ≈ 0.007 音分，完全听不出来。
    //------------------------------------------------------------------

    // 当前轮到的那个键（取高 18 位，精度约 0.1 音分）
    wire [17:0] tr_dsel = dcur[tr_gi] >> 14;

    // ★ 全工程共用的【那一个】移调乘法器（18×18）
    wire [35:0] tr_prod   = tr_dsel * tr_ratio_r;
    // ★ 等价改写：原来是「34 位结果赋给 32 位」被静默截掉高 2 位（EX3791 告警）。
    //   直接写位选 tr_prod[33:2]，取值完全一样，告警消失。
    wire [31:0] tr_result = tr_prod[33:2];     // ≡ dcur × ratio / 65536

    //------------------------------------------------------------------
    // ★★ V32：事件八度偏移 —— 在【这条共享路径】上加一次移位就行 ★★
    //
    //   一个八度就是频率 ×2，也就是相位增量左移 1 位（降八度就是右移）。
    //   而 inc[] 本来就已经是"dcur × 移调比例"算出来的运行时可变量，
    //   所以这里只是在乘法结果后面再接一级桶形移位（±3 位），
    //   36 路音频路径一个字都不用动 —— 这是最省的做法。
    //
    //   ★ 但"哪一位偏移"必须【跟着声部自己那次起音】走（voice_oct[]），
    //     不能用当前事件的那个全局值：inc[] 是所有声部共用的，
    //     一改，正在放释音尾巴的音也会被一起搬八度（尾巴会突然跳音）。
    //     所以这里读的是 voice_oct[tr_gi] —— 轮询轮到谁就用谁自己的偏移。
    //------------------------------------------------------------------
    wire [2:0] oct_of_gi = voice_oct[tr_gi];
    // 有符号 3 位 -> 移位量（0..4）和方向
    wire [2:0] oct_mag   = oct_of_gi[2] ? (~oct_of_gi + 3'd1) : oct_of_gi;
    wire       oct_down  = oct_of_gi[2];
    wire [31:0] tr_oct   = oct_down ? (tr_result >> oct_mag)
                                    : (tr_result << oct_mag);

    //------------------------------------------------------------------
    // ★ 轮询不停机：每个音频时钟算一个键，36 拍（2.9us）刷完全部。
    //   比滑音的更新速率（每个采样周期才动一次）快十几倍，所以 dcur 一动
    //   inc 立刻跟上，不需要任何"变化检测"。
    //   代价只有 1 个乘法器，路径也很短（乘法+移位直接写寄存器）。
    //------------------------------------------------------------------
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            tr_now     <= 5'd31;               // 非法值 -> 上电必刷一轮
            tr_gi      <= 5'd0;
            tr_ratio_r <= 18'd65536;
        end else begin
            if (trans_audio != tr_now) begin
                tr_now     <= trans_audio;
                tr_ratio_r <= trans_ratio(trans_audio);
            end
            tr_gi <= (tr_gi == NUM_KEYS-1) ? 5'd0 : (tr_gi + 1'b1);
        end
    end

    //------------------------------------------------------------------
    // ★ inc[] 必须用 generate 展开成 36 个【独立寄存器】。
    //
    //   千万别写成 inc[tr_gi] <= ... 这种变量下标赋值 ——
    //   综合器会把整个数组当成寄存器堆，36 路并行读各挂一个 36:1
    //   选择器，光是选择器就要多烧几千个 LUT。
    //   展开之后每个键是固定下标，相位累加那边读写都不需要选择器。
    //
    //   复位值直接填原调，避免 X 传播进相位累加器。
    //------------------------------------------------------------------
    genvar gt;
    generate
        for (gt = 0; gt < NUM_KEYS; gt = gt + 1) begin : gen_tune
            always @(posedge clk_audio or negedge rst_n) begin
                if (!rst_n)             inc[gt] <= delta[gt];
                else if (tr_gi == gt)   inc[gt] <= tr_oct;   // ★ V32：带上本声部的八度偏移
            end
        end
    endgenerate

    // ---- 琴键3（键 25~36）：A4 ~ G#4 = 琴键1 的 4 倍 ----
    assign delta[24] = calc_delta(32'd466); // 键25 模块键1  黑    A#4   466 Hz
    assign delta[25] = calc_delta(32'd523); // 键26 模块键2  黑    C5   523 Hz
    assign delta[26] = calc_delta(32'd622); // 键27 模块键3  黑    D#5   622 Hz
    assign delta[27] = calc_delta(32'd698); // 键28 模块键4  黑    F5   698 Hz
    assign delta[28] = calc_delta(32'd784); // 键29 模块键5  黑    G5   784 Hz
    assign delta[29] = calc_delta(32'd440); // 键30 模块键6  白    A4   440 Hz
    assign delta[30] = calc_delta(32'd494); // 键31 模块键7  白    B4   494 Hz
    assign delta[31] = calc_delta(32'd554); // 键32 模块键8  白    C#5   554 Hz
    assign delta[32] = calc_delta(32'd587); // 键33 模块键9  白    D5   587 Hz
    assign delta[33] = calc_delta(32'd659); // 键34 模块键10 白    E5   659 Hz
    assign delta[34] = calc_delta(32'd740); // 键35 模块键11 白    F#5   740 Hz
    assign delta[35] = calc_delta(32'd831); // 键36 模块键12 白    G#5   831 Hz

    //------------------------------------------------------------------
    // 采样使能（★ V49：采样率 97.656 kHz；时间语义见下面的 ctrl_en）
    //------------------------------------------------------------------
    reg [15:0] sample_cnt;
    reg        sample_en;

    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            sample_cnt <= 16'd0;
            sample_en  <= 1'b0;
        end else begin
            if (sample_cnt >= SAMPLE_CNT_MAX) begin
                sample_cnt <= 16'd0;
                sample_en  <= 1'b1;
            end else begin
                sample_cnt <= sample_cnt + 1'b1;
                sample_en  <= 1'b0;
            end
        end
    end

    //------------------------------------------------------------------
    // ★★★ V49：控制节拍 ctrl_en（= 原来 48.828 kHz 的"时间语义"）★★★
    //
    //   采样率翻倍后（97.656 kHz），如果什么都不做，所有【按采样点计时】
    //   的东西都会快一倍：包络衰减时间减半、滑音快一倍、颤音/合唱/延迟/
    //   混响的时间全部减半、LCD 电平扫描快一倍。
    //
    //   所以把"时间类"逻辑统一挂到控制节拍上（每 2 个采样点一次 =
    //   48.828 kHz 等效），【听感与 V48 逐项一致】：
    //       包络/门控、滑音、增益斜坡、颤音、
    //       合唱/延迟/混响/Haas 的读写推进、LCD 电平扫描
    //   而"内容类"逻辑仍然全速跑 97.656 kHz：
    //       36 路相位累加器、波表读相位、逐键包络乘法、S1/S2 组装、
    //       KS 弦（它按 sample_cnt 的 0~7 拍推进，改采样率后
    //       NI/FR 参数表按新 fs 重新生成，见 tools\ks_model.py）
    //
    //   ★ 相位是确定的：ctrl_ph 每个 sample_en 翻转一次，
    //     所以 ctrl_en 落在第 1、3、5… 个采样点上，开机即可复现。
    //------------------------------------------------------------------
    reg ctrl_ph;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) ctrl_ph <= 1'b0;
        else if (sample_en) ctrl_ph <= ~ctrl_ph;
    end
    wire ctrl_en = sample_en && ctrl_ph;

    //==================================================================
    // ★★ 诊断开关：TEST_SINE ★★（放在音频域开头，后面多处会用到）
    //   1'b1 → 完全绕过包络 / 增益 / 谐波 / AGC / 换调，直接输出一个
    //          固定音量的纯正弦，用来把"音调数据产生链"和"包络混音链"
    //          彻底分开定位：
    //     听到干净稳定的纯音 → I2S + DAC + 时钟是好的，
    //                          问题在被绕开的那一段里；
    //     仍然嘶哑/断续      → 问题在 I2S / DAC / 时钟本身。
    //   测完务必改回 1'b0 再出正式版。
    //==================================================================
    // ★ V17 诊断已用完，恢复 1'b0。
    //   经过它确认：显示通路（跨时钟 + wave_buf + 绘制）完全正常，
    //   "正弦 + 和 X 轴重叠的直线" 的真正原因是 lcd_master.v 里
    //   固定纵向刻度 `v >>> 7` 的【绝对值地板 128】把弱信号压成 0。
    //   已改成自动刻度，不再需要这个诊断。
    localparam TEST_SINE = 1'b0;   // ★ V19：正式发声模式（诊断已结束）★

    // 测试音相位累加（每个采样点走一步）
    //   频率 = SAMPLE_RATE_HZ × TEST_INC / 2^32
    //   TEST_INC = 2^24 â†’ 43402 Ã— 2^24 / 2^32 â‰ˆ 169.6 Hz
    reg [31:0] test_phase;
    localparam [31:0] TEST_INC = 32'd16777216;

    always @(posedge clk_audio) begin
        if (rst_audio_sync) test_phase <= 32'd0;
        else if (ctrl_en)  test_phase <= test_phase + TEST_INC;
    end

    //==================================================================
    // NUM_KEYS 路 ADSR 包络
    //
    //   参考 ADSR1 的思路，但补齐了 Sustain 电平，并且解决了它的破音问题。
    //
    //   状态流转（每个键独立）：
    //     按下      ：从当前电平快速 Attack 冲到 255
    //     继续按住  ：以 Decay 速率衰减到 SUSTAIN_LV，然后保持
    //     松开      ：以 Release 速率衰减到 0（这就是"延音"尾巴）
    //     尾巴没结束时再按下：从当前电平接着往上冲（不断音、不跳变）
    //
    //   时间（采样率 43403Hz，当前参数下）：
    //     Attack  ：255/8 = 32 个采样点 ≈ 0.74ms
    //     Decay   ：(255-0)×280 = 71400 个采样点 ≈ 1.64s（SUSTAIN_LV=0 时）
    //     Release ：同样速率，从当前电平衰减到 0
    //
    //   ★ 参数说明：
    //     ENV_SUSTAIN_LV = 0  →  按住时会一直衰减到静音（钢琴式，当前设置）
    //     ENV_SUSTAIN_LV = 200 → 按住时衰减到 200 就保持（约 -2dB 的持续音）
    //     ENV_SUSTAIN_LV = 255 →  完全没有衰减段，按住恒定为满幅
    //==================================================================
    // ★ 恢复成 25k K44 那版的取值（那版包络/音色是能正常发声的基准）。
    //   曾经试过把台阶降到 1 想让音头更平滑，结果 60k 上发声反而不对，
    //   所以这里保持和 K44 完全一致，不要再单独改这一个数。
    localparam [7:0]  ENV_ATTACK_STEP = 8'd8;      // 每次采样上升量
    // ★ V18 诊断：ENV_SUSTAIN_LV 0 -> 255，验证 DAC 上那条直线是不是"衰减到静音"
    //
    //   现象（示波器实测 DAC 输出）：一段正弦 + 一条直线。
    //   已确认 sine_rom.v 的表是【完整 256 点整周期正弦】：
    //   512 点逐点核对与 32767*sin(2*pi*k/256) 误差全在 2% 内，
    //   没有任何连续相等的点（无平顶）—— 所以不是"表只存了部分点"。
    //
    //   那么"直线"只能来自表之后的某级把输出变成常数。最大嫌疑是包络：
    //     ENV_SUSTAIN_LV = 0 -> 按住期间 env 会【一直衰减到 0】，
    //     env 归 0 后输出恒为 0 -> 示波器上就是一条直线。
    //     （钢琴式衰减：256 级 x 280 采样点 ~ 1.46 秒走完）
    //
    //   改 255 = 完全不衰减，按住就是恒定满包络 -> 应该是一条纯正弦。
    //   用示波器看 DAC：
    //     · 直线消失、变成连续正弦 -> 就是包络衰减，收工
    //       （想保留一点衰减感，把这里改成 200 之类：按住有音 + 轻微衰减）
    //     · 直线还在 -> 包络不是原因，下一级查 S1~S4 组装
    localparam [7:0]  ENV_SUSTAIN_LV  = 8'd255;    // ★ 诊断值，原为 8'd0      // æŒ‰ä½æ—¶çš„ä¿æŒç”µå¹³ï¼ˆ0 = é’¢ç´å¼ï¼šä¸€ç›´è¡°å‡åˆ°é™éŸ³ï¼‰
    localparam [15:0] ENV_DECAY_DIV   = 16'd280;   // 每多少个采样点衰减 1
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

    //==================================================================
    // ★★ V30：吉他音色（Karplus-Strong 弦模型）的包络参数 ★★
    //   吉他和笛子不一样：弦自己的能量衰减在 ks_string 里算（每绕延迟线一圈
    //   乘一次 1-2^-DP），所以这里【只动释音】—— 抬手后让包络慢慢放掉，
    //   剩下的余韵交给弦模型自己响完。
    //       释音 255→0 = 255 级 × 420 点 ≈ 107100 点 ≈ 2.19s
    //   ★ 这个值不能小：调小了会把弦的余音一起掐掉，听起来就"没有吉他味"。
    //==================================================================
    localparam [15:0] ENV_REL_DIV_GTR = 16'd420;

    //==================================================================
    // ★★ V48.2：小号音色（tone_sel = 3）的包络参数 ★★
    //   ★ 只在这里加常数，上面所有 ENV_* 一个都没改 ——
    //     普通/笛子/吉他三档【逐位不受影响】（小号没选中时那几个三目
    //     取到的还是原来的值，看下面 env_* 那几行的写法）。
    //
    //   小号要的是"吹出来的"感觉，和拨弦/吹管都不一样：
    //     起音 255/8 = 32 拍 × 4 点 ≈ 128 点 ≈ 2.6ms
    //       —— 比普通(1 点)稍慢一点点、比笛子(46 点)快得多；
    //          这一点点起音时间是"铜管起振"的听感来源，0 就变成电子音了
    //     保持 255 = 满幅
    //       —— 铜管按住就是稳稳一条，不像拨弦会自己衰下去
    //     释音 255→0 = 255 级 × 140 点 ≈ 35700 点 ≈ 731ms
    //       —— 比普通(280)短一半：松手/松开就收，像真的把气断了
    //==================================================================
    localparam [5:0]  ENV_ATK_DIV_TRP = 6'd4;       // 起音节拍分频（一点点起振时间）
    localparam [7:0]  ENV_SUS_LV_TRP  = 8'd255;     // 按住保持电平（满幅）
    localparam [15:0] ENV_DEC_DIV_TRP = 16'd280;    // 衰减速度（和普通一致）
    localparam [15:0] ENV_REL_DIV_TRP = 16'd140;    // 释音（比普通干脆一半）

    // 三目选择：吉他 -> 笛子 -> 普通。后两支的取值一个字没改，
    // 普通那一支仍然逐位等于原来的 ENV_RELEASE_DIV。
    // ★★ V48.2：小号那一支【写在最后】，判据是 trumpet_on_audio。
    //    ★ 写法上刻意保持"原来两支的判断顺序和取值一个字没动"：
    //      只要 trumpet_on_audio = 0，这三行取到的就是和以前完全相同的值。
    wire [5:0]  env_atk_div = trumpet_on_audio ? ENV_ATK_DIV_TRP :
                              flute_on_audio   ? ENV_ATK_DIV_FLUTE : 6'd1;
    wire [7:0]  env_sus_lv  = trumpet_on_audio ? ENV_SUS_LV_TRP  :
                              flute_on_audio   ? ENV_SUS_LV_FLUTE  : ENV_SUSTAIN_LV;
    wire [15:0] env_dec_div = trumpet_on_audio ? ENV_DEC_DIV_TRP :
                              flute_on_audio   ? ENV_DEC_DIV_FLUTE : ENV_DECAY_DIV;
    wire [15:0] env_rel_div = trumpet_on_audio ? ENV_REL_DIV_TRP :
                              guitar_on_audio  ? ENV_REL_DIV_GTR :
                              flute_on_audio   ? ENV_REL_DIV_FLUTE : ENV_RELEASE_DIV;

    reg [7:0]  env [0:NUM_KEYS-1];

    reg [15:0] dec_cnt, rel_cnt;
    wire dec_tick = (dec_cnt >= env_dec_div - 1'b1);
    wire rel_tick = (rel_cnt >= env_rel_div - 1'b1);

    // ★ 起音节拍：现有音色 env_atk_div=1 -> att_tick 恒为 1，等于没有这一级
    //   （att_cnt 一直停在 0），所以现有音色的起音【逐位不变】。
    reg  [5:0] att_cnt;
    wire       att_tick = (att_cnt >= env_atk_div - 1'b1);
    always @(posedge clk_audio) begin
        if (rst_audio_sync) att_cnt <= 6'd0;
        else if (ctrl_en) att_cnt <= att_tick ? 6'd0 : (att_cnt + 1'b1);
    end

    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            dec_cnt <= 16'd0;
            rel_cnt <= 16'd0;
        end else if (ctrl_en) begin
            dec_cnt <= dec_tick ? 16'd0 : (dec_cnt + 1'b1);
            rel_cnt <= rel_tick ? 16'd0 : (rel_cnt + 1'b1);
        end
    end

    //------------------------------------------------------------------
    // ★ 根治"同一个音被重弹一次（回声）"用的 gate 去抖慢拍
    //
    //   包络靠 attacking 标志区分"还在攻击"和"攻击已完成"。
    //   原来只要 sounding_mask_aud 有一拍为 0（该键读到松开）就立刻清 attacking，
    //   键位一恢复 env 马上重新 attack → 同一个音再响一次，听感就是"回声"。
    //   而 key_mask 的瞬间掉落常常来自【单轮坏读数】（TTP229 被干扰读错），
    //   并不是真的松手 —— 它同时也是"回声只在快速连按时明显"的原因：
    //   按得慢时，掉落发生在音符已经衰减到接近 0 的时候，重弹听不出来。
    //
    //   根治办法：松手必须【去抖】—— 连续低满 8ms 才承认。
    //     未确认期间 env 和 attacking 全部保持不动：
    //     音符不断、相位不重置、更不会重新攻击。
    //   8ms 的判据：1ms 慢拍计到 bit3（值>=8）。单轮坏读数 2.26ms 最多计 3，
    //   连续 3 轮坏读数(6.8ms)最多计 7，都到不了 8 → 一律滤掉。
    //   而人最快的连续重复弹奏间隔也远大于 8ms，真正的重复按键不会被吞。
    //------------------------------------------------------------------
    localparam [5:0] ENV_GATE_TICK_DIV = 6'd42;    // 43 个采样点 @43402Hz ≈ 0.99ms

    reg  [5:0] gate_tick_cnt;
    wire       gate_tick = (gate_tick_cnt >= ENV_GATE_TICK_DIV);

    always @(posedge clk_audio) begin
        if (rst_audio_sync) gate_tick_cnt <= 6'd0;
        else if (ctrl_en) gate_tick_cnt <= gate_tick ? 6'd0 : (gate_tick_cnt + 1'b1);
    end

    genvar ge;
    generate
        for (ge = 0; ge < NUM_KEYS; ge = ge + 1) begin : gen_env
            // ★ attacking：显式记住"攻击已经完成"。
            //   不能拿 env < 255 当判据，否则衰减走出一步(255→254)后判据又成立 →
            //   立刻重新攻击回 255，包络永远停在满幅、长按变成一直响。
            reg attacking;
            // ★ gate_off_cnt：gate 已经连续低了多少个 1ms 拍（4 位饱和）。
            //   bit3 = 1 表示已连续低满 8ms → 确认真的松手了。
            reg [3:0] gate_off_cnt;

            // ★★★ V47.2：这个键这一拍的【包络峰值】= 逐音力度 ★★★
            //
            //   · 力度 127（满幅 = 不带 0xC3 的默认）或【不是乐谱弹的音】→ 255
            //     → 下面那句 `env[ge] >= 255 - ENV_ATTACK_STEP` 逐字等于原代码，
            //       综合出来的逻辑【逐位相同】。手弹和关掉力度时完全不受影响。
            //   · 否则 = 力度×2|1（0→1，127→255，64→129），把这个音自己的
            //     包络峰值降下来。
            //
            //   ★★ 为什么这才是对的做法：
            //      力度缩放的是【这个音自己的包络峰值】—— 起音就从正确的音量开始，
            //      【没有任何增益跳变】，所以不会有"噗"；
            //      而且它完全不碰混音总增益，正在延音的音一个比特都不动。
            //   ★ 不需要任何新存储器：midi_keys_aud 在整个音期间保持为 1，
            //     midi_vel_aud 在整个事件期间也不变，所以这是纯组合逻辑。
            wire [7:0] vel_peak_k = (midi_keys_aud[ge] && (midi_vel_aud != 7'd127))
                                    ? ({midi_vel_aud, 1'b0} | 8'd1)
                                    : 8'd255;

            always @(posedge clk_audio) begin
                if (rst_audio_sync) begin
                    env[ge]       <= 8'd0;
                    attacking     <= 1'b0;
                    gate_off_cnt  <= 4'd0;
                end else if (ctrl_en) begin
                    if (sounding_mask_aud[ge]) begin
                        // ---- 发声（物理按下 或 延音锁存）----
                        gate_off_cnt <= 4'd0;        // 键还在 → 去抖计数复位
                        if (sustain_latch_aud[ge]) begin
                            // ★ 延音锁存的键：不衰减，钉在满幅
                            //   （用户要求"点过的琴键一直响"，包络等于旁路）
                            env[ge]   <= 8'd255;
                            attacking <= 1'b1;
                        end else if (!attacking) begin
                            // Attack：冲到顶，到顶后置标志
                            // ★ 笛子模式用 att_tick 放慢起音；现有音色下
                            //   env_atk_div=1 -> att_tick 恒为 1 -> 逐位不变
                            if (att_tick) begin
                                // ★ V47.2：攻击上限从【常量 255】换成这个键自己的
                                //   vel_peak_k。力度=127 或手弹时 vel_peak_k 恒为 255，
                                //   这两句逐字等于原代码 —— 逻辑逐位相同。
                                if (env[ge] >= vel_peak_k - ENV_ATTACK_STEP) begin
                                    env[ge]   <= vel_peak_k;
                                    attacking <= 1'b1;
                                end else
                                    env[ge]   <= env[ge] + ENV_ATTACK_STEP;
                            end
                        end else if (env[ge] > env_sus_lv) begin
                            // Decay：从峰值衰减到 Sustain 电平（现在是 255 = 不衰减，笛子是 200）
                            if (dec_tick) env[ge] <= env[ge] - 8'd1;
                        end
                        // 到达 Sustain 后保持不动
                    end else begin
                        // ---- gate 掉了：先别急着当"松手" ----
                        // ★★ 这里就是"重弹/回声"的根治点 ★★
                        //   原来无条件 attacking <= 1'b0，单轮坏读数（2.26ms）也会清掉它；
                        //   键位一恢复 env 就重新 attack → 同一个音再响一次。
                        //   现在必须连续低满 8ms 才承认松手；未确认期间
                        //   env 和 attacking 全部保持 → 音符不断、相位不重置、不重弹。
                        if (gate_tick && !gate_off_cnt[3])
                            gate_off_cnt <= gate_off_cnt + 1'b1;

                        if (gate_off_cnt[3]) begin
                            attacking <= 1'b0;
                            if (rel_tick && env[ge] > 8'd0)
                                env[ge] <= env[ge] - 8'd1;
                        end
                    end
                end
            end
        end
    endgenerate

    //------------------------------------------------------------------
    // "正在发声"判定：env != 0（含延音尾巴）
    //   ★ 混音门控要用它，而不是用 key_mask ——
    //     否则松开按键的瞬间延音尾巴会被直接掐掉。
    //
    //   ★★ 但它【不能拿来门控相位累加器】★★ —— 见下面相位累加那段的说明。
    //------------------------------------------------------------------
    wire [NUM_KEYS-1:0] sounding_mask;
    generate
        for (ge = 0; ge < NUM_KEYS; ge = ge + 1) begin : gen_snd
            assign sounding_mask[ge] = (env[ge] != 8'd0);
        end
    endgenerate

    //==================================================================
    // ★★ V30：送给 LCD 的"每键电平"改成【一根一根扫描式】送 ★★
    //
    //   原来：key_levels[35:0] 是 36 位并行（谁在响 = 1），LCD 那边再自己
    //         编成柱高。问题是"响就是响"，按下去一排等高柱子，不像频谱。
    //   现在：音频域把 36 个键各自的包络值 env[g] 一根一根轮流送出去 ——
    //         就像 TTP229 键盘那样，一次只送一个：
    //             (6 位键号 + 8 位电平 + 1 位握手) = 15 根线
    //         LCD 侧收到后写进自己那 36 项小表，画到哪一列就读哪一项，
    //         于是每根柱子各按【自己那个键】的包络长高、变矮。
    //
    //   配对规则：scan_val 和"下一拍的 scan_idx"是同拍更新的，
    //     所以任意时刻读出的 (idx, val) 必定是配对的；
    //     接收侧在握手翻转沿之后才采，那时两个信号都已经稳了(>= 2 拍)。
    //
    //   速率：每键每 (SCAN_DIV+1) 个采样周期，
    //         36 键一轮 = 36*(SCAN_DIV+1)/48.83kHz（控制节拍 ctrl_en，V49 后仍是这一档）
    //         SCAN_DIV=7 -> 5.9ms 一轮 -> 每键约 164 次/秒，够用了。
    //==================================================================
    localparam [5:0] SCAN_LAST = 6'd35;      // 0..35 共 36 个
    localparam [7:0] SCAN_DIV  = 8'd7;       // 每个键 8 个采样周期
    reg [5:0] scan_idx;
    reg [7:0] scan_val;
    reg       scan_tgl;
    reg [7:0] scan_div;

    wire [5:0] scan_next = (scan_idx >= SCAN_LAST) ? 6'd0 : scan_idx + 6'd1;

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            scan_idx <= 6'd0;
            scan_val <= 8'd0;
            scan_tgl <= 1'b0;
            scan_div <= 8'd0;
        end else if (ctrl_en) begin
            if (scan_div >= SCAN_DIV) begin
                scan_div <= 8'd0;
                scan_val <= env[scan_next];    // 和下一拍的 idx 配对
                scan_idx <= scan_next;
                scan_tgl <= ~scan_tgl;         // 告诉 50MHz 域：这一组有效
            end else begin
                scan_div <= scan_div + 8'd1;
            end
        end
    end

    //==================================================================
    // 滑音（Portamento）
    //
    //   开启后，每个新按下的键从【上一个音的音高】开始，
    //   以固定步长滑到自己的音高。音程越大滑得越久 —— 和真实滑音一致。
    //
    //   ★ 每个键有自己的一份"当前频率字" dcur：
    //       按下瞬间初始化为 last_delta（上一个音）
    //       之后每个采样点向 delta[g]（本音）靠近 GLIDE_STEP
    //   ★ 3/4 次谐波的相位是从 phase 派生的，所以会自动跟着一起滑。
    //==================================================================
    //   每一步的频率字增量。★ 只改这一个数就能调滑音速度：
    //       8192  → 一个八度约 145 ms（像"连奏"，很快）
    //       4096  → 一个八度约 290 ms
    //       2048  → 一个八度约 580 ms（当前值，听感最明显）  ★
    //       1024  → 一个八度约 1.16 s（很夸张的滑音）
    localparam [31:0] GLIDE_STEP = 32'd2048;   // 每采样点滑多少；调大=滑得更快

    // 滑音开关跨时钟（慢信号，2 级同步足够）
    reg glide_s1, glide_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            glide_s1 <= 1'b0;
            glide_s2 <= 1'b0;
        end else begin
            glide_s1 <= glide_en;
            glide_s2 <= glide_s1;
        end
    end
    wire glide_on = glide_s2;

    // 最近一次按下的键的频率字，作为下一个音的滑音起点
    reg [31:0] last_delta;

    // 音频域上升沿检测：这一拍刚按下的键
    reg  [NUM_KEYS-1:0] key_mask_d;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) key_mask_d <= {NUM_KEYS{1'b0}};
        else        key_mask_d <= key_mask;
    end
    wire [NUM_KEYS-1:0] key_new = key_mask & ~key_mask_d;

    //==================================================================
    // ★★ V30：吉他音色 —— 36 条 Karplus-Strong 弦（karplus/ks_string）★★
    //
    //   一条弦一个 ks_string 实例，全部在音频域，靠 sample_cnt 的 0~7 拍推进
    //   （每个采样点 8 个时钟）。36 条完全并行，谁也不等谁。
    //
    //   起振（拨弦）条件两条：
    //     · 这个键【刚按下】—— key_new 是"按下那一瞬间"的单拍脉冲。
    //       它来自 key_mask，所以物理琴键 / MIDI / 蓝牙乐谱按下都算，
    //       而且 V29 的按键映射对它是生效的（映射在 key_mask 之前就做完了）。
    //     · 或者【刚从别的音色切到吉他】那一拍，把正在按着的键也拨一下，
    //       否则切过去会是"按着没声音"，得松手重按才有。
    //
    //   弦长 / 插值小数 / 阻尼三个常数按键号在 ks_string 里【常数化】了
    //   （NI = NI_TAB(KEY) 之类），运行时既不查表也不做选择，
    //   每条弦只出自己的那一小份逻辑。
    //     ★ 代价与限制：弦长是常数，所以【滑音和移调对吉他都不起作用】
    //       （拨弦改弦长本来也不物理），吉他永远按琴键自己的音高发声。
    //==================================================================
    reg guitar_on_d;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) guitar_on_d <= 1'b0;
        else        guitar_on_d <= guitar_on_audio;
    end
    wire guitar_start = guitar_on_audio & ~guitar_on_d;

    wire signed [15:0] ks_smp [0:NUM_KEYS-1];

    genvar kg;
    generate
        for (kg = 0; kg < NUM_KEYS; kg = kg + 1) begin : gen_ks
            ks_string #(.KEY(kg)) u_ks (
                .clk        (clk_audio),
                .rst_n      (rst_n),
                .sample_cnt (sample_cnt),
                .run        (guitar_on_audio),
                .pluck      (key_new[kg] | (guitar_start & key_mask[kg])),
                .vel        (vel_audio),
                .smp        (ks_smp[kg])
            );
        end
    endgenerate

    //==================================================================
    // 延音锁存
    //
    //   延音打开后，每次【新点下一个琴键】就把该键 OR 进锁存；
    //   再点同一个键（它已在锁存里）就把它去掉；多个键叠加。
    //   关闭延音时锁存一次性清零（全部释放）。
    //
    //   ★ 用 key_new 做切换：它是"按下那一瞬间"的单拍脉冲，天然一次事件。
    //     若改用 key_mask 做 XOR，按住期间 key_mask 一直为 1，
    //     会每个时钟翻转一次，根本锁不住。
    //   ★ 放在音频域，避免跨时钟域引用 50MHz 的信号。
    //==================================================================
    reg [NUM_KEYS-1:0] sustain_latch_aud;

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            sustain_latch_aud <= {NUM_KEYS{1'b0}};
        end else if (!sustain_on_audio) begin
            sustain_latch_aud <= {NUM_KEYS{1'b0}};              // 关延音 → 全部释放
        end else if (key_new != {NUM_KEYS{1'b0}}) begin
            sustain_latch_aud <= sustain_latch_aud ^ key_new;   // 点了就加，再点就去掉
        end
    end

    // 真正驱动发声的掩码 = 物理按下的键 ∪ 延音锁存的键
    wire [NUM_KEYS-1:0] sounding_mask_aud = key_mask | sustain_latch_aud;

    integer ki;
    always @(posedge clk_audio) begin
        // ★ 这里绝对不能加 sample_en 门控！key_new 只有 1 拍宽，
        //   而 sample_en 每 128 拍（一个采样周期）才来一次，加门控后绝大多数按键会漏掉，
        //   last_delta 永远停在 0 → 滑音永远不触发。
        for (ki = 0; ki < NUM_KEYS; ki = ki + 1)
            if (key_new[ki]) last_delta <= delta[ki];
    end

    //------------------------------------------------------------------
    // NUM_KEYS 路相位累加器
    //   ★ 用 dcur（滑音中的当前频率字）而不是 delta
    //
    //   ★★ 这里原来用 sounding_mask（= env != 0）门控，是错的 ★★
    //
    //     原来的写法：
    //         if (sounding_mask[g]) phase[g] <= phase[g] + inc[g];
    //         else                  phase[g] <= 32'd0;
    //
    //     问题出在【衰减尾巴】：env 是 8bit，Release 阶段 env 从 1 掉到 0
    //     之后，"松手去抖"逻辑会让它一直停在 0，此时 sounding_mask = 0，
    //     于是 phase 被【反复清零】。而 I2S 和混音还在跑，
    //     结果每 280 个采样点就把正弦波的相位从任意位置硬跳回 0 ——
    //     听感就是：
    //        · 声音变得很不连续（相位反复跳变）
    //        · 结尾出现一个明显的"收尾声"（最后一次跳相）
    //     正确的做法是渐弱然后消失（包络自然衰减到 0）。
    //
    //     顺带一提 env=1..7 时 sounding_mask 仍是 1（判据是 != 0），
    //     所以真正被清相的只是 env 已经归零的那些拍。
    //
    //   ★ 正确做法（现在就按这个写）：
    //       · 音符【起始】才把相位清 0 —— 音头从 sin(0)=0 开始，干净；
    //       · 之后相位【持续累加】，衰减期间绝不跳相，
    //         音量完全交给包络（正弦 × env），结尾自然是渐弱消失；
    //       · 音符彻底静音（sounding_mask=0，即 env 归 0）后相位停在 0，
    //         这样不会自引用、也保证异常状态下能收回。
    //------------------------------------------------------------------
    reg [31:0] phase [0:NUM_KEYS-1];

    // 音符起始（单拍脉冲）→ 相位清 0
    wire [NUM_KEYS-1:0] phase_rst = key_new;

    genvar g;
    generate
        for (g = 0; g < NUM_KEYS; g = g + 1) begin : gen_phase
            // ★ V37：本键这次起音【将要用的】八度
            //     · 乐谱起音 → 这个键自己的逐键八度（0xCD 指定，或整格八度）
            //     · 手弹/其它 → 沿用 oct_s2（★ 手弹路径与 V36 逐位相同）
            //   滑音判据必须比的是"本键的新八度"：跨八度大跳时如果拿旧判据
            //   （整格八度）去比，就会从【另一个八度】滑上来 —— 那正是 V33
            //   修过的"大跳时音头跑到别的八度"以新形式复现。
            wire [2:0] oct_new_g = score_new[g] ? midi_octs_s2[g*3 +: 3] : oct_s2;
            always @(posedge clk_audio) begin
                if (rst_audio_sync) begin
                    phase[g] <= 32'd0;
                    dcur[g]  <= 32'd0;
                end else begin
                    // ---- 滑音起点：按键瞬间设定 ----
                    // ★ 这一段绝对不能放进 sample_en 里！
                    //   key_new 只有 1 拍宽，而 sample_en 每 128 拍（一个采样周期）才来一次，
                    //   放进去 99.7% 的按键会错过初始化，dcur 就一直等于
                    //   delta[g]（上电后斜坡早就把它拉到位了）→ 完全听不到滑音。
                    if (key_new[g])
                        // 开了滑音就从上一个音起滑；否则直接就位。
                        // last_delta==0 说明刚上电还没弹过，直接就位避免"从 0 滑上来"
                        //
                        // ★★ V33：八度偏移变了就【不起滑】，直接就位（这是个 BUG 修复）★★
                        //   last_delta 记的是【上一个音在它自己那个八度域里】的频率字；
                        //   而这一个音可能被事件八度偏移挪到了别处（inc = dcur×比例>>八度）。
                        //   两者八度不同还硬起滑的话，发声会从"上一个音被移了八度"的位置
                        //   开始爬 —— 听感就是大跳时音头跑到别的八度、像被截断/丢音。
                        //   八度相同（= 手弹、以及绝大多数同格偏移一样的情况）时行为
                        //   和以前【逐位相同】。
                        dcur[g] <= (glide_on && (last_delta != 32'd0)
                                    && (voice_oct[g] == oct_new_g))
                                   ? last_delta : delta[g];

                    // ★★ V49：这一段原来和相位累加共用 `if (sample_en)`，
                    //   采样率翻倍后必须拆开 —— 滑音是【时间语义】（听感
                    //   要跟 V48 一致）走 ctrl_en；相位累加是【内容】，
                    //   必须继续全速跑 97.656 kHz，否则音高会掉一半。
                    if (ctrl_en) begin
                        // ---- 滑音斜坡：向本音频率字靠近 ----
                        // ★ 目标必须是【常数】delta[g]！
                        //   一旦改成运行时寄存器，24 个键各要一套真的
                        //   32 位加减法器，实测多烧 2300 个 ALU。
                        //   移调不在这里做，放到后面的 inc[] 去乘比例。
                        if (!key_new[g]) begin
                            if (dcur[g] < delta[g]) begin
                                if (dcur[g] + GLIDE_STEP >= delta[g]) dcur[g] <= delta[g];
                                else                                  dcur[g] <= dcur[g] + GLIDE_STEP;
                            end else if (dcur[g] > delta[g]) begin
                                if (dcur[g] - GLIDE_STEP <= delta[g]) dcur[g] <= delta[g];
                                else                                  dcur[g] <= dcur[g] - GLIDE_STEP;
                            end
                        end
                    end

                    // ---- 相位累加（用已加移调的 inc[]）----
                    //   起始清 0（音头干净），之后只累加、绝不跳相。
                    //   env 归 0 后无音可发（din 会被算成 0），
                    //   相位每拍 +inc 会自然回绕，不影响任何东西。
                    //   ★ 这里必须是 sample_en（全速 97.656 kHz）：
                    //     一个采样点累加一次，音高才等于 delta[g]。
                    if (sample_en) begin
                        if (phase_rst[g])      phase[g] <= 32'd0;
                        else                   phase[g] <= phase[g] + inc[g];
                    end
                end
            end
        end
    endgenerate

    //------------------------------------------------------------------
    // NUM_KEYS 个 pROM（每个 512×32，占 1 块 BSRAM）
    //
    //   ★ 一个采样周期内把【同一张表】读 3 次，取到 4 个谐波，
    //     所以 BSRAM 仍然是 1 块/键（24 键 = 24 块，第 3 片照样装得下）。
    //     原理：一个采样周期有 128 个音频时钟（V49），而查一次表只要 2 拍。
    //
    //   表内容：entry[X] = { sine[X & 0xFF], sine[X >> 1] }
    //           （生成 sine_table_packed.mi 时定的，不要改这张表的格式）
    //
    //   读相位由 sample_cnt 前 3 拍决定（cnt=0 送地址，cnt=1 收到数据）：
    //     相位0  X = P[31:23]     低半字 = sine[P[31:24]]    = 1 次谐波
    //                             高半字 = sine[P[30:23]]    = 2 次谐波
    //     相位1  X = (3P)[31:23]  低半字 = sine[(3P)[31:24]] = 3 次谐波
    //     相位2  X = P[29:21]     低半字 = sine[P[29:22]]    = 4 次谐波
    //                             （因为 4P 的高 8 位正好是 P[29:22]）
    //------------------------------------------------------------------
    // 读相位：每 2 拍一个相位
    //   cnt=0,2,4,6 送地址；cnt=1,3,5,7 收到数据（ROM 有 1 拍延迟）
    wire [1:0] rdsel = sample_cnt[2:1];

    wire [31:0] rom_dout [0:NUM_KEYS-1];

    generate
        for (g = 0; g < NUM_KEYS; g = g + 1) begin : gen_rom
            // 3 倍相位：组合算，省掉一路 32 位累加器
            wire [31:0] p3_g = phase[g] + {phase[g][30:0], 1'b0};

            // ★ 4 个相位都把目标谐波放在【低半字】（低半字 = sine[X>>1]），
            //   所以地址取 X = 目标地址 << 1：
            //     相位0  X = P[31:23]              低半字 = 1 次谐波
            //     相位1  X = {P[30:23], 1'b0}      低半字 = 2 次谐波
            //     相位2  X = {(3P)[31:24], 1'b0}   低半字 = 3 次谐波
            //     相位3  X = {P[29:22], 1'b0}      低半字 = 4 次谐波
            wire [8:0] ad_g = TEST_SINE ? test_phase[31:23] :
                              (rdsel == 2'd0) ? phase[g][31:23]        :
                              (rdsel == 2'd1) ? {phase[g][30:23], 1'b0} :
                              (rdsel == 2'd2) ? {p3_g[31:24],     1'b0} :
                                                {phase[g][29:22],  1'b0};

            // 60k 版：用推断 ROM（sine_rom.v）替代 Gowin_pROM IP，
            //   数据与 sine_table_packed.mi 完全相同，同样是 1 拍读延迟。
            sine_rom u_rom (
                .clk   (clk_audio),
                .ad    (ad_g),
                .dout  (rom_dout[g])
            );
        end
    endgenerate

    //------------------------------------------------------------------
    // 诊断用测试正弦：取第 0 个 ROM 的低半字（= 基波表，256 点/周期）。
    //   幅度右移 2 位 → 满幅的 1/4，保证不削顶。
    //   （相位累加器 test_phase 与 TEST_SINE 定义在音频域开头）
    //------------------------------------------------------------------
    wire signed [15:0] test_sine_raw = $signed(rom_dout[0][15:0]);
    wire signed [15:0] test_sine_sample = (test_sine_raw >>> 2);

    //------------------------------------------------------------------
    // ★逐键乘包络★（ADSR 就落在这里）+ 每键只用 1 个乘法器
    //
    //   4 个读相位每个只出一个谐波值（都在低半字），所以每个相位
    //   只需要乘一次包络 —— 同一个乘法器在 4 个相位上复用。
    //   24 键 = 24 个乘法器（不是 48 个），DSP 占用从"28 个用满 + LUT 溢出"
    //   降到 24 个。
    //
    //   ★ 包络在【谐波混合之前】逐键施加，所以因式分解完全保留 ——
    //     力度仍然只需要 1 个全局乘法器：
    //         Î£ enváµ¢Â·(h1áµ¢ + cÂ·Háµ¢)  =  Î£ enváµ¢Â·h1áµ¢  +  c Â· Î£ enváµ¢Â·Háµ¢
    //     （如果改成"先求和再乘包络"，那变成全局包络，就不是逐键的了）
    //
    //   乘积最大 32767×255 = 8,355,585 < 2^23，取高 16 位即 ÷256。
    //------------------------------------------------------------------
    wire signed [15:0] env_a [0:NUM_KEYS-1];   // 当前相位的谐波 × 本键包络

    generate
        for (g = 0; g < NUM_KEYS; g = g + 1) begin : gen_unpack
            // ★ V30：吉他模式下每个键的"音源"从正弦 ROM 换成它自己那条弦的输出。
            //   两者都是 16 位有符号、同一个量级，所以后面（乘包络、求和、
            //   混响/合唱/延迟）一个字都不用改 —— 这也是为什么弦模型放在
            //   【包络之前】这一级：包络/力度/效果全都照旧作用在它身上。
            wire signed [15:0] src_g = guitar_on_audio ? ks_smp[g]
                                                       : $signed(rom_dout[g][15:0]);
            wire signed [16:0] s_raw = $signed(src_g);
            wire signed [23:0] s_p   = s_raw * $signed({1'b0, env[g]});
            assign env_a[g] = $signed(s_p[23:8]);
        end
    endgenerate

    //------------------------------------------------------------------
    // 当前读相位的「跨键求和」
    //
    //   每个读相位只出一个谐波（都在低半字，且已乘本键包络），
    //   所以这里做一个跨键和，再按读相位抓进 S1~S4：
    //     cnt=1 → S1（1 次谐波）   cnt=3 → S2（2 次谐波）
    //     cnt=5 → S3（3 次谐波）   cnt=7 → S4（4 次谐波）
    //
    //   ★★ 全局力度能做这么便宜，靠的是因式分解：
    //        Î£ enváµ¢Â·(h1áµ¢ + cÂ·Háµ¢)  =  Î£ enváµ¢Â·h1áµ¢  +  c Â· Î£ enváµ¢Â·Háµ¢
    //      力度 c 对所有键、所有谐波都是同一个值 → 先求和，最后只乘一次，
    //      所以无论 24 键还是 36 键、几个谐波，力度都只要 1 个乘法器。
    //
    //   ★ 用 sounding_mask 而不是 key_mask 门控：
    //     延音尾巴期间这一路必须继续参与混音，否则尾巴会被掐掉。
    //
    //   ★ 必须写成单个表达式（综合器自动搭 log2(N) 级平衡加法树），
    //     绝不能改成 for 循环串行累加 —— 那样会变成 N 级串行链，
    //     12.5MHz 下必然时序违例。
    //   ★PAD3-7  第 3 片（[24]~[35]）已追加
    //
    //   22 位足够：36 键 × 32767 = 1,179,612 < 2^21
    //------------------------------------------------------------------
    // ==================================================================
    // ★★ 优化（数学上完全等价，不改任何功能）★★
    //
    //   env_a[g] = (正弦值 × env[g]) >> 8，而 sounding_mask[g] 的定义
    //   恰好就是 (env[g] != 0)。也就是说：
    //       env[g] == 0  ->  sounding_mask[g] == 0  且  env_a[g] == 0
    //   所以原来的每一项
    //       sounding_mask[g] ? env_a[g] : 22'sd0
    //   恒等于 env_a[g] 本身 —— 这 36 个 22 位选择器是纯冗余。
    //   去掉之后：省 36×22 ≈ 790 个 LUT，加法树少一级扇出，
    //   sounding_mask 的扇出也大幅下降（它只再供 any_key / 屏幕频谱用）。
    //
    //   ★ 仍然是【单个表达式】，综合器照样自动搭 log2(N) 级平衡加法树，
    //     千万不要改成 for 循环串行累加。
    //   ★ 22 位足够：36 × 32767 = 1,179,612 < 2^21
    // ==================================================================
    wire signed [21:0] sum_env =
        env_a[0]  + env_a[1]  + env_a[2]  + env_a[3]  + env_a[4]  + env_a[5]  +
        env_a[6]  + env_a[7]  + env_a[8]  + env_a[9]  + env_a[10] + env_a[11] +
        env_a[12] + env_a[13] + env_a[14] + env_a[15] + env_a[16] + env_a[17] +
        env_a[18] + env_a[19] + env_a[20] + env_a[21] + env_a[22] + env_a[23] +
        env_a[24] + env_a[25] + env_a[26] + env_a[27] + env_a[28] + env_a[29] +
        env_a[30] + env_a[31] + env_a[32] + env_a[33] + env_a[34] + env_a[35];

    //------------------------------------------------------------------
    // 按读相位把跨键和抓成 4 个谐波总和
    //   S1 = Σ env·(1 次)   S2 = Σ env·(2 次)
    //   S3 = Σ env·(3 次)   S4 = Σ env·(4 次)
    //
    //   ★ 每个累加器一个采样周期只写一次，不需要清零。
    //   ★ V19: 这一级只负责"记满一拍就锁存"；它跟 I2S 帧没有固定相位关系，
    //     所以原来注释里"第 8 拍就全部就绪、I2S 在帧边界随时采都拿完整值"
    //     那句话并不成立 —— 真正保证一致的是
    //     【SAMPLE_CNT_MAX 与 FRAME_DIV 配套】（都等于 256）。
    //     改这三个常数时请一起推。
    //------------------------------------------------------------------
    reg signed [21:0] S1, S2, S3, S4;

    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            S1 <= 22'sd0; S2 <= 22'sd0; S3 <= 22'sd0; S4 <= 22'sd0;
        end else begin
            case (sample_cnt)
                16'd1: S1 <= sum_env;
                16'd3: S2 <= sum_env;
                16'd5: S3 <= sum_env;
                16'd7: S4 <= sum_env;
                default: ;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 统计按下的键数
    // ★ 必须写成"单个表达式"而不是 for 循环累加：
    //   for 循环会被综合成 N 级串行加法链，12.5MHz 下直接时序违例；
    //   写成单个表达式，综合器会自动搭成 log2(N) 级平衡加法树。
    // ★PAD3-7  第 3 片（key_mask[24]~key_mask[35]）已追加
    //------------------------------------------------------------------
    // ★★ 归一化增益：由「整数个数」改成「连续能量」
    //
    //   原来：n_keys = Σ(env[i] > 8)，是个【整数个数】，增益表又是 6dB 一档：
    //         1 个音 → 64    2 个音 → 32    3~16 个 → 16 ...
    //   后果：延音尾巴衰减到跨过 env=8 的那一拍，n_keys 少 1，增益瞬间翻倍(+6dB)。
    //         斜坡只有 0.37ms，等于一个台阶 —— 听感就是"正在响的那个音又被弹了一次"。
    //         这正是"前一个音的包络没结束时按下就会触发"：
    //           按下新键时旧音 env 还在 8 以上 → n_keys +1（音量先被压低 6dB）；
    //           等旧音尾巴掉到 8 以下     → n_keys -1（增益 +6dB 跳回去）。
    //         间隔长时旧音 env 早就是 0，所以只有"连续按、间隔短"才听得到。
    //
    //   改法：把每个键的包络【按比例】计入能量，尾巴衰减时能量连续下降，
    //         增益跟着连续上升，再也没有"某一刻突然跳 6dB"。
    //           env_sum = Σ (env[i] >> 5)       每键 0~7，36 键最大 252（8 位够）
    //         增益取 gain ≈ 64 / max(1, env_sum/7)，夹在 4~64。
    //         它在 1 / 2 / 4 个满幅音上与原来的档位基本一致（64 / 37 / 16），
    //         并且始终满足防削顶条件 gain <= 128/n，比原来的曲线更安全。
    //   ★PAD3-7  第 3 片（env[24]~env[35]）已追加
    //------------------------------------------------------------------
    //==================================================================
    // feed-forward auto gain (ported from music_test0 V7/V8)
    //
    //   G = min(1, C / L),  C = 65280,  L = sum of active envelopes
    //
    //   Why the clamp at 1 matters (this was music_test0 V7's bug, and it
    //   is almost certainly why the old table here produced tail crackle):
    //     * Without the clamp, a single note gives G = C/env, which cancels
    //       env out entirely -> the note stops decaying.
    //     * Worse, in the low-L region the neighbouring table entries differ
    //       by 30~60%.  Those steps land straight in the output: the envelope
    //       advances one entry every 8 steps (~46 ms) at the tail, i.e. a
    //       string of jumps = continuous crackle, worst at the end.
    //     * With the clamp: L <= 255 (one full-envelope note) -> G is exactly
    //       1, so that whole region has NO steps at all and the envelope
    //       decays linearly as intended.  Only L > 255 compresses, and there
    //       neighbouring entries differ by <= 1.5%.
    //
    //   C is chosen so the two regions join at L = 255:
    //       full-load out = h*C/(256*1024)  ==  single-note h*255/1024
    //     ->  C = 65280
    //
    //   36-key version: L is widened to 14 bits (36 x 255 = 9180 max).
    //   The table is generated; do NOT hand-edit.
    //==================================================================
    wire [13:0] env_load =
          ((env[0]  > 8'd8) ? {6'b0, env[0]}  : 14'd0)
        + ((env[1]  > 8'd8) ? {6'b0, env[1]}  : 14'd0)
        + ((env[2]  > 8'd8) ? {6'b0, env[2]}  : 14'd0)
        + ((env[3]  > 8'd8) ? {6'b0, env[3]}  : 14'd0)
        + ((env[4]  > 8'd8) ? {6'b0, env[4]}  : 14'd0)
        + ((env[5]  > 8'd8) ? {6'b0, env[5]}  : 14'd0)
        + ((env[6]  > 8'd8) ? {6'b0, env[6]}  : 14'd0)
        + ((env[7]  > 8'd8) ? {6'b0, env[7]}  : 14'd0)
        + ((env[8]  > 8'd8) ? {6'b0, env[8]}  : 14'd0)
        + ((env[9]  > 8'd8) ? {6'b0, env[9]}  : 14'd0)
        + ((env[10] > 8'd8) ? {6'b0, env[10]} : 14'd0)
        + ((env[11] > 8'd8) ? {6'b0, env[11]} : 14'd0)
        + ((env[12] > 8'd8) ? {6'b0, env[12]} : 14'd0)
        + ((env[13] > 8'd8) ? {6'b0, env[13]} : 14'd0)
        + ((env[14] > 8'd8) ? {6'b0, env[14]} : 14'd0)
        + ((env[15] > 8'd8) ? {6'b0, env[15]} : 14'd0)
        + ((env[16] > 8'd8) ? {6'b0, env[16]} : 14'd0)
        + ((env[17] > 8'd8) ? {6'b0, env[17]} : 14'd0)
        + ((env[18] > 8'd8) ? {6'b0, env[18]} : 14'd0)
        + ((env[19] > 8'd8) ? {6'b0, env[19]} : 14'd0)
        + ((env[20] > 8'd8) ? {6'b0, env[20]} : 14'd0)
        + ((env[21] > 8'd8) ? {6'b0, env[21]} : 14'd0)
        + ((env[22] > 8'd8) ? {6'b0, env[22]} : 14'd0)
        + ((env[23] > 8'd8) ? {6'b0, env[23]} : 14'd0)
        + ((env[24] > 8'd8) ? {6'b0, env[24]} : 14'd0)
        + ((env[25] > 8'd8) ? {6'b0, env[25]} : 14'd0)
        + ((env[26] > 8'd8) ? {6'b0, env[26]} : 14'd0)
        + ((env[27] > 8'd8) ? {6'b0, env[27]} : 14'd0)
        + ((env[28] > 8'd8) ? {6'b0, env[28]} : 14'd0)
        + ((env[29] > 8'd8) ? {6'b0, env[29]} : 14'd0)
        + ((env[30] > 8'd8) ? {6'b0, env[30]} : 14'd0)
        + ((env[31] > 8'd8) ? {6'b0, env[31]} : 14'd0)
        + ((env[32] > 8'd8) ? {6'b0, env[32]} : 14'd0)
        + ((env[33] > 8'd8) ? {6'b0, env[33]} : 14'd0)
        + ((env[34] > 8'd8) ? {6'b0, env[34]} : 14'd0)
        + ((env[35] > 8'd8) ? {6'b0, env[35]} : 14'd0);

    // gain_lut[k] = min(256, round(65280 / (8k+4)))      (256 = gain 1.0)
    // index = env_load[13:3]  (512 entries, each covering 8 L values)
    //
    // ★ V19 速度优化：加 syn_ramstyle 属性，让综合器把这个 512×10 的
    //   常表落成【块 RAM ROM】，而不是展开成 512:1 的 LUT MUX 树
    //   （几千个 LUT，是综合慢的主因之一）。
    //
    //   ★★ 初始化方式【一个字都没改】★★
    //      仍然用下面的 initial 逐项赋值，没有用 $readmemh。
    //      表内容始终写在 .v 文件里，【没有外部文件依赖】，
    //      不会因为综合时找不到 .mi 而导致表变空、音频坏掉。
    //      （之前改成 $readmemh + 外部 .mi 就是这么坏掉的。）
    //
    //   只加属性是零功能风险的：仿真行为与加之前逐位相同。
    (* syn_ramstyle = "block_ram" *) reg [9:0] gain_lut [0:511];
    initial begin
        gain_lut[9'd0] = 10'd256;
        gain_lut[9'd1] = 10'd256;
        gain_lut[9'd2] = 10'd256;
        gain_lut[9'd3] = 10'd256;
        gain_lut[9'd4] = 10'd256;
        gain_lut[9'd5] = 10'd256;
        gain_lut[9'd6] = 10'd256;
        gain_lut[9'd7] = 10'd256;
        gain_lut[9'd8] = 10'd256;
        gain_lut[9'd9] = 10'd256;
        gain_lut[9'd10] = 10'd256;
        gain_lut[9'd11] = 10'd256;
        gain_lut[9'd12] = 10'd256;
        gain_lut[9'd13] = 10'd256;
        gain_lut[9'd14] = 10'd256;
        gain_lut[9'd15] = 10'd256;
        gain_lut[9'd16] = 10'd256;
        gain_lut[9'd17] = 10'd256;
        gain_lut[9'd18] = 10'd256;
        gain_lut[9'd19] = 10'd256;
        gain_lut[9'd20] = 10'd256;
        gain_lut[9'd21] = 10'd256;
        gain_lut[9'd22] = 10'd256;
        gain_lut[9'd23] = 10'd256;
        gain_lut[9'd24] = 10'd256;
        gain_lut[9'd25] = 10'd256;
        gain_lut[9'd26] = 10'd256;
        gain_lut[9'd27] = 10'd256;
        gain_lut[9'd28] = 10'd256;
        gain_lut[9'd29] = 10'd256;
        gain_lut[9'd30] = 10'd256;
        gain_lut[9'd31] = 10'd256;
        gain_lut[9'd32] = 10'd251;
        gain_lut[9'd33] = 10'd244;
        gain_lut[9'd34] = 10'd237;
        gain_lut[9'd35] = 10'd230;
        gain_lut[9'd36] = 10'd224;
        gain_lut[9'd37] = 10'd218;
        gain_lut[9'd38] = 10'd212;
        gain_lut[9'd39] = 10'd207;
        gain_lut[9'd40] = 10'd201;
        gain_lut[9'd41] = 10'd197;
        gain_lut[9'd42] = 10'd192;
        gain_lut[9'd43] = 10'd188;
        gain_lut[9'd44] = 10'd183;
        gain_lut[9'd45] = 10'd179;
        gain_lut[9'd46] = 10'd175;
        gain_lut[9'd47] = 10'd172;
        gain_lut[9'd48] = 10'd168;
        gain_lut[9'd49] = 10'd165;
        gain_lut[9'd50] = 10'd162;
        gain_lut[9'd51] = 10'd158;
        gain_lut[9'd52] = 10'd155;
        gain_lut[9'd53] = 10'd153;
        gain_lut[9'd54] = 10'd150;
        gain_lut[9'd55] = 10'd147;
        gain_lut[9'd56] = 10'd144;
        gain_lut[9'd57] = 10'd142;
        gain_lut[9'd58] = 10'd139;
        gain_lut[9'd59] = 10'd137;
        gain_lut[9'd60] = 10'd135;
        gain_lut[9'd61] = 10'd133;
        gain_lut[9'd62] = 10'd131;
        gain_lut[9'd63] = 10'd129;
        gain_lut[9'd64] = 10'd127;
        gain_lut[9'd65] = 10'd125;
        gain_lut[9'd66] = 10'd123;
        gain_lut[9'd67] = 10'd121;
        gain_lut[9'd68] = 10'd119;
        gain_lut[9'd69] = 10'd117;
        gain_lut[9'd70] = 10'd116;
        gain_lut[9'd71] = 10'd114;
        gain_lut[9'd72] = 10'd113;
        gain_lut[9'd73] = 10'd111;
        gain_lut[9'd74] = 10'd110;
        gain_lut[9'd75] = 10'd108;
        gain_lut[9'd76] = 10'd107;
        gain_lut[9'd77] = 10'd105;
        gain_lut[9'd78] = 10'd104;
        gain_lut[9'd79] = 10'd103;
        gain_lut[9'd80] = 10'd101;
        gain_lut[9'd81] = 10'd100;
        gain_lut[9'd82] = 10'd99;
        gain_lut[9'd83] = 10'd98;
        gain_lut[9'd84] = 10'd97;
        gain_lut[9'd85] = 10'd95;
        gain_lut[9'd86] = 10'd94;
        gain_lut[9'd87] = 10'd93;
        gain_lut[9'd88] = 10'd92;
        gain_lut[9'd89] = 10'd91;
        gain_lut[9'd90] = 10'd90;
        gain_lut[9'd91] = 10'd89;
        gain_lut[9'd92] = 10'd88;
        gain_lut[9'd93] = 10'd87;
        gain_lut[9'd94] = 10'd86;
        gain_lut[9'd95] = 10'd85;
        gain_lut[9'd96] = 10'd85;
        gain_lut[9'd97] = 10'd84;
        gain_lut[9'd98] = 10'd83;
        gain_lut[9'd99] = 10'd82;
        gain_lut[9'd100] = 10'd81;
        gain_lut[9'd101] = 10'd80;
        gain_lut[9'd102] = 10'd80;
        gain_lut[9'd103] = 10'd79;
        gain_lut[9'd104] = 10'd78;
        gain_lut[9'd105] = 10'd77;
        gain_lut[9'd106] = 10'd77;
        gain_lut[9'd107] = 10'd76;
        gain_lut[9'd108] = 10'd75;
        gain_lut[9'd109] = 10'd75;
        gain_lut[9'd110] = 10'd74;
        gain_lut[9'd111] = 10'd73;
        gain_lut[9'd112] = 10'd73;
        gain_lut[9'd113] = 10'd72;
        gain_lut[9'd114] = 10'd71;
        gain_lut[9'd115] = 10'd71;
        gain_lut[9'd116] = 10'd70;
        gain_lut[9'd117] = 10'd69;
        gain_lut[9'd118] = 10'd69;
        gain_lut[9'd119] = 10'd68;
        gain_lut[9'd120] = 10'd68;
        gain_lut[9'd121] = 10'd67;
        gain_lut[9'd122] = 10'd67;
        gain_lut[9'd123] = 10'd66;
        gain_lut[9'd124] = 10'd66;
        gain_lut[9'd125] = 10'd65;
        gain_lut[9'd126] = 10'd65;
        gain_lut[9'd127] = 10'd64;
        gain_lut[9'd128] = 10'd64;
        gain_lut[9'd129] = 10'd63;
        gain_lut[9'd130] = 10'd63;
        gain_lut[9'd131] = 10'd62;
        gain_lut[9'd132] = 10'd62;
        gain_lut[9'd133] = 10'd61;
        gain_lut[9'd134] = 10'd61;
        gain_lut[9'd135] = 10'd60;
        gain_lut[9'd136] = 10'd60;
        gain_lut[9'd137] = 10'd59;
        gain_lut[9'd138] = 10'd59;
        gain_lut[9'd139] = 10'd58;
        gain_lut[9'd140] = 10'd58;
        gain_lut[9'd141] = 10'd58;
        gain_lut[9'd142] = 10'd57;
        gain_lut[9'd143] = 10'd57;
        gain_lut[9'd144] = 10'd56;
        gain_lut[9'd145] = 10'd56;
        gain_lut[9'd146] = 10'd56;
        gain_lut[9'd147] = 10'd55;
        gain_lut[9'd148] = 10'd55;
        gain_lut[9'd149] = 10'd55;
        gain_lut[9'd150] = 10'd54;
        gain_lut[9'd151] = 10'd54;
        gain_lut[9'd152] = 10'd54;
        gain_lut[9'd153] = 10'd53;
        gain_lut[9'd154] = 10'd53;
        gain_lut[9'd155] = 10'd52;
        gain_lut[9'd156] = 10'd52;
        gain_lut[9'd157] = 10'd52;
        gain_lut[9'd158] = 10'd51;
        gain_lut[9'd159] = 10'd51;
        gain_lut[9'd160] = 10'd51;
        gain_lut[9'd161] = 10'd51;
        gain_lut[9'd162] = 10'd50;
        gain_lut[9'd163] = 10'd50;
        gain_lut[9'd164] = 10'd50;
        gain_lut[9'd165] = 10'd49;
        gain_lut[9'd166] = 10'd49;
        gain_lut[9'd167] = 10'd49;
        gain_lut[9'd168] = 10'd48;
        gain_lut[9'd169] = 10'd48;
        gain_lut[9'd170] = 10'd48;
        gain_lut[9'd171] = 10'd48;
        gain_lut[9'd172] = 10'd47;
        gain_lut[9'd173] = 10'd47;
        gain_lut[9'd174] = 10'd47;
        gain_lut[9'd175] = 10'd46;
        gain_lut[9'd176] = 10'd46;
        gain_lut[9'd177] = 10'd46;
        gain_lut[9'd178] = 10'd46;
        gain_lut[9'd179] = 10'd45;
        gain_lut[9'd180] = 10'd45;
        gain_lut[9'd181] = 10'd45;
        gain_lut[9'd182] = 10'd45;
        gain_lut[9'd183] = 10'd44;
        gain_lut[9'd184] = 10'd44;
        gain_lut[9'd185] = 10'd44;
        gain_lut[9'd186] = 10'd44;
        gain_lut[9'd187] = 10'd44;
        gain_lut[9'd188] = 10'd43;
        gain_lut[9'd189] = 10'd43;
        gain_lut[9'd190] = 10'd43;
        gain_lut[9'd191] = 10'd43;
        gain_lut[9'd192] = 10'd42;
        gain_lut[9'd193] = 10'd42;
        gain_lut[9'd194] = 10'd42;
        gain_lut[9'd195] = 10'd42;
        gain_lut[9'd196] = 10'd42;
        gain_lut[9'd197] = 10'd41;
        gain_lut[9'd198] = 10'd41;
        gain_lut[9'd199] = 10'd41;
        gain_lut[9'd200] = 10'd41;
        gain_lut[9'd201] = 10'd40;
        gain_lut[9'd202] = 10'd40;
        gain_lut[9'd203] = 10'd40;
        gain_lut[9'd204] = 10'd40;
        gain_lut[9'd205] = 10'd40;
        gain_lut[9'd206] = 10'd40;
        gain_lut[9'd207] = 10'd39;
        gain_lut[9'd208] = 10'd39;
        gain_lut[9'd209] = 10'd39;
        gain_lut[9'd210] = 10'd39;
        gain_lut[9'd211] = 10'd39;
        gain_lut[9'd212] = 10'd38;
        gain_lut[9'd213] = 10'd38;
        gain_lut[9'd214] = 10'd38;
        gain_lut[9'd215] = 10'd38;
        gain_lut[9'd216] = 10'd38;
        gain_lut[9'd217] = 10'd38;
        gain_lut[9'd218] = 10'd37;
        gain_lut[9'd219] = 10'd37;
        gain_lut[9'd220] = 10'd37;
        gain_lut[9'd221] = 10'd37;
        gain_lut[9'd222] = 10'd37;
        gain_lut[9'd223] = 10'd37;
        gain_lut[9'd224] = 10'd36;
        gain_lut[9'd225] = 10'd36;
        gain_lut[9'd226] = 10'd36;
        gain_lut[9'd227] = 10'd36;
        gain_lut[9'd228] = 10'd36;
        gain_lut[9'd229] = 10'd36;
        gain_lut[9'd230] = 10'd35;
        gain_lut[9'd231] = 10'd35;
        gain_lut[9'd232] = 10'd35;
        gain_lut[9'd233] = 10'd35;
        gain_lut[9'd234] = 10'd35;
        gain_lut[9'd235] = 10'd35;
        gain_lut[9'd236] = 10'd35;
        gain_lut[9'd237] = 10'd34;
        gain_lut[9'd238] = 10'd34;
        gain_lut[9'd239] = 10'd34;
        gain_lut[9'd240] = 10'd34;
        gain_lut[9'd241] = 10'd34;
        gain_lut[9'd242] = 10'd34;
        gain_lut[9'd243] = 10'd34;
        gain_lut[9'd244] = 10'd33;
        gain_lut[9'd245] = 10'd33;
        gain_lut[9'd246] = 10'd33;
        gain_lut[9'd247] = 10'd33;
        gain_lut[9'd248] = 10'd33;
        gain_lut[9'd249] = 10'd33;
        gain_lut[9'd250] = 10'd33;
        gain_lut[9'd251] = 10'd32;
        gain_lut[9'd252] = 10'd32;
        gain_lut[9'd253] = 10'd32;
        gain_lut[9'd254] = 10'd32;
        gain_lut[9'd255] = 10'd32;
        gain_lut[9'd256] = 10'd32;
        gain_lut[9'd257] = 10'd32;
        gain_lut[9'd258] = 10'd32;
        gain_lut[9'd259] = 10'd31;
        gain_lut[9'd260] = 10'd31;
        gain_lut[9'd261] = 10'd31;
        gain_lut[9'd262] = 10'd31;
        gain_lut[9'd263] = 10'd31;
        gain_lut[9'd264] = 10'd31;
        gain_lut[9'd265] = 10'd31;
        gain_lut[9'd266] = 10'd31;
        gain_lut[9'd267] = 10'd31;
        gain_lut[9'd268] = 10'd30;
        gain_lut[9'd269] = 10'd30;
        gain_lut[9'd270] = 10'd30;
        gain_lut[9'd271] = 10'd30;
        gain_lut[9'd272] = 10'd30;
        gain_lut[9'd273] = 10'd30;
        gain_lut[9'd274] = 10'd30;
        gain_lut[9'd275] = 10'd30;
        gain_lut[9'd276] = 10'd30;
        gain_lut[9'd277] = 10'd29;
        gain_lut[9'd278] = 10'd29;
        gain_lut[9'd279] = 10'd29;
        gain_lut[9'd280] = 10'd29;
        gain_lut[9'd281] = 10'd29;
        gain_lut[9'd282] = 10'd29;
        gain_lut[9'd283] = 10'd29;
        gain_lut[9'd284] = 10'd29;
        gain_lut[9'd285] = 10'd29;
        gain_lut[9'd286] = 10'd28;
        gain_lut[9'd287] = 10'd28;
        gain_lut[9'd288] = 10'd28;
        gain_lut[9'd289] = 10'd28;
        gain_lut[9'd290] = 10'd28;
        gain_lut[9'd291] = 10'd28;
        gain_lut[9'd292] = 10'd28;
        gain_lut[9'd293] = 10'd28;
        gain_lut[9'd294] = 10'd28;
        gain_lut[9'd295] = 10'd28;
        gain_lut[9'd296] = 10'd28;
        gain_lut[9'd297] = 10'd27;
        gain_lut[9'd298] = 10'd27;
        gain_lut[9'd299] = 10'd27;
        gain_lut[9'd300] = 10'd27;
        gain_lut[9'd301] = 10'd27;
        gain_lut[9'd302] = 10'd27;
        gain_lut[9'd303] = 10'd27;
        gain_lut[9'd304] = 10'd27;
        gain_lut[9'd305] = 10'd27;
        gain_lut[9'd306] = 10'd27;
        gain_lut[9'd307] = 10'd27;
        gain_lut[9'd308] = 10'd26;
        gain_lut[9'd309] = 10'd26;
        gain_lut[9'd310] = 10'd26;
        gain_lut[9'd311] = 10'd26;
        gain_lut[9'd312] = 10'd26;
        gain_lut[9'd313] = 10'd26;
        gain_lut[9'd314] = 10'd26;
        gain_lut[9'd315] = 10'd26;
        gain_lut[9'd316] = 10'd26;
        gain_lut[9'd317] = 10'd26;
        gain_lut[9'd318] = 10'd26;
        gain_lut[9'd319] = 10'd26;
        gain_lut[9'd320] = 10'd25;
        gain_lut[9'd321] = 10'd25;
        gain_lut[9'd322] = 10'd25;
        gain_lut[9'd323] = 10'd25;
        gain_lut[9'd324] = 10'd25;
        gain_lut[9'd325] = 10'd25;
        gain_lut[9'd326] = 10'd25;
        gain_lut[9'd327] = 10'd25;
        gain_lut[9'd328] = 10'd25;
        gain_lut[9'd329] = 10'd25;
        gain_lut[9'd330] = 10'd25;
        gain_lut[9'd331] = 10'd25;
        gain_lut[9'd332] = 10'd25;
        gain_lut[9'd333] = 10'd24;
        gain_lut[9'd334] = 10'd24;
        gain_lut[9'd335] = 10'd24;
        gain_lut[9'd336] = 10'd24;
        gain_lut[9'd337] = 10'd24;
        gain_lut[9'd338] = 10'd24;
        gain_lut[9'd339] = 10'd24;
        gain_lut[9'd340] = 10'd24;
        gain_lut[9'd341] = 10'd24;
        gain_lut[9'd342] = 10'd24;
        gain_lut[9'd343] = 10'd24;
        gain_lut[9'd344] = 10'd24;
        gain_lut[9'd345] = 10'd24;
        gain_lut[9'd346] = 10'd24;
        gain_lut[9'd347] = 10'd23;
        gain_lut[9'd348] = 10'd23;
        gain_lut[9'd349] = 10'd23;
        gain_lut[9'd350] = 10'd23;
        gain_lut[9'd351] = 10'd23;
        gain_lut[9'd352] = 10'd23;
        gain_lut[9'd353] = 10'd23;
        gain_lut[9'd354] = 10'd23;
        gain_lut[9'd355] = 10'd23;
        gain_lut[9'd356] = 10'd23;
        gain_lut[9'd357] = 10'd23;
        gain_lut[9'd358] = 10'd23;
        gain_lut[9'd359] = 10'd23;
        gain_lut[9'd360] = 10'd23;
        gain_lut[9'd361] = 10'd23;
        gain_lut[9'd362] = 10'd23;
        gain_lut[9'd363] = 10'd22;
        gain_lut[9'd364] = 10'd22;
        gain_lut[9'd365] = 10'd22;
        gain_lut[9'd366] = 10'd22;
        gain_lut[9'd367] = 10'd22;
        gain_lut[9'd368] = 10'd22;
        gain_lut[9'd369] = 10'd22;
        gain_lut[9'd370] = 10'd22;
        gain_lut[9'd371] = 10'd22;
        gain_lut[9'd372] = 10'd22;
        gain_lut[9'd373] = 10'd22;
        gain_lut[9'd374] = 10'd22;
        gain_lut[9'd375] = 10'd22;
        gain_lut[9'd376] = 10'd22;
        gain_lut[9'd377] = 10'd22;
        gain_lut[9'd378] = 10'd22;
        gain_lut[9'd379] = 10'd22;
        gain_lut[9'd380] = 10'd21;
        gain_lut[9'd381] = 10'd21;
        gain_lut[9'd382] = 10'd21;
        gain_lut[9'd383] = 10'd21;
        gain_lut[9'd384] = 10'd21;
        gain_lut[9'd385] = 10'd21;
        gain_lut[9'd386] = 10'd21;
        gain_lut[9'd387] = 10'd21;
        gain_lut[9'd388] = 10'd21;
        gain_lut[9'd389] = 10'd21;
        gain_lut[9'd390] = 10'd21;
        gain_lut[9'd391] = 10'd21;
        gain_lut[9'd392] = 10'd21;
        gain_lut[9'd393] = 10'd21;
        gain_lut[9'd394] = 10'd21;
        gain_lut[9'd395] = 10'd21;
        gain_lut[9'd396] = 10'd21;
        gain_lut[9'd397] = 10'd21;
        gain_lut[9'd398] = 10'd20;
        gain_lut[9'd399] = 10'd20;
        gain_lut[9'd400] = 10'd20;
        gain_lut[9'd401] = 10'd20;
        gain_lut[9'd402] = 10'd20;
        gain_lut[9'd403] = 10'd20;
        gain_lut[9'd404] = 10'd20;
        gain_lut[9'd405] = 10'd20;
        gain_lut[9'd406] = 10'd20;
        gain_lut[9'd407] = 10'd20;
        gain_lut[9'd408] = 10'd20;
        gain_lut[9'd409] = 10'd20;
        gain_lut[9'd410] = 10'd20;
        gain_lut[9'd411] = 10'd20;
        gain_lut[9'd412] = 10'd20;
        gain_lut[9'd413] = 10'd20;
        gain_lut[9'd414] = 10'd20;
        gain_lut[9'd415] = 10'd20;
        gain_lut[9'd416] = 10'd20;
        gain_lut[9'd417] = 10'd20;
        gain_lut[9'd418] = 10'd19;
        gain_lut[9'd419] = 10'd19;
        gain_lut[9'd420] = 10'd19;
        gain_lut[9'd421] = 10'd19;
        gain_lut[9'd422] = 10'd19;
        gain_lut[9'd423] = 10'd19;
        gain_lut[9'd424] = 10'd19;
        gain_lut[9'd425] = 10'd19;
        gain_lut[9'd426] = 10'd19;
        gain_lut[9'd427] = 10'd19;
        gain_lut[9'd428] = 10'd19;
        gain_lut[9'd429] = 10'd19;
        gain_lut[9'd430] = 10'd19;
        gain_lut[9'd431] = 10'd19;
        gain_lut[9'd432] = 10'd19;
        gain_lut[9'd433] = 10'd19;
        gain_lut[9'd434] = 10'd19;
        gain_lut[9'd435] = 10'd19;
        gain_lut[9'd436] = 10'd19;
        gain_lut[9'd437] = 10'd19;
        gain_lut[9'd438] = 10'd19;
        gain_lut[9'd439] = 10'd19;
        gain_lut[9'd440] = 10'd19;
        gain_lut[9'd441] = 10'd18;
        gain_lut[9'd442] = 10'd18;
        gain_lut[9'd443] = 10'd18;
        gain_lut[9'd444] = 10'd18;
        gain_lut[9'd445] = 10'd18;
        gain_lut[9'd446] = 10'd18;
        gain_lut[9'd447] = 10'd18;
        gain_lut[9'd448] = 10'd18;
        gain_lut[9'd449] = 10'd18;
        gain_lut[9'd450] = 10'd18;
        gain_lut[9'd451] = 10'd18;
        gain_lut[9'd452] = 10'd18;
        gain_lut[9'd453] = 10'd18;
        gain_lut[9'd454] = 10'd18;
        gain_lut[9'd455] = 10'd18;
        gain_lut[9'd456] = 10'd18;
        gain_lut[9'd457] = 10'd18;
        gain_lut[9'd458] = 10'd18;
        gain_lut[9'd459] = 10'd18;
        gain_lut[9'd460] = 10'd18;
        gain_lut[9'd461] = 10'd18;
        gain_lut[9'd462] = 10'd18;
        gain_lut[9'd463] = 10'd18;
        gain_lut[9'd464] = 10'd18;
        gain_lut[9'd465] = 10'd18;
        gain_lut[9'd466] = 10'd17;
        gain_lut[9'd467] = 10'd17;
        gain_lut[9'd468] = 10'd17;
        gain_lut[9'd469] = 10'd17;
        gain_lut[9'd470] = 10'd17;
        gain_lut[9'd471] = 10'd17;
        gain_lut[9'd472] = 10'd17;
        gain_lut[9'd473] = 10'd17;
        gain_lut[9'd474] = 10'd17;
        gain_lut[9'd475] = 10'd17;
        gain_lut[9'd476] = 10'd17;
        gain_lut[9'd477] = 10'd17;
        gain_lut[9'd478] = 10'd17;
        gain_lut[9'd479] = 10'd17;
        gain_lut[9'd480] = 10'd17;
        gain_lut[9'd481] = 10'd17;
        gain_lut[9'd482] = 10'd17;
        gain_lut[9'd483] = 10'd17;
        gain_lut[9'd484] = 10'd17;
        gain_lut[9'd485] = 10'd17;
        gain_lut[9'd486] = 10'd17;
        gain_lut[9'd487] = 10'd17;
        gain_lut[9'd488] = 10'd17;
        gain_lut[9'd489] = 10'd17;
        gain_lut[9'd490] = 10'd17;
        gain_lut[9'd491] = 10'd17;
        gain_lut[9'd492] = 10'd17;
        gain_lut[9'd493] = 10'd17;
        gain_lut[9'd494] = 10'd17;
        gain_lut[9'd495] = 10'd16;
        gain_lut[9'd496] = 10'd16;
        gain_lut[9'd497] = 10'd16;
        gain_lut[9'd498] = 10'd16;
        gain_lut[9'd499] = 10'd16;
        gain_lut[9'd500] = 10'd16;
        gain_lut[9'd501] = 10'd16;
        gain_lut[9'd502] = 10'd16;
        gain_lut[9'd503] = 10'd16;
        gain_lut[9'd504] = 10'd16;
        gain_lut[9'd505] = 10'd16;
        gain_lut[9'd506] = 10'd16;
        gain_lut[9'd507] = 10'd16;
        gain_lut[9'd508] = 10'd16;
        gain_lut[9'd509] = 10'd16;
        gain_lut[9'd510] = 10'd16;
        gain_lut[9'd511] = 10'd16;
    end

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

    //------------------------------------------------------------------
    // 音色：按钮切换【谐波数量】—— 只有两种模式
    //     t=0：只有基波            （无谐波，最干净）
    //     t=1：基波 + 2 次          （两个谐波，稍亮）
    //   ★ 按一下【按钮4】在 0/1 之间来回切。
    //   S1~S4 已经逐键乘过包络，所以这里是"Σ env·h_n"，因式分解依然成立。
    //   ★ 3 次 / 4 次谐波不再使用（S3/S4 仍照常计算，只是不参与混合，
    //     综合器会把它们优化掉；如果想让它们彻底消失，可以把采样状态机
    //     里 S3/S4 的锁存去掉，但没必要动）。
    //------------------------------------------------------------------
    wire [1:0] timbre = {1'b0, timbre_audio};

    // ★ 力度 vel_audio (0~255) 决定谐波比例：
    //     vel=0   -> 系数 0    -> 只有基波，音色柔和
    //     vel=255 -> 系数 255  -> S2 × 255 / 512 ≈ S2/2，音色明亮
    //   S2 是 22 位有符号，乘 8 位系数得 30 位，取 >>>9 回到 22 位量级
    wire signed [29:0] harm_mul   = S2 * $signed({1'b0, vel_audio});
    // ★ 等价改写：右移不改变表达式位宽（仍是 30 位），原来赋给 23 位变量时
    //   取的是低 23 位；这里先落到 30 位 wire 再显式取 [22:0]，
    //   取值完全一致，只是消掉 EX3791 截断告警。
    wire signed [29:0] harm_shift = harm_mul >>> 9;
    wire signed [22:0] harm_raw   = timbre[0] ? harm_shift[22:0] : 23'sd0;
    // ★ 笛子音色：2 次谐波压一半，音色更纯更"空心"
    //   （摇杆 Y 的"力度 -> 谐波"效果照样有效，只是幅度减半）
    //   flute_on_audio=0 时 harm_sum 取的就是 harm_raw，逐位不变。
    //
    // ★★ V48.2：小号音色 —— 【原样】再垫一份固定量的 2 次谐波（S2 >>> 2）。
    //   为什么这么做：这个设计的"亮度"只有一个旋钮（2 次谐波的量，由力度给）。
    //   普通音色是"力度小就柔和"；而铜管的特点是**不管吹多轻都一直是亮的**，
    //   所以小号 = 力度那一份照旧 + 一份固定垫底 —— 用摇杆/MIDI 力度还能
    //   继续调亮度，但永远有个"铜管底色"，不会一轻就变回纯正弦。
    //   ★ 位宽：harm_raw 最大约 S2/2，再加上 S2/4，合计 < S2，23 位装得下。
    //   ★ trumpet_on_audio = 0 时下面那一行取到的就是原来的表达式，逐位不变。
    wire signed [22:0] harm_brass = harm_raw + (S2 >>> 2);
    wire signed [22:0] harm_sum   = guitar_on_audio ? 23'sd0 :
                                    trumpet_on_audio ? harm_brass :
                                    (flute_on_audio ? (harm_raw >>> 1) : harm_raw);
    wire signed [23:0] harm_scaled = harm_sum;

    //------------------------------------------------------------------
    // ★★ V34：多声部电平归一（治"同时响多个音就不干净"）★★
    //
    //   问题：原来这里是 `mix = mix_pre >>> 1` —— 一个【固定】的右移。
    //         于是总电平随"在响的声部数"线性涨：按注释那套标定，大约
    //         4 个声部满包络就到满幅，再多就顶穿 16 位输出（削顶/回绕）。
    //         听感就是"两个音以上发糊、不干净，低音一强音高还发飘"。
    //   ★ 这和"是不是和弦"无关：只要同时响的音多于一个就会犯。
    //
    //   修法：按在响声部数给 headroom，系数取 1/sqrt(N) 那一档
    //         （听感最接近"音多了也不炸"）：
    //             N=1 -> 256(×1.00)   N=2 -> 192(×0.75)  N=3 -> 154(×0.60)
    //             N=4 -> 128(×0.50)   N=5 -> 115         N=6 -> 102
    //             N>=7 -> 90
    //         ★★ N=1 时 256/512 = 1/2 —— 和原来的 `>>> 1` 【逐位相同】★★
    //            也就是说【单音、以及所有只有一个声部在响的曲子，输出一个
    //            采样点都不会变】，原来正常的听感原样保留。
    //   ★ 代价：一个 24×9 乘法器（占 1 个 DSP）+ 一个 36 位 popcount（LUT）。
    //
    //   ★ 为什么不用"限幅"了事：限幅只把超出部分砍掉，音多的时候照样糊；
    //     先按声部数把电平让开，才是从根上让多声部都在量程内。
    //------------------------------------------------------------------
    //   在响的声部数（36 位 popcount；按位相加，综合器会搭加法树，不占 DSP）
    reg [5:0] n_voice;
    integer pi;
    always @(*) begin
        n_voice = 6'd0;
        for (pi = 0; pi < NUM_KEYS; pi = pi + 1)
            n_voice = n_voice + {5'd0, sounding_mask_aud[pi]};
    end

    //   声部数 -> 缩放系数（分母 512：N=1 时 256/512 = 1/2 = 原来的 >>>1）
    function [8:0] voice_scale;
        input [5:0] n;
        case (n)
            6'd0, 6'd1: voice_scale = 9'd256;   // ×1.00（= 原行为）
            6'd2:       voice_scale = 9'd192;   // ×0.75
            6'd3:       voice_scale = 9'd154;   // ×0.60
            6'd4:       voice_scale = 9'd128;   // ×0.50
            6'd5:       voice_scale = 9'd115;
            6'd6:       voice_scale = 9'd102;
            default:    voice_scale = 9'd90;
        endcase
    endfunction

    //------------------------------------------------------------------
    // ★★ V38：系数斜坡 —— 治"每个音起音时那一声噗" ★★
    //
    //   病灶（V34 引入）：mix_scale 直接取 voice_scale(n_voice) 的查表值，
    //   而 n_voice 是【整数声部数】。每起音一个音 n_voice 就 +1，系数整档跳
    //   （N=1→2 是 256→192，所有在响的音同时掉 25%），在一个采样点内完成 ——
    //   听感就是每个音头上那一声"噗"。放音时反向跳回来，又是一声。
    //
    //   修法：让系数【每采样点走一小步】爬到目标值，而不是一步到位。
    //     SCALE_STEP = 1 → 最坏一档（256→90）走 166 个采样点 ≈ 3.4ms，
    //     人耳对 3ms 的电平变化无感，台阶却没了。
    //
    //   ★★ mix_scale_r 是【寄存输出】，但 mix 仍然用它做组合乘法 ——
    //      没有额外打拍，整条链路的时序与 V34~V37 完全一致。
    //   ★★ N=1（单音、以及所有只有一个声部在响的曲子）时 scale_tgt 恒为 256，
    //      而复位值也是 256 → mix_scale_r 一直不动，取到的值与改之前
    //      【逐位相同】。斜坡只在"声部数真的变了"的时候才动。
    //------------------------------------------------------------------
    //=========================================================================
    // ★★★ V47.1：斜坡【只跟声部数】，力度【瞬时】乘在斜坡之后 ★★★
    //
    //   ★ 为什么改（V47 实测出的问题："加了力度以后每个音都有噗一下"）：
    //     V47 把力度接到了 mix_scale_r 的【斜坡目标】上。而那条斜坡
    //     （SCALE_STEP=1/采样点）是为"声部数变化"设计的，落差 96 步要 ~2ms。
    //     于是每个音起音那一瞬：n_voice+1 且 力度从 127 掉到本音力度
    //     → scale_tgt 猛掉（比如 256→160），但斜坡要 2ms 才走完
    //     → 【这 2ms 里起音是按旧的大音量出来的】，然后再缩下去
    //     → 听感就是每个音一声"噗"。
    //     ★ 是"把力度塞进斜坡目标"这件事本身错了，不是力度错。
    //
    //   ★ 修法：把力度从斜坡目标里【拿出去】——
    //       scale_tgt 恢复成只跟声部数（= V46 逐位相同，V38 的治噗声原样保留）
    //       力度改成乘在斜坡【之后】，瞬时生效。
    //
    //   ★ 代价（如实说明）：力度瞬时变化时，正在延音的那些音也会跟着跳一下。
    //     但这和 V38 那个"每个音都噗"不是一回事：V38 是【每次加音都掉一档】，
    //     这里是【只在力度真的变了的时候跳】，而且跳的瞬间正好被新音的起音盖住。
    //=========================================================================
    wire [8:0]  scale_tgt = voice_scale(n_voice);

    localparam [8:0] SCALE_STEP = 9'd1;

    reg [8:0] mix_scale_r;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) mix_scale_r <= 9'd256;    // = N=1，等价于原来的 >>>1
        else if (ctrl_en) begin
            if (mix_scale_r < scale_tgt)
                mix_scale_r <= (mix_scale_r + SCALE_STEP >= scale_tgt)
                               ? scale_tgt : (mix_scale_r + SCALE_STEP);
            else if (mix_scale_r > scale_tgt)
                mix_scale_r <= (mix_scale_r - SCALE_STEP <= scale_tgt)
                               ? scale_tgt : (mix_scale_r - SCALE_STEP);
        end
    end

    wire [8:0] mix_scale = mix_scale_r;

    // ★★★ V47.2：力度【不再】乘在这里 ★★★
    //   V47.1 曾把力度瞬时乘在斜坡之后 —— 虽然解决了"斜坡滞后导致的噗"，
    //   但它动的是【混音总增益】这条共用路径：力度一变，所有正在响的音
    //   （包括延音尾巴）都跟着跳。用户实测"噗声还在"，根子就在这里。
    //   现在力度改走【每个音自己的包络峰值】（见 gen_env 里 vel_peak_k），
    //   这一整段恢复成 V46 原样 —— mix_scale 只跟声部数有关。
    wire [8:0]  mix_scale_f = mix_scale;

    // 基波 + 谐波混合
    //   N=1 时 (mix_pre × 256) >>> 9 ≡ mix_pre >>> 1（逐位相同）
    wire signed [23:0] mix_pre = S1 + harm_scaled;
    wire signed [32:0] mix_mul = mix_pre * $signed({1'b0, mix_scale_f});
    wire signed [23:0] mix     = mix_mul >>> 9;

    // 与增益表那一拍读出对齐（gain_k 是寄存输出，晚一拍）
    reg signed [23:0] mix_r;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) mix_r <= 24'sd0;
        else                mix_r <= mix;
    end

    //------------------------------------------------------------------
    // 最终增益：mix_r x gain_k >> 10   （>>10 即 /1024）
    //   C = 65280 与 >>10 是配套的，改一个必须一起改
    //   （满负荷 = h*65280/(256*1024) = 0.249h，与单音满包络 h*255/1024 对齐）
    //------------------------------------------------------------------
    wire signed [33:0] mix_prod = mix_r * $signed({1'b0, gain_k});
    wire signed [23:0] vol_norm = mix_prod >>> 10;

    //------------------------------------------------------------------
    // 摇杆 Y 轴 -> 总音量
    //
    //   ★★ 这一环是 K44/25K 上原本就有的，被我重写增益级时弄丢了 ★★
    //      K44 的接法（供对照）：
    //          localparam [7:0] VOL_MIN = 8'd32;
    //          wire [7:0]  vol_use   = (vol_audio < VOL_MIN) ? VOL_MIN : vol_audio;
    //          wire [15:0] gain_prod = gain_t * vol_use;
    //          wire [7:0]  gain_comb = gain_prod[15:8];
    //          ... gain_cur 斜坡 ...  输出 = mix * gain_cur
    //      K44 里 vol_audio 就是通过 gain_t × vol_use 进到输出的。
    //
    //   本版改成等效写法：归一化增益 gain_k(上限 256) 直接算完 mix，
    //   再乘一级音量。这样就【不必再要 gain_cur 斜坡】——
    //     · gain_k ≤ 256 已由增益表钳住，不会放大；
    //     · vol_use 也只在 0~255 之间，乘出来不会削顶；
    //     · 单一乘级 = 单一斜坡，比 K44 的两级串联更干净。
    //   定标 vol_use/256：255 → 0.996 倍，32 → 0.125 倍（保持 VOL_MIN 下限）。
    //------------------------------------------------------------------
    // ★ 总音量固定，不再由摇杆 Y 控制
    //   摇杆 Y 已经改去驱动"力度"（谐波比例）
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
        else if (ctrl_en) trem_acc <= trem_acc + TREM_INC;
    end

    // 三角波 0~63~0
    wire [5:0] trem_tri = trem_acc[31] ? ~trem_acc[30:25] : trem_acc[30:25];

    // 目标衰减量 = tri/64 × DEPTH
    //  ★★ 这里原来有个位宽陷阱（颤音听不出来的根因）★★
    //     Verilog 里 a*b 的结果宽度 = max(操作数宽度) 再被赋值左边卡住：
    //     trem_tri(6位) × 110 最大 6930（要 13 位），但直接写
    //         wire [7:0] trem_tgt = (trem_tri * TREM_DEPTH) >> 6;
    //     乘法是在 8 位上下文里做的 → 6930 被截成 18 → >>6 恒为 0/1
    //     → vol_dyn 只会在 220/219 之间抖 0.5% → 完全听不出来。
    //     必须先把乘积落到够宽的中间 wire 上再移位。
    //=========================================================================
    // ★★★ V19 新增：ADC2 摇杆控制颤音深度 ★★★
    //
    //   颤音的【开关】仍然是按钮5；这里第二个摇杆（ADS1115 的 AIN2）
    //   只控制【抖多深】。摇杆不动（或推向中位）= 原来的深度，推到底最深。
    //
    //   ★ 映射方向（V19 修正）：摇杆【静止 = 40（浅，约 -1.7dB）】，
    //     往两边推越深，推到底 = 200（很深，谷底只剩 9%，约 -21dB）。
    //
    //     ★ 为什么把量程拉到 40~200：
    //       原来的 TREM_DEPTH=110 相对 VOL_FIXED=220 只降一半（-6dB），
    //       本来就温和；摇杆要推到很远才有感觉，用起来像"控制不了"。
    //       现在静止就有听得出的浅颤音，一推立刻变深，推到底是
    //       "几乎断断续续"的强颤音 —— 控制感和可听度都够了。
    //
    //     因为 js_tdep 取的是 |偏移|，所以推左推右效果一样。
    //=========================================================================
    // ★ V19：js_tdep 现在【本身就是深度值】(43~200)，直接用它即可。
    //   原来这里还有一张 16 档二次映射表（把 0~255 压到 40~200），
    //   现在输入范围已经是 43~200，那张表成了恒等映射，去掉，
    //   既省逻辑又少一级延迟。
    wire [7:0] trem_depth_dyn = tdep_audio;

    wire [13:0] trem_prod = trem_tri * trem_depth_dyn;   // 14 位上下文，乘积不丢
    wire [7:0]  trem_tgt  = trem_prod >> 6;

    // 衰减量斜坡：关掉时回 0；打开时追目标值
    reg [7:0] trem_amt;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) trem_amt <= 8'd0;
        else if (ctrl_en) begin
            if (!trem_on_audio) begin
                if (trem_amt != 8'd0)         trem_amt <= trem_amt - 1'b1;
            end else if (trem_amt < trem_tgt) trem_amt <= trem_amt + 1'b1;
            else if     (trem_amt > trem_tgt) trem_amt <= trem_amt - 1'b1;
        end
    end

    // 颤音后的等效音量：VOL_FIXED 扣掉衰减量（钳住，不出现负数）
    //   ★ 想让颤音更深：改 trem_depth_dyn 的表尾
    //   ★ 想换速度：改 TREM_INC，freq ≈ TREM_INC × 48828 / 2^32
    wire [7:0] vol_dyn = (trem_amt >= VOL_FIXED) ? 8'd0 : (VOL_FIXED - trem_amt);

    //=========================================================================
    // ★★★ V19：力度音量 与 颤音 共用同一个乘法器 ★★★
    //
    //   原来摇杆 Y 只送给【谐波比例】（音色亮度），音量恒定 VOL_FIXED。
    //   真乐器上"用力"是【又亮又响】，所以要把力度也接到音量上。
    //
    //   一开始的实现是【两级串联乘法】：
    //        vol_norm × vol_dyn(颤音) -> vol_out
    //        vol_out  × vol_vel(力度) -> vol_vel_out      ← 多了一个 24×9 乘法
    //   那会多占一个宽乘法器、拖慢时序收敛。
    //
    //   ★ 现在合成一级：两个音量系数都是【小正数】，先相乘再一起乘 vol_norm：
    //        vol_comb = (vol_dyn × vol_vel) >> 8        (8位 × 9位 -> 小乘法)
    //        vol_out  = (vol_norm × vol_comb) >> 8      (复用原来那个大乘法)
    //   结果与两级串联【数学上等价】：
    //        vol_norm×vol_dyn/256×vol_vel/256
    //        = vol_norm × ((vol_dyn×vol_vel)>>8) / 256
    //   省掉一个 24×9 乘法器，且乘法链少一级、时序更好收敛。
    //
    //   ★★ 拐点卡在 220，不是 255 ★★
    //       vel_audio 的默认值（JOY_BYPASS / 摇杆还没读到时）是 8'd220。
    //       若按"0~255 线性、255 才 1.0"算，默认只能拿 0.86 ——
    //       等于平白无故把音量砍掉 14%。拐点放 220 后，
    //       【什么都不动时音量和现在一模一样】，只有主动下拉摇杆才变轻。
    //
    //   查表 = 256 - DEPTH + DEPTH*min(vel,220)/220
    //       vel   0 -> 0.602 (-4.4dB)     vel 136 -> 0.848
    //       vel  68 -> 0.723              vel 220 -> 1.000  <- 默认值，原音量
    //       vel 204 -> 0.969              vel 255 -> 1.000
    //
    //   ★ 摇杆2 接 ADS1115 的 A2（已确认）
    //   ★ 想调力度感就改 VOL_VEL_DEPTH：
    //       0 -> 关掉；102 -> 摇到底 60%（当前）；128 -> 50%；160 -> 37%
    //=========================================================================
    localparam integer VOL_VEL_DEPTH = 102;

    reg [8:0] vol_vel;
    always @(*) begin
        case (vel_audio[7:4])          // 取高 4 位，16 档查表（省资源、无除法）
            4'd0 : vol_vel = 9'd154;
            4'd1 : vol_vel = 9'd161;
            4'd2 : vol_vel = 9'd169;
            4'd3 : vol_vel = 9'd177;
            4'd4 : vol_vel = 9'd185;
            4'd5 : vol_vel = 9'd193;
            4'd6 : vol_vel = 9'd201;
            4'd7 : vol_vel = 9'd209;
            4'd8 : vol_vel = 9'd217;
            4'd9 : vol_vel = 9'd224;
            4'd10: vol_vel = 9'd232;
            4'd11: vol_vel = 9'd240;
            4'd12: vol_vel = 9'd248;
            default: vol_vel = 9'd256;   // 13/14/15 -> 满音量
        endcase
    end

    // ★ 共用一个乘法：先把两个音量系数相乘（都是小正数），再一起乘 vol_norm
    //      vol_comb = (vol_dyn × vol_vel) >> 8
    //      vol_out  = (vol_norm × vol_comb) >> 8
    wire [16:0] vol_comb = (vol_dyn * vol_vel) >> 8;        // 8位 × 9位，小乘法

    wire signed [31:0] vol_prod = vol_norm * $signed({1'b0, vol_comb});
    wire signed [23:0] vol_out  = vol_prod >>> 8;

    //------------------------------------------------------------------
    // ★ 干声 = 原来直接写进 sample_vol 的那个值（钳位到 16 位有符号）
    //
    //=========================================================================
    // ★★★ V19 音量修正：输出级补一级增益（带限幅）★★★
    //
    //   修好 ROM 之后实测：sample_vol 峰值只有 ±3504，也就是 16 位动态范围的
    //   约 10% —— 波形是对的，但白白浪费了 90% 的余量，所以听感偏小。
    //
    //   （★ 顺带说明：修 ROM 前听着"更响"，是因为那个坏 ROM 把基波变成
    //     2 倍频、低音被顶高了一个八度。高八度本来就比低音扎耳，
    //     不是音量真的更大。现在音高正确了，需要把电平补回来。）
    //
    //   这里在最终输出前做一次左移增益，并【硬限幅】到 16 位：
    //     饱和而不是回卷，所以就算多键同时按也不会出现爆音/反相。
    //
    //   ★ 想调音量就改 VOL_OUT_SHIFT 这一个数：
    //       0 -> 原始（约 ±3500，太小声）
    //       1 -> 2 倍
    //       2 -> 4 倍   （峰值约 ±14000，43% 满幅）
    //       3 -> 8 倍   ← 当前：峰值约 ±28000，约 86% 满幅，接近满音量
    //       4 -> 16 倍  （会压限幅，可能削顶失真，慎用）
    //
    //   ★ 为什么旧版听着更响：旧版那个 512×32 ROM 在硬件上把基波变成了
    //     2 倍频，低音被顶高一个八度。高八度本来就比低音扎耳。
    //     新 ROM 音高正确后，同样的电平听感会明显变弱，所以这里要补回来。
    //=========================================================================
    localparam integer VOL_OUT_SHIFT = 3;

    wire signed [23:0] vol_out_g = vol_out <<< VOL_OUT_SHIFT;

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
        else if (ctrl_en && !fx_ready)  fx_fill <= fx_fill + 1'b1;
    end

    // ---- LFO：32 位相位累加器取三角波（和颤音同一套做法）----
    reg [31:0] cho_acc;
    always @(posedge clk_audio) begin
        if (rst_audio_sync) cho_acc <= 32'd0;
        else if (ctrl_en) cho_acc <= cho_acc + CHO_INC;
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
        if (ctrl_en) begin
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
        else if (ctrl_en && !dly_ready) dly_fill <= dly_fill + 1'b1;
    end

    wire signed [15:0] dly_use = dly_ready ? dly_tap : 16'sd0;

    // ★★ 位宽提醒：乘法结果宽度 = max(操作数宽度)，会被左边卡住。
    //    这里所有乘法的结果都必须先落到 24 位 wire 上（颤音就栽在这上面）。
    wire signed [23:0] dly_fb_p = dly_use * $signed({1'b0, DLY_FB});   // 24 位上下文
    wire signed [15:0] dly_fb   = dly_fb_p >>> 8;                       // ≈ tap × 0.375
    wire signed [16:0] dly_wsum = s_cho + (dly_on_audio ? dly_fb : 16'sd0);
    wire signed [15:0] dly_wr   = (dly_wsum >  17'sd32767) ?  16'sd32767 :
                                  (dly_wsum < -17'sd32768) ? -16'sd32768 :
                                                              dly_wsum[15:0];

    always @(posedge clk_audio) begin
        if (ctrl_en) begin
            dly_mem[dly_wp] <= dly_ready ? dly_wr : 16'sd0;   // 写端口（含反馈）
            dly_tap         <= dly_mem[dly_rp];               // 读端口（= L 点之前）
            dly_wp          <= dly_wp + 1'b1;
            dly_rp          <= dly_rp + 1'b1;
        end
    end

    wire signed [23:0] dly_wet_p = dly_use * $signed({1'b0, DLY_WET});  // 24 位上下文
    wire signed [15:0] dly_wet   = dly_wet_p >>> 8;                     // ≈ tap × 0.43
    wire signed [16:0] dly_msum  = s_cho + dly_wet;
    wire signed [15:0] dly_mix   = (dly_msum >  17'sd32767) ?  16'sd32767 :
                                   (dly_msum < -17'sd32768) ? -16'sd32768 :
                                                               dly_msum[15:0];

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
        end else if (ctrl_en) begin
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

    reg signed [15:0] sample_vol;


    always @(posedge clk_audio) begin
        if (rst_audio_sync) sample_vol <= 16'sd0;
        else if (sample_cnt == 16'd10)
            sample_vol <= rev_on_audio ? rvb_mix : s_dly;
    end

    //==================================================================
    // ★★★ V36：真正的双声道输出（中/侧分解，只在输出级）★★★
    //
    //   板子上 DAC 两个声道都接了喇叭，而 I2S 帧的两个 32 位槽原先装的是
    //   同一个 mono 样本。这里在【输出级】做一次中/侧（mid/side）分解，
    //   把已经算好的湿声搬到 side 上，两个槽就分别拿到 L / R。
    //
    //   ① 合成链路一个字不动：36 声部 / 包络 / 滑音 / 谐波 / 音色 / KS 吉他 /
    //      颤音 / 延迟 / 混响 / 合唱全部原样；sample_vol 仍然是原来那个
    //      单声道最终输出，这里把它当作 mid。分叉只发生在 sample_vol 之后，
    //      没有任何一条回边（side/L/R 都不参与发声）。
    //
    //   ② side 只从【已经算出来的】信号取，一个 RAM 读口都不加：
    //        合唱：(dry_sample - s_cho)   —— 合唱关掉时 s_cho ≡ dry_sample，
    //                                      这一项【逐位为 0】
    //        延迟：-dly_wet               —— 延迟关掉时被门控成 0
    //        混响：-rvb_wet               —— 混响关掉时被门控成 0
    //      ★ 为什么不新增读口：本工程的教训是"给已有存储器多开一个读口，
    //        综合器就不肯再把它推断成 BSRAM"。所以 side 一律复用已经读出来
    //        的那几个抽头（cho_* / dly_tap / rvb_t*），一个都没多读。
    //        本块里唯一的新存储器是下面 Haas 那条【新的】1024×16 单读单写
    //        小 RAM（1 块 BSRAM），不是给旧存储器加端口。
    //
    //   ③ L = mid + side、R = mid - side  =>  L + R = 2 × mid
    //      单声道和（mono sum）和现在完全一致，立体声不会在单声道回放时掉内容。
    //      ★ 全效果关掉时 side 逐位为 0 -> L = R = sample_vol（现在的值），
    //        也就是【自动退化成原来的 mono 行为】。
    //
    //   ④ 电平（"别比现在的 mono 更响"）：
    //      合唱那一项在"干湿各半"（cho_mix = (dry + wet)/2）前提下正好凑成
    //          side = (dry - s_cho) >>> 2 = (dry - wet)/8
    //          L = 0.625·dry + 0.375·wet
    //          R = 0.375·dry + 0.625·wet
    //      系数【全是正的、且和为 1】（凸组合）-> 峰值不会超过 max(|dry|,|wet|)，
    //      也就是不比现在的 mono 响，也不会凭空多出削顶。
    //      延迟/混响的湿声再各自 >>>3（÷8）后相加，量级远小于主信号；
    //      末尾仍然按本工程一贯的做法做【硬限幅】（饱和，不回卷）兜底。
    //
    //   ⑤ 左右微调（用户点名要的：干声略偏左、湿声略偏右）：
    //      实现上只体现在 side 的【符号】上 —— 干声取 +、湿声取 −，
    //      于是 L 里干声多、湿声少，R 里反过来。零额外电路。
    //      （★ 干声本身没法再单独做增益倾斜：只要 side 里带上"一直非零"的
    //        干声分量，全效果关掉时 side 就不为 0，L/R 会立刻不再相等，
    //        直接违反"全关必须逐位退化成 mono"这条硬约束。所以倾斜只能
    //        这样"相对"地做：干 vs 湿之间分，而不是给干声单独加权。）
    //
    //   ⑥ Haas（可选，默认开）：R 用的 side 再走 ~15ms 延迟，靠优先效应
    //      （precedence effect）把像拉宽。
    //      ★ 只延迟 side，【不延迟整条 R】：整条 R 被延迟的话，side=0 时
    //        L 和 R 就不再逐位相同了（R 会比 L 晚），直接违反硬约束。
    //      ★ 代价（写在明处）：Haas 开着时 L + R = 2·mid + (side - side_d)，
    //        即多出一份"干湿两份 side 之差"的残差。这是【任何】通道间延时
    //        都躲不掉的（左右不同时刻，和就一定不等于 2×mid）；
    //        但 mid 本身在 L/R 里都没有被延时，所以单声道兼容性不受影响。
    //        HAAS_ON=0 或 side=0 时，L + R = 2×mid 是【严格恒等】。
    //==================================================================
    localparam STEREO_ON = 1'b1;          // 0 -> 两个声道槽【逐位】回到现在的 mono
    localparam HAAS_ON   = 1'b1;          // 右声道 Haas 延迟开关（0 时整块 RAM 被裁掉）
    localparam integer ST_CHO_SHIFT = 2;  // 合唱差 >>>2（见 ④ 的凸组合推导）
    localparam integer ST_WET_SHIFT = 3;  // 延迟/混响湿声 >>>3

    // ---- 1) side 的三个分量：全部来自已经算好的信号 ----
    wire signed [16:0] st_cho_d = dry_sample - s_cho;   // 合唱差；合唱关 = 逐位 0
    wire signed [16:0] st_cho_s = st_cho_d >>> ST_CHO_SHIFT;
    wire signed [16:0] st_dly_w = dly_wet;              // 16 -> 17 位带符号扩展
    wire signed [16:0] st_dly_s = dly_on_audio ? (st_dly_w >>> ST_WET_SHIFT) : 17'sd0;
    wire signed [18:0] st_rvb_w = rvb_wet;              // 19 位，原样
    wire signed [18:0] st_rvb_s = rev_on_audio ? (st_rvb_w >>> ST_WET_SHIFT) : 19'sd0;

    // 干声取 +、湿声取 −（= 在 L 里被加/被减），只翻符号
    // ★ 位宽：|st_cho_s|<=16383、|st_dly_s|<=1750、|st_rvb_s|<=1320，
    //   和 <= 19453 < 32768 -> 下面写进 16 位 RAM 取低 16 位不会截到有效位。
    //   ★ 三项都是 19 位（两项前面补符号位、第三项本来就 19 位），
    //     位宽相同就不涉及符号扩展，加减出来的位模式是确定的。
    wire signed [18:0] st_side = {{2{st_cho_s[16]}}, st_cho_s}
                               - {{2{st_dly_s[16]}}, st_dly_s}
                               - st_rvb_s;

    // ---- 2) Haas：右声道那份 side 再延迟 ~15ms（1024 点 = 1 块 BSRAM）----
    //   ★ 单写口 + 单读口的标准推断写法（和下面 dly_mem 一个套路）：
    //     写指针 haas_wp 每采样 +1；读地址取 (wp+1-TAP)，读数据先打一拍。
    //     于是本拍读到的正好是 TAP 个采样点之前写进去的那个值。
    //   ★ 环长必须 > TAP，否则会读到还没被覆盖的位置（1024 > 732 ✔）。
    localparam integer HAAS_BITS = 10;
    localparam integer HAAS_LEN  = 1 << HAAS_BITS;
    localparam [9:0]   HAAS_TAP  = 10'd732;      // 732 / 48828Hz ≈ 15.0ms

    reg  [15:0]          haas_mem [0:HAAS_LEN-1];
    reg  [HAAS_BITS-1:0] haas_wp, haas_rp;
    reg  signed [15:0]   haas_tap;

    always @(posedge clk_audio) begin
        if (ctrl_en) begin
            haas_mem[haas_wp] <= st_side[15:0];        // 写：本拍的 side
            haas_tap          <= haas_mem[haas_rp];    // 读：TAP 点之前的 side
            haas_wp           <= haas_wp + 1'b1;
            // ★ +1 是为了抵消"读地址是上一拍算出来的"这一拍延迟：
            //   本拍算出的 rp = (wp+1)-TAP，下一拍拿来当地址，正好差 TAP 点。
            haas_rp           <= haas_wp + 1'b1 - HAAS_TAP;
        end
    end

    // Haas 关掉 = 直接用本拍的 side（逐位同一份信号）。
    // ★ 还要等 fx_ready（= 上电填充期结束，复用合唱/混响那一个标志）：
    //   BSRAM 上电内容未知，填充期之前读出来的是垃圾，直接顶上会"啪"一下。
    //   填充期里就用本拍的 side，和延迟/混响那两条线的处理方式一致。
    wire st_haas_ok = HAAS_ON & fx_ready;
    wire signed [18:0] st_side_r = st_haas_ok ? {{3{haas_tap[15]}}, haas_tap}
                                              : st_side;

    // ---- 3) L = mid + side、R = mid - side（中线仍是 sample_vol）----
    //   21 位中间结果：最大 |mid|+|side| = 32767+19453 = 52220，绝不会溢出
    wire signed [20:0] st_l_s = {{5{sample_vol[15]}}, sample_vol}
                              + {{2{st_side[18]}}, st_side};
    wire signed [20:0] st_r_s = {{5{sample_vol[15]}}, sample_vol}
                              - {{2{st_side_r[18]}}, st_side_r};

    // 硬限幅（饱和，不回卷）—— 本工程每一级都这么收尾
    wire signed [15:0] st_l_c = (st_l_s >  21'sd32767) ?  16'sd32767 :
                                (st_l_s < -21'sd32768) ? -16'sd32768 : st_l_s[15:0];
    wire signed [15:0] st_r_c = (st_r_s >  21'sd32767) ?  16'sd32767 :
                                (st_r_s < -21'sd32768) ? -16'sd32768 : st_r_s[15:0];

    // ★★ 总开关：STEREO_ON=0 时两个声道【都】取 sample_vol[15:0]，
    //    和改这一版之前的 mono 完全一致（逐位相同）。
    //    ★★ 注意这里【故意不打拍】：下面 I2S 的装载值直接用 st_l / st_r，
    //       和原来"直接用 sample_vol"是同一个时刻、同一条组合路径。
    //       要是为了缩短路径先打一拍（st_l_q/st_r_q），DAC 就会比原来的
    //       mono 版本【晚一个采样点】（20.5µs）—— 数值序列一样，但
    //       "STEREO_ON=0 逐位回到原样"这句话就不再是严格成立了。
    //       音频域是 12.5MHz（80ns/拍），这条路径（几个 21 位加法 + 限幅）
    //       余量充足，不需要为了时序插寄存器。
    wire signed [15:0] st_l = STEREO_ON ? st_l_c : sample_vol[15:0];
    wire signed [15:0] st_r = STEREO_ON ? st_r_c : sample_vol[15:0];
    //==================================================================
    // ★ 波形显示：把音频采样镜像到 50MHz 域
    //
    //   音频域每产生一个新采样（sample_cnt==10 时锁存进 sample_vol），
    //   就把它推到 wave_send，并翻转 wave_tgl 通知 LCD 域来取。
    //   LCD 域（50MHz）用翻转沿握手，把每个采样写进一个 512 深的环形缓冲。
    //
    //   ★★ 李萨如（X-Y 示波器）的三个轴对（V36 起三屏）★★
    //     左×右档（liss_src=0）：X = st_l（输出级算出的左声道样本）
    //                            Y = st_r（同一拍的右声道样本）
    //     干声档（liss_src=1）：Y = sample_vol（全效果之后的最终输出）
    //                           X = dry_sample（混音后、进效果器之前的干声）
    //     相位档（liss_src=2）：X = sample_vol
    //                           Y = sample_vol 经【一阶全通移相 90°】（见下面）
    //   ★ 四个都在【同一个音频沿】锁进寄存器，共用同一个 wave_tgl 翻沿 ——
    //     所以 LCD 域取到的天生就是同一个采样时刻的 (X,Y)，
    //     不用再开第二条握手线，也不会出现两路采样错拍（那样图形会乱转）。
    //   ★ 纯"取信号"：dry_sample 在这条链路上本来就存在（合唱/延迟都读它），
    //     st_l / st_r 是输出级刚算出来的（同一拍），这里只是多挂两个寄存器
    //     看它们一眼，发声链路一个字节没动。
    //
    // ★★★ 相位档的 Y 轴：一级一阶全通移相器 ★★★
    //
    //   ★★ 硬约束：这个移相器【只喂显示】，绝不进可听通路 ★★
    //     它的输入是 sample_vol，输出只写进 wave_send_py 这一个"给 LCD 看"
    //     的寄存器；sample_vol / s_dly / rvb_mix / DAC / I2S 一个都不读它。
    //     所以【单音的可听输出逐位不变】—— 加它之前之后，
    //     送到 I2S 的 16 位样本序列完全一样。
    //     （这也是它敢放在音频域里的原因：它跟采样节拍同步、跟 wave_send
    //       同拍锁存，天生对齐；但它是个纯旁路 tap，不是信号链的一环。）
    //
    //   差分方程（一阶全通的标准形式）：
    //       y[n] = a*x[n] + x[n-1] - a*y[n-1]
    //   整理成【只用 1 个乘法器】的写法（这一条很重要，DSP 只肯给 1 个）：
    //       y[n] = a*(x[n] - y[n-1]) + x[n-1]
    //              \_______________/
    //               一次乘法
    //
    //   ★ 系数 a 为什么是【负数】-0.96185（用户给的公式差点让这里埋一个坑）★
    //     用户给的 90° 频点近似式  f90 = SR*(1-|a|)/(2*pi*sqrt(|a|))  是对的，
    //     但它是按【极点落在 z=+|a|】那一支写的；而递归式 y=a*x+x1-a*y1 的
    //     极点落在 z=-a。所以要让 90° 落在 300Hz 附近，a 必须取负号：
    //         a = -0.96185  ->  极点 z = +0.96185  ->  f90 = 302Hz   ✔
    //         a = +0.96185  ->  极点 z = -0.96185  ->  f90 = 24.1kHz  ✘（白做）
    //     正号那一版在音频带内的相移只有 0.04°~2°，图形还是那条对角线。
    //     精确解：f90 = SR/4 - (SR/pi)*atan(|a|)，在 |a|→1 时与上面的近似式一致。
    //     Q15 量化：round(0.961842*32768) = 31518 -> a = -31518/32768。
    //
    //   ★ 实测相移（SR = 12.5MHz/256 = 48828Hz，扫过整个键盘音域）：
    //        110Hz ->  -40.00°      220Hz ->  -72.11°
    //        440Hz -> -111.05°      830Hz -> -140.02°
    //     即：以 302Hz 为正中，两端各偏 50°。这是【一级全通能做到的极限】——
    //     一阶全通的相移在带内是单调的，跨度 100°；想再平只能加级数，
    //     但两级一阶全通的相移跨度是两级之和（≥100°，见 testfiles/liss_phase_check.py
    //     里的穷举：两级最优也只有 52.6° 偏差，反而更差），所以【就一级】。
    //     好在两端只是椭圆扁一点（屏上短长轴比 ≈0.34），中音区是正圆，
    //     而且任何音都不会再退化成一条直线（相移永远不落在 0°/180° 上）。
    //==================================================================
    reg signed [15:0] wave_send;
    reg signed [15:0] wave_send_x;
    reg signed [15:0] wave_send_py;      // ★ 相位档的 Y：移相器输出
    // ★ V36 左×右档的两轴：输出级的 L / R，和 wave_send 同拍锁存
    reg signed [15:0] wave_send_l;
    reg signed [15:0] wave_send_r;
    reg               wave_tgl;

    // ---- 一阶全通移相器（Q15 系数，1 个乘法器）----
    //   ★ 位宽为什么这么取（都按"最坏情况不溢出"倒推）：
    //       x[n],y[n-1] ∈ [-32768,32767]
    //       ph_d = x[n]-y[n-1] ∈ [-65535,65535]        -> 17 位有符号刚好
    //       ph_m = ph_d * (-31518)，|ph_m| ≤ 2.066e9 < 2^31  -> 32 位有符号够
    //       ph_ap = ph_m[31:15]（算术右移 15 位）∈ ±63036   -> 17 位
    //       ph_ys = ph_ap + x[n-1] ∈ ±95804                -> 18 位
    //   ★ 末尾那两级饱和只是保险：全通是恒增益的，正常发声不会撞到 ±32767；
    //     但按"上电头几拍状态寄存器是 0、输入可能突变"的瞬态留一手，
    //     免得回卷成反相的大尖峰在屏幕上闪一个亮点。
    localparam signed [15:0] PH_A = -16'sd31518;    // a = -0.961853（Q15）

    reg  signed [15:0] ph_x1;                       // x[n-1]
    reg  signed [15:0] ph_y1;                       // y[n-1]
    wire signed [15:0] ph_x  = sample_vol;          // 输入：全效果后的最终输出
    wire signed [16:0] ph_d  = {ph_x[15],  ph_x} - {ph_y1[15], ph_y1};
    wire signed [31:0] ph_m  = ph_d * PH_A;
    wire signed [16:0] ph_ap = ph_m[31:15];         // >>>15，切片自带符号位
    wire signed [17:0] ph_ys = {{1{ph_ap[16]}}, ph_ap} + {{2{ph_x1[15]}}, ph_x1};
    wire signed [15:0] ph_y  = (ph_ys >  18'sd32767) ?  16'sd32767 :
                               (ph_ys < -18'sd32768) ? -16'sd32768 : ph_ys[15:0];

    // 状态推进和 wave_send 用【同一个节拍判据】，保证 X/Y 是同一个样点
    // ★★ V49：判据加 ctrl_ph —— 送显的节奏保持 48.828 kHz ★★
    //   采样率翻倍后如果照旧"每个采样点送一次"，LCD 那边每秒收到的点
    //   也翻倍 → 波形屏的时间窗减半、李萨如图形密度翻倍，和 V48 的
    //   画面就不一样了（liss 的几何常数都是按 48.828 kHz 标定的）。
    //   所以送显挂到控制节拍上：画面与 V48 逐点一致。
    //   ★ 两个 always 块必须用【完全同一个】判据，否则 (X,Y) 会差一个
    //     采样点、李萨如会出现"斜线分叉"。
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            ph_x1 <= 16'sd0;
            ph_y1 <= 16'sd0;
        end else if ((sample_cnt == 16'd10) && ctrl_ph) begin
            ph_x1 <= ph_x;
            ph_y1 <= ph_y;
        end
    end

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            wave_send    <= 16'sd0;
            wave_send_x  <= 16'sd0;
            wave_send_l  <= 16'sd0;
            wave_send_r  <= 16'sd0;
            wave_send_py <= 16'sd0;
            wave_tgl     <= 1'b0;
        end else if ((sample_cnt == 16'd10) && ctrl_ph) begin
            wave_send    <= sample_vol;
            wave_send_x  <= dry_sample;
            // ★ V36 左×右档的两轴：st_l / st_r 是【组合】由本拍之前的
            //   sample_vol 算出来的，和 wave_send 取到的是同一个样点，
            //   所以这一档的 (X,Y) 天生同拍，不需要额外对齐逻辑。
            //   （★ 不能用打了一拍的 st_l_q/st_r_q：那两个比 wave_send 早
            //     一个采样点，L/R 之间虽然还是同拍，但和其它屏不同拍。）
            wave_send_l  <= st_l;
            wave_send_r  <= st_r;
            // ★ 和 wave_send 同一个沿取同一个样点：wave_send 取到的是
            //   "本拍之前的 sample_vol"，ph_y 也正是拿这个值算出来的，
            //   所以屏幕上 (X,Y) 天生同相位对齐，不需要额外对齐逻辑。
            wave_send_py <= ph_y;
            wave_tgl     <= ~wave_tgl;
        end
    end
    // ★ 只要还有包络没衰减完的通道，I2S 就不能停，
    //   否则松开按键的瞬间延音尾巴会被直接掐掉。
    wire any_key = |sounding_mask;

    //==================================================================
    // I2S 发送（BCK/LCK/DIN）
    //   BCLK_HALF / FRAME_DIV / 采样率已在上面统一定义，此处不再重复声明
    //==================================================================
    reg [3:0] bclk_cnt;
    reg       bclk_r;

    // ★ 诊断：I2S_SPEC=1 时时钟常开（不受 any_key 门控），
    //   方便用 TEST_A/TEST_C 单独验证 DAC 通路。
    // any_key = |sounding_mask（延音尾巴期间也要继续送，否则尾巴被掐掉）
    // TEST_SINE 打开时强制常开，方便单独验证 DAC 通路。
    wire i2s_run = TEST_SINE | (I2S_ALWAYS != 0) | any_key;

    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            bclk_cnt <= 4'd0;
            bclk_r   <= 1'b0;
        end else if (i2s_run) begin
            if (bclk_cnt >= BCLK_HALF - 1'b1) begin
                bclk_cnt <= 4'd0;
                bclk_r   <= ~bclk_r;
            end else begin
                bclk_cnt <= bclk_cnt + 1'b1;
            end
        end else begin
            bclk_cnt <= 4'd0;
            bclk_r   <= 1'b0;
        end
    end

    reg bclk_r_d;
    always @(posedge clk_audio)
        bclk_r_d <= bclk_r;

    wire bclk_fall = (~bclk_r & bclk_r_d);

    reg [5:0]  i2s_bit_cnt;
    reg        ws_r;
    reg        sd_r;
    // V19: 33-bit shift register -- the extra bit holds the standard-I2S
    // dummy slot that precedes each channel's MSB (see the transmitter below).
    reg [32:0] shift_reg;

    // -----------------------------------------------------------------
    // 帧参数（I2S_SPEC / BCLK_HALF / FRAME_BITS 已在音频域开头定义）
    //   48bit 帧 → 左右各 24bit 槽（左对齐 16bit 数据）
    //   64bit 帧 → 左右各 32bit 槽（左对齐 16bit 数据）
    // -----------------------------------------------------------------
    localparam integer I2S_HALF_BITS = FRAME_BITS / 2;
    localparam integer I2S_LAST_BIT  = FRAME_BITS - 1;
    localparam integer I2S_HALF_IDX  = I2S_HALF_BITS - 1;

    // -----------------------------------------------------------------
    // I2S 移位发送
    //   ★ 数据一律【左对齐】放在每声道的高 16 位：
    //       48bit 帧 → {sample[15:0], 8'b0}    （左对齐 24bit 槽，16bit 数据在顶）
    //       64bit 帧 → {sample[15:0], 16'b0}   （左对齐 32bit 槽，16bit 数据在顶）
    //     PCM5102A 是 MSB-first + 2's complement，取每声道开头若干位作为样本，
    //     所以左对齐是正确做法（数据手册对 16bit 数据的标准接法）。
    //   ★ 移位寄存器固定 32 位宽，避免以前那种
    //     「24 位变量被赋 32 位表达式」的截断（EX3791 警告）。
    //     发送时从 bit31 起左移，前 I2S_HALF_BITS 位就是该声道的内容。
    // -----------------------------------------------------------------
    // 静音（any_key=0）时送全 0，而不是停时钟
    //
    // ★★★ V36：左槽 = L、右槽 = R（真正双声道）★★★
    //   原来两个槽装的是同一个 mono 样本（都是 sample_vol）。
    //   现在左槽取 st_l、右槽取 st_r —— 就是输出级刚算出来的
    //   mid ± side（Haas 开时右槽用的是延迟过的 side）。
    //   ★ 数据仍然【左对齐】：{样本[15:0], 16'b0}，位宽/时序一个字没改。
    //   ★ STEREO_ON=0 时 st_l 本身就是 sample_vol[15:0]（三目被常量折叠），
    //     于是两个 load 都退回 {sample_vol[15:0], …} = 改之前的 mono，
    //     而且是【同一个时刻】，不是晚一拍的版本。
    //   ★ 这里只改【数据源】，BCLK_HALF / FRAME_BITS / I2S_SPEC 一个都没碰。
    wire [31:0] i2s_load = any_key
                         ? (I2S_SPEC ? {st_l[15:0], 16'b0}
                                     : {st_l[15:0],  8'b0})
                         : 32'd0;
    // 右声道槽的装载值。
    //   ★ STEREO_ON=0 时它【就是 i2s_load 本身】—— 两个槽逐位相同，
    //     与改立体声之前的行为完全一致（这一条有自检脚本盯着）。
    wire [31:0] i2s_load_r = STEREO_ON
                           ? (any_key
                              ? (I2S_SPEC ? {st_r[15:0], 16'b0}
                                          : {st_r[15:0],  8'b0})
                              : 32'd0)
                           : i2s_load;

    //=========================================================================
    // V19 CRITICAL FIX: the one-BCK I2S delay
    //
    //   Before: data was driven IMMEDIATELY after the LRCK edge
    //   (left-justified): the 1st BCK after LRCK carried sample[15] (MSB).
    //
    //   The PCM5102A in hardware I2S mode follows the STANDARD I2S format:
    //   the 1st BCK after the LRCK edge is a dummy slot and the real MSB
    //   starts on the 2nd BCK.  So the DAC was actually latching
    //   {sample[14:0], next bit} -- the whole sample shifted left by one.
    //
    //   Measured with iverilog by decoding bck/lck/din bit by bit:
    //     DAC word = 8 x sample_vol   (only ~8 effective bits, huge steps),
    //   i.e. low bits scrambled and the top bit lost.  On a scope that is a
    //   coarse staircase, and the strong half of the wave looks flattened.
    //
    //   Fix: insert one dummy BCK per channel and shift the sample one place
    //   right, so the MSB lands exactly on the 2nd BCK after the LRCK edge:
    //       load value {sample[15:0], 16'b0} -> 33 bits {1'b0, ..., 1'b0}
    //       1st BCK sends the dummy 0, 2nd BCK sends sample[15] = MSB
    //   The DAC then receives the sample's full 16 bits, unshifted.
    //
    //   In a 64-bit frame / 32-bit slot there is still room for 15 trailing
    //   zero bits after the right shift.
    //=========================================================================
    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            i2s_bit_cnt <= 6'd0;
            ws_r        <= 1'b0;
            sd_r        <= 1'b0;
            shift_reg   <= 33'd0;
        end else if (bclk_fall) begin
            if (i2s_bit_cnt == I2S_LAST_BIT) begin
                // left-channel start: 1st bit is the dummy slot (one-BCK delay)
                i2s_bit_cnt <= 6'd0;
                ws_r        <= 1'b0;
                sd_r        <= 1'b0;
                shift_reg   <= {1'b0, i2s_load[31:0], 1'b0};
            end else if (i2s_bit_cnt == I2S_HALF_IDX) begin
                // right-channel start: same one-BCK dummy slot
                // ★ V36：右槽装载值从 i2s_load 换成 i2s_load_r（= R 声道）。
                //   两个分支的结构（一个 dummy + 32 位左对齐数据）完全相同，
                //   所以左右槽的位宽/时序关系一个字没变，只是数据源不同。
                i2s_bit_cnt <= I2S_HALF_BITS[5:0];
                ws_r        <= 1'b1;
                sd_r        <= 1'b0;
                shift_reg   <= {1'b0, i2s_load_r[31:0], 1'b0};
            end else begin
                i2s_bit_cnt <= i2s_bit_cnt + 1'b1;
                sd_r        <= shift_reg[32];
                shift_reg   <= {shift_reg[31:0], 1'b0};
            end
        end
    end

    // ★ TEST_SINE=1'b1 时为诊断模式：I2S 常开、数据取测试正弦。
    // ★ 时钟常开（I2S_SPEC=1），数据在静音时自然为 0
    //   （sample_vol 由 sounding_mask 门控算出 0）。
    // ★ I2S_ALWAYS=1 时三根线常开（静音时 DIN 自然是 0）。
    assign bck = i2s_run ? bclk_r : 1'b0;
    assign lck = i2s_run ? ws_r   : 1'b0;
    assign din = i2s_run ? sd_r   : 1'b0;

    //=========================================================================
    // ★★ V44：延迟测量测试脚（外部示波器）★★
    //   详细用法见端口处那段注释。这里只讲实现上的两个要点：
    //     ★ 脉冲【拉宽到约 1µs】、上升沿仍在事件那一拍（只延长后沿）
    //     ★ 两路都用"事件沿"触发，不用电平 —— 音频过零时电平会抖成一片，
    //       沿才是一个干净的触发点（用户选的 (b) 方案）
    //=========================================================================
    //   ★ 宽度 = 2µs（V49；V44.2 时是 10µs）：
    //     ★★ 上限是【被测量间隔】—— T1→T2 只有一个音频帧。
    //        V48 及以前帧周期 20.48µs，10µs 刚好是半个帧；
    //        V49 采样率翻倍后帧周期只剩 10.24µs，再发 10µs 脉冲就会顶到
    //        下一个事件、两路糊在一起量不出来 —— 所以跟着缩到 2µs
    //        （仍远大于示波器带宽需要的宽度，且 ≪ 5.12µs 的半帧）。
    //     ★ 慢时基（20ms/div）下 2µs 只有 0.01 格，更看不见形状了 ——
    //       慢时基只用来【找事件】，量精确间隔请放大到 2~10µs/div 或用延迟扫描。
    localparam [9:0] LAT_HOLD_50M = 10'd99;   // 100 拍 × 20ns  = 2µs
    localparam [7:0] LAT_HOLD_AUD = 8'd24;    // 25 拍 × 80ns   = 2µs

    // ---- T1：琴键端（C19）----
    //   ★ LAT_KEY_TAP 用来【分两段测】，改一个数、重构建一次就能换段，
    //     不用改接线：
    //       0 = 原始按键检测（clk_50m 域）→ 量"触摸扫描"那一段
    //       1 = 跨到音频域之后（key_new）→ 量"合成输出链"那一段
    localparam LAT_KEY_TAP = 0;

    generate
        if (LAT_KEY_TAP == 0) begin : g_lat_key_raw
            // ★ clk_50m 域：TTP229 三片的原始按键状态，任何一位【新按下】就打
            wire [35:0] lat_keys = {kp3, kp2, kp1};
            reg  [35:0] lat_keys_d;
            always @(posedge clk_50m or negedge rst_n) begin
                if (!rst_n) lat_keys_d <= 36'd0;
                else        lat_keys_d <= lat_keys;
            end
            wire lat_evt_raw = |(lat_keys & ~lat_keys_d);
            // ★★★ V44.2：保证【按一次键恰好一个脉冲】★★★
            //   只靠 holdoff 不够：10µs 之后键状态若再跳一次（指抖/坏读数/多键
            //   分几次扫描到）就还会再打一个。这里改成【按住期间锁死】：
            //     按下 → lat_busy 置 1；必须【所有键都松手】才解锁。
            //   于是 "按下…松手" 一个完整周期 = 恰好一个 T1 脉冲。
            //   key_stable 是 ttp229_reader 的 2/2 去抖输出，不是原始扫描值，
            //   所以抖动本身已经被滤过一道。
            reg lat_busy;
            always @(posedge clk_50m or negedge rst_n) begin
                if (!rst_n)                  lat_busy <= 1'b0;
                else if (lat_evt_raw)        lat_busy <= 1'b1;   // 按下 → 锁住
                else if (lat_keys == 36'd0)  lat_busy <= 1'b0;   // 全部松手 → 解锁
            end
            wire lat_evt = lat_evt_raw & ~lat_busy;
            reg  [9:0] lat_hold;
            always @(posedge clk_50m or negedge rst_n) begin
                if (!rst_n)                lat_hold <= 10'd0;
                else if (lat_evt)          lat_hold <= LAT_HOLD_50M;
                else if (lat_hold != 10'd0) lat_hold <= lat_hold - 10'd1;
            end
            assign lat_key = (lat_hold != 10'd0) | lat_evt;
        end else begin : g_lat_key_aud
            // ★ clk_audio 域：按键已经过跨域同步 + 自定义映射（key_new = 新按下的键）
            wire lat_evt = |key_new;
            reg  [7:0] lat_hold;
            always @(posedge clk_audio or negedge rst_n) begin
                if (!rst_n)               lat_hold <= 8'd0;
                else if (lat_evt)         lat_hold <= LAT_HOLD_AUD;
                else if (lat_hold != 8'd0) lat_hold <= lat_hold - 8'd1;
            end
            assign lat_key = (lat_hold != 8'd0) | lat_evt;
        end
    endgenerate

    // ---- T2：PCM 端（C18）----
    //   ★ 取在【实际装载进 I2S 移位寄存器的值】上 —— 那就是真正会到 DAC 的数据。
    //   ★ 判据 = 由 0 变非 0（静音 → 有声）的【沿】，不是电平：
    //     电平会在音频过零时抖成一片，沿才是干净的触发点。
    wire lat_sound = (i2s_load != 32'd0) || (i2s_load_r != 32'd0);
    reg  lat_sound_d;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) lat_sound_d <= 1'b0;
        else        lat_sound_d <= lat_sound;
    end
    // ★★★ V44.2：lat_pcm 必须【每按一次键只出一个脉冲】★★★
    //   ★ 原来的写法是错的：lat_sound = (样本 != 0)，而声音是波形，
    //     【每个周期过零两次】→ lat_sound 跟着 0/1/0/1 跳 →
    //     lat_pcm 会按过零率疯狂出脉冲，根本不是"一个音一个"。
    //   ★ 改法：用【按键事件】来武装，发出一个 T2 之后就撤掉，等下一次按键。
    //     于是严格保证：按一次键 → 恰好一个 T1、一个 T2。
    //   ★ lat_armed 是 reg、lat_sound_d 也是 reg，所以这里没有组合环。
    wire lat_arm_evt = |key_new;          // 音频域的"新按键"（和 lat_key 同一类事件）
    reg  lat_armed;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n)                lat_armed <= 1'b0;
        else if (lat_arm_evt)      lat_armed <= 1'b1;   // 按键 → 武装
        else if (lat_pcm_evt)      lat_armed <= 1'b0;   // 发过就撤
    end
    wire lat_pcm_evt = lat_armed & lat_sound & ~lat_sound_d;
    reg  [7:0] lat_pcm_hold;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n)                    lat_pcm_hold <= 8'd0;
        else if (lat_pcm_evt)          lat_pcm_hold <= LAT_HOLD_AUD;
        else if (lat_pcm_hold != 8'd0) lat_pcm_hold <= lat_pcm_hold - 8'd1;
    end
    assign lat_pcm = (lat_pcm_hold != 8'd0) | lat_pcm_evt;

    //==================================================================
    // ADS1115：4 通道 ADC 采集
    //   与电子琴共用 clk_50m / rst_n，完全独立于 12.5MHz 音频域
    //   AIN0/AIN1 驱动上面的"摇杆移调 / 总音量"
    //   （ain 信号已在前面"摇杆"一节声明）
    //
    //   ★ UART 上报（VOFA+）已按要求整体移除：
    //     串口引脚 uart_tx 已从端口和 music.cst 里删掉，引脚空出来了。
    //     uart.v / uart_rx.v / uart_tx.v / uart_tx_ads.v 这几个文件
    //     还留在 src\ 目录里，但没有被实例化，不参与综合；
    //     哪天想恢复上报，重新接上 uart_tx_ads 即可。
    //==================================================================
    ads1115_ctrl u_ads (
        .clk        (clk_50m),
        .rst_n      (rst_n),
        .i2c_scl    (i2c_scl),
        .i2c_sda    (i2c_sda),
        .ain0_data  (ain0_data),
        .ain1_data  (ain1_data),
        .ain2_data  (ain2_data),
        .ain3_data  (ain3_data)
    );

    //==================================================================
    // Bluetooth + SPI LCD  (ported from the bluetooth project)
    //
    //   Clock: clk_50m_ext = PLL CLKOUT1 = 50MHz
    //     - the original project ran at 100MHz; UART BAUD_DIV was
    //       changed 10417 -> 5208 so the baud rate stays 9600
    //       (same 0.004% error as the original)
    //     - LCD SPI clock is therefore halved (6.25MHz); screen
    //       content is unchanged, refresh takes about twice as long
    //     - the audio domain (clk_audio 12.5MHz) is untouched
    //==================================================================

    // ---- score storage : 256 bytes, written by BT, read by LCD ----
    wire [7:0]  score_addr;
    wire        score_we;
    wire [7:0]  score_waddr;
    wire [7:0]  score_wdata;
    reg  [7:0]  score_data;
    reg  [7:0]  score_mem [0:255];

    always @(posedge clk_50m_ext) begin
        if (score_we)
            score_mem[score_waddr] <= score_wdata;
        score_data <= score_mem[score_addr];
    end

    // ---- BT packet receiver : writes score, raises play_start ----
    wire        play_start;
    wire [7:0]  play_len;
    wire [3:0]  lcd_state;

    // ★ 标准 MIDI 解析出来的按键掩码（Note On/Off -> 36 键）
    //   它就是"标准的琴键弹奏"，所以和物理琴键一样：既发声、也上频谱。
    wire [35:0] midi_keys;
    // ★ V42/V43：只给 LED 用的两条指引掩码（不参与发声）
    //   midi_guide     = 当前格（跟谱模式等待期间也保持显示）
    //   midi_keys_next = 下一格（30% 提前预告）
    wire [35:0] midi_guide;
    wire [35:0] midi_keys_next;
    // ★ V43：跟谱演奏模式（0 = 正常，1 = 跟谱）
    wire        wait_mode;
    // ★ V37：逐键八度（36 键 × 3 位 = 108 位扁平），来自 uart_packet_echo。
    //   第 g 键的八度 = midi_octs[g*3 +: 3]，有符号 3 位，语义同 0xCC。
    //   ★ 它和 midi_keys 是【同一拍、同一个 always 块、同一个 gate】出来的，
    //     所以同步过来之后两者天生配对（见下面 midi_octs_s1/s2 那段）。
    wire [107:0] midi_octs;
    wire [35:0] bt_keys;
    //==================================================================
    // ★ 蓝牙自动演奏
    //
    //   蓝牙把音符字节流(0x00~0x23 = 0~35 号键)写进 score_mem[]，
    //   收到 0xFF 后 play_start 拉高。这里按固定节拍逐个读出音符，
    //   转成 36 位按键掩码，与物理按键合并后一起送进发声链路
    //   —— 于是琴能自己弹，屏幕上的柱子也会跟着跳。
    //
    //   读地址用 play_idx，而 play_idx 恒为 note_idx+1：
    //   score_mem 是同步读、数据晚一拍，这个 +1 正好补偿，
    //   所以当前拍读到的就是 note_idx 号的音符（不是下一个）。
    //
    //   节拍：AUTO_PLAY_DIV = 0.5 秒一个音，想快改小这个常数。
    //==================================================================
    localparam [25:0] AUTO_PLAY_DIV = 26'd25_000_000;   // 50MHz x 0.5s

    reg [5:0]  note_idx;
    reg [25:0] note_cnt;
    reg [7:0]  play_idx;
    reg [35:0] auto_keys;
    reg        auto_en;        // ★ 只有收到过数据才使能自动演奏

    // ★ play_start 是电平保持的，必须取上升沿；
    //   否则每拍都复位 note_idx，自动演奏会卡在第一个音。
    reg        play_start_d;
    wire       play_start_pulse = play_start & ~play_start_d;

    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            note_idx     <= 6'd0;
            note_cnt     <= 26'd0;
            play_idx     <= 8'd0;
            auto_en      <= 1'b0;
            play_start_d <= 1'b0;
        end else begin
            play_start_d <= play_start;

            // 读地址恒为 note_idx+1（补偿同步读的 1 拍延迟）
            play_idx <= {2'b0, note_idx} + 8'd1;

            if (play_start_pulse) begin
                // 新曲谱到达：只在这一拍复位一次，并使能自动演奏
                note_idx <= 6'd0;
                note_cnt <= 26'd0;
                auto_en  <= 1'b1;
            end
            else if (note_cnt >= AUTO_PLAY_DIV) begin
                note_cnt <= 26'd0;
                if (play_len == 8'd0)                  note_idx <= 6'd0;
                else if (note_idx >= play_len - 1'b1)  note_idx <= 6'd0;   // 循环演奏
                else                                   note_idx <= note_idx + 1'b1;
            end
            else begin
                note_cnt <= note_cnt + 1'b1;
            end
        end
    end

    // 音符 -> 36 位按键掩码
    // ★ 只在使能后才点亮：否则上电时 note_idx=0 会让 0 号键（第一根柱子）
    //   永久高电平 —— 屏幕上就是"00 那根柱一直亮、对应琴键没反应"。
    always @(*) begin
        auto_keys = 36'd0;
        if (auto_en) auto_keys[note_idx] = 1'b1;
    end

    // ★ 屏幕频谱【只跟物理琴键走】
    //
    //   对应关系：按第 i 个物理琴键 -> key_levels 第 (i-1) 位 -> 第 (i-1) 根柱子
    //   lcd_master 里 bar_key_on = key_levels[play_idx]、
    //              bar_x      = 16 + play_idx*5
    //   两者共用 play_idx，所以柱号 = 琴键号，严格一一对应。
    //
    //   ★ 绝对不要把 auto_keys 并进来：蓝牙自动演奏时会点亮音符对应的柱子，
    //     频谱就不再是"你按下的琴键"了 —— 这正是之前频谱对不上的原因。
    //     自动演奏只影响发声（见下面 auto_keys_aud -> key_mask）。
    // ★ 频谱来源 = 物理琴键 ∪ MIDI 音符（两者都是"标准的按键事件"，位置一一对应）
    //   注意不要并入 auto_keys：它是缓冲播放的常驻音符，会让柱子一直占着。
    // 屏幕频谱来源 = 物理琴键（去抖后的稳定快照）∪ MIDI 音符
    //   ★ 用两级同步的 pressed_mask（= ttp229_reader 里 key_stable 的拼接），
    //     不用自动演奏音符：它是缓冲播放的常驻音符，会让柱子一直占着。
    //     （原来这里用的是 last_pressed，它只在事件沿更新，显示会抖；
    //       现在直接跟稳定快照走，和 music_test0 的做法一致。）
    // ★ 频谱来源 = 音频域真实发声状态（谁 env != 0 谁亮）
    //   ★★ V30：这一路【不再】并成 36 位往 LCD 送 ★★
    //     原来这里是 sounding_mask -> 两级同步 -> key_levels[35:0] 并行送出去。
    //     现在改成音频域里那套 scan_idx/scan_val/scan_tgl 扫描口（见上面
    //     "送给 LCD 的每键电平"那一节）：LCD 那边自己维护 36 项电平表，
    //     于是每根柱子显示的是【它自己那个键】当前的包络高度，
    //     而不是"响=1"的等高柱子。
    //     sounding_s1/s2/key_levels 这三个信号因此删掉了 —— 它们的最后
    //     一个使用者（u_lcd 的 key_levels 端口）也一起换成了扫描口。

    // ---- 自动演奏音符跨到音频域参与发声（两级同步，慢信号足够）----
    reg [35:0] auto_keys_s1, auto_keys_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            auto_keys_s1 <= 36'd0;
            auto_keys_s2 <= 36'd0;
        end else begin
            auto_keys_s1 <= auto_keys;
            auto_keys_s2 <= auto_keys_s1;
        end
    end
    wire [35:0] auto_keys_aud = auto_keys_s2;

    // ★ MIDI 音符跨到音频域参与发声（两级同步，50MHz->12.5MHz 慢信号足够）
    reg [35:0] midi_keys_s1, midi_keys_s2;
    // ★★ V32：乐谱音符的"起音沿"要用来锁存八度偏移 ★★
    //   midi_keys_s3 比 s2 晚一拍 → s2 & ~s3 就是【多等了一拍】的上升沿。
    //   为什么要多等这一拍：八度偏移（oct_s2）和音符掩码是同时从 UART 域
    //   变过来的，刚同步过来的那一拍，3 位总线理论上可能是个中间态；
    //   等一拍再采，它一定已经稳了（慢信号，变化间隔 >= 20ms）。
    reg [35:0] midi_keys_s3;
    reg [2:0]  oct_s1, oct_s2;
    // ★★ V37：逐键八度（108 位）的同步链 ★★
    //   ★ 只打【两级】：midi_keys 的"当前格"就是 midi_keys_s2（score_new
    //     用的就是它），逐键八度必须与它同拍对齐，所以同样两级。
    //   ★ 108 位比 36 位宽得多，但它和 midi_keys 一样是慢信号 ——
    //     整个事件期间（>= 20ms）保持不变，两级同步足够。
    //   ★ 为什么要 108 位一起过：uart_packet_echo 那边 midi_keys 与 midi_octs
    //     是同一个 always 块、同一个 gate 出来的，两条链级数一样之后，
    //     掩码和它的八度就永远是同一个采样点的一对。
    reg [107:0] midi_octs_s1, midi_octs_s2;
    // ★ V42/V43：两条【只给 LED 用】的指引掩码的同步链。
    //   ★ 级数必须和 midi_keys 完全一样（都是两级）——
    //     这样"当前格 / 下一格 / 手弹"在音频域里是同一采样点的一对，
    //     不会出现"新当前配旧预告"。
    //   ★ 它俩和 midi_keys 一样是慢信号（整个事件期间 >= 20ms 不变），两级足够。
    reg [35:0] midi_guide_s1, midi_guide_s2;
    reg [35:0] midi_keys_next_s1, midi_keys_next_s2;
    // ★ V43：跟谱模式的同步链（给 led_guide 用来决定 30% 预告要不要显示）
    reg wait_mode_s1, wait_mode_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            midi_keys_s1 <= 36'd0;
            midi_keys_s2 <= 36'd0;
            midi_keys_s3 <= 36'd0;
            midi_octs_s1 <= 108'd0;
            midi_octs_s2 <= 108'd0;
            midi_guide_s1 <= 36'd0;
            midi_guide_s2 <= 36'd0;
            midi_keys_next_s1 <= 36'd0;
            midi_keys_next_s2 <= 36'd0;
            wait_mode_s1 <= 1'b0;
            wait_mode_s2 <= 1'b0;
            oct_s1       <= 3'd0;
            oct_s2       <= 3'd0;
        end else begin
            midi_keys_s1 <= midi_keys;
            midi_keys_s2 <= midi_keys_s1;
            midi_keys_s3 <= midi_keys_s2;
            midi_octs_s1 <= midi_octs;
            midi_octs_s2 <= midi_octs_s1;
            midi_guide_s1 <= midi_guide;
            midi_guide_s2 <= midi_guide_s1;
            midi_keys_next_s1 <= midi_keys_next;
            midi_keys_next_s2 <= midi_keys_next_s1;
            wait_mode_s1 <= wait_mode;
            wait_mode_s2 <= wait_mode_s1;
            oct_s1       <= oct_cur;
            oct_s2       <= oct_s1;
        end
    end
    wire [35:0] midi_keys_aud = midi_keys_s2;
    // ★★ V47：逐事件力度（0~127）从 clk_50m_ext 同步到音频域 ★★
    //   ★ 它是【准静态】的：一个事件持续几十毫秒以上，值在整个事件里不变，
    //     所以两级同步器足够（和 midi_octs 同一性质）。
    //   ★ 复位值给 127（满幅）—— 这样上电初始状态就等于"没有力度"，
    //     和以前逐位相同。
    //   ★ midi_vel / midi_vel_aud 两条 wire 已经在前面统一声明过了
    //     （必须提前声明，见那里的注释）。
    reg  [6:0] midi_vel_s1, midi_vel_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            midi_vel_s1 <= 7'd127;
            midi_vel_s2 <= 7'd127;
        end else begin
            midi_vel_s1 <= midi_vel;
            midi_vel_s2 <= midi_vel_s1;
        end
    end
    assign midi_vel_aud = midi_vel_s2;
    // ★ V42/V43：音频域的两条指引掩码 + 跟谱模式（都和 midi_keys_aud 同拍）
    wire [35:0] midi_guide_aud     = midi_guide_s2;
    wire [35:0] midi_keys_next_aud = midi_keys_next_s2;
    wire        wait_mode_aud      = wait_mode_s2;
    // ★ 乐谱自己的起音沿（手弹 / 自动演奏不算）
    wire [35:0] score_new = midi_keys_s2 & ~midi_keys_s3;

    //------------------------------------------------------------------
    // ★★ V32：每个声部自己的八度偏移（跟着它那次起音走）★★
    //   · 乐谱弹的音：用它所在那一格的偏移（oct_s2 就是当前事件的偏移）
    //   · 手弹 / 其它来源：一律回原调 0
    //   ★ 为什么必须"每个声部各存一份"：inc[] 是所有声部共用的，
    //     用全局偏移去改它，正在放释音尾巴的音会被一起搬八度 —— 尾巴会跳音。
    //   存储只有 36×3 = 108 个触发器；读的时候只有轮询那一个读者（36:1 三选），
    //   所以很便宜（见上面 tr_oct 那段）。
    //------------------------------------------------------------------
    generate
        for (gt = 0; gt < NUM_KEYS; gt = gt + 1) begin : gen_voct
            always @(posedge clk_audio or negedge rst_n) begin
                if (!rst_n)                   voice_oct[gt] <= 3'd0;
                // ★ V37：乐谱起音改用【这个键自己的】八度 —— 0xCD 指定过就用它，
                //   没指定过就是整格八度（uart_packet_echo 落盘时已经烘好）。
                //   ★ 老谱子（一个 0xCD 都没有）里 midi_octs_s2 的每三位都等于
                //     oct_s2，所以这里取到的值与 V36【逐位相同】。
                else if (score_new[gt])       voice_oct[gt] <= midi_octs_s2[gt*3 +: 3];
                else if (key_new[gt])         voice_oct[gt] <= 3'd0;
            end
        end
    endgenerate


    // 延音锁存从音频域同步回 50MHz 域（仅用于屏幕显示）
    reg [NUM_KEYS-1:0] sustain_latch_a_s1, sustain_latch_a_s2;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sustain_latch_a_s1 <= {NUM_KEYS{1'b0}};
            sustain_latch_a_s2 <= {NUM_KEYS{1'b0}};
        end else begin
            sustain_latch_a_s1 <= sustain_latch_aud;
            sustain_latch_a_s2 <= sustain_latch_a_s1;
        end
    end
    wire [NUM_KEYS-1:0] sustain_latch_a_sync = sustain_latch_a_s2;

    //==================================================================
    // ★★ 诊断：按键数据 UART 上报（DIAG_UART）★★
    //
    //   DIAG_UART = 1'b1 时，uart_txd 不走 packet_echo，改成每约 0.4s
    //   发送一行按键状态，ASCII 9600 8N1：
    //
    //       M=hhhhhhhhhh K=nn

    //
    //   M= 后面 10 位十六进制对应 40 位，低 36 位是 pressed_mask
    //      （bit0 = 琴键1 ... bit35 = 琴键36，高 4 位恒 0）
    //   K= 是当前按下键的编号 1~36，无键按下时为 '0'
    //
    //   用法：串口助手 9600 打开板载调试器那个口，逐个按琴键看 M 的位：
    //     · 按了键但对应位一直 0 -> TTP229 没读到该键（模式/上拉/接线）
    //     · 位正常变 1 但没声音   -> 音频侧的问题
    //   测完改回 1'b0 即恢复原来的 packet_echo。
    //==================================================================
    localparam DIAG_UART = 1'b0;   // ★ 走蓝牙乐谱协议（=1 时改成诊断上报，乐谱包不回传）

    // 起帧脉冲（先声明，后面多个 always 都要用）
    reg  [24:0] dg_cnt;
    reg         dg_start_r;
    wire        dg_start = (dg_cnt >= 25'd20_000_000) & ~dg_start_r;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) dg_start_r <= 1'b0;
        else        dg_start_r <= (dg_cnt >= 25'd20_000_000);
    end

    // 分频：每约 0.4s 起一帧（50MHz x 0.4s = 20_000_000）
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) dg_cnt <= 25'd0;
        else if (dg_start) dg_cnt <= 25'd0;
        else               dg_cnt <= dg_cnt + 1'b1;
    end

    // 发送节拍：等 uart_tx 应答(wready_o)后再发下一个字节
    wire        dg_wready;      // ★ 显式声明：原来靠隐式网络，综合日志报 Undeclared symbol
    reg [4:0] dg_idx;
    wire dg_firing_now = (dg_idx >= 5'd1) && (dg_idx <= 5'd16);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) dg_idx <= 5'd0;
        else if (dg_start) begin
            dg_idx <= 5'd0;
        end else if (dg_firing_now && dg_wready) begin
            dg_idx <= (dg_idx >= 5'd16) ? 5'd17 : (dg_idx + 1'b1);
        end
    end

    // 当前按下键的编号（0 = 无）
    wire [5:0] dg_keynum = pressed_mask[0]  ? 6'd1  : pressed_mask[1]  ? 6'd2  :
                           pressed_mask[2]  ? 6'd3  : pressed_mask[3]  ? 6'd4  :
                           pressed_mask[4]  ? 6'd5  : pressed_mask[5]  ? 6'd6  :
                           pressed_mask[6]  ? 6'd7  : pressed_mask[7]  ? 6'd8  :
                           pressed_mask[8]  ? 6'd9  : pressed_mask[9]  ? 6'd10 :
                           pressed_mask[10] ? 6'd11 : pressed_mask[11] ? 6'd12 :
                           pressed_mask[12] ? 6'd13 : pressed_mask[13] ? 6'd14 :
                           pressed_mask[14] ? 6'd15 : pressed_mask[15] ? 6'd16 :
                           pressed_mask[16] ? 6'd17 : pressed_mask[17] ? 6'd18 :
                           pressed_mask[18] ? 6'd19 : pressed_mask[19] ? 6'd20 :
                           pressed_mask[20] ? 6'd21 : pressed_mask[21] ? 6'd22 :
                           pressed_mask[22] ? 6'd23 : pressed_mask[23] ? 6'd24 :
                           pressed_mask[24] ? 6'd25 : pressed_mask[25] ? 6'd26 :
                           pressed_mask[26] ? 6'd27 : pressed_mask[27] ? 6'd28 :
                           pressed_mask[28] ? 6'd29 : pressed_mask[29] ? 6'd30 :
                           pressed_mask[30] ? 6'd31 : pressed_mask[31] ? 6'd32 :
                           pressed_mask[32] ? 6'd33 : pressed_mask[33] ? 6'd34 :
                           pressed_mask[34] ? 6'd35 : pressed_mask[35] ? 6'd36 : 6'd0;

    // 4bit -> 十六进制 ASCII（Verilog 不支持字符串字面量索引，必须用函数）
    function [7:0] hexasc;
        input [3:0] v;
        begin
            hexasc = (v < 4'd10) ? (8'd48 + {4'b0, v})
                                 : (8'd55 + {4'b0, v});   // 'A'=65, 65-10=55
        end
    endfunction

    wire [7:0] dg_hexhi = hexasc(pressed_mask[35:32]);
    wire [7:0] dg_hex1  = hexasc(pressed_mask[31:28]);
    wire [7:0] dg_hex2  = hexasc(pressed_mask[27:24]);
    wire [7:0] dg_hex3  = hexasc(pressed_mask[23:20]);
    wire [7:0] dg_hex4  = hexasc(pressed_mask[19:16]);
    wire [7:0] dg_hex5  = hexasc(pressed_mask[15:12]);
    wire [7:0] dg_hex6  = hexasc(pressed_mask[11:8]);
    wire [7:0] dg_hex7  = hexasc(pressed_mask[7:4]);
    wire [7:0] dg_hex8  = hexasc(pressed_mask[3:0]);

    // 按实际值发送的十位十进制键号（" 0"~"36"）
    wire [7:0] dg_k10 = (dg_keynum >= 6'd10) ? (8'd48 + ({2'b0, dg_keynum} / 10)) : 8'd32;
    wire [7:0] dg_k1  = 8'd48 + (dg_keynum % 10);

    reg [7:0] dg_char;
    always @(*) begin
        case (dg_idx)
            5'd1:  dg_char = "M";
            5'd2:  dg_char = "=";
            5'd3:  dg_char = dg_hexhi;
            5'd4:  dg_char = dg_hex1;
            5'd5:  dg_char = dg_hex2;
            5'd6:  dg_char = dg_hex3;
            5'd7:  dg_char = dg_hex4;
            5'd8:  dg_char = dg_hex5;
            5'd9:  dg_char = dg_hex6;
            5'd10: dg_char = dg_hex7;
            5'd11: dg_char = dg_hex8;
            5'd12: dg_char = " ";
            5'd13: dg_char = "K";
            5'd14: dg_char = "=";
            5'd15: dg_char = dg_k10;
            5'd16: dg_char = dg_k1;
            default: dg_char = 8'h00;
        endcase
    end

    wire diag_txd;
    uart_tx u_diag_tx (
        .clk      (clk_50m),
        .rst      (rst_n),
        .parity_i (2'd0),          // 无校验
        .wdata_i  (dg_char),
        .baud_i   (16'd5208),      // 50MHz / 5208 = 9600
        .wvalid_i (dg_firing_now),
        .wready_o (dg_wready),
        .tx_o     (diag_txd)
    );

    // DIAG_UART=1 -> 诊断上报；=0 -> 原来的 packet_echo
    wire uart_pkt_txd;
    assign uart_txd = DIAG_UART ? diag_txd : uart_pkt_txd;

    //==================================================================
    // ★ 录音回传：把"板子打算让哪些键响"和"录音按钮"送到蓝牙模块那边
    //
    //   uart_packet_echo 跑的是 clk_50m_ext；而
    //     · sounding_mask_aud 在 clk_audio（12.5MHz）
    //     · 按钮（按钮板第 10 键）在 clk_50m
    //   两者都要跨过来。
    //
    //   ★★ 为什么送 sounding_mask_aud（门控）而不是 sounding_mask（包络输出）★★
    //     sounding_mask     = env[键] != 0                    —— 包络输出，带释音尾巴
    //     sounding_mask_aud = key_mask | sustain_latch_aud    —— 门控，"打算让它响"
    //
    //   送带尾巴的那个看着更"实际"，其实不行，两个原因：
    //     ① ★ 分不出"重击同一个键"：第一次的尾巴还没走完就再按一次，
    //        包络非 0 一直成立 —— 掩码连着不变，两下被合成一个长音。
    //        实测（diag_rec_timing.py）：音高 62 弹两次（1000ms / 2600ms）
    //        合成了一个 3730ms 的长音。
    //     ② 掩码里"一直为 1"的键会在【每个事件】上重复触发 ——
    //        8 个音弹出来是 32 次起音，听感完全不是原来那回事。
    //   门控信号的好处：起音 / 放音 / 重击的边沿全是真的；延音锁存也天然
    //   在里面（sounding_mask_aud 本身就 = key_mask | sustain_latch_aud）。
    //
    //   释音尾巴改由【上位机按 RTL 的常数补】：松手后先 8ms 消抖，再按
    //   ENV_SUSTAIN_LV × env_rel_div / 采样率 衰减到静音
    //   （默认 255×280/48828 ≈ 1.46 秒；笛子 200×61 ≈ 0.25 秒）。
    //   这几个常数由 check_rtl_contract.py 盯着 —— RTL 改了、上位机没跟着改，
    //   校验会报红。
    //
    //   录音按钮是个 20ns 的单拍脉冲，直接跨域必丢 —— 所以先做成
    //   "翻转保持"（btn_rec_hold），过来之后再用边沿检测还原成单拍脉冲。
    //   这和 top.v 里摇杆那个 toggle 握手的思路一样。
    //==================================================================
    reg btn_rec_hold;                       // clk_50m 域：按一下翻一次
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        btn_rec_hold <= 1'b0;
        else if (btn_new[9]) btn_rec_hold <= ~btn_rec_hold;   // 按钮板第 10 键（1~9 已有功能）
    end

    reg btn_rec_s1, btn_rec_s2, btn_rec_s3;                 // clk_50m_ext 域：三级同步
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            btn_rec_s1 <= 1'b0;
            btn_rec_s2 <= 1'b0;
            btn_rec_s3 <= 1'b0;
        end else begin
            btn_rec_s1 <= btn_rec_hold;
            btn_rec_s2 <= btn_rec_s1;
            btn_rec_s3 <= btn_rec_s2;
        end
    end
    wire btn_rec_pulse = btn_rec_s2 ^ btn_rec_s3;           // 还原成 1 拍脉冲

    //==================================================================
    // ★★ V43：TTP 按键13 -> 翻转"跟谱演奏模式" ★★
    //
    //   完全照抄上面录音按钮那一套（同一套做法，不引入新写法）：
    //     clk_50m 域翻转保持 -> 三级同步到 clk_50m_ext -> 边沿还原成单拍脉冲
    //   ★ 真正翻转 wait_mode 的动作在 uart_packet_echo 里做（和 0xCE 共用
    //     同一个寄存器，只能有一个驱动者）—— 这里只送一个脉冲过去。
    //   ★ 用一个【空闲】按钮：按钮 1..10 已经有功能了（见上面那一堆 btn_*），
    //     按键13 = btn_new[12]，之前没分配过。
    //==================================================================
    reg btn_mode_hold;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)          btn_mode_hold <= 1'b0;
        else if (btn_new[12]) btn_mode_hold <= ~btn_mode_hold;   // 按钮板第 13 键
    end

    reg btn_mode_s1, btn_mode_s2, btn_mode_s3;
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            btn_mode_s1 <= 1'b0;
            btn_mode_s2 <= 1'b0;
            btn_mode_s3 <= 1'b0;
        end else begin
            btn_mode_s1 <= btn_mode_hold;
            btn_mode_s2 <= btn_mode_s1;
            btn_mode_s3 <= btn_mode_s2;
        end
    end
    wire btn_mode_tgl = btn_mode_s2 ^ btn_mode_s3;          // 1 拍脉冲

    // ★★★ V43.7：跟谱模式判"按对了没有"——在 clk_audio 域算，只同步 1 位 ★★★
    //   ★ 为什么不在 clk_50m_ext 那边比：V43.6 试过（把 36 位 hand_mask 同步过去），
    //     加了一条 72 个触发器的同步链 + 一个 36 位比较，实测 CLKOUT1 从
    //     59.197 掉到 47.157MHz，而且【重跑结果一模一样】= 真实退化，不是撞运气。
    //   ★ 现在在这里算：clk_audio 域本来就有 hand_mask，而且
    //     midi_guide_aud 就是【当前格】的指引掩码（级数和 midi_keys 一样）。
    //     本域余量 2.3 倍，36 位比较放这儿很便宜；跨域只走 1 位。
    //   ★★ 判据必须是 hand_mask（手弹物理按键），【绝不能】用
    //      sounding_mask_aud —— 那个里含 midi_keys_aud（乐谱自己的音），
    //      用它判会让乐谱一响就"过关"，表现是"一开跟谱就自己往下播"。
    wire keys_ok_aud = ((hand_mask & midi_guide_aud) == midi_guide_aud);

    // ★★★ V48.4：这一位也【先在音频域打一拍，再跨到 clk_50m_ext】★★★
    //   和下面 `sounding_mask_gate` 是同一个治法、同一个原因：
    //   `keys_ok_aud` 是 36 位与比较的**组合输出**，直接接同步器第一级
    //   (`keys_ok_s1`) 就成了一条【跨域 + 长组合】的链：
    //       midi_guide_s1(clk_audio) -> 36 位展开 -> midi_guide_aud
    //         -> 36 位与/比较 -> keys_ok_s1.D(clk_50m_ext)
    //   V48.3 归档时它还有 **+0.581ns** 余量，当时在版本说明里写明了
    //   "**以后再往音频域/按键掩码那一带加逻辑，这条是第一个要看的**" ——
    //   V48.4 加 bar_bad（纯 LCD 域）之后布局一扰动，它果然翻成 **-0.771ns**
    //   （全工程唯一 setup 违例，CLKOUT0 -> CLKOUT1）。
    //   ★ 打一拍之后：长链的终点变成 clk_audio 自己的寄存器（要求 80ns，
    //     余量几十 ns），跨域那段只剩"寄存器 -> 寄存器"的布线延迟。
    //   ★ 代价：跟谱判据晚 80ns。这是"人按下键没有按对"的判据，
    //     时间尺度是几十 ms，80ns 完全无感（`keys_ok_s1/s2` 本来就是
    //     两级同步过来再用的用法）。
    //   ★★ `keys_ok_aud` 本身一个字节没动，音频域内部用法不变。
    reg keys_ok_gate;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) keys_ok_gate <= 1'b0;
        else        keys_ok_gate <= keys_ok_aud;
    end

    reg  keys_ok_s1, keys_ok_s2;
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            keys_ok_s1 <= 1'b0;
            keys_ok_s2 <= 1'b0;
        end else begin
            keys_ok_s1 <= keys_ok_gate;      // ★ V48.4：已在音频域打过一拍
            keys_ok_s2 <= keys_ok_s1;
        end
    end

    //------------------------------------------------------------------
    // ★★ V48.3：录音门控掩码【先在音频域打一拍，再跨到 clk_50m_ext】★★
    //
    //   为什么要加这一拍（这是 V48.3 构建后 STA 报出来的**全工程唯一 setup 违例**）：
    //     `rec_mask_s1` 是两级同步器的第一级，它的 D 端原来直接接【纯组合】的
    //     `sounding_mask_aud`，而 `sounding_mask_aud = key_mask | sustain_latch_aud`，
    //     `key_mask` 里又含 `midi_keys_aud`（从当前格的指引掩码展开出来的组合逻辑）。
    //     于是形成一条【跨两个时钟域、四层组合】的长链：
    //         midi_guide_s1(clk_audio) -> 展开 -> key_mask
    //           -> sounding_mask_aud -> rec_mask_s1.D(clk_50m_ext)
    //     实测：`SETUP slack = -0.155ns`（CLKOUT0 -> CLKOUT1，
    //     端点 `rec_mask_s1_0_s18/DI[1]`，数据延迟 20.120ns / 要求 20.000ns）。
    //
    //   ★ 为什么 V48.2 没报：那时这条链还有 +1.03ns 以上的余量，
    //     而这一版加了小号音色（音频域）和 js_z 流水（50M 域）之后**布局扰动**，
    //     瓶颈就从 js_z 跳到了这条链上 —— 坑 45 附注早就写过"插完流水要重新看
    //     三个域的 Fmax，因为瓶颈会从一条链跳到另一条链"，这次跳的是跨域链。
    //
    //   ★ 治法就是本项目一直用的"插流水"：在【源域】先打一拍，
    //     让跨域那一段变成【寄存器 -> 寄存器】（组合逻辑几乎为零）。
    //     · 这条链的终点改到 clk_audio 自己的寄存器上之后，要求时间从 20ns
    //       变成 80ns（clk_audio 周期），余量一下子有几十 ns；
    //     · 跨域那段只剩布线延迟（ns 级），对 20ns 的要求毫无压力。
    //   ★ 代价：门控掩码晚 80ns（一个 clk_audio 周期）。录音事件的时标是 ms 级，
    //     80ns 完全看不见；`rec_mask_s1/s2` 本来就是"同步过来再比"的用法。
    //   ★★ `sounding_mask_aud` 本身【一个字节都没动】★★ —— 音频域内部
    //     （包络起音判据、`n_voice` 计数）用的还是原来那个组合值，
    //     所以**声音逐位不变**；这里只是【多引出一个打拍版本】专门用于跨域。
    //------------------------------------------------------------------
    reg [35:0] sounding_mask_gate;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) sounding_mask_gate <= 36'd0;
        else        sounding_mask_gate <= sounding_mask_aud;
    end

    reg [35:0] rec_mask_s1, rec_mask_s2;              // 门控掩码跨到 clk_50m_ext
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            rec_mask_s1 <= 36'd0;
            rec_mask_s2 <= 36'd0;
        end else begin
            rec_mask_s1 <= sounding_mask_gate;          // ★ 门控（按键 | 延音锁存），已在音频域打过一拍
            rec_mask_s2 <= rec_mask_s1;
        end
    end

    //------------------------------------------------------------------
    // ★ 录音回传：影响声音的 8 个效果开关
    //
    //   上位机要把"效果"在 MIDI 里近似还原，就得知道这些开关什么时候动过。
    //   全是二值开关（按一下翻一次），8 位正好一个字节：
    //     bit0 滑音 glide_en   bit1 延音锁存 sustain_on
    //     bit2 音色 timbre_50m bit3 颤音 trem_on
    //     bit4 延迟 dly_on     bit5 笛子 flute_on
    //     bit6 混响 rev_on     bit7 合唱 cho_on
    //
    //   ★ 按钮 3 的 wave_mode 故意不在里面：它只喂给 lcd_master（频谱/波形显示），
    //     对音频链路一个采样点的影响都没有，录它没意义。
    //
    //   ★ 这一整块是【只读】的 —— 只是把这几个寄存器"看"一眼，
    //     一根线都没有连回效果链路，效果本身的行为一个字节没变。
    //     所以它不会影响出声，也不会影响任何按钮。
    //     （效果寄存器都在 clk_50m 域，按钮是人手按的，变化间隔以秒计，
    //       两级同步足够；就算偶尔采到一个中间态，10ms 后那一帧就纠正回来了。）
    //------------------------------------------------------------------
    wire [7:0] fx_state = {cho_on,      // bit7 合唱
                           rev_on,      // bit6 混响
                           flute_on,    // bit5 笛子
                           dly_on,      // bit4 延迟
                           trem_on,     // bit3 颤音
                           timbre_50m,  // bit2 音色
                           sustain_on,  // bit1 延音锁存
                           glide_en};   // bit0 滑音

    reg [7:0] fx_s1, fx_s2;
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            fx_s1 <= 8'd0;
            fx_s2 <= 8'd0;
        end else begin
            fx_s1 <= fx_state;
            fx_s2 <= fx_s1;
        end
    end

    uart_packet_echo u_uart_packet_echo (
        .clk         (clk_50m_ext),
        .rst_n       (rst_n),
        .uart_rxd    (uart_rxd),
        .uart_txd    (uart_pkt_txd),
        .gate_mask_in (rec_mask_s2),   // ★ 录音：门控掩码（按键 | 延音锁存）
        // ★ V43.7：跟谱模式判"按对了没有"——已在 clk_audio 域算好，只同步 1 位。
        //   ★ 判据用的是 hand_mask（手弹物理按键），不是 sounding_mask_aud
        //     —— 后者含乐谱自己的音，会让跟谱模式"自己往下播"。
        .keys_ok_in  (keys_ok_s2),
        .fx_state_in (fx_s2),              // ★ 录音：影响声音的 8 个效果开关
        .lcd_rst_tgl (lcd_rst_tgl),        // ★★ V48.5：0xD1 手动重置 LCD 的请求沿
        .btn_rec     (btn_rec_pulse),      // ★ 录音开关按钮（按钮板第 10 键）
        // ★ 按键自定义映射的写口：UART 域(clk_50m_ext) -> key_map 所在的音频域(clk_audio)
        //   由上面那段"翻转信号 + 边沿检测"接收
        .keymap_we_tgl (keymap_we_tgl),
        .keymap_key    (keymap_key),
        .keymap_note   (keymap_note),
        // ★ V31：效果开关 / 音色的写口（同样的翻转沿 + 数据握手），
        //   以及"当前音色"送过去给上位机回报用
        .fx_tgl        (fx_tgl),
        .fx_bit        (fx_bit),
        .tone_we_tgl   (tone_we_tgl),
        .tone_val      (tone_val),
        .tone_sel_in   (tone_sel),
        // ★ V32：当前事件的八度偏移（给 voice_oct[] 用，见上面"共享移调路径"）
        .oct_cur       (oct_cur),
        .score_we    (score_we),
        .score_waddr (score_waddr),
        .score_wdata (score_wdata),
        .play_start  (play_start),
        .play_len    (play_len),
        .midi_keys   (midi_keys),
        // ★ V42/V43：只给 LED 用的两条指引掩码 + 跟谱模式 + 按键13
        //   ★ 这三条都不参与发声（midi_keys 才是发声用的那条，没动）。
        .midi_guide      (midi_guide),
        .midi_keys_next  (midi_keys_next),
        .wait_mode       (wait_mode),
        .btn_mode_tgl    (btn_mode_tgl),
        .midi_octs   (midi_octs),      // ★ V37：逐键八度（36 键 × 3 位 = 108 位）
        .bt_keys     (bt_keys),
        // ★★ V45：节拍器（板载 READY 灯 U12）★★
        //   ★ 这四条是【纯新增】的输出，节拍器不参与任何其它模块。
        //   ★ metro_beat_in 只用于状态回报（0x5E 帧的第 5 个字节），
        //     板子自己判断节拍【不看它】。
        .metro_bpm   (metro_bpm),
        .metro_num   (metro_num),
        .metro_den   (metro_den),
        .metro_tgl   (metro_tgl),
        .metro_beat_in (metro_beat_ext),
        // ★★ V48：LCD 显示模式（0xD0 下行设定 / 0x5F 上行回报）★★
        //   ★ 这两条路【都不走 armed 开窗】：显示模式随时可以设，
        //     和 0xC9/0xCA/0xCE/0xCF 一样不受"正在发谱子"的限制。
        //   · disp_tgl / disp_val = 写口（本模块 -> top.v 的 clk_50m 域）
        //   · disp_mode_rep       = 板子当前生效值（top.v 的 clk_50m_ext 域，
        //                           已经在上面做完跨时钟握手，本域直接用）
        .disp_tgl     (disp_tgl),
        .disp_val     (disp_val),
        .disp_mode_in (disp_mode_rep),
        // ★ V47：当前事件的力度（0~127）。不带 0xC3 时恒 127 = 满幅。
        .midi_vel    (midi_vel)        
    );

    //=========================================================================
    // ★★★ V45：节拍器（板载 READY 灯 U12）★★★
    //
    //   用户要求：默认 4/4、速度默认 100 BPM（范围 30~240），通过蓝牙调；
    //   【不参与任何其它模块】，纯粹当外部参考。每拍闪得一样长
    //   （不区分强拍弱拍）。BPM = 0 表示把节拍器关掉。
    //
    //   ★ 为什么放 clk_50m 域：
    //     · led 这根脚本来就是 clk_50m 域驱动的（上面那个 assign）；
    //     · clk_50m_ext（UART/LCD）是工程里余量最紧的域
    //       （V40 起只剩 ~4%），没必要去挤它；
    //     · clk_50m 实测 Fmax 65.15MHz（约束 50），余量最宽。
    //
    //   ★ 参数怎么跨过来：三条参数是【准静态】的（只有用户在 UI 上点一下
    //     才会变一次），所以用工程既有的"翻转沿 + 数据握手"：
    //     metro_tgl 跳变时，metro_bpm/num/den 早已稳定了不知多少个周期，
    //     直接锁存就是安全的。
    //=========================================================================
    reg metro_tgl_s1, metro_tgl_s2, metro_tgl_s3;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) {metro_tgl_s3, metro_tgl_s2, metro_tgl_s1} <= 3'b000;
        else        {metro_tgl_s3, metro_tgl_s2, metro_tgl_s1}
                        <= {metro_tgl_s2, metro_tgl_s1, metro_tgl};
    end

    reg [7:0] mt_bpm, mt_num, mt_den;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            // ★ 板子默认：100 BPM、4/4（分母编码 1 = 4 分音符）、开着
            mt_bpm <= 8'd100; mt_num <= 8'd4; mt_den <= 8'd1;
        end else if (metro_tgl_s2 ^ metro_tgl_s3) begin
            // ★ 只在沿上锁存一次；夹紧已经在 uart_packet_echo 里做过了
            mt_bpm <= metro_bpm; mt_num <= metro_num; mt_den <= metro_den;
        end
    end
    // ★ mt_den 目前只用于回报（0 -> 2 分，1 -> 4 分，2 -> 8 分）。
    //   因为用户选了"每拍闪得一样"，分母不改闪烁行为 —— 见版本说明。

    // ---- 1ms 时间基准：clk_50m = 50MHz -> 50000 拍 = 1ms ----
    reg [15:0] mt_ms;
    wire       mt_ms_tc = (mt_ms == 16'd49999);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        mt_ms <= 16'd0;
        else if (mt_ms_tc) mt_ms <= 16'd0;
        else               mt_ms <= mt_ms + 16'd1;
    end

    // ---- 拍点生成：★ 累加器法（不用除法、也不用乘法）----
    //   一拍 = 60000/BPM 毫秒。每 1ms 往 acc 里加 BPM，加到 >= 60000 就出一拍，
    //   同时把 60000 减掉 —— 余数留在 acc 里，所以【平均】周期是精确的：
    //   60000 除不尽的 BPM（比如 101）会自动在相邻两拍之间一长一短地补偿。
    //   取值范围：acc 恒 < 60000，BPM <= 240 -> acc + BPM < 60240，16 位装得下。
    reg [15:0] mt_acc;
    wire [15:0] mt_sum = mt_acc + {8'd0, mt_bpm};
    wire        mt_beat_evt = mt_ms_tc && (mt_bpm != 8'd0) && (mt_sum >= 16'd60000);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)           mt_acc <= 16'd0;
        else if (mt_beat_evt) mt_acc <= mt_sum - 16'd60000;
        // ★ 关掉节拍器（BPM=0）时把累加器按住归零 ——
        //   不然它会一直涨到 16 位溢出；等用户再把 BPM 打开时，
        //   残留的余数会让第一拍来得莫名其妙地早。
        else if (mt_ms_tc)    mt_acc <= (mt_bpm == 8'd0) ? 16'd0 : mt_sum;
    end

    // ---- 每拍亮灯时长：80ms ----
    //   ★ 想改闪烁长短就改这一个数。240 BPM 时一拍 250ms，
    //     80ms 亮 / 170ms 灭，仍然看得出是在"闪"。
    reg [7:0] mt_flash;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)                              mt_flash <= 8'd0;
        else if (mt_beat_evt)                    mt_flash <= 8'd80;
        else if (mt_ms_tc && (mt_flash != 8'd0)) mt_flash <= mt_flash - 8'd1;
    end
    assign metro_led = (mt_flash != 8'd0);     // 1 = 亮

    // ---- 小节内第几拍（0 起算）----
    //   ★ 因为"每拍闪得一样"，这个计数【不参与闪烁】，只用于 0x5E 状态回报，
    //     让上位机/安卓能显示"第几拍 / 共几拍"。
    reg [3:0] mt_beat_idx;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) mt_beat_idx <= 4'd0;
        else if (mt_beat_evt)
            mt_beat_idx <= ((mt_beat_idx + 4'd1) >= {1'b0, mt_num})
                           ? 4'd0 : (mt_beat_idx + 4'd1);
    end
    assign metro_beat_50m = mt_beat_idx[2:0];   // 分子最大 8 -> 0..7，3 位够

    // ★ 回传给 clk_50m_ext 域的 uart_packet_echo（0x5E 帧第 5 字节）——
    //   过两级同步器。它【只用于回报、不参与任何判断】，所以就算多比特
    //   在跳变中途被采到，顶多让回报的拍号差一拍，没有功能风险。
    reg [2:0] mt_beat_s1, mt_beat_s2;
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) {mt_beat_s2, mt_beat_s1} <= 6'd0;
        else        {mt_beat_s2, mt_beat_s1} <= {mt_beat_s1, metro_beat_50m};
    end
    assign metro_beat_ext = mt_beat_s2;

    //=========================================================================
    // ★★ V41：LED 演奏指引（36 颗，IS31FL3236A，第二路 I2C）★★
    //
    //   · 整个模块跑在 clk_audio（12.5MHz）域 —— 因为它要用的两根线
    //     midi_keys_aud 和 hand_mask 【本来就已经在音频域】：
    //       midi_keys_aud = midi_keys_s2（上面早就做好了从 clk_50m_ext 过来的
    //       两级同步），hand_mask 更是在音频域里生成的。
    //     → 所以这里【一处跨时钟同步都不用新增】。
    //   · 刻意【不】放进 clk_50m_ext（UART/LCD）域：那个域 V40 实测余量只剩
    //     0.014%（50.007MHz / 约束 50MHz），往那儿加逻辑很危险。
    //     音频域 Fmax 25.9MHz（2.07 倍余量），放这里最安全。
    //   · I2C 时钟只要 ≥ 400×SCL：12.5MHz/100kHz = 125，够用
    //     （QUARTER = 12_500_000/(100_000×4) = 31 → 实际 SCL ≈ 100.8kHz）。
    //
    //   灯怎么亮：乐谱这一格要弹的键 → 暗；手按下的键 → 亮（手弹优先）。
    //   图案没变时【一个字节都不发】，总线平时是安静的。
    //=========================================================================
    led_guide #(
        .CLK_HZ     (12_500_000),     // ★ clk_audio，不是 clk_50m_ext
        .SCL_HZ     (100_000),
        .DEV_ADDR   (7'h3C),          // 本板实测可用（独立工程点亮验过）
        // ★★ V43：四档亮度（用户定的档位，换算到 0..255）★★
        //   优先级：90(现在+下一格) > 75(现在) > 30(下一格) > 70(手弹) > 灭
        //   ★ 手弹故意放最后：谱子要求的键保持自己的档位，
        //     不然一按对灯反而从 90% 掉到 70%。
        .DUTY_BOTH  (8'hE6),          // 90%  现在要弹、下一格也要弹
        .DUTY_CUR   (8'hBF),          // 75%  现在要弹
        .DUTY_HAND  (8'hB3),          // 70%  手弹按下的键（不播放时按琴键点亮）
        .DUTY_NEXT  (8'h4D),          // 30%  下一格要弹（提前预告）
        .PWM_FREQ   (8'h01),          // 22kHz，高于音频，避免啸叫
        .PWRUP_MS   (50),
        .SCAN_MS    (5)
    ) u_led_guide (
        .clk        (clk_audio),
        .rst_n      (rst_n),
        .score_keys (midi_guide_aud), // ★ V43：当前格指引（跟谱等待期间也保持显示）
        .next_keys  (midi_keys_next_aud), // ★ V42：下一格 → 30% 提前预告
        .hand_keys  (hand_mask),      // 手弹：物理按键（已经过按键自定义映射）
        .led_scl    (led_scl),
        .led_sda    (led_sda)
    );

    // ---- LCD driver ----
    lcd_master u_lcd (
        .sys_clk    (clk_50m_ext),
        .rst_n      (rst_n),
        .play_start (play_start),
        .play_len   (play_len),
        .score_addr (score_addr),
        .score_data (score_data),
        // ★ V30：电平从"36 位并行"改成扫描口（键号 + 电平 + 握手翻转）
        .key_lvl_idx (scan_idx),
        .key_lvl_val (scan_val),
        .key_lvl_tgl (scan_tgl),
        .timbre_idx ({1'b0, timbre_50m}),

        // ★ 新增
        .wave_mode  (wave_mode),
        .wave_send  (wave_send),
        .wave_tgl   (wave_tgl),
        // ★★ 李萨如：X 轴干声（和 wave_send 同一个翻沿）+ 开关
        .wave_send_x (wave_send_x),
        .liss_on     (liss_on),
        // ★★ 相位式李萨如：Y 轴 = 输出经一阶全通移相 90°（同一个翻沿）+
        //    三屏李萨如的 XY 来源选择（0=左×右 1=干声×输出 2=输出×移相输出）
        .wave_send_py (wave_send_py),
        .liss_src     (liss_src),
        // ★★ V36 左×右档的两轴：输出级的 L / R（同样和 wave_send 同一个翻沿）
        .wave_send_l  (wave_send_l),
        .wave_send_r  (wave_send_r),
        .lcd_rst_tgl  (lcd_rst_tgl),   // ★★ V48.5：0xD1 手动重置 LCD（同域，直接连）

        .state_out  (lcd_state),
        .lcd_sck    (lcd_sck),
        .lcd_mosi   (lcd_mosi),
        .lcd_dc     (lcd_dc),
        .lcd_cs     (lcd_cs),
        .lcd_blk    (lcd_blk)
    );

    // lcd_state is for debug observation only
    wire [3:0] lcd_state_unused = lcd_state;

endmodule


//=============================================================================
// ★★ 以下这个模块原本是单独的 src/ks_string.v，现在【并进 top.v 里】★★
//   原因（踩过）：Gowin IDE 打开工程时会把文件列表锁住，新加进去的 .v 文件
//   不被吃进去，重新加载工程就报
//       ERROR (EX3937) Instantiating unknown module 'ks_string'
//   放进 top.v（本来就在工程文件列表里）就绕过了这一层。
//   ★ 所以别再把它拆回单独文件，也别指望 tools/ks_tables.v —— 那张表只是
//     tools/ks_model.py 生成的片段，已经贴在下面的 NI_TAB/FR_TAB/DMP_TAB 里了。
//=============================================================================
//=============================================================================
// ks_string —— 一条弦：Karplus-Strong 拨弦物理模型（一份代码，36 份并行）
//
//   ★ 这是"吉他音色"的发声核心，不是波表、不是采样：
//       延迟线 + 两点低通 + 每圈损耗 + 噪声起振 —— 真的在算这一根弦。
//
//   为什么能这么省：
//     · 每条弦只要 1 块延迟 RAM（★ V49：512×16 → 1024×16；
//       96 kHz 下最低音 A2 要 888 格，而 1024×16 = 16Kbit 仍只占 1 块 SDPB，
//       所以加深【没有多花一块 BSRAM】）
//     · 乘法只有 1 个，而且还是常数小乘 (r1-r0)×FR（FR 是 6 位常数），
//       综合器多半直接用 LUT 拼掉、不占 DSP
//     · 每圈损耗用移位减 (y - (y>>DP))，完全不占乘法器
//     · 每个采样点（V49：128 个音频时钟）只推进一格，靠 sample_cnt 的 0~7 拍
//       就干完了，剩下 120 拍全在闲着 —— 时序毫无压力
//
//   拍点分配（top.v 给 sample_cnt，一个采样点 = 128 拍（V49））：
//     cnt=0 : 处理起振请求（ptr/xi1 清零、exc = Ni）
//     cnt=1 : 发读地址 ram[ptr]
//     cnt=2 : 收 r0；发下一个读地址 ram[ptr+1]（到 Ni 回绕）
//     cnt=3 : 收 r1
//     cnt=4 : 线性插值 xi = r0 + ((r1-r0)*FR)>>6     ← 唯一的乘法
//     cnt=5 : 推进 LFSR（起振噪声源）
//     cnt=6 : y = (xi+xi1)>>1（环路低通）；绕圈损耗；写回 RAM + 输出
//     cnt=7 : 清写使能
//
//   音准：环路总延迟 = Ni + frac/64 + 0.5（个采样），等于 fs/f 就准。
//         Ni / FR / DP 三个常数由 tools/ks_model.py 算好（36 个键，
//         音高偏差 < 1.3 音分），在本模块里按 KEY 常数化 —— 综合后每个
//         实例只出自己那一份逻辑，运行时没有任何查表/选择器。
//
//   起振：本键按下时，往延迟线里灌【一整圈（Ni 个点）】的噪声；
//         噪声幅度跟着力度 vel[7:4] 走 —— 轻拨闷、重拨亮。
//   输出：smp 是 16 位有符号，和 sine_rom 的 rom_dout 同量级，
//         所以 top.v 里那 36 个"×包络"乘法器原封不动就能用。
//=============================================================================
`timescale 1ns / 1ps

module ks_string #(
    parameter [5:0] KEY = 6'd0
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [15:0]        sample_cnt,   // 音频域采样计数 0..255
    input  wire               run,          // 吉他音色时才推进（省功耗）
    input  wire               pluck,        // 本键刚按下：起振脉冲（1 拍宽）
    input  wire [7:0]         vel,          // 力度 0~255（用高 4 位 = 16 档）
    output reg  signed [15:0] smp           // 弦输出
);

    //-------------------------------------------------------------------------
    // 每键常数（由 tools/ks_model.py 生成，见 tools/ks_tables.v）
    //   NI : 延迟线回绕长度（整数格）
    //   FR : 小数插值系数 /64（把音调修到 1 音分以内）
    //   DP : 每绕一圈的损耗位移，gain = 1 - 2^-DP（定衰减时间）
    //-------------------------------------------------------------------------
    function [9:0] NI_TAB(input [5:0] k);
        case (k)
            6'd0 : NI_TAB = 10'd835 ;  6'd1 : NI_TAB = 10'd745 ;  6'd2 : NI_TAB = 10'd626 ;  6'd3 : NI_TAB = 10'd558 ;  
            6'd4 : NI_TAB = 10'd498 ;  6'd5 : NI_TAB = 10'd888 ;  6'd6 : NI_TAB = 10'd794 ;  6'd7 : NI_TAB = 10'd703 ;  
            6'd8 : NI_TAB = 10'd664 ;  6'd9 : NI_TAB = 10'd592 ;  6'd10: NI_TAB = 10'd528 ;  6'd11: NI_TAB = 10'd469 ;  
            6'd12: NI_TAB = 10'd419 ;  6'd13: NI_TAB = 10'd373 ;  6'd14: NI_TAB = 10'd314 ;  6'd15: NI_TAB = 10'd280 ;  
            6'd16: NI_TAB = 10'd249 ;  6'd17: NI_TAB = 10'd444 ;  6'd18: NI_TAB = 10'd395 ;  6'd19: NI_TAB = 10'd353 ;  
            6'd20: NI_TAB = 10'd332 ;  6'd21: NI_TAB = 10'd296 ;  6'd22: NI_TAB = 10'd264 ;  6'd23: NI_TAB = 10'd235 ;  
            6'd24: NI_TAB = 10'd210 ;  6'd25: NI_TAB = 10'd187 ;  6'd26: NI_TAB = 10'd157 ;  6'd27: NI_TAB = 10'd140 ;  
            6'd28: NI_TAB = 10'd125 ;  6'd29: NI_TAB = 10'd222 ;  6'd30: NI_TAB = 10'd198 ;  6'd31: NI_TAB = 10'd176 ;  
            6'd32: NI_TAB = 10'd166 ;  6'd33: NI_TAB = 10'd148 ;  6'd34: NI_TAB = 10'd132 ;  6'd35: NI_TAB = 10'd118 ;  
            default: NI_TAB = 10'd444;
        endcase
    endfunction

    function [5:0] FR_TAB(input [5:0] k);
        case (k)
            6'd0 : FR_TAB = 6'd53;  6'd1 : FR_TAB = 6'd2 ;  6'd2 : FR_TAB = 6'd32;  6'd3 : FR_TAB = 6'd30;  6'd4 : FR_TAB = 6'd16;  6'd5 : FR_TAB = 6'd46;  
            6'd6 : FR_TAB = 6'd35;  6'd7 : FR_TAB = 6'd60;  6'd8 : FR_TAB = 6'd11;  6'd9 : FR_TAB = 6'd41;  6'd10: FR_TAB = 6'd40;  6'd11: FR_TAB = 6'd0 ;  
            6'd12: FR_TAB = 6'd24;  6'd13: FR_TAB = 6'd49;  6'd14: FR_TAB = 6'd32;  6'd15: FR_TAB = 6'd44;  6'd16: FR_TAB = 6'd24;  6'd17: FR_TAB = 6'd39;  
            6'd18: FR_TAB = 6'd8 ;  6'd19: FR_TAB = 6'd61;  6'd20: FR_TAB = 6'd21;  6'd21: FR_TAB = 6'd37;  6'd22: FR_TAB = 6'd36;  6'd23: FR_TAB = 6'd12;  
            6'd24: FR_TAB = 6'd60;  6'd25: FR_TAB = 6'd50;  6'd26: FR_TAB = 6'd32;  6'd27: FR_TAB = 6'd38;  6'd28: FR_TAB = 6'd60;  6'd29: FR_TAB = 6'd35;  
            6'd30: FR_TAB = 6'd52;  6'd31: FR_TAB = 6'd14;  6'd32: FR_TAB = 6'd9 ;  6'd33: FR_TAB = 6'd20;  6'd34: FR_TAB = 6'd34;  6'd35: FR_TAB = 6'd63;  
            default: FR_TAB = 6'd0;
        endcase
    endfunction

    function [3:0] DMP_TAB(input [5:0] k);
        case (k)
            6'd0 : DMP_TAB = 4'd6 ;  6'd1 : DMP_TAB = 4'd6 ;  6'd2 : DMP_TAB = 4'd6 ;  6'd3 : DMP_TAB = 4'd6 ;  6'd4 : DMP_TAB = 4'd6 ;  6'd5 : DMP_TAB = 4'd6 ;  6'd6 : DMP_TAB = 4'd6 ;  6'd7 : DMP_TAB = 4'd6 ;  6'd8 : DMP_TAB = 4'd6 ;  
            6'd9 : DMP_TAB = 4'd6 ;  6'd10: DMP_TAB = 4'd6 ;  6'd11: DMP_TAB = 4'd6 ;  6'd12: DMP_TAB = 4'd6 ;  6'd13: DMP_TAB = 4'd6 ;  6'd14: DMP_TAB = 4'd6 ;  6'd15: DMP_TAB = 4'd6 ;  6'd16: DMP_TAB = 4'd6 ;  6'd17: DMP_TAB = 4'd6 ;  
            6'd18: DMP_TAB = 4'd6 ;  6'd19: DMP_TAB = 4'd6 ;  6'd20: DMP_TAB = 4'd6 ;  6'd21: DMP_TAB = 4'd6 ;  6'd22: DMP_TAB = 4'd6 ;  6'd23: DMP_TAB = 4'd7 ;  6'd24: DMP_TAB = 4'd7 ;  6'd25: DMP_TAB = 4'd7 ;  6'd26: DMP_TAB = 4'd7 ;  
            6'd27: DMP_TAB = 4'd7 ;  6'd28: DMP_TAB = 4'd7 ;  6'd29: DMP_TAB = 4'd7 ;  6'd30: DMP_TAB = 4'd7 ;  6'd31: DMP_TAB = 4'd7 ;  6'd32: DMP_TAB = 4'd7 ;  6'd33: DMP_TAB = 4'd7 ;  6'd34: DMP_TAB = 4'd7 ;  6'd35: DMP_TAB = 4'd7 ;  
            default: DMP_TAB = 4'd6;
        endcase
    endfunction

    localparam [9:0] NI = NI_TAB(KEY);
    localparam [5:0] FR = FR_TAB(KEY);
    localparam [3:0] DP = DMP_TAB(KEY);

    //-------------------------------------------------------------------------
    // 延迟线（每条弦一块 BSRAM）
    //-------------------------------------------------------------------------
    (* syn_ramstyle = "block_ram" *) reg signed [15:0] ram [0:1023];

    reg  [9:0]  ptr;          // 当前读写位置（V49：9 -> 10 位，1024 格）
    reg  signed [16:0] xi1;   // 上一次的插值结果（环路低通用）
    reg  [9:0]  exc;          // 还要灌几个噪声点（起振）
    reg         pluck_pend;   // 起振请求（按下可能不在 cnt=0 那一拍）

    reg  [9:0]  raddr;
    reg         we;
    reg  [9:0]  waddr;
    reg  signed [15:0] wdata;
    reg  signed [15:0] ram_q;

    reg  signed [16:0] r0, r1;
    reg  signed [16:0] xi;

    reg  [15:0] lfsr;

    // 下一个地址（到 Ni 回绕）
    wire [9:0] p1 = (ptr >= (NI - 10'd1)) ? 10'd0 : (ptr + 10'd1);

    // ---- 延迟线：单写单读 + 一拍同步 ----
    always @(posedge clk) begin
        if (we) ram[waddr] <= wdata;
        ram_q <= ram[raddr];
    end

    // ---- 组合运算（都在 cnt=6 那一拍落地）----
    wire signed [16:0] dif    = r1 - r0;

    // dif × FR ：FR 是 6 位【常数】，所以用移位相加实现，不占 DSP。
    //   一开始写成 dif * FR，综合器 36 份各要一个乘法器，
    //   实测 DSP 从 53.5 涨到 104.5/118（89%），太满；
    //   常数小乘改移位相加后只用几级 ALU（LUT 才 47%，富余）。
    reg signed [22:0] dif_f;
    integer mi;
    always @(*) begin
        dif_f = 23'sd0;
        for (mi = 0; mi < 6; mi = mi + 1)
            if (FR[mi]) dif_f = dif_f + ({{6{dif[16]}}, dif} <<< mi);
    end

    // 插值：xi = r0 + ((r1-r0)*FR)>>6 —— 注意 <<< 的优先级高于 >>>
    wire signed [22:0] xi_sum = {{6{r0[16]}}, r0} + (dif_f >>> 6);
    wire signed [16:0] xi_c   = xi_sum[16:0];
    wire signed [16:0] y_c    = (xi + xi1) >>> 1;
    wire signed [16:0] y2_c   = (ptr == (NI - 10'd1)) ? (y_c - (y_c >>> DP)) : y_c;

    // 起振噪声：8 位有符号 × 力度档（高 4 位），再左移定幅度
    wire signed [15:0] nz     = $signed({1'b0, lfsr[15:8]}) - 16'sd128;
    //  ★★ 用【移位相加】代替乘法器（省资源，别改回 *）★★
    //    vel[7:4] 只有 4 位，就是 4 个移位相加：
    //        nz*vel = nz*v0 + 2*nz*v1 + 4*nz*v2 + 8*nz*v3
    //    实测原来 36 条弦各吃一个 MULT12X12（DSP 里那 +37 个就是它，占 61%），
    //    换成移位相加后只用几十个 LUT/条 —— 腾出来的 DSP 留给以后加乐器
    //    （古筝/鼓的模态谐振器都要乘法器）。精度完全一样（vel 是整数）。
    reg signed [15:0] nz_mul;
    integer vi;
    always @(*) begin
        nz_mul = 16'sd0;
        for (vi = 0; vi < 4; vi = vi + 1)
            if (vel[4 + vi]) nz_mul = nz_mul + (nz <<< vi);
    end
    wire signed [15:0] nz_exc = nz_mul <<< 4;

    // 写回内容：起振期灌噪声，之后灌环路值（带饱和，绝不回绕）
    wire signed [16:0] w_pre  = (exc != 10'd0) ? $signed({{1{nz_exc[15]}}, nz_exc}) : y2_c;
    wire signed [16:0] w_lim  = (w_pre > 17'sd32767)  ? 17'sd32767  :
                                (w_pre < -17'sd32768) ? -17'sd32768 : w_pre;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ptr        <= 10'd0;
            xi1        <= 17'sd0;
            exc        <= 10'd0;
            pluck_pend <= 1'b0;
            raddr      <= 10'd0;
            we         <= 1'b0;
            waddr      <= 10'd0;
            wdata      <= 16'sd0;
            r0         <= 17'sd0;
            r1         <= 17'sd0;
            xi         <= 17'sd0;
            lfsr       <= 16'hACE1 ^ {10'd0, KEY};
            smp        <= 16'sd0;
        end else begin
            if (pluck) pluck_pend <= 1'b1;

            if (!run) begin
                // 非吉他音色：整条弦冻住（省功耗），写使能一定清掉
                we <= 1'b0;
            end else begin
                case (sample_cnt[7:0])
                    8'd0: begin
                        we <= 1'b0;
                        if (pluck_pend) begin
                            // 起振：指针归零、低通历史清零、准备灌一整圈噪声
                            ptr        <= 10'd0;
                            xi1        <= 17'sd0;
                            exc        <= NI;
                            // 同一拍又来了新按下就继续接着灌，绝不吞掉音符
                            pluck_pend <= pluck;
                        end
                    end
                    8'd1: raddr <= ptr;
                    8'd2: begin r0 <= {ram_q[15], ram_q}; raddr <= p1; end
                    8'd3: r1 <= {ram_q[15], ram_q};
                    8'd4: xi <= xi_c;                       // 线性插值（唯一乘法）
                    8'd5: lfsr <= {lfsr[0] ^ lfsr[2] ^ lfsr[3] ^ lfsr[5], lfsr[15:1]};
                    8'd6: begin
                        waddr <= ptr;
                        wdata <= w_lim[15:0];
                        we    <= 1'b1;
                        smp   <= y2_c[15:0];
                        xi1   <= xi;
                        ptr   <= p1;
                        if (exc != 10'd0) exc <= exc - 10'd1;
                    end
                    8'd7: we <= 1'b0;
                    default: ;
                endcase
            end
        end
    end

endmodule
