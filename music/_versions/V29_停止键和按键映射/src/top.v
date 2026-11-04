`timescale 1ns / 1ps
//=============================================================================
// top.v  (Gowin GW5A-25ï¼šå¤šç‰‡ TTP229 ç”µå­ç´ + ADS1115 å››é€šé“é‡‡é›†)
module top (
    input  wire        clk_50m,
    input  wire        rst_n,

    // ---- TTP229ï¼šæ¯ç‰‡ä¸€ç»„ scl/sdo ----
    output wire        scl,        // ç´é”®1
    inout  wire        sdo,
    output wire        scl2,       // ç´é”®2
    inout  wire        sdo2,
    // â˜…PAD3-1  ç¬¬ 3 ç‰‡é”®ç›˜ï¼ˆä½Žå…«åº¦ï¼‰ï¼šConsole 60K ä¸Š SCL = A9ï¼ŒSDO = A11
    output wire        scl3,       // ç´é”®3
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

    // ---- ADS1115 (I2C) ----
    //   æ‘‡æ† X/Y å°±æŽ¥åœ¨å®ƒçš„ A0/A1 ä¸Š
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    // ---- Bluetooth HC-05/06 (UART) ----
    input  wire        uart_rxd,     // BT TXD
    output wire        uart_txd,     // BT RXD

    // ---- SPI LCD (ST7789 320x240) ----
    output wire        lcd_sck,
    output wire        lcd_mosi,
    output wire        lcd_dc,
    output wire        lcd_cs,
    output wire        lcd_blk
);

    //==================================================================
    // â˜…PAD3-2  é”®æ•°ï¼š3 ç‰‡ = 36ï¼ˆç¬¬ 3 ç‰‡å·²åŠ å…¥ï¼Œä½Žå…«åº¦ï¼‰
    //==================================================================
    localparam NUM_KEYS = 36;

    //==================================================================
    // 摇杆模块（ADS1115 双通道：X 轴 / Y 轴）
    //
    //   A0 / AIN0 â†’ X è½´ â†’ ç§»è°ƒï¼šæ•´å°ç´å‡é™åŠéŸ³ï¼Œæ¯è¾¹æœ€å¤š JS_STEPS ä¸ª
    //   A1 / AIN1 â†’ Y è½´ â†’ æ€»éŸ³é‡ï¼šæ‹‰åˆ°åº•é™éŸ³ï¼ŒæŽ¨åˆ°å¤´ = åŽŸæ¥çš„æ­£å¸¸å“åº¦
    //   â˜… åŽŸåŽ‹åŠ›ä¼ æ„Ÿå™¨ï¼ˆåŠ›åº¦ï¼‰å·²æ•´ä½“ç§»é™¤ï¼Œè°æ³¢æ¯”ä¾‹æ”¹å›žç”±éŸ³è‰²æ¡£ä½å†³å®šã€‚
    //==================================================================
    localparam [15:0] JS_CENTER = 16'd13200;   // æ‘‡æ†å±…ä¸­æ—¶çš„è¯»æ•°
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
    localparam [15:0] JS_TRAVEL = 16'd12500;   // ä»Žä¸­ä½æŽ¨åˆ°æžç«¯çš„è¡Œç¨‹
    localparam integer JS_STEPS = 12;          // æ¯è¾¹æœ€å¤šç§»å‡ ä¸ªåŠéŸ³

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
        .reset   (1'b0)            // 60k ç‰ˆ IP å¤šäº†è¿™ä¸ªç«¯å£ï¼ŒæŽ¥ 0ï¼ˆ= åŽŸå·¥ç¨‹ I_RST æŽ¥åœ°ï¼‰
    );
    //==================================================================
    // ★ TTP229 当按钮：第 4 片（16 键模式）
    //   键1 -> 滑音开关（代替 AA13）
    //   键2 -> 延音开关（代替 AB13）
    //   键3 -> 波形 / 频谱显示切换
    //   键4 -> 音色切换
    //   键5 -> 颤音（Tremolo）开关
    //   键6 -> 延迟（Delay / Echo）开关
    //   键7~16 暂时不用
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

    // ★ 只认 TTP229 键2
    reg sustain_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)            sustain_on <= 1'b0;
        else if (btn_sustain)  sustain_on <= ~sustain_on;
    end

    // ★ 只认 TTP229 键3
    wire timbre_cycle = btn_timbre;

    //   板载按键是【瞬时按键】，按一下翻转一次，不再需要长按 2 秒。
    //   ★ 极性：AA13 / AB13 是【板载按键】：bank9 / 1.5V / 内部上拉 / 按下 = 低电平。
    //   led_glide 直接指示 glide_en（亮 = 滑音开）。
    // ★ 只认 TTP229 键1
    wire glide_toggle = btn_glide;

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

    reg wave_mode;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        wave_mode <= 1'b0;    // 上电默认频谱
        else if (btn_wave) wave_mode <= ~wave_mode;
    end

    //==================================================================
    // ★★ 颤音（Tremolo）效果器开关 ★★
    //
    //   ★ 当前按钮分配（16 键触摸模块）—— 与下面代码逐位对齐：
    //       按钮1(bit0)=滑音    按钮2(bit1)=sustain
    //       按钮3(bit2)=波形    按钮4(bit3)=音色
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
        if (!rst_n)        trem_on <= 1'b0;          // 上电默认【关】
        else if (btn_trem) trem_on <= ~trem_on;      // 按一下开，再按一下关
    end

    reg dly_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)       dly_on <= 1'b0;            // 上电默认【关】
        else if (btn_dly) dly_on <= ~dly_on;         // 按一下开，再按一下关
    end

    //==================================================================
    // ★★ 按钮8 = 混响（Reverb）  按钮9 = 合唱（Chorus）★★
    //   两个都是【纯新增】：关掉时整条音频链路逐位等于现在，其它功能一律不动。
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
    wire btn_flute = btn_new[6];             // 按钮 7 -> 笛子音色

    reg flute_on;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)         flute_on <= 1'b0;        // 上电默认【现有音色】
        else if (btn_flute) flute_on <= ~flute_on;   // 按一下换笛子，再按换回来
    end
    //------------------------------------------------------------------
    // LED æŒ‡ç¤ºå½“å‰éŸ³è‰²ï¼šæ…¢é—ª / å¿«é—ª / åŒé—ª / å¸¸äº®
    //------------------------------------------------------------------
    reg [23:0] led_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) led_cnt <= 24'd0;
        else        led_cnt <= led_cnt + 1'b1;
    end

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
    localparam LED_ACTIVE_LOW = 1'b1;      // Console 60K = 1ï¼ˆä½Žç”µå¹³ç‚¹äº®ï¼‰

    assign led = LED_ACTIVE_LOW ? ~led_out : led_out;

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
    // éŸ³è‰²åˆ‡æ¢è„‰å†²ï¼ˆç¿»è½¬æ²¿ï¼‰ï¼Œè·¨æ—¶é’Ÿåˆ°éŸ³é¢‘åŸŸ
    //------------------------------------------------------------------
    reg timbre_toggle_50m;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) timbre_toggle_50m <= 1'b0;
        else if (timbre_cycle) timbre_toggle_50m <= ~timbre_toggle_50m;
    end

    //==================================================================
    // 50MHz åŸŸï¼šTTP229 è¯»å–ï¼ˆæ¯ç‰‡ä¸€ä¸ªå®žä¾‹ï¼Œå¹¶è¡Œç‹¬ç«‹è¿è¡Œï¼‰
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

    // â˜… Console 60K ç‰ˆï¼šé”®ç›˜2 å·²ä¿®å¥½ï¼Œä¸‰ç‰‡å…¨éƒ¨å¯ç”¨ã€‚
    //   ä¸‡ä¸€å“ªå¤©æŸä¸€ç‰‡åˆåæŽ‰ï¼ŒæŠŠ KB2_DISABLED æ”¹å›ž 1'b1 å°±èƒ½æ•´æ®µé™é»˜éš”ç¦»ï¼š
    //     Â· scl2 æ’è¾“å‡º 0 â€”â€” TTP229 çš„ SCL ç©ºé—²æœ¬æ¥å°±æ˜¯ä½Žï¼Œä¸ä¼šé¡¶ç‰›
    //     Â· sdo2 ä¸å†ç”±è¯»å–å™¨é©±åŠ¨ï¼Œåªå½“ä¸€ä¸ªè¾“å…¥ç”¨
    //     Â· kp2 æ’ 0 â€”â€” è¯¥ç‰‡æ°¸è¿œ"æ²¡æœ‰é”®æŒ‰ä¸‹"ï¼Œä¸ä¼šå†å‡ºçŽ° 16 ä¸ªå¹½çµéŸ³
    localparam KB2_DISABLED = 1'b0;

    generate
        if (KB2_DISABLED) begin : g_kb2_off
            assign scl2 = 1'b0;
            assign kp2  = 12'd0;
            assign rdy2 = 1'b0;
            wire sdo2_dummy = sdo2;      // ä¿ç•™è¾“å…¥ç¼“å†²ï¼ˆå¼•è„šæŒ‰ .cst çš„ PULL_MODE è®¾ç½®ï¼‰
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

    // â˜…PAD3-3  ç¬¬ 3 ç‰‡é”®ç›˜ï¼ˆä½Žå…«åº¦ï¼‰ï¼šSCL = L11ï¼ŒSDO = E10
    ttp229_reader u_kp3 (
        .clk         (clk_50m),
        .rst_n       (rst_n),
        .scl         (scl3),
        .sdo         (sdo3),
        .key_stable  (kp3),
        .rdy_toggle  (rdy3)
    );

    //==================================================================
    // æŒ‰é”®æŽ©ç ï¼šä½Žä½ = ç´é”®1ï¼Œä¸­é—´ = ç´é”®2ï¼Œé«˜ä½ = ç´é”®3
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
    // è·¨æ—¶é’Ÿï¼šæŒ‰é”®äº‹ä»¶ 50MHz -> 12.5MHz
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
    // è·¨æ—¶é’Ÿï¼šéŸ³è‰²ç¿»è½¬æ²¿ 50MHz -> 12.5MHz
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

    // å»¶éŸ³å¼€å…³è·¨åˆ°éŸ³é¢‘åŸŸï¼ˆæ…¢ä¿¡å·ï¼Œ2 çº§åŒæ­¥è¶³å¤Ÿï¼‰
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
    // æ‘‡æ†ï¼šADS1115 A0 = X è½´ï¼ˆç§»è°ƒï¼‰ï¼ŒA1 = Y è½´ï¼ˆæ€»éŸ³é‡ï¼‰
    //
    //   â˜… åŽŸåŽ‹åŠ›ä¼ æ„Ÿå™¨ï¼ˆåŠ›åº¦ + å³°å€¼é”å­˜ï¼‰å·²æ•´ä½“ç§»é™¤ã€‚
    //   ADC æ˜¯å•ç«¯è¾“å…¥ã€PGA=Â±4.096Vï¼Œ32767 å¯¹åº” 4.096Vï¼›
    //   æ‘‡æ† 3.3V ä¾›ç”µ â†’ æ»¡å¹…è¯»æ•°çº¦ 26400ï¼Œä¸ä¼šé¥±å’Œã€‚
    //   A0/A1 çš„åŽŸå§‹è¯»æ•°åŽŸæ ·ä¸ŠæŠ¥åˆ° VOFA+ ç¬¬ 1ã€2 è·¯ï¼Œç¬¬ä¸€æ¬¡ä¸Šæ¿å¯ä»¥
    //   ç›´æŽ¥çœ‹æ³¢å½¢æ¥æ ¡å‡† JS_CENTER / JS_TRAVELã€‚
    //==================================================================
    wire [15:0] ain0_data, ain1_data, ain2_data, ain3_data;

    // â˜… æš‚æ—¶æ²¡æŽ¥æ‘‡æ†æ—¶ï¼ŒæŠŠè¿™ä¸ªæ”¹æˆ 1'b1ï¼š
    //   å¿½ç•¥æ‘‡æ†è¾“å…¥ï¼Œå›ºå®š"åŽŸè°ƒ + æ»¡éŸ³é‡"ï¼Œå…ˆéªŒè¯ç´æœ¬èº«æ­£å¸¸ã€‚
    //   ï¼ˆå› ä¸º Y è½´è¯»æ•°å¦‚æžœé£˜åˆ° 0ï¼ŒéŸ³é‡å°±æ˜¯ 0ï¼Œä¼šè¡¨çŽ°ä¸ºå®Œå…¨æ²¡å£°ï¼‰
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
    // 1) 50MHz åŸŸï¼šå¯¹ ADS1115 åŽŸå§‹è¯»æ•°åšã€å¤šç‚¹å¹³å‡ã€‘é™å™ª
    //
    //   â˜… ä¸ºä»€ä¹ˆè¦æ¢æŽ‰åŽŸæ¥çš„ä¸€é˜¶ IIRï¼š
    //     ADS1115 æ˜¯ 860SPS å››é€šé“è½®è¯¢ï¼ŒåŒä¸€ä¸ªé€šé“å¤§çº¦æ¯ 4.65ms æ‰æ›´æ–°
    //     ä¸€æ¬¡ã€‚è€ŒåŽŸæ¥çš„ IIR æ¯ 20.5us å°±èµ°ä¸€æ­¥ã€æ—¶é—´å¸¸æ•°åªæœ‰ 0.66ms â€”â€”
    //     å®ƒåœ¨ä¸¤æ¬¡ ADC æ›´æ–°ä¹‹é—´æ—©å°±èµ°åˆ°ä½äº†ï¼Œç­‰äºŽåªæŠŠå°é˜¶æŠ¹åœ†äº†ä¸€ç‚¹ï¼Œ
    //     å¯¹ ADC è‡ªèº«çš„å™ªå£°å‡ ä¹Žæ²¡æœ‰ä»»ä½•æŠ‘åˆ¶ã€‚
    //     å¹³å‡çª—å£å¿…é¡»è·¨è¿‡è‡³å°‘ä¸¤ä¸ª ADC å‘¨æœŸæ‰çœŸæ­£èµ·ä½œç”¨ã€‚
    //------------------------------------------------------------------
    // ★ V17：16 点 -> 4 点。
    //   ADS1115 是 128SPS（见 ads1115_ctrl.v 的 CFG_L=0x83），
    //   一个通道约 20ms 才出一个【新】读数；而 16 点窗口只有 10.5ms，
    //   等于把同一个样本重复平均，噪声没压下去，只白白加了 10.5ms 延迟。
    //   128SPS 本身就足够安静，改成 4 点（2.6ms）当毛刺滤波即可。
    localparam JS_AVG_LOG2 = 2;                       // 2^2 = 4 点
    localparam JS_AVG_N    = (1 << JS_AVG_LOG2);      // 4
    localparam [15:0] JS_TAP_DIV = 16'd32768;         // æ¯ 655us é‡‡ä¸€ç‚¹

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

    // æ¯ç§’ 1024 æ‹ï¼ˆâ‰ˆ20.5usï¼‰ä¸€æ¬¡çš„å¿«èŠ‚æ‹ï¼Œç»™ä¸‹é¢çš„æ¢æ¡£è¿Ÿæ»žç”¨
    reg [9:0] js_tick;
    wire      js_upd = (js_tick == 10'd1023);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) js_tick <= 10'd0;
        else        js_tick <= js_upd ? 10'd0 : (js_tick + 1'b1);
    end

    //------------------------------------------------------------------
    // 2) X è½´ â†’ ç§»è°ƒæ¡£ä½ 0~24ï¼ˆ12 = åŽŸè°ƒï¼‰
    //    |x - ä¸­å¿ƒ| å…ˆæ‰£æŽ‰æ­»åŒºï¼Œå†æŒ‰è¡Œç¨‹çº¿æ€§æ˜ å°„æˆ 0~JS_STEPS ä¸ªåŠéŸ³ã€‚
    //    æ­»åŒºå¿…é¡»ç•™ï¼šæ‘‡æ†é™æ­¢æ—¶è¯»æ•°ä¼šåœ¨å‡ ç™¾ä¸ª count å†…æŠ–ï¼Œ
    //    ä¸æ‰£æ­»åŒºä¼šä¸€ç›´åœ¨ç›¸é‚»åŠéŸ³ä¹‹é—´è·³ã€‚
    //------------------------------------------------------------------
    wire signed [16:0] js_xd = $signed({1'b0, js_x}) - $signed({1'b0, JS_CENTER});
    wire        [15:0] js_xa = js_xd[16] ? (~js_xd[15:0] + 1'b1) : js_xd[15:0];  // |åç§»|
    wire        [15:0] js_xe = (js_xa > JS_DEAD) ? (js_xa - JS_DEAD) : 16'd0;    // æ‰£æ­»åŒº

    wire [31:0] js_xn32 = ((js_xe * JS_XMUL) + 32'd32768) >> 16;                 // å››èˆäº”å…¥
    wire [4:0]  js_xn   = (js_xn32 > JS_STEPS) ? JS_STEPS[4:0] : js_xn32[4:0];

    //------------------------------------------------------------------
    // â˜… è¿Ÿæ»žï¼šå€™é€‰æ¡£ä½å¿…é¡»è¿žç»­ 8 æ¬¡éƒ½ä¸åŒæ‰çœŸæ­£æ¢æ¡£ï¼ˆ8Ã—20.5us â‰ˆ 164usï¼‰ã€‚
    //
    //   æ­»åŒºåªèƒ½æŒ¡ä½"ä¸­å¿ƒé™„è¿‘"çš„æŠ–åŠ¨ï¼ŒæŒ¡ä¸ä½ã€æ ¼å­è¾¹ç•Œã€‘ä¸Šçš„æŠ–åŠ¨ï¼š
    //   æ‘‡æ†é™æ­¢æ—¶è¯»æ•°æ­£å¥½åŽ‹åœ¨ä¸¤ä¸ªåŠéŸ³çš„ä¸´ç•Œç‚¹ä¸Šï¼Œæ¡£ä½å°±ä¼šä¸¤æ ¼ä¹‹é—´
    //   æ¥å›žè·³ï¼Œæ¯è·³ä¸€æ¬¡ inc[] å°±è¦é‡ç®— â†’ dcur è·Ÿç€æ¥å›žæ»‘ â†’
    //   å¬æ„Ÿå°±æ˜¯ä¸€å±‚ç²—ç³™çš„æ²™æ²™å£°ã€‚è¿™æ®µè¿Ÿæ»žä¸“é—¨æ²»å®ƒã€‚
    //------------------------------------------------------------------
    reg [4:0] trans_cand, trans_idx;    // 0~24ï¼Œ12 = åŽŸè°ƒ
    reg [3:0] trans_hcnt;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            trans_cand <= 5'd12;
            trans_idx  <= 5'd12;
            trans_hcnt <= 4'd0;
        end else if (js_upd) begin
            // 1) ç®—å‡ºå€™é€‰æ¡£ä½
            if (js_xa <= JS_DEAD)   trans_cand <= 5'd12;
            else if (js_xd[16])     trans_cand <= 5'd12 - js_xn;   // å¾€ä½Žç§»
            else                    trans_cand <= 5'd12 + js_xn;   // å¾€é«˜ç§»

            // 2) è¿žç»­ç¨³å®šæ‰æäº¤
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
    // 3) Y è½´ â†’ æ€»éŸ³é‡ 0~255ï¼ˆ0 = é™éŸ³ï¼Œ255 = æ”¹é€ å‰çš„æ­£å¸¸å“åº¦ï¼‰
    //    â˜… 255 æ¡£å®žé™…æ˜¯ Ã—255/256ï¼Œæ‰€ä»¥æŽ¨åˆ°å¤´ä¸ä¼šæ¯”ä»¥å‰æ›´æ—©å‰Šé¡¶ã€‚
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
    wire [5:0]  tk      = js_za[15:10];
    wire [7:0]  tk_mul  = {2'b0, tk} * 8'd21;
    wire [7:0]  js_tdep = (tk <= 6'd8) ? (8'd200 - tk_mul) : 8'd10;


    //------------------------------------------------------------------
    // 4) è·¨åˆ° 12.5MHz éŸ³é¢‘åŸŸ
    //
    //   â˜…â˜… å¤šä½æ€»çº¿ã€å¿…é¡»ã€‘ç”¨ç¿»è½¬æ¡æ‰‹ï¼Œä¸èƒ½ç”¨æ™®é€šä¸¤çº§åŒæ­¥å™¨ï¼
    //
    //     æ™®é€šä¸¤çº§åŒæ­¥å™¨åªä¿è¯"æ¯ä¸€ä½"ä¸äºšç¨³æ€ï¼Œä¸ä¿è¯å„ä½åœ¨åŒä¸€æ¬¡
    //     é‡‡æ ·é‡Œè¢«åˆ¤æˆåŒä¸€ä¸ªæ—¶åˆ»çš„å€¼ã€‚js_vol ä»Ž 127 å˜ 128 æ—¶ 8 ä½åŒæ—¶
    //     åœ¨ç¿»ï¼ŒéŸ³é¢‘åŸŸå¯èƒ½é‡‡åˆ°"é«˜ä½æ˜¯æ–°çš„ã€ä½Žä½æ˜¯æ—§çš„"è¿™ç§ä¸åˆæ³•çš„
    //     ç»„åˆï¼ˆæ¯”å¦‚ç›´æŽ¥é‡‡æˆ 255ï¼‰â†’ è¾“å‡ºçž¬é—´è·³ä¸€ä¸ªå¤§å°é˜¶ â†’
    //     å¬æ„Ÿå°±æ˜¯"åŠˆé‡Œå•ªå•¦"çš„ç”µæµå£°/çˆ†éŸ³ã€‚
    //
    //     è€Œä¸”è¶Šæ˜¯é¢‘ç¹æ”¹è¶Šå®¹æ˜“æ’žä¸Š â€”â€” è°ƒéŸ³é‡æ—¶ js_vol æŒç»­åœ¨å˜ï¼Œ
    //     å‘½ä¸­æ¦‚çŽ‡æš´æ¶¨ï¼Œæ­£å¥½å¯¹åº”"è°ƒéŸ³é‡æ—¶å°¤ä¸ºé¢‘ç¹"ã€‚
    //     trans_idx è™½ç„¶åªæœ‰ 5 ä½ï¼ŒåŒæ ·ä¼šä¸­æ‹›ï¼Œé‡‡é”™å°±çž¬é—´è·³è°ƒã€‚
    //
    //   åšæ³•ï¼šæ•°æ®å…ˆåœ¨ 50MHz åŸŸå¯„å­˜ç¨³å®šï¼Œå†ç”¨ä¸€ä¸ªç¿»è½¬ä½é€šçŸ¥éŸ³é¢‘åŸŸ
    //   "å¯ä»¥å–äº†"ï¼›éŸ³é¢‘åŸŸæ£€å‡ºç¿»è½¬æ²¿æ‰åŽ»é‡‡ï¼Œæ­¤æ—¶æ•°æ®å·²ç¨³å®šå¾ˆä¹…ã€‚
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
                     (tdep_send != js_tdep)) begin
            tr_send   <= trans_idx;
            vel_send  <= js_vol;
            tdep_send <= js_tdep;
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
    // 12.5MHz åŸŸï¼šNUM_KEYS è·¯ DDS + æ··éŸ³ + I2S
    //==================================================================
    localparam CLK_FREQ_HZ = 12_500_000;

    //=========================================================================
    // â˜…â˜… I2S å‚æ•°ï¼ˆå”¯ä¸€å¼€å…³ï¼šI2S_SPECï¼‰â˜…â˜…
    //
    //  ä¸ºä»€ä¹ˆè¦æœ‰è¿™ä¸ªå¼€å…³ â€”â€” PCM5102A æ•°æ®æ‰‹å†Œ ZHCS323B çš„ç¡¬è¦æ±‚ï¼š
    //     "System clock frequency = 64, 128, 192, 256, 384, 512, 768,
    //      1024, 1152, 1536, 2048, or 3072 Ã— fS"
    //     "SCK rates that are not common to standard audio clocks, between
    //      1MHz and 50MHz, are only supported in software mode, available
    //      only in the PCM512x and PCM514x devices"
    //  PCM5102A æ²¡æœ‰è½¯ä»¶æ¨¡å¼ã€‚è€Œæœ¬æ¿ PCM5102 æ¨¡å—çš„ SCK æŽ¥ GNDï¼Œ
    //  DAC æ‹¿ BCK å½“ç³»ç»Ÿæ—¶é’Ÿï¼Œæ‰€ä»¥ BCK/LRCK å°±æ˜¯è¿™ä¸ªæ¯”å€¼ã€‚
    //
    //     I2S_SPEC = 1'b0ï¼ˆåŽ†å²åŽŸæ ·ï¼Œ25k K44 åŒæ¬¾ï¼‰ï¼š
    //         BCK åŠå‘¨æœŸ 3 æ‹ â†’ BCK = 12.5M/6 = 2.0833 MHz
    //         ä¸€å¸§ 48 ä¸ª BCK  â†’ fs  = 12.5M/(48*6) = 43.40 kHz
    //         æ¯”å€¼ 48Ã—  â† â˜…ä¸åœ¨å…è®¸è¡¨é‡Œï¼Œè¶…è§„æ ¼è¿è¡Œ
    //
    //     I2S_SPEC = 1'b1ï¼ˆæœ¬æ¬¡æ”¹å¯¹ï¼‰ï¼š
    //         BCK åŠå‘¨æœŸ 2 æ‹ â†’ BCK = 12.5M/4 = 3.1250 MHz
    //         ä¸€å¸§ 64 ä¸ª BCK  â†’ fs  = 12.5M/(64*4) = 48.83 kHz
    //         æ¯”å€¼ 64Ã—  â† å…è®¸è¡¨é‡Œçš„æœ€å°å€¼ï¼Œåˆæ³•
    //         å·¦/å³å„ 32bit æ§½ï¼Œ16bit æ ·æœ¬å·¦å¯¹é½æ”¾åœ¨æ¯æ§½é«˜ 16 ä½
    //
    //  â˜… é‡‡æ ·çŽ‡ã€SAMPLE_CNT_MAX å…¨éƒ¨ç”±ä¸‹é¢è¿™ç»„å¸¸æ•°æŽ¨å¯¼ï¼Œ
    //    ä¿è¯ã€ŒDDS æ¯äº§ç”Ÿä¸€ä¸ªæ ·æœ¬ï¼ŒI2S æ­£å¥½å‘ä¸€ä¸ªå¸§ã€ï¼Œä¸ä¼šä¸¢æ ·æœ¬ã€‚
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

    // â˜…â˜… ä½å®½æ•™è®­ï¼ˆè¸©è¿‡ä¸€æ¬¡ï¼Œå¯¼è‡´æ•´å—éŸ³é¢‘åŸŸè¢«ç»¼åˆæŽ‰ â†’ å®Œå…¨æ²¡å£°éŸ³ï¼‰â˜…â˜…
    //   ç»å¯¹ä¸èƒ½å†™æˆ  localparam [5:0] FRAME_BITS = 6'd64;
    //   å› ä¸º 64 éœ€è¦ 7 ä½ï¼ˆ6 ä½åªèƒ½è¡¨ç¤º 0~63ï¼‰ï¼Œä¼šè¢«æˆªæ–­æˆ 0ï¼ŒäºŽæ˜¯
    //       FRAME_DIV = FRAME_BITS * BCLK_HALF * 2 = 0
    //       SAMPLE_RATE_HZ = CLK_FREQ_HZ / 0   â†’ x
    //   ç»¼åˆæ—¥å¿—é‡Œçš„è¯æ®ï¼š
    //       WARN (EX3792): Literal value 'd64 truncated to fit in 6 bits
    //       WARN (EX3810): Attempt to divide by 0, returning x
    //   åŽæžœä¸æ˜¯æŠ¥é”™ï¼Œè€Œæ˜¯ã€é™é»˜åœ°æŠŠ DDS/ROM/åŒ…ç»œ/æ··éŸ³å…¨éƒ¨ä¼˜åŒ–æŽ‰ã€‘ï¼Œ
    //   ç½‘è¡¨èµ„æºä»Ž REG 4319 æŽ‰åˆ° 124ï¼Œçƒ§è¿›åŽ»å®Œå…¨æ²¡å£°éŸ³ã€‚
    //   æ‰€ä»¥è¿™é‡Œä¸€å¾‹ç”¨ã€æ— ä½å®½çš„æ•´æ•°ã€‘åšå¸¸æ•°ç®—æœ¯ï¼Œé¿å…ä»»ä½•æˆªæ–­ã€‚
    localparam integer BCLK_HALF  = I2S_SPEC ? 2 : 3;
    localparam integer FRAME_BITS = I2S_SPEC ? 64 : 48;

    // 一帧 = FRAME_BITS 个 BCK = FRAME_BITS × 2 × BCLK_HALF 个音频时钟
    //   64bit 帧 × 2 × 2 = 256  → fs = 12.5M/256 = 48.83kHz, BCK/fs = 64×
    //   48bit 帧 × 2 × 3 = 288  → fs = 12.5M/288 = 43.40kHz, BCK/fs = 48×
    localparam integer FRAME_DIV = FRAME_BITS * BCLK_HALF * 2;

    //=========================================================================
    // ★ 采样周期 = I2S 帧周期（这一条本来就是对的，这里明确写下来）
    //
    //   FRAME_DIV = 64 × 2 × 2 = 256 个音频时钟
    //     → DAC 端 fs = 12.5MHz / 256 = 48.83 kHz
    //   SAMPLE_CNT_MAX = FRAME_DIV - 1 = 255
    //     → 音频域每 256 拍产生一个新采样，与帧边界一一对齐
    //
    //   两者必须相等：DAC 每帧取一次数，音频域每帧也给一个新数，
    //   这样"一个采样正好占一帧"，不会被连发两遍或漏掉；
    //   同时 SAMPLE_RATE_HZ 必须等于真实交付给 DAC 的采样率
    //   （12.5MHz/256 = 48828Hz），否则 calc_delta() 算出的频率字
    //   整体偏高 12.5%，所有音都会偏亮。
    //
    //   ★ 注意：不要把它写成 43402 之类的旧值 —— 那个数字对应的是
    //     48bit 帧/BLCK_HALF=3 的老配置（288 拍）。改 I2S_SPEC 时
    //     这三个常数必须一起跟着推，不能单独写死。
    //=========================================================================
    localparam integer SAMPLE_RATE_HZ = CLK_FREQ_HZ / FRAME_DIV;
    localparam integer SAMPLE_CNT_MAX = FRAME_DIV - 1;

    // éŸ³é¢‘åŸŸå¤ä½åŒæ­¥
    reg rst_a1, rst_a2;
    always @(posedge clk_audio) begin
        rst_a1 <= ~rst_n;
        rst_a2 <= rst_a1;
    end
    wire rst_audio_sync = rst_a2;

    //------------------------------------------------------------------
    // éŸ³ç¬¦é¢‘çŽ‡ â†’ é¢‘çŽ‡å­—
    //   delta = freq * 2^32 / 48000
    //   ç‰‡å†…åºå·ï¼š0~4 = é»‘é”® C# D# F# G# A#ï¼›5~11 = ç™½é”® C D E F G A B
    //   ç¬¬ n ç‰‡ = ç¬¬ 0 ç‰‡ << nï¼ˆæ­£å¥½é«˜ n ä¸ªå…«åº¦ï¼Œ2:1 ç²¾ç¡®å…«åº¦ï¼‰
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
    reg  [31:0] inc   [0:NUM_KEYS-1];   // 送进相位累加器的增量（已加移调）
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
    // ★ 轮询不停机：每个音频时钟算一个键，36 拍（2.9us）刷完全部。
    //   比滑音的更新速率（每 288 拍一次）快 12 倍，所以 dcur 一动
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
                else if (tr_gi == gt)   inc[gt] <= tr_result;
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
    // 48kHz é‡‡æ ·ä½¿èƒ½
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

    //==================================================================
    // â˜…â˜… è¯Šæ–­å¼€å…³ï¼šTEST_SINE â˜…â˜…ï¼ˆæ”¾åœ¨éŸ³é¢‘åŸŸå¼€å¤´ï¼ŒåŽé¢å¤šå¤„ä¼šç”¨åˆ°ï¼‰
    //   1'b1 â†’ å®Œå…¨ç»•è¿‡åŒ…ç»œ / å¢žç›Š / è°æ³¢ / AGC / æ¢è°ƒï¼Œç›´æŽ¥è¾“å‡ºä¸€ä¸ª
    //          å›ºå®šéŸ³é‡çš„çº¯æ­£å¼¦ï¼Œç”¨æ¥æŠŠ"éŸ³è°ƒæ•°æ®äº§ç”Ÿé“¾"å’Œ"åŒ…ç»œæ··éŸ³é“¾"
    //          å½»åº•åˆ†å¼€å®šä½ï¼š
    //     å¬åˆ°å¹²å‡€ç¨³å®šçš„çº¯éŸ³ â†’ I2S + DAC + æ—¶é’Ÿæ˜¯å¥½çš„ï¼Œ
    //                          é—®é¢˜åœ¨è¢«ç»•å¼€çš„é‚£ä¸€æ®µé‡Œï¼›
    //     ä»ç„¶å˜¶å“‘/æ–­ç»­      â†’ é—®é¢˜åœ¨ I2S / DAC / æ—¶é’Ÿæœ¬èº«ã€‚
    //   æµ‹å®ŒåŠ¡å¿…æ”¹å›ž 1'b0 å†å‡ºæ­£å¼ç‰ˆã€‚
    //==================================================================
    // ★ V17 诊断已用完，恢复 1'b0。
    //   经过它确认：显示通路（跨时钟 + wave_buf + 绘制）完全正常，
    //   "正弦 + 和 X 轴重叠的直线" 的真正原因是 lcd_master.v 里
    //   固定纵向刻度 `v >>> 7` 的【绝对值地板 128】把弱信号压成 0。
    //   已改成自动刻度，不再需要这个诊断。
    localparam TEST_SINE = 1'b0;   // ★ V19：正式发声模式（诊断已结束）★

    // æµ‹è¯•éŸ³ç›¸ä½ç´¯åŠ ï¼ˆæ¯ä¸ªé‡‡æ ·ç‚¹èµ°ä¸€æ­¥ï¼‰
    //   é¢‘çŽ‡ = SAMPLE_RATE_HZ Ã— TEST_INC / 2^32
    //   TEST_INC = 2^24 â†’ 43402 Ã— 2^24 / 2^32 â‰ˆ 169.6 Hz
    reg [31:0] test_phase;
    localparam [31:0] TEST_INC = 32'd16777216;

    always @(posedge clk_audio) begin
        if (rst_audio_sync) test_phase <= 32'd0;
        else if (sample_en)  test_phase <= test_phase + TEST_INC;
    end

    //==================================================================
    // NUM_KEYS è·¯ ADSR åŒ…ç»œ
    //
    //   å‚è€ƒ ADSR1 çš„æ€è·¯ï¼Œä½†è¡¥é½äº† Sustain ç”µå¹³ï¼Œå¹¶ä¸”è§£å†³äº†å®ƒçš„ç ´éŸ³é—®é¢˜ã€‚
    //
    //   çŠ¶æ€æµè½¬ï¼ˆæ¯ä¸ªé”®ç‹¬ç«‹ï¼‰ï¼š
    //     æŒ‰ä¸‹      ï¼šä»Žå½“å‰ç”µå¹³å¿«é€Ÿ Attack å†²åˆ° 255
    //     ç»§ç»­æŒ‰ä½  ï¼šä»¥ Decay é€ŸçŽ‡è¡°å‡åˆ° SUSTAIN_LVï¼Œç„¶åŽä¿æŒ
    //     æ¾å¼€      ï¼šä»¥ Release é€ŸçŽ‡è¡°å‡åˆ° 0ï¼ˆè¿™å°±æ˜¯"å»¶éŸ³"å°¾å·´ï¼‰
    //     å°¾å·´æ²¡ç»“æŸæ—¶å†æŒ‰ä¸‹ï¼šä»Žå½“å‰ç”µå¹³æŽ¥ç€å¾€ä¸Šå†²ï¼ˆä¸æ–­éŸ³ã€ä¸è·³å˜ï¼‰
    //
    //   æ—¶é—´ï¼ˆé‡‡æ ·çŽ‡ 43403Hzï¼Œå½“å‰å‚æ•°ä¸‹ï¼‰ï¼š
    //     Attack  ï¼š255/8 = 32 ä¸ªé‡‡æ ·ç‚¹ â‰ˆ 0.74ms
    //     Decay   ï¼š(255-0)Ã—280 = 71400 ä¸ªé‡‡æ ·ç‚¹ â‰ˆ 1.64sï¼ˆSUSTAIN_LV=0 æ—¶ï¼‰
    //     Release ï¼šåŒæ ·é€ŸçŽ‡ï¼Œä»Žå½“å‰ç”µå¹³è¡°å‡åˆ° 0
    //
    //   â˜… å‚æ•°è¯´æ˜Žï¼š
    //     ENV_SUSTAIN_LV = 0  â†’  æŒ‰ä½æ—¶ä¼šä¸€ç›´è¡°å‡åˆ°é™éŸ³ï¼ˆé’¢ç´å¼ï¼Œå½“å‰è®¾ç½®ï¼‰
    //     ENV_SUSTAIN_LV = 200 â†’ æŒ‰ä½æ—¶è¡°å‡åˆ° 200 å°±ä¿æŒï¼ˆçº¦ -2dB çš„æŒç»­éŸ³ï¼‰
    //     ENV_SUSTAIN_LV = 255 â†’  å®Œå…¨æ²¡æœ‰è¡°å‡æ®µï¼ŒæŒ‰ä½æ’å®šä¸ºæ»¡å¹…
    //==================================================================
    // â˜… æ¢å¤æˆ 25k K44 é‚£ç‰ˆçš„å–å€¼ï¼ˆé‚£ç‰ˆåŒ…ç»œ/éŸ³è‰²æ˜¯èƒ½æ­£å¸¸å‘å£°çš„åŸºå‡†ï¼‰ã€‚
    //   æ›¾ç»è¯•è¿‡æŠŠå°é˜¶é™åˆ° 1 æƒ³è®©éŸ³å¤´æ›´å¹³æ»‘ï¼Œç»“æžœ 60k ä¸Šå‘å£°åè€Œä¸å¯¹ï¼Œ
    //   æ‰€ä»¥è¿™é‡Œä¿æŒå’Œ K44 å®Œå…¨ä¸€è‡´ï¼Œä¸è¦å†å•ç‹¬æ”¹è¿™ä¸€ä¸ªæ•°ã€‚
    localparam [7:0]  ENV_ATTACK_STEP = 8'd8;      // æ¯æ¬¡é‡‡æ ·ä¸Šå‡é‡
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
    localparam [15:0] ENV_DECAY_DIV   = 16'd280;   // æ¯å¤šå°‘ä¸ªé‡‡æ ·ç‚¹è¡°å‡ 1
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
    wire [15:0] env_rel_div = flute_on_audio ? ENV_REL_DIV_FLUTE : ENV_RELEASE_DIV;   // é‡Šæ”¾é€ŸçŽ‡ï¼ˆåŒä¸Šï¼‰

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
        else if (sample_en) att_cnt <= att_tick ? 6'd0 : (att_cnt + 1'b1);
    end

    always @(posedge clk_audio) begin
        if (rst_audio_sync) begin
            dec_cnt <= 16'd0;
            rel_cnt <= 16'd0;
        end else if (sample_en) begin
            dec_cnt <= dec_tick ? 16'd0 : (dec_cnt + 1'b1);
            rel_cnt <= rel_tick ? 16'd0 : (rel_cnt + 1'b1);
        end
    end

    //------------------------------------------------------------------
    // â˜… æ ¹æ²»"åŒä¸€ä¸ªéŸ³è¢«é‡å¼¹ä¸€æ¬¡ï¼ˆå›žå£°ï¼‰"ç”¨çš„ gate åŽ»æŠ–æ…¢æ‹
    //
    //   åŒ…ç»œé  attacking æ ‡å¿—åŒºåˆ†"è¿˜åœ¨æ”»å‡»"å’Œ"æ”»å‡»å·²å®Œæˆ"ã€‚
    //   åŽŸæ¥åªè¦ sounding_mask_aud æœ‰ä¸€æ‹ä¸º 0ï¼ˆè¯¥é”®è¯»åˆ°æ¾å¼€ï¼‰å°±ç«‹åˆ»æ¸… attackingï¼Œ
    //   é”®ä½ä¸€æ¢å¤ env é©¬ä¸Šé‡æ–° attack â†’ åŒä¸€ä¸ªéŸ³å†å“ä¸€æ¬¡ï¼Œå¬æ„Ÿå°±æ˜¯"å›žå£°"ã€‚
    //   è€Œ key_mask çš„çž¬é—´æŽ‰è½å¸¸å¸¸æ¥è‡ªã€å•è½®åè¯»æ•°ã€‘ï¼ˆTTP229 è¢«å¹²æ‰°è¯»é”™ï¼‰ï¼Œ
    //   å¹¶ä¸æ˜¯çœŸçš„æ¾æ‰‹ â€”â€” å®ƒåŒæ—¶ä¹Ÿæ˜¯"å›žå£°åªåœ¨å¿«é€Ÿè¿žæŒ‰æ—¶æ˜Žæ˜¾"çš„åŽŸå› ï¼š
    //   æŒ‰å¾—æ…¢æ—¶ï¼ŒæŽ‰è½å‘ç”Ÿåœ¨éŸ³ç¬¦å·²ç»è¡°å‡åˆ°æŽ¥è¿‘ 0 çš„æ—¶å€™ï¼Œé‡å¼¹å¬ä¸å‡ºæ¥ã€‚
    //
    //   æ ¹æ²»åŠžæ³•ï¼šæ¾æ‰‹å¿…é¡»ã€åŽ»æŠ–ã€‘â€”â€” è¿žç»­ä½Žæ»¡ 8ms æ‰æ‰¿è®¤ã€‚
    //     æœªç¡®è®¤æœŸé—´ env å’Œ attacking å…¨éƒ¨ä¿æŒä¸åŠ¨ï¼š
    //     éŸ³ç¬¦ä¸æ–­ã€ç›¸ä½ä¸é‡ç½®ã€æ›´ä¸ä¼šé‡æ–°æ”»å‡»ã€‚
    //   8ms çš„åˆ¤æ®ï¼š1ms æ…¢æ‹è®¡åˆ° bit3ï¼ˆå€¼>=8ï¼‰ã€‚å•è½®åè¯»æ•° 2.26ms æœ€å¤šè®¡ 3ï¼Œ
    //   è¿žç»­ 3 è½®åè¯»æ•°(6.8ms)æœ€å¤šè®¡ 7ï¼Œéƒ½åˆ°ä¸äº† 8 â†’ ä¸€å¾‹æ»¤æŽ‰ã€‚
    //   è€Œäººæœ€å¿«çš„è¿žç»­é‡å¤å¼¹å¥é—´éš”ä¹Ÿè¿œå¤§äºŽ 8msï¼ŒçœŸæ­£çš„é‡å¤æŒ‰é”®ä¸ä¼šè¢«åžã€‚
    //------------------------------------------------------------------
    localparam [5:0] ENV_GATE_TICK_DIV = 6'd42;    // 43 ä¸ªé‡‡æ ·ç‚¹ @43402Hz â‰ˆ 0.99ms

    reg  [5:0] gate_tick_cnt;
    wire       gate_tick = (gate_tick_cnt >= ENV_GATE_TICK_DIV);

    always @(posedge clk_audio) begin
        if (rst_audio_sync) gate_tick_cnt <= 6'd0;
        else if (sample_en) gate_tick_cnt <= gate_tick ? 6'd0 : (gate_tick_cnt + 1'b1);
    end

    genvar ge;
    generate
        for (ge = 0; ge < NUM_KEYS; ge = ge + 1) begin : gen_env
            // â˜… attackingï¼šæ˜¾å¼è®°ä½"æ”»å‡»å·²ç»å®Œæˆ"ã€‚
            //   ä¸èƒ½æ‹¿ env < 255 å½“åˆ¤æ®ï¼Œå¦åˆ™è¡°å‡èµ°å‡ºä¸€æ­¥(255â†’254)åŽåˆ¤æ®åˆæˆç«‹ â†’
            //   ç«‹åˆ»é‡æ–°æ”»å‡»å›ž 255ï¼ŒåŒ…ç»œæ°¸è¿œåœåœ¨æ»¡å¹…ã€é•¿æŒ‰å˜æˆä¸€ç›´å“ã€‚
            reg attacking;
            // â˜… gate_off_cntï¼šgate å·²ç»è¿žç»­ä½Žäº†å¤šå°‘ä¸ª 1ms æ‹ï¼ˆ4 ä½é¥±å’Œï¼‰ã€‚
            //   bit3 = 1 è¡¨ç¤ºå·²è¿žç»­ä½Žæ»¡ 8ms â†’ ç¡®è®¤çœŸçš„æ¾æ‰‹äº†ã€‚
            reg [3:0] gate_off_cnt;

            always @(posedge clk_audio) begin
                if (rst_audio_sync) begin
                    env[ge]       <= 8'd0;
                    attacking     <= 1'b0;
                    gate_off_cnt  <= 4'd0;
                end else if (sample_en) begin
                    if (sounding_mask_aud[ge]) begin
                        // ---- å‘å£°ï¼ˆç‰©ç†æŒ‰ä¸‹ æˆ– å»¶éŸ³é”å­˜ï¼‰----
                        gate_off_cnt <= 4'd0;        // é”®è¿˜åœ¨ â†’ åŽ»æŠ–è®¡æ•°å¤ä½
                        if (sustain_latch_aud[ge]) begin
                            // â˜… å»¶éŸ³é”å­˜çš„é”®ï¼šä¸è¡°å‡ï¼Œé’‰åœ¨æ»¡å¹…
                            //   ï¼ˆç”¨æˆ·è¦æ±‚"ç‚¹è¿‡çš„ç´é”®ä¸€ç›´å“"ï¼ŒåŒ…ç»œç­‰äºŽæ—è·¯ï¼‰
                            env[ge]   <= 8'd255;
                            attacking <= 1'b1;
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
                            // Decay：从峰值衰减到 Sustain 电平（现在是 255 = 不衰减，笛子是 200）
                            if (dec_tick) env[ge] <= env[ge] - 8'd1;
                        end
                        // åˆ°è¾¾ Sustain åŽä¿æŒä¸åŠ¨
                    end else begin
                        // ---- gate æŽ‰äº†ï¼šå…ˆåˆ«æ€¥ç€å½“"æ¾æ‰‹" ----
                        // â˜…â˜… è¿™é‡Œå°±æ˜¯"é‡å¼¹/å›žå£°"çš„æ ¹æ²»ç‚¹ â˜…â˜…
                        //   åŽŸæ¥æ— æ¡ä»¶ attacking <= 1'b0ï¼Œå•è½®åè¯»æ•°ï¼ˆ2.26msï¼‰ä¹Ÿä¼šæ¸…æŽ‰å®ƒï¼›
                        //   é”®ä½ä¸€æ¢å¤ env å°±é‡æ–° attack â†’ åŒä¸€ä¸ªéŸ³å†å“ä¸€æ¬¡ã€‚
                        //   çŽ°åœ¨å¿…é¡»è¿žç»­ä½Žæ»¡ 8ms æ‰æ‰¿è®¤æ¾æ‰‹ï¼›æœªç¡®è®¤æœŸé—´
                        //   env å’Œ attacking å…¨éƒ¨ä¿æŒ â†’ éŸ³ç¬¦ä¸æ–­ã€ç›¸ä½ä¸é‡ç½®ã€ä¸é‡å¼¹ã€‚
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
    // "æ­£åœ¨å‘å£°"åˆ¤å®šï¼šenv != 0ï¼ˆå«å»¶éŸ³å°¾å·´ï¼‰
    //   â˜… æ··éŸ³é—¨æŽ§è¦ç”¨å®ƒï¼Œè€Œä¸æ˜¯ç”¨ key_mask â€”â€”
    //     å¦åˆ™æ¾å¼€æŒ‰é”®çš„çž¬é—´å»¶éŸ³å°¾å·´ä¼šè¢«ç›´æŽ¥æŽæŽ‰ã€‚
    //
    //   â˜…â˜… ä½†å®ƒã€ä¸èƒ½æ‹¿æ¥é—¨æŽ§ç›¸ä½ç´¯åŠ å™¨ã€‘â˜…â˜… â€”â€” è§ä¸‹é¢ç›¸ä½ç´¯åŠ é‚£æ®µçš„è¯´æ˜Žã€‚
    //------------------------------------------------------------------
    wire [NUM_KEYS-1:0] sounding_mask;
    generate
        for (ge = 0; ge < NUM_KEYS; ge = ge + 1) begin : gen_snd
            assign sounding_mask[ge] = (env[ge] != 8'd0);
        end
    endgenerate

    //==================================================================
    // æ»‘éŸ³ï¼ˆPortamentoï¼‰
    //
    //   å¼€å¯åŽï¼Œæ¯ä¸ªæ–°æŒ‰ä¸‹çš„é”®ä»Žã€ä¸Šä¸€ä¸ªéŸ³çš„éŸ³é«˜ã€‘å¼€å§‹ï¼Œ
    //   ä»¥å›ºå®šæ­¥é•¿æ»‘åˆ°è‡ªå·±çš„éŸ³é«˜ã€‚éŸ³ç¨‹è¶Šå¤§æ»‘å¾—è¶Šä¹… â€”â€” å’ŒçœŸå®žæ»‘éŸ³ä¸€è‡´ã€‚
    //
    //   â˜… æ¯ä¸ªé”®æœ‰è‡ªå·±çš„ä¸€ä»½"å½“å‰é¢‘çŽ‡å­—" dcurï¼š
    //       æŒ‰ä¸‹çž¬é—´åˆå§‹åŒ–ä¸º last_deltaï¼ˆä¸Šä¸€ä¸ªéŸ³ï¼‰
    //       ä¹‹åŽæ¯ä¸ªé‡‡æ ·ç‚¹å‘ delta[g]ï¼ˆæœ¬éŸ³ï¼‰é è¿‘ GLIDE_STEP
    //   â˜… 3/4 æ¬¡è°æ³¢çš„ç›¸ä½æ˜¯ä»Ž phase æ´¾ç”Ÿçš„ï¼Œæ‰€ä»¥ä¼šè‡ªåŠ¨è·Ÿç€ä¸€èµ·æ»‘ã€‚
    //==================================================================
    //   æ¯ä¸€æ­¥çš„é¢‘çŽ‡å­—å¢žé‡ã€‚â˜… åªæ”¹è¿™ä¸€ä¸ªæ•°å°±èƒ½è°ƒæ»‘éŸ³é€Ÿåº¦ï¼š
    //       8192  â†’ ä¸€ä¸ªå…«åº¦çº¦ 145 msï¼ˆåƒ"è¿žå¥"ï¼Œå¾ˆå¿«ï¼‰
    //       4096  â†’ ä¸€ä¸ªå…«åº¦çº¦ 290 ms
    //       2048  â†’ ä¸€ä¸ªå…«åº¦çº¦ 580 msï¼ˆå½“å‰å€¼ï¼Œå¬æ„Ÿæœ€æ˜Žæ˜¾ï¼‰  â˜…
    //       1024  â†’ ä¸€ä¸ªå…«åº¦çº¦ 1.16 sï¼ˆå¾ˆå¤¸å¼ çš„æ»‘éŸ³ï¼‰
    localparam [31:0] GLIDE_STEP = 32'd2048;   // æ¯é‡‡æ ·ç‚¹æ»‘å¤šå°‘ï¼›è°ƒå¤§=æ»‘å¾—æ›´å¿«

    // æ»‘éŸ³å¼€å…³è·¨æ—¶é’Ÿï¼ˆæ…¢ä¿¡å·ï¼Œ2 çº§åŒæ­¥è¶³å¤Ÿï¼‰
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

    // æœ€è¿‘ä¸€æ¬¡æŒ‰ä¸‹çš„é”®çš„é¢‘çŽ‡å­—ï¼Œä½œä¸ºä¸‹ä¸€ä¸ªéŸ³çš„æ»‘éŸ³èµ·ç‚¹
    reg [31:0] last_delta;

    // éŸ³é¢‘åŸŸä¸Šå‡æ²¿æ£€æµ‹ï¼šè¿™ä¸€æ‹åˆšæŒ‰ä¸‹çš„é”®
    reg  [NUM_KEYS-1:0] key_mask_d;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) key_mask_d <= {NUM_KEYS{1'b0}};
        else        key_mask_d <= key_mask;
    end
    wire [NUM_KEYS-1:0] key_new = key_mask & ~key_mask_d;

    //==================================================================
    // å»¶éŸ³é”å­˜
    //
    //   å»¶éŸ³æ‰“å¼€åŽï¼Œæ¯æ¬¡ã€æ–°ç‚¹ä¸‹ä¸€ä¸ªç´é”®ã€‘å°±æŠŠè¯¥é”® OR è¿›é”å­˜ï¼›
    //   å†ç‚¹åŒä¸€ä¸ªé”®ï¼ˆå®ƒå·²åœ¨é”å­˜é‡Œï¼‰å°±æŠŠå®ƒåŽ»æŽ‰ï¼›å¤šä¸ªé”®å åŠ ã€‚
    //   å…³é—­å»¶éŸ³æ—¶é”å­˜ä¸€æ¬¡æ€§æ¸…é›¶ï¼ˆå…¨éƒ¨é‡Šæ”¾ï¼‰ã€‚
    //
    //   â˜… ç”¨ key_new åšåˆ‡æ¢ï¼šå®ƒæ˜¯"æŒ‰ä¸‹é‚£ä¸€çž¬é—´"çš„å•æ‹è„‰å†²ï¼Œå¤©ç„¶ä¸€æ¬¡äº‹ä»¶ã€‚
    //     è‹¥æ”¹ç”¨ key_mask åš XORï¼ŒæŒ‰ä½æœŸé—´ key_mask ä¸€ç›´ä¸º 1ï¼Œ
    //     ä¼šæ¯ä¸ªæ—¶é’Ÿç¿»è½¬ä¸€æ¬¡ï¼Œæ ¹æœ¬é”ä¸ä½ã€‚
    //   â˜… æ”¾åœ¨éŸ³é¢‘åŸŸï¼Œé¿å…è·¨æ—¶é’ŸåŸŸå¼•ç”¨ 50MHz çš„ä¿¡å·ã€‚
    //==================================================================
    reg [NUM_KEYS-1:0] sustain_latch_aud;

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            sustain_latch_aud <= {NUM_KEYS{1'b0}};
        end else if (!sustain_on_audio) begin
            sustain_latch_aud <= {NUM_KEYS{1'b0}};              // å…³å»¶éŸ³ â†’ å…¨éƒ¨é‡Šæ”¾
        end else if (key_new != {NUM_KEYS{1'b0}}) begin
            sustain_latch_aud <= sustain_latch_aud ^ key_new;   // ç‚¹äº†å°±åŠ ï¼Œå†ç‚¹å°±åŽ»æŽ‰
        end
    end

    // çœŸæ­£é©±åŠ¨å‘å£°çš„æŽ©ç  = ç‰©ç†æŒ‰ä¸‹çš„é”® âˆª å»¶éŸ³é”å­˜çš„é”®
    wire [NUM_KEYS-1:0] sounding_mask_aud = key_mask | sustain_latch_aud;

    integer ki;
    always @(posedge clk_audio) begin
        // â˜… è¿™é‡Œç»å¯¹ä¸èƒ½åŠ  sample_en é—¨æŽ§ï¼key_new åªæœ‰ 1 æ‹å®½ï¼Œ
        //   è€Œ sample_en æ¯ 288 æ‹æ‰æ¥ä¸€æ¬¡ï¼ŒåŠ é—¨æŽ§åŽ 99.7% çš„æŒ‰é”®ä¼šæ¼æŽ‰ï¼Œ
        //   last_delta æ°¸è¿œåœåœ¨ 0 â†’ æ»‘éŸ³æ°¸è¿œä¸è§¦å‘ã€‚
        for (ki = 0; ki < NUM_KEYS; ki = ki + 1)
            if (key_new[ki]) last_delta <= delta[ki];
    end

    //------------------------------------------------------------------
    // NUM_KEYS è·¯ç›¸ä½ç´¯åŠ å™¨
    //   â˜… ç”¨ dcurï¼ˆæ»‘éŸ³ä¸­çš„å½“å‰é¢‘çŽ‡å­—ï¼‰è€Œä¸æ˜¯ delta
    //
    //   â˜…â˜… è¿™é‡ŒåŽŸæ¥ç”¨ sounding_maskï¼ˆ= env != 0ï¼‰é—¨æŽ§ï¼Œæ˜¯é”™çš„ â˜…â˜…
    //
    //     åŽŸæ¥çš„å†™æ³•ï¼š
    //         if (sounding_mask[g]) phase[g] <= phase[g] + inc[g];
    //         else                  phase[g] <= 32'd0;
    //
    //     é—®é¢˜å‡ºåœ¨ã€è¡°å‡å°¾å·´ã€‘ï¼šenv æ˜¯ 8bitï¼ŒRelease é˜¶æ®µ env ä»Ž 1 æŽ‰åˆ° 0
    //     ä¹‹åŽï¼Œ"æ¾æ‰‹åŽ»æŠ–"é€»è¾‘ä¼šè®©å®ƒä¸€ç›´åœåœ¨ 0ï¼Œæ­¤æ—¶ sounding_mask = 0ï¼Œ
    //     äºŽæ˜¯ phase è¢«ã€åå¤æ¸…é›¶ã€‘ã€‚è€Œ I2S å’Œæ··éŸ³è¿˜åœ¨è·‘ï¼Œ
    //     ç»“æžœæ¯ 280 ä¸ªé‡‡æ ·ç‚¹å°±æŠŠæ­£å¼¦æ³¢çš„ç›¸ä½ä»Žä»»æ„ä½ç½®ç¡¬è·³å›ž 0 â€”â€”
    //     å¬æ„Ÿå°±æ˜¯ï¼š
    //        Â· å£°éŸ³å˜å¾—å¾ˆä¸è¿žç»­ï¼ˆç›¸ä½åå¤è·³å˜ï¼‰
    //        Â· ç»“å°¾å‡ºçŽ°ä¸€ä¸ªæ˜Žæ˜¾çš„"æ”¶å°¾å£°"ï¼ˆæœ€åŽä¸€æ¬¡è·³ç›¸ï¼‰
    //     æ­£ç¡®çš„åšæ³•æ˜¯æ¸å¼±ç„¶åŽæ¶ˆå¤±ï¼ˆåŒ…ç»œè‡ªç„¶è¡°å‡åˆ° 0ï¼‰ã€‚
    //
    //     é¡ºå¸¦ä¸€æ env=1..7 æ—¶ sounding_mask ä»æ˜¯ 1ï¼ˆåˆ¤æ®æ˜¯ != 0ï¼‰ï¼Œ
    //     æ‰€ä»¥çœŸæ­£è¢«æ¸…ç›¸çš„åªæ˜¯ env å·²ç»å½’é›¶çš„é‚£äº›æ‹ã€‚
    //
    //   â˜… æ­£ç¡®åšæ³•ï¼ˆçŽ°åœ¨å°±æŒ‰è¿™ä¸ªå†™ï¼‰ï¼š
    //       Â· éŸ³ç¬¦ã€èµ·å§‹ã€‘æ‰æŠŠç›¸ä½æ¸… 0 â€”â€” éŸ³å¤´ä»Ž sin(0)=0 å¼€å§‹ï¼Œå¹²å‡€ï¼›
    //       Â· ä¹‹åŽç›¸ä½ã€æŒç»­ç´¯åŠ ã€‘ï¼Œè¡°å‡æœŸé—´ç»ä¸è·³ç›¸ï¼Œ
    //         éŸ³é‡å®Œå…¨äº¤ç»™åŒ…ç»œï¼ˆæ­£å¼¦ Ã— envï¼‰ï¼Œç»“å°¾è‡ªç„¶æ˜¯æ¸å¼±æ¶ˆå¤±ï¼›
    //       Â· éŸ³ç¬¦å½»åº•é™éŸ³ï¼ˆsounding_mask=0ï¼Œå³ env å½’ 0ï¼‰åŽç›¸ä½åœåœ¨ 0ï¼Œ
    //         è¿™æ ·ä¸ä¼šè‡ªå¼•ç”¨ã€ä¹Ÿä¿è¯å¼‚å¸¸çŠ¶æ€ä¸‹èƒ½æ”¶å›žã€‚
    //------------------------------------------------------------------
    reg [31:0] phase [0:NUM_KEYS-1];

    // éŸ³ç¬¦èµ·å§‹ï¼ˆå•æ‹è„‰å†²ï¼‰â†’ ç›¸ä½æ¸… 0
    wire [NUM_KEYS-1:0] phase_rst = key_new;

    genvar g;
    generate
        for (g = 0; g < NUM_KEYS; g = g + 1) begin : gen_phase
            always @(posedge clk_audio) begin
                if (rst_audio_sync) begin
                    phase[g] <= 32'd0;
                    dcur[g]  <= 32'd0;
                end else begin
                    // ---- æ»‘éŸ³èµ·ç‚¹ï¼šæŒ‰é”®çž¬é—´è®¾å®š ----
                    // â˜… è¿™ä¸€æ®µç»å¯¹ä¸èƒ½æ”¾è¿› sample_en é‡Œï¼
                    //   key_new åªæœ‰ 1 æ‹å®½ï¼Œè€Œ sample_en æ¯ 288 æ‹æ‰æ¥ä¸€æ¬¡ï¼Œ
                    //   æ”¾è¿›åŽ» 99.7% çš„æŒ‰é”®ä¼šé”™è¿‡åˆå§‹åŒ–ï¼Œdcur å°±ä¸€ç›´ç­‰äºŽ
                    //   delta[g]ï¼ˆä¸Šç”µåŽæ–œå¡æ—©å°±æŠŠå®ƒæ‹‰åˆ°ä½äº†ï¼‰â†’ å®Œå…¨å¬ä¸åˆ°æ»‘éŸ³ã€‚
                    if (key_new[g])
                        // å¼€äº†æ»‘éŸ³å°±ä»Žä¸Šä¸€ä¸ªéŸ³èµ·æ»‘ï¼›å¦åˆ™ç›´æŽ¥å°±ä½ã€‚
                        // last_delta==0 è¯´æ˜Žåˆšä¸Šç”µè¿˜æ²¡å¼¹è¿‡ï¼Œç›´æŽ¥å°±ä½é¿å…"ä»Ž 0 æ»‘ä¸Šæ¥"
                        dcur[g] <= (glide_on && (last_delta != 32'd0))
                                   ? last_delta : delta[g];

                    if (sample_en) begin
                        // ---- æ»‘éŸ³æ–œå¡ï¼šå‘æœ¬éŸ³é¢‘çŽ‡å­—é è¿‘ ----
                        // â˜… ç›®æ ‡å¿…é¡»æ˜¯ã€å¸¸æ•°ã€‘delta[g]ï¼
                        //   ä¸€æ—¦æ”¹æˆè¿è¡Œæ—¶å¯„å­˜å™¨ï¼Œ24 ä¸ªé”®å„è¦ä¸€å¥—çœŸçš„
                        //   32 ä½åŠ å‡æ³•å™¨ï¼Œå®žæµ‹å¤šçƒ§ 2300 ä¸ª ALUã€‚
                        //   ç§»è°ƒä¸åœ¨è¿™é‡Œåšï¼Œæ”¾åˆ°åŽé¢çš„ inc[] åŽ»ä¹˜æ¯”ä¾‹ã€‚
                        if (!key_new[g]) begin
                            if (dcur[g] < delta[g]) begin
                                if (dcur[g] + GLIDE_STEP >= delta[g]) dcur[g] <= delta[g];
                                else                                  dcur[g] <= dcur[g] + GLIDE_STEP;
                            end else if (dcur[g] > delta[g]) begin
                                if (dcur[g] - GLIDE_STEP <= delta[g]) dcur[g] <= delta[g];
                                else                                  dcur[g] <= dcur[g] - GLIDE_STEP;
                            end
                        end

                        // ---- ç›¸ä½ç´¯åŠ ï¼ˆç”¨å·²åŠ ç§»è°ƒçš„ inc[]ï¼‰----
                        //   èµ·å§‹æ¸… 0ï¼ˆéŸ³å¤´å¹²å‡€ï¼‰ï¼Œä¹‹åŽåªç´¯åŠ ã€ç»ä¸è·³ç›¸ã€‚
                        //   env å½’ 0 åŽæ— éŸ³å¯å‘ï¼ˆdin ä¼šè¢«ç®—æˆ 0ï¼‰ï¼Œ
                        //   ç›¸ä½æ¯æ‹ +inc ä¼šè‡ªç„¶å›žç»•ï¼Œä¸å½±å“ä»»ä½•ä¸œè¥¿ã€‚
                        if (phase_rst[g])      phase[g] <= 32'd0;
                        else                   phase[g] <= phase[g] + inc[g];
                    end
                end
            end
        end
    endgenerate

    //------------------------------------------------------------------
    // NUM_KEYS ä¸ª pROMï¼ˆæ¯ä¸ª 512Ã—32ï¼Œå  1 å— BSRAMï¼‰
    //
    //   â˜… ä¸€ä¸ªé‡‡æ ·å‘¨æœŸå†…æŠŠã€åŒä¸€å¼ è¡¨ã€‘è¯» 3 æ¬¡ï¼Œå–åˆ° 4 ä¸ªè°æ³¢ï¼Œ
    //     æ‰€ä»¥ BSRAM ä»ç„¶æ˜¯ 1 å—/é”®ï¼ˆ24 é”® = 24 å—ï¼Œç¬¬ 3 ç‰‡ç…§æ ·è£…å¾—ä¸‹ï¼‰ã€‚
    //     åŽŸç†ï¼šä¸€ä¸ªé‡‡æ ·å‘¨æœŸæœ‰ 288 ä¸ªéŸ³é¢‘æ—¶é’Ÿï¼Œè€ŒæŸ¥ä¸€æ¬¡è¡¨åªè¦ 2 æ‹ã€‚
    //
    //   è¡¨å†…å®¹ï¼šentry[X] = { sine[X & 0xFF], sine[X >> 1] }
    //           ï¼ˆç”Ÿæˆ sine_table_packed.mi æ—¶å®šçš„ï¼Œä¸è¦æ”¹è¿™å¼ è¡¨çš„æ ¼å¼ï¼‰
    //
    //   è¯»ç›¸ä½ç”± sample_cnt å‰ 3 æ‹å†³å®šï¼ˆcnt=0 é€åœ°å€ï¼Œcnt=1 æ”¶åˆ°æ•°æ®ï¼‰ï¼š
    //     ç›¸ä½0  X = P[31:23]     ä½ŽåŠå­— = sine[P[31:24]]    = 1 æ¬¡è°æ³¢
    //                             é«˜åŠå­— = sine[P[30:23]]    = 2 æ¬¡è°æ³¢
    //     ç›¸ä½1  X = (3P)[31:23]  ä½ŽåŠå­— = sine[(3P)[31:24]] = 3 æ¬¡è°æ³¢
    //     ç›¸ä½2  X = P[29:21]     ä½ŽåŠå­— = sine[P[29:22]]    = 4 æ¬¡è°æ³¢
    //                             ï¼ˆå› ä¸º 4P çš„é«˜ 8 ä½æ­£å¥½æ˜¯ P[29:22]ï¼‰
    //------------------------------------------------------------------
    // è¯»ç›¸ä½ï¼šæ¯ 2 æ‹ä¸€ä¸ªç›¸ä½
    //   cnt=0,2,4,6 é€åœ°å€ï¼›cnt=1,3,5,7 æ”¶åˆ°æ•°æ®ï¼ˆROM æœ‰ 1 æ‹å»¶è¿Ÿï¼‰
    wire [1:0] rdsel = sample_cnt[2:1];

    wire [31:0] rom_dout [0:NUM_KEYS-1];

    generate
        for (g = 0; g < NUM_KEYS; g = g + 1) begin : gen_rom
            // 3 å€ç›¸ä½ï¼šç»„åˆç®—ï¼ŒçœæŽ‰ä¸€è·¯ 32 ä½ç´¯åŠ å™¨
            wire [31:0] p3_g = phase[g] + {phase[g][30:0], 1'b0};

            // â˜… 4 ä¸ªç›¸ä½éƒ½æŠŠç›®æ ‡è°æ³¢æ”¾åœ¨ã€ä½ŽåŠå­—ã€‘ï¼ˆä½ŽåŠå­— = sine[X>>1]ï¼‰ï¼Œ
            //   æ‰€ä»¥åœ°å€å– X = ç›®æ ‡åœ°å€ << 1ï¼š
            //     ç›¸ä½0  X = P[31:23]              ä½ŽåŠå­— = 1 æ¬¡è°æ³¢
            //     ç›¸ä½1  X = {P[30:23], 1'b0}      ä½ŽåŠå­— = 2 æ¬¡è°æ³¢
            //     ç›¸ä½2  X = {(3P)[31:24], 1'b0}   ä½ŽåŠå­— = 3 æ¬¡è°æ³¢
            //     ç›¸ä½3  X = {P[29:22], 1'b0}      ä½ŽåŠå­— = 4 æ¬¡è°æ³¢
            wire [8:0] ad_g = TEST_SINE ? test_phase[31:23] :
                              (rdsel == 2'd0) ? phase[g][31:23]        :
                              (rdsel == 2'd1) ? {phase[g][30:23], 1'b0} :
                              (rdsel == 2'd2) ? {p3_g[31:24],     1'b0} :
                                                {phase[g][29:22],  1'b0};

            // 60k ç‰ˆï¼šç”¨æŽ¨æ–­ ROMï¼ˆsine_rom.vï¼‰æ›¿ä»£ Gowin_pROM IPï¼Œ
            //   æ•°æ®ä¸Ž sine_table_packed.mi å®Œå…¨ç›¸åŒï¼ŒåŒæ ·æ˜¯ 1 æ‹è¯»å»¶è¿Ÿã€‚
            sine_rom u_rom (
                .clk   (clk_audio),
                .ad    (ad_g),
                .dout  (rom_dout[g])
            );
        end
    endgenerate

    //------------------------------------------------------------------
    // è¯Šæ–­ç”¨æµ‹è¯•æ­£å¼¦ï¼šå–ç¬¬ 0 ä¸ª ROM çš„ä½ŽåŠå­—ï¼ˆ= åŸºæ³¢è¡¨ï¼Œ256 ç‚¹/å‘¨æœŸï¼‰ã€‚
    //   å¹…åº¦å³ç§» 2 ä½ â†’ æ»¡å¹…çš„ 1/4ï¼Œä¿è¯ä¸å‰Šé¡¶ã€‚
    //   ï¼ˆç›¸ä½ç´¯åŠ å™¨ test_phase ä¸Ž TEST_SINE å®šä¹‰åœ¨éŸ³é¢‘åŸŸå¼€å¤´ï¼‰
    //------------------------------------------------------------------
    wire signed [15:0] test_sine_raw = $signed(rom_dout[0][15:0]);
    wire signed [15:0] test_sine_sample = (test_sine_raw >>> 2);

    //------------------------------------------------------------------
    // â˜…é€é”®ä¹˜åŒ…ç»œâ˜…ï¼ˆADSR å°±è½åœ¨è¿™é‡Œï¼‰+ æ¯é”®åªç”¨ 1 ä¸ªä¹˜æ³•å™¨
    //
    //   4 ä¸ªè¯»ç›¸ä½æ¯ä¸ªåªå‡ºä¸€ä¸ªè°æ³¢å€¼ï¼ˆéƒ½åœ¨ä½ŽåŠå­—ï¼‰ï¼Œæ‰€ä»¥æ¯ä¸ªç›¸ä½
    //   åªéœ€è¦ä¹˜ä¸€æ¬¡åŒ…ç»œ â€”â€” åŒä¸€ä¸ªä¹˜æ³•å™¨åœ¨ 4 ä¸ªç›¸ä½ä¸Šå¤ç”¨ã€‚
    //   24 é”® = 24 ä¸ªä¹˜æ³•å™¨ï¼ˆä¸æ˜¯ 48 ä¸ªï¼‰ï¼ŒDSP å ç”¨ä»Ž"28 ä¸ªç”¨æ»¡ + LUT æº¢å‡º"
    //   é™åˆ° 24 ä¸ªã€‚
    //
    //   â˜… åŒ…ç»œåœ¨ã€è°æ³¢æ··åˆä¹‹å‰ã€‘é€é”®æ–½åŠ ï¼Œæ‰€ä»¥å› å¼åˆ†è§£å®Œå…¨ä¿ç•™ â€”â€”
    //     åŠ›åº¦ä»ç„¶åªéœ€è¦ 1 ä¸ªå…¨å±€ä¹˜æ³•å™¨ï¼š
    //         Î£ enváµ¢Â·(h1áµ¢ + cÂ·Háµ¢)  =  Î£ enváµ¢Â·h1áµ¢  +  c Â· Î£ enváµ¢Â·Háµ¢
    //     ï¼ˆå¦‚æžœæ”¹æˆ"å…ˆæ±‚å’Œå†ä¹˜åŒ…ç»œ"ï¼Œé‚£å˜æˆå…¨å±€åŒ…ç»œï¼Œå°±ä¸æ˜¯é€é”®çš„äº†ï¼‰
    //
    //   ä¹˜ç§¯æœ€å¤§ 32767Ã—255 = 8,355,585 < 2^23ï¼Œå–é«˜ 16 ä½å³ Ã·256ã€‚
    //------------------------------------------------------------------
    wire signed [15:0] env_a [0:NUM_KEYS-1];   // å½“å‰ç›¸ä½çš„è°æ³¢ Ã— æœ¬é”®åŒ…ç»œ

    generate
        for (g = 0; g < NUM_KEYS; g = g + 1) begin : gen_unpack
            wire signed [16:0] s_raw = $signed(rom_dout[g][15:0]);
            wire signed [23:0] s_p   = s_raw * $signed({1'b0, env[g]});
            assign env_a[g] = $signed(s_p[23:8]);
        end
    endgenerate

    //------------------------------------------------------------------
    // å½“å‰è¯»ç›¸ä½çš„ã€Œè·¨é”®æ±‚å’Œã€
    //
    //   æ¯ä¸ªè¯»ç›¸ä½åªå‡ºä¸€ä¸ªè°æ³¢ï¼ˆéƒ½åœ¨ä½ŽåŠå­—ï¼Œä¸”å·²ä¹˜æœ¬é”®åŒ…ç»œï¼‰ï¼Œ
    //   æ‰€ä»¥è¿™é‡Œåšä¸€ä¸ªè·¨é”®å’Œï¼Œå†æŒ‰è¯»ç›¸ä½æŠ“è¿› S1~S4ï¼š
    //     cnt=1 â†’ S1ï¼ˆ1 æ¬¡è°æ³¢ï¼‰   cnt=3 â†’ S2ï¼ˆ2 æ¬¡è°æ³¢ï¼‰
    //     cnt=5 â†’ S3ï¼ˆ3 æ¬¡è°æ³¢ï¼‰   cnt=7 â†’ S4ï¼ˆ4 æ¬¡è°æ³¢ï¼‰
    //
    //   â˜…â˜… å…¨å±€åŠ›åº¦èƒ½åšè¿™ä¹ˆä¾¿å®œï¼Œé çš„æ˜¯å› å¼åˆ†è§£ï¼š
    //        Î£ enváµ¢Â·(h1áµ¢ + cÂ·Háµ¢)  =  Î£ enváµ¢Â·h1áµ¢  +  c Â· Î£ enváµ¢Â·Háµ¢
    //      åŠ›åº¦ c å¯¹æ‰€æœ‰é”®ã€æ‰€æœ‰è°æ³¢éƒ½æ˜¯åŒä¸€ä¸ªå€¼ â†’ å…ˆæ±‚å’Œï¼Œæœ€åŽåªä¹˜ä¸€æ¬¡ï¼Œ
    //      æ‰€ä»¥æ— è®º 24 é”®è¿˜æ˜¯ 36 é”®ã€å‡ ä¸ªè°æ³¢ï¼ŒåŠ›åº¦éƒ½åªè¦ 1 ä¸ªä¹˜æ³•å™¨ã€‚
    //
    //   â˜… ç”¨ sounding_mask è€Œä¸æ˜¯ key_mask é—¨æŽ§ï¼š
    //     å»¶éŸ³å°¾å·´æœŸé—´è¿™ä¸€è·¯å¿…é¡»ç»§ç»­å‚ä¸Žæ··éŸ³ï¼Œå¦åˆ™å°¾å·´ä¼šè¢«æŽæŽ‰ã€‚
    //
    //   â˜… å¿…é¡»å†™æˆå•ä¸ªè¡¨è¾¾å¼ï¼ˆç»¼åˆå™¨è‡ªåŠ¨æ­ log2(N) çº§å¹³è¡¡åŠ æ³•æ ‘ï¼‰ï¼Œ
    //     ç»ä¸èƒ½æ”¹æˆ for å¾ªçŽ¯ä¸²è¡Œç´¯åŠ  â€”â€” é‚£æ ·ä¼šå˜æˆ N çº§ä¸²è¡Œé“¾ï¼Œ
    //     12.5MHz ä¸‹å¿…ç„¶æ—¶åºè¿ä¾‹ã€‚
    //   â˜…PAD3-7  ç¬¬ 3 ç‰‡ï¼ˆ[24]~[35]ï¼‰å·²è¿½åŠ 
    //
    //   22 ä½è¶³å¤Ÿï¼š36 é”® Ã— 32767 = 1,179,612 < 2^21
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
    // æŒ‰è¯»ç›¸ä½æŠŠè·¨é”®å’ŒæŠ“æˆ 4 ä¸ªè°æ³¢æ€»å’Œ
    //   S1 = Î£ envÂ·(1 æ¬¡)   S2 = Î£ envÂ·(2 æ¬¡)
    //   S3 = Î£ envÂ·(3 æ¬¡)   S4 = Î£ envÂ·(4 æ¬¡)
    //
    //   â˜… æ¯ä¸ªç´¯åŠ å™¨ä¸€ä¸ªé‡‡æ ·å‘¨æœŸåªå†™ä¸€æ¬¡ï¼Œä¸éœ€è¦æ¸…é›¶ã€‚
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
    // ç»Ÿè®¡æŒ‰ä¸‹çš„é”®æ•°
    // â˜… å¿…é¡»å†™æˆ"å•ä¸ªè¡¨è¾¾å¼"è€Œä¸æ˜¯ for å¾ªçŽ¯ç´¯åŠ ï¼š
    //   for å¾ªçŽ¯ä¼šè¢«ç»¼åˆæˆ N çº§ä¸²è¡ŒåŠ æ³•é“¾ï¼Œ12.5MHz ä¸‹ç›´æŽ¥æ—¶åºè¿ä¾‹ï¼›
    //   å†™æˆå•ä¸ªè¡¨è¾¾å¼ï¼Œç»¼åˆå™¨ä¼šè‡ªåŠ¨æ­æˆ log2(N) çº§å¹³è¡¡åŠ æ³•æ ‘ã€‚
    // â˜…PAD3-7  ç¬¬ 3 ç‰‡ï¼ˆkey_mask[24]~key_mask[35]ï¼‰å·²è¿½åŠ 
    //------------------------------------------------------------------
    // â˜…â˜… å½’ä¸€åŒ–å¢žç›Šï¼šç”±ã€Œæ•´æ•°ä¸ªæ•°ã€æ”¹æˆã€Œè¿žç»­èƒ½é‡ã€
    //
    //   åŽŸæ¥ï¼šn_keys = Î£(env[i] > 8)ï¼Œæ˜¯ä¸ªã€æ•´æ•°ä¸ªæ•°ã€‘ï¼Œå¢žç›Šè¡¨åˆæ˜¯ 6dB ä¸€æ¡£ï¼š
    //         1 ä¸ªéŸ³ â†’ 64    2 ä¸ªéŸ³ â†’ 32    3~16 ä¸ª â†’ 16 ...
    //   åŽæžœï¼šå»¶éŸ³å°¾å·´è¡°å‡åˆ°è·¨è¿‡ env=8 çš„é‚£ä¸€æ‹ï¼Œn_keys å°‘ 1ï¼Œå¢žç›Šçž¬é—´ç¿»å€(+6dB)ã€‚
    //         æ–œå¡åªæœ‰ 0.37msï¼Œç­‰äºŽä¸€ä¸ªå°é˜¶ â€”â€” å¬æ„Ÿå°±æ˜¯"æ­£åœ¨å“çš„é‚£ä¸ªéŸ³åˆè¢«å¼¹äº†ä¸€æ¬¡"ã€‚
    //         è¿™æ­£æ˜¯"å‰ä¸€ä¸ªéŸ³çš„åŒ…ç»œæ²¡ç»“æŸæ—¶æŒ‰ä¸‹å°±ä¼šè§¦å‘"ï¼š
    //           æŒ‰ä¸‹æ–°é”®æ—¶æ—§éŸ³ env è¿˜åœ¨ 8 ä»¥ä¸Š â†’ n_keys +1ï¼ˆéŸ³é‡å…ˆè¢«åŽ‹ä½Ž 6dBï¼‰ï¼›
    //           ç­‰æ—§éŸ³å°¾å·´æŽ‰åˆ° 8 ä»¥ä¸‹     â†’ n_keys -1ï¼ˆå¢žç›Š +6dB è·³å›žåŽ»ï¼‰ã€‚
    //         é—´éš”é•¿æ—¶æ—§éŸ³ env æ—©å°±æ˜¯ 0ï¼Œæ‰€ä»¥åªæœ‰"è¿žç»­æŒ‰ã€é—´éš”çŸ­"æ‰å¬å¾—åˆ°ã€‚
    //
    //   æ”¹æ³•ï¼šæŠŠæ¯ä¸ªé”®çš„åŒ…ç»œã€æŒ‰æ¯”ä¾‹ã€‘è®¡å…¥èƒ½é‡ï¼Œå°¾å·´è¡°å‡æ—¶èƒ½é‡è¿žç»­ä¸‹é™ï¼Œ
    //         å¢žç›Šè·Ÿç€è¿žç»­ä¸Šå‡ï¼Œå†ä¹Ÿæ²¡æœ‰"æŸä¸€åˆ»çªç„¶è·³ 6dB"ã€‚
    //           env_sum = Î£ (env[i] >> 5)       æ¯é”® 0~7ï¼Œ36 é”®æœ€å¤§ 252ï¼ˆ8 ä½å¤Ÿï¼‰
    //         å¢žç›Šå– gain â‰ˆ 64 / max(1, env_sum/7)ï¼Œå¤¹åœ¨ 4~64ã€‚
    //         å®ƒåœ¨ 1 / 2 / 4 ä¸ªæ»¡å¹…éŸ³ä¸Šä¸ŽåŽŸæ¥çš„æ¡£ä½åŸºæœ¬ä¸€è‡´ï¼ˆ64 / 37 / 16ï¼‰ï¼Œ
    //         å¹¶ä¸”å§‹ç»ˆæ»¡è¶³é˜²å‰Šé¡¶æ¡ä»¶ gain <= 128/nï¼Œæ¯”åŽŸæ¥çš„æ›²çº¿æ›´å®‰å…¨ã€‚
    //   â˜…PAD3-7  ç¬¬ 3 ç‰‡ï¼ˆenv[24]~env[35]ï¼‰å·²è¿½åŠ 
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
    wire signed [22:0] harm_sum   = flute_on_audio ? (harm_raw >>> 1) : harm_raw;
    wire signed [23:0] harm_scaled = harm_sum;

    //------------------------------------------------------------------
    // 基波 + 谐波混合
    //   >>> 1 对应原设计末尾那次右移（防溢出）
    //------------------------------------------------------------------
    wire signed [23:0] mix_pre = S1 + harm_scaled;
    wire signed [23:0] mix     = mix_pre >>> 1;

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
        else if (sample_en) trem_acc <= trem_acc + TREM_INC;
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
        else if (sample_en) begin
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
    wire signed [16:0] dly_wsum = s_cho + (dly_on_audio ? dly_fb : 16'sd0);
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

    reg signed [15:0] sample_vol;


    always @(posedge clk_audio) begin
        if (rst_audio_sync) sample_vol <= 16'sd0;
        else if (sample_cnt == 16'd10)
            sample_vol <= rev_on_audio ? rvb_mix : s_dly;
    end
    //==================================================================
    // ★ 波形显示：把音频采样镜像到 50MHz 域
    //
    //   音频域每产生一个新采样（sample_cnt==10 时锁存进 sample_vol），
    //   就把它推到 wave_send，并翻转 wave_tgl 通知 LCD 域来取。
    //   LCD 域（50MHz）用翻转沿握手，把每个采样写进一个 512 深的环形缓冲。
    //==================================================================
    reg signed [15:0] wave_send;
    reg               wave_tgl;

    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            wave_send <= 16'sd0;
            wave_tgl  <= 1'b0;
        end else if (sample_cnt == 16'd10) begin
            wave_send <= sample_vol;
            wave_tgl  <= ~wave_tgl;
        end
    end
    // â˜… åªè¦è¿˜æœ‰åŒ…ç»œæ²¡è¡°å‡å®Œçš„é€šé“ï¼ŒI2S å°±ä¸èƒ½åœï¼Œ
    //   å¦åˆ™æ¾å¼€æŒ‰é”®çš„çž¬é—´å»¶éŸ³å°¾å·´ä¼šè¢«ç›´æŽ¥æŽæŽ‰ã€‚
    wire any_key = |sounding_mask;

    //==================================================================
    // I2S å‘é€ï¼ˆBCK/LCK/DINï¼‰
    //   BCLK_HALF / FRAME_DIV / é‡‡æ ·çŽ‡å·²åœ¨ä¸Šé¢ç»Ÿä¸€å®šä¹‰ï¼Œæ­¤å¤„ä¸å†é‡å¤å£°æ˜Ž
    //==================================================================
    reg [3:0] bclk_cnt;
    reg       bclk_r;

    // â˜… è¯Šæ–­ï¼šI2S_SPEC=1 æ—¶æ—¶é’Ÿå¸¸å¼€ï¼ˆä¸å— any_key é—¨æŽ§ï¼‰ï¼Œ
    //   æ–¹ä¾¿ç”¨ TEST_A/TEST_C å•ç‹¬éªŒè¯ DAC é€šè·¯ã€‚
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
    // å¸§å‚æ•°ï¼ˆI2S_SPEC / BCLK_HALF / FRAME_BITS å·²åœ¨éŸ³é¢‘åŸŸå¼€å¤´å®šä¹‰ï¼‰
    //   48bit å¸§ â†’ å·¦å³å„ 24bit æ§½ï¼ˆå·¦å¯¹é½ 16bit æ•°æ®ï¼‰
    //   64bit å¸§ â†’ å·¦å³å„ 32bit æ§½ï¼ˆå·¦å¯¹é½ 16bit æ•°æ®ï¼‰
    // -----------------------------------------------------------------
    localparam integer I2S_HALF_BITS = FRAME_BITS / 2;
    localparam integer I2S_LAST_BIT  = FRAME_BITS - 1;
    localparam integer I2S_HALF_IDX  = I2S_HALF_BITS - 1;

    // -----------------------------------------------------------------
    // I2S ç§»ä½å‘é€
    //   â˜… æ•°æ®ä¸€å¾‹ã€å·¦å¯¹é½ã€‘æ”¾åœ¨æ¯å£°é“çš„é«˜ 16 ä½ï¼š
    //       48bit å¸§ â†’ {sample[15:0], 8'b0}    ï¼ˆå·¦å¯¹é½ 24bit æ§½ï¼Œ16bit æ•°æ®åœ¨é¡¶ï¼‰
    //       64bit å¸§ â†’ {sample[15:0], 16'b0}   ï¼ˆå·¦å¯¹é½ 32bit æ§½ï¼Œ16bit æ•°æ®åœ¨é¡¶ï¼‰
    //     PCM5102A æ˜¯ MSB-first + 2's complementï¼Œå–æ¯å£°é“å¼€å¤´è‹¥å¹²ä½ä½œä¸ºæ ·æœ¬ï¼Œ
    //     æ‰€ä»¥å·¦å¯¹é½æ˜¯æ­£ç¡®åšæ³•ï¼ˆæ•°æ®æ‰‹å†Œå¯¹ 16bit æ•°æ®çš„æ ‡å‡†æŽ¥æ³•ï¼‰ã€‚
    //   â˜… ç§»ä½å¯„å­˜å™¨å›ºå®š 32 ä½å®½ï¼Œé¿å…ä»¥å‰é‚£ç§
    //     ã€Œ24 ä½å˜é‡è¢«èµ‹ 32 ä½è¡¨è¾¾å¼ã€çš„æˆªæ–­ï¼ˆEX3791 è­¦å‘Šï¼‰ã€‚
    //     å‘é€æ—¶ä»Ž bit31 èµ·å·¦ç§»ï¼Œå‰ I2S_HALF_BITS ä½å°±æ˜¯è¯¥å£°é“çš„å†…å®¹ã€‚
    // -----------------------------------------------------------------
    // 静音（any_key=0）时送全 0，而不是停时钟
    wire [31:0] i2s_load = any_key
                         ? (I2S_SPEC ? {sample_vol[15:0], 16'b0}
                                     : {sample_vol[15:0],  8'b0})
                         : 32'd0;

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
                i2s_bit_cnt <= I2S_HALF_BITS[5:0];
                ws_r        <= 1'b1;
                sd_r        <= 1'b0;
                shift_reg   <= {1'b0, i2s_load[31:0], 1'b0};
            end else begin
                i2s_bit_cnt <= i2s_bit_cnt + 1'b1;
                sd_r        <= shift_reg[32];
                shift_reg   <= {shift_reg[31:0], 1'b0};
            end
        end
    end

    // â˜… TEST_SINE=1'b1 æ—¶ä¸ºè¯Šæ–­æ¨¡å¼ï¼šI2S å¸¸å¼€ã€æ•°æ®å–æµ‹è¯•æ­£å¼¦ã€‚
    // â˜… æ—¶é’Ÿå¸¸å¼€ï¼ˆI2S_SPEC=1ï¼‰ï¼Œæ•°æ®åœ¨é™éŸ³æ—¶è‡ªç„¶ä¸º 0
    //   ï¼ˆsample_vol ç”± sounding_mask é—¨æŽ§ç®—å‡º 0ï¼‰ã€‚
    // ★ I2S_ALWAYS=1 时三根线常开（静音时 DIN 自然是 0）。
    assign bck = i2s_run ? bclk_r : 1'b0;
    assign lck = i2s_run ? ws_r   : 1'b0;
    assign din = i2s_run ? sd_r   : 1'b0;

    //==================================================================
    // ADS1115ï¼š4 é€šé“ ADC é‡‡é›†
    //   ä¸Žç”µå­ç´å…±ç”¨ clk_50m / rst_nï¼Œå®Œå…¨ç‹¬ç«‹äºŽ 12.5MHz éŸ³é¢‘åŸŸ
    //   AIN0/AIN1 é©±åŠ¨ä¸Šé¢çš„"æ‘‡æ†ç§»è°ƒ / æ€»éŸ³é‡"
    //   ï¼ˆain ä¿¡å·å·²åœ¨å‰é¢"æ‘‡æ†"ä¸€èŠ‚å£°æ˜Žï¼‰
    //
    //   â˜… UART ä¸ŠæŠ¥ï¼ˆVOFA+ï¼‰å·²æŒ‰è¦æ±‚æ•´ä½“ç§»é™¤ï¼š
    //     ä¸²å£å¼•è„š uart_tx å·²ä»Žç«¯å£å’Œ music.cst é‡Œåˆ æŽ‰ï¼Œå¼•è„šç©ºå‡ºæ¥äº†ã€‚
    //     uart.v / uart_rx.v / uart_tx.v / uart_tx_ads.v è¿™å‡ ä¸ªæ–‡ä»¶
    //     è¿˜ç•™åœ¨ src\ ç›®å½•é‡Œï¼Œä½†æ²¡æœ‰è¢«å®žä¾‹åŒ–ï¼Œä¸å‚ä¸Žç»¼åˆï¼›
    //     å“ªå¤©æƒ³æ¢å¤ä¸ŠæŠ¥ï¼Œé‡æ–°æŽ¥ä¸Š uart_tx_ads å³å¯ã€‚
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

    // â˜… æ ‡å‡† MIDI è§£æžå‡ºæ¥çš„æŒ‰é”®æŽ©ç ï¼ˆNote On/Off -> 36 é”®ï¼‰
    //   å®ƒå°±æ˜¯"æ ‡å‡†çš„ç´é”®å¼¹å¥"ï¼Œæ‰€ä»¥å’Œç‰©ç†ç´é”®ä¸€æ ·ï¼šæ—¢å‘å£°ã€ä¹Ÿä¸Šé¢‘è°±ã€‚
    wire [35:0] midi_keys;
    wire [35:0] bt_keys;
    //==================================================================
    // â˜… è“ç‰™è‡ªåŠ¨æ¼”å¥
    //
    //   è“ç‰™æŠŠéŸ³ç¬¦å­—èŠ‚æµ(0x00~0x23 = 0~35 å·é”®)å†™è¿› score_mem[]ï¼Œ
    //   æ”¶åˆ° 0xFF åŽ play_start æ‹‰é«˜ã€‚è¿™é‡ŒæŒ‰å›ºå®šèŠ‚æ‹é€ä¸ªè¯»å‡ºéŸ³ç¬¦ï¼Œ
    //   è½¬æˆ 36 ä½æŒ‰é”®æŽ©ç ï¼Œä¸Žç‰©ç†æŒ‰é”®åˆå¹¶åŽä¸€èµ·é€è¿›å‘å£°é“¾è·¯
    //   â€”â€” äºŽæ˜¯ç´èƒ½è‡ªå·±å¼¹ï¼Œå±å¹•ä¸Šçš„æŸ±å­ä¹Ÿä¼šè·Ÿç€è·³ã€‚
    //
    //   è¯»åœ°å€ç”¨ play_idxï¼Œè€Œ play_idx æ’ä¸º note_idx+1ï¼š
    //   score_mem æ˜¯åŒæ­¥è¯»ã€æ•°æ®æ™šä¸€æ‹ï¼Œè¿™ä¸ª +1 æ­£å¥½è¡¥å¿ï¼Œ
    //   æ‰€ä»¥å½“å‰æ‹è¯»åˆ°çš„å°±æ˜¯ note_idx å·çš„éŸ³ç¬¦ï¼ˆä¸æ˜¯ä¸‹ä¸€ä¸ªï¼‰ã€‚
    //
    //   èŠ‚æ‹ï¼šAUTO_PLAY_DIV = 0.5 ç§’ä¸€ä¸ªéŸ³ï¼Œæƒ³å¿«æ”¹å°è¿™ä¸ªå¸¸æ•°ã€‚
    //==================================================================
    localparam [25:0] AUTO_PLAY_DIV = 26'd25_000_000;   // 50MHz x 0.5s

    reg [5:0]  note_idx;
    reg [25:0] note_cnt;
    reg [7:0]  play_idx;
    reg [35:0] auto_keys;
    reg        auto_en;        // â˜… åªæœ‰æ”¶åˆ°è¿‡æ•°æ®æ‰ä½¿èƒ½è‡ªåŠ¨æ¼”å¥

    // â˜… play_start æ˜¯ç”µå¹³ä¿æŒçš„ï¼Œå¿…é¡»å–ä¸Šå‡æ²¿ï¼›
    //   å¦åˆ™æ¯æ‹éƒ½å¤ä½ note_idxï¼Œè‡ªåŠ¨æ¼”å¥ä¼šå¡åœ¨ç¬¬ä¸€ä¸ªéŸ³ã€‚
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

            // è¯»åœ°å€æ’ä¸º note_idx+1ï¼ˆè¡¥å¿åŒæ­¥è¯»çš„ 1 æ‹å»¶è¿Ÿï¼‰
            play_idx <= {2'b0, note_idx} + 8'd1;

            if (play_start_pulse) begin
                // æ–°æ›²è°±åˆ°è¾¾ï¼šåªåœ¨è¿™ä¸€æ‹å¤ä½ä¸€æ¬¡ï¼Œå¹¶ä½¿èƒ½è‡ªåŠ¨æ¼”å¥
                note_idx <= 6'd0;
                note_cnt <= 26'd0;
                auto_en  <= 1'b1;
            end
            else if (note_cnt >= AUTO_PLAY_DIV) begin
                note_cnt <= 26'd0;
                if (play_len == 8'd0)                  note_idx <= 6'd0;
                else if (note_idx >= play_len - 1'b1)  note_idx <= 6'd0;   // å¾ªçŽ¯æ¼”å¥
                else                                   note_idx <= note_idx + 1'b1;
            end
            else begin
                note_cnt <= note_cnt + 1'b1;
            end
        end
    end

    // éŸ³ç¬¦ -> 36 ä½æŒ‰é”®æŽ©ç 
    // â˜… åªåœ¨ä½¿èƒ½åŽæ‰ç‚¹äº®ï¼šå¦åˆ™ä¸Šç”µæ—¶ note_idx=0 ä¼šè®© 0 å·é”®ï¼ˆç¬¬ä¸€æ ¹æŸ±å­ï¼‰
    //   æ°¸ä¹…é«˜ç”µå¹³ â€”â€” å±å¹•ä¸Šå°±æ˜¯"00 é‚£æ ¹æŸ±ä¸€ç›´äº®ã€å¯¹åº”ç´é”®æ²¡ååº”"ã€‚
    always @(*) begin
        auto_keys = 36'd0;
        if (auto_en) auto_keys[note_idx] = 1'b1;
    end

    // â˜… å±å¹•é¢‘è°±ã€åªè·Ÿç‰©ç†ç´é”®èµ°ã€‘
    //
    //   å¯¹åº”å…³ç³»ï¼šæŒ‰ç¬¬ i ä¸ªç‰©ç†ç´é”® -> key_levels ç¬¬ (i-1) ä½ -> ç¬¬ (i-1) æ ¹æŸ±å­
    //   lcd_master é‡Œ bar_key_on = key_levels[play_idx]ã€
    //              bar_x      = 16 + play_idx*5
    //   ä¸¤è€…å…±ç”¨ play_idxï¼Œæ‰€ä»¥æŸ±å· = ç´é”®å·ï¼Œä¸¥æ ¼ä¸€ä¸€å¯¹åº”ã€‚
    //
    //   â˜… ç»å¯¹ä¸è¦æŠŠ auto_keys å¹¶è¿›æ¥ï¼šè“ç‰™è‡ªåŠ¨æ¼”å¥æ—¶ä¼šç‚¹äº®éŸ³ç¬¦å¯¹åº”çš„æŸ±å­ï¼Œ
    //     é¢‘è°±å°±ä¸å†æ˜¯"ä½ æŒ‰ä¸‹çš„ç´é”®"äº† â€”â€” è¿™æ­£æ˜¯ä¹‹å‰é¢‘è°±å¯¹ä¸ä¸Šçš„åŽŸå› ã€‚
    //     è‡ªåŠ¨æ¼”å¥åªå½±å“å‘å£°ï¼ˆè§ä¸‹é¢ auto_keys_aud -> key_maskï¼‰ã€‚
    // â˜… é¢‘è°±æ¥æº = ç‰©ç†ç´é”® âˆª MIDI éŸ³ç¬¦ï¼ˆä¸¤è€…éƒ½æ˜¯"æ ‡å‡†çš„æŒ‰é”®äº‹ä»¶"ï¼Œä½ç½®ä¸€ä¸€å¯¹åº”ï¼‰
    //   æ³¨æ„ä¸è¦å¹¶å…¥ auto_keysï¼šå®ƒæ˜¯ç¼“å†²æ’­æ”¾çš„å¸¸é©»éŸ³ç¬¦ï¼Œä¼šè®©æŸ±å­ä¸€ç›´å ç€ã€‚
    // 屏幕频谱来源 = 物理琴键（去抖后的稳定快照）∪ MIDI 音符
    //   ★ 用两级同步的 pressed_mask（= ttp229_reader 里 key_stable 的拼接），
    //     不用自动演奏音符：它是缓冲播放的常驻音符，会让柱子一直占着。
    //     （原来这里用的是 last_pressed，它只在事件沿更新，显示会抖；
    //       现在直接跟稳定快照走，和 music_test0 的做法一致。）
    // ★ 频谱来源 = 音频域真实发声状态（谁 env != 0 谁亮）
    reg [NUM_KEYS-1:0] sounding_s1, sounding_s2;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sounding_s1 <= {NUM_KEYS{1'b0}};
            sounding_s2 <= {NUM_KEYS{1'b0}};
        end else begin
            sounding_s1 <= sounding_mask;
            sounding_s2 <= sounding_s1;
        end
    end
    wire [35:0] key_levels = sounding_s2;

    // ---- è‡ªåŠ¨æ¼”å¥éŸ³ç¬¦è·¨åˆ°éŸ³é¢‘åŸŸå‚ä¸Žå‘å£°ï¼ˆä¸¤çº§åŒæ­¥ï¼Œæ…¢ä¿¡å·è¶³å¤Ÿï¼‰----
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

    // â˜… MIDI éŸ³ç¬¦è·¨åˆ°éŸ³é¢‘åŸŸå‚ä¸Žå‘å£°ï¼ˆä¸¤çº§åŒæ­¥ï¼Œ50MHz->12.5MHz æ…¢ä¿¡å·è¶³å¤Ÿï¼‰
    reg [35:0] midi_keys_s1, midi_keys_s2;
    always @(posedge clk_audio or negedge rst_n) begin
        if (!rst_n) begin
            midi_keys_s1 <= 36'd0;
            midi_keys_s2 <= 36'd0;
        end else begin
            midi_keys_s1 <= midi_keys;
            midi_keys_s2 <= midi_keys_s1;
        end
    end
    wire [35:0] midi_keys_aud = midi_keys_s2;


    // å»¶éŸ³é”å­˜ä»ŽéŸ³é¢‘åŸŸåŒæ­¥å›ž 50MHz åŸŸï¼ˆä»…ç”¨äºŽå±å¹•æ˜¾ç¤ºï¼‰
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

    reg [35:0] rec_mask_s1, rec_mask_s2;              // 门控掩码跨到 clk_50m_ext
    always @(posedge clk_50m_ext or negedge rst_n) begin
        if (!rst_n) begin
            rec_mask_s1 <= 36'd0;
            rec_mask_s2 <= 36'd0;
        end else begin
            rec_mask_s1 <= sounding_mask_aud;           // ★ 门控（按键 | 延音锁存）
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
        .fx_state_in (fx_s2),              // ★ 录音：影响声音的 8 个效果开关
        .btn_rec     (btn_rec_pulse),      // ★ 录音开关按钮（按钮板第 10 键）
        // ★ 按键自定义映射的写口：UART 域(clk_50m_ext) -> key_map 所在的音频域(clk_audio)
        //   由上面那段"翻转信号 + 边沿检测"接收
        .keymap_we_tgl (keymap_we_tgl),
        .keymap_key    (keymap_key),
        .keymap_note   (keymap_note),
        .score_we    (score_we),
        .score_waddr (score_waddr),
        .score_wdata (score_wdata),
        .play_start  (play_start),
        .play_len    (play_len),
        .midi_keys   (midi_keys),
        .bt_keys     (bt_keys)        
    );

    // ---- LCD driver ----
    lcd_master u_lcd (
        .sys_clk    (clk_50m_ext),
        .rst_n      (rst_n),
        .play_start (play_start),
        .play_len   (play_len),
        .score_addr (score_addr),
        .score_data (score_data),
        .key_levels (key_levels),
        .timbre_idx ({1'b0, timbre_50m}),

        // ★ 新增
        .wave_mode  (wave_mode),
        .wave_send  (wave_send),
        .wave_tgl   (wave_tgl),

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
