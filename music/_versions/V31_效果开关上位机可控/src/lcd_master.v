//=============================================================================
// lcd_master.v  ——  ST7789 320x240 LCD 驱动（K18）
//
//   显示模式（由 TTP229 键4 的 wave_mode 切换）：
//     wave_mode = 0 : 60 根频谱柱（谐波一起显示）
//     wave_mode = 1 : 实时波形图（最近 320 个音频采样，纵向 ±90）
//
//   两种模式共用 y=44~223 这块主区，同一时刻只画一种，切换时清区。
//
//   ★ 波形模式改法：每列先整列刷黑（180 像素），再在采样对应的
//     1~3 个像素上画青色。旧点被黑覆盖，所以不会“画过不消退”。
//
//   ★★ 柱高改法（本次）★★
//     以前：顶层送 36 位并行 key_levels（只有亮/灭）+ 一个全局 bar_gain，
//           所有柱子共用同一高度 —— 一按就是一片等高柱子，不像真频谱。
//     现在：顶层把 36 个键各自的包络 env[g]【一根一根轮流扫着送】过来
//           （key_lvl_idx / key_lvl_val / key_lvl_tgl，像 TTP229 扫键盘），
//           这里存进 36 项小表 col_lvl[]，画哪一列就读哪一列对应的键 =>
//           每根柱子按【自己那个音】的包络长高、变矮，互不相干。
//     省资源：跨时钟线从 36 位并行 + 8 位增益 变成 6+8+1 = 15 位。
//=============================================================================
`timescale 1ns / 1ps

module lcd_master (
    input  wire sys_clk,      // 50MHz
    input  wire rst_n,

    input  wire play_start,
    input  wire [7:0] play_len,
    output reg  [7:0] score_addr,
    input  wire [7:0] score_data,

    // ★ 逐键电平扫描输入（代替原来的 36 位 key_levels + bar_gain）
    input  wire [5:0]  key_lvl_idx,   // 正在送的第几个键 0~35
    input  wire [7:0]  key_lvl_val,   // 该键的实时电平（包络）0~255
    input  wire        key_lvl_tgl,   // 每换一根键翻转一次（握手沿）
    input  wire [1:0]  timbre_idx,

    // ★ 新增：显示模式 0=频谱 1=波形
    input  wire        wave_mode,
    // ★ 新增：音频采样值（12.5MHz 域，用翻转沿握手送入）
    input  wire signed [15:0] wave_send,
    input  wire        wave_tgl,

    output wire [3:0] state_out,
    output reg  lcd_sck,
    output reg  lcd_mosi,
    output reg  lcd_dc,
    output reg  lcd_cs,
    output reg  lcd_blk
);

    // ============================================================
    // 参数
    // ============================================================
    localparam INIT_COUNT  = 16'd30;
    localparam CLEAR_COUNT = 32'd153600;
    localparam WIN_COUNT   = 16'd11;

    localparam SCK_DIV    = 8'd1;
    localparam PWR_DELAY  = 24'd10_000_000;
    localparam ONE_SEC_CNT= 26'd50_000_000;

    // ---------- 频谱柱参数 ----------
    localparam BAR_NUM    = 6'd60;
    localparam BAR_W      = 16'd3;
    localparam BAR_STEP   = 16'd5;
    localparam BAR_MARGIN = 16'd16;
    localparam BAR_H      = 16'd180;
    localparam BAR_Y_TOP  = 16'd44;
    localparam BAR_Y_BOT  = 16'd223;
    localparam [7:0] BAR_ROW_MAX = 8'd179;

    localparam [15:0] BAR_H0 = 16'd60;
    localparam [15:0] BAR_H1 = 16'd100;
    localparam [15:0] BAR_H2 = 16'd140;
    localparam [15:0] BAR_H3 = 16'd180;

    // ---------- 波形参数 ----------
    localparam [15:0] WAVE_Y_MID  = 16'd133;      // 主区中心 (44+223)/2
    localparam [15:0] WAVE_Y_TOP  = 16'd44;
    localparam [15:0] WAVE_Y_BOT  = 16'd223;
    localparam [9:0]  WAVE_N      = 10'd320;      // 一屏 320 列
    localparam [15:0] WAVE_COL_H  = WAVE_Y_BOT - WAVE_Y_TOP + 16'd1;  // 180
    // 纵向钳位：采样 >>8 后范围 ±128，超过 ±90 直接顶到边界
    localparam signed [15:0] WAVE_CLAMP = 16'sd90;

    localparam [15:0] COLOR_WAVE  = 16'h07FF;     // 波形：青
    localparam [15:0] COLOR_AXIS  = 16'h4208;     // 中线：暗灰
    localparam [15:0] COLOR_BG_W  = 16'h0000;     // 背景：黑

    // ---------- 数字位置 ----------
    localparam DIGIT_W    = 16'd3;
    localparam DIGIT_H    = 16'd5;
    localparam DIGIT_Y    = 16'd232;

    localparam COLOR_BAR   = 16'hF800;   // 白键：亮红
    localparam COLOR_KEY_BK= 16'h7800;   // 黑键：暗红
    localparam COLOR_BG    = 16'h0000;

    localparam COLOR_H2  = 16'hFFE0;     // 2 次：黄
    localparam COLOR_H3  = 16'h07E0;     // 3 次：绿
    localparam COLOR_H4  = 16'h07FF;     // 4 次：青

    // ---------- 状态 ----------
    localparam S_POWER_ON   = 4'd0;
    localparam S_INIT       = 4'd1;
    localparam S_IDLE       = 4'd2;
    localparam S_CLEAR_WIN  = 4'd3;
    localparam S_CLEAR      = 4'd4;
    localparam S_BAR_WIN    = 4'd5;
    localparam S_BAR_DATA   = 4'd6;
    localparam S_DIGIT_WIN  = 4'd7;
    localparam S_DIGIT_DATA = 4'd8;
    localparam S_IDLE_WAIT  = 4'd9;
    // ★ 新增
    localparam S_CLEAR_BAR  = 4'd10;     // 只清主区（切模式用）
    localparam S_WAVE_WIN   = 4'd11;     // 波形：一列的窗口命令
    localparam S_WAVE_DATA  = 4'd12;     // 波形：一列刷黑 + 画 1~3 个像素
    localparam S_CLEAR_BAR_DATA = 4'd13;

    reg [3:0]  state;
    reg [23:0] delay_cnt;
    reg [31:0] send_idx;
    reg [1:0]  phase;
    reg [7:0]  sck_div_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  cur_data;
    reg        cur_dc;

    // 柱子控制
    reg [5:0]  bar_prev;
    reg [25:0] sec_cnt;
    reg [15:0] bar_x;
    reg [16:0] bar_pix_cnt;   // ★ 17 位：S_CLEAR_BAR_DATA 要数到 115199
    reg [15:0] bar_color;
    reg [7:0]  bar_row;
    reg [3:0]  bar_colc;

    // ★ 本列正在画的高度/颜色（窗口一开始就锁住，画的过程中不变）
    reg [7:0]  bar_h_cur;      // 0~179 像素
    reg [1:0]  bar_cls_cur;    // 0=基波 1=2 次谐波
    reg [8:0]  bar_target_cur; // 本列画完之后要记下的"脏判据"
    reg [1:0]  bar_armed;      // ★ 判断脏判据前先空转两拍（流水线对齐）

    reg [8:0]  bar_state [0:59];   // ★ 每列上次画完的 {高度>>1, 颜色类别}
    integer    dm_i;

    // ★ 波形控制
    reg [9:0]  wave_col;                  // 当前画到第几列 0~319
    reg        wave_mode_d;               // 模式上一次的值，用于检测切换
    reg [15:0] wave_ys, wave_ye;          // 保留（现在不用了）
    reg [1:0]  wave_pix;                  // 保留（现在不用了）

    // ★ 波形采样环形缓冲（50MHz 域接收）
    reg signed [15:0] wave_buf [0:4095];
    reg [11:0]        wave_wr;
    reg [11:0]        wave_wr_sync;
    reg [11:0]        frame_wr;            // ★ 帧快照：一帧开始时锁住的写指针
    reg wave_t1, wave_t2, wave_t3;
    reg signed [15:0] wave_in;

    // ============================================================
    // ★★ 每键电平表 + 扫描接收（本次改动的核心）
    //
    //   顶层像 TTP229 扫键盘那样，一根一根轮流把 (键号, 电平) 送过来；
    //   这里在握手翻转沿把它记到"列电平表"里。
    //
    //   键号 -> 列号 的对应关系和原来的 col_fund 一模一样
    //   （每 12 键一组，组内顺序 5,0,6,1,7,8,2,9,3,10,4,11），
    //   所以画面上的键位、黑白键颜色、2 次谐波列都保持原样。
    // ============================================================
    reg [7:0] col_lvl [0:59];      // 列电平表（36..59 列没有键，恒 0）

    reg       lvl_t1, lvl_t2, lvl_t3;
    wire      lvl_we = lvl_t2 ^ lvl_t3;

    reg [5:0] key_col;
    always @(*) begin
        case (key_lvl_idx)
            6'd0:  key_col = 6'd1;      6'd1:  key_col = 6'd3;
            6'd2:  key_col = 6'd6;      6'd3:  key_col = 6'd8;
            6'd4:  key_col = 6'd10;     6'd5:  key_col = 6'd0;
            6'd6:  key_col = 6'd2;      6'd7:  key_col = 6'd4;
            6'd8:  key_col = 6'd5;      6'd9:  key_col = 6'd7;
            6'd10: key_col = 6'd9;      6'd11: key_col = 6'd11;
            6'd12: key_col = 6'd13;     6'd13: key_col = 6'd15;
            6'd14: key_col = 6'd18;     6'd15: key_col = 6'd20;
            6'd16: key_col = 6'd22;     6'd17: key_col = 6'd12;
            6'd18: key_col = 6'd14;     6'd19: key_col = 6'd16;
            6'd20: key_col = 6'd17;     6'd21: key_col = 6'd19;
            6'd22: key_col = 6'd21;     6'd23: key_col = 6'd23;
            6'd24: key_col = 6'd25;     6'd25: key_col = 6'd27;
            6'd26: key_col = 6'd30;     6'd27: key_col = 6'd32;
            6'd28: key_col = 6'd34;     6'd29: key_col = 6'd24;
            6'd30: key_col = 6'd26;     6'd31: key_col = 6'd28;
            6'd32: key_col = 6'd29;     6'd33: key_col = 6'd31;
            6'd34: key_col = 6'd33;     default: key_col = 6'd35;
        endcase
    end

    integer lvl_i;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            lvl_t1 <= 1'b0;
            lvl_t2 <= 1'b0;
            lvl_t3 <= 1'b0;
            for (lvl_i = 0; lvl_i < 60; lvl_i = lvl_i + 1)
                col_lvl[lvl_i] <= 8'd0;
        end else begin
            lvl_t1 <= key_lvl_tgl;
            lvl_t2 <= lvl_t1;
            lvl_t3 <= lvl_t2;
            // ★ 用 t2^t3 判决：tgl 已同步两拍，(idx,val) 稳定 >= 2 拍
            if (lvl_we) col_lvl[key_col] <= key_lvl_val;
        end
    end

    // 数字控制
    reg [5:0]  dig_idx;
    reg [15:0] dig_x;
    reg [15:0] dig_y;
    reg [15:0] dig_pix_cnt;

    // 演奏控制
    reg [7:0]  play_idx;
    reg        playing;
    reg        play_start_d;
    wire       play_start_pulse = play_start & ~play_start_d;

    reg [8:0]  tbl_out;

    wire [5:0]  bar_score_val = (score_data < 8'd36) ? score_data[5:0] : 6'd0;

    wire [15:0] bar_h_tbl = (timbre_idx == 2'd0) ? BAR_H0 :
                            (timbre_idx == 2'd1) ? BAR_H1 :
                            (timbre_idx == 2'd2) ? BAR_H2 : BAR_H3;

    // ---- 本列（= 本键）自己的电平 ----
    //   play_idx 0..59 环扫；0..35 列对应 36 个琴键，36..59 列没有键。
    wire [7:0]  lvl_col1 = col_lvl[play_idx[5:0]];

    // ---- 2 次谐波列：原来 col_h2 = col_fund << 12 ----
    //   第 i 列的 2 次谐波来自第 (i-12) 列那个键（低一个八度），
    //   高度也按【那个键自己的电平】来。
    wire [7:0]  lvl_col2 = (play_idx >= 6'd12 && play_idx < 6'd48)
                         ? col_lvl[play_idx[5:0] - 6'd12] : 8'd0;

    // ★★ 流水线第一级：先把"本列电平"打一拍。
    //   60:1 的选择器 + 后面那个 16×8 乘法 + 拼脏判据，串起来实测有
    //   25ns，50MHz 下跑不动（STA 报 setup 违例）。
    //   拆成两级之后，最长的一段只剩"查表"（约 4ns）和"乘法+拼位"（约 10ns）。
    reg [7:0] lvl_a1, lvl_a2;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            lvl_a1 <= 8'd0;
            lvl_a2 <= 8'd0;
        end else begin
            lvl_a1 <= lvl_col1;
            lvl_a2 <= lvl_col2;
        end
    end

    // ---- 谐波判据（与原来一致，只是判据换成"电平是否为 0"）----
    wire        col_on1 = (lvl_a1 != 8'd0);
    wire        col_on2 = timbre_idx[0] && (lvl_a2 != 8'd0);
    wire        col_any = col_on1 | col_on2;
    wire [1:0]  harm_cls = col_on1 ? 2'd0 : 2'd1;

    // ---- 本列高度 = 音色满高 × 本键电平 / 256 ----
    //   基波用满高；2 次谐波列只有一半高（沿用原来的比例）。
    wire [7:0]  col_lvl_use = col_on1 ? lvl_a1 : lvl_a2;
    wire [15:0] col_h_full  = (harm_cls == 2'd0) ? bar_h_tbl : (bar_h_tbl >> 1);
    wire [23:0] col_h_mul   = col_h_full * {8'd0, col_lvl_use};
    wire [15:0] col_h_pix   = col_h_mul[23:8];       // 0~179 像素

    // ★ 脏判据：存"已经画出来的高度（2 像素一档）+ 颜色类别"。
    //   只有真的差 >= 2 像素才重画这一列 —— 否则包络每掉 1 个 count
    //   都要重画一列（一列 1.4ms），LCD 根本追不上。
    wire [8:0]  col_target = col_any ? {col_h_pix[8:1], harm_cls[0]} : 9'd0;

    // ★★ 流水线第二级：高度/颜色/脏判据再打一拍。
    //   判断 bar_dirty 时用下面这些寄存器，不再用组合值。
    //   代价：S_BAR_WIN 判断前多空转两拍（bar_armed 计数），
    //   60 列一轮多 120 拍 = 2.4µs，跟一列 1.4ms 比完全不算事。
    reg [8:0] tgt_q;
    reg [7:0] h_q;
    reg [1:0] cls_q;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            tgt_q <= 9'd0;
            h_q   <= 8'd0;
            cls_q <= 2'd0;
        end else begin
            tgt_q <= col_target;
            h_q   <= col_h_pix[7:0];
            cls_q <= harm_cls;
        end
    end
    wire        bar_dirty  = (bar_state[play_idx[5:0]] != tgt_q);

    wire [3:0]  col_in_oct = play_idx % 12;
    wire        col_is_blk = (col_in_oct == 4'd1) || (col_in_oct == 4'd3) ||
                             (col_in_oct == 4'd6) || (col_in_oct == 4'd8) ||
                             (col_in_oct == 4'd10);
    wire [15:0] bar_color_use = col_is_blk ? COLOR_KEY_BK : COLOR_BAR;

    // 颜色、高度都用本列开画时锁住的那份（画的过程中不再变）
    wire [15:0] bar_colr  = (bar_cls_cur == 2'd0) ? bar_color_use : COLOR_H2;

    wire [15:0] bar_pix_lim = BAR_H * BAR_W * 2;
    wire [15:0] bar_ys = BAR_Y_TOP;
    wire [15:0] bar_ye = BAR_Y_TOP + BAR_H - 1;

    wire [7:0]  bar_dist   = BAR_ROW_MAX - bar_row;
    wire        bar_row_on = ({8'd0, bar_dist} < {8'd0, bar_h_cur});
    wire [7:0]  bar_pix_hi = bar_row_on ? bar_colr[15:8] : 8'h00;
    wire [7:0]  bar_pix_lo = bar_row_on ? bar_colr[7:0]  : 8'h00;

    // ============================================================
    // 字符编码（与原来完全一致）
    // ============================================================
    wire [3:0] pos_in_group = dig_idx % 12;
    reg  [4:0] dig_code;
    always @(*) begin
        case (pos_in_group)
            4'd0:  dig_code = 5'd1;
            4'd1:  dig_code = 5'd10;
            4'd2:  dig_code = 5'd2;
            4'd3:  dig_code = 5'd11;
            4'd4:  dig_code = 5'd3;
            4'd5:  dig_code = 5'd4;
            4'd6:  dig_code = 5'd12;
            4'd7:  dig_code = 5'd5;
            4'd8:  dig_code = 5'd13;
            4'd9:  dig_code = 5'd6;
            4'd10: dig_code = 5'd14;
            4'd11: dig_code = 5'd7;
            default: dig_code = 5'd1;
        endcase
    end

    reg [14:0] glyph;
    always @(*) begin
        case (dig_code)
            5'd1:  glyph = 15'b010_110_010_010_111;
            5'd2:  glyph = 15'b111_001_111_100_111;
            5'd3:  glyph = 15'b111_001_111_001_111;
            5'd4:  glyph = 15'b101_101_111_001_001;
            5'd5:  glyph = 15'b111_100_111_001_111;
            5'd6:  glyph = 15'b111_100_111_101_111;
            5'd7:  glyph = 15'b111_001_010_010_010;
            5'd10: glyph = 15'b110_101_111_101_101;
            5'd11: glyph = 15'b111_101_110_101_111;
            5'd12: glyph = 15'b011_100_100_100_011;
            5'd13: glyph = 15'b110_101_101_101_110;
            5'd14: glyph = 15'b111_100_111_100_111;
            default: glyph = 15'b000_000_000_000_000;
        endcase
    end

    wire [3:0] pix_n   = dig_pix_cnt[4:1];
    wire [2:0] pix_row = pix_n / 3;
    wire [2:0] pix_col = pix_n % 3;
    wire       dig_on  = glyph[(4 - pix_row) * 3 + (2 - pix_col)];

    // ============================================================
    // 初始化 / 清屏 查表
    // ============================================================
    always @(*) begin
        tbl_out = 9'h000;
        case (state)
            S_INIT: begin
                case (send_idx[7:0])
                    8'd0:  tbl_out = {1'b0, 8'h36};
                    8'd1:  tbl_out = {1'b1, 8'h70};
                    8'd2:  tbl_out = {1'b0, 8'h3A};
                    8'd3:  tbl_out = {1'b1, 8'h05};
                    8'd4:  tbl_out = {1'b0, 8'hB2};
                    8'd5:  tbl_out = {1'b1, 8'h0C};
                    8'd6:  tbl_out = {1'b1, 8'h0C};
                    8'd7:  tbl_out = {1'b1, 8'h00};
                    8'd8:  tbl_out = {1'b1, 8'h33};
                    8'd9:  tbl_out = {1'b1, 8'h33};
                    8'd10: tbl_out = {1'b0, 8'hB7};
                    8'd11: tbl_out = {1'b1, 8'h35};
                    8'd12: tbl_out = {1'b0, 8'hBB};
                    8'd13: tbl_out = {1'b1, 8'h19};
                    8'd14: tbl_out = {1'b0, 8'hC0};
                    8'd15: tbl_out = {1'b1, 8'h2C};
                    8'd16: tbl_out = {1'b0, 8'hC2};
                    8'd17: tbl_out = {1'b1, 8'h01};
                    8'd18: tbl_out = {1'b0, 8'hC3};
                    8'd19: tbl_out = {1'b1, 8'h12};
                    8'd20: tbl_out = {1'b0, 8'hC4};
                    8'd21: tbl_out = {1'b1, 8'h20};
                    8'd22: tbl_out = {1'b0, 8'hC6};
                    8'd23: tbl_out = {1'b1, 8'h0F};
                    8'd24: tbl_out = {1'b0, 8'hD0};
                    8'd25: tbl_out = {1'b1, 8'hA4};
                    8'd26: tbl_out = {1'b1, 8'hA1};
                    8'd27: tbl_out = {1'b0, 8'h21};
                    8'd28: tbl_out = {1'b0, 8'h11};
                    8'd29: tbl_out = {1'b0, 8'h29};
                    default: tbl_out = 9'h000;
                endcase
            end

            S_CLEAR_WIN: begin
                case (send_idx[3:0])
                    4'd0:  tbl_out = {1'b0, 8'h2A};
                    4'd1:  tbl_out = {1'b1, 8'h00};
                    4'd2:  tbl_out = {1'b1, 8'h00};
                    4'd3:  tbl_out = {1'b1, 8'h01};
                    4'd4:  tbl_out = {1'b1, 8'h3F};
                    4'd5:  tbl_out = {1'b0, 8'h2B};
                    4'd6:  tbl_out = {1'b1, 8'h00};
                    4'd7:  tbl_out = {1'b1, 8'h00};
                    4'd8:  tbl_out = {1'b1, 8'h00};
                    4'd9:  tbl_out = {1'b1, 8'hEF};   // y: 0..239 整屏
                    4'd10: tbl_out = {1'b0, 8'h2C};
                    default: tbl_out = 9'h000;
                endcase
            end
            // ★ 只清主区：x=0..319, y=44..223（320×180）
            S_CLEAR_BAR: begin
                case (send_idx[3:0])
                    4'd0:  tbl_out = {1'b0, 8'h2A};
                    4'd1:  tbl_out = {1'b1, 8'h00};
                    4'd2:  tbl_out = {1'b1, 8'h00};
                    4'd3:  tbl_out = {1'b1, 8'h01};
                    4'd4:  tbl_out = {1'b1, 8'h3F};   // x: 0..319
                    4'd5:  tbl_out = {1'b0, 8'h2B};
                    4'd6:  tbl_out = {1'b1, 8'h00};
                    4'd7:  tbl_out = {1'b1, 8'h2C};   // y: 44
                    4'd8:  tbl_out = {1'b1, 8'h00};
                    4'd9:  tbl_out = {1'b1, 8'hDF};   // y: 223
                    4'd10: tbl_out = {1'b0, 8'h2C};
                    default: tbl_out = 9'h000;
                endcase
            end

            default: tbl_out = 9'h000;
        endcase
    end

    wire [7:0] tbl_data = tbl_out[7:0];
    wire       tbl_dc   = tbl_out[8];

    // ============================================================
    // 柱子窗口命令
    // ============================================================
    reg [8:0]  bar_cmd_out;
    wire [15:0] bar_xs = bar_x;
    wire [15:0] bar_xe = bar_x + BAR_W - 1;

    always @(*) begin
        bar_cmd_out = 9'h000;
        case (send_idx[3:0])
            4'd0:  bar_cmd_out = {1'b0, 8'h2A};
            4'd1:  bar_cmd_out = {1'b1, bar_xs[15:8]};
            4'd2:  bar_cmd_out = {1'b1, bar_xs[7:0]};
            4'd3:  bar_cmd_out = {1'b1, bar_xe[15:8]};
            4'd4:  bar_cmd_out = {1'b1, bar_xe[7:0]};
            4'd5:  bar_cmd_out = {1'b0, 8'h2B};
            4'd6:  bar_cmd_out = {1'b1, bar_ys[15:8]};
            4'd7:  bar_cmd_out = {1'b1, bar_ys[7:0]};
            4'd8:  bar_cmd_out = {1'b1, bar_ye[15:8]};
            4'd9:  bar_cmd_out = {1'b1, bar_ye[7:0]};
            4'd10: bar_cmd_out = {1'b0, 8'h2C};
            default: bar_cmd_out = 9'h000;
        endcase
    end
    wire [7:0] bar_cmd_data = bar_cmd_out[7:0];
    wire       bar_cmd_dc   = bar_cmd_out[8];

    // ============================================================
    // 数字窗口命令
    // ============================================================
    reg [8:0]  dig_cmd_out;
    wire [15:0] dig_xs = dig_x;
    wire [15:0] dig_xe = dig_x + DIGIT_W - 1;
    wire [15:0] dig_ys = dig_y;
    wire [15:0] dig_ye = dig_y + DIGIT_H - 1;

    always @(*) begin
        dig_cmd_out = 9'h000;
        case (send_idx[3:0])
            4'd0:  dig_cmd_out = {1'b0, 8'h2A};
            4'd1:  dig_cmd_out = {1'b1, dig_xs[15:8]};
            4'd2:  dig_cmd_out = {1'b1, dig_xs[7:0]};
            4'd3:  dig_cmd_out = {1'b1, dig_xe[15:8]};
            4'd4:  dig_cmd_out = {1'b1, dig_xe[7:0]};
            4'd5:  dig_cmd_out = {1'b0, 8'h2B};
            4'd6:  dig_cmd_out = {1'b1, dig_ys[15:8]};
            4'd7:  dig_cmd_out = {1'b1, dig_ys[7:0]};
            4'd8:  dig_cmd_out = {1'b1, dig_ye[15:8]};
            4'd9:  dig_cmd_out = {1'b1, dig_ye[7:0]};
            4'd10: dig_cmd_out = {1'b0, 8'h2C};
            default: dig_cmd_out = 9'h000;
        endcase
    end
    wire [7:0] dig_cmd_data = dig_cmd_out[7:0];
    wire       dig_cmd_dc   = dig_cmd_out[8];

    // ============================================================
    // ★ 波形窗口命令（1 列宽，整列主区高度）
    //   ★ 窗口固定为 y=WAVE_Y_TOP..WAVE_Y_BOT（180 像素），
    //     由 S_WAVE_DATA 在窗口内按采样值决定哪几个像素画青、其余刷黑。
    // ============================================================
    reg [8:0]  wave_cmd_out;
    wire [15:0] wave_xs = {6'd0, wave_col};      // 0..319
    wire [15:0] wave_xe = {6'd0, wave_col};
    // ★ 窗口上下固定为整列主区
    wire [15:0] wave_win_ys = WAVE_Y_TOP;
    wire [15:0] wave_win_ye = WAVE_Y_BOT;

    always @(*) begin
        wave_cmd_out = 9'h000;
        case (send_idx[3:0])
            4'd0:  wave_cmd_out = {1'b0, 8'h2A};
            4'd1:  wave_cmd_out = {1'b1, wave_xs[15:8]};
            4'd2:  wave_cmd_out = {1'b1, wave_xs[7:0]};
            4'd3:  wave_cmd_out = {1'b1, wave_xe[15:8]};
            4'd4:  wave_cmd_out = {1'b1, wave_xe[7:0]};
            4'd5:  wave_cmd_out = {1'b0, 8'h2B};
            4'd6:  wave_cmd_out = {1'b1, wave_win_ys[15:8]};
            4'd7:  wave_cmd_out = {1'b1, wave_win_ys[7:0]};
            4'd8:  wave_cmd_out = {1'b1, wave_win_ye[15:8]};
            4'd9:  wave_cmd_out = {1'b1, wave_win_ye[7:0]};
            4'd10: wave_cmd_out = {1'b0, 8'h2C};
            default: wave_cmd_out = 9'h000;
        endcase
    end
    wire [7:0] wave_cmd_data = wave_cmd_out[7:0];
    wire       wave_cmd_dc   = wave_cmd_out[8];

    // ============================================================
    // ★ 波形采样接收（翻转沿握手，50MHz 域）
    //
    //   ★★ V17c 决定性修复 ★★
    //
    //   先算账（前几版为什么都白改）：
    //     采样 48.83kHz。屏幕 320 列，一列一个点 => 一屏只有 6.55ms。
    //     而 LCD 一帧【至少要】把 320 列各写一段：每列平均 45 像素才
    //     塞得进 41.9ms（= 环形缓冲 2048 点写满的时间）。
    //     老代码每列无条件写 180 像素 => 152ms/帧 => 写指针在一帧里
    //     绕 3.6 圈 => 读到的一半是【没写过/被覆盖】的格子 => 0 => y=133
    //     => 一条直线。所以问题从来不在"快照"，而在【帧太长、窗太短】。
    //
    //   修法：采样先做 8:1 抽取再入缓冲。
    //     48.83kHz / 8 = 6.1kHz 有效采样率。
    //     一屏 320 点 => 时间窗 320/6103 = 52.4ms（原来是 6.55ms）。
    //     琴键最高音 831Hz => 一屏 43 个周期；最低音 110Hz => 5.7 个周期。
    //     抽取后最高可测频率 = 6103/2 = 3.05kHz，琴键 831Hz 远低于它，
    //     不会混叠。
    //     更关键：8:1 之后写指针慢 8 倍 => 2048 点要 335ms 才写满，
    //     而一帧 152ms < 335ms，【终于追得上】，不再被覆盖。
    //     这样连快照都不需要，最省资源。
    // ============================================================
    localparam [2:0] WAVE_DECIM = 3'd7;      // 每 8 个采样取 1 个（0 => 每 1 个）

    reg [2:0]  wave_dcnt;
    reg        wave_we;                       // 本拍真的要写缓冲

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            wave_t1 <= 1'b0; wave_t2 <= 1'b0; wave_t3 <= 1'b0;
            wave_wr <= 12'd0;
            wave_wr_sync <= 12'd0;
            wave_in <= 16'sd0;
            wave_dcnt <= 3'd0;
            wave_we   <= 1'b0;
        end else begin
            wave_t1 <= wave_tgl;
            wave_t2 <= wave_t1;
            wave_t3 <= wave_t2;
            wave_in <= wave_send;         // 数据先跟一拍

            // ★ 写指针打一拍给读侧用，避开"正在写的那格被读"
            wave_wr_sync <= wave_wr;

            wave_we <= 1'b0;
            if (wave_t2 ^ wave_t3) begin
                // 检测到新采样：按 WAVE_DECIM 抽取后再写
                if (wave_dcnt >= WAVE_DECIM) begin
                    wave_dcnt <= 3'd0;
                    wave_we   <= 1'b1;
                    wave_buf[wave_wr] <= wave_in;
                    wave_wr <= wave_wr + 12'd1;
                end else begin
                    wave_dcnt <= wave_dcnt + 3'd1;
                end
            end
        end
    end

    // ★ 12 位无符号回绕：frame_wr - 320 在 frame_wr < 320 时自动绕到 4096 深处，
    //   正好是环形缓冲里往前数 320 个的正确地址。
    //   用 13 位再截 12 位会在负值时算错，导致读到 0，画成一条直线。
    wire [11:0] wave_rd_addr0 = frame_wr - 12'd320 + {2'b00, wave_col};
    wire [11:0] wave_rd_addr1 = wave_rd_addr0 + 12'd1;

    // ★ 同步读：Gowin 才认成 BSRAM。
    //   组合读会被展开成 4096:1 选择器（几十万 LUT），布线器直接卡死。
    reg signed [15:0] wave_v0, wave_v1;
    always @(posedge sys_clk) begin
        wave_v0 <= wave_buf[wave_rd_addr0];
        wave_v1 <= wave_buf[wave_rd_addr1];
    end

    // ============================================================
    // ★★ V17c 真正的根因：有符号 / 无符号比较 ★★
    //
    //   原代码：
    //       wire [15:0] wave_y0 = (wave_y0_s > WAVE_Y_BOT) ? WAVE_Y_BOT :
    //                             (wave_y0_s < WAVE_Y_TOP) ? WAVE_Y_TOP :
    //                                                        wave_y0_s[15:0];
    //   而 WAVE_Y_TOP / WAVE_Y_BOT 是 localparam [15:0]（无符号）。
    //
    //   Verilog 规则：只要有一个操作数无符号，整个比较就按【无符号】做。
    //   于是 wave_y0_s 的 signed 被吃掉，负数变成巨大的无符号数：
    //
    //       y = 133 + (v >>> WAVE_SHIFT)
    //       v = -5000  ->  y = -23  ->  无符号看是 65513
    //                   ->  65513 > 223 成立  ->  钳到 WAVE_Y_BOT = 223
    //
    //   结果：负半周凡深过约 -4256 的部分，全部被贴到【屏幕底部一条直线】。
    //   正半周的 ">223" 本来就该钳，所以看着还像正弦 ——
    //   这就是"截取一个周期看，前半段正弦、后半段直线"的真正来源。
    //
    //   修法：钳位必须和【带符号】常量比较（下面用 WAVE_Y_TOP_S/BOT_S）。
    // ============================================================
    // ============================================================
    // ★★★ V17i 根因：固定纵向刻度 -> 自动刻度 ★★★
    //
    //   原来：y = 133 + (v >>> WAVE_SHIFT)，WAVE_SHIFT 固定 = 7。
    //
    //   `>>> 7` 是一道【绝对值地板】：凡是 |v| < 128 的采样一律变成 0，
    //   也就是画在 y = 133（零点、和 X 轴重叠的那条线）上。
    //
    //   而本工程单音衰减后的幅度只有几百 —— 于是大半个周期都低于地板、
    //   全部塌到 0，屏幕上就是"正弦 + 一条和 X 轴重叠的直线"。
    //   地板是绝对值，所以【改刻度时那条线不动，只有波形缩放】。
    //
    //   实测三项全部对上（top.v 的 TEST_SINE 诊断）：
    //     TEST_SINE 幅度 8191   -> 地板只占 1.6%  -> 完美正弦
    //     TEST_SINE 满幅 32767  -> 占 0.4%，但削顶 -> 方波，也没有直线
    //     真实单音（衰减后几百） -> 地板占 40%~60% -> 一条横线
    //   双音叠加幅度大 -> 地板占比低 -> "几乎没直线"。
    //
    //   修法：地板改成【相对】的 —— 跟踪峰值 peak，
    //         刻度 sh = msb(peak) - 5，使 peak>>sh 落在 [32,64)。
    //   于是地板永远是"峰值的 1/32 左右"，
    //   信号再轻也不会塌成直线，信号再响也不会削顶（半高是 90）。
    //
    //   ★ 另外把符号和右移彻底分开：先取绝对值 -> 逻辑右移（无歧义）
    //     -> 再补回符号。全程不用 >>>，不依赖任何有符号右移语义。
    // ============================================================
    wire [15:0] v0_abs = wave_v0[15] ? (~wave_v0[15:0] + 16'd1) : wave_v0[15:0];
    wire [15:0] v1_abs = wave_v1[15] ? (~wave_v1[15:0] + 16'd1) : wave_v1[15:0];

    // 峰值跟踪：快攻（立刻跟上）+ 慢放（约 84ms 时间常数）
    reg [15:0] wave_peak;
    reg [15:0] peak_div;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            wave_peak <= 16'd2048;
            peak_div  <= 16'd0;
        end else begin
            peak_div <= peak_div + 16'd1;
            if (v0_abs > wave_peak)
                wave_peak <= v0_abs;
            else if ((peak_div == 16'hFFFF) && (wave_peak > 16'd2048))
                wave_peak <= wave_peak - (wave_peak >> 6);
        end
    end

    // sh = msb(peak) - 5  ->  peak>>sh 落在 [32,64)，恒小于可用半高 90
    wire [3:0] peak_msb = wave_peak[15] ? 4'd15 : wave_peak[14] ? 4'd14 :
                          wave_peak[13] ? 4'd13 : wave_peak[12] ? 4'd12 :
                          wave_peak[11] ? 4'd11 : wave_peak[10] ? 4'd10 :
                          wave_peak[9]  ? 4'd9  : wave_peak[8]  ? 4'd8  :
                          wave_peak[7]  ? 4'd7  : wave_peak[6]  ? 4'd6  :
                          wave_peak[5]  ? 4'd5  : wave_peak[4]  ? 4'd4  :
                          wave_peak[3]  ? 4'd3  : wave_peak[2]  ? 4'd2  :
                          wave_peak[1]  ? 4'd1  : 4'd0;
    wire [3:0] wave_sh = (peak_msb > 4'd5) ? (peak_msb - 4'd5) : 4'd0;

    // 幅值缩放（无符号逻辑右移，绝无歧义）
    wire [15:0] v0_sc = v0_abs >> wave_sh;
    wire [15:0] v1_sc = v1_abs >> wave_sh;

    // 补回符号（17 位有符号，足够容纳 -32768）
    wire signed [16:0] v0_sg = wave_v0[15] ? -$signed({1'b0, v0_sc})
                                           :  $signed({1'b0, v0_sc});
    wire signed [16:0] v1_sg = wave_v1[15] ? -$signed({1'b0, v1_sc})
                                           :  $signed({1'b0, v1_sc});

    // 叠加到中线，用 17 位有符号常量钳位
    wire signed [16:0] wave_y0_s = 17'sd133 + v0_sg;
    wire signed [16:0] wave_y1_s = 17'sd133 + v1_sg;
    wire [15:0] wave_y0 = (wave_y0_s > 17'sd223) ? 16'd223 :
                          (wave_y0_s < 17'sd44)  ? 16'd44  :
                                                   wave_y0_s[15:0];
    wire [15:0] wave_y1 = (wave_y1_s > 17'sd223) ? 16'd223 :
                          (wave_y1_s < 17'sd44)  ? 16'd44  :
                                                   wave_y1_s[15:0];

    // ★ 本列的竖线区间 = [min(y0,y1), max(y0,y1)]
    //   相邻两列共享一个采样点，所以列与列之间天然接上，是连续折线
    wire [15:0] wave_seg_lo = (wave_y0 < wave_y1) ? wave_y0 : wave_y1;
    wire [15:0] wave_seg_hi = (wave_y0 < wave_y1) ? wave_y1 : wave_y0;

    // 本列颜色：青色
    wire [7:0] wave_hi = COLOR_WAVE[15:8];
    wire [7:0] wave_lo = COLOR_WAVE[7:0];

    // ★ 当前像素在主区里的 y 坐标
    //   bar_pix_cnt[15:1] = 本列第几个像素（0..179）
    wire [15:0] wave_pix_idx = bar_pix_cnt[15:1];
    wire [15:0] wave_pix_y   = WAVE_Y_TOP + wave_pix_idx;
    // ★ 该像素是否落在 [y0, y1] 这段区间内
    wire        wave_draw_seg = (wave_pix_y >= wave_seg_lo) && (wave_pix_y <= wave_seg_hi);
    wire [7:0]  wave_pix_hi   = wave_draw_seg ? wave_hi : 8'h00;
    wire [7:0]  wave_pix_lo   = wave_draw_seg ? wave_lo : 8'h00;

    // 本列总像素数固定 = 整列主区高度
    localparam [16:0] WAVE_COL_PIX = 17'd180;
    localparam [16:0] WAVE_COL_LIM = WAVE_COL_PIX * 2;   // 360

    // ============================================================
    // 主状态机
    // ============================================================
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_POWER_ON;
            delay_cnt   <= 24'd0;
            send_idx    <= 32'd0;
            bit_idx     <= 3'd0;
            cur_data    <= 8'h00;
            cur_dc      <= 1'b0;
            phase       <= 2'd0;
            sck_div_cnt <= 8'd0;
            lcd_sck     <= 1'b0;
            lcd_mosi    <= 1'b0;
            lcd_dc      <= 1'b0;
            lcd_cs      <= 1'b1;
            lcd_blk     <= 1'b0;

            bar_prev    <= 6'd59;
            sec_cnt     <= 26'd0;
            bar_x       <= 16'd0;
            bar_pix_cnt <= 16'd0;
            bar_color   <= COLOR_BG;
            bar_row     <= 8'd0;
            bar_colc    <= 4'd0;
            bar_h_cur      <= 8'd0;
            bar_cls_cur    <= 2'd0;
            bar_target_cur <= 9'd0;
            bar_armed      <= 2'd0;
            for (dm_i = 0; dm_i < 60; dm_i = dm_i + 1) bar_state[dm_i] <= 9'd0; 

            dig_idx     <= 6'd0;
            dig_x       <= 16'd0;
            dig_y       <= 16'd0;
            dig_pix_cnt <= 16'd0;

            play_idx    <= 8'd0;
            playing     <= 1'b0;
            score_addr  <= 8'd0;

            // ★ 波形复位
            wave_col    <= 10'd0;
            wave_mode_d <= 1'b0;
            wave_pix    <= 2'd0;
            frame_wr    <= 12'd0;
        end
        else begin
            case (state)

                S_POWER_ON: begin
                    lcd_cs  <= 1'b1;
                    lcd_sck <= 1'b0;
                    lcd_blk <= 1'b1;
                    if (delay_cnt >= PWR_DELAY) begin
                        delay_cnt   <= 24'd0;
                        state       <= S_INIT;
                        lcd_cs      <= 1'b0;
                        send_idx    <= 32'd0;
                        bit_idx     <= 3'd0;
                        phase       <= 2'd0;
                        sck_div_cnt <= 8'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                S_INIT: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 8'd0;
                        case (phase)
                            2'd0: begin lcd_sck <= 0; cur_data <= tbl_data; cur_dc <= tbl_dc; lcd_dc <= tbl_dc; phase <= 2'd1; end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == INIT_COUNT - 1) begin
                                        state  <= S_IDLE;
                                        lcd_cs <= 1'b1;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                S_IDLE: begin
                    lcd_sck     <= 0;
                    lcd_cs      <= 1'b0;
                    send_idx    <= 0;
                    bit_idx     <= 0;
                    phase       <= 0;
                    sck_div_cnt <= 0;
                    state       <= S_CLEAR_WIN;
                end

                S_CLEAR_WIN: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin lcd_sck <= 0; cur_data <= tbl_data; cur_dc <= tbl_dc; lcd_dc <= tbl_dc; phase <= 2'd1; end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == WIN_COUNT - 1) begin
                                        send_idx <= 0;
                                        state    <= S_CLEAR;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // 全屏清黑（只在开机做一次）
                S_CLEAR: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin lcd_sck <= 0; cur_data <= 8'h00; cur_dc <= 1'b1; lcd_dc <= 1'b1; phase <= 2'd1; end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == CLEAR_COUNT - 1) begin
                                        send_idx <= 0;
                                        dig_idx     <= 6'd0;
                                        dig_x       <= BAR_MARGIN;
                                        dig_y       <= DIGIT_Y;
                                        dig_pix_cnt <= 0;
                                        state       <= S_DIGIT_WIN;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end
                // ★ 只清主区（320×180 = 57600 像素）
                S_CLEAR_BAR: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin lcd_sck <= 0; cur_data <= tbl_data; cur_dc <= tbl_dc; lcd_dc <= tbl_dc; phase <= 2'd1; end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == WIN_COUNT - 1) begin
                                        send_idx    <= 0;
                                        bar_pix_cnt <= 0;
                                        state       <= S_CLEAR_BAR_DATA;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // ★ 主区刷黑：57600 像素 × 2 字节 = 115200
                S_CLEAR_BAR_DATA: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin lcd_sck <= 0; cur_data <= 8'h00; cur_dc <= 1'b1; lcd_dc <= 1'b1; phase <= 2'd1; end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (bar_pix_cnt >= (16'd57600 * 2 - 1)) begin
                                        // 清完了 -> 回到 S_IDLE_WAIT 重新分派
                                        bar_pix_cnt <= 0;
                                        send_idx    <= 0;
                                        state       <= S_IDLE_WAIT;
                                    end else
                                        bar_pix_cnt <= bar_pix_cnt + 1'b1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // ---------------- 频谱柱 ----------------
                S_BAR_WIN: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin
                                lcd_sck <= 0;

                                // ★ 频谱循环里也检查模式切换
                                if (wave_mode != wave_mode_d) begin
                                    wave_mode_d <= wave_mode;
                                    bar_pix_cnt <= 0;
                                    bar_armed   <= 2'd0;
                                    state       <= S_CLEAR_BAR;
                                    phase       <= 2'd0;
                                end
                                // ★★ 窗口一旦开送就必须发完 11 个字节。
                                //   以前每个字节的 phase0 都重判 bar_dirty，
                                //   现在电平是实时扫描更新的，发到一半若电平
                                //   变了就会走进 else 分支 → 只发出半个窗口 → 花屏。
                                //   所以只在 send_idx==0（窗口还没开始）时判一次。
                                else if (send_idx != 0) begin
                                    cur_data  <= bar_cmd_data;
                                    cur_dc    <= bar_cmd_dc;
                                    lcd_dc    <= bar_cmd_dc;
                                    phase     <= 2'd1;
                                end
                                // ★★ 判断前先空转两拍：让"查表 → 算高度"这两级
                                //   流水线跟上当前 play_idx（lvl_a*、tgt_q 都是寄存器）。
                                //   不多这两拍的话，判据就是上一列的值 —— 柱子会错列。
                                else if (bar_armed != 2'd2) begin
                                    bar_armed <= bar_armed + 2'd1;
                                    phase     <= 2'd0;
                                end
                                else begin
                                    score_addr <= play_idx[7:0];
                                    bar_x <= BAR_MARGIN + play_idx * BAR_STEP;
                                    bar_armed <= 2'd0;
                                    if (bar_dirty) begin
                                        // ★ 本列的高度/颜色/脏判据全部锁住
                                        //   （用的都是打了一拍的值，见上面的 tgt_q）
                                        bar_h_cur      <= h_q;
                                        bar_cls_cur    <= cls_q;
                                        bar_target_cur <= tgt_q;
                                        cur_data  <= bar_cmd_data;
                                        cur_dc    <= bar_cmd_dc;
                                        lcd_dc    <= bar_cmd_dc;
                                        phase     <= 2'd1;
                                    end else begin
                                        if (play_idx >= 6'd59) play_idx <= 8'd0;
                                        else                   play_idx <= play_idx + 1'b1;
                                        phase <= 2'd0;
                                    end
                                end
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == WIN_COUNT - 1) begin
                                        send_idx    <= 0;
                                        bar_pix_cnt <= 0;
                                        bar_row     <= 8'd0;
                                        bar_colc    <= 4'd0;
                                        state       <= S_BAR_DATA;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                S_BAR_DATA: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin
                                lcd_sck  <= 0;
                                if (bar_pix_cnt[0] == 1'b0)
                                    cur_data <= bar_pix_hi;
                                cur_dc   <= 1'b1;
                                lcd_dc   <= 1'b1;
                                phase    <= 2'd1;
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (bar_pix_cnt[0] == 1'b0)
                                        cur_data <= bar_pix_lo;
                                    else begin
                                        if (bar_colc >= (BAR_W - 1)) begin
                                            bar_colc <= 4'd0;
                                            if (bar_row < BAR_ROW_MAX)
                                                bar_row <= bar_row + 1'b1;
                                        end else
                                            bar_colc <= bar_colc + 1'b1;
                                    end

                                    if (bar_pix_cnt >= (bar_pix_lim - 1)) begin
                                        send_idx    <= 0;
                                        bit_idx     <= 0;
                                        phase       <= 2'd0;
                                        sck_div_cnt <= 0;
                                        bar_state[play_idx[5:0]] <= bar_target_cur;
                                        bar_prev <= play_idx[5:0];
                                        if (play_idx >= 6'd59) play_idx <= 8'd0;
                                        else                   play_idx <= play_idx + 1'b1;
                                        state <= S_BAR_WIN;
                                    end else
                                        bar_pix_cnt <= bar_pix_cnt + 1'b1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // ---------------- 数字 ----------------
                S_DIGIT_WIN: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin
                                lcd_sck  <= 0;
                                dig_x    <= BAR_MARGIN + dig_idx * BAR_STEP;
                                dig_y    <= DIGIT_Y;
                                cur_data <= dig_cmd_data;
                                cur_dc   <= dig_cmd_dc;
                                lcd_dc   <= dig_cmd_dc;
                                phase    <= 2'd1;
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == WIN_COUNT - 1) begin
                                        send_idx    <= 0;
                                        dig_pix_cnt <= 0;
                                        state       <= S_DIGIT_DATA;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                S_DIGIT_DATA: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin
                                lcd_sck  <= 0;
                                cur_data <= dig_on ? 8'hFF : 8'h00;
                                cur_dc   <= 1'b1;
                                lcd_dc   <= 1'b1;
                                phase    <= 2'd1;
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (dig_pix_cnt[0] == 1'b0) begin
                                        cur_data <= dig_on ? 8'hFF : 8'h00;
                                        dig_pix_cnt <= dig_pix_cnt + 1'b1;
                                    end else begin
                                        dig_pix_cnt <= dig_pix_cnt + 1'b1;
                                    end

                                    if (dig_pix_cnt >= (DIGIT_W * DIGIT_H * 2 - 1)) begin
                                        send_idx    <= 0;
                                        bit_idx     <= 0;
                                        phase       <= 2'd0;
                                        sck_div_cnt <= 0;

                                        if (dig_idx == BAR_NUM - 1) begin
                                            dig_idx   <= 6'd0;
                                            play_idx  <= 8'd0;
                                            sec_cnt   <= 26'd0;
                                            // ★ 进模式分派
                                            state     <= S_IDLE_WAIT;
                                        end else begin
                                            dig_idx <= dig_idx + 1'b1;
                                            state   <= S_DIGIT_WIN;
                                        end
                                    end
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // =================================================
                // ★ 模式分派 + 波形循环
                // =================================================
                S_IDLE_WAIT: begin
                    lcd_sck <= 0;
                    lcd_cs  <= 1'b0;

                    play_start_d <= play_start;

                    // 检测模式切换
                    if (wave_mode != wave_mode_d) begin
                        wave_mode_d <= wave_mode;
                        // 切模式：先清主区，然后重新开始画
                        bar_pix_cnt <= 0;
                        state       <= S_CLEAR_BAR;
                    end
                    else if (wave_mode) begin
                        // 波形模式：每圈从第 0 列开始
                        // ★ 进波形前锁一次写指针：用 wave_wr（稳定值），
                        //   不要用 wave_wr_sync（它可能正好在翻转沿那一拍变）
                        frame_wr <= wave_wr;
                        wave_col <= 10'd0;
                        state    <= S_WAVE_WIN;
                    end
                    else begin
                        // 频谱模式：先看要不要演奏
                        if (play_start_pulse) begin
                            play_idx <= 8'd0;
                            playing  <= 1'b1;
                            sec_cnt  <= 26'd0;
                            state    <= S_BAR_WIN;
                        end
                        else if (playing) begin
                            if (sec_cnt >= ONE_SEC_CNT / 4) begin
                                sec_cnt <= 26'd0;
                                if (play_idx >= play_len - 1) begin
                                    playing <= 1'b0;
                                    state   <= S_IDLE_WAIT;
                                end
                                else begin
                                    play_idx <= play_idx + 1'b1;
                                    state    <= S_BAR_WIN;
                                end
                            end
                            else begin
                                sec_cnt <= sec_cnt + 1;
                            end
                        end
                        else begin
                            sec_cnt <= 26'd0;
                            // 频谱循环由 S_BAR_WIN 自己推进
                            state   <= S_BAR_WIN;
                        end
                    end
                end

                S_WAVE_WIN: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                           2'd0: begin
                                lcd_sck  <= 0;
                                cur_data <= wave_cmd_data;
                                cur_dc   <= wave_cmd_dc;
                                lcd_dc   <= wave_cmd_dc;
                                phase    <= 2'd1;
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (send_idx == WIN_COUNT - 1) begin
                                        send_idx    <= 0;
                                        bar_pix_cnt <= 0;
                                        state       <= S_WAVE_DATA;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // ★ 波形一列：180 个像素全写一遍
                //   落在采样对应区间（1~3 像素）的写青色，其余写黑色。
                //   上一帧这一列的旧点因此被黑覆盖，不会残留。
                S_WAVE_DATA: begin
                    if (sck_div_cnt >= SCK_DIV) begin
                        sck_div_cnt <= 0;
                        case (phase)
                            2'd0: begin
                                lcd_sck  <= 0;
                                if (bar_pix_cnt[0] == 1'b0)
                                    cur_data <= wave_pix_hi;
                                cur_dc   <= 1'b1;
                                lcd_dc   <= 1'b1;
                                phase    <= 2'd1;
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (bar_pix_cnt[0] == 1'b0) begin
                                        cur_data    <= wave_pix_lo;
                                        bar_pix_cnt <= bar_pix_cnt + 1'b1;
                                    end else begin
                                        // 低字节发完
                                        if (bar_pix_cnt >= (WAVE_COL_LIM - 1)) begin
                                            // 本列 180 像素全写完，下一列
                                            send_idx    <= 0;
                                            bit_idx     <= 0;
                                            phase       <= 2'd0;
                                            sck_div_cnt <= 0;
                                            if (wave_col >= WAVE_N - 1) begin
                                                wave_col <= 10'd0;
                                                state    <= S_IDLE_WAIT;
                                            end else begin
                                                wave_col <= wave_col + 10'd1;
                                                state    <= S_WAVE_WIN;
                                            end
                                        end else begin
                                            cur_data    <= wave_pix_hi;   // 下一个像素的高字节
                                            bar_pix_cnt <= bar_pix_cnt + 1'b1;
                                        end
                                    end
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                default: state <= S_IDLE_WAIT;
            endcase
        end
    end

    assign state_out = state;

endmodule