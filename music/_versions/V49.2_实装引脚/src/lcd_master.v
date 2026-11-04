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
//
//   ★★ 李萨如（X-Y 示波器）—— 按钮 3 循环切到的第 3 / 第 4 / 第 5 屏 ★★
//
//     三屏共用【同一套】显存 / 打点 / 余辉 / 边框 / 十字，只有 XY 来源不同，
//     由 liss_src 选（liss_on 在档 2/3/4 都为 1）：
//       liss_src = 0（档 2，V36 新加）：X = 左声道输出 L
//                                       Y = 右声道输出 R
//       liss_src = 1（档 3，原有）：X = 干声 dry_sample
//                                   Y = 全效果之后的输出 sample_vol
//       liss_src = 2（档 4，原有）：X = 输出 sample_vol
//                                   Y = 输出经一阶全通移相 90°（在 top.v 里做好送过来）
//     四路采样都由顶层在【同一个音频沿】锁进寄存器、共用同一个 wave_tgl 翻沿，
//     所以这里取到的天生就是同一时刻的 (X,Y)，不用额外对齐。
//
//     ★ 左×右档看到的图形怎么读：
//         · 开合唱/延迟/混响 → 图形张开成环/螺旋/花（左右确实不一样了）
//         · 效果全关 → L 和 R 逐位相同 → 画的是一条【对角线】
//       ★ 对角线是【正确】的，不是 bug：它说明此刻送出去的两个声道
//         完全相同（mono 内容里本来就没有立体声信息）。mono 音源接成
//         立体声时，示波器上就是这个样子。
//
//     ★ 为什么档 4 要存在（用户点名要的）：效果全关时 dry_sample 和 sample_vol
//       逐位相同，档 3 画的是一条对角线，不像教科书上的李萨如；档 4 让两轴差
//       90°，单音才画成圆/椭圆那样的闭合环。
//
//     ★ 余辉衰减和推屏要挤在【同一遍扫描】里：
//       显存是单读单写的 BSRAM，一帧 76800 个像素；推屏本来就要把每个像素
//       读出来，顺手把"减一档"的结果写回去，就省掉一整遍只读不显示的
//       扫描（那一遍要 1.5ms，而且再占一次显存端口）。
//     ★ 只有 liss_on=1 才走这条通路：频谱/波形那两屏的状态机、常数、
//       刷新节奏一个字都没动（liss_on=0 时新加的寄存器全部原地不动）。
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
    // ★★★ V48.5：软件"手动重置 LCD"请求（0xD1）—— 翻转沿 ★★★
    //   和 uart_packet_echo **同在 clk_50m_ext 域**（不需要跨时钟域握手），
    //   这里仍然打两拍 + 边沿检测：既把它变成干净的单拍脉冲，
    //   也顺手挡掉输入上的毛刺。
    input  wire        lcd_rst_tgl,
    input  wire [1:0]  timbre_idx,

    // ★ 新增：显示模式 0=频谱 1=波形
    input  wire        wave_mode,
    // ★ 新增：音频采样值（12.5MHz 域，用翻转沿握手送入）
    input  wire signed [15:0] wave_send,
    input  wire        wave_tgl,
    // ★★ 李萨如的 X 轴 = 干声。和 wave_send 共用同一个翻转沿（wave_tgl）
    //   送过来 —— 顶层在同一个音频沿把两个采样一起推到寄存器上，
    //   所以它们天生就是同一个采样点的 X/Y，不需要第二条握手线。
    input  wire signed [15:0] wave_send_x,
    // ★★ 相位档的 Y 轴 = 输出经一阶全通移相 90°（同样和 wave_send 同一个沿）。
    //   顶层做的是"纯旁路 tap"：移相器只读 sample_vol，输出只喂给显示，
    //   可听通路一个字节都没碰。
    input  wire signed [15:0] wave_send_py,
    // ★★ V36 左×右档的两轴 = 输出级的 L / R 两个样本（同一个 wave_tgl 翻沿）。
    //   它们就是送到 I2S 两个槽的那两个值，所以这一档画出来的就是
    //   【真正送去喇叭的左右信号】之间的李萨如。
    input  wire signed [15:0] wave_send_l,
    input  wire signed [15:0] wave_send_r,
    // ★★ 李萨如开关（按钮 3 循环到第 3/4/5 屏时为 1）。
    //   它压过 wave_mode：开着画李萨如，关掉回 wave_mode 选的那一屏。
    input  wire        liss_on,
    // ★★ 三屏李萨如选哪一屏（V36 从 1 位扩成 2 位）：
    //     0 = X:左(L)      Y:右(R)
    //     1 = X:干声       Y:全效果输出   （原"档 2"）
    //     2 = X:全效果输出 Y:移相 90°输出 （原"档 3"）
    input  wire [1:0]  liss_src,

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

    // ---------- 李萨如参数 ----------
    // ★ 余辉衰减档位：每 LISS_DECAY_DIV 帧把整屏减一档（值域 0..3）。
    //     1 = 每帧都减（默认，拖尾 ≈ 3 帧）
    //     2 = 每 2 帧减一档（拖尾长一倍，余晖更亮更黏）
    //     4 = 每 4 帧（很长，快速变化的图形会糊成一团）
    //   想调拖尾就只改这一个数。
    localparam [3:0] LISS_DECAY_DIV = 4'd1;

    // ★ 李萨如专用的 SPI 分频（只影响这条通路，别的模式仍然用 SCK_DIV）。
    //   本状态机一个 bit 要 4 个 phase，所以 SCK = 50MHz / (4*(div+1))：
    //     0 -> 12.5MHz，整屏 19200B × 8bit × 80ns = 98ms ≈ 10fps（当前）
    //     1 ->  6.25MHz，整屏 196ms ≈ 5fps（拖尾看着更"厚"，但刷新更慢）
    //   ★ 为什么不直接把 SCK_DIV 调快：SCK_DIV 一动，频谱/波形那两屏的
    //     刷新节奏就跟着变，而那两屏说好了一个字不动，所以给李萨如单开一个。
    localparam [7:0] SCK_DIV_LISS = 8'd0;

    localparam [8:0]  LISS_COL_LAST  = 9'd319;    // 最后一列
    localparam [8:0]  LISS_COL_CROSS = 9'd160;    // 中心十字的竖线
    localparam [7:0]  LISS_ROW_LAST  = 8'd239;    // 最后一行
    localparam [7:0]  LISS_ROW_CROSS = 8'd120;    // 中心十字的横线
    localparam [16:0] LISS_WORDS     = 17'd76800; // 320 × 240 个字（1 个字 = 1 个像素）

    // 余辉 4 档颜色（RGB565，只拉绿通道）+ 边框/十字的暗灰
    localparam [15:0] COLOR_L0    = 16'h0000;     // 0 档：黑（已经凉透）
    localparam [15:0] COLOR_L1    = 16'h0100;     // 1 档：暗绿（G=8）
    localparam [15:0] COLOR_L2    = 16'h0400;     // 2 档：中绿（G=32）
    localparam [15:0] COLOR_L3    = 16'h07E0;     // 3 档：亮绿（G=63，刚打上去的点）
    localparam [15:0] COLOR_FRAME = 16'h4208;     // 边框 / 十字：暗灰（和波形中线同色）

    // ---------- 数字位置 ----------
    localparam DIGIT_W    = 16'd3;
    localparam DIGIT_H    = 16'd5;
    localparam DIGIT_Y    = 16'd232;

    localparam COLOR_BAR   = 16'hF800;   // 白键：亮红
    // ★★ V48.3：黑键的谱线颜色 16'h7800（半亮暗红）-> 16'h07FF（亮青）★★
    //   用户实测原话："黑键对应的谱线的颜色…现在的太暗了，基本和背景融为一体了"。
    //   原来那个 0x7800 是"同一个红色系、亮度减半"（R=15/31）—— 黑背景上确实看不清。
    //
    //   ★★ 中间走过一次弯路，记下来（V48.3 自检抓出来的）：
    //     我第一版把黑键色改成 **16'h001F（纯蓝）**，理由写的是
    //     "换成蓝色 = 满亮度、红蓝对比一眼分得开"。**这句话是错的**：
    //       纯蓝 (R=0,G=0,B=31) 的感知亮度只有 **0.11**，
    //       而原来的暗红 0x7800 (R=15) 是 **0.145** ——
    //       也就是说那一版**比原来还暗**，正好和用户要的"明亮一点"相反。
    //     人眼对蓝最不敏感（三通道权重 ≈ R 0.30 / G 0.59 / B 0.11），
    //     "满量程"≠"看着亮"。★ 判据已经写进 check_rtl_contract.py 第 [12] 节：
    //     新颜色的感知亮度必须 ≥ 0.55（旧色 0.145）。
    //
    //   ★ 现在取 **青色 16'h07FF**（G=63 + B=31，感知亮度 **0.70**，约旧色的 4.8 倍）：
    //     · 和"白键亮红 0xF800"、"2 次谐波黄 0xFFE0"、"背景黑 0x0000"、
    //       "边框暗灰 0x4208" **四者都不撞色**（这一屏只有这几种颜色）；
    //     · 波形那一屏用的也是青色，但那是**另一个画面**，不会同屏出现。
    //   ★ 想再换：白 16'hFFFF（最亮，1.00，但和黄色谐波略近）、
    //     品红 16'hF81F（0.41）、橙 16'hFD20（0.68，和黄色略近）。
    //     或者"还是红色系但更亮" 16'hC800（R=25，0.24）。
    //   ★★ 只改这一个常数，谱线/网格/背景的其余部分一个像素没动。
    localparam COLOR_KEY_BK= 16'h07FF;   // 黑键：亮青（V48.3 提亮改色，感知亮度 0.70）
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
    // ★ 李萨如新增。★ 注意：原来的 4 位已经用到 13，装不下这 4 个，
    //   所以 state 加宽到 5 位（state_out 仍然只送低 4 位，调试口不变）。
    localparam S_LISS_CLR   = 5'd14;     // 进模式：整片显存清 0
    localparam S_LISS_WIN   = 5'd15;     // 李萨如：整屏窗口命令（复用清屏那张表）
    localparam S_LISS_HOUSE = 5'd16;     // 李萨如：一个字节的显存维护（读-衰减-写回 + 插打点）
    localparam S_LISS_PUSH  = 5'd17;     // 李萨如：把本字节的 4 个像素（8 个 SPI 字节）推出去

    reg [4:0]  state;
    reg [25:0] delay_cnt;      // ★ V48.5：24 -> 26 位（手动重置的 1.0s 装得下）
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

    reg [8:0]  bar_state [0:59];

    // ★★ V48.4：每列一位的"作废位"（看门狗恢复之后让 60 列各重画一次）
    //   为什么不用"再写一遍 9'h1FF"：那是给 540 位的数组再加一个写入源，
    //   实测 +1023 个 LUT（u_lcd 3000 -> 4023）。拆成 1 位/列之后只加在 60 位上。
    //   ★ 它由【画完那一列的同一处】清掉（自消耗，无窗口、无计时器），
    //     与 V48 否掉的 `bar_force`+计时器不是一回事（详见 bar_dirty 那里）。
    reg [59:0] bar_bad;
    // ============================================================
    // ★★★ V48：切显示模式后把"脏缓存"作废 ★★★
    // ============================================================
    //   用户 2026-10-11 报："切到李萨如图形后再切回来，频谱就被清空了，
    //   但是那些音还在发出！"
    //
    //   病灶：bar_state[] 是【每列上次画完的高度】这个缓存，
    //         下面 bar_dirty = (bar_state[col] != tgt_q) 决定要不要重画某一列。
    //         切模式时把主区【清了屏】，但【没有清 bar_state】——
    //         切回来时每列算出的高度和缓存里的一模一样 -> bar_dirty = 0
    //         -> 一根都不重画 -> 屏幕就是空的。
    //   ★ 数据没丢、声音也没受影响 —— 丢的是"该重画"这个标记。
    //   ★ 这类"缓存没失效"的 bug，症状永远是"屏幕不对但功能正常"。
    //
    //   修法：切模式时把 bar_state[] 整片写成【哨兵值 9'h1FF】。
    //     它是 tgt_q 永远取不到的值（col_target ≤ 179），
    //     于是 bar_dirty 对每一列都成立，直到那一列真的被重画完 ——
    //     而 S_BAR_DATA 画完一列就会写回真实值（bar_state[play_idx] <=
    //     bar_target_cur），所以这个"强制"是【逐列自动消耗】的：
    //       · 不需要计时器、不需要任何窗口长度假设
    //       · 用户连着快按按钮 3 也不会漏（每次切模式都重新作废）
    //       · **bar_dirty 的表达式一个字都没改**（回到 V47.2 原样）
    //
    //   ★★ 为什么不能用"置一个强制标志 + 计时器清掉"（第一版就是那么写的，
    //      已经废掉）：那个方案要求"计时窗口必须比清屏 + 重画一整轮还长"，
    //      而实测 清屏 147.5ms + 重画 60 列 83.8ms ≈ 231ms（全屏清更是 ~197ms）——
    //      窗口一旦不够长（第一版就因为 24 位装不下 17,500,000 被截断成 14.5ms），
    //      标志会在清屏还没结束时就掉，一列都不会重画，bug 原样复现。
    //      "作废缓存"没有这个前提，天然不会踩。
    //   ★ 本文件的三处"切模式"点都要作废（见下面各自的位置）：
    //       ① S_BAR_WIN 里发现 wave_mode 变了       -> S_CLEAR_BAR
    //       ② S_IDLE_WAIT 里发现进出李萨如          -> 退出时 S_CLEAR_WIN
    //       ③ S_IDLE_WAIT 里发现 wave_mode 变了     -> S_CLEAR_BAR
    //     ★★ 只做 ① 是不够的：用户报的那条路是【频谱 ↔ 李萨如】，
    //        而频谱和 3 档李萨如的 wave_mode 全都是 0，永远不进 ① 那个分支 ——
    //        必须在 ② 里也作废，否则修完了用户的 bug 还在。
    //        而且频谱显示时机是在 S_BAR_WIN ↔ S_BAR_DATA 里自循环、
    //        【根本不回 S_IDLE_WAIT】，所以模式切换只能在 ①/②/③ 这三处被发现。
    //   ★ 代价：bar_state 多一条"整片写"的通路（60×9 位各加一级同步装载），
    //     实测资源影响很小（细节见版本说明）。
    // ============================================================

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
    // ★ 李萨如的 X 轴：和 wave_in 同一个翻转沿、同一拍锁存的两个采样点，
    //   所以画出来天生就是同一个采样时刻的 (X,Y)。
    reg signed [15:0] wave_in_x;
    // ★ 相位档的 Y 轴：同样是同一拍锁存的第三路（输出经一阶全通移相 90°）。
    //   三屏李萨如共用下面所有显存/打点/余辉逻辑，只是从这几路里挑两路。
    reg signed [15:0] wave_in_py;
    // ★ V36 左×右档的两轴：输出级送来的 L / R，同样和 wave_in 同一拍锁存。
    reg signed [15:0] wave_in_l;
    reg signed [15:0] wave_in_r;

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
    //   ★ V48：表达式【逐字回到 V47.2 原样】—— 切模式的强制重画不在这里做，
    //     而是靠"把 bar_state 写成哨兵值"（见 bar_state 声明处那一大段）。
    //     ★ 所以正常刷屏路径零影响："哪一列要不要重画"的判据一个字没变。
    //
    //   ★★ V48.4：多了一个【每列一位】的作废位 bar_bad，为什么不是"再写一次哨兵"：
    //     看门狗恢复之后也必须让 60 列重画一次，最直觉的写法是照抄那三处
    //     "把 bar_state 整片写 9'h1FF"。实测**代价是 1023 个 LUT**
    //     （u_lcd 的 Lut 从 3000 涨到 4023）：因为 bar_state 是
    //     **60 列 × 9 位 = 540 位**的数组，再给它加一个写入源，
    //     等于**每一位都多一级选择**。
    //     把"作废"这一位从 9 位高度里**拆出来单独放**，写入源就只加在
    //     **60 位**上（写入源多一个 = 60 个 LUT 量级），语义完全一样。
    //   ★★ 它和 V48 明确否掉的 `bar_force` 标志**不是一回事**（别搞混）：
    //       · `bar_force` 是**一个**标志位 + **一个计时器**清它 ——
    //         "窗口要多长"这个假设正是当初翻车的地方（常量被截断成 14.5ms）；
    //       · `bar_bad` 是**每列一位**，由**画完那一列的同一处**清掉
    //         （和哨兵值"画完写回真实值逐列消耗"是同一个机制），
    //         **不需要任何窗口、任何计时器**。
    wire        bar_dirty  = bar_bad[play_idx[5:0]] ||
                             (bar_state[play_idx[5:0]] != tgt_q);

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

            // ★ 整屏窗口 0..319 × 0..239：开机清屏和【李萨如整屏推屏】共用这张表，
            //   两边的窗口字节本来就一模一样，没必要抄第二份（抄错一个字节就是花屏）。
            S_CLEAR_WIN, S_LISS_WIN: begin
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
            wave_in_x <= 16'sd0;
            wave_in_py <= 16'sd0;
            wave_in_l <= 16'sd0;
            wave_in_r <= 16'sd0;
            wave_dcnt <= 3'd0;
            wave_we   <= 1'b0;
        end else begin
            wave_t1 <= wave_tgl;
            wave_t2 <= wave_t1;
            wave_t3 <= wave_t2;
            wave_in <= wave_send;         // 数据先跟一拍
            // ★ 李萨如的 X 跟着同一拍走：wave_tgl 翻沿时 (wave_in_x, wave_in)
            //   就是同一次采样，不用再开一条握手线。
            wave_in_x <= wave_send_x;
            // ★ 相位档的 Y 同理：几路都在同一拍锁存，翻沿时是同一个采样时刻。
            wave_in_py <= wave_send_py;
            // ★ V36 左×右档的两轴：同样在这一拍锁存，所以翻沿时
            //   (wave_in_l, wave_in_r) 是【同一时刻的 L / R】，图形不会乱转。
            wave_in_l <= wave_send_l;
            wave_in_r <= wave_send_r;

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

    //==================================================================
    // ★★ 把"峰值 -> 移位量"打一拍（时序治本，别删）★★
    //
    //   原来这一串是【一个 50MHz 周期里】跑完的：
    //       wave_peak -> 15 级优先编码器(peak_msb) -> 16 位桶形移位(>>wave_sh)
    //                 -> 补符号 -> 画点 -> cur_data
    //   实测数据延迟 22.5ns > 周期 20ns —— LCD 所在的 CLKOUT1(clk_50m_ext) 域
    //   一直有 setup 违例（V26/V27 就有名，V31 -1.1ns，V32 涨到 -3.1ns）。
    //   峰值是【快攻 + 84ms 慢放】的慢信号，移位量跟着它走，
    //   这里插一级寄存器只是让缩放比峰值晚 20ns 生效 —— 显示上完全看不出来，
    //   但把那条最长路径从中间切断了。
    //==================================================================
    reg [3:0] peak_msb_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) peak_msb_r <= 4'd0;
        else        peak_msb_r <= peak_msb;
    end
    wire [3:0] wave_sh = (peak_msb_r > 4'd5) ? (peak_msb_r - 4'd5) : 4'd0;

    // 幅值缩放（无符号逻辑右移，绝无歧义）
    wire [15:0] v0_sc_c = v0_abs >> wave_sh;
    wire [15:0] v1_sc_c = v1_abs >> wave_sh;

    //==================================================================
    // ★★ V41 第二刀：把"缩放结果"再打一拍（第一刀见下面 wave_y0_r 那段）★★
    //
    //   只加第一刀（wave_y0_r）之后，最长路径前移到了它前面：
    //       wave_buf/DO -> 取绝对值 -> 16 位桶形移位(>> wave_sh)
    //         -> 补符号 -> 17 位加中线 -> 限幅 -> wave_y0_r
    //   V41 构建实测 20.179ns，还差 -0.233ns（3 条违例全是它）。
    //   ★ 桶形移位是这一段里最贵的（wave_sh ∈ 0..10，要 4 级 16 位 mux），
    //     所以在它【后面】切，正好把前半条链一分为二。
    //   ★ 为什么一定安全：wave_v0/v1 和 wave_sh 在【一整列里都是不变的】——
    //     读地址 wave_rd_addr0 只跟 wave_col 走（frame_wr 是"一帧开始时锁住的
    //     写指针"，见上面第 246 行，不是连续推进的），
    //     而 wave_col 一整列才动一次：列周期 = 360 像素 × 2 字节 × 4 相 ×
    //     (SCK_DIV+1=2) 拍 ≈ 5760 拍。所以这里每一拍算出来的值完全相同，
    //     打一拍只是让它晚 20ns 生效，数值一模一样（2/5760 = 0.03%）。
    //   ★ 符号位必须【同步】打一拍：下面 v0_sg 要用 v0_sgn_r，
    //     不能再用 wave_v0[15] —— 否则符号比幅值早一拍，负半周会画反。
    //==================================================================
    reg [15:0] v0_sc_r, v1_sc_r;
    reg        v0_sgn_r, v1_sgn_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            v0_sc_r  <= 16'd0;   v1_sc_r  <= 16'd0;
            v0_sgn_r <= 1'b0;    v1_sgn_r <= 1'b0;
        end else begin
            v0_sc_r  <= v0_sc_c;      v1_sc_r  <= v1_sc_c;
            v0_sgn_r <= wave_v0[15];  v1_sgn_r <= wave_v1[15];
        end
    end
    wire [15:0] v0_sc = v0_sc_r;
    wire [15:0] v1_sc = v1_sc_r;

    // 补回符号（17 位有符号，足够容纳 -32768）
    wire signed [16:0] v0_sg = v0_sgn_r ? -$signed({1'b0, v0_sc})
                                        :  $signed({1'b0, v0_sc});
    wire signed [16:0] v1_sg = v1_sgn_r ? -$signed({1'b0, v1_sc})
                                        :  $signed({1'b0, v1_sc});

    // 叠加到中线，用 17 位有符号常量钳位
    wire signed [16:0] wave_y0_s = 17'sd133 + v0_sg;
    wire signed [16:0] wave_y1_s = 17'sd133 + v1_sg;
    wire [15:0] wave_y0_c = (wave_y0_s > 17'sd223) ? 16'd223 :
                            (wave_y0_s < 17'sd44)  ? 16'd44  :
                                                     wave_y0_s[15:0];
    wire [15:0] wave_y1_c = (wave_y1_s > 17'sd223) ? 16'd223 :
                            (wave_y1_s < 17'sd44)  ? 16'd44  :
                                                     wave_y1_s[15:0];

    //==================================================================
    // ★★ V41：把"采样值 -> 本列 y 区间"打一拍（时序治本，别删）★★
    //
    //   原来这一串是【一个 50MHz 周期里】跑完的：
    //       BSRAM 输出(wave_v0/v1) -> 取绝对值 -> 16 位桶形移位(>> wave_sh)
    //         -> 补符号 -> 17 位加中线 -> 上下限幅 -> 线段比较 -> 像素字节 -> cur_data
    //   V41 构建报告里 CLKOUT1 的 8 条 setup 违例【全是这一条】：
    //       起 wave_buf.../DO[1]  终 cur_data_*_s1/D
    //       数据延迟 20.1~20.7ns > 周期 20ns，slack -0.19 ~ -0.82ns。
    //   （V40 那条其实也只有 +0.003ns，一直贴在约束线上。）
    //
    //   ★ 为什么插在这里就一定安全：
    //     wave_y0/y1 只由【这一列的 wave_col】决定，而 wave_col 是
    //     lcd_master 在 S_WAVE_DATA 末尾（一列 360 个像素全画完之后）才 +1 的，
    //     之后 state 先跳到 S_WAVE_WIN 把整条窗口命令发完（WIN_COUNT=11 个字节，
    //     88 拍）才开始读第一个像素。
    //     而 wave_y0 在 wave_col 变化后 3 拍内就稳定（BSRAM 输出 1 拍 +
    //     v0_sc_r 1 拍 + wave_y0_r 1 拍），远早于第一次使用 ——
    //     所以【显示上完全看不出来】。一整列约 5760 拍，3 拍占 0.05%。
    //   ★ 改的只是"什么时候算好"，不是"算什么"：
    //     下游（wave_seg_lo/hi、wave_draw_seg、wave_pix_hi/lo）一个字没动，
    //     波形的"哪一列用哪两个采样点"逐位不变。
    //   ★ 和上面 peak_msb_r 是同一个套路（V17c 那次把 22.5ns 从中间切断）。
    //     这条现在是全工程 CLKOUT1 域唯一的最长路径。
    //==================================================================
    reg [15:0] wave_y0_r, wave_y1_r;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            wave_y0_r <= 16'd133;      // 中线，和复位后的显示一致
            wave_y1_r <= 16'd133;
        end else begin
            wave_y0_r <= wave_y0_c;
            wave_y1_r <= wave_y1_c;
        end
    end
    wire [15:0] wave_y0 = wave_y0_r;
    wire [15:0] wave_y1 = wave_y1_r;

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
    // ★★ 李萨如（X-Y 示波器）★★
    //
    //   映射（满量程 ±32767 映射到 ±140 / ±100 像素）：
    //       col = 160 + (X * 140) >> 15      -> 20 ~ 300
    //       row = 120 - (Y * 100) >> 15      -> 20 ~ 220
    //
    //   ★ 乘法器一个都不许加（DSP 已经占了 54.5/118，相位档还要用掉 1 个），
    //     所以把常数拆成移位相加：
    //       140 = 128 + 8 + 4        100 = 64 + 32 + 4
    //     24 位有符号上下文里最大 32768 × 140 = 4,587,520 < 2^23，不会溢出。
    //   ★ 右移 15 位也【不写 ">>>"】：本工程在 V17c 踩过"有符号被无符号比较
    //     吃掉"的坑（见上面那段长注释），所以这里显式取符号位 + 高 8 位，
    //     算术右移的语义一眼可见，不依赖任何有符号右移规则。
    //   ★ 夹取在数学上永远用不到（|dx|<=140、|dy|<=100 是硬上界），但留着当
    //     保险：万一以后采样位宽变了，也不会画到窗口外面去。
    //
    //   ★★ 三屏李萨如的 XY 来源选择（V36 从 2 选 1 扩成 3 选 1）★★
    //     几路采样是同一个时刻的、又共用同一套映射，所以"选哪一屏"只是在
    //     这里插一级 3 选 1 的线，后面的移位相加、打点、余辉、边框一行都不用改。
    //       liss_src_s2 = 0（档 2，V36 新加）：X = 左声道 wave_in_l，Y = 右声道 wave_in_r
    //       liss_src_s2 = 1（档 3）：X = 干声 wave_in_x，Y = 输出 wave_in
    //       liss_src_s2 = 2（档 4）：X = 输出 wave_in，Y = 移相输出 wave_in_py
    //     ★ 切换时【不清显存】（无缝）：三屏的坐标范围、映射完全相同，旧轨迹
    //       会在 LISS_DECAY_DIV=1 的余辉里 3~4 帧（约 0.3~0.4s）自己淡掉，
    //       强行清一次反而会黑一下。要清也只需要把 liss_cleared 清 0，
    //       但没必要 —— 清了还得再等 1.5ms 的整片写 0，白闪一下。
    //     ★ liss_src 也是人手按的慢信号，和 liss_on / wave_mode 一样两级同步。
    //       它只影响这一级的 3 选 1，不会去动状态机，所以切换格外安全。
    // ============================================================
    reg liss_s1, liss_s2;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            liss_s1 <= 1'b0;
            liss_s2 <= 1'b0;
        end else begin
            liss_s1 <= liss_on;      // 人手按的慢信号，两级同步足够
            liss_s2 <= liss_s1;
        end
    end

    reg [1:0] liss_src_s1, liss_src_s2;
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            liss_src_s1 <= 2'd0;
            liss_src_s2 <= 2'd0;
        end else begin
            liss_src_s1 <= liss_src;   // 同上：人手按的慢信号，两级足够
            liss_src_s2 <= liss_src_s1;
        end
    end

    // ★ 3 选 1：选完的名字就叫 liss_x / liss_y，下面的映射只认这两个。
    //   （映射、打点、余辉、边框、十字全都不用改 —— 还是原来那一套。）
    wire signed [15:0] liss_x = (liss_src_s2 == 2'd0) ? wave_in_l :
                                (liss_src_s2 == 2'd1) ? wave_in_x : wave_in;
    wire signed [15:0] liss_y = (liss_src_s2 == 2'd0) ? wave_in_r :
                                (liss_src_s2 == 2'd1) ? wave_in   : wave_in_py;

    wire signed [23:0] xs24 = {{8{liss_x[15]}}, liss_x};
    wire signed [23:0] ys24 = {{8{liss_y[15]}}, liss_y};
    // ★★ V35.1：X 和 Y 的刻度必须【相等】，否则正圆会被画成椭圆 ★★
    //   原来 X 用 ×140、Y 用 ×100 —— 于是信号空间里的正圆（相移正好 90°、
    //   幅度相等）在屏上变成短/长轴 100/140 = 0.714 的椭圆，看着"不像圆"。
    //   这不是相移不够，是映射系数没取一致（上一版误当成"映射本身的性质"）。
    //   现在两边都是 ×120：屏幕上 ±120 像素，屏高 240 —— 上下正好顶到边，
    //   圆就是圆。（再大就出屏了；想缩一圈把两个 120 一起改小即可。）
    //   120 = 64 + 32 + 16 + 8（四个移位相加，不占 DSP）
    wire signed [23:0] x120 = (xs24 <<< 6) + (xs24 <<< 5) + (xs24 <<< 4) + (xs24 <<< 3);
    wire signed [23:0] y120 = (ys24 <<< 6) + (ys24 <<< 5) + (ys24 <<< 4) + (ys24 <<< 3);
    // 算术右移 15 位 = 取 bit23..bit15（bit23 就是符号位，切片里本来就带上了）
    // ★ 别写成 {x120[23], x120[23:15]} —— 那是 10 位：符号位重复了一次，
    //   赋给 9 位 wire 时【截掉低位】，实际变成了右移 16 位，图形幅度直接小一半。
    //   综合日志里就报 "Expression size 10 truncated to fit in target size 9"。
    wire signed [8:0]  liss_dx = x120[23:15];
    wire signed [8:0]  liss_dy = y120[23:15];
    wire signed [9:0]  liss_col_s = 10'sd160 + {{1{liss_dx[8]}}, liss_dx};
    wire signed [9:0]  liss_row_s = 10'sd120 - {{1{liss_dy[8]}}, liss_dy};
    wire [8:0] liss_col_c = (liss_col_s < 10'sd0)   ? 9'd0   :
                            (liss_col_s > 10'sd319) ? 9'd319 : liss_col_s[8:0];
    wire [7:0] liss_row_c = (liss_row_s < 10'sd0)   ? 8'd0   :
                            (liss_row_s > 10'sd239) ? 8'd239 : liss_row_s[7:0];

    // ---- 打点请求 ----
    //   显存这会儿被推屏占着，所以先存下来，等扫描走到【像素边界】再插进去。
    //   一个像素要 69 拍（1.38µs），采样 20.5µs 一个 —— 每个点都打得进。
    //
    //   ★ 握手用【翻转沿对】（new / done），不用"取走脉冲"：
    //     脉冲方案里 pend 由采集侧清、脉冲由状态机发，两个 always 块隔一拍，
    //     正好来新采样时会把新点误清掉。翻转沿对账天生没有这个缝：
    //       采集侧：来采样就翻 new，并把地址写进 liss_pt_addr
    //       状态机：打完了就把 done 跟到 new
    //       有待打点 = (new != done)
    //     两边各写各的寄存器，谁也不清谁的东西。
    reg        liss_pt_new;        // 采集侧翻转（来一个采样翻一次）
    reg        liss_pt_done;       // 状态机翻转（打完一个跟一次）
    reg [16:0] liss_pt_addr;       // 采集侧写，状态机读
    wire liss_pt_pend = (liss_pt_new != liss_pt_done);

    wire liss_sample_we = (wave_t2 ^ wave_t3) & liss_s2;

    // row*320 = row*256 + row*64；列直接加。全是移位相加，不占乘法器。
    // 地址 = row*320 + col，两个字宽 17 位（最大 239*320+319 = 76799）。
    wire [16:0] liss_pt_r320 = {liss_row_c, 8'b0} + {liss_row_c, 6'b0};

    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            liss_pt_new  <= 1'b0;
            liss_pt_addr <= 17'd0;
        end else if (liss_sample_we) begin
            liss_pt_addr <= liss_pt_r320 + {8'd0, liss_col_c};
            liss_pt_new  <= ~liss_pt_new;
        end
    end

    // ============================================================
    // ★ 李萨如显存：320×240、2bit/像素（4 档灰做余辉）= 76800 个字
    //
    //   ★★ 为什么是"一个字一个像素"（2bit × 76800），不是"4 像素挤一个字节" ★★
    //     一开始按 8bit × 19200（一字节 4 像素）写，综合出来【16 块 BSRAM】，
    //     153600 bit 只装了 294912 bit 的容量的一半。原因：GowinSynthesis 是
    //     【按深度贪心】选模式的 —— 它优先用 16K×1 那个最深档，于是
    //        8bit： ceil(19200/16384)=2 深 × 8 宽 = 16 块
    //        2bit： ceil(76800/16384)=5 深 × 2 宽 = 10 块   ← 少 6 块
    //     实测（同样 153600 bit，单独综合一次看 BSRAM NUMBER）：
    //        8bit×19200 -> 16 块    4bit×38400 -> 12 块    2bit×76800 -> 10 块
    //        1bit×153600 -> 10 块   10bit×15360 -> 10 块   16bit×9600 -> 16 块
    //     顺带一个大好处：整个字就是一个像素以后，打点【不用读改写了】，
    //     直接写 2'b11 就行（原来要先读出那个字节、改 2bit 再写回）。
    //
    //   推断成 BSRAM 的写法（和本文件 wave_buf 一个套路）：
    //     · 数组规整，只有【一个写口 + 一个读口】；
    //     · 读口写同步读 rd <= mem[ra]，组合读会被展开成 76800:1 选择器；
    //     · 写口另起一个 always 块。
    //   ★ 踩过的坑：变址写的寄存器数组（多个写地址 / 位选写）会退化成触发器，
    //     153600 bit 就是 15 万个 FF，器件直接撑爆。
    // ============================================================
    reg [1:0]  liss_fb [0:76799];
    reg [1:0]  liss_fb_rd;
    reg [16:0] liss_fb_ra, liss_fb_wa;
    reg [1:0]  liss_fb_wd;
    reg        liss_fb_we;

    always @(posedge sys_clk) begin
        if (liss_fb_we) liss_fb[liss_fb_wa] <= liss_fb_wd;   // 写口
    end

    always @(posedge sys_clk) begin
        liss_fb_rd <= liss_fb[liss_fb_ra];                   // 读口（同步读）
    end

    // ---- 扫描/推屏的过程量 ----
    reg [7:0]  liss_row;      // 正在处理的行 0~239
    reg [8:0]  liss_col;      // 正在处理的列 0~319
    reg        liss_pix;      // 本像素的第几个 SPI 字节 0=高字节 1=低字节
    reg [2:0]  liss_hs;       // S_LISS_HOUSE 的微步骤
    reg [16:0] liss_clr_i;    // 进模式清显存的计数器
    reg        liss_cleared;  // 本次进入已经清过显存
    reg        liss_on_d;     // liss_s2 的上一次值，用于检测模式切换
    reg [3:0]  liss_dcnt;     // 余辉分频计数
    reg [15:0] liss_c0;       // 本像素的 RGB565

    wire liss_decay_en = (liss_dcnt == 4'd0);

    wire [16:0] liss_row320 = {liss_row, 8'b0} + {liss_row, 6'b0};    // row*320
    wire [16:0] liss_scan_a = liss_row320 + {8'd0, liss_col};

    // 减一档（0 不动）。写成 v - |v|：0-0=0、1-1=0、2-1=1、3-1=2，一行搞定。
    wire [1:0] liss_dec = liss_fb_rd - {1'b0, |liss_fb_rd};

    // ---- 边框 + 中心十字：推屏时按坐标叠加 ----
    //   ★ 为什么不写进显存：写进去就得跟着余辉一起衰减（每帧还得重画一遍，
    //     白占显存端口），而且会被"减到 0 再重画"闪一下。叠加在推屏这一级，
    //     网格永远满亮度、不参与衰减，显存里还是干净的 4 档信号。
    wire liss_ovr = (liss_row == 8'd0) || (liss_row == LISS_ROW_LAST) ||
                    (liss_row == LISS_ROW_CROSS) ||
                    (liss_col == 9'd0) || (liss_col == LISS_COL_LAST) ||
                    (liss_col == LISS_COL_CROSS);

    function [15:0] liss_gray;
        input [1:0] v;
        begin
            case (v)
                2'd0:    liss_gray = COLOR_L0;
                2'd1:    liss_gray = COLOR_L1;
                2'd2:    liss_gray = COLOR_L2;
                default: liss_gray = COLOR_L3;
            endcase
        end
    endfunction

    // ★ 只有显存是 0 的地方才铺网格 —— 有信号的像素优先，别被网格盖掉。
    //   （暗灰的网格压在亮绿轨迹上会把轨迹切断，看着像断线。）
    wire [15:0] liss_lc = ((liss_fb_rd == 2'd0) && liss_ovr) ? COLOR_FRAME
                                                            : liss_gray(liss_fb_rd);

    // 本像素的两个 SPI 字节：先高后低
    wire [7:0] liss_byte = liss_pix ? liss_c0[7:0] : liss_c0[15:8];

    // ============================================================
    // ★★★ V47.2：LCD 刷屏看门狗 ★★★
    // ============================================================
    //   用户 2026-10-11 报："LCD 有时卡死定住不动"，而且
    //   【同时连续按下多个琴键】就会出现；卡死时蓝牙/UART 还能用
    //   -> clk_50m_ext 域是活的，是这台状态机卡住了（或模组自己挂了）。
    //
    //   ★ 为什么用看门狗而不是去堵具体那个状态：
    //     lcd_master.v 主状态机里嵌了 13 处 case(phase)
    //     （1217/1251/1276/1304/1330/1357/1433/1483/1516/1650/1684/1763/1839），
    //     子阶段一旦走到未定义编码、或者某个推进条件被连续变化的数据错过，
    //     那个 case 就什么都不做 -> state 不变 -> 永远停在那里。
    //     看门狗【不管根因是什么都能自愈】，而且正常工作【一次都不会触发】。
    //
    //   判据：state 连续 LCD_WD_LIMIT 个时钟没有变化 = 卡住了。
    //   ★ 阈值要大于"某个状态合法停留的最长时间"。本文件里最长的合法等待是
    //     那个 sec_cnt 的 ~250ms，所以取 0.8s（40,000,000 @ 50MHz）留足余量。
    //   ★ 正常刷屏时状态推进得很频繁，计数器一直被清零，永远不会溢出。
    //   ★ 触发后回到 S_POWER_ON（不是直接 S_INIT）—— 会连 PWR_DELAY 一起重走，
    //     对模组更稳妥。
    //
    //   ================================================================
    //   ★★★ V48.4：上面那句"正常工作一次都不会触发"是【错的】，用户实测抓到了 ★★★
    //   ================================================================
    //   用户报："LCD 不会卡死了，但是**延音时谱线会被刷掉**，
    //            即延音没办法保持谱线，这个看门狗把延音状态当卡死给刷掉了"。
    //
    //   【根因 1：判据漏了一种"合法的原地踏步"】
    //     频谱这一屏的状态机是 `S_BAR_WIN <-> S_BAR_DATA` 自循环的，
    //     而"这一列要不要重画"由 `bar_dirty` 决定：
    //       · 有列脏  -> 每画完一列都在 S_BAR_WIN/S_BAR_DATA 之间跳 -> state 一直在变；
    //       · 没有列脏（**高度不变**：延音锁住、或干脆没弹） ->
    //         S_BAR_WIN 的 phase0 只做两件事：`play_idx` 自增 + `phase <= 0`，
    //         **state 一直是 S_BAR_WIN，一个时钟都不变**。
    //     于是一屏静态画面超过 0.8s，看门狗就误判成卡死 —— 而"延音"正是
    //     "高度恒定不变"的典型场景，所以用户只在延音时看见它。
    //     ★ 也就是说：**"没活干"和"卡死"在 state 这一维上是同一种形状**，
    //       只看 state 的看门狗必然把前者当后者。
    //   【根因 2：恢复路径不作废谱线缓存】（与 V48 那个"切模式后频谱空白"同源）
    //     误判之后走 S_POWER_ON -> S_INIT -> S_CLEAR 全屏清一遍，
    //     但 `bar_state[]` 里存的还是清屏前的高度，**和当前目标高度一模一样**
    //     -> `bar_dirty` 恒 0 -> **一列都不重画** -> 屏幕就空在那里
    //     （高度不变时永远等不到"重画"）。这就是用户看到的"谱线被刷掉"。
    //
    //   【修法】两条一起做，缺一条都不行：
    //     ① 把"谱线在空转"也算成活着：**play_idx 变了就清计数器**。
    //        空转一圈只要几微秒（60 列 × 约 6 拍），所以计数器再也攒不到 0.8s；
    //        而真正的卡死里 play_idx 是冻住的，照样会触发 —— 看门狗的本职没丢。
    //     ② 看门狗一旦真的触发，**同时把 bar_state 整片作废**（写哨兵 9'h1FF），
    //        这样恢复之后每一列都会重画一遍（哨兵值靠"画完写回真实值"逐列消耗）。
    //        ★ 这是 V48 那三个作废点之外的**第 4 个作废点** ——
    //          凡是"清了屏/重开一屏"的地方，都必须作废这个缓存。
    //   ================================================================
    localparam [25:0] LCD_WD_LIMIT = 26'd40000000;   // 0.8s @ 50MHz
    reg [25:0] lcd_wd_cnt;
    reg [4:0]  lcd_state_d;
    reg [5:0]  lcd_play_d;                            // ★ V48.4：谱线空转的活性位
    // ================================================================
    // ★★★ V49.2：上电等待也要算"活着" —— 修掉手动重置（0xD1）卡死 ★★★
    // ================================================================
    //   用户实测："按【重置LCD】之后 LCD 卡死不动"。
    //   根因（两个常数撞车，纯时序算术）：
    //     · 手动重置在 S_POWER_ON 里等 PWR_DELAY_JOG = 50,000,000 拍 = 1.0s
    //     · 而看门狗窗口 LCD_WD_LIMIT = 40,000,000 拍 = 0.8s
    //   而活性判据只看 `state` 和 `play_idx` —— **这两者在 S_POWER_ON 里都是冻的**
    //   （整机就停在那儿等），于是计数器照跑：
    //       0.8s 时 lcd_wd_ovf=1 -> 走看门狗分支 (state<=S_POWER_ON, delay_cnt<=0)
    //       而 0xD1 的 lcd_rst_evt 早在 1 拍后就没了（它是脉冲），拦不住
    //     -> 1.0s 的等待永远走不完，每 0.8s 被打断一次 -> **死循环，永远到不了
    //        S_INIT** -> 屏停在没重新初始化的状态 = 用户看到的"卡死不动"。
    //   ★ 为什么正常开机没事：PWR_DELAY 只有 0.2s < 0.8s，等待总能在窗口内走完。
    //   ★ 为什么现在才发现：V48.5 加这条通道时没上板验过（1.0s > 0.8s 这个冲突
    //     从那天起就潜伏着），用户这次是第一次真按这个键。
    //   【修法】把"上电等待计数在推进"也算成活性：
    //     `delay_cnt` 只在 S_POWER_ON 里自增，所以
    //       · 等待期间它每拍都在变 -> 看门狗不计数 -> 1.0s 能安然走完；
    //       · 而真正的卡死（状态机停住、计数也冻住）它就不变 -> 看门狗照常触发，
    //         本职一点没丢（S_POWER_ON 是【有界等待】，上限就是 pwr_delay_use）。
    //   ★ 不用"把 1.0s 改成 <0.8s"这种修法：那会悄悄把两个常数耦死，
    //     下次谁再调其中一个又会复发；这里是从判据上根治。
    // ================================================================
    reg [25:0] lcd_delay_d;                           // ★ V49.2：上电等待的活性位
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_wd_cnt  <= 26'd0;
            lcd_state_d <= 5'd0;
            lcd_play_d  <= 6'd0;
            lcd_delay_d <= 26'd0;
        end else begin
            lcd_state_d <= state;
            lcd_play_d  <= play_idx[5:0];
            lcd_delay_d <= delay_cnt;
            // ★ V48.4：state 动了【或者】谱线列号动了 -> 都算"活着"
            // ★ V49.2：再加上"上电等待计数在动"（手动重置的 1.0s 等待靠它）
            if ((state != lcd_state_d) || (play_idx[5:0] != lcd_play_d)
                || (delay_cnt != lcd_delay_d))
                lcd_wd_cnt <= 26'd0;
            else if (lcd_wd_cnt != LCD_WD_LIMIT) lcd_wd_cnt <= lcd_wd_cnt + 1'b1;
        end
    end
    wire lcd_wd_ovf = (lcd_wd_cnt == LCD_WD_LIMIT);

    // ============================================================
    // ★★★ V48.5：软件"手动重置 LCD"（0xD1）★★★
    // ============================================================
    //   用户报："**还是容易在数据量多时卡死**，虽然在软件上各个按键状态同步
    //   没问题，但是就是 **LCD 卡住不动**"，而且实测三条：
    //     ① 卡住时**连换显示模式画面都不变**；
    //     ② 屏幕**完全定住**；
    //     ③ **看门狗毫无动作**（不闪、不清）。
    //   ★★ 这三条合起来说明：**FPGA 的状态机是活的，是【屏自己失步了】**
    //      （排线干扰 / 被截断的命令 / 掉电抖动 —— 密集按键时 36 颗 LED
    //       一起亮，电流和噪声都上去，最容易出这种事）。
    //   ★★ 而**看门狗救不了这一类**：它只看状态机（`state`/`play_idx`），
    //      **看不到屏**；而"屏失步"和"画面本来就是静态的"在没有反馈的情况下
    //      **长得一模一样** —— V48.4 刚把"合法的静态"从误触发里摘出来，
    //      绝不能再按时间瞎重置（那就退回"每 0.8 秒刷一次屏"的老毛病）。
    //      所以这一类只能给**人来按**的逃生通道。
    //   【这条通道做什么】= 把"开机那套"重跑一遍：
    //     S_POWER_ON（★ 用更长的等待，见下）-> S_INIT（重发整张初始化表，
    //     表尾是 SLPOUT + DISPON，足以把被假命令搞乱的屏救回来）->
    //     S_IDLE -> 全屏清 -> 重画。
    //   ★ 为什么等待要【比开机更长】：屏如果是掉电/干扰失了步，给它久一点
    //     稳定时间是值得的（开机的 PWR_DELAY 只有 0.2s）。这里用 1.0s。
    //   ★ 顺手把 `bar_bad` 整片置 1（和看门狗那条一样）—— 否则清完屏
    //     一列都不会重画（这正是 V48.4 修的那个坑，不能在新路径上再犯一次）。
    //   ★ 优先级【高于看门狗】：手动请求是用户明确要求的动作，先满足它。
    // ============================================================
    localparam [25:0] PWR_DELAY_JOG = 26'd50_000_000;   // 手动重置：1.0s @50MHz
    reg [1:0] lcd_rst_s;
    wire lcd_rst_evt = lcd_rst_s[1] ^ lcd_rst_s[0];     // 还原成 1 拍脉冲
    reg       lcd_rst_long;                 // 本次 S_POWER_ON 用长等待
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_rst_s    <= 2'b00;
            lcd_rst_long <= 1'b0;
        end else begin
            lcd_rst_s <= {lcd_rst_s[0], lcd_rst_tgl};
            if (lcd_rst_evt)                 lcd_rst_long <= 1'b1;  // 请求 -> 长等待
            else if (state != S_POWER_ON)    lcd_rst_long <= 1'b0;  // 离开上电态 -> 复原
        end
    end
    // S_POWER_ON 里真正用的等待时长（手动重置那一次用长的）
    wire [25:0] pwr_delay_use = lcd_rst_long ? PWR_DELAY_JOG : {2'b00, PWR_DELAY};

    // ============================================================
    // 主状态机
    // ============================================================
    always @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_POWER_ON;
            delay_cnt   <= 26'd0;
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
            bar_bad     <= 60'd0;

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

            // ★ 李萨如复位（显存 liss_fb 是 BSRAM，不上电初始化，
            //   所以进模式时靠 S_LISS_CLR 整片写 0，见那段注释）
            liss_row     <= 8'd0;
            liss_col     <= 9'd0;
            liss_pix     <= 1'b0;
            liss_hs      <= 3'd0;
            liss_clr_i   <= 17'd0;
            liss_cleared <= 1'b0;
            liss_on_d    <= 1'b0;
            liss_dcnt    <= 4'd0;
            liss_fb_ra   <= 17'd0;
            liss_fb_wa   <= 17'd0;
            liss_fb_wd   <= 2'd0;
            liss_fb_we   <= 1'b0;
            liss_c0      <= 16'h0000;
            liss_pt_done <= 1'b0;
        end
        // ★★★ V48.5：软件手动重置（0xD1）—— **优先级最高** ★★★
        //   ★ 放在看门狗【前面】：这是用户明确按下的动作，先满足它。
        //   ★ 做的事和开机完全一样（S_POWER_ON -> S_INIT -> 全屏清 -> 重画），
        //     区别只有一处：**上电等待用 1.0s**（给失步/掉电抖动的屏
        //     更长的稳定时间），见 pwr_delay_use。
        //   ★ `bar_bad` 整片置 1：清完屏要让 60 列各重画一次
        //     （不做的话就是 V48.4 那个"清完一列都不画"的坑）。
        else if (lcd_rst_evt) begin
            state       <= S_POWER_ON;
            delay_cnt   <= 26'd0;
            send_idx    <= 32'd0;
            bit_idx     <= 3'd0;
            phase       <= 2'd0;
            sck_div_cnt <= 8'd0;
            lcd_cs      <= 1'b1;      // 干净地放开片选，让模组从头开始
            lcd_sck     <= 1'b0;
            lcd_dc      <= 1'b0;
            bar_bad     <= {60{1'b1}};
        end
        // ★★★ V47.2：看门狗优先于 case —— 卡死了就整台重来 ★★★
        //   ★ 这是【纯附加】的：正常刷屏时 lcd_wd_ovf 恒为 0，
        //     这条分支一次都不会走到，下面的 case 逐位不变。
        //   ★★ V48.4：触发时【必须顺手作废谱线缓存】——
        //     否则全屏清完之后 bar_dirty 恒为 0，一列都不重画，
        //     屏幕就空在那里（用户报的"延音时谱线被刷掉"就是这条）。
        else if (lcd_wd_ovf) begin
            state       <= S_POWER_ON;
            delay_cnt   <= 26'd0;
            send_idx    <= 32'd0;
            bit_idx     <= 3'd0;
            phase       <= 2'd0;
            sck_div_cnt <= 8'd0;
            lcd_cs      <= 1'b1;      // 松开片选，让模组看到一次干净的重新开始
            lcd_sck     <= 1'b0;
            lcd_dc      <= 1'b0;
            // ★★ V48.4：整片作废 60 列（每列一位；画完那一列会自动清掉那一位）
            //   为什么单独一位而不是再写一遍 9'h1FF：省 1023 个 LUT，
            //   语义完全一样（见 bar_state 声明处与 bar_dirty 那两段注释）。
            bar_bad <= {60{1'b1}};
        end
        else begin
            // ★★ V48：这里原来有 `if (bar_force_clr) bar_force <= 1'b0;` ——
            //   改成"作废 bar_state"之后不再需要任何强制标志/计时器，
            //   所以这一段整个删掉了。切模式的作废在各触发点就地做。
            case (state)

                S_POWER_ON: begin
                    lcd_cs  <= 1'b1;
                    lcd_sck <= 1'b0;
                    lcd_blk <= 1'b1;
                    // ★ V48.5：等待时长看 pwr_delay_use ——
                    //   正常上电/看门狗恢复用 PWR_DELAY（0.2s）；
                    //   用户按了"重置 LCD"（0xD1）那一次用 PWR_DELAY_JOG（1.0s）。
                    if (delay_cnt >= pwr_delay_use) begin
                        delay_cnt   <= 26'd0;
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
                                    // ★★ V48 作废点①：切模式了 -> 脏缓存整片作废
                                    //   （不清的话清屏之后每列高度和 bar_state 里
                                    //     的缓存一样，bar_dirty 恒为 0，一根都不重画）
                                    for (dm_i = 0; dm_i < 60; dm_i = dm_i + 1)
                                        bar_state[dm_i] <= 9'h1FF;
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
                                        bar_bad[play_idx[5:0]]  <= 1'b0;   // ★ V48.4：画完就消耗掉这一位
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

                    // ★★ 李萨如优先（按钮 3 循环到第 3/4/5 屏时 liss_s2=1）★★
                    //   开着就画李萨如、压过 wave_mode 的选择；关掉回 wave_mode
                    //   指的那一屏 —— wave_mode 的原语义（0=频谱 1=波形）一个字没动。
                    //   ★ 三屏李萨如之间的切换（只翻 liss_src，liss_s2 不变）
                    //     【故意不进这个分支】：三屏共用显存和映射，不清屏、
                    //     不重进状态机，旧轨迹靠余辉自己淡掉 = 无缝切换。
                    if (liss_s2 != liss_on_d) begin
                        liss_on_d    <= liss_s2;
                        bar_pix_cnt  <= 0;
                        liss_cleared <= 1'b0;
                        // ★★ V48 作废点②：★★★ 用户报的那条路就是这一条 ★★★
                        //   频谱 -> … -> 李萨如 -> … -> 频谱 时，wave_mode 全程为 0
                        //   （频谱和 3 档李萨如的 wave_mode 都是 0），所以永远进不了
                        //   作废点① —— 必须在"进出李萨如"这里也把频谱缓存作废，
                        //   否则退出李萨如、全屏清完回到频谱时，bar_dirty 恒 0，
                        //   一根柱子都不画（用户实测："频谱被清空了，音还在响"）。
                        for (dm_i = 0; dm_i < 60; dm_i = dm_i + 1)
                            bar_state[dm_i] <= 9'h1FF;
                        // 进：这一拍先不动，下一拍进 S_IDLE_WAIT 走"清显存"那一支
                        // 出：整屏都被李萨如写满过（连 60 个数字一起盖掉了），
                        //     必须走原来的【全屏清 + 补数字】那条路（S_CLEAR_WIN
                        //     -> S_CLEAR -> S_DIGIT_WIN），不能只清主区。
                        if (!liss_s2) state <= S_CLEAR_WIN;
                    end
                    else if (liss_s2) begin
                        if (!liss_cleared) begin
                            liss_cleared <= 1'b1;
                            liss_clr_i   <= 17'd0;
                            state        <= S_LISS_CLR;
                        end else begin
                            liss_hs  <= 3'd0;
                            liss_pix <= 1'b0;
                            liss_row <= 8'd0;
                            liss_col <= 9'd0;
                            state    <= S_LISS_WIN;
                        end
                    end
                    // 检测模式切换
                    else if (wave_mode != wave_mode_d) begin
                        wave_mode_d <= wave_mode;
                        // 切模式：先清主区，然后重新开始画
                        bar_pix_cnt <= 0;
                        state       <= S_CLEAR_BAR;
                        // ★★ V48 作废点③：波形 -> 频谱 这条方向是在这里被发现的
                        //   （频谱时状态机在 S_BAR_WIN↔S_BAR_DATA 里自循环，不回
                        //     S_IDLE_WAIT，所以 频谱->波形 由作废点① 负责）。
                        for (dm_i = 0; dm_i < 60; dm_i = dm_i + 1)
                            bar_state[dm_i] <= 9'h1FF;
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

                // =================================================
                // ★★ 李萨如（X-Y 示波器）★★
                // =================================================

                // 进模式先整片把显存写 0：
                //   BSRAM 上电内容未知（不写就是雪花点），而且上次退出时里面
                //   还留着旧图。76800 拍 @50MHz = 1.5ms，比一帧 106ms 小两个
                //   数量级，随便花 —— 比起"边扫边清"还省掉了打点被误清的麻烦。
                S_LISS_CLR: begin
                    liss_fb_we <= 1'b1;
                    liss_fb_wa <= liss_clr_i;
                    liss_fb_wd <= 2'b00;
                    liss_clr_i <= liss_clr_i + 17'd1;
                    // ★ 判据写 >= LISS_WORDS（不是 WORDS-1）：最后一个字的写
                    //   必须原样发出去，多跑一拍才是"收尾"那一拍 —— 写成
                    //   WORDS-1 的话最后一拍里 we 会被后面的赋值改回 0，
                    //   第 76799 个字永远清不掉。
                    if (liss_clr_i >= LISS_WORDS) begin
                        liss_fb_we <= 1'b0;
                        liss_fb_wa <= 17'd0;
                        liss_clr_i <= 17'd0;
                        liss_hs    <= 3'd0;
                        liss_pix   <= 1'b0;
                        liss_row   <= 8'd0;
                        liss_col   <= 9'd0;
                        state      <= S_LISS_WIN;
                    end
                end

                // 整屏窗口命令（复用 tbl_out 里 S_CLEAR_WIN 那张表，字节完全一样）
                S_LISS_WIN: begin
                    if (sck_div_cnt >= SCK_DIV_LISS) begin
                        sck_div_cnt <= 8'd0;
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
                                        liss_hs  <= 3'd0;
                                        liss_pix <= 1'b0;
                                        state    <= S_LISS_HOUSE;
                                    end else
                                        send_idx <= send_idx + 1;
                                end else
                                    bit_idx <= bit_idx + 1;
                                phase <= 2'd0;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                // 一个像素的显存维护：5 拍干完
                //   hs0 发扫描读地址
                //   hs1 等同步读回来（BSRAM 读天生打一拍，这拍不能省）
                //   hs2 数据到 -> ①锁本像素的推屏颜色（按【当前值】取，
                //       余辉层次就是它）②把"减一档"写回同一个地址
                //   hs3 撤销衰减写；有待打的点就【直接写满 3】
                //       —— 一个字就是一个像素，不用先读出来改
                //   hs4 撤销写 -> 去推这 2 个 SPI 字节
                //   ★ 衰减写落在 hs2、打点写落在 hs3，两个不同拍，绝不抢写口；
                //     点正好落在正在扫的那个像素上时打点写在后，赢的是打点，
                //     所以那一帧这一点的余晖是满的 —— 正是我们要的。
                S_LISS_HOUSE: begin
                    case (liss_hs)
                        3'd0: begin
                            liss_fb_ra <= liss_scan_a;
                            liss_hs    <= 3'd1;
                        end
                        3'd1: begin
                            liss_hs <= 3'd2;
                        end
                        3'd2: begin
                            liss_c0    <= liss_lc;
                            // ★ 推屏和衰减共用这一遍扫描，不再跑第二遍显存
                            liss_fb_wa <= liss_scan_a;
                            liss_fb_wd <= liss_dec;
                            liss_fb_we <= liss_decay_en;
                            liss_pix   <= 1'b0;
                            liss_hs    <= 3'd3;
                        end
                        3'd3: begin
                            liss_fb_we <= 1'b0;
                            if (liss_pt_pend) begin
                                // 打点：该像素写满 3（整个字就是这个像素）
                                liss_fb_wa   <= liss_pt_addr;
                                liss_fb_wd   <= 2'b11;
                                liss_fb_we   <= 1'b1;
                                liss_pt_done <= liss_pt_new;
                            end
                            liss_hs <= 3'd4;
                        end
                        default: begin
                            liss_fb_we <= 1'b0;
                            liss_hs    <= 3'd0;
                            state      <= S_LISS_PUSH;
                        end
                    endcase
                end

                // 本像素 = 2 个 SPI 字节（RGB565 先高后低），SPI 位时序和其它模式
                // 一样（phase/bit_idx 那套），只有分频换成李萨如专用的。
                S_LISS_PUSH: begin
                    if (sck_div_cnt >= SCK_DIV_LISS) begin
                        sck_div_cnt <= 8'd0;
                        case (phase)
                            2'd0: begin
                                lcd_sck  <= 0;
                                cur_data <= liss_byte;
                                cur_dc   <= 1'b1;
                                lcd_dc   <= 1'b1;
                                phase    <= 2'd1;
                            end
                            2'd1: begin lcd_sck <= 0; lcd_mosi <= cur_data[3'd7 - bit_idx]; phase <= 2'd2; end
                            2'd2: begin lcd_sck <= 1; phase <= 2'd3; end
                            2'd3: begin
                                lcd_sck <= 1;
                                phase   <= 2'd0;
                                if (bit_idx == 3'd7) begin
                                    bit_idx <= 0;
                                    if (liss_pix == 1'b1) begin
                                        liss_pix <= 1'b0;
                                        // 一个像素的两个字节推完 -> 下一个像素
                                        if (liss_col == LISS_COL_LAST) begin
                                            liss_col <= 9'd0;
                                            if (liss_row == LISS_ROW_LAST) begin
                                                // 一帧结束：翻余辉分频，回分派
                                                liss_row  <= 8'd0;
                                                liss_dcnt <= (liss_dcnt + 4'd1 >= LISS_DECAY_DIV)
                                                             ? 4'd0 : (liss_dcnt + 4'd1);
                                                state     <= S_IDLE_WAIT;
                                            end else begin
                                                liss_row <= liss_row + 8'd1;
                                                state    <= S_LISS_HOUSE;
                                            end
                                        end else begin
                                            liss_col <= liss_col + 9'd1;
                                            state    <= S_LISS_HOUSE;
                                        end
                                    end else begin
                                        liss_pix <= 1'b1;
                                    end
                                end else
                                    bit_idx <= bit_idx + 1;
                            end
                        endcase
                    end else sck_div_cnt <= sck_div_cnt + 1;
                end

                default: state <= S_IDLE_WAIT;
            endcase
        end
    end

    // ★ 调试口还是 4 位：李萨如的 4 个新状态只体现在内部 state[4]
    assign state_out = state[3:0];

endmodule