`timescale 1ns / 1ps

//=============================================================================
// uart_packet_echo.v   ——  蓝牙乐谱帧接收器 + 播放器（协议 v2：和弦 / 速度 / 时值）
//
//   ★ 和弦为什么这么容易 ★
//     音频引擎（top.v）本来就是 36 路独立发声，`midi_keys` 就是 36 位掩码，
//     同时置多个位 = 同时响多个音。所以"和弦"这件事在板子上【本来就支持】，
//     缺的只是"协议怎么把同一时刻的多个音送进来"。
//
//   ★ 本版的核心简化 ★
//     旧版把【字节流】存进 BRAM，播放时再逐字节解析 → 想支持和弦就得写一个
//     "向后扫描到 0xC1"的状态机 + 多周期 BRAM 读，很啰嗦。
//     现在改成：【接收时就把和弦拼好】，每个时间格只存一条记录：
//         slot_mask[i] = 这一格要点亮哪些键（36 位掩码）
//         slot_dur[i]  = 这一格要占几个时间格（1..255）
//     于是播放器退化成一句 `midi_keys <= slot_mask[当前格]`，
//     原来的演奏结构（换格、按住、停播）几乎一行没动。
//
//   ---- 帧结构 ----
//     [0xFF × 16]  前导冲刷（可选；关窗状态下只被忽略，真正"停播+清空"靠 0xAA）
//     [0xAA]       开窗 + 写指针归零 + 停播 + 把设置复位成默认
//     [0xC4 + 3B]  设置时间格（µs，小端）      可选
//     [0xC5 + 3B]  设置释音提前量（µs，小端）  可选
//     [事件...]    单音 / 休止 / 和弦，各自可跟一个 0xC2
//     [0xFF]       帧尾：关窗 + 开播 + 回传一个 0xFF
//
//   ---- 事件字节 ----
//     单音   键号 0x00..0x23        键号 就是 top.v 里 delta[键号] 的下标
//     休止   0xEE                   这一格不发声
//     和弦   0xC0 音号{1..36} 0xC1  这一格里所有音【同时】发声
//     时值   0xC2 + 1 字节 N        紧跟在上一个事件之后：该事件共占 1+N 格
//
//   ---- 操作数不受控制字节解释 ----
//     0xC2 的 N、0xC4/0xC5 的 3 个参数，即使值恰好是 0xAA/0xFF/0xEE，
//     也一律当【原始数据】，由"待收操作数"计数器保证。
//
//   ★ 与 v1 的兼容 ★
//     不带任何新字节的 v1 帧 → 时间格取默认 500ms、释音提前 300ms、每事件 1 格
//     → 每个音响 0.2s、留白 0.3s，**与 v1 逐位等价**。
//
//   ★★★ 键号到底是什么（以 top.v 的 delta[] 表为准，别再按"MIDI−48"算）★★★
//
//     键号【不是】按音高排的，它是 top.v 里 delta[键号] 的下标。按下表：
//
//         键号 0 = A#2  (117Hz)   键号 12 = A#3  (233Hz)   键号 24 = A#4  (466Hz)
//         键号 1 = C3   (131Hz)   键号 13 = C4   (262Hz)   键号 25 = C5   (523Hz)
//         键号 2 = D#3  (156Hz)   键号 14 = D#4  (311Hz)   键号 26 = D#5  (622Hz)
//         键号 3 = F3   (175Hz)   键号 15 = F4   (349Hz)   键号 27 = F5   (698Hz)
//         键号 4 = G3   (196Hz)   键号 16 = G4   (392Hz)   键号 28 = G5   (784Hz)
//         键号 5 = A2   (110Hz)   键号 17 = A3   (220Hz)   键号 29 = A4   (440Hz)
//         键号 6 = B2   (123Hz)   键号 18 = B3   (247Hz)   键号 30 = B4   (494Hz)
//         键号 7 = C#3  (139Hz)   键号 19 = C#4  (277Hz)   键号 31 = C#5  (554Hz)
//         键号 8 = D3   (147Hz)   键号 20 = D4   (294Hz)   键号 32 = D5   (587Hz)
//         键号 9 = E3   (165Hz)   键号 21 = E4   (330Hz)   键号 33 = E5   (659Hz)
//         键号10 = F#3  (185Hz)   键号 22 = F#4  (370Hz)   键号 34 = F#5  (740Hz)
//         键号11 = G#3  (208Hz)   键号 23 = G#4  (415Hz)   键号 35 = G#5  (831Hz)
//
//     换算公式（上位机用）。一个八度里的接线顺序是：
//         先 5 个黑键 A# C D# F G，再 7 个白键 A B C# D E F# G#
//         —— 不是按音高从低到高，所以键号 13 才是 C4、键号 29 才是 A4。
//         PERM = [5,0,6,1,7,8,2,9,3,10,4,11]
//         键号 = PERM[(MIDI − 45) mod 12] + 12 × ⌊(MIDI − 45) / 12⌋
//     板子的音域 = MIDI 45(A2) .. 80(G#5)，共 36 键。
//     ★ 这条不是"约定"，是被 top.v 的 delta[] 表钉死的：例如 MIDI 69(A4,440Hz)
//       → 键号 29；这同时也是判断"发出去到底响没响对"的唯一标准。
//     ★ 上位机侧由 check_rtl_contract.py 拿 delta[] 自动核对，改错了会红。
//
//   ---- 演奏录音回传（v2.2 新增，方向与上面相反：板子 → 上位机）----
//     板子把"手弹了什么"回传上去，上位机合成 MIDI 并在电脑上放出来：
//       [0x5A] [mask 36 位·小端 5B] [dt 2B·小端，单位 10ms]   一个事件
//       [0x5B] [0/1]                                          开关状态
//       [0x5C]                                                上位机 → 板子：翻转开关
//     录音开关两端都能控制：板子【按钮板第 10 键】或上位机发 0x5C。
//     ★ mask 是 key_mask 的快照，好几位就是同时按着好几个键 —— 和弦天然支持。
//     ★ 录音期间【整体抑制回显】：回显字节是任意值，撞上 0x5A 会被上位机
//       当成帧头，凭空多出一个事件还把时间轴带偏。详见第 4b/4c 节的注释。
//=============================================================================
module uart_packet_echo (
    input  wire        clk,            // 50MHz（clk_50m_ext）
    input  wire        rst_n,
    input  wire        uart_rxd,       // 蓝牙模块 TXD -> FPGA
    output reg         uart_txd,       // FPGA -> 蓝牙模块 RXD

    //==================================================================
    // ★ 录音回传（新增）
    //
    //   录的是"板子打算让哪些键响"：top.v 的 sounding_mask_aud（门控）
    //   = key_mask | sustain_latch_aud。每 10ms 采一次，一变就回传一个事件帧：
    //       [0x5A] [mask 36 位，小端 5 字节] [dt 2 字节，单位 10ms]
    //   另外还把【效果开关】回传上去（变了才发，开始录音时强制发一次）：
    //       [0x5D] [8 位：滑音/延音/音色/颤音/延迟/笛子/混响/合唱]
    //   上位机靠它把效果在 MIDI 里近似还原。
    //   开关两端都能控制：
    //       · 板子上按【按钮板的第 10 键】
    //       · 上位机发一个 0x5C
    //   开关状态变化时回传 [0x5B] [0/1]，让上位机同步界面。
    //
    //   ★★ 为什么送"门控"，而不是"包络输出" sounding_mask = (env != 0) ★★
    //     包络输出带着 1.46 秒释音尾巴，看着更"实际"，但有两个致命问题：
    //       ① 尾巴没走完就重击同一个键，掩码连着不变 —— 两下会并成一个长音
    //          （实测：音高 62 弹两次 1000ms / 2600ms，合成了一个 3730ms 长音）；
    //       ② 掩码里一直为 1 的键会在【每个事件】上重复触发 ——
    //          8 个音弹出来是 32 次起音。
    //     门控信号的起音/放音/重击边沿才是真的，延音锁存也天然在里面
    //     （sounding_mask_aud 本身就 = key_mask | sustain_latch_aud）。
    //     释音尾巴改由上位机按 RTL 常数补（见 top.v 那段注释与
    //     tools\协议说明_v2_和弦扩展.md §9.2）。
    //
    //   ★ 录音期间【不回显】—— 回显的字节是任意值，会和 0x5A 帧头撞车。
    //     反正录音时上位机也不在发谱子，不回显不影响任何事。
    //==================================================================
    input  wire [35:0] gate_mask_in,    // 门控掩码 = 按键 | 延音锁存（已同步到本时钟域）
    input  wire [7:0]  fx_state_in,      // 影响声音的 8 个效果开关（已同步到本时钟域）
    input  wire        btn_rec,          // 录音按钮：单拍脉冲（已同步到本时钟域）

    // ---- 按键自定义映射的写口（给 top.v 的 key_map[] 用）----
    //   ★ 跨时钟域：top.v 那边用"翻转信号 + 边沿检测"接收（和 btn_rec 同一套做法），
    //     所以这里只保证 key/note 两个字节在翻转沿到来之前已经稳定。
    output reg         keymap_we_tgl,    // 每写一条完整映射翻转一次
    output reg  [5:0]  keymap_key,       // 物理键号 0..35
    output reg  [5:0]  keymap_note,      // 目标音符 0..35；>=36 表示禁用该键

    // ---- 效果开关 / 音色的写口（给 top.v 的效果寄存器用）----
    //   ★ 和上面 keymap 同一套跨时钟域做法：翻转信号 + 边沿检测，
    //     数据位在翻转沿到来之前保持稳定，接收侧在沿上采。
    output reg         fx_tgl,           // 每收到一条"翻转效果"命令翻转一次
    output reg  [2:0]  fx_bit,           // 翻转哪一位（0..7；5 表示音色循环）
    output reg         tone_we_tgl,      // 每收到一条"设定音色"命令翻转一次
    output reg  [1:0]  tone_val,         // 目标音色 0 普通 / 1 笛子 / 2 吉他
    //   ★ 当前音色（top.v 送来，用于回报）；本域里两级同步后再用
    input  wire [1:0]  tone_sel_in,

    // ---- ★ V32：当前正在播放的【事件八度偏移】（送给 top.v 的声部）----
    //   有符号 3 位：0 = 原调，+n = 升 n 个八度，-n = 降 n 个八度（-4..+3）。
    //   它跟着事件走：装载下一个事件时一起换，事件放完自动回 0。
    output reg  [2:0]  oct_cur,

    // ---- LCD 相关输出：top.v 里那条自动演奏链已停用，这里保持端口不变 ----
    output reg         score_we,
    output reg [7:0]   score_waddr,
    output reg [7:0]   score_wdata,
    output reg         play_start,
    output reg [7:0]   play_len,

    // ---- 发声：36 键掩码，直接 OR 进 top.v 的 key_mask ----
    output reg  [35:0] midi_keys,      // 当前时间格要点亮的键（和弦 = 多个位）
    output reg  [35:0] bt_keys         // 兼容端口，恒 0（保留是为了不动 top.v 的例化）
);

    //==================================================================
    // 参数
    //==================================================================
    localparam BAUD_DIV  = 16'd5208;      // 50MHz / 5208 = 9600
    localparam SLOT_MAX  = 1024;          // 最多 1024 个事件（一个事件 = 一个时间格组）
                                          //   252 事件的小步舞曲只用到 1/4，留足余量
    localparam [10:0] SLOT_MAXN = 11'd1024;

    // ---- 录音回传（全部落在"未分配字节区"，老固件收到只会静默丢弃）----
    localparam [7:0]  REC_EV_HEAD  = 8'h5A;   // FPGA -> 上位机：录音事件帧头
    localparam [7:0]  REC_ST_HEAD  = 8'h5B;   // FPGA -> 上位机：录音开关状态
    localparam [7:0]  REC_CMD_BYTE = 8'h5C;   // 上位机 -> FPGA：翻转录音开关
    localparam [7:0]  REC_FX_HEAD  = 8'h5D;   // FPGA -> 上位机：效果开关状态

    // ---- 演奏控制（上位机 -> FPGA，不受开窗限制，随时可用）----
    localparam [7:0]  KEYMAP_SET   = 8'hC6;   // 写一条按键->音符映射：+键号 +音符
    localparam [7:0]  STOP_BYTE    = 8'hC7;   // 停止演奏（★ 保留乐谱，可重播）
    localparam [7:0]  REPLAY_BYTE  = 8'hC8;   // 重播板内已存的那一帧
    // ---- ★ V31：效果开关 / 音色（上位机也能切，板上的 TTP 按钮照旧）----
    localparam [7:0]  FX_TGL_BYTE  = 8'hC9;   // 翻转一个效果开关：+位号 0..7
                                              //   位号和 fx_state 的位一一对应；
                                              //   位 5 = 音色（三档，等价于按按钮 7）
    localparam [7:0]  TONE_SET_BYTE= 8'hCA;   // 直接选音色：+0 普通 / 1 笛子 / 2 吉他
    localparam [7:0]  FX_REQ_BYTE  = 8'hCB;   // 请求立刻回报一次状态（不带操作数）
    // ---- ★ V32：事件八度偏移（用来拓宽蓝牙播放的音域）----
    //   板子只有 36 个声部（A2..G#5），超出范围的音原来一律【折八度】进来。
    //   现在上位机可以给某一格事件加一个八度偏移：这一格里的音整体升/降 n 个八度，
    //   于是"高两个八度的那句旋律"能按原八度弹出来，而不是被折回来。
    //     0xCC + 1 字节：有符号 3 位（0..3 = +0..+3；4..7 = -4..-1），只作用于【下一个事件】
    localparam [7:0]  OCT_BYTE     = 8'hCC;
    localparam [19:0] REC_TICK_DIV = 20'd500_000;   // 采样间隔 10ms @ 50MHz

    localparam [7:0] REST_BYTE  = 8'hEE;  // 休止符
    localparam [7:0] CLR_BYTE   = 8'hAA;  // 开窗 + 写指针归零 + 停播
    localparam [7:0] END_BYTE   = 8'hFF;  // 帧尾
    localparam [7:0] CHORD_ON   = 8'hC0;  // 和弦帧头
    localparam [7:0] CHORD_OFF  = 8'hC1;  // 和弦帧尾
    localparam [7:0] DUR_EXT    = 8'hC2;  // 时值延长：+1 字节 N
    localparam [7:0] SET_TICK   = 8'hC4;  // 时间格：+3 字节 µs
    localparam [7:0] SET_REL    = 8'hC5;  // 释音提前量：+3 字节 µs
    localparam [7:0] NOTE_MAX   = 8'h23;  // 最大键号 35

    // 默认设置：与 v1 完全一致（0.5s 一格、每事件末尾提前 0.3s 松键）
    localparam [23:0] TICK_DEF_US = 24'd500_000;
    localparam [23:0] REL_DEF_US  = 24'd300_000;

    // 50MHz：1µs = 50 个周期
    localparam [29:0] TICK_MIN_CYC = 30'd1_000_000;   // 时间格下限 20ms
    localparam [29:0] GAP_MIN_CYC  = 30'd500_000;     // 每事件末尾至少 10ms 静音
                                                      // （top.v 包络要求 gate 低满 8ms）

    //==================================================================
    // 1) 时间参数寄存器（0xC4 / 0xC5 可改；0xAA 时复位成默认）
    //==================================================================
    reg [23:0] tick_us_r, rel_us_r;

    // ×50 用移位加法，不用乘法器
    wire [29:0] tick_x50 = ({6'b0, tick_us_r} << 5)
                         + ({6'b0, tick_us_r} << 4)
                         + ({6'b0, tick_us_r} << 1);
    wire [29:0] rel_x50  = ({6'b0, rel_us_r}  << 5)
                         + ({6'b0, rel_us_r}  << 4)
                         + ({6'b0, rel_us_r}  << 1);

    wire [29:0] tick_cyc_n = (tick_x50 < TICK_MIN_CYC) ? TICK_MIN_CYC : tick_x50;

    reg  [29:0] tick_cyc, rel_cyc;
    wire [29:0] rel_max    = tick_cyc - GAP_MIN_CYC;     // 保证 gate_last >= 10ms
    wire [29:0] rel_cyc_n  = (rel_x50 > rel_max) ? rel_max : rel_x50;
    wire [29:0] gate_last  = tick_cyc - rel_cyc;         // 最后一个格里的发声时长

    // ★ 注意：tick_us_r / rel_us_r 由下面的【接收块】独占驱动（它负责复位默认值、
    //   响应 0xC4/0xC5、以及 0xAA 时复位）。这里只把 µs 换算成周期数寄存下来，
    //   绝不能再给 tick_us_r 赋值 —— 否则会报 EX2000 重复驱动。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cyc  <= 30'd25_000_000;                 // 500ms
            rel_cyc   <= 30'd15_000_000;                 // 300ms
        end else begin
            tick_cyc <= tick_cyc_n;
            rel_cyc  <= rel_cyc_n;
        end
    end

    //==================================================================
    // 2) UART 接收：起始位中点采样，8 数据位，无校验，1 停止位
    //==================================================================
    reg [1:0]  rx_sync;
    reg        rx_d;
    wire       rx_negedge = (rx_d == 1'b1) && (rx_sync[1] == 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
            rx_d    <= 1'b1;
        end else begin
            rx_sync <= {rx_sync[0], uart_rxd};
            rx_d    <= rx_sync[1];
        end
    end

    localparam RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;

    reg [1:0]  rx_state;
    reg [15:0] rx_baud_cnt;
    reg [2:0]  rx_bit_idx;
    reg [7:0]  rx_data_tmp;
    reg [7:0]  rx_data;
    reg        rx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state    <= RX_IDLE;
            rx_baud_cnt <= 16'd0;
            rx_bit_idx  <= 3'd0;
            rx_data_tmp <= 8'd0;
            rx_data     <= 8'd0;
            rx_valid    <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            case (rx_state)
                RX_IDLE: begin
                    rx_baud_cnt <= 16'd0;
                    rx_bit_idx  <= 3'd0;
                    if (rx_negedge) rx_state <= RX_START;
                end
                RX_START: begin
                    if (rx_baud_cnt >= (BAUD_DIV / 2) - 1) begin
                        rx_baud_cnt <= 16'd0;
                        if (rx_sync[1] == 1'b0) rx_state <= RX_DATA;
                        else                    rx_state <= RX_IDLE;
                    end else rx_baud_cnt <= rx_baud_cnt + 1'b1;
                end
                RX_DATA: begin
                    if (rx_baud_cnt >= BAUD_DIV - 1) begin
                        rx_baud_cnt <= 16'd0;
                        rx_data_tmp[rx_bit_idx] <= rx_sync[1];
                        if (rx_bit_idx >= 3'd7) rx_state <= RX_STOP;
                        else                    rx_bit_idx <= rx_bit_idx + 1'b1;
                    end else rx_baud_cnt <= rx_baud_cnt + 1'b1;
                end
                RX_STOP: begin
                    if (rx_baud_cnt >= BAUD_DIV - 1) begin
                        rx_baud_cnt <= 16'd0;
                        rx_data     <= rx_data_tmp;   // 停止位不校验，容错优先
                        rx_valid    <= 1'b1;
                        rx_state    <= RX_IDLE;
                    end else rx_baud_cnt <= rx_baud_cnt + 1'b1;
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    //==================================================================
    // 3) 乐谱存储：每个事件一条记录
    //      slot_mask[i] = 这个事件点亮哪些键（和弦 = 多个位）
    //      slot_dur[i]  = 这个事件占几个时间格（>=1）
    //
    //  ★★ 关键：数组【只能有一个写地址】★★
    //     一开始写成"先写 slot_dur[i]=1，遇到 0xC2 再回改 slot_dur[i-1]"，
    //     结果同一个数组出现两个写地址，综合器无法推断 RAM，直接退化成
    //     2048 个触发器（实测 Register 从 5474 涨到 7712）。
    //     现在改成：事件先缓存在 pend_* 寄存器里，时值（0xC2）确定后才
    //     一次性落盘，于是所有写都落在 slot_xxx[wr_slots] 这【一个】地址上。
    //==================================================================
    //★★ V33.1：数据先稳定、沿后翻（见上面 fx_set/tone_set 的说明）★★
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fx_tgl      <= 1'b0;
            tone_we_tgl <= 1'b0;
            fx_dly      <= 2'd0;
            tone_dly    <= 2'd0;
        end else begin
            fx_dly   <= {fx_dly[0],   fx_set};
            tone_dly <= {tone_dly[0], tone_set};
            if (fx_dly[1])   fx_tgl      <= ~fx_tgl;
            if (tone_dly[1]) tone_we_tgl <= ~tone_we_tgl;
        end
    end

    reg [35:0] slot_mask [0:SLOT_MAX-1];
    // ★ V32：这一格除了"点亮哪些键"，还带 3 位【八度偏移】
    //   {oct[2:0], dur[7:0]} = 11 位。1024×11 = 11kbit，还是占【同一块】BSRAM
    //   （原来 1024×8 也只用了半块），所以一个块都没多花。
    //   ★ 读地址仍然只有 rd_next 这一个（见下面播放器），不能为它再开一个读口，
    //     否则综合器不肯推断 RAM，会退化成触发器。
    reg [10:0] slot_dur  [0:SLOT_MAX-1];

    reg [10:0] wr_slots;       // 已落盘的事件数（0..1024）
    reg        armed;           // 接收窗口：0xAA 开、0xFF 关
    reg        in_chord;        // 正在 0xC0..0xC1 之间
    reg [35:0] acc_mask;        // 和弦累积掩码
    reg        clr_pulse;       // 收到 0xAA（通知播放器停播）
    reg        stop_pulse;      // 收到 0xC7：停播但【保留乐谱】
    reg        replay_pulse;    // 收到 0xC8：重播板内已存的那一帧

    // 按键->音符 映射的写入（0xC6 + 2 字节）
    //   ★ 写口在 clk_50m_ext，key_map 本体在 top.v 的 clk_audio 域，
    //     所以这里只出"翻转信号 + 两个稳定字节"，由 top.v 那边同步 + 边沿检测。
    reg [1:0]  map_left;        // 0=空闲；2=等键号；1=等音符
    //   （keymap_key / keymap_note / keymap_we_tgl 本身就是 output reg，直接当寄存器用）

    // ★★ V33.1：翻转沿必须【晚于数据】两拍 ★★
    //   原来是"写数据"和"翻沿"同一个时钟沿发出的。接收侧(clk_50m)把
    //   随变信号各打两拍同步，两条链的布线延迟不一样 —— 采到哪一组全看运气。
    //   表现就是：上一次按的是「滑音」（位号 0），下一次按「谐波」时，
    //   接收侧还采到上一次的 0，于是翻的是滑音。
    //   现在数据在 T 拍写、沿在 T+2 拍翻，接收侧一定先看到新数据、再看到沿。
    //   （和 V29 的按键映射写口同一个纪律：key/note 先稳定，we_tgl 后翻。）
    reg        fx_set, tone_set;      // 操作数到位（单拍）
    reg [1:0]  fx_dly, tone_dly;      // 延两拍再翻沿

    // ★ V31：效果开关 / 音色的命令接收
    //   fx_left   : 1 = 下一个字节是"翻转哪个效果位"的操作数（原始数据！）
    //   tone_left : 1 = 下一个字节是"目标音色"的操作数（原始数据！）
    //   ★ 两个操作数同样要计数器挡着：它们可以是 0xAA/0xFF/0xC0 任意值，
    //     没有计数器就会被当成命令字解释（和 0xC2/0xC6 同一个道理）。
    //   fx_req    : 收到 0xCB 后置位，等效果帧真正发出去了才清 —— 用【电平】
    //               而不是脉冲，免得正好赶上 UART 忙时把请求丢了。
    //   ★ 为什么 fx_req 和 fx_req_set 是两个寄存器：
    //     置位发生在"收字节"那个 always 块里，清位发生在"发效果帧"那个
    //     always 块里 —— 同一个 reg 被两个 always 块驱动，综合器直接报
    //     EX2000（重复驱动）。所以收字节那边只出一个单拍脉冲 fx_req_set，
    //     由发帧那边自己置位/清位，各管各的。
    reg        fx_left;
    reg        tone_left;
    reg        fx_req_set;       // 收到 0xCB 的单拍脉冲（本块内部用）
    // ★ V32：八度偏移的接收
    //   oct_left : 1 = 下一个字节是 0xCC 的操作数（原始数据，同样要计数器挡着）
    //   oct_hold : 最近一条 0xCC 给的偏移；挂到下一个事件上，之后就清 0
    //              （所以"只作用于下一个事件"，不会粘住后面的事件）
    //   pend_oct : 跟着待落盘事件一起走，落盘时写进 slot 里
    reg        oct_left;
    reg [2:0]  oct_hold;
    reg [2:0]  pend_oct;
    //   tone_sel 在本域里两级同步后再用（top.v 那侧是 clk_50m 域）
    reg [1:0]  tone_s1, tone_s2;
    wire [1:0] tone_sel_sync = tone_s2;

    // 待落盘事件（时值要等后面的 0xC2 才知道，所以先缓存）
    reg [35:0] pend_mask;
    reg [7:0]  pend_dur;
    reg        pend_valid;

    // 设置参数接收
    reg [1:0]  cfg_left;        // 0=空闲；3/2/1 = 还要收几个参数字节
    reg        cfg_which;       // 0=SET_TICK, 1=SET_REL
    reg [15:0] cfg_acc;         // 已收的低两字节
    reg        dur_wait;        // 1 = 下一个字节是 0xC2 的操作数 N

    // 时值：总格数 = 1 + N（饱和到 255）
    wire [8:0] dur_patch   = {1'b0, rx_data} + 9'd1;
    wire [7:0] dur_patch_c = (dur_patch > 9'd255) ? 8'd255 : dur_patch[7:0];

    // 回显请求
    reg        tx_req;
    reg [7:0]  tx_byte;

    // 帧起播
    reg        frame_start;
    reg [10:0] frame_slots;    // 本帧的事件数（在收到 0xFF 那一拍锁存）

    //------------------------------------------------------------------
    // 组合：本拍要不要落盘、要不要挂新的待落盘事件
    //------------------------------------------------------------------
    // ★★ V33.1：命令的其余操作数也【不算音符】★★
    //   踩过的坑：上位机现在会把控制命令【插队】夹在谱子中间发（见电脑版的
    //   _send_front），而这一段 in_window 是【组合逻辑】，原来只挡了
    //   cfg_left / dur_wait —— 于是 0xC9/0xCA/0xCC/0xC6 的操作数（比如 0x02）
    //   一旦落在开窗期内，就同时触发了 plain_note -> 往谱子里凭空多塞一个音。
    //   所以所有"正在等操作数"的状态都要挡掉。
    wire op_wait     = (map_left != 2'd0) || oct_left || fx_left || tone_left;
    wire in_window   = rx_valid && armed && (cfg_left == 2'd0) && !dur_wait && !op_wait;
    wire chord_close = in_window && (rx_data == CHORD_OFF) && in_chord;
    wire plain_note  = in_window && (rx_data <= NOTE_MAX) && !in_chord;
    wire plain_rest  = in_window && (rx_data == REST_BYTE) && !in_chord;
    wire frame_end   = in_window && (rx_data == END_BYTE);

    // ★★ 0xC0（和弦帧头）绝对不能当"落盘触发" ★★
    //   落盘是"延后一拍"的：这一拍的 flush 写下去的是【上一次】set_pend 挂上的
    //   pend_*，而本拍新挂的事件要等下一次 flush 才落盘。
    //   一个和弦本来只该产生【一次】set_pend（在 0xC1 那一拍），
    //   如果 0xC0 也算触发，就等于一个和弦吃了两次 flush：
    //       0xC0 拍：把上一个事件又写了一遍   <- 前一个音被重复一次
    //       0xC1 拍：才写上这个和弦
    //   结果每个和弦前面那个音都会被写成两格（听感：那个音拖长一倍）。
    //   去掉 chord_open 之后，"事件 -> 下一次触发时落盘"这条流水线才是对的。
    wire        flush_now  = chord_close || plain_note || plain_rest || frame_end;
    wire        set_pend   = chord_close || plain_note || plain_rest;
    wire [35:0] new_p_mask = chord_close ? acc_mask
                           : plain_note  ? (36'd1 << rx_data[5:0])
                                         : 36'd0;      // 休止 = 空掩码
    wire        flush_ok   = flush_now && pend_valid && (wr_slots < SLOT_MAXN);
    wire [10:0] slots_next = wr_slots + (flush_ok ? 11'd1 : 11'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_slots    <= 11'd0;
            armed       <= 1'b0;
            in_chord    <= 1'b0;
            acc_mask    <= 36'd0;
            clr_pulse   <= 1'b0;
            stop_pulse  <= 1'b0;
            replay_pulse<= 1'b0;
            pend_mask   <= 36'd0;
            pend_dur    <= 8'd1;
            pend_valid  <= 1'b0;
            cfg_left    <= 2'd0;
            cfg_which   <= 1'b0;
            cfg_acc     <= 16'd0;
            dur_wait    <= 1'b0;
            map_left       <= 2'd0;
            keymap_key     <= 6'd0;
            keymap_note    <= 6'd0;
            keymap_we_tgl  <= 1'b0;
            fx_left        <= 1'b0;      // ★ V31
            tone_left      <= 1'b0;
            fx_req_set     <= 1'b0;
            oct_left       <= 1'b0;      // ★ V32
            oct_hold       <= 3'd0;
            pend_oct       <= 3'd0;
            // ★ oct_cur 由【播放器那个 always 块】独占驱动，这里不能再赋值
            //   （两个块驱动同一个 reg 会报 EX2000 重复驱动）
            // ★ V33.1：fx_tgl / tone_we_tgl 由下面"数据先稳定、沿后翻"那个
            //   独立 always 块驱动，这里【不能】再赋值（两个块驱动同一个 reg
            //   会报 EX2000 重复驱动）。只留数据位和计数器。
            fx_bit         <= 3'd0;
            tone_val       <= 2'd0;
            frame_start <= 1'b0;
            frame_slots <= 11'd0;
            tx_req      <= 1'b0;
            tx_byte     <= 8'd0;
            bt_keys     <= 36'd0;        // ★ 只有这个块驱动 bt_keys
            tick_us_r   <= TICK_DEF_US;  // ★ 时间参数也只由这个块驱动
            rel_us_r    <= REL_DEF_US;
            // ★ score_we/score_waddr/score_wdata/play_start/play_len
            //   由下面独立的"死端口"块统一驱动，这里不能再赋值（EX2000 重复驱动）
        end else begin
            frame_start <= 1'b0;
            clr_pulse   <= 1'b0;
            stop_pulse  <= 1'b0;
            replay_pulse<= 1'b0;
            tx_req      <= 1'b0;
            rec_toggle  <= 1'b0;
            fx_req_set  <= 1'b0;     // ★ V31：单拍脉冲，每拍自动清
            fx_set      <= 1'b0;     // ★ V33.1：单拍
            tone_set    <= 1'b0;

            //------------------------------------------------------------
            // (0) 统一落盘：★ 全文件只有这一处写 slot_mask / slot_dur ★
            //------------------------------------------------------------
            if (flush_ok) begin
                slot_mask[wr_slots] <= pend_mask;
                slot_dur [wr_slots] <= {pend_oct, pend_dur};   // ★ V32：带上八度偏移
                wr_slots            <= slots_next;
            end

            //------------------------------------------------------------
            // (0b) 挂起新的待落盘事件（时值在 0xC2 到来时再改 pend_dur）
            //------------------------------------------------------------
            if (set_pend) begin
                pend_mask  <= new_p_mask;
                pend_dur   <= 8'd1;
                pend_valid <= 1'b1;
                // ★ V32：把"上一条 0xCC 给的八度偏移"挂到这个事件上，然后清掉 ——
                //   于是它只作用一个事件；下一个事件默认回原调（0）。
                pend_oct   <= oct_hold;
                oct_hold   <= 3'd0;
            end

            if (rx_valid) begin
                //--------------------------------------------------------
                // (a) 设置参数的操作数（3 字节小端）—— 原始数据，不作解释
                //--------------------------------------------------------
                if (cfg_left != 2'd0) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (cfg_left == 2'd1) begin
                        // 第 3 字节到位：整字提交
                        if (cfg_which == 1'b0) tick_us_r <= {rx_data, cfg_acc};
                        else                   rel_us_r  <= {rx_data, cfg_acc};
                        cfg_left <= 2'd0;
                    end else begin
                        if (cfg_left == 2'd3) cfg_acc[7:0]  <= rx_data;
                        else                  cfg_acc[15:8] <= rx_data;
                        cfg_left <= cfg_left - 1'b1;
                    end
                end
                //--------------------------------------------------------
                // (b) 0xC2 的操作数 N —— 原始数据，不作解释
                //     只改 pend_dur 寄存器，★ 不碰数组 ★
                //--------------------------------------------------------
                else if (dur_wait) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    dur_wait <= 1'b0;
                    if (!in_chord && pend_valid) pend_dur <= dur_patch_c;
                end
                //--------------------------------------------------------
                // (b3) 0xC6 写一条【按键 -> 音符】映射：后跟 2 字节
                //        第 1 字节 = 物理键号 0..35
                //        第 2 字节 = 目标音符 0..35；给 >=36 的值（比如 0x3F）
                //                   表示【禁用这个键】
                //      ★ 两个操作数是【原始数据】：哪怕它们是 0xAA/0xFF/0xC0，
                //        也一律当数据，所以要用 map_left 计数器挡一下 ——
                //        和 0xC2/0xC4 的操作数一个道理。
                //      ★ 不受开窗限制：配置随时可以做。
                //--------------------------------------------------------
                else if (map_left != 2'd0) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (map_left == 2'd2) begin
                        keymap_key <= rx_data[5:0];
                        map_left   <= 2'd1;
                    end else begin
                        keymap_note   <= rx_data[5:0];
                        keymap_we_tgl <= ~keymap_we_tgl;   // 两字节齐了，通知音频域
                        map_left      <= 2'd0;
                    end
                end
                //--------------------------------------------------------
                // (b4) 0xC7 停止演奏（★ 保留乐谱，可以重播）
                //      只停播 + 放键，wr_slots / frame_slots 一个字不动。
                //      和 0xAA 的区别：0xAA 顺手把乐谱清了，这个不清。
                //--------------------------------------------------------
                else if (rx_data == STOP_BYTE) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= STOP_BYTE;
                    stop_pulse <= 1'b1;
                end
                //--------------------------------------------------------
                // (b5) 0xC8 重播板上已经存着的那一帧
                //      用的是 frame_slots（上次 0xFF 时锁存的），不用重发谱子。
                //--------------------------------------------------------
                else if (rx_data == REPLAY_BYTE) begin
                    tx_req       <= 1'b1;
                    tx_byte      <= REPLAY_BYTE;
                    replay_pulse <= 1'b1;
                end
                //--------------------------------------------------------
                // (b2) 0x5C 上位机翻转录音开关（不受开窗限制，随时可切）
                //      真正的翻转在 rec_on 那个 always 块里做，这里只发 1 拍脉冲
                //--------------------------------------------------------
                else if (rx_data == REC_CMD_BYTE) begin
                    rec_toggle <= 1'b1;
                    tx_req     <= 1'b1;                // 回显一个确认
                    tx_byte    <= REC_CMD_BYTE;
                end
                //--------------------------------------------------------
                // (b6) 0xC6 命令字本身：等两个操作数
                //--------------------------------------------------------
                else if (rx_data == KEYMAP_SET) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= KEYMAP_SET;
                    map_left <= 2'd2;
                end
                //--------------------------------------------------------
                // (b7) ★ V31：0xC9 / 0xCA 的操作数 —— 原始数据，不作解释
                //        fx_left  : 下一个字节 = 翻转哪个效果位（0..7）
                //        tone_left: 下一个字节 = 目标音色（0 普通 / 1 笛子 / 2 吉他）
                //      ★ 操作数可以是 0xAA / 0xFF / 0xC0 任意值，所以必须由
                //        计数器挡在前面（和 0xC2/0xC6 完全同理）。
                //        位号只取低 3 位、音色只取低 2 位，越界值一律当 0，
                //        这样即使上位机发错也只会退化成"最保守的那一档"。
                //--------------------------------------------------------
                else if (fx_left) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    fx_bit  <= rx_data[2:0];
                    fx_left <= 1'b0;
                    // ★ 只在这里写【数据】，翻沿交给下面延两拍的 fx_dly ——
                    //   数据与沿同拍变化的话，接收侧两条同步链谁先到看布线，
                    //   会采到【上一次的位号】（按谐波结果翻了滑音，踩过）。
                    fx_set  <= 1'b1;
                end
                else if (tone_left) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= rx_data;
                    tone_val   <= (rx_data > 8'd2) ? 2'd0 : rx_data[1:0];
                    tone_left  <= 1'b0;
                    tone_set   <= 1'b1;
                end
                //--------------------------------------------------------
                // (b8) 0xC9 命令字本身：等"翻转哪一位"
                //--------------------------------------------------------
                else if (rx_data == FX_TGL_BYTE) begin
                    tx_req  <= 1'b1;
                    tx_byte <= FX_TGL_BYTE;
                    fx_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (b9) 0xCA 命令字本身：等"目标音色"
                //--------------------------------------------------------
                else if (rx_data == TONE_SET_BYTE) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= TONE_SET_BYTE;
                    tone_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (b10) 0xCB 请求回报一次状态（不带操作数）
                //       只置 fx_req 这个【电平】，真正的 3 字节状态帧由下面
                //       那一路发；发出去了才清 —— 免得赶上忙的时候把请求丢了。
                //--------------------------------------------------------
                else if (rx_data == FX_REQ_BYTE) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= FX_REQ_BYTE;
                    fx_req_set <= 1'b1;
                end
                //--------------------------------------------------------
                // (b11) ★ V32：0xCC 的操作数 = 事件八度偏移（原始数据）
                //        有符号 3 位，只作用于【下一个事件】。
                //--------------------------------------------------------
                else if (oct_left) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    oct_hold <= rx_data[2:0];
                    oct_left <= 1'b0;
                end
                //--------------------------------------------------------
                // (b12) 0xCC 命令字本身：等那个偏移字节
                //--------------------------------------------------------
                else if (rx_data == OCT_BYTE) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= OCT_BYTE;
                    oct_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (c) 0xAA 开窗（同时停播、复位设置、清空缓冲）
                //--------------------------------------------------------
                else if (rx_data == CLR_BYTE) begin
                    armed     <= 1'b1;
                    wr_slots  <= 11'd0;
                    in_chord  <= 1'b0;
                    acc_mask  <= 36'd0;
                    dur_wait  <= 1'b0;
                    cfg_left  <= 2'd0;
                    pend_valid<= 1'b0;
                    clr_pulse <= 1'b1;                 // 通知播放器停
                    tick_us_r <= TICK_DEF_US;
                    rel_us_r  <= REL_DEF_US;
                    // ★ 0xAA 不回显（与 v1 一致）
                end
                //--------------------------------------------------------
                // (d) 0xFF 帧尾
                //     slots_next 已经把"还没落盘的最后一个事件"算进去了
                //--------------------------------------------------------
                else if (rx_data == END_BYTE) begin
                    tx_req <= 1'b1;
                    tx_byte <= END_BYTE;
                    if (armed) begin
                        frame_slots <= slots_next;
                        frame_start <= 1'b1;
                    end
                    armed      <= 1'b0;                // 关窗，后面的字节全丢
                    in_chord   <= 1'b0;
                    pend_valid <= 1'b0;
                end
                //--------------------------------------------------------
                // (e) 设置命令
                //--------------------------------------------------------
                else if (rx_data == SET_TICK || rx_data == SET_REL) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= rx_data;
                    cfg_left  <= 2'd3;
                    cfg_which <= (rx_data == SET_REL);
                end
                //--------------------------------------------------------
                // (f) 时值延长命令
                //--------------------------------------------------------
                else if (rx_data == DUR_EXT) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    dur_wait <= 1'b1;
                end
                //--------------------------------------------------------
                // (g) 和弦帧头 / 帧尾
                //--------------------------------------------------------
                else if (rx_data == CHORD_ON) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    in_chord <= 1'b1;
                    acc_mask <= 36'd0;
                end
                else if (rx_data == CHORD_OFF) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (in_chord) in_chord <= 1'b0;    // 落盘由 (0) 统一做
                end
                //--------------------------------------------------------
                // (h) 音号 / 休止符
                //--------------------------------------------------------
                else if (rx_data <= NOTE_MAX) begin
                    tx_req <= 1'b1;
                    tx_byte <= rx_data;
                    if (in_chord) acc_mask[rx_data[5:0]] <= 1'b1;   // 累进和弦
                end
                else if (rx_data == REST_BYTE) begin
                    tx_req <= 1'b1;
                    tx_byte <= rx_data;
                    // 和弦里出现休止符：忽略（上位机不应产生）
                end
                // 其它字节：丢弃，不回显（与 v1 一致）
            end
        end
    end

    //==================================================================
    // 4) 播放器
    //    每个事件占 slot_dur 个时间格；gate 在整个事件里保持高，
    //    只在【最后一个格】的末尾留出释音提前量。
    //
    //    发声 = dur × tick_cyc - rel_cyc
    //       "前 dur-1 个格全程发声" + "最后一个格发声 (tick_cyc - rel_cyc)"
    //     = (dur-1)×tick_cyc + tick_cyc - rel_cyc
    //     = dur × tick_cyc - rel_cyc          ∎
    //    所以这里只需要两个计数器，不需要任何宽乘法。
    //
    //  ★ 存储推断的硬约束：数组必须【单读单写】
    //    slot_dur 原来被读两个地址（当前 rd_slot 与前瞻 rd_slot+1），
    //    单口 RAM 装不下两个读地址，综合器就拒绝推断 RAM、退回 1024 个触发器。
    //    现在改成：只读 rd_next = rd_slot+1 这一个地址，
    //    "ev_left == 0" 表示"该装载下一个事件了"，装载时顺带把时长读进来。
    //    代价是事件之间多 1 个时钟（20ns）的过渡，相对最短 20ms 的时间格
    //    完全可以忽略，且此时 gate=0、不发键，听不出来。
    //
    //  ★★ 板子上"一声不响"的真凶（V21/V22 都栽在这）★★
    //    rd_slot 是 11 位；原写法 rd_next = rd_slot + 11'd1 在 rd_slot=1023 时
    //    得到的是 **1024**，不会自己绕回 0（要 11 位表示 1024 绰绰有余）。
    //    而结束时判据是 `rd_next >= frame_slots` —— 1024 对任何帧都成立，
    //    于是"装载事件 0"这一拍直接被判成"本帧放完"，play_run 一拍就掉回 0，
    //    midi_keys 从头到尾都是 0：**一个音都不响**。
    //    （V21 之所以"有声音"，是因为 play_start 被误接成 frame_start，
    //      点亮了 top.v 那条自动演奏链，循环弹 0/1/2/3 —— 那不是乐谱。
    //      把那条链改回恒 0 之后，就只剩这个 bug 的"完全静音"了。）
    //
    //    修法：
    //      ① rd_next 显式做【模 1024 绕回】；
    //      ② 结束条件改用 ev_remain（还剩几个事件没装载）——
    //         不能拿 rd_next 当结束条件，因为它会绕回。
    //==================================================================
    reg [9:0]  rd_slot;        // 当前正在放的事件索引（0..1023）
    reg [11:0] ev_remain;      // 还剩几个事件没装载（0..1024）
    reg [7:0]  ev_left;         // 本事件还剩几个格；0 = 需要装载下一个事件
    reg [29:0] tick_cnt;        // 当前格内的周期计数
    reg        play_run;

    // ★ slot_dur 的唯一读地址：★ 用 10 位存 0..1023，"+1" 自然会截断成 0 ★
    //   10 位最大就是 1023，1023+1 = 1024 -> 截成 10 位 = 0，绕回是【免费的】。
    //   写成 11 位再做三目判断（(rd_slot==1023)?0:rd_slot+1）会多出一个
    //   11 位比较器 + 选择器，正好串在 BSRAM 地址前面 —— 实测会把本来只有
    //   零点几纳秒余量的 clk_50m_ext 压垮（u_lcd 那边直接翻负）。
    wire [9:0]  rd_next  = rd_slot + 10'd1;
    // ★ V32：slot_dur 现在是 {八度偏移[2:0], 时值[7:0]}，拆开用。
    //   八度偏移在【装载事件的那一拍】一起取走（读的还是 rd_next 这一个地址，
    //   没有多开读口），之后整格事件期间保持不变。
    wire [7:0]  dur_next = (slot_dur[rd_next][7:0] == 8'd0) ? 8'd1 : slot_dur[rd_next][7:0];
    wire [2:0]  oct_next = slot_dur[rd_next][10:8];
    wire [35:0] slot_cur = ({1'b0, rd_slot} < frame_slots) ? slot_mask[rd_slot] : 36'd0;
    wire        gate     = play_run && (ev_left != 8'd0)
                           && ((ev_left > 8'd1) || (tick_cnt < gate_last));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_slot   <= 10'd1023;     // 哨兵：让第一次 rd_next 绕回 0
            ev_remain <= 12'd0;
            ev_left   <= 8'd0;
            tick_cnt  <= 30'd0;
            play_run  <= 1'b0;
            oct_cur   <= 3'd0;         // ★ V32
        end else if (clr_pulse) begin
            // 收到 0xAA：立即停播并放开所有键
            play_run  <= 1'b0;
            tick_cnt  <= 30'd0;
            rd_slot   <= 10'd1023;
            ev_remain <= 12'd0;
            ev_left   <= 8'd0;
            oct_cur   <= 3'd0;         // ★ V32：回原调
        end else if (stop_pulse) begin
            // ★ 收到 0xC7：停播 + 放键，但【乐谱一个字都不动】
            //   和上面 0xAA 的区别就在这：这里不碰 wr_slots / frame_slots，
            //   所以随后发一个 0xC8 就能拿板子里原来那份谱子重播。
            //   （rd_slot 归到哨兵，重播时第一拍 rd_next 绕回 0 就是事件 0）
            play_run  <= 1'b0;
            tick_cnt  <= 30'd0;
            rd_slot   <= 10'd1023;
            ev_remain <= 12'd0;
            ev_left   <= 8'd0;
            // ★ V32：这里【故意不把 oct_cur 清 0】—— 停止时正在响的那些音
            //   已经各自锁好了自己的八度（在 top.v 的 voice_oct[] 里），
            //   清掉它反而会让释音尾巴在下一拍被搬回原调（听起来跳音）。
            //   下一个事件装载时会从它自己的 slot 里取到正确的值。
        end else if (replay_pulse) begin
            // ★ 收到 0xC8：拿板内已经存着的那一帧重播，上位机不用重发谱子
            if (frame_slots == 11'd0) begin
                play_run <= 1'b0;              // 板子里本来就没谱子
                ev_left  <= 8'd0;
            end else begin
                play_run  <= 1'b1;
                rd_slot   <= 10'd1023;
                ev_remain <= {1'b0, frame_slots};
                ev_left   <= 8'd0;
                tick_cnt  <= 30'd0;
            end
        end else if (frame_start) begin
            if (frame_slots == 11'd0) begin
                play_run  <= 1'b0;             // 空帧（前导冲刷用的 FF）：不开播
                ev_left   <= 8'd0;
                ev_remain <= 12'd0;
            end else begin
                play_run  <= 1'b1;
                rd_slot   <= 10'd1023;         // 下一拍 rd_next 绕回 0 -> 装载事件 0
                ev_remain <= {1'b0, frame_slots};
                ev_left   <= 8'd0;
                tick_cnt  <= 30'd0;
            end
        end else if (play_run) begin
            if (ev_left == 8'd0) begin
                // ---- 装载下一个事件 ----
                if (ev_remain == 12'd0) begin
                    play_run <= 1'b0;                       // 本帧放完，放键
                end else begin
                    rd_slot   <= rd_next;
                    ev_left   <= dur_next;
                    ev_remain <= ev_remain - 12'd1;
                    tick_cnt  <= 30'd0;
                    oct_cur   <= oct_next;      // ★ V32：这一格的八度偏移，跟着事件走
                end
            end else if (tick_cnt >= tick_cyc - 1'b1) begin
                tick_cnt <= 30'd0;
                ev_left  <= (ev_left > 8'd1) ? (ev_left - 8'd1) : 8'd0;
            end else begin
                tick_cnt <= tick_cnt + 30'd1;
            end
        end
    end

    // 键掩码：gate 落就放键
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) midi_keys <= 36'd0;
        else        midi_keys <= gate ? slot_cur : 36'd0;
    end

    //==================================================================
    // 死端口：这五个信号恒 0 —— ★★ 必须恒 0，不是"没用随便给" ★★
    //
    //   top.v 里 play_start 接的是一条【自动演奏链】：
    //       play_start 上升沿 -> auto_en <= 1'b1
    //       auto_keys[note_idx] = 1      (note_idx 每 0.5s 走一格)
    //       key_mask = ... | auto_keys_aud | ...
    //   也就是说，只要 play_start 出现一次上升沿，top.v 就会【永久】多点亮
    //   一个键，并且每 0.5 秒换下一个 —— 听感是"乐谱之外一直有个音在拖"。
    //
    //   v1 把 play_start 恒 0，这条链就是死的（LCD 乐谱显示也一直是废的）。
    //   本版曾把 play_start 接成 frame_start、play_len 接成事件数，结果就是
    //   每帧结束后多出一个长鸣音 —— 已改回恒 0，与 v1 完全一致。
    //   和弦完全不依赖这条链：它走 midi_keys（36 位掩码）直接发声。
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            score_we    <= 1'b0;
            score_waddr <= 8'd0;
            score_wdata <= 8'd0;
            play_start  <= 1'b0;
            play_len    <= 8'd0;
        end else begin
            score_we    <= 1'b0;
            score_waddr <= 8'd0;
            score_wdata <= 8'd0;
            play_start  <= 1'b0;
            play_len    <= 8'd0;
        end
    end

    //==================================================================
    // 4b) 发送 FIFO（16 字节）
    //
    //   原来只有单字节 tx_req：tx 一忙，当拍来的字节直接丢。
    //   回显时收发同速（都是 1.04ms/字节）所以看不出来，
    //   但录音事件帧一次 8 个字节，不排队必丢。
    //
    //   ★ 指针必须是 5 位。16 深的 FIFO 用 4 位指针时 wr 会绕回 0，
    //     于是"装满了 16 个"和"空"读出来是同一个值 —— txf_full 永远为假，
    //     第 17 个字节会【静默覆盖】最早那个（有可能正好是帧头 0x5A）。
    //     多一位之后 cnt 落在 0..31，full 就是明确的 16。
    //
    //   ★ 为什么 16 够用（这条是设计前提，改采样率/波特率要重新算）：
    //     录音事件最短间隔 = 采样周期 10ms，一帧 8 字节；
    //     9600 波特下 8 字节要发 8.3ms，比 10ms 短。
    //     即"产得比发得慢"，稳态下 FIFO 里最多攒几个字节，离 16 很远。
    //==================================================================
    reg  [7:0]  txf_mem [0:15];
    reg  [4:0]  txf_wr, txf_rd;        // ★ 5 位（见上）
    wire [5:0]  txf_cnt   = {1'b0, txf_wr} - {1'b0, txf_rd};
    wire        txf_full  = (txf_cnt >= 6'd16);
    wire        txf_empty = (txf_cnt == 6'd0);

    //==================================================================
    // 4c) 录音回传
    //
    //   录的是"真正在响的音"：top.v 的 sounding_mask（= 包络非 0，含 ADSR 尾巴），
    //   每 10ms 采一次，一变就回传一个 8 字节事件帧：
    //       [0x5A] [mask 36 位 小端 5 字节] [dt 2 字节小端，单位 10ms]
    //   另外把【效果开关】也回传（变了才发，开录时强制发一次）：
    //       [0x5D] [8 位：滑音/延音/音色/颤音/延迟/笛子/混响/合唱]
    //   开关两端都能翻：
    //       · 板子上按【按钮板的第 10 键】（btn_rec，单拍脉冲）
    //       · 上位机发一个 0x5C
    //   开关一变就回传 [0x5B] [0/1]，让上位机同步界面。
    //
    //   ★ 录音期间【不回显】—— 回显字节是任意值，会和 0x5A 帧头撞车。
    //     录音时上位机本来也不发谱子，不回显不影响任何事。
    //==================================================================
    reg rec_on;
    reg rec_toggle;                    // 收到 0x5C 时置 1 拍

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rec_on <= 1'b0;
        else if (btn_rec | rec_toggle) rec_on <= ~rec_on;
    end

    // ---- 三种回传帧的状态（声明统一放在采样器【前面】，避免前向引用）----
    //   帧格式：事件 [0x5A][mask 5B][dt 2B] / 开关 [0x5B][0/1] / 效果 [0x5D][8 位]
    reg [63:0] rec_shift;              // 事件帧移位寄存器
    reg [3:0]  rec_tx_idx;             // 0 = 空闲；1..8 = 正在发第几个字节
    reg        rec_on_d;               // 上一次【已经回报过】的开关
    reg [1:0]  rec_st_idx;             // 开关帧：0 空闲 / 1 帧头 / 2 数据
    reg [7:0]  fx_last;                // 上一次已经回报过的效果状态
    reg [1:0]  tone_last;              // 上一次已经回报过的音色
    reg [7:0]  fx_byte_r;              // 本次要发的效果状态
    reg [2:0]  fx_idx;                 // 效果帧：0 空闲 / 1 帧头 / 2 状态 / 3 音色
    reg        fx_force;               // 开始录音时强制发一次初始状态
    reg        fx_req;                 // ★ V31：0xCB 请求的快照（电平，发出后清）

    // ★ V31：当前音色在本域里两级同步（top.v 那侧是 clk_50m 域）
    //   只有 2 位、而且是人手/上位机触发的慢变化，两级足够；
    //   万一同步到一个中间态，下一拍就自己纠正（顶多多发一帧状态）。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tone_s1 <= 2'd0;
            tone_s2 <= 2'd0;
        end else begin
            tone_s1 <= tone_sel_in;
            tone_s2 <= tone_s1;
        end
    end

    // 采样器用的寄存器（也必须声明在 rec_load 之前）
    reg [19:0] rec_tick;
    reg [15:0] rec_dt;                 // 距上一个事件的 10ms 数
    reg [35:0] rec_mask_r;             // 上一次采到的掩码
    reg        rec_fire;               // 有事件待发（电平，被取走才清）
    reg [35:0] rec_fire_mask;
    reg [15:0] rec_fire_dt;
    reg        rec_force;              // 开始录音时强制报一次当前掩码

    //==================================================================
    // ★★ 三种回传帧的【启动】判据：同一拍最多只有一个为真 ★★
    //
    //   这是本模块最容易写错的地方。写成"各路自己判自己的条件"，
    //   就会出现两路在同一拍同时启动：两边的移位寄存器都往前走了，
    //   但 FIFO 只收得下先写的那一个 —— 另一路的字节被悄悄吞掉
    //   （事件帧因此缺一个字节，上位机整帧错位，还查不出来）。
    //
    //   所以在这里统一排出优先级，后面三路都用这几个 wire：
    //       开关帧 > 效果帧 > 事件帧 > 回显
    //   ★ 判据里必须带 !st_start / !fx_start，不能只带"那一路当前是不是忙"
    //     —— 同一拍里那一路的 index 还是 0，"忙"是下一拍才成立的。
    //==================================================================
    wire st_want = (rec_on != rec_on_d);
    //==================================================================
    // ★★ V31：效果状态帧的启动判据改了 ★★
    //   原来：rec_on && (fx_force || 状态变了)  —— 只有录音时才回报。
    //   现在：三个来源都算 ——
    //     · fx_req            : 上位机发 0xCB 主动要一次快照
    //     · fx_force && rec_on: 开始录音时强制报一次初始状态（原逻辑不动）
    //     · 状态/音色变了      : 谁按的都算（板上的 TTP 按钮、上位机、按键映射）
    //   ★ 为什么必须"变了就报"：软件那边的开关要跟着板子走 ——
    //     用户在板子上按了按钮，上位机的开关也得动，否则两边不一致。
    //     效果开关是人手按的，很稀疏，这点带宽可以忽略。
    //==================================================================
    wire fx_want = fx_req || (fx_force && rec_on)
                   || (fx_state_in != fx_last) || (tone_sel_sync != tone_last);
    wire ev_want = rec_fire;

    wire st_start = st_want && (rec_tx_idx == 4'd0) && (fx_idx == 3'd0);
    wire fx_start = fx_want && (rec_tx_idx == 4'd0) && (rec_st_idx == 2'd0) && !st_start;
    wire rec_load = ev_want && (rec_tx_idx == 4'd0) && (rec_st_idx == 2'd0)
                    && (fx_idx == 3'd0) && !st_start && !fx_start;

    wire rec_busy = (rec_tx_idx != 4'd0) || (rec_st_idx != 2'd0) || (fx_idx != 3'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_tick <= 20'd0; rec_dt <= 16'd0; rec_mask_r <= 36'd0;
            rec_fire <= 1'b0; rec_fire_mask <= 36'd0; rec_fire_dt <= 16'd0;
            rec_force <= 1'b1;
        end else begin
            // ★ rec_fire 是【电平】：帧真正被装载时才清，不能每拍无条件清。
            //   每拍清的话，如果这次按键变化正好赶上上一帧还在往 FIFO 里推
            //   （要 8 拍），这个脉冲就被吃掉了 —— 那个音整条丢失，而且
            //   后面的 dt 是从"丢失的那次"继续算的，整条时间轴都会偏。
            //   注意这行写在采样逻辑【前面】，所以同一拍里"新事件置位"优先。
            if (rec_load) rec_fire <= 1'b0;
            if (!rec_on) begin
                // 关着时跟着当前掩码走：这样开录那一拍不会拿"上一次的状态"
                // 当变化报出去（真正的初始状态由下面的 rec_force 负责报一次）
                rec_tick   <= 20'd0;
                rec_dt     <= 16'd0;
                rec_mask_r <= gate_mask_in;
                rec_force  <= 1'b1;
            end else if (rec_tick >= REC_TICK_DIV - 1'b1) begin
                rec_tick <= 20'd0;
                // ★ 开始录音时强制报一次当前掩码（dt=0）。
                //   否则"按着键再开录音"这种情形下，那个键从头到尾都不会
                //   作为"变化"出现 —— 它整条不在录音里。
                //   报一次之后上位机就知道 t=0 时什么在响，不用猜。
                if (rec_force || (gate_mask_in != rec_mask_r)) begin
                    rec_mask_r    <= gate_mask_in;
                    rec_fire      <= 1'b1;
                    rec_fire_mask <= gate_mask_in;
                    // ★★ dt 必须把【这一拍】算进去 ★★
                    //   原来报的是 rec_dt（= 两次变化之间隔了几个"没变的"
                    //   采样边界），每来一个事件就少 10ms。12 个事件累积
                    //   −110ms，几百个事件就是好几秒 —— 听感就是"间隔不对、
                    //   越放越赶"。采样边界本身就是过去了 10ms，当然要算。
                    rec_fire_dt   <= rec_force ? 16'd0 :
                                     ((rec_dt == 16'hFFFF) ? 16'hFFFF
                                                           : (rec_dt + 1'b1));
                    rec_dt        <= 16'd0;
                    rec_force     <= 1'b0;
                end else if (rec_dt != 16'hFFFF) begin
                    rec_dt <= rec_dt + 1'b1;
                end
            end else begin
                rec_tick <= rec_tick + 1'b1;
            end
        end
    end

    // ---- 组帧（移位寄存器 / rec_load 的声明见上面采样器之前）----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_shift <= 64'd0; rec_tx_idx <= 4'd0;
        end else if (rec_load) begin
            // ★ 装载不等 txf_full：数据先搬进 rec_shift 就安全了。
            //   等的话，FIFO 一满、新的按键变化又会覆盖 rec_fire_*，
            //   等于白等一场还丢一个事件。
            rec_shift  <= {rec_fire_dt, 4'd0, rec_fire_mask, REC_EV_HEAD};
            rec_tx_idx <= 4'd1;
        end else if (rec_tx_idx != 4'd0 && !txf_full) begin
            rec_shift  <= {8'd0, rec_shift[63:8]};
            rec_tx_idx <= (rec_tx_idx == 4'd8) ? 4'd0 : (rec_tx_idx + 1'b1);
        end
    end

    wire       rec_push = (rec_tx_idx != 4'd0) && !txf_full;
    wire [7:0] rec_byte = rec_shift[7:0];

    // ---- 开关状态回报：[0x5B] [0/1] ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rec_on_d <= 1'b0; rec_st_idx <= 2'd0; end
        else if (rec_st_idx == 2'd0) begin
            // ★ rec_on_d 只在【真的要发】的那一拍才更新。
            //   写成"每拍 rec_on_d <= rec_on"的话，"状态变了"这个条件只维持一拍，
            //   被挡住一下之后就再也补不上了（开关回报丢失）。
            if (st_start) begin
                rec_on_d   <= rec_on;
                rec_st_idx <= 2'd1;
            end
        end else if (!txf_full) begin
            rec_st_idx <= (rec_st_idx == 2'd1) ? 2'd2 : 2'd0;
        end
    end
    wire       rec_st_push = (rec_st_idx != 2'd0) && !txf_full;
    wire [7:0] rec_st_byte = (rec_st_idx == 2'd1) ? REC_ST_HEAD : {7'd0, rec_on};

    //==================================================================
    // 效果状态帧：[0x5D] [8 位开关]
    //
    //   ★ 只在【状态变了】的时候发。效果开关是人手按的，很稀疏，
    //     一个演奏里通常就几次 —— 带宽可以忽略（事件帧才是大头）。
    //   ★ 开始录音时【强制】发一次初始状态，保证上位机一定知道
    //     "这一遍是从什么效果配置开始的"，不用去猜。
    //
    //   位定义（和 top.v 里打的那包一致）：
    //     bit0 滑音 glide_en   bit1 延音锁存 sustain_on
    //     bit2 音色 timbre     bit3 颤音 trem_on
    //     bit4 延迟 dly_on     bit5 笛子 flute_on
    //     bit6 混响 rev_on     bit7 合唱 cho_on
    //   ★ 按钮 3 的 wave_mode【不在】这里 —— 它只切 LCD 的显示（频谱/波形），
    //     一个采样点都不影响，录它没有意义。
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fx_last <= 8'd0; tone_last <= 2'd0; fx_byte_r <= 8'd0;
            fx_idx <= 3'd0; fx_force <= 1'b1; fx_req <= 1'b0;
        end else begin
            // 关着录音时把"强制报一次"重新武装（下面那帧如果在飞，让它先飞完）
            if (!rec_on && (fx_idx == 3'd0)) fx_force <= 1'b1;
            // ★ 0xCB 的请求：置位到这帧真正开始发为止（电平，丢了也能自己补）
            if (fx_req_set)   fx_req <= 1'b1;
            else if (fx_start) fx_req <= 1'b0;

            if (fx_idx == 3'd0) begin
                if (fx_start) begin
                    fx_last   <= fx_state_in;
                    tone_last <= tone_sel_sync;
                    fx_byte_r <= fx_state_in;
                    fx_idx    <= 3'd1;
                    fx_force  <= 1'b0;
                end
            end else if (!txf_full) begin
                // 1 -> 2 -> 3 -> 0：帧头、效果状态、音色，三个字节
                fx_idx <= (fx_idx == 3'd3) ? 3'd0 : (fx_idx + 3'd1);
            end
        end
    end
    wire       fx_push = (fx_idx != 3'd0) && !txf_full;
    wire [7:0] fx_byte = (fx_idx == 3'd1) ? REC_FX_HEAD :
                         (fx_idx == 3'd2) ? fx_byte_r   :
                                            {6'd0, tone_last};

    // ---- 统一入队：开关帧 > 效果帧 > 事件帧 > 回显 ----
    //   ★ 前三者已被 rec_busy 保证不会同时发生，这里的优先级只是兜底。
    //   ★ 回显也要让路：录音刚关掉的那 2 个字节（[0x5B][0]）还在飞，
    //     这时候上位机要是发了个字节，两路又会抢。
    wire       echo_push = tx_req && !rec_on && !rec_busy;
    wire       tx_push   = rec_st_push | fx_push | rec_push | echo_push;
    wire [7:0] tx_push_data = rec_st_push ? rec_st_byte :
                              fx_push     ? fx_byte     :
                              rec_push    ? rec_byte    : rx_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) txf_wr <= 5'd0;
        else if (tx_push && !txf_full) txf_wr <= txf_wr + 1'b1;
    end
    always @(posedge clk) begin
        if (tx_push && !txf_full) txf_mem[txf_wr[3:0]] <= tx_push_data;
    end

    //==================================================================
    // 5) 发送：收到什么就原样回传什么（0xAA 除外），帧尾补 0xFF
    //    即时回传比"整包缓存再回传"更好：收发共用一块状态，不会互相多驱动。
    //==================================================================
    localparam TX_IDLE = 3'd0, TX_START = 3'd1, TX_DATA = 3'd2,
               TX_STOP = 3'd3, TX_GAP   = 3'd4;

    reg [2:0]  tx_state;
    reg [15:0] tx_baud_cnt;
    reg [2:0]  tx_bit_idx;
    reg [7:0]  tx_data;
    reg        tx_have;

    wire tx_load = !txf_empty && (tx_state == TX_IDLE) && !tx_have;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            uart_txd    <= 1'b1;
            tx_baud_cnt <= 16'd0;
            tx_bit_idx  <= 3'd0;
            tx_data     <= 8'h00;
            tx_have     <= 1'b0;
            txf_rd      <= 5'd0;
        end else begin
            if (tx_load) begin
                tx_data <= txf_mem[txf_rd[3:0]];
                tx_have <= 1'b1;
                txf_rd  <= txf_rd + 1'b1;
            end

            case (tx_state)
                TX_IDLE: begin
                    uart_txd    <= 1'b1;
                    tx_baud_cnt <= 16'd0;
                    tx_bit_idx  <= 3'd0;
                    if (tx_have) begin
                        tx_have  <= 1'b0;
                        tx_state <= TX_START;
                    end
                end
                TX_START: begin
                    uart_txd <= 1'b0;
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        tx_bit_idx  <= 3'd0;
                        tx_state    <= TX_DATA;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                TX_DATA: begin
                    uart_txd <= tx_data[tx_bit_idx];
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        if (tx_bit_idx >= 3'd7) tx_state <= TX_STOP;
                        else                    tx_bit_idx <= tx_bit_idx + 1'b1;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                TX_STOP: begin
                    uart_txd <= 1'b1;
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        tx_state    <= TX_GAP;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                TX_GAP: begin
                    uart_txd <= 1'b1;
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        tx_state    <= TX_IDLE;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
