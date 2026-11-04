`timescale 1ns / 1ps
//=============================================================================
// ttp229_reader.v  ——  单片 TTP229（16 位模式）读取器
//
//   协议（与原 top.v 内联状态机完全一致）：
//     SDO 输出高 93us  →  输出低 10us 启动一次传输
//     → 释放 SDO  →  打 16 个 SCL，每个 SCL 下降沿读 1 位（bit0 先出）
//     → 等 2ms  →  再来一轮
//
//   key_pressed：最近一次【完整读数】的按下掩码，1 = 按下，低 12 位有效。
//
//   ★ 为什么要有 key_pressed 这个锁存寄存器，而不直接把 DATA 给外面用：
//     DATA 在每轮读周期的 IDLE / PIN_IN 会被清零，然后在 16 个时钟里
//     从 bit0 到 bit15 逐位填充。读的过程中 DATA 是"半成品"，此时
//     ~DATA 的高位全是 1，会被当成"一堆键被按下" → 虚报按键。
//     所以只在数据已经完整、且有 2ms 稳定窗口的 WAIT_2MS 第一拍，
//     把它锁存进 key_pressed，外部任何时候读到的都是上一次的完整结果。
//
//   rdy_toggle：每次锁存到一份新读数就翻转一次，供外部做"数据更新"判据。
//=============================================================================
module ttp229_reader #(
    parameter KEY_BITS = 12            // ★ 琴键用默认 12；按钮片传 16
)(
    input  wire        clk,           // 50MHz
    input  wire        rst_n,

    output wire        scl,
    inout  wire        sdo,

    output reg  [KEY_BITS-1:0] key_stable,   // ★ 位宽参数化
    output reg         rdy_toggle
);

    //------------------------------------------------------------------
    // 时序常数（50MHz，20ns/周期）—— 与原设计一致，勿随意改
    //------------------------------------------------------------------
    localparam DELAY_93US_CNT = 4650;     // 93us
    localparam DELAY_10US_CNT = 500;      // 10us
    localparam DELAY_2MS_CNT  = 100000;   // 2ms
    localparam SCL_HALF_CNT   = 250;      // SCL 半周期 5us → SCL 100kHz

    localparam IDLE     = 4'd0,
               PIN_OUT  = 4'd1,
               SDO_HIGH = 4'd2,
               SDO_LOW  = 4'd3,
               PIN_IN   = 4'd4,
               BIT_HIGH = 4'd5,
               BIT_LOW  = 4'd6,
               WAIT_2MS = 4'd7;

    reg  [3:0]  state;
    reg  [31:0] delay_cnt;
    reg  [15:0] DATA;
    reg  [4:0]  bit_cnt;
    // ★ 位宽必须跟着 KEY_BITS 走：原来写死 12 位，16 键那片实例的
    //   raw_d 只有 12 位（综合日志 WARN EX3791: 16 截成 12），
    //   于是 bit12~15 的"两轮一致"滤波里 raw_d 恒为 0，
    //   key_stable 的 bit12~15 永远不可能变成 1 —— 按钮13~16 直接是死的。
    reg  [KEY_BITS-1:0] raw_d;  // ★ 上一轮读数的原始值（抗干扰滤波用）

    reg sdo_out_en;
    reg sdo_out_data;
    reg scl_reg;

    assign scl = scl_reg;
    assign sdo = (sdo_out_en) ? sdo_out_data : 1'bz;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            DATA         <= 16'h0000;
            bit_cnt      <= 5'd0;
            delay_cnt    <= 32'd0;
            sdo_out_en   <= 1'b0;
            sdo_out_data <= 1'b1;
            scl_reg      <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    DATA    <= 16'h0000;
                    bit_cnt <= 5'd0;
                    scl_reg <= 1'b0;
                    state   <= PIN_OUT;
                end
                PIN_OUT: begin
                    sdo_out_en   <= 1'b1;
                    sdo_out_data <= 1'b1;
                    delay_cnt    <= 32'd0;
                    state        <= SDO_HIGH;
                end
                SDO_HIGH: begin
                    if (delay_cnt >= DELAY_93US_CNT) begin
                        delay_cnt <= 32'd0;
                        state     <= SDO_LOW;
                    end else delay_cnt <= delay_cnt + 1'b1;
                end
                SDO_LOW: begin
                    sdo_out_data <= 1'b0;
                    if (delay_cnt >= DELAY_10US_CNT) begin
                        delay_cnt <= 32'd0;
                        state     <= PIN_IN;
                    end else delay_cnt <= delay_cnt + 1'b1;
                end
                PIN_IN: begin
                    sdo_out_en <= 1'b0;
                    bit_cnt    <= 5'd0;
                    DATA       <= 16'h0000;
                    delay_cnt  <= 32'd0;
                    state      <= BIT_HIGH;
                end
                BIT_HIGH: begin
                    scl_reg <= 1'b1;
                    if (delay_cnt >= SCL_HALF_CNT) begin
                        delay_cnt <= 32'd0;
                        state     <= BIT_LOW;
                    end else delay_cnt <= delay_cnt + 1'b1;
                end
                BIT_LOW: begin
                    scl_reg <= 1'b0;
                    if (delay_cnt >= SCL_HALF_CNT) begin
                        delay_cnt <= 32'd0;
                        if (sdo == 1'b1) DATA[bit_cnt] <= 1'b1;
                        else             DATA[bit_cnt] <= 1'b0;

                        if (bit_cnt >= 15) state <= WAIT_2MS;
                        else begin
                            bit_cnt <= bit_cnt + 1'b1;
                            state   <= BIT_HIGH;
                        end
                    end else delay_cnt <= delay_cnt + 1'b1;
                end
                WAIT_2MS: begin
                    scl_reg <= 1'b0;
                    if (delay_cnt >= DELAY_2MS_CNT) begin
                        delay_cnt <= 32'd0;
                        state     <= IDLE;
                    end else delay_cnt <= delay_cnt + 1'b1;
                end
                default: state <= IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------
    // 每次进入 WAIT_2MS 的第一拍（delay_cnt 刚被清零）锁存完整读数
    //
    //   ★★ 竞态修复（照 music_test0 的结论，这是"按一次响两次/响一片"主因）★★
    //
    //   原实现把 key_pressed = ~DATA 直接送出去，而 DATA 在 IDLE 态会被清零
    //   （IDLE: DATA <= 16'h0000）—— 此后约 0.25ms 里 ~DATA 变成全 1，
    //   也就是"12 个键全按下"。上层如果正好在这个窗口里采样，
    //   就会把一整片键当成按下 → 12/36 路包络一起 Attack，听感是一记
    //   闷响或和弦咔声，紧跟在正常音头后面。
    //
    //   改法：只在 WAIT_2MS 第一拍（DATA 刚读完 16 位、还没被 IDLE 清零）
    //   锁存"整轮扫描期内恒定"的快照，外部只读这个快照 → 竞态消失。
    //
    //   抗干扰（2/2 去抖）：一轮读数可能被外部干扰打坏（LCD 的 SPI 时钟
    //   紧挨着 scl/sdo，快速沿很容易串进来），把"按住的键"读成"松开"。
    //   这种单轮坏读数会造成：key_mask 掉一拍 → 包络 attacking 被清零
    //   → 键位恢复后 env 直接跳回 255 → 同一个音被重弹一次（回声）。
    //   所以要【连续两轮读数一致】才承认变化：
    //       两次都为 1 → 1；两次都为 0 → 0；两轮不一致 → 保持上一次的结果。
    //   代价只是按下/松开各晚一轮读数（约 2.26ms），人感觉不到。
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_stable <= {KEY_BITS{1'b0}};
            rdy_toggle <= 1'b0;
            raw_d      <= {KEY_BITS{1'b0}};
        end else if (state == WAIT_2MS && delay_cnt == 32'd0) begin
            raw_d       <= ~DATA[KEY_BITS-1:0];
            key_stable  <= (~DATA[KEY_BITS-1:0] & raw_d)
                         | (key_stable & (~DATA[KEY_BITS-1:0] | raw_d));
            rdy_toggle  <= ~rdy_toggle;
        end
    end

endmodule
