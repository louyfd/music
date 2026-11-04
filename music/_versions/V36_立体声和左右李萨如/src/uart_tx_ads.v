`timescale 1ns / 1ps
//=============================================================================
// uart_tx_ads.v  —  VOFA+ JustFloat 格式
// 每 1 秒发一帧：4 个 float32（小端） + 帧尾 00 00 80 7F
//=============================================================================
module uart_tx_ads(
    input  wire        clk,        // 50MHz
    input  wire        rst_n,
    input  wire [15:0] ain0_data,
    input  wire [15:0] ain1_data,
    input  wire [15:0] ain2_data,
    input  wire [15:0] ain3_data,
    input  wire [15:0] force_data,   // 锁存+平滑后的力度值 (0~255)
    input  wire [15:0] fraw_data,    // 换算后、未锁存的原始力度 (0~255)
    input  wire [15:0] snd_data,     // 正在发声的通道数（ADSR 诊断）
    output wire        uart_tx
);

    localparam CLK_FREQ = 50_000_000;
    localparam BAUD     = 115200;
    localparam BAUD_CNT = CLK_FREQ / BAUD;   // 434

    // 20ms @ 50MHz = 1_000_000
    reg [19:0] tick_cnt;
    reg        tick_flag;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 20'd0;
            tick_flag <= 1'b0;
        end else begin
            if (tick_cnt >= 20'd999_999) begin
                tick_cnt  <= 20'd0;
                tick_flag <= 1'b1;
            end else begin
                tick_cnt  <= tick_cnt + 1'b1;
                tick_flag <= 1'b0;
            end
        end
    end

    //=========================================================
    // raw → float32（±4.096V，1LSB=125uV，结果单位 V）
    //   单端模式 raw 是 0~32767 的正数
    //   voltage = raw * 125e-6
    //=========================================================
    function [31:0] raw_to_float;
        input [15:0] raw;
        reg   [31:0] tmp;
        begin
            // raw * 125 / 1_000_000  → 浮点乘法在 RTL 里不好写，
            // 这里用整数算 mV，再转 float 的整数部分 + 小数部分
            // 更简单：直接把 raw 当整数发，VOFA+ 那边看原始值
            // 若要看电压，raw * 125e-6 = raw * 0.000125
            //
            // 为了简单，这里用定点：mV = raw * 125 / 1000
            // 然后转 float32
            //
            // 但 float32 转换需要 IEEE754 编码，手写比较麻烦。
            // 下面用最直接的方式：把 raw 直接当 float32 的整数部分发，
            // 即 float(raw)。VOFA+ 显示的就是 raw 值。
            //
            // 若要发电压(V)，建议在 VOFA+ 里用公式换算，
            // 或者用 FPGA 的浮点 IP。
            //
            // 下面给出 raw → float(raw) 的简化实现（0~32767 范围）
            tmp = {16'd0, raw};
            raw_to_float = int_to_float(tmp);
        end
    endfunction

    //---------------------------------------------------------
    // 整数 → IEEE754 float32（0~2^23-1 范围内精确）
    // raw 最大 32767，最高位 ≤ 14，一定走 exp < 23 分支
    //---------------------------------------------------------
    function [31:0] int_to_float;
        input [31:0] ival;
        reg [7:0]  exp;
        reg [22:0] mant;
        reg [31:0] v_shift;
        begin
            if (ival == 0) begin
                int_to_float = 32'h0000_0000;
            end else begin
                // 优先级编码：找最高位 1 的位置（0~23 范围）
                if      (ival[23]) exp = 8'd23;
                else if (ival[22]) exp = 8'd22;
                else if (ival[21]) exp = 8'd21;
                else if (ival[20]) exp = 8'd20;
                else if (ival[19]) exp = 8'd19;
                else if (ival[18]) exp = 8'd18;
                else if (ival[17]) exp = 8'd17;
                else if (ival[16]) exp = 8'd16;
                else if (ival[15]) exp = 8'd15;
                else if (ival[14]) exp = 8'd14;
                else if (ival[13]) exp = 8'd13;
                else if (ival[12]) exp = 8'd12;
                else if (ival[11]) exp = 8'd11;
                else if (ival[10]) exp = 8'd10;
                else if (ival[ 9]) exp = 8'd9;
                else if (ival[ 8]) exp = 8'd8;
                else if (ival[ 7]) exp = 8'd7;
                else if (ival[ 6]) exp = 8'd6;
                else if (ival[ 5]) exp = 8'd5;
                else if (ival[ 4]) exp = 8'd4;
                else if (ival[ 3]) exp = 8'd3;
                else if (ival[ 2]) exp = 8'd2;
                else if (ival[ 1]) exp = 8'd1;
                else               exp = 8'd0;

                // 尾数 = 去掉最高位后，左移对齐到 bit22
                v_shift = ival << (23 - exp);
                mant    = v_shift[22:0];

                int_to_float = {1'b0, (8'd127 + exp), mant};
            end
        end
    endfunction

    //=========================================================
    // 发送缓存：32 字节（7 路 float32 + 帧尾）
    //   [0..3]  AIN0 float32 小端          ← 压力传感器原始读数
    //   [4..7]  AIN1
    //   [8..11] AIN2
    //   [12..15]AIN3
    //   [16..19]力度值（锁存+平滑后，真正驱动音色的值）
    //   [20..23]力度值（换算后、未锁存）
    //   [24..27]正在发声的通道数（ADSR 诊断：松开后应慢慢降回 0）
    //   [28..31]帧尾 00 00 80 7F
    //   ★ 后三路是 music_all 新增的调试通道，原 ads1115 工程只有 4 路。
    //     VOFA+ 按帧尾自动识别通道数，看到 7 路是正常的。
    //=========================================================
    reg [7:0]  tx_buf [0:31];
    reg [7:0]  tx_idx;
    reg [7:0]  tx_len;
    reg        tx_start;
    reg        tx_busy;

    reg [31:0] f0, f1, f2, f3, f4, f5, f6;
    integer k;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_start <= 1'b0;
            tx_len   <= 8'd32;
            for (k = 0; k < 32; k = k + 1)
                tx_buf[k] <= 8'h00;
        end else begin
            tx_start <= 1'b0;

            if (tick_flag && !tx_busy) begin
                // raw → float32
                f0 = raw_to_float(ain0_data);
                f1 = raw_to_float(ain1_data);
                f2 = raw_to_float(ain2_data);
                f3 = raw_to_float(ain3_data);
                f4 = raw_to_float(force_data);
                f5 = raw_to_float(fraw_data);
                f6 = raw_to_float(snd_data);

                // 小端：低字节先发
                tx_buf[0]  <= f0[7:0];
                tx_buf[1]  <= f0[15:8];
                tx_buf[2]  <= f0[23:16];
                tx_buf[3]  <= f0[31:24];

                tx_buf[4]  <= f1[7:0];
                tx_buf[5]  <= f1[15:8];
                tx_buf[6]  <= f1[23:16];
                tx_buf[7]  <= f1[31:24];

                tx_buf[8]  <= f2[7:0];
                tx_buf[9]  <= f2[15:8];
                tx_buf[10] <= f2[23:16];
                tx_buf[11] <= f2[31:24];

                tx_buf[12] <= f3[7:0];
                tx_buf[13] <= f3[15:8];
                tx_buf[14] <= f3[23:16];
                tx_buf[15] <= f3[31:24];

                tx_buf[16] <= f4[7:0];
                tx_buf[17] <= f4[15:8];
                tx_buf[18] <= f4[23:16];
                tx_buf[19] <= f4[31:24];

                tx_buf[20] <= f5[7:0];
                tx_buf[21] <= f5[15:8];
                tx_buf[22] <= f5[23:16];
                tx_buf[23] <= f5[31:24];

                tx_buf[24] <= f6[7:0];
                tx_buf[25] <= f6[15:8];
                tx_buf[26] <= f6[23:16];
                tx_buf[27] <= f6[31:24];

                // 帧尾
                tx_buf[28] <= 8'h00;
                tx_buf[29] <= 8'h00;
                tx_buf[30] <= 8'h80;
                tx_buf[31] <= 8'h7F;

                tx_len   <= 8'd32;
                tx_start <= 1'b1;
            end
        end
    end

    //=========================================================
    // UART 发送
    //=========================================================
    reg [3:0]  uart_state;
    reg [7:0]  uart_data;
    reg [15:0] baud_cnt;
    reg [3:0]  bit_cnt;
    reg        uart_tx_reg;

    assign uart_tx = uart_tx_reg;

    localparam U_IDLE  = 4'd0;
    localparam U_START = 4'd1;
    localparam U_DATA  = 4'd2;
    localparam U_STOP  = 4'd3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_state  <= U_IDLE;
            uart_tx_reg <= 1'b1;
            baud_cnt    <= 16'd0;
            bit_cnt     <= 4'd0;
            tx_busy     <= 1'b0;
            tx_idx      <= 8'd0;
        end else begin
            case (uart_state)
                U_IDLE: begin
                    uart_tx_reg <= 1'b1;
                    if (tx_start) begin
                        tx_busy    <= 1'b1;
                        tx_idx     <= 8'd0;
                        uart_data  <= tx_buf[0];
                        baud_cnt   <= 16'd0;
                        uart_state <= U_START;
                    end else begin
                        tx_busy <= 1'b0;
                    end
                end

                U_START: begin
                    uart_tx_reg <= 1'b0;
                    if (baud_cnt >= BAUD_CNT-1) begin
                        baud_cnt   <= 16'd0;
                        uart_state <= U_DATA;
                        bit_cnt    <= 4'd0;
                    end else
                        baud_cnt <= baud_cnt + 1'b1;
                end

                U_DATA: begin
                    uart_tx_reg <= uart_data[bit_cnt];
                    if (baud_cnt >= BAUD_CNT-1) begin
                        baud_cnt <= 16'd0;
                        if (bit_cnt == 4'd7)
                            uart_state <= U_STOP;
                        else
                            bit_cnt <= bit_cnt + 1'b1;
                    end else
                        baud_cnt <= baud_cnt + 1'b1;
                end

                U_STOP: begin
                    uart_tx_reg <= 1'b1;
                    if (baud_cnt >= BAUD_CNT-1) begin
                        baud_cnt <= 16'd0;
                        if (tx_idx >= tx_len - 1) begin
                            uart_state <= U_IDLE;
                            tx_busy    <= 1'b0;
                        end else begin
                            tx_idx    <= tx_idx + 1'b1;
                            uart_data <= tx_buf[tx_idx + 1'b1];
                            uart_state<= U_START;
                        end
                    end else
                        baud_cnt <= baud_cnt + 1'b1;
                end

                default: uart_state <= U_IDLE;
            endcase
        end
    end

endmodule