`timescale 1ns / 1ps
//=============================================================================
// ads1115_ctrl.v  —  修正版（读事务前先写指针 0x00）
//
// 写配置：0x90 → 0x01 → cfg_h → cfg_l → STOP
// 等 8ms
// 写指针：0x90 → 0x00 → STOP
// 读结果：0x91 → 读高 → 读低 → NACK → STOP
// 通道 ch 轮询 0~3
//=============================================================================
module ads1115_ctrl(
    input  wire        clk,        // 50MHz
    input  wire        rst_n,

    inout  wire        i2c_scl,
    inout  wire        i2c_sda,

    output reg  [15:0] ain0_data,
    output reg  [15:0] ain1_data,
    output reg  [15:0] ain2_data,
    output reg  [15:0] ain3_data
);

    //-------------------------------------------------------------------------
    // SCL 节拍：四相，SCL 周期 = 4 × DIV × clk
    // DIV=500 → clk_en 10us → SCL 40us → 25kHz
    //-------------------------------------------------------------------------
    localparam DIV = 16'd125;

    reg [15:0] div_cnt;
    reg        clk_en;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
            clk_en  <= 1'b0;
        end else if (div_cnt >= DIV-1) begin
            div_cnt <= 16'd0;
            clk_en  <= 1'b1;
        end else begin
            div_cnt <= div_cnt + 1'b1;
            clk_en  <= 1'b0;
        end
    end

    //-------------------------------------------------------------------------
    // 开漏
    //-------------------------------------------------------------------------
    reg scl_low;
    reg sda_low;
    assign i2c_scl = scl_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_low ? 1'b0 : 1'bz;
    wire sda_in = i2c_sda;

    //-------------------------------------------------------------------------
    // 地址与寄存器
    //-------------------------------------------------------------------------
    localparam ADDR_W  = 8'h90;   // 0x48<<1 | W
    localparam ADDR_R  = 8'h91;   // 0x48<<1 | R
    localparam PTR_CFG = 8'h01;   // 配置寄存器
    localparam PTR_CONV= 8'h00;   // 转换结果寄存器
    localparam CFG_L   = 8'h83;   // DR[2:0]=100 -> 128SPS (转换 7.8ms), COMP 关
                                  // ★ 注释原来写"860SPS"是错的：860SPS 是 0xE3。
                                  //   0x83 = 128SPS，S_WAIT 的 9ms 就是配它的。

    //-------------------------------------------------------------------------
    // 状态
    //-------------------------------------------------------------------------
    localparam S_IDLE      = 5'd0;
    localparam S_START     = 5'd1;
    localparam S_SEND      = 5'd2;
    localparam S_ACK       = 5'd3;
    localparam S_RECV      = 5'd4;
    localparam S_SEND_ACK  = 5'd5;
    localparam S_STOP      = 5'd6;
    localparam S_WAIT      = 5'd7;
    localparam S_NEXT_CH   = 5'd8;

    reg [4:0]  state;
    reg [1:0]  phase;        // 四相
    reg [3:0]  bit_cnt;
    reg [7:0]  shifter;
    reg [7:0]  rx_byte;
    reg [7:0]  rd_hi;
    reg [2:0]  ch;
    reg        is_read;      // 0=写事务，1=读事务
    reg        ptr_only;     // 1=只写指针，不写配置
    reg [1:0]  step;         // 写事务步骤
    reg [1:0]  byte_cnt;     // 读事务：0=读高, 1=读低
    reg [31:0] wait_cnt;

    //-------------------------------------------------------------------------
    // 配置高字节：OS=1, MUX 随 ch 变, PGA=001(±4.096V), MODE=1(单次)
    //   AIN0: 0xC3, AIN1: 0xD3, AIN2: 0xE3, AIN3: 0xF3
    //-------------------------------------------------------------------------
    wire [2:0] ch_mux = (ch == 3'd0) ? 3'b100 :
                        (ch == 3'd1) ? 3'b101 :
                        (ch == 3'd2) ? 3'b110 :
                                       3'b111;

    wire [7:0] cfg_h  = {1'b1, ch_mux, 3'b001, 1'b1};

    //-------------------------------------------------------------------------
    // 主状态机
    //-------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            phase     <= 2'd0;
            scl_low   <= 1'b0;
            sda_low   <= 1'b0;
            bit_cnt   <= 4'd0;
            shifter   <= 8'd0;
            rx_byte   <= 8'd0;
            rd_hi     <= 8'd0;
            ch        <= 3'd0;
            is_read   <= 1'b0;
            ptr_only  <= 1'b0;
            step      <= 2'd0;
            byte_cnt  <= 2'd0;
            wait_cnt  <= 32'd0;
            ain0_data <= 16'd0;
            ain1_data <= 16'd0;
            ain2_data <= 16'd0;
            ain3_data <= 16'd0;
        end else if (clk_en) begin
            case (state)

                //-------------------------------------------------------------
                // 空闲：发起写配置事务
                //-------------------------------------------------------------
                S_IDLE: begin
                    scl_low  <= 1'b0;
                    sda_low  <= 1'b0;
                    phase    <= 2'd0;
                    is_read  <= 1'b0;
                    ptr_only <= 1'b0;
                    step     <= 2'd0;
                    state    <= S_START;
                end

                //-------------------------------------------------------------
                // START：SCL 高时 SDA 由高变低
                //-------------------------------------------------------------
                S_START: begin
                    case (phase)
                        2'd0: begin
                            scl_low <= 1'b0;   // SCL 高
                            sda_low <= 1'b0;   // SDA 高
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            scl_low <= 1'b0;   // SCL 高
                            sda_low <= 1'b1;   // SDA 拉低 → START
                            phase   <= 2'd2;
                        end
                        2'd2: begin
                            scl_low <= 1'b1;   // SCL 低
                            sda_low <= 1'b1;
                            shifter <= is_read ? ADDR_R : ADDR_W;
                            bit_cnt <= 4'd8;
                            phase   <= 2'd0;
                            state   <= S_SEND;
                        end
                        default: phase <= 2'd0;
                    endcase
                end

                //-------------------------------------------------------------
                // 发送 8 位（四相）
                //-------------------------------------------------------------
                S_SEND: begin
                    case (phase)
                        2'd0: begin
                            scl_low <= 1'b1;              // SCL 低
                            sda_low <= ~shifter[7];
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            scl_low <= 1'b0;              // SCL 高
                            phase   <= 2'd2;
                        end
                        2'd2: begin
                            scl_low <= 1'b0;              // SCL 高保持
                            phase   <= 2'd3;
                        end
                        2'd3: begin
                            scl_low <= 1'b1;              // SCL 低
                            shifter <= {shifter[6:0], 1'b0};
                            bit_cnt <= bit_cnt - 1'b1;
                            phase   <= 2'd0;
                            if (bit_cnt == 4'd1) begin
                                sda_low <= 1'b0;          // 释放 SDA
                                state   <= S_ACK;
                                phase   <= 2'd0;
                            end
                        end
                    endcase
                end

                //-------------------------------------------------------------
                // 收 ACK，并根据事务类型分流
                //-------------------------------------------------------------
                S_ACK: begin
                    case (phase)
                        2'd0: begin
                            scl_low <= 1'b1;   // SCL 低
                            sda_low <= 1'b0;   // 释放 SDA
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            scl_low <= 1'b0;   // SCL 高
                            phase   <= 2'd2;
                        end
                        2'd2: begin
                            scl_low <= 1'b0;   // SCL 高
                            phase   <= 2'd3;
                        end
                        2'd3: begin
                            scl_low <= 1'b1;   // SCL 低
                            phase   <= 2'd0;

                            if (is_read) begin
                                // 读事务：地址+R 的 ACK 到了 → 开始读高字节
                                bit_cnt <= 4'd8;
                                state   <= S_RECV;
                            end else begin
                                // 写事务：根据 step 发下一字节
                                step <= step + 1'b1;
                                case (step)
                                    2'd0: begin   // 地址W 完成 → 发指针
                                        if (ptr_only)
                                            shifter <= PTR_CONV;  // 0x00
                                        else
                                            shifter <= PTR_CFG;   // 0x01
                                        bit_cnt <= 4'd8;
                                        state   <= S_SEND;
                                    end
                                    2'd1: begin   // 指针完成
                                        if (ptr_only) begin
                                            // 只写指针 → 直接 STOP
                                            state <= S_STOP;
                                        end else begin
                                            // 写配置 → 发 cfg_h
                                            shifter <= cfg_h;
                                            bit_cnt <= 4'd8;
                                            state   <= S_SEND;
                                        end
                                    end
                                    2'd2: begin   // cfg_h 完成 → 发 cfg_l
                                        shifter <= CFG_L;
                                        bit_cnt <= 4'd8;
                                        state   <= S_SEND;
                                    end
                                    2'd3: begin   // cfg_l 完成 → STOP
                                        state   <= S_STOP;
                                    end
                                    default: state <= S_STOP;
                                endcase
                            end
                        end
                    endcase
                end

                //-------------------------------------------------------------
                // STOP：SCL 高时 SDA 由低变高
                //-------------------------------------------------------------
                S_STOP: begin
                    case (phase)
                        2'd0: begin
                            scl_low <= 1'b1;   // SCL 低
                            sda_low <= 1'b1;   // SDA 低
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            scl_low <= 1'b0;   // SCL 高
                            sda_low <= 1'b1;   // SDA 低
                            phase   <= 2'd2;
                        end
                        2'd2: begin
                            scl_low <= 1'b0;   // SCL 高
                            sda_low <= 1'b0;   // SDA 高 → STOP
                            phase   <= 2'd3;
                        end
                        2'd3: begin
                            scl_low <= 1'b1;   // SCL 低
                            phase   <= 2'd0;

                            if (!is_read) begin
                                if (ptr_only) begin
                                    // 指针写完 → 立即发起读事务（不需要等 8ms）
                                    ptr_only <= 1'b0;
                                    is_read  <= 1'b1;
                                    byte_cnt <= 2'd0;
                                    scl_low  <= 1'b0;
                                    sda_low  <= 1'b0;
                                    phase    <= 2'd0;
                                    state    <= S_START;
                                end else begin
                                    // 配置写完 → 等 8ms 转换
                                    wait_cnt <= 32'd0;
                                    state    <= S_WAIT;
                                end
                            end else begin
                                // 读事务结束 → 切通道
                                state <= S_NEXT_CH;
                            end
                        end
                    endcase
                end

                //-------------------------------------------------------------
                // 等转换完成
                //
                // ★★ V17 修的就是这里 —— 摇杆"不灵敏"的根因 ★★
                //
                //   这个 wait_cnt 在 `else if (clk_en)` 里面递增，所以它的
                //   单位是【clk_en 拍】而不是 clk 周期：
                //        clk_en 周期 = DIV × clk = 125 / 50MHz = 2.5us
                //   原来写 75_000，实际等的是 75000 × 2.5us = 187.5ms！
                //   加上 I2C 事务，一个通道 188ms，两个通道轮一圈 377ms
                //   —— 摇杆动完要等将近 0.4 秒才在音高上反映出来，
                //   这就是"推了没反应 / 有点不灵敏"的来源。
                //
                //   CFG_L = 0x83 -> DR[2:0]=100 = 128SPS -> 转换需要 7.8ms。
                //   留 9ms 余量 = 9ms / 2.5us = 3600 拍。
                //   改完：单通道 9ms + I2C 0.9ms，两通道各约 20ms 更新一次（50Hz），
                //   比原来的 377ms 快 19 倍。
                //   ★ 以后改这里必须按 clk_en 拍算，不要再按 50MHz 周期算。
                //-------------------------------------------------------------
                S_WAIT: begin
                    if (wait_cnt >= 32'd3_600) begin
                        wait_cnt <= 32'd0;
                        // 先写指针 0x00（转换结果寄存器）
                        is_read  <= 1'b0;
                        ptr_only <= 1'b1;
                        step     <= 2'd0;
                        scl_low  <= 1'b0;
                        sda_low  <= 1'b0;
                        phase    <= 2'd0;
                        state    <= S_START;
                    end else begin
                        wait_cnt <= wait_cnt + 1'b1;
                    end
                end

                //-------------------------------------------------------------
                // 接收 8 位（四相）
                //-------------------------------------------------------------
                S_RECV: begin
                    case (phase)
                        2'd0: begin
                            scl_low <= 1'b1;   // SCL 低
                            sda_low <= 1'b0;   // 释放 SDA 给从机
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            scl_low <= 1'b0;   // SCL 高
                            phase   <= 2'd2;
                        end
                        2'd2: begin
                            scl_low <= 1'b0;   // SCL 高
                            rx_byte <= {rx_byte[6:0], sda_in};
                            bit_cnt <= bit_cnt - 1'b1;
                            phase   <= 2'd3;
                        end
                        2'd3: begin
                            scl_low <= 1'b1;   // SCL 低
                            phase   <= 2'd0;
                            if (bit_cnt == 4'd0) begin
                                state <= S_SEND_ACK;
                            end
                        end
                    endcase
                end

                //-------------------------------------------------------------
                // 发 ACK / NACK
                //-------------------------------------------------------------
                S_SEND_ACK: begin
                    case (phase)
                        2'd0: begin
                            scl_low <= 1'b1;   // SCL 低
                            if (byte_cnt == 2'd0)
                                sda_low <= 1'b1;   // ACK：拉低
                            else
                                sda_low <= 1'b0;   // NACK：释放
                            phase   <= 2'd1;
                        end
                        2'd1: begin
                            scl_low <= 1'b0;   // SCL 高
                            phase   <= 2'd2;
                        end
                        2'd2: begin
                            scl_low <= 1'b0;   // SCL 高
                            phase   <= 2'd3;
                        end
                        2'd3: begin
                            scl_low <= 1'b1;   // SCL 低
                            sda_low <= 1'b0;   // 释放 SDA
                            phase   <= 2'd0;

                            if (byte_cnt == 2'd0) begin
                                // 高字节读完，继续读低字节
                                rd_hi    <= rx_byte;
                                byte_cnt <= 2'd1;
                                bit_cnt  <= 4'd8;
                                state    <= S_RECV;
                            end else begin
                                // 低字节读完，组装数据
                                case (ch)
                                    3'd0: ain0_data <= {rd_hi, rx_byte};
                                    3'd1: ain1_data <= {rd_hi, rx_byte};
                                    3'd2: ain2_data <= {rd_hi, rx_byte};
                                    3'd3: ain3_data <= {rd_hi, rx_byte};
                                endcase
                                state <= S_STOP;
                            end
                        end
                    endcase
                end

                //-------------------------------------------------------------
                // 下一通道
                //-------------------------------------------------------------
                S_NEXT_CH: begin
                    // ★★ V19 修正：原来这里是 if (ch == 3'd1) —— 只轮询 0 和 1，
                    //    也就是【AIN2 / AIN3 从来没被采样过】，ain2_data 永远
                    //    停在复位值，所以接在 A2 上的第二个摇杆完全没反应。
                    //    参考 D:\FPGA_60K\ADS11172（那个工程摇杆正常，它能读到 A2）。
                    //
                    //    ★ 这里轮询【3 个通道】：0(AIN0)、1(AIN1)、2(AIN2)
                    //        AIN3 没接东西，不轮询，省下 1/4 的时间。
                    //        单通道刷新周期 = 3 × 7.8ms ≈ 23ms
                    //        （原来 2 通道约 15ms，4 通道要 31ms）
                    if (ch == 3'd2)
                        ch <= 3'd0;
                    else
                        ch <= ch + 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule