//=============================================================================
// i2c_master.v —— 极简 I2C 主机（只做"写一个寄存器"这一种事务）
//
//   为什么只做这一种：
//     IS31FL3236A **没有任何读寄存器的指令**（数据手册明确"只支持写"），
//     所有操作都是同一个形状：
//         START + 从机地址(7位<<1 | W=0) + 寄存器地址 + 数据 + STOP
//     事务只有这一种，状态机就能写得最短、最不容易错。
//     ★ 这也是从 STM32 那份驱动移植过来的：它的 IS31_WriteReg() 用的
//       HAL_I2C_Mem_Write(..., I2C_MEMADD_SIZE_8BIT, &val, 1) 正是这个形状。
//
//   电气：
//     ★ I2C 是【开漏总线】：两边都只会"拉低"，高电平完全靠外部上拉电阻。
//       所以这里 scl/sda 都用三态 —— oe=1 时输出（且只输出 0 或 1），
//       oe=0 时高阻。真正的"高"是上拉电阻给的。
//       ★ 约束文件里把这两个脚设成 PULL_MODE=UP 作为兜底（用户板子上
//         如果已经有 4.7k 上拉，内部弱上拉不影响）。
//     ★ 从机地址是 7 位，总线上发的是 (dev << 1) | 0（最后一位 R/W = 0 = 写）。
//
//   时序：
//     SCL = CLK_HZ / (SCL_HZ × 4)，占空比 50%。每个 SCL 周期切成 4 个
//     "quarter"，状态机按 quarter 走：
//         q0：SCL=0，建立数据（SDA 在这时变化，从机在 SCL 高时采）
//         q1：SCL=1（上升沿）
//         q2：SCL=1（保持，从机在这里采数据 / 回 ACK）
//         q3：SCL=0（下降沿，读 ACK 就在这之前采）
//     默认 100kHz —— 比芯片上限 400kHz 保守，长排线或上拉偏弱时更稳。
//     ★ 第一次点亮用慢一点更容易成功；确认能亮之后想加快只需改 SCL_HZ。
//=============================================================================
`timescale 1ns / 1ps

module i2c_master #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SCL_HZ = 100_000
)(
    input  wire       clk,
    input  wire       rst_n,

    // ---- 事务口 ----
    input  wire       go,        // 单拍脉冲：发起一次事务
    input  wire [6:0] dev,       // 7 位从机地址
    input  wire [7:0] reg_a,     // 寄存器地址
    input  wire [7:0] dat,       // 要写入的数据

    output reg        busy,      // 1 = 事务进行中
    output reg        done,      // 单拍脉冲：事务结束
    output reg        nack,      // 1 = 本次事务里出现过 NACK（地址/寄存器/数据任一）

    // ---- 总线 ----
    output wire       scl,
    inout  wire       sda
);

    // 每个 quarter 的时钟数（至少 1，防止参数配得太极端时除出 0）
    localparam integer QUARTER = (CLK_HZ / (SCL_HZ * 4)) < 1 ? 1
                                                             : (CLK_HZ / (SCL_HZ * 4));

    localparam [3:0] S_IDLE    = 4'd0,
                     S_START   = 4'd1,   // SCL 高时把 SDA 拉低
                     S_ADDR    = 4'd2,   // 发 8 位从机地址
                     S_ACK1    = 4'd3,   // 收地址的 ACK
                     S_REG     = 4'd4,   // 发 8 位寄存器地址
                     S_ACK2    = 4'd5,   // 收寄存器地址的 ACK
                     S_DATA    = 4'd6,   // 发 8 位数据
                     S_ACK3    = 4'd7,   // 收数据的 ACK
                     S_STOP    = 4'd8,   // SCL 高时把 SDA 释放（拉高）
                     S_DONE    = 4'd9;

    reg [3:0]  st;
    reg [1:0]  q;             // quarter 0..3
    reg [15:0] qcnt;          // quarter 内的时钟计数
    reg [2:0]  bitn;          // 位计数 0..7
    reg [7:0]  shreg;         // 发送移位寄存器

    reg        sda_oe;
    reg        sda_out;
    reg        scl_r;

    assign scl = scl_r;
    assign sda = sda_oe ? sda_out : 1'bz;

    // 一个 quarter 走完了吗
    wire qend = (qcnt >= QUARTER[15:0] - 16'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st      <= S_IDLE;
            q       <= 2'd0;
            qcnt    <= 16'd0;
            bitn    <= 3'd0;
            shreg   <= 8'd0;
            sda_oe  <= 1'b1;
            sda_out <= 1'b1;
            scl_r   <= 1'b1;      // 空闲时两根线都高（靠上拉）
            busy    <= 1'b0;
            done    <= 1'b0;
            nack    <= 1'b0;
        end else begin
            done <= 1'b0;

            // ---- quarter 推进：q 从 0 走到 3 就回到 0，并让状态机走一步 ----
            if (qend) begin
                qcnt <= 16'd0;
                q    <= (q == 2'd3) ? 2'd0 : (q + 2'd1);
            end else begin
                qcnt <= qcnt + 16'd1;
            end
            // qtick = 本拍正好是一个 quarter 的最后一拍（= 该 quarter 结束）
            //   下面的状态跳转都只在 qtick 上发生，保证每位占满 4 个 quarter。
            //   （用组合判断，避免把 q 的变化再延一拍。）

            case (st)
                //----------------------------------------------------------
                S_IDLE: begin
                    scl_r   <= 1'b1;
                    sda_oe  <= 1'b1;
                    sda_out <= 1'b1;
                    busy    <= 1'b0;
                    if (go) begin
                        busy  <= 1'b1;
                        nack  <= 1'b0;
                        shreg <= {dev, 1'b0};   // 地址 + W=0
                        bitn  <= 3'd0;
                        q     <= 2'd0;
                        qcnt  <= 16'd0;
                        st    <= S_START;
                    end
                end
                //----------------------------------------------------------
                // START：SCL 保持高，SDA 由高变低
                S_START: begin
                    scl_r <= 1'b1;
                    if (q == 2'd0) begin
                        sda_oe  <= 1'b1;
                        sda_out <= 1'b1;
                    end else if (q == 2'd2 && qend) begin
                        sda_out <= 1'b0;        // 这一下就是 START
                    end
                    if (q == 2'd3 && qend) begin
                        scl_r <= 1'b0;          // 拉低 SCL，准备发第一位
                        st    <= S_ADDR;
                    end
                end
                //----------------------------------------------------------
                // 发 8 位（地址 / 寄存器地址 / 数据都走这里，靠 shreg 区分）
                S_ADDR, S_REG, S_DATA: begin
                    if (q == 2'd0) begin
                        scl_r   <= 1'b0;
                        sda_oe  <= 1'b1;
                        sda_out <= shreg[7];    // 最高位先发
                    end else if (q == 2'd2 && qend) begin
                        scl_r <= 1'b1;          // 从机在这时采样
                    end else if (q == 2'd3 && qend) begin
                        scl_r <= 1'b0;
                        shreg <= {shreg[6:0], 1'b0};
                        if (bitn == 3'd7) begin
                            bitn <= 3'd0;
                            // 8 位发完 -> 进对应的 ACK 相位
                            st   <= (st == S_ADDR) ? S_ACK1 :
                                    (st == S_REG)  ? S_ACK2 : S_ACK3;
                        end else begin
                            bitn <= bitn + 3'd1;
                        end
                    end
                end
                //----------------------------------------------------------
                // 收 ACK：第 9 个 SCL 周期，主设备释放 SDA，从机拉低表示 ACK
                S_ACK1, S_ACK2, S_ACK3: begin
                    if (q == 2'd0) begin
                        scl_r  <= 1'b0;
                        sda_oe <= 1'b0;         // ★ 释放 SDA，让从机驱动
                    end else if (q == 2'd2 && qend) begin
                        scl_r <= 1'b1;          // 拉高 SCL（非阻塞，下一拍才生效）
                    end else if (q == 2'd3 && qend) begin
                        // ★★ 采样点必须在【SCL 已经为高】的时候 ★★
                        //   scl_r 是 q2 末才置 1 的，也就是说到了 q3 这一拍
                        //   SCL 才真的是高电平 —— 从机的 ACK 在这一段才保证有效。
                        //   ★ 千万别在 q2 那一拍采（第一版就是这么写的）：
                        //     那时 scl_r <= 1 还没生效、SCL 仍是低，
                        //     采到的不是 ACK，而是从机在低电平期间的空闲值。
                        //   高 = NACK。
                        if (sda) nack <= 1'b1;
                        scl_r <= 1'b0;
                        sda_oe <= 1'b1;
                        sda_out <= 1'b0;
                        // 下一步：地址的 ACK 后发寄存器地址；寄存器地址的 ACK 后
                        // 发数据；数据的 ACK 后收尾。
                        if (st == S_ACK1) begin
                            shreg <= reg_a;
                            st    <= S_REG;
                        end else if (st == S_ACK2) begin
                            shreg <= dat;
                            st    <= S_DATA;
                        end else begin
                            st <= S_STOP;
                        end
                    end
                end
                //----------------------------------------------------------
                // STOP：★ SCL 必须【全程保持高】，SDA 才允许由低变高 ★
                //   （第一版这里有个 bug：SCL 只在 q1 拉高、q2 又被那句默认的
                //     `scl_r <= 1'b0` 拉回去了，于是释放 SDA 的时刻 SCL 是低的 ——
                //     那不构成 STOP，从机不会结束事务。仿真抓出来的。）
                //----------------------------------------------------------
                S_STOP: begin
                    if (q == 2'd0) begin
                        scl_r   <= 1'b0;        // 先拉低，把 SDA 建立成 0
                        sda_oe  <= 1'b1;
                        sda_out <= 1'b0;
                    end else if (q == 2'd1) begin
                        scl_r <= 1'b1;
                    end else if (q == 2'd2 && qend) begin
                        scl_r  <= 1'b1;         // ★ 保持高
                        sda_oe <= 1'b0;         // ★ SCL 高时释放 SDA = STOP
                    end else if (q == 2'd3 && qend) begin
                        scl_r   <= 1'b1;        // ★ 保持高
                        sda_oe  <= 1'b1;
                        sda_out <= 1'b1;
                        st      <= S_DONE;
                    end
                end
                //----------------------------------------------------------
                S_DONE: begin
                    scl_r <= 1'b1;
                    sda_oe  <= 1'b1;
                    sda_out <= 1'b1;
                    busy  <= 1'b0;
                    done  <= 1'b1;             // 单拍脉冲
                    st    <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
