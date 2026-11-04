module uart(
    input clk,
    input rst,
    //tx
    input  [7: 0] wdata_i,
    input         wvalid_i,
    output        wready_o,
    output        tx_o,

    //rx
    input         rx_i,
    input         rready_i,
    output        rvalid_o,
    output [7: 0] rdata_o

);

//波特率
`define baud_4800    16'd5624
`define baud_9600    16'd2812
`define baud_19200   16'd1406
`define baud_38400   16'd703
`define baud_57600   16'd468
`define baud_115200  16'd234

`define parity_none  2'd0
`define parity_odd   2'd1
`define parity_even  2'd2

uart_tx tx_inst (
    .clk        (clk ),
    .rst        (rst),
    .wdata_i    (wdata_i),
    .wvalid_i   (wvalid_i),
    .parity_i   (`parity_none),
    .baud_i     (`baud_115200),
    .wready_o   (wready_o),
    .tx_o       (tx_o)
);

uart_rx rx_inst (
    .clk        (clk),
    .rst        (rst),
    .rx_i       (rx_i),
    .parity_i   (`parity_none),
    .baud_i     (`baud_115200),
    .rready_i   (rready_i),
    .rvalid_o   (rvalid_o),
    .rdata_o    (rdata_o)
);

endmodule