`define MPRJ_IO_PADS 38


module rest_top (
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

  assign io_oeb[6]     = 1'b0;
  assign io_oeb[5]     = 1'b1;
  assign io_oeb[8]     = 1'b1;
  assign io_oeb[33:24] = 10'b1;
  assign io_oeb[34]    = 1'b0;
  
  assign io_out[33:24] = 10'b0;
  assign io_out[5]     = 1'b0;
  assign io_out[8]     = 1'b0;
  
  logic rx;
  logic tx;
  
  assign io_out[34] = tx;
  assign io_oeb[37] = 1'b0;
  assign io_oeb[34] = 1'b0;
  assign io_out[6]  = tx;
  
  assign rx = io_in[35] ? io_in[5] : io_in[36];
  
  assign io_oeb[35] = 1'b1;
  assign io_out[35] = 1'b0;
  assign io_oeb[36] = 1'b1;
  assign io_out[36] = 1'b0;
  
azadi_soc_top u_soc(
`ifdef USE_POWER_PINS
   .vccd1	(),
   .vssd1	(),
`endif
  .clk_i	(wb_clk_i),
  .rst_ni	(~wb_rst_i),
  .prog		(io_in[8]),
  .d_up		(io_out[37]),
  .uart_init    (io_in[32]),
  .uart_init_rx (io_in[33]),
  .baud_sel     (io_in[31:24]),
  
  .clks_per_bit	(la_data_in[15:0]), 

  // uart-periph interface
  .uart_tx	(tx),
  .uart_rx	(rx)

);

endmodule	// user_project_wrapper