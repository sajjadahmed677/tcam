
module azadi_soc_top (
`ifdef USE_POWER_PINS
   inout vccd1,
   inout vssd1,
`endif
  input logic clk_i,
  input logic rst_ni,
  input wire prog,
  
  input  logic [15:0] clks_per_bit, 

  // uart-periph interface
  
  output logic       uart_tx,
  input  logic       uart_rx

);

  logic prog_rst_n;
  logic system_rst_ni;
  logic intr_u_rx;
  logic intr_u_tx;
  logic        prog_uart_rx;
  logic         instr_valid;
  logic [11:0]  tlul_addr;
  logic         req_i;
  logic [31:0]  tlul_data;

  assign prog_uart_rx = uart_rx; 

 // instruction sram interface 
  logic        instr_csb;
  logic [11:0] instr_addr;
  logic [31:0] instr_wdata;
  logic [3:0]  instr_wmask;
  logic        instr_we;
  logic [31:0] instr_rdata;
  
   // data sram interface
  logic        data_csb;
  logic [11:0] data_addr;
  logic [31:0] data_wdata;
  logic [3:0]  data_wmask;
  logic        data_we;
  logic [31:0] data_rdata;
  
  logic [31:0] iccm_ctrl_data;
  logic        iccm_ctrl_we;
  logic [11:0] iccm_ctrl_addr_o;

        
  tlul_pkg::tl_h2d_t ifu_to_xbar;
  tlul_pkg::tl_d2h_t xbar_to_ifu;
  tlul_pkg::tl_h2d_t xbar_to_iccm;
  tlul_pkg::tl_d2h_t iccm_to_xbar;

  tlul_pkg::tl_h2d_t lsu_to_xbar;
  tlul_pkg::tl_d2h_t xbar_to_lsu;

  tlul_pkg::tl_h2d_t xbar_to_dccm;
  tlul_pkg::tl_d2h_t dccm_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_tcam;
  tlul_pkg::tl_d2h_t tcam_to_xbar;

  tlul_pkg::tl_h2d_t xbar_to_uart;
  tlul_pkg::tl_d2h_t uart_to_xbar;



brq_core_top #(
    .PMPEnable        (1'b0),
    .PMPGranularity   (0), 
    .PMPNumRegions    (4), 
    .MHPMCounterNum   (0), 
    .MHPMCounterWidth (40), 
    .RV32E            (1'b0), 
    .RV32M            (ibex_pkg::RV32MSlow), 
    .RV32B            (ibex_pkg::RV32BNone), 
    .RegFile          (ibex_pkg::RegFileFF), 
    .BranchTargetALU  (1'b1), 
    .WritebackStage   (1'b1), 
    .ICache           (1'b0), 
    .ICacheECC        (1'b0), 
    .BranchPredictor  (1'b0), 
    .DbgTriggerEn     (1'b1), 
    .DbgHwBreakNum    (1), 
    .Securebrq        (1'b0),
    .DmHaltAddr       ('0), 
    .DmExceptionAddr  ('0) 
) u_top (
    .clk_i (clk_i),
    .rst_ni (system_rst_ni),

  // instruction memory interface 
    .tl_i_i (xbar_to_ifu),
    .tl_i_o (ifu_to_xbar),

  // data memory interface 
    .tl_d_i (xbar_to_lsu),
    .tl_d_o (lsu_to_xbar),

    //.test_en_i   (1'b0),     // enable all clk_i gates for testing

    .hart_id_i   (32'b0), 
    .boot_addr_i (32'h20000000),

        // Interrupt inputs
    .irq_software_i (1'b0),
    .irq_timer_i    (1'b0),
    .irq_external_i (intr_u_rx | intr_u_tx),
    .irq_fast_i     ('0),
    .irq_nm_i       (1'b0),       // non-maskeable interrupt

    // Debug Interface
    .debug_req_i    ('0),
    // CPU Control Signals
    .fetch_enable_i (1'b1),
    .alert_minor_o  (),
    .alert_major_o  (),
    .core_sleep_o   ()
);


// main xbar module
  tl_xbar_main main_swith (
  .clk_i         (clk_i),
  .rst_ni        (system_rst_ni),

  // Host interfaces
  .tl_brqif_i         (ifu_to_xbar),
  .tl_brqif_o         (xbar_to_ifu),
  .tl_brqlsu_i        (lsu_to_xbar),
  .tl_brqlsu_o        (xbar_to_lsu),

  // Device interfaces
  .tl_iccm_o          (xbar_to_iccm),
  .tl_iccm_i          (iccm_to_xbar),
  .tl_dccm_o          (xbar_to_dccm),
  .tl_dccm_i          (dccm_to_xbar),
  .tl_tcam_o          (xbar_to_tcam),
  .tl_tcam_i          (tcam_to_xbar),
  .tl_uart_o          (xbar_to_uart),
  .tl_uart_i          (uart_to_xbar)
);


rstmgr reset_manager(
  .clk_i  (clk_i),
  .rst_ni (rst_ni),
  //.prog_i (prog),
  .prog_rst_ni(prog_rst_ni),
  .sys_rst_ni(system_rst_ni)
);

tcam_core u_tcam(
  .clk_i	(clk_i),
  .rst_ni	(system_rst_ni),

// tl-ul insterface
  .tl_d_i	(xbar_to_tcam),
  .tl_d_o	(tcam_to_xbar)

);

 uart_top  u_uart(

    .clk_i  (clk_i),
    .rst_ni (system_rst_ni),
    
    .tl_i   (xbar_to_uart),
    .tl_o   (uart_to_xbar),
    
    .tx_o   (uart_tx),
    .rx_i   (uart_rx),
    
    .intr_tx (intr_u_tx),
    .intr_rx (intr_u_rx)
);

logic rx_dv_i;
logic [7:0] rx_byte_i;
	
iccm_controller u_dut(
    .clk_i      (clk_i),
	.rst_ni     (rst_ni),
	.prog_i     (prog),
	.rx_dv_i    (rx_dv_i),
	.rx_byte_i  (rx_byte_i),
	.we_o       (iccm_ctrl_we),
	.addr_o     (iccm_ctrl_addr_o),
	.wdata_o    (iccm_ctrl_data),
	.reset_o    (prog_rst_ni)
);
	
uart_rx_prog u_uart_rx_prog(
	.clk_i         (clk_i),
	.rst_ni        (rst_ni),
	.i_Rx_Serial   (prog_uart_rx),
	.CLKS_PER_BIT  (clks_per_bit),
	.o_Rx_DV       (rx_dv_i),
	.o_Rx_Byte     (rx_byte_i)
);


// dummy instruction memory
instr_mem_top iccm_adapter(
  .clk_i            (clk_i),
  .rst_ni           (system_rst_ni),
  
  .tl_i             (xbar_to_iccm),
  .tl_o             (iccm_to_xbar),
// iccm controller interface 
  .iccm_ctrl_addr   (iccm_ctrl_addr_o),
  .iccm_ctrl_wdata  (iccm_ctrl_data),
  .iccm_ctrl_we     (iccm_ctrl_we),
  .prog_rst_ni      (prog_rst_ni),
    

// instruction sram interface 
  .csb              (instr_csb),
  .addr_o           (instr_addr),
  .wdata_o          (instr_wdata),
  .wmask_o          (instr_wmask),
  .we_o             (instr_we),
  .rdata_i          (instr_rdata)
);


sram_32x1024 u_iccm(
  .clk_i	(clk_i),
  .rst_ni	(system_rst_ni),
  
  .csb_i	(instr_csb),
  .web_i	(instr_we),
  .wmask_i	(instr_wmask),
  .addr_i	(instr_addr),
  .wdata_i	(instr_wdata),
  .rdata_o	(instr_rdata)
);
    
// dummy data memory

data_mem_top dccm_adapter(
  .clk_i    (clk_i),
  .rst_ni   (system_rst_ni),

// tl-ul insterface
  .tl_d_i   (xbar_to_dccm),
  .tl_d_o   (dccm_to_xbar),
  
  // sram interface
   .csb     (data_csb),
   .addr_o  (data_addr[9:0]),
   .wdata_o (data_wdata),
   .wmask_o (data_wmask),
   .we_o    (data_we),
   .rdata_i (data_rdata)
);


sram_32x1024 u_dccm(
  .clk_i	(clk_i),
  .rst_ni	(system_rst_ni),
  
  .csb_i	(data_csb),
  .web_i	(data_we),
  .wmask_i	(data_wmask),
  .addr_i	(data_addr),
  .wdata_i	(data_wdata),
  .rdata_o	(data_rdata)
);

endmodule
