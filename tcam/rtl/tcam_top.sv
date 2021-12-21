`define OPENRAM 


module tcam_top 
(
  input logic clk_i,
  input logic rst_ni,

  input  logic         csb_i,
  input  logic         web_i,
  input  logic [3:0]   wmask_i,
  input  logic [31:0]  addr_i,
  input  logic [31:0]  wdata_i,
  output logic [31:0]  rdata_o

);

  logic        csbn;
  logic        webn;
  logic [3:0]  wmaskn;
  logic [27:0] addrn;
  logic [31:0] wdatan;
  logic [5:0]  rdata;
 
`ifdef OPENRAM
  always_ff @ (negedge clk_i or negedge rst_ni) begin
    if(!rst_ni) begin
      csbn   <= 1'b1;
      webn   <= 1'b0;
      wmaskn <= 4'b0;
      wdatan <= 32'b0;
      addrn  <= 32'b0;
    end else begin
      csbn   <= csb_i;
      webn   <= web_i;
      wmaskn <= wmask_i;
      wdatan <= wdata_i;
      addrn  <= addr_i[27:0];  
    end
  end
`else 

  always_comb begin
      csbn   = csb_i;
      webn   = web_i;
      wmaskn = wmask_i;
      wdatan = wdata_i;
      addrn  = addr_i; 
  end
  
`endif
  
  tcam_32x28 u_tcam(
  .clk_i	(clk_i),
  
  .csb_i	(csbn),
  .web_i	(webn),
  .wmask_i	(wmaskn),
  .addr_i	(addrn[27:0]),
  .wdata_i	(wdatan),
  .rdata_o	(rdata)
);

assign rdata_o = {26'b0, rdata};

endmodule
