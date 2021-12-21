module tcam_core
(
  input logic clk_i,
  input logic rst_ni,

// tl-ul insterface
  input tlul_pkg::tl_h2d_t tl_d_i,
  output tlul_pkg::tl_d2h_t tl_d_o

);

  logic        tl_req;
  logic [31:0] tl_wmask;
  logic        rvalid_o;
  logic [31:0] rdata;
  logic        re;
  
  logic        data_csb;
  logic [31:0] data_addr;
  logic [3:0]  data_wmask;
  logic        data_we;
  logic [31:0] data_wdata;
  
   assign data_wmask[0] = (tl_wmask[7:0]   != 8'b0) ? 1'b1: 1'b0;
   assign data_wmask[1] = (tl_wmask[15:8]  != 8'b0) ? 1'b1: 1'b0;
   assign data_wmask[2] = (tl_wmask[23:16] != 8'b0) ? 1'b1: 1'b0;
   assign data_wmask[3] = (tl_wmask[31:24] != 8'b0) ? 1'b1: 1'b0; 



tcam_top u_tcam_top(
  .clk_i	(clk_i),
  .rst_ni	(rst_ni),

  .csb_i	(~tl_req),
  .web_i	(~data_we),
  .wmask_i	(data_wmask),
  .addr_i	(data_addr),
  .wdata_i	(data_wdata),
  .rdata_o	(rdata)
);

  
 tlul_sram_adapter1 #(
   .SramAw       (32),
   .SramDw       (32), 
   .Outstanding  (4),  
   .ByteAccess   (1),
   .ErrOnWrite   (0),  
   .ErrOnRead    (0) 
 
 ) u_tcam_adapter (
     .clk_i    (clk_i),
     .rst_ni   (rst_ni),
     .tl_i     (tl_d_i),
     .tl_o     (tl_d_o), 
     .req_o    (tl_req),
     .gnt_i    (1'b1),
     .we_o     (data_we),
     .addr_o   (data_addr),
     .wdata_o  (data_wdata),
     .wmask_o  (tl_wmask),
     .rdata_i  (rdata), 
     .rvalid_i (rvalid_o),
     .rerror_i (2'b0)
 
 );



  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rvalid_o <= 1'b0;
    end else if (data_we) begin
      rvalid_o <= 1'b0;
    end else begin 
      rvalid_o <= tl_req;
    end
  end

endmodule