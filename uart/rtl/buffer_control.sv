
module buffer_control #(
  parameter int BUFFER_DEPTH = 256,
  parameter int BUFFER_WIDTH = 8,
  parameter int ADDR_WIDTH   = 8
)
(
  input logic clk_i,
  input logic rst_ni,
  
  input logic re_i,
  input logic we_i,
  input logic clr_i,
  input logic rst_i, 
  input logic [BUFFER_WIDTH-1:0] wdata_i,
  
  output logic buffer_full,
  output logic [BUFFER_WIDTH:0] rdata_o,
  
  output logic [ADDR_WIDTH:0] bsize_o
);
  
  logic [ADDR_WIDTH:0] raddr;
  logic [ADDR_WIDTH:0] waddr;
  logic msb_comp;
  logic pointer_equal;
  logic buffer_empty;
 // logic buffer_overflow;
  
  // write address
  always_ff @ (posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      waddr <= '0;
    end else begin
      if(clr_i) begin
	waddr <= '0;
      end else if(we_i) begin
	waddr <= waddr + 1;
      end else begin
	waddr <= waddr;
      end
    end
   end
  // read adderss
  always_ff @ (posedge clk_i or negedge rst_ni) begin
    if(!rst_ni) begin
      raddr <= '0;
    end else begin 
      if(rst_i) begin
	raddr <= '0;
      end else if (re_i & (~buffer_empty)) begin
	raddr <= raddr + 1;
      end else begin
	raddr <= raddr;
      end
    end
  end

  always_ff @ (posedge clk_i or negedge rst_ni) begin
    if(!rst_ni) begin
      buffer_full <= 1'b0;
    end else begin 
      if(clr_i) begin
	buffer_full <= 1'b0;
      end else if((waddr >= 8'hff) & we_i) begin
	buffer_full <= 1'b1;
      end else if(raddr > 8'h00) begin
	buffer_full <= 1'b0;
      end
    end
  end
  
  always_ff @ (posedge clk_i or negedge rst_ni) begin
    if(!rst_ni) begin
      buffer_empty <= 1'b0;
    end else begin
      if(rst_i) begin
	buffer_empty <= 1'b0;
      end else if((raddr >= 8'hff) & re_i) begin
	buffer_empty <= 1'b1;
      end 
    end
  end
  
  
  buffer_array #(
    .BUFFER_DEPTH  (BUFFER_DEPTH),
    .BUFFER_WIDTH  (BUFFER_WIDTH),
    .ADDR_WIDTH    (ADDR_WIDTH)
  ) fifo (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
  
    .re_i    (re_i & (~buffer_empty)),
    .we_i    (we_i & (~buffer_full)),
    .clr_i   (clr_i),
    .waddr_i (waddr[7:0]),
    .raddr_i (raddr),
    .wdata_i (wdata_i),
    .rdata_o (rdata_o)
  
);
  
  assign bsize_o = waddr;
  
endmodule 