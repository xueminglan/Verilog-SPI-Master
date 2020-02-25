////////////////////////////////////////////////////////////////////////////////
//  File name     :  axi4_master_adapter.v
//  Project       :  AXI4 infrastructure
//  Description   :  AXI4 master adapter
//  Created by    :  Henry
//  Create Date   :  2012-10-15
////////////////////////////////////////////////////////////////////////////////
//                                History
////////////////////////////////////////////////////////////////////////////////
//  Revision 0.1  2012/10/15  Henry / Initial revision
//  Revision 0.2  2012/10/20  Henry / Some fix and more feature
//  Revision 0.3  2014/01/02  Henry / Change to sync reset
////////////////////////////////////////////////////////////////////////////////

// Features:
// 1. Low latency, simple and high speed
// 2. Constant burst size, cache and protect
// 3. Not support outstanding transfer

module axi4_master_adapter #(
	parameter DATA_WIDTH = 32, // default data width is 32, can be 8, 16, 32 ... 1024
	parameter ADDR_WIDTH = 32, // default address width is 32, can be 1, 2 ...
	parameter LENGTH_WIDTH = 8, // default burst length width is 8, can be 4 or 8
	parameter ID_WIDTH = 4, // default ID width is 4, can be 1-16
	parameter MASTER_ID = 0, // default ID is 0
	parameter TIMEOUT_WIDTH = 0 // timeout counter width, can be 0(no timeout), 1, 2, 3 ...
) (
	// AXI4_master
	output                    m_axi_aclk, // global
	output                    m_axi_aresetn,
	output [    ID_WIDTH-1:0] m_axi_awid, // write address and control
	output [  ADDR_WIDTH-1:0] m_axi_awaddr,
	output [LENGTH_WIDTH-1:0] m_axi_awlen,
	output [             2:0] m_axi_awsize,
	output [             1:0] m_axi_awburst,
	output [             3:0] m_axi_awcache,
	output [             2:0] m_axi_awprot,
	output                    m_axi_awvalid,
	input                     m_axi_awready,
	output [  DATA_WIDTH-1:0] m_axi_wdata, // write data
	output [DATA_WIDTH/8-1:0] m_axi_wstrb,
	output                    m_axi_wlast,
	output                    m_axi_wvalid,
	input                     m_axi_wready,
	input  [    ID_WIDTH-1:0] m_axi_bid, // write response
	input  [             1:0] m_axi_bresp,
	input                     m_axi_bvalid,
	output                    m_axi_bready,
	output [    ID_WIDTH-1:0] m_axi_arid, // read address and control
	output [  ADDR_WIDTH-1:0] m_axi_araddr,
	output [LENGTH_WIDTH-1:0] m_axi_arlen,
	output [             2:0] m_axi_arsize,
	output [             1:0] m_axi_arburst,
	output [             3:0] m_axi_arcache,
	output [             2:0] m_axi_arprot,
	output                    m_axi_arvalid,
	input                     m_axi_arready,
	input  [    ID_WIDTH-1:0] m_axi_rid, // read data
	input  [  DATA_WIDTH-1:0] m_axi_rdata,
	input  [             1:0] m_axi_rresp,
	input                     m_axi_rlast,
	input                     m_axi_rvalid,
	output                    m_axi_rready,
	// master_local_bus
	input                     m_clk,
	input                     m_rst,
	input                     m_write, // write command ------
    input  [             1:0] m_wmode, // write address mode, 00: fixed, 01: incr, 10: wrap
	input  [  ADDR_WIDTH-1:0] m_waddr, // write start address, ignore lower bits
	input  [LENGTH_WIDTH-1:0] m_wlen, // write burst length
	output                    m_wdata_req, // write data request
	input  [  DATA_WIDTH-1:0] m_wdata, // write data
	input  [DATA_WIDTH/8-1:0] m_wbe, // write byte enable
	output                    m_wbusy, // write busy
	output                    m_wtimeout, // write time out
	output                    m_werror, // write error
	input                     m_read, // read command ------
    input  [             1:0] m_rmode, // read address mode, 00: fixed, 01: incr, 10: wrap
	input  [  ADDR_WIDTH-1:0] m_raddr, // read start address, ignore lower bits
	input  [LENGTH_WIDTH-1:0] m_rlen, // read burst length
	output                    m_rvalid, // read data valid
	output [  DATA_WIDTH-1:0] m_rdata, // read data
	output                    m_rbusy, // read busy
	output                    m_rtimeout, // read time out
	output                    m_rerror // read error
);

// write state define
localparam WRITE_IDLE = 2'h0;
localparam WRITE_CMD = 2'h1;
localparam WRITE_DATA = 2'h2;
localparam WRITE_RES = 2'h3;
// read state define
localparam READ_IDLE = 2'h0;
localparam READ_CMD = 2'h1;
localparam READ_DATA = 2'h2;

reg [             1:0] write_state = WRITE_IDLE;
reg [LENGTH_WIDTH-1:0] write_cnt = 0;
reg [             1:0] awburst_reg = 2'b0;
reg [  ADDR_WIDTH-1:0] awaddr_reg = 0;
reg [LENGTH_WIDTH-1:0] awlen_reg = 0;
reg                    awvalid_reg = 1'b0;
reg                    wvalid_reg = 1'b0;
reg                    bready_reg = 1'b0;
reg                    wbusy = 1'b1;
reg [ TIMEOUT_WIDTH:0] write_time_cnt = 0;
reg                    wtimeout = 1'b0;
reg                    werror = 1'b0;

reg [             1:0] read_state = READ_IDLE;
reg [LENGTH_WIDTH-1:0] read_cnt = 0;
reg [             1:0] arburst_reg = 2'b0;
reg [  ADDR_WIDTH-1:0] araddr_reg = 0;
reg [LENGTH_WIDTH-1:0] arlen_reg = 0;
reg                    arvalid_reg = 1'b0;
reg                    rbusy = 1'b1;
reg [ TIMEOUT_WIDTH:0] read_time_cnt = 0;
reg                    rtimeout = 1'b0;
reg                    rerror = 1'b0;

assign m_axi_aclk = m_clk;
assign m_axi_aresetn = ~m_rst;

assign m_axi_awid = MASTER_ID;
assign m_axi_awaddr = awaddr_reg;
assign m_axi_awlen = awlen_reg;
assign m_axi_awsize = $clog2(DATA_WIDTH/8);
assign m_axi_awburst = awburst_reg;
assign m_axi_awcache = 4'h3; // bufferable + modifiable
assign m_axi_awprot = 3'h0; // normal + secure + data access
assign m_axi_awvalid = awvalid_reg;
assign m_axi_wdata = m_wdata;
assign m_axi_wstrb = m_wbe;
assign m_axi_wlast = wvalid_reg & (write_cnt == 0);
assign m_axi_wvalid = wvalid_reg;
assign m_axi_bready = bready_reg;

assign m_axi_arid = MASTER_ID;
assign m_axi_araddr = araddr_reg;
assign m_axi_arlen = arlen_reg;
assign m_axi_arsize = $clog2(DATA_WIDTH/8);
assign m_axi_arburst = arburst_reg;
assign m_axi_arcache = 4'h3; // bufferable + modifiable
assign m_axi_arprot = 3'h0; // normal + secure + data access
assign m_axi_arvalid = arvalid_reg;
assign m_axi_rready = 1'b1;

assign m_wdata_req = (m_axi_awvalid & m_axi_awready) | 
					 (m_axi_wvalid & m_axi_wready & (write_cnt != 0));
assign m_rvalid = m_axi_rvalid & m_axi_rready;
assign m_rdata = m_axi_rdata;

assign m_wbusy = wbusy;
assign m_wtimeout = wtimeout;
assign m_werror = werror;

assign m_rbusy = rbusy;
assign m_rtimeout = rtimeout;
assign m_rerror = rerror;

// write state machine
always @ (posedge m_clk)
begin
	if(m_rst)
	begin
		write_state <= WRITE_IDLE;
	end
	else
	begin
		case(write_state)
			WRITE_IDLE:
			begin
				if(m_write)
					write_state <= WRITE_CMD;
			end
			WRITE_CMD:
			begin
				if(m_axi_awready)
					write_state <= WRITE_DATA;
				else if(wtimeout)
					write_state <= WRITE_IDLE;
			end
			WRITE_DATA:
			begin
				if(write_cnt == 0 && m_axi_wready && m_axi_wvalid)
					write_state <= WRITE_RES;
				else if(wtimeout)
					write_state <= WRITE_IDLE;
			end
			WRITE_RES:
			begin
				if(m_axi_bvalid || wtimeout)
					write_state <= WRITE_IDLE;
			end
		endcase
	end
end

always @ (posedge m_clk)
begin
	if(m_rst)
	begin
		write_cnt <= 0;
		awburst_reg <= 2'h0;
		awaddr_reg <= 0;
		awlen_reg <= 0;
		awvalid_reg <= 1'b0;
		wvalid_reg <= 1'b0;
		bready_reg <= 1'b0;
		
		wbusy <= 1'b1;
		write_time_cnt <= 0;
		wtimeout <= 1'b0;
		werror <= 1'b0;
	end
	else
	begin
		case(write_state)
			WRITE_IDLE:
			begin
				awburst_reg <= m_wmode;
				awaddr_reg <= {m_waddr[ADDR_WIDTH-1:$clog2(DATA_WIDTH/8)], 
							   {log2(DATA_WIDTH/8){1'b0}}};
				awlen_reg <= m_wlen;
				awvalid_reg <= m_write;
				write_cnt <= m_wlen;
				wvalid_reg <= 1'b0;
				bready_reg <= 1'b1;
				
				wbusy <= m_write;
				write_time_cnt <= 0;
				wtimeout <= 1'b0;
				werror <= 1'b0;
			end
			WRITE_CMD:
			begin
				if(m_axi_awready)
				begin
					awvalid_reg <= 1'b0;
				end
				write_cnt <= write_cnt;
				wvalid_reg <= 1'b0;
				bready_reg <= 1'b0;
				
				wbusy <= 1'b1;
				if(m_axi_awready)
					write_time_cnt <= 0;
				else write_time_cnt <= write_time_cnt + 1'b1;
				wtimeout <= &write_time_cnt && (TIMEOUT_WIDTH != 0);
				werror <= 1'b0;
			end
			WRITE_DATA:
			begin
				awvalid_reg <= 1'b0;
				if(m_axi_wready && m_axi_wvalid)
					write_cnt <= write_cnt - 1'b1;
				
				if(write_cnt == 0 && m_axi_wready && m_axi_wvalid)
				begin
					wvalid_reg <= 1'b0;
					bready_reg <= 1'b1;
				end
				else
				begin
					wvalid_reg <= 1'b1;
					bready_reg <= 1'b0;
				end
				
				wbusy <= 1'b1;
				if(m_axi_wready)
					write_time_cnt <= 0;
				else write_time_cnt <= write_time_cnt + 1'b1;
				wtimeout <= &write_time_cnt && (TIMEOUT_WIDTH != 0);
				werror <= 1'b0;
			end
			WRITE_RES:
			begin
				awvalid_reg <= 1'b0;
				write_cnt <= write_cnt;
				wvalid_reg <= 1'b0;
				bready_reg <= ~m_axi_bvalid;
				
				wbusy <= ~m_axi_bvalid;
				write_time_cnt <= write_time_cnt + 1'b1;
				wtimeout <= &write_time_cnt && (TIMEOUT_WIDTH != 0);
				werror <= (m_axi_bresp != 2'h0 || m_axi_bid != MASTER_ID) & m_axi_bvalid;
			end
		endcase
	end
end

// read state machine
always @ (posedge m_clk)
begin
	if(m_rst)
	begin
		read_state <= READ_IDLE;
	end
	else
	begin
		case(read_state)
			READ_IDLE:
			begin
				if(m_read)
					read_state <= READ_CMD;
			end
			READ_CMD:
			begin
				if(m_axi_arready)
					read_state <= READ_DATA;
				else if(rtimeout)
					read_state <= READ_IDLE;
			end
			READ_DATA:
			begin
				if((read_cnt == 0 && m_rvalid) || rtimeout)
					read_state <= READ_IDLE;
			end
			default:
			begin
				read_state <= READ_IDLE;
			end
		endcase
	end
end

always @ (posedge m_clk)
begin
	if(m_rst)
	begin
		read_cnt <= 0;
		arburst_reg <= 2'h0;
		araddr_reg <= 0;
		arlen_reg <= 0;
		arvalid_reg <= 1'b0;
		
		rbusy <= 1'b1;
		read_time_cnt <= 0;
		rtimeout <= 1'b0;
		rerror <= 1'b0;
	end
	else
	begin
		case(read_state)
			default:
			begin
				arburst_reg <= m_rmode;
				araddr_reg <= {m_raddr[ADDR_WIDTH-1:$clog2(DATA_WIDTH/8)], 
							   {log2(DATA_WIDTH/8){1'b0}}};
				arlen_reg <= m_rlen;
				arvalid_reg <= m_read;
				read_cnt <= m_rlen;
				
				rbusy <= m_read;
				read_time_cnt <= 0;
				rtimeout <= 1'b0;
				rerror <= 1'b0;
			end
			READ_CMD:
			begin
				if(m_axi_arready)
					arvalid_reg <= 1'b0;
				read_cnt <= read_cnt;
				
				rbusy <= 1'b1;
				if(m_axi_arready)
					read_time_cnt <= 0;
				else read_time_cnt <= read_time_cnt + 1'b1;
				rtimeout <= &read_time_cnt && (TIMEOUT_WIDTH != 0);
				rerror <= 1'b0;
			end
			READ_DATA:
			begin
				arvalid_reg <= 1'b0;
				if(m_rvalid)
					read_cnt <= read_cnt - 1'b1;
				
				rbusy <= ~((read_cnt == 0) & m_rvalid);
				if(m_axi_rvalid)
					read_time_cnt <= 0;
				else read_time_cnt <= read_time_cnt + 1'b1;
				rtimeout <= &read_time_cnt && (TIMEOUT_WIDTH != 0);
				rerror <= ((m_axi_rresp != 2'h0 || m_axi_rid != MASTER_ID) & m_axi_rvalid)
							| (m_axi_rlast & (read_cnt != 0));
			end
		endcase
	end
end

// log2
function [31:0] log2;
input [31:0] din;
begin
    log2 = 0;
    din = din >> 1;
    while(din)
    begin
        din = din >> 1;
        log2 = log2 + 1;
    end
end
endfunction

endmodule
