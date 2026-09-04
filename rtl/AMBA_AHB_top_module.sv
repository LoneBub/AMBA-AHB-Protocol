module first_master(hgrant_m1,hready_m1,hresp_m1,hclk_m1,hrst_m1,hrdata_m1,hbusreq_m1,htrans_m1,haddr_m1,hburst_m1,hsize_m1,hwdata_m1,hwrite_m1);
input hgrant_m1;
input hready_m1;
input [1:0]hresp_m1;
input hclk_m1;
input hrst_m1;
input [31:0]hrdata_m1;
output reg hbusreq_m1;
output reg [1:0]htrans_m1;
output reg [31:0]haddr_m1;
output reg [2:0]hburst_m1,hsize_m1;
output reg [31:0]hwdata_m1;
output reg hwrite_m1;
parameter idle_mast_first = 4'b0000, busreq_mast_first = 4'b0001, non_seq_read_mast_first = 4'b0010, non_seq_write_mast_first = 4'b0011, seq_read_mast_first = 4'b0100,
seq_write_mast_first = 4'b0101,write_wait_mast_first = 4'b0110, read_wait_mast_first = 4'b0111,last_write_mast_first = 4'b1000, last_read_mast_first = 4'b1001;
parameter reg[2:0] burst_out_m1 = 3'b100;
parameter reg[31:0] start_addr_m1 = 32'h0000_0A00;
parameter reg[31:0] data_m1 = 32'h01a7_d24c;
parameter reg req_m1 = 1;
parameter reg write_m1 = 0;
reg transfer_start_req_m1 = 1;
int count_m1,burst_size_m1;
bit stay_m1;
reg [3:0]state_m1,next_state_m1;
reg [31:0]data_out_m1,addr_out_m1;
reg load_data_m1;   //difference between int and integer (study)

//Address function variables required
reg load_addr_m1;
reg [7:0]wrap_byte_var_m1;
reg [31:0]quotient_temp_m1,remainder_temp_m1,start_wrap_addr_m1,end_wrap_addr_m1;

//next data generation logic
always @(posedge hclk_m1 or negedge hrst_m1)
begin
if (!hrst_m1)
begin
    data_out_m1 <= data_m1;
end
else if (load_data_m1 && hready_m1)
begin
    data_out_m1 <= data_out_m1 + 32'h0000_0010;
end
end

// Next address generation logic
always @(posedge hclk_m1 or negedge hrst_m1)
begin
  if (!hrst_m1)
  addr_out_m1 = start_addr_m1;
  else if (load_addr_m1 && hready_m1)
  begin
      if (hburst_m1 == 3'b010 || hburst_m1 == 3'b100 || hburst_m1 == 3'b110)
      begin
          if (remainder_temp_m1 != 0)
          addr_out_m1 <= (addr_out_m1 == end_wrap_addr_m1)?start_wrap_addr_m1:((addr_out_m1 == start_addr_m1-4)?addr_out_m1:(addr_out_m1+4));
          else
          addr_out_m1 <= haddr_m1 == ((start_addr_m1 + wrap_byte_var_m1)-4)?addr_out_m1:(addr_out_m1+4);
      end
      else
      addr_out_m1 <= haddr_m1 == ((start_addr_m1 + wrap_byte_var_m1)-4)?addr_out_m1:(addr_out_m1+4); 
  end
  else 
  addr_out_m1 = addr_out_m1;
end

//address_generation calculation
always_comb
begin
wrap_byte_var_m1 = 4 * burst_size_m1;
remainder_temp_m1 = start_addr_m1 % wrap_byte_var_m1;
quotient_temp_m1 = start_addr_m1 / wrap_byte_var_m1;
start_wrap_addr_m1 = wrap_byte_var_m1 * quotient_temp_m1;
end_wrap_addr_m1 = start_wrap_addr_m1 + wrap_byte_var_m1 - 4;
end

// first master FSM Logic
always_comb
begin
case(state_m1)
idle_mast_first: begin
if (!hgrant_m1)
begin
	if(transfer_start_req_m1)
	next_state_m1 = busreq_mast_first;
	else
	next_state_m1 = idle_mast_first;
end
else if(hready_m1 && hresp_m1 == 2'b00 && transfer_start_req_m1)
next_state_m1 = (write_m1) ? non_seq_write_mast_first : non_seq_read_mast_first;
else
next_state_m1 = idle_mast_first;
end

busreq_mast_first: begin
if (hready_m1 && hresp_m1 == 2'b00)
begin
	if(hgrant_m1)
	next_state_m1 = non_seq_write_mast_first;
	else
	next_state_m1 = busreq_mast_first;
end
end

non_seq_write_mast_first: begin
if (hgrant_m1)
begin
	if(burst_size_m1 == 1)
	next_state_m1 = write_wait_mast_first;
	else
	next_state_m1 = seq_write_mast_first;
end
else
next_state_m1 = last_write_mast_first;
end

write_wait_mast_first: begin
if (hgrant_m1)
begin
	if(hready_m1 && hresp_m1 == 2'b00)
	next_state_m1 = idle_mast_first;
end
end

seq_write_mast_first: begin
if (hgrant_m1)
begin
	next_state_m1 = (count_m1 == burst_size_m1)? write_wait_mast_first : seq_write_mast_first;
	stay_m1 = (count_m1 == burst_size_m1) ? 0:1;
end
else
next_state_m1 = last_write_mast_first;
end

non_seq_read_mast_first: begin
if (hgrant_m1)
begin
	if(burst_size_m1 == 1)
	next_state_m1 = read_wait_mast_first;
	else
	next_state_m1 = seq_read_mast_first;
end
else
next_state_m1 <= last_read_mast_first;
end

read_wait_mast_first: begin
if (hgrant_m1)
begin
	if(hready_m1 && hresp_m1 == 2'b00)
	next_state_m1 = idle_mast_first;
end
end

seq_read_mast_first: begin
if (hgrant_m1)
begin
	stay_m1 = (count_m1 == burst_size_m1-1) ? 0:1;
	next_state_m1 = (count_m1 == burst_size_m1-1)? read_wait_mast_first : seq_read_mast_first;
	
end
else
next_state_m1 <= last_read_mast_first;
end

last_write_mast_first: begin
if (hgrant_m1)
next_state_m1 = non_seq_write_mast_first;
else
next_state_m1 = last_write_mast_first;
end

last_read_mast_first: begin
if (hgrant_m1)
next_state_m1 = non_seq_read_mast_first;
else
next_state_m1 = last_read_mast_first;
end

endcase
end

// Counter logic
always_ff @(posedge hclk_m1)
begin
if(stay_m1 && hready_m1)
count_m1 = count_m1 + 1;
else if(!stay_m1)
count_m1 <= 1;
end

always_ff @(posedge hclk_m1, negedge hrst_m1)
begin
if(!hrst_m1)
state_m1 <= idle_mast_first;
else
state_m1 <= next_state_m1;
end

always_comb
begin
if (state_m1 == idle_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b00;          hburst_m1 = burst_out_m1;   load_addr_m1 = 0;
hwrite_m1 = write_m1;       hbusreq_m1 = 0;
end
else if (state_m1 == busreq_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b00;          hburst_m1 = burst_out_m1;   load_addr_m1 = 0;
hwrite_m1 = write_m1;       hbusreq_m1 = req_m1;
end
else if (state_m1 == non_seq_write_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b10;          hburst_m1 = burst_out_m1;   load_addr_m1 = 1;
hwrite_m1 = write_m1;       hbusreq_m1 = req_m1;
end
else if (state_m1 == write_wait_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b10;          hburst_m1 = burst_out_m1;   load_addr_m1 = 0;
hwrite_m1 = write_m1;       hbusreq_m1 = req_m1;        hwdata_m1 = hwdata_m1;   transfer_start_req_m1 = 0;
end
else if (state_m1 == seq_write_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 1;
htrans_m1 = 2'b11;          hburst_m1 = burst_out_m1;   load_addr_m1 = 1;
hwrite_m1 = write_m1;       hbusreq_m1 = 0;             hwdata_m1 = data_out_m1;
end
else if (state_m1 == non_seq_read_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b10;          hburst_m1 = burst_out_m1;   load_addr_m1 = 1;
hwrite_m1 = write_m1;       hbusreq_m1 = 0;             hwdata_m1 = data_out_m1;
end
else if (state_m1 == read_wait_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b10;          hburst_m1 = burst_out_m1;   load_addr_m1 = 0;
hwrite_m1 = write_m1;       hbusreq_m1 = req_m1;        hwdata_m1 = data_out_m1;   transfer_start_req_m1 = 0;
end
else if (state_m1 == seq_read_mast_first) begin
haddr_m1 = addr_out_m1;     hsize_m1 = 3'b010;          load_data_m1 = 0;
htrans_m1 = 2'b11;          hburst_m1 = burst_out_m1;   load_addr_m1 = 1;
hwrite_m1 = write_m1;       hbusreq_m1 = 0;             hwdata_m1 = data_out_m1;
end
end

//burst_size generation in terms of beats
always_comb
begin
unique case(hburst_m1)
3'b001: burst_size_m1 = 6;
3'b010: burst_size_m1 = 4;
3'b011: burst_size_m1 = 4;
3'b100: burst_size_m1 = 8;
3'b101: burst_size_m1 = 8;
3'b110: burst_size_m1 = 16;
3'b111: burst_size_m1 = 16;
default: burst_size_m1 = 1;
endcase
end
endmodule

/////////////////////////////////////////// SECOND - MASTER /////////////////////////////////////

module second_master(hgrant_m2,hready_m2,hresp_m2,hclk_m2,hrst_m2,hrdata_m2,hbusreq_m2,htrans_m2,haddr_m2,hburst_m2,hsize_m2,hwdata_m2,hwrite_m2);
input hgrant_m2;
input hready_m2;
input [1:0]hresp_m2;
input hclk_m2;
input hrst_m2;
input [31:0]hrdata_m2;
output reg hbusreq_m2;
output reg [1:0]htrans_m2;
output reg [31:0]haddr_m2;
output reg [2:0]hburst_m2,hsize_m2;
output reg [31:0]hwdata_m2;
output reg hwrite_m2;
parameter idle_mast_second = 4'b0000, busreq_mast_second = 4'b0001, non_seq_read_mast_second = 4'b0010, non_seq_write_mast_second = 4'b0011, seq_read_mast_second = 4'b0100,
seq_write_mast_second = 4'b0101,write_wait_mast_second = 4'b0110, read_wait_mast_second = 4'b0111,last_write_mast_second = 4'b1000, last_read_mast_second = 4'b1001;
parameter reg[2:0] burst_out_m2 = 3'b100;
parameter reg[31:0] start_addr_m2 = 32'h0000_0210;
parameter reg[31:0] data_m2 = 32'h01a7_d24c;
parameter reg req_m2 = 1;
parameter reg write_m2 = 0;
reg transfer_start_req_m2 = 1;
int count_m2,burst_size_m2;
bit stay_m2;
reg [3:0]state_m2,next_state_m2;
reg [31:0]data_out_m2,addr_out_m2;
reg load_data_m2;

//Address function variables required
reg load_addr_m2;
reg [7:0]wrap_byte_var_m2;
reg [31:0]quotient_temp_m2,remainder_temp_m2,start_wrap_addr_m2,end_wrap_addr_m2;

//next data generation logic
always @(posedge hclk_m2 or negedge hrst_m2)
begin
if (!hrst_m2)
begin
    data_out_m2 <= data_m2;
end
else if (load_data_m2 && hready_m2)
begin
    data_out_m2 <= data_out_m2 + 32'h0000_0010;
end
end

// Next address generation logic
always @(posedge hclk_m2 or negedge hrst_m2)
begin
  if (!hrst_m2)
  addr_out_m2 = start_addr_m2;
  else if (load_addr_m2 && hready_m2)
  begin
      if (hburst_m2 == 3'b010 || hburst_m2 == 3'b100 || hburst_m2 == 3'b110)
      begin
          if (remainder_temp_m2 != 0)
          addr_out_m2 <= (addr_out_m2 == end_wrap_addr_m2)?start_wrap_addr_m2:((addr_out_m2 == start_addr_m2-4)?addr_out_m2:(addr_out_m2+4));
          else
          addr_out_m2 <= haddr_m2 == ((start_addr_m2 + wrap_byte_var_m2)-4)?addr_out_m2:(addr_out_m2+4);
      end
      else
      addr_out_m2 <= haddr_m2 == ((start_addr_m2 + wrap_byte_var_m2)-4)?addr_out_m2:(addr_out_m2+4); 
  end
  else 
  addr_out_m2 = addr_out_m2;
end

//address_generation calculation
always_comb
begin
wrap_byte_var_m2 = 4 * burst_size_m2;
remainder_temp_m2 = start_addr_m2 % wrap_byte_var_m2;
quotient_temp_m2 = start_addr_m2 / wrap_byte_var_m2;
start_wrap_addr_m2 = wrap_byte_var_m2 * quotient_temp_m2;
end_wrap_addr_m2 = start_wrap_addr_m2 + wrap_byte_var_m2 - 4;
end

// second master FSM Logic
always_comb
begin
case(state_m2)
idle_mast_second: begin
if (!hgrant_m2)
begin
	if(transfer_start_req_m2)
	next_state_m2 = busreq_mast_second;
	else
	next_state_m2 = idle_mast_second;
end
else if(hready_m2 && hresp_m2 == 2'b00 && transfer_start_req_m2)
next_state_m2 = (write_m2) ? non_seq_write_mast_second : non_seq_read_mast_second;
else
next_state_m2 = idle_mast_second;
end

busreq_mast_second: begin
if (hready_m2 && hresp_m2 == 2'b00)
begin
	if(hgrant_m2)
	next_state_m2 = non_seq_write_mast_second;
	else
	next_state_m2 = busreq_mast_second;
end
end

non_seq_write_mast_second: begin
if (hgrant_m2)
begin
	if(burst_size_m2 == 1)
	next_state_m2 = write_wait_mast_second;
	else
	next_state_m2 = seq_write_mast_second;
end
else
next_state_m2 = last_write_mast_second;
end

write_wait_mast_second: begin
if (hgrant_m2)
begin
	if(hready_m2 && hresp_m2 == 2'b00)
	next_state_m2 = idle_mast_second;
end
end

seq_write_mast_second: begin
if (hgrant_m2)
begin
	next_state_m2 = (count_m2 == burst_size_m2)? write_wait_mast_second : seq_write_mast_second;
	stay_m2 = (count_m2 == burst_size_m2) ? 0:1;
end
else
next_state_m2 = last_write_mast_second;
end

non_seq_read_mast_second: begin
if (hgrant_m2)
begin
	if(burst_size_m2 == 1)
	next_state_m2 = read_wait_mast_second;
	else
	next_state_m2 = seq_read_mast_second;
end
else
next_state_m2 <= last_read_mast_second;
end

read_wait_mast_second: begin
if (hgrant_m2)
begin
	if(hready_m2 && hresp_m2 == 2'b00)
	next_state_m2 = idle_mast_second;
end
end

seq_read_mast_second: begin
if (hgrant_m2)
begin
	stay_m2 = (count_m2 == burst_size_m2-1) ? 0:1;
	next_state_m2 = (count_m2 == burst_size_m2-1)? read_wait_mast_second : seq_read_mast_second;
	
end
else
next_state_m2 <= last_read_mast_second;
end

last_write_mast_second: begin
if (hgrant_m2)
next_state_m2 = non_seq_write_mast_second;
else
next_state_m2 = last_write_mast_second;
end

last_read_mast_second: begin
if (hgrant_m2)
next_state_m2 = non_seq_read_mast_second;
else
next_state_m2 = last_read_mast_second;
end

endcase
end

// Counter logic
always_ff @(posedge hclk_m2)
begin
if(stay_m2 && hready_m2)
count_m2 = count_m2 + 1;
else if(!stay_m2)
count_m2 <= 1;
end

always_ff @(posedge hclk_m2, negedge hrst_m2)
begin
if(!hrst_m2)
state_m2 <= idle_mast_second;
else
state_m2 <= next_state_m2;
end

always_comb
begin
if (state_m2 == idle_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b00;          hburst_m2 = burst_out_m2;   load_addr_m2 = 0;
hwrite_m2 = write_m2;       hbusreq_m2 = 0;
end
else if (state_m2 == busreq_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b00;          hburst_m2 = burst_out_m2;   load_addr_m2 = 0;
hwrite_m2 = write_m2;       hbusreq_m2 = req_m2;
end
else if (state_m2 == non_seq_write_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b10;          hburst_m2 = burst_out_m2;   load_addr_m2 = 1;
hwrite_m2 = write_m2;       hbusreq_m2 = req_m2;
end
else if (state_m2 == write_wait_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b10;          hburst_m2 = burst_out_m2;   load_addr_m2 = 0;
hwrite_m2 = write_m2;       hbusreq_m2 = req_m2;        hwdata_m2 = hwdata_m2;   transfer_start_req_m2 = 0;
end
else if (state_m2 == seq_write_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 1;
htrans_m2 = 2'b11;          hburst_m2 = burst_out_m2;   load_addr_m2 = 1;
hwrite_m2 = write_m2;       hbusreq_m2 = 0;             hwdata_m2 = data_out_m2;
end
else if (state_m2 == non_seq_read_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b10;          hburst_m2 = burst_out_m2;   load_addr_m2 = 1;
hwrite_m2 = write_m2;       hbusreq_m2 = 0;             hwdata_m2 = data_out_m2;
end
else if (state_m2 == read_wait_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b10;          hburst_m2 = burst_out_m2;   load_addr_m2 = 0;
hwrite_m2 = write_m2;       hbusreq_m2 = req_m2;        hwdata_m2 = data_out_m2;   transfer_start_req_m2 = 0;
end
else if (state_m2 == seq_read_mast_second) begin
haddr_m2 = addr_out_m2;     hsize_m2 = 3'b010;          load_data_m2 = 0;
htrans_m2 = 2'b11;          hburst_m2 = burst_out_m2;   load_addr_m2 = 1;
hwrite_m2 = write_m2;       hbusreq_m2 = 0;             hwdata_m2 = data_out_m2;
end
end

//burst_size generation in terms of beats
always_comb
begin
unique case(hburst_m2)
3'b001: burst_size_m2 = 6;
3'b010: burst_size_m2 = 4;
3'b011: burst_size_m2 = 4;
3'b100: burst_size_m2 = 8;
3'b101: burst_size_m2 = 8;
3'b110: burst_size_m2 = 16;
3'b111: burst_size_m2 = 16;
default: burst_size_m2 = 1;
endcase
end
endmodule

///////////////////////////////////////// THIRD - MASTER ////////////////////////////////////////////
module third_master(hgrant_m3,hready_m3,hresp_m3,hclk_m3,hrst_m3,hrdata_m3,hbusreq_m3,htrans_m3,haddr_m3,hburst_m3,hsize_m3,hwdata_m3,hwrite_m3);
input hgrant_m3;
input hready_m3;
input [1:0]hresp_m3;
input hclk_m3;
input hrst_m3;
input [31:0]hrdata_m3;
output reg hbusreq_m3;
output reg [1:0]htrans_m3;
output reg [31:0]haddr_m3;
output reg [2:0]hburst_m3,hsize_m3;
output reg [31:0]hwdata_m3;
output reg hwrite_m3;
parameter idle_mast_third = 4'b0000, busreq_mast_third = 4'b0001, non_seq_read_mast_third = 4'b0010, non_seq_write_mast_third = 4'b0011, seq_read_mast_third = 4'b0100,
seq_write_mast_third = 4'b0101,write_wait_mast_third = 4'b0110, read_wait_mast_third = 4'b0111,last_write_mast_third = 4'b1000, last_read_mast_third = 4'b1001;
parameter reg[2:0] burst_out_m3 = 3'b100;
parameter reg[31:0] start_addr_m3 = 32'h0000_0600;
parameter reg[31:0] data_m3 = 32'h01a7_d24c;
parameter reg req_m3 = 1;
parameter reg write_m3 = 0;
reg transfer_start_req_m3 = 1;
int count_m3,burst_size_m3;
bit stay_m3;
reg [3:0]state_m3,next_state_m3;
reg [31:0]data_out_m3,addr_out_m3;
reg load_data_m3;   //difference between int and integer (study)

//Address function variables required
reg load_addr_m3;
reg [7:0]wrap_byte_var_m3;
reg [31:0]quotient_temp_m3,remainder_temp_m3,start_wrap_addr_m3,end_wrap_addr_m3;

//next data generation logic
always @(posedge hclk_m3 or negedge hrst_m3)
begin
if (!hrst_m3)
begin
    data_out_m3 <= data_m3;
end
else if (load_data_m3 && hready_m3)
begin
    data_out_m3 <= data_out_m3 + 32'h0000_0010;
end
end

// Next address generation logic
always @(posedge hclk_m3 or negedge hrst_m3)
begin
  if (!hrst_m3)
  addr_out_m3 = start_addr_m3;
  else if (load_addr_m3 && hready_m3)
  begin
      if (hburst_m3 == 3'b010 || hburst_m3 == 3'b100 || hburst_m3 == 3'b110)
      begin
          if (remainder_temp_m3 != 0)
          addr_out_m3 <= (addr_out_m3 == end_wrap_addr_m3)?start_wrap_addr_m3:((addr_out_m3 == start_addr_m3-4)?addr_out_m3:(addr_out_m3+4));
          else
          addr_out_m3 <= haddr_m3 == ((start_addr_m3 + wrap_byte_var_m3)-4)?addr_out_m3:(addr_out_m3+4);
      end
      else
      addr_out_m3 <= haddr_m3 == ((start_addr_m3 + wrap_byte_var_m3)-4)?addr_out_m3:(addr_out_m3+4); 
  end
  else 
  addr_out_m3 = addr_out_m3;
end

//address_generation calculation
always_comb
begin
wrap_byte_var_m3 = 4 * burst_size_m3;
remainder_temp_m3 = start_addr_m3 % wrap_byte_var_m3;
quotient_temp_m3 = start_addr_m3 / wrap_byte_var_m3;
start_wrap_addr_m3 = wrap_byte_var_m3 * quotient_temp_m3;
end_wrap_addr_m3 = start_wrap_addr_m3 + wrap_byte_var_m3 - 4;
end

// third master FSM Logic
always_comb
begin
case(state_m3)
idle_mast_third: begin
if (!hgrant_m3)
begin
	if(transfer_start_req_m3)
	next_state_m3 = busreq_mast_third;
	else
	next_state_m3 = idle_mast_third;
end
else if(hready_m3 && hresp_m3 == 2'b00 && transfer_start_req_m3)
next_state_m3 = (write_m3) ? non_seq_write_mast_third : non_seq_read_mast_third;
else
next_state_m3 = idle_mast_third;
end

busreq_mast_third: begin
if (hready_m3 && hresp_m3 == 2'b00)
begin
	if(hgrant_m3)
	next_state_m3 = non_seq_write_mast_third;
	else
	next_state_m3 = busreq_mast_third;
end
end

non_seq_write_mast_third: begin
if (hgrant_m3)
begin
	if(burst_size_m3 == 1)
	next_state_m3 = write_wait_mast_third;
	else
	next_state_m3 = seq_write_mast_third;
end
else
next_state_m3 = last_write_mast_third;
end

write_wait_mast_third: begin
if (hgrant_m3)
begin
	if(hready_m3 && hresp_m3 == 2'b00)
	next_state_m3 = idle_mast_third;
end
end

seq_write_mast_third: begin
if (hgrant_m3)
begin
	next_state_m3 = (count_m3 == burst_size_m3)? write_wait_mast_third : seq_write_mast_third;
	stay_m3 = (count_m3 == burst_size_m3) ? 0:1;
end
else
next_state_m3 = last_write_mast_third;
end

non_seq_read_mast_third: begin
if (hgrant_m3)
begin
	if(burst_size_m3 == 1)
	next_state_m3 = read_wait_mast_third;
	else
	next_state_m3 = seq_read_mast_third;
end
else
next_state_m3 <= last_read_mast_third;
end

read_wait_mast_third: begin
if (hgrant_m3)
begin
	if(hready_m3 && hresp_m3 == 2'b00)
	next_state_m3 = idle_mast_third;
end
end

seq_read_mast_third: begin
if (hgrant_m3)
begin
	stay_m3 = (count_m3 == burst_size_m3-1) ? 0:1;
	next_state_m3 = (count_m3 == burst_size_m3-1)? read_wait_mast_third : seq_read_mast_third;
	
end
else
next_state_m3 <= last_read_mast_third;
end

last_write_mast_third: begin
if (hgrant_m3)
next_state_m3 = non_seq_write_mast_third;
else
next_state_m3 = last_write_mast_third;
end

last_read_mast_third: begin
if (hgrant_m3)
next_state_m3 = non_seq_read_mast_third;
else
next_state_m3 = last_read_mast_third;
end

endcase
end

// Counter logic
always_ff @(posedge hclk_m3)
begin
if(stay_m3 && hready_m3)
count_m3 = count_m3 + 1;
else if(!stay_m3)
count_m3 <= 1;
end

always_ff @(posedge hclk_m3, negedge hrst_m3)
begin
if(!hrst_m3)
state_m3 <= idle_mast_third;
else
state_m3 <= next_state_m3;
end

always_comb
begin
if (state_m3 == idle_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b00;          hburst_m3 = burst_out_m3;   load_addr_m3 = 0;
hwrite_m3 = write_m3;       hbusreq_m3 = 0;
end
else if (state_m3 == busreq_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b00;          hburst_m3 = burst_out_m3;   load_addr_m3 = 0;
hwrite_m3 = write_m3;       hbusreq_m3 = req_m3;
end
else if (state_m3 == non_seq_write_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b10;          hburst_m3 = burst_out_m3;   load_addr_m3 = 1;
hwrite_m3 = write_m3;       hbusreq_m3 = req_m3;
end
else if (state_m3 == write_wait_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b10;          hburst_m3 = burst_out_m3;   load_addr_m3 = 0;
hwrite_m3 = write_m3;       hbusreq_m3 = req_m3;        hwdata_m3 = hwdata_m3;   transfer_start_req_m3 = 0;
end
else if (state_m3 == seq_write_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 1;
htrans_m3 = 2'b11;          hburst_m3 = burst_out_m3;   load_addr_m3 = 1;
hwrite_m3 = write_m3;       hbusreq_m3 = 0;             hwdata_m3 = data_out_m3;
end
else if (state_m3 == non_seq_read_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b10;          hburst_m3 = burst_out_m3;   load_addr_m3 = 1;
hwrite_m3 = write_m3;       hbusreq_m3 = 0;             hwdata_m3 = data_out_m3;
end
else if (state_m3 == read_wait_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b10;          hburst_m3 = burst_out_m3;   load_addr_m3 = 0;
hwrite_m3 = write_m3;       hbusreq_m3 = req_m3;        hwdata_m3 = data_out_m3;   transfer_start_req_m3 = 0;
end
else if (state_m3 == seq_read_mast_third) begin
haddr_m3 = addr_out_m3;     hsize_m3 = 3'b010;          load_data_m3 = 0;
htrans_m3 = 2'b11;          hburst_m3 = burst_out_m3;   load_addr_m3 = 1;
hwrite_m3 = write_m3;       hbusreq_m3 = 0;             hwdata_m3 = data_out_m3;
end
end

//burst_size generation in terms of beats
always_comb
begin
unique case(hburst_m3)
3'b001: burst_size_m3 = 6;
3'b010: burst_size_m3 = 4;
3'b011: burst_size_m3 = 4;
3'b100: burst_size_m3 = 8;
3'b101: burst_size_m3 = 8;
3'b110: burst_size_m3 = 16;
3'b111: burst_size_m3 = 16;
default: burst_size_m3 = 1;
endcase
end
endmodule



/////////////// DEFAULT-MASTER ////////////////////////////

module default_master(hgrant_def,hready_def,hresp_def,hclk_def,hrst_def,hrdata_def,hbusreq_def,htrans_def,haddr_def,hburst_def,hsize_def,hwdata_def,hwrite_def);
input hgrant_def;
input hready_def;
input [1:0]hresp_def;
input hclk_def;
input hrst_def;
input [31:0]hrdata_def;
output reg hbusreq_def;
output reg [1:0]htrans_def;
output reg [31:0]haddr_def;
output reg [2:0]hburst_def,hsize_def;
output reg [31:0]hwdata_def;
output reg hwrite_def;
parameter idle_default_mast = 2'b00, transfer_default_mast = 2'b01;
reg [1:0]state_def,next_state_def;

// default master FSM Logic
always_comb
begin
case(state_def)
idle_default_mast: begin
if (hgrant_def)
next_state_def = transfer_default_mast;
else
next_state_def = idle_default_mast;
end
transfer_default_mast: begin
if (!hgrant_def)
next_state_def = idle_default_mast;
else
next_state_def = transfer_default_mast;
end
endcase
end

always_ff @(posedge hclk_def, negedge hrst_def)
begin
if(!hrst_def)
state_def <= idle_default_mast;
else
state_def <= next_state_def;
end

always_comb
begin
if (state_def == idle_default_mast) begin
haddr_def = 32'b0;     hsize_def = 3'b010;    hwdata_def = 32'b0;
htrans_def = 2'b00;    hburst_def = 3'b000;
hwrite_def = 1'b0;     hbusreq_def = 1'b1;
end
else if (state_def == transfer_default_mast) begin
haddr_def = 32'b0;     hsize_def = 3'b010;    hwdata_def = 32'b0;
htrans_def = 2'b00;    hburst_def = 3'b000;
hwrite_def = 1'b0;     hbusreq_def = 1'b0;
end
else
begin
haddr_def = 32'b0;     hsize_def = 3'b010;    hwdata_def = 32'b0;
htrans_def = 2'b00;    hburst_def = 3'b000;
hwrite_def = 1'b0;     hbusreq_def = 1'b1;
end
end
endmodule

//////////////////////// FIRST - SLAVE //////////////////////////////

module first_slave(haddr_s1,hclk_s1,hrst_s1,hwrite_s1,hmaster_s1,htrans_s1,hsize_s1,hsel_s1,hburst_s1,hready_s1,hwdata_s1,hready_out_s1,hrdata_s1,hresp_s1); 
input [31:0]haddr_s1,hwdata_s1;
input hclk_s1,hrst_s1,hwrite_s1;
input [3:0]hmaster_s1;
input [1:0]htrans_s1;
input [2:0]hsize_s1,hburst_s1;
input hsel_s1,hready_s1;
output reg hready_out_s1;
output reg [31:0]hrdata_s1;
output reg [1:0]hresp_s1;

reg [7:0]slave_mem_s1[0:1023];
reg [3:0]state_s1,next_state_s1;
reg stay_s1, load_data_s1, store_data_s1, load_addr_s1, load_resp_s1;
int count_s1, burst_size_s1;
reg [1:0]resp_out_s1;
reg [31:0]data_out_s1,data_mem_s1,address_in_s1;
parameter idle_slave_first = 4'b0000, seq_write_slave_first = 4'b0001, seq_read_slave_first = 4'b0010, read_wait_slave_first = 4'b0011, write_wait_slave_first = 4'b0100,
wait_state_slave_first = 4'b0101;
parameter reg [31:0]data_memory_s1 = 32'h1a5b_203b;

//data_generation
always_comb
begin
if(!hrst_s1)
begin
	data_mem_s1 = data_memory_s1;
	for(int i=512; i<1024; i=i+4)
	begin
		slave_mem_s1[i]   = data_mem_s1[31:24];
		slave_mem_s1[i+1] = data_mem_s1[23:16];
		slave_mem_s1[i+2] = data_mem_s1[15:8];
		slave_mem_s1[i+3] = data_mem_s1[7:0];
		data_mem_s1 = data_mem_s1 + 4;
	end
end
else if(store_data_s1 == 1)
begin
	slave_mem_s1[address_in_s1]   <= hwdata_s1[31:24];
	slave_mem_s1[address_in_s1+1] <= hwdata_s1[23:16];
	slave_mem_s1[address_in_s1+2] <= hwdata_s1[15:8];
	slave_mem_s1[address_in_s1+3] <= hwdata_s1[7:0];
end
end

always @(posedge hclk_s1)
begin
if(load_data_s1)
begin
	if(hready_s1)
	data_out_s1 <= ({slave_mem_s1[address_in_s1],slave_mem_s1[address_in_s1+1],slave_mem_s1[address_in_s1+2],slave_mem_s1[address_in_s1+3]});
end
end

///// address_loader
always_ff @(posedge hclk_s1)
begin
if(load_addr_s1)
	begin
	if(haddr_s1<512)
		begin
		if(hwrite_s1)
		  address_in_s1 <= haddr_s1;
	end
	else
	begin
		if(!hwrite_s1)
		 address_in_s1 <= haddr_s1;
	end
	end
end

//Response_generator
always @(posedge hclk_s1)
begin
if(load_resp_s1)
begin
	if(haddr_s1<512)
	begin
		if(hwrite_s1)
		  resp_out_s1 <= 2'b00;
		else
		  resp_out_s1 <= 2'b01;
	end
	else
	begin
		if(!hwrite_s1)
		  resp_out_s1 <= 2'b00;
		else
		  resp_out_s1 <= 2'b01;
	end
end
else
resp_out_s1 <= 2'b00;
end	

// first slave FSM Logic
always_comb
begin
case(state_s1)
idle_slave_first: begin
if (hready_s1)
begin
	if(hmaster_s1 == 4'b0000)
	next_state_s1 = idle_slave_first;
	else
	begin
		if(hsel_s1 && htrans_s1 != 2'b00)
		  next_state_s1 = hwrite_s1?((hburst_s1 == 3'b000)? write_wait_slave_first : seq_write_slave_first):((hburst_s1 == 3'b000)? read_wait_slave_first : seq_read_slave_first);
		else if(htrans_s1 == 2'b00)
		  next_state_s1 = idle_slave_first;
		else
		  next_state_s1 = idle_slave_first;
	end
end
end

write_wait_slave_first: begin
if(hmaster_s1 == 4'b0000)
 next_state_s1 = idle_slave_first;
else
 next_state_s1 = wait_state_slave_first;
end

seq_write_slave_first: begin
if (hmaster_s1 == 4'b0000)
 next_state_s1 = idle_slave_first;
else
begin
	stay_s1 = ((count_s1 == burst_size_s1-1) ? 0:1);
	next_state_s1 = ((count_s1 == burst_size_s1-1)? write_wait_slave_first : seq_write_slave_first);
end
end

read_wait_slave_first: begin
if(hmaster_s1 == 4'b0000)
 next_state_s1 = idle_slave_first;
else
 next_state_s1 = wait_state_slave_first;
end

seq_read_slave_first: begin
if (hmaster_s1 == 4'b0000)
 next_state_s1 = idle_slave_first;
else
begin
	stay_s1 = (count_s1 == burst_size_s1) ? 0:1;  //s1-1
	next_state_s1 = (count_s1 == burst_size_s1)? read_wait_slave_first : seq_read_slave_first;
end
end

wait_state_slave_first: begin
next_state_s1 = idle_slave_first;
end
endcase
end

// Counter logic
always_ff @(posedge hclk_s1)
begin
if(stay_s1)
count_s1 <= count_s1 + 1;
else
count_s1 <= 1;
end

always_ff @(posedge hclk_s1, negedge hrst_s1)
begin
if(!hrst_s1)
state_s1 <= idle_slave_first;
else
state_s1 <= next_state_s1;
end

always_comb
begin
if (state_s1 == idle_slave_first) begin
hready_out_s1 = 1;         store_data_s1 = 0;   //hrdata_s1 = 0;
hresp_s1 = resp_out_s1;    load_addr_s1  = 1;   
load_data_s1 = 0;          load_resp_s1  = 0;
end
else if (state_s1 == write_wait_slave_first) begin
hready_out_s1 = 1;         store_data_s1 = 1;   //hrdata_s1 = 0;
hresp_s1 = resp_out_s1;    load_addr_s1 = 0;
load_data_s1 = 0;          load_resp_s1 = 0;
end
else if (state_s1 == seq_write_slave_first) begin
hready_out_s1 = 1;         store_data_s1 = 1;   //hrdata_s1 = 0;
hresp_s1 = resp_out_s1;    load_addr_s1 = 1;   
load_data_s1 = 0;          load_resp_s1 = 1;
end
else if (state_s1 == read_wait_slave_first) begin
hready_out_s1 = 1;         store_data_s1 = 0;   hrdata_s1 = data_out_s1;
hresp_s1 = resp_out_s1;    load_addr_s1 = 0;   
load_data_s1 = 0;          load_resp_s1 = 0;
end
else if (state_s1 == seq_read_slave_first) begin
hready_out_s1 = 1;         store_data_s1 = 0;   hrdata_s1 = data_out_s1;
hresp_s1 = resp_out_s1;    load_addr_s1 = 1;   
load_data_s1 = 1;          load_resp_s1 = 1;
end
else if (state_s1 == wait_state_slave_first) begin
hready_out_s1 = 1;         store_data_s1 = 0;   hrdata_s1 = data_out_s1;
hresp_s1 = resp_out_s1;    load_addr_s1 = 0;   
load_data_s1 = 0;          load_resp_s1 = 0;
end
end

//burst_size generation in terms of beats
always_comb
begin
unique case(hburst_s1)
3'b001: burst_size_s1 = 6;
3'b010: burst_size_s1 = 4;
3'b011: burst_size_s1 = 4;
3'b100: burst_size_s1 = 8;
3'b101: burst_size_s1 = 8;
3'b110: burst_size_s1 = 16;
3'b111: burst_size_s1 = 16;
default: burst_size_s1 = 1;
endcase
end

endmodule

//////////////////////// SECOND - SLAVE //////////////////////////////

module second_slave(haddr_s2,hclk_s2,hrst_s2,hwrite_s2,hmaster_s2,htrans_s2,hsize_s2,hsel_s2,hburst_s2,hready_s2,hwdata_s2,hready_out_s2,hrdata_s2,hresp_s2); 
input [31:0]haddr_s2,hwdata_s2;
input hclk_s2,hrst_s2,hwrite_s2;
input [3:0]hmaster_s2;
input [1:0]htrans_s2;
input [2:0]hsize_s2,hburst_s2;
input hsel_s2,hready_s2;
output reg hready_out_s2;
output reg [31:0]hrdata_s2;
output reg [1:0]hresp_s2;

reg [7:0]slave_mem_s2[1024:2047];
reg [3:0]state_s2,next_state_s2;
reg stay_s2, stay_wait_s2, load_data_s2, store_data_s2, load_addr_s2, load_resp_s2;
int count_s2,count_wait_s2,burst_size_s2;
reg [1:0]resp_out_s2;
reg [31:0]data_out_s2,data_mem_s2,address_in_s2;
parameter idle_slave_second = 4'b0000, seq_write_slave_second = 4'b0001, seq_read_slave_second = 4'b0010, read_wait_slave_second = 4'b0011, write_wait_slave_second = 4'b0100,
wait_state_slave_second = 4'b0101, after_wait_state_slave_second = 4'b0110, last_wait_state_slave_second = 4'b0111;
parameter reg [31:0]data_memory_s2 = 32'h1a5b_203b;

//data_generation
always_comb
begin
if(!hrst_s2)
begin
	data_mem_s2 = data_memory_s2;
	for(int i=1536; i<2048; i=i+4)
	begin
		slave_mem_s2[i]   = data_mem_s2[31:24];
		slave_mem_s2[i+1] = data_mem_s2[23:16];
		slave_mem_s2[i+2] = data_mem_s2[15:8];
		slave_mem_s2[i+3] = data_mem_s2[7:0];
		data_mem_s2 = data_mem_s2 + 4;
	end
end
else if(store_data_s2 == 1)
begin
	slave_mem_s2[address_in_s2]   <= hwdata_s2[31:24];
	slave_mem_s2[address_in_s2+1] <= hwdata_s2[23:16];
	slave_mem_s2[address_in_s2+2] <= hwdata_s2[15:8];
	slave_mem_s2[address_in_s2+3] <= hwdata_s2[7:0];
end
end

always @(posedge hclk_s2)
begin
if(load_data_s2)
begin
	if(hready_s2)
	data_out_s2 <= {slave_mem_s2[address_in_s2],slave_mem_s2[address_in_s2+1],slave_mem_s2[address_in_s2+2],slave_mem_s2[address_in_s2+3]};
end
end

///// address_loader
always_ff @(posedge hclk_s2)
begin
if(load_addr_s2)
	begin
	if(haddr_s2<1536)
		begin
		if(hwrite_s2)
		  address_in_s2 <= haddr_s2;
		end
	else
		if(!hwrite_s2)
		 address_in_s2 <= haddr_s2;
	end
end

//Response_generator
always @(posedge hclk_s2)
begin
if(load_resp_s2)
begin
	if(haddr_s2<512)
	begin
		if(hwrite_s2)
		  resp_out_s2 <= 2'b00;
		else
		  resp_out_s2 <= 2'b01;
	end
	else
	begin
		if(!hwrite_s2)
		  resp_out_s2 <= 2'b00;
		else
		  resp_out_s2 <= 2'b01;
	end
end
else
resp_out_s2 <= 2'b00;
end	

// second slave FSM Logic
always_comb
begin
case(state_s2)
idle_slave_second: begin
if (hready_s2)
begin
	if(hmaster_s2 == 4'b0000)
	next_state_s2 = idle_slave_second;
	else
	begin
		if(hsel_s2 && htrans_s2 != 2'b00)
		  next_state_s2 = hwrite_s2?((hburst_s2 == 3'b000)? write_wait_slave_second : seq_write_slave_second):((hburst_s2 == 3'b000)? read_wait_slave_second : seq_read_slave_second);
		else if(htrans_s2 == 2'b00)
		  next_state_s2 = idle_slave_second;
		else
		  next_state_s2 = idle_slave_second;
	end
end
end

write_wait_slave_second: begin
next_state_s2 = last_wait_state_slave_second;
end

seq_write_slave_second: begin
	stay_s2 = (count_s2 == burst_size_s2-1) ? 0:1;
	next_state_s2 = (count_s2 == burst_size_s2-1)? write_wait_slave_second : wait_state_slave_second;
end

read_wait_slave_second: begin
next_state_s2 = last_wait_state_slave_second;
end

seq_read_slave_second: begin
	stay_s2 = (count_s2 == burst_size_s2-1) ? 0:1;
	next_state_s2 = (count_s2 == burst_size_s2-1)? read_wait_slave_second : wait_state_slave_second;
end

wait_state_slave_second: begin
stay_wait_s2 = (count_wait_s2 == 2)?0:1;
next_state_s2 = (count_wait_s2 == 2)?after_wait_state_slave_second:wait_state_slave_second;
end

after_wait_state_slave_second: begin
if(hwrite_s2)
next_state_s2 = seq_write_slave_second;
else
next_state_s2 = seq_read_slave_second;
end

last_wait_state_slave_second: begin
next_state_s2 = idle_slave_second;
end

endcase
end

// Counter logic
always_ff @(posedge hclk_s2)
begin
if(stay_s2 && hready_s2)
count_s2 = count_s2 + 1;
else if(!stay_s2)
count_s2 <= 1;
end

always_ff @(posedge hclk_s2)
begin
if(stay_wait_s2)
count_wait_s2 = count_wait_s2 + 1;
else
count_wait_s2 <= 1;
end

always_ff @(posedge hclk_s2, negedge hrst_s2)
begin
if(!hrst_s2)
state_s2 <= idle_slave_second;
else
state_s2 <= next_state_s2;
end

always_comb
begin
if (state_s2 == idle_slave_second) begin
hready_out_s2 = 1;         store_data_s2 = 0;   //hrdata_s2 = 0;
hresp_s2 = resp_out_s2;    load_addr_s2  = 1;   
load_data_s2 = (hwrite_s2)?0:1;          load_resp_s2  = 0;
end
else if (state_s2 == write_wait_slave_second) begin
hready_out_s2 = 1;         store_data_s2 = 1;   //hrdata_s2 = 0;
hresp_s2 = resp_out_s2;    load_addr_s2 = 0;
load_data_s2 = 0;          load_resp_s2 = 0;
end
else if (state_s2 == seq_write_slave_second) begin
hready_out_s2 = 0;         store_data_s2 = 1;   //hrdata_s2 = 0;
hresp_s2 = resp_out_s2;    load_addr_s2 = 0;   
load_data_s2 = 0;          load_resp_s2 = 1;
end
else if (state_s2 == read_wait_slave_second) begin
hready_out_s2 = 1;         store_data_s2 = 0;   hrdata_s2 = data_out_s2;
hresp_s2 = resp_out_s2;    load_addr_s2 = 0;   
load_data_s2 = 0;          load_resp_s2 = 0;
end
else if (state_s2 == seq_read_slave_second) begin
hready_out_s2 = 0;         store_data_s2 = 0;   hrdata_s2 = data_out_s2;
hresp_s2 = resp_out_s2;    load_addr_s2 = 0;   
load_data_s2 = 1;          load_resp_s2 = 1;
end
else if (state_s2 == wait_state_slave_second) begin
hready_out_s2 = 0;         store_data_s2 = 0;   hrdata_s2 = data_out_s2;
hresp_s2 = resp_out_s2;    load_addr_s2 = 0;   
load_data_s2 = 1;          load_resp_s2 = 1;
end
else if (state_s2 == after_wait_state_slave_second) begin
hready_out_s2 = 1;         store_data_s2 = 0;   hrdata_s2 = data_out_s2;
hresp_s2 = resp_out_s2;    load_addr_s2 = 1;   
load_data_s2 = 1;          load_resp_s2 = 1;
end
else if (state_s2 == last_wait_state_slave_second) begin
hready_out_s2 = 1;         store_data_s2 = 0;   hrdata_s2 = data_out_s2;
hresp_s2 = resp_out_s2;    load_addr_s2 = 0;   
load_data_s2 = 0;          load_resp_s2 = 0;
end
end

//burst_size generation in terms of beats
always_comb
begin
unique case(hburst_s2)
3'b001: burst_size_s2 = 6;
3'b010: burst_size_s2 = 4;
3'b011: burst_size_s2 = 4;
3'b100: burst_size_s2 = 8;
3'b101: burst_size_s2 = 8;
3'b110: burst_size_s2 = 16;
3'b111: burst_size_s2 = 16;
default: burst_size_s2 = 1;
endcase
end

endmodule

////////////////////////////////// THIRD - SLAVE ///////////////////////////////////////
 
module third_slave(haddr_s3,hclk_s3,hrst_s3,hwrite_s3,hmaster_s3,htrans_s3,hsize_s3,hsel_s3,hburst_s3,hready_s3,hwdata_s3,hready_out_s3,hrdata_s3,hresp_s3); 
input [31:0]haddr_s3,hwdata_s3;
input hclk_s3,hrst_s3,hwrite_s3;
input [3:0]hmaster_s3;
input [1:0]htrans_s3;
input [2:0]hsize_s3,hburst_s3;
input hsel_s3,hready_s3;
output reg hready_out_s3;
output reg [31:0]hrdata_s3;
output reg [1:0]hresp_s3;

reg [7:0]slave_mem_s3[2048:3071];
reg [3:0]state_s3,next_state_s3;
reg stay_s3, load_data_s3, store_data_s3, load_addr_s3, load_resp_s3;
int count_s3, burst_size_s3;
reg [1:0]resp_out_s3;
reg [31:0]data_out_s3,data_mem_s3,address_in_s3,in_data_s3;
parameter idle_slave_third = 4'b0000, seq_write_slave_third = 4'b0001, seq_read_slave_third = 4'b0010, read_wait_slave_third = 4'b0011, write_wait_slave_third = 4'b0100,
wait_state_slave_third = 4'b0101;
parameter reg [31:0]data_memory_s3 = 32'h1a5b_203b;

always_comb
begin
if(hwrite_s3)
in_data_s3 = hwdata_s3;
else
in_data_s3 = 32'bx;
end

//data_generation
always_comb
begin
if(!hrst_s3)
begin
	data_mem_s3 = data_memory_s3;
	for(int i=2560; i<3072; i=i+4) //1536, 2048
	begin
		slave_mem_s3[i]   = data_mem_s3[31:24];
		slave_mem_s3[i+1] = data_mem_s3[23:16];
		slave_mem_s3[i+2] = data_mem_s3[15:8];
		slave_mem_s3[i+3] = data_mem_s3[7:0];
		data_mem_s3 = data_mem_s3 + 4;
	end
end
else if(store_data_s3 == 1)
begin
	slave_mem_s3[address_in_s3]   <= hwdata_s3[31:24];
	slave_mem_s3[address_in_s3+1] <= hwdata_s3[23:16];
	slave_mem_s3[address_in_s3+2] <= hwdata_s3[15:8];
	slave_mem_s3[address_in_s3+3] <= hwdata_s3[7:0];
end
end

always @(posedge hclk_s3)
begin
if(load_data_s3)
begin
	if(hready_s3)
	data_out_s3 <= {slave_mem_s3[address_in_s3],slave_mem_s3[address_in_s3+1],slave_mem_s3[address_in_s3+2],slave_mem_s3[address_in_s3+3]};
end
end

///// address_loader
always_ff @(posedge hclk_s3)
begin
if(load_addr_s3)
	begin
	if(haddr_s3<2560)
		begin
		if(hwrite_s3)
		  address_in_s3 <= haddr_s3;
		end
	else
		if(!hwrite_s3)
		 address_in_s3 <= haddr_s3;
	end
end

//Response_generator
always @(posedge hclk_s3)
begin
if(load_resp_s3)
begin
	if(haddr_s3<512)
	begin
		if(hwrite_s3)
		  resp_out_s3 <= 2'b00;
		else
		  resp_out_s3 <= 2'b01;
	end
	else
	begin
		if(!hwrite_s3)
		  resp_out_s3 <= 2'b00;
		else
		  resp_out_s3 <= 2'b01;
	end
end
else
resp_out_s3 <= 2'b00;
end	

// third slave FSM Logic
always_comb
begin
case(state_s3)
idle_slave_third: begin
if (hready_s3)
begin
	if(hmaster_s3 == 4'b0000)
	next_state_s3 = idle_slave_third;
	else
	begin
		if(hsel_s3 && htrans_s3 != 2'b00)
		  next_state_s3 = hwrite_s3?((hburst_s3 == 3'b000)? write_wait_slave_third : seq_write_slave_third):((hburst_s3 == 3'b000)? read_wait_slave_third : seq_read_slave_third);
		else if(htrans_s3 == 2'b00)
		  next_state_s3 = idle_slave_third;
		else
		  next_state_s3 = idle_slave_third;
	end
end
end

write_wait_slave_third: begin
if(hmaster_s3 == 4'b0000)
 next_state_s3 = idle_slave_third;
else
 next_state_s3 = wait_state_slave_third;
end

seq_write_slave_third: begin
if (hmaster_s3 == 4'b0000)
 next_state_s3 = idle_slave_third;
else
begin
	stay_s3 = (count_s3 == burst_size_s3-1) ? 0:1;
	next_state_s3 = (count_s3 == burst_size_s3-1)? write_wait_slave_third : seq_write_slave_third;
end
end

read_wait_slave_third: begin
if(hmaster_s3 == 4'b0000)
 next_state_s3 = idle_slave_third;
else
 next_state_s3 = wait_state_slave_third;
end

seq_read_slave_third: begin
if (hmaster_s3 == 4'b0000)
 next_state_s3 = idle_slave_third;
else
begin
	stay_s3 = (count_s3 == burst_size_s3) ? 0:1;
	next_state_s3 = (count_s3 == burst_size_s3)? read_wait_slave_third : seq_read_slave_third;
end
end

wait_state_slave_third: begin
next_state_s3 = idle_slave_third;
end
endcase
end

// Counter logic
always_ff @(posedge hclk_s3)
begin
if(stay_s3)
count_s3 = count_s3 + 1;
else
count_s3 <= 1;
end

always_ff @(posedge hclk_s3, negedge hrst_s3)
begin
if(!hrst_s3)
state_s3 <= idle_slave_third;
else
state_s3 <= next_state_s3;
end

always_comb
begin
if (state_s3 == idle_slave_third) begin
hready_out_s3 = 1;         store_data_s3 = 0;   //hrdata_s3 = 0;
hresp_s3 = resp_out_s3;    load_addr_s3  = 1;   
load_data_s3 = 0;          load_resp_s3  = 0;
end
else if (state_s3 == write_wait_slave_third) begin
hready_out_s3 = 1;         store_data_s3 = 1;   //hrdata_s3 = 0;
hresp_s3 = resp_out_s3;    load_addr_s3 = 0;
load_data_s3 = 0;          load_resp_s3 = 0;
end
else if (state_s3 == seq_write_slave_third) begin
hready_out_s3 = 1;         store_data_s3 = 1;   //hrdata_s3 = 0;
hresp_s3 = resp_out_s3;    load_addr_s3 = 1;   
load_data_s3 = 0;          load_resp_s3 = 1;
end
else if (state_s3 == read_wait_slave_third) begin
hready_out_s3 = 1;         store_data_s3 = 0;   hrdata_s3 = data_out_s3;
hresp_s3 = resp_out_s3;    load_addr_s3 = 0;   
load_data_s3 = 0;          load_resp_s3 = 0;
end
else if (state_s3 == seq_read_slave_third) begin
hready_out_s3 = 1;         store_data_s3 = 0;   hrdata_s3 = data_out_s3;
hresp_s3 = resp_out_s3;    load_addr_s3 = 1;   
load_data_s3 = 1;          load_resp_s3 = 1;
end
else if (state_s3 == wait_state_slave_third) begin
hready_out_s3 = 1;         store_data_s3 = 0;   hrdata_s3 = data_out_s3;
hresp_s3 = resp_out_s3;    load_addr_s3 = 0;   
load_data_s3 = 0;          load_resp_s3 = 0;
end
end

//burst_size generation in terms of beats
always_comb
begin
unique case(hburst_s3)
3'b001: burst_size_s3 = 6;
3'b010: burst_size_s3 = 4;
3'b011: burst_size_s3 = 4;
3'b100: burst_size_s3 = 8;
3'b101: burst_size_s3 = 8;
3'b110: burst_size_s3 = 16;
3'b111: burst_size_s3 = 16;
default: burst_size_s3 = 1;
endcase
end

endmodule

//////////////////////// DEFAULT - SLAVE //////////////////////////////

module def_slave(haddr_def,hclk_def,hrst_def,hwrite_def,hmaster_def,htrans_def,hsize_def,hsel_def,hburst_def,hready_def,hwdata_def,hready_out_def,hrdata_def,hresp_def); 
input [31:0]haddr_def,hwdata_def;
input hclk_def,hrst_def,hwrite_def;
input [3:0]hmaster_def;
input [1:0]htrans_def;
input [2:0]hsize_def,hburst_def;
input hsel_def,hready_def;
output reg hready_out_def;
output reg [31:0]hrdata_def;
output reg [1:0]hresp_def;

reg [3:0]state_def,next_state_def;
parameter idle_slave_def = 4'b0000, ok_slave_def = 4'b0001, error_slave_def = 4'b0010, error_cycle_slave_def = 4'b0011;

// def slave FSM Logic
always_comb
begin
case(state_def)
idle_slave_def: begin
if (hsel_def)
 next_state_def = ((htrans_def == 2'b10) || (htrans_def == 2'b11))?error_slave_def:ok_slave_def;
end

ok_slave_def: begin
if(!hsel_def)
 next_state_def = idle_slave_def;
else
 next_state_def = ((htrans_def == 2'b01) || (htrans_def == 2'b00))?ok_slave_def:idle_slave_def;
end

error_slave_def: begin
if (!hsel_def)
 next_state_def = idle_slave_def;
else
 next_state_def = error_cycle_slave_def;
end

error_cycle_slave_def: begin
if(!hsel_def)
 next_state_def = idle_slave_def;
else if((htrans_def == 2'b10) || (htrans_def == 2'b11))
 next_state_def = error_cycle_slave_def;
else
 next_state_def = idle_slave_def;
end

endcase
end

always_ff @(posedge hclk_def, negedge hrst_def)
begin
if(!hrst_def)
state_def <= idle_slave_def;
else
state_def <= next_state_def;
end

always_comb
begin
if (state_def == idle_slave_def) begin
hready_out_def = 1;  hresp_def = 2'b00;
end
else if (state_def == ok_slave_def) begin
hready_out_def = 1;  hresp_def = 2'b00;
end
else if (state_def == error_slave_def) begin
hready_out_def = 0;  hresp_def = 2'b01;
end
else if (state_def == error_cycle_slave_def) begin
hready_out_def = 1;  hresp_def = 2'b01;
end
else begin
hready_out_def = 1;  hresp_def = 2'b00;
end
end

endmodule
//slaves end here

///////////////////// ADDRESS CONTROL MUX ////////////////////////////
module addr_control_mux(haddr_def,haddr_m1,haddr_m2,hmaster,haddr_out,
			haddr_m3);
input [31:0]haddr_def,haddr_m1,haddr_m2,haddr_m3;
input [3:0]hmaster;
output reg [31:0]haddr_out;

always_comb
begin
case(hmaster)
4'b0000: haddr_out = haddr_def;
4'b0001: haddr_out = haddr_m1;
4'b0010: haddr_out = haddr_m2;
4'b0011: haddr_out = haddr_m3;
default: haddr_out = 32'bxxxxxxxx;
endcase
end
endmodule

///////////////////// MASTER TO SLAVE CONTROL MUX ////////////////////////////
module mastr_to_slave_control_mux(hwrite_def,hburst_def,hsize_def,htrans_def,hmaster,hwrite,hsize,hburst,htrans,
				  hwrite_m1,hburst_m1,hsize_m1,htrans_m1, hwrite_m2,hburst_m2,hsize_m2,htrans_m2,
                                  htrans_m3,hburst_m3,hsize_m3,hwrite_m3);
input hwrite_def,hwrite_m1,hwrite_m2,hwrite_m3;
input [2:0]hburst_def,hsize_def,hburst_m1,hsize_m1,hburst_m2,hsize_m2,hburst_m3,hsize_m3;
input [1:0]htrans_def,htrans_m1,htrans_m2,htrans_m3;
input [3:0]hmaster;
output reg hwrite;
output reg [2:0]hsize,hburst;
output reg [1:0]htrans;

always_comb
begin
unique case(hmaster)
4'b0000: begin hwrite = hwrite_def; hsize = hsize_def; hburst = hburst_def; htrans = htrans_def; end
4'b0001: begin hwrite = hwrite_m1; hsize = hsize_m1; hburst = hburst_m1; htrans = htrans_m1; end
4'b0010: begin hwrite = hwrite_m2; hsize = hsize_m2; hburst = hburst_m2; htrans = htrans_m2; end
4'b0011: begin hwrite = hwrite_m3; hsize = hsize_m3; hburst = hburst_m3; htrans = htrans_m3; end
default: begin hwrite = hwrite_def; hsize = hsize_def; hburst = hburst_def; htrans = htrans_def; end
endcase
end
endmodule

///////////////////// MASTER TO SLAVE DATA MUX  ////////////////////////////
module mastr_to_slave_data_mux(hdata_def,hdata_m1,hdata_m2,hdata_sel,hdata_mast,
				hdata_m3);
input [31:0]hdata_def,hdata_m1,hdata_m2,hdata_m3;
input [3:0]hdata_sel;
output reg [31:0]hdata_mast; //check this signal name

always_comb
begin
case(hdata_sel)
4'b0000: hdata_mast = hdata_def;
4'b0001: hdata_mast = hdata_m1;
4'b0010: hdata_mast = hdata_m2;
4'b0011: hdata_mast = hdata_m3;
default: hdata_mast = 32'hxxxxxxxx;
endcase
end
endmodule

///////////////////// SLAVE TO MASTER MUX ////////////////////////////

module slave_to_master_mux(hready_out_def,hrdata_def,hresp_def,mux_select,hready_out,hrdata,hresp,
			   hready_out_s1,hrdata_s1,hresp_s1,
			   hready_out_s2,hrdata_s2,hresp_s2,
			   hready_out_s3,hrdata_s3,hresp_s3);
input hready_out_def,hready_out_s1,hready_out_s2,hready_out_s3;
input [31:0]hrdata_def,hrdata_s1,hrdata_s2,hrdata_s3;
input [1:0]mux_select;
input [1:0]hresp_def,hresp_s1,hresp_s2,hresp_s3;
output reg hready_out;
output reg [31:0]hrdata;
output reg [1:0]hresp;

always_comb
begin
case(mux_select)
2'b00: begin hready_out = hready_out_def; hrdata = hrdata_def; hresp = hresp_def; end
2'b01: begin hready_out = hready_out_s1; hrdata = hrdata_s1; hresp = hresp_s1; end
2'b10: begin hready_out = hready_out_s2; hrdata = hrdata_s2; hresp = hresp_s2; end
2'b11: begin hready_out = hready_out_s3; hrdata = hrdata_s3; hresp = hresp_s3; end
default: begin hready_out = hready_out_def; hrdata = hrdata_def; hresp = hresp_def; end
endcase
end
endmodule

////////////////////////////////// SLAVE SELECT DECODER ///////////////////////////

module slave_select_decoder (haddr,hsel_def,hsel_1,hsel_2,hsel_3,mux_select);
input [31:0] haddr;
output reg hsel_def,hsel_1,hsel_2,hsel_3;
output reg [1:0]mux_select;

always_comb 
begin
if (haddr < 32'd3072) begin
case (haddr[11:10])
2'b00: begin          // 0 - 1023
hsel_def = 1'b0; hsel_1 = 1'b1; hsel_2 = 1'b0; hsel_3 = 1'b0; mux_select = 2'b01;
end

2'b01: begin          // 1024 - 2047
hsel_def = 1'b0; hsel_1 = 1'b0; hsel_2 = 1'b1; hsel_3 = 1'b0; mux_select = 2'b10;
end

2'b10: begin          // 2048 - 3071
hsel_def = 1'b0; hsel_1 = 1'b0; hsel_2 = 1'b0; hsel_3 = 1'b1; mux_select = 2'b11;
end

default: begin
hsel_def = 1'b1; hsel_1 = 1'b0; hsel_2 = 1'b0; hsel_3 = 1'b0; mux_select = 2'b00;
end
endcase
end

else begin  // Address outside 0-3071
hsel_def = 1'b1; hsel_1 = 1'b0; hsel_2 = 1'b0; hsel_3 = 1'b0; mux_select = 2'b00;
end
end

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// FULL - MASTER - MODULE //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

module full_master(haddr_mast,hwdata_mast,hwrite_mast,hbusreq_def_mast,hbusreq_m1_mast,hbusreq_m2_mast,hsize_mast,hburst_mast,htrans_mast,
                   hrst_mast,hclk_mast,hready_mast,hgrant_def_mast,hgrant_m1_mast,hgrant_m2_mast,hresp_mast,hrdata_mast,hmaster_arb,
		   hbusreq_m3_mast,hgrant_m3_mast);
output [31:0]haddr_mast,hwdata_mast;
output hwrite_mast,hbusreq_def_mast,hbusreq_m1_mast,hbusreq_m2_mast,hbusreq_m3_mast;
output [2:0]hsize_mast,hburst_mast;
output [1:0]htrans_mast;
input hrst_mast,hclk_mast,hready_mast,hgrant_def_mast,hgrant_m1_mast,hgrant_m2_mast,hgrant_m3_mast;
input [1:0]hresp_mast;
input [31:0]hrdata_mast;
input [3:0]hmaster_arb;

wire [31:0]def_addr,m1_addr,m2_addr,m3_addr, def_data,m1_data,m2_data,m3_data;
wire write_def,write_m1,write_m2,write_m3;
wire [2:0]size_def,size_m1,size_m2,size_m3,burst_def,burst_m1,burst_m2,burst_m3;
wire [1:0]trans_def,trans_m1,trans_m2,trans_m3;

first_master fm1(.hgrant_m1(hgrant_m1_mast),.hready_m1(hready_mast),.hresp_m1(hresp_mast),.hclk_m1(hclk_mast),.hrst_m1(hrst_mast),.hrdata_m1(hrdata_mast),
             .hbusreq_m1(hbusreq_m1_mast),.htrans_m1(trans_m1),.haddr_m1(m1_addr),.hburst_m1(burst_m1),.hsize_m1(size_m1),.hwdata_m1(m1_data),.hwrite_m1(write_m1));

second_master sm1(.hgrant_m2(hgrant_m2_mast),.hready_m2(hready_mast),.hresp_m2(hresp_mast),.hclk_m2(hclk_mast),.hrst_m2(hrst_mast),.hrdata_m2(hrdata_mast),.hbusreq_m2(hbusreq_m2_mast),
              .htrans_m2(trans_m2),.haddr_m2(m2_addr),.hburst_m2(burst_m2),.hsize_m2(size_m2),.hwdata_m2(m2_data),.hwrite_m2(write_m2));

third_master tm1(.hgrant_m3(hgrant_m3_mast),.hready_m3(hready_mast),.hresp_m3(hresp_mast),.hclk_m3(hclk_mast),.hrst_m3(hrst_mast),.hrdata_m3(hrdata_mast),.hbusreq_m3(hbusreq_m3_mast),
              .htrans_m3(trans_m3),.haddr_m3(m3_addr),.hburst_m3(burst_m3),.hsize_m3(size_m3),.hwdata_m3(m3_data),.hwrite_m3(write_m3));

default_master dm1(.hgrant_def(hgrant_def_mast),.hready_def(hready_mast),.hresp_def(hresp_mast),.hclk_def(hclk_mast),.hrst_def(hrst_mast),.hrdata_def(hrdata_mast),.hbusreq_def(hbusreq_def_mast),
              .htrans_def(trans_def),.haddr_def(def_addr),.hburst_def(burst_def),.hsize_def(size_def),.hwdata_def(def_data),.hwrite_def(write_def));

addr_control_mux acm1(.haddr_def(def_addr),.haddr_m1(m1_addr),.haddr_m2(m2_addr),.hmaster(hmaster_arb),.haddr_out(haddr_mast),.haddr_m3(m3_addr));

mastr_to_slave_control_mux ccm1(.hwrite_def(write_def),.hburst_def(burst_def),.hsize_def(size_def),.htrans_def(trans_def),.hmaster(hmaster_arb),.hwrite(hwrite_mast),.hsize(hsize_mast),.hburst(hburst_mast),
                  .htrans(htrans_mast),.hwrite_m1(write_m1),.hburst_m1(burst_m1),.hsize_m1(size_m1),.htrans_m1(trans_m1), .hwrite_m2(write_m2),.hburst_m2(burst_m2),.hsize_m2(size_m2),.htrans_m2(trans_m2),
		  .htrans_m3(trans_m3),.hburst_m3(burst_m3),.hsize_m3(size_m3),.hwrite_m3(write_m3));

mastr_to_slave_data_mux dcm1(.hdata_def(def_data),.hdata_m1(m1_data),.hdata_m2(m2_data),.hdata_m3(m3_data),.hdata_sel(hmaster_arb),.hdata_mast(hwdata_mast));

endmodule

module tb_full_master;

wire [31:0]haddr_mast,hwdata_mast;
wire hwrite_mast,hbusreq_def_mast,hbusreq_m1_mast,hbusreq_m2_mast,hbusreq_m3_mast;
wire [2:0]hsize_mast,hburst_mast;
wire [1:0]htrans_mast;
reg hrst_mast,hclk_mast,hready_mast,hgrant_def_mast,hgrant_m1_mast,hgrant_m2_mast,hgrant_m3_mast;
reg [1:0]hresp_mast;
reg [31:0]hrdata_mast;
reg [3:0]hmaster_arb;

full_master full_m1(haddr_mast,hwdata_mast,hwrite_mast,hbusreq_def_mast,hbusreq_m1_mast,hbusreq_m2_mast,hsize_mast,hburst_mast,htrans_mast,
                   hrst_mast,hclk_mast,hready_mast,hgrant_def_mast,hgrant_m1_mast,hgrant_m2_mast,hresp_mast,hrdata_mast,hmaster_arb,
		   hbusreq_m3_mast,hgrant_m3_mast);

always #2 hclk_mast = ~hclk_mast;

initial begin
hresp_mast = 2'b00;
#8 hmaster_arb = 4'b0001;
end

initial
begin
hgrant_def_mast = 0;hgrant_m1_mast = 0;hgrant_m2_mast = 0;hgrant_m3_mast = 0;
#8 hgrant_m1_mast = 1;
end

initial
hready_mast = 1;

initial
begin
hclk_mast = 0;
hrst_mast = 0;
#1 hrst_mast = 1;
end
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// FULL - SLAVE - MODULE ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

module full_slave(hready_out_slv,hresp_slv,hrdata_slv,haddr_slv,hwdata_slv,hsize_slv,hburst_slv,hwrite_slv,hclk_slv,hrst_slv,hready_slv,htrans_slv,hmaster_slv);
output reg hready_out_slv;
output reg [1:0]hresp_slv;
output reg [31:0]hrdata_slv;
input [31:0]haddr_slv,hwdata_slv;
input [2:0]hsize_slv,hburst_slv;
input hwrite_slv,hclk_slv,hrst_slv,hready_slv;
input [1:0]htrans_slv;
input [3:0]hmaster_slv; //check

wire hsel_def,hsel_s1,hsel_s2,hsel_s3, hreadyout_def,hreadyout_s1,hreadyout_s2,hreadyout_s3;
wire [1:0]mux_select_slv,hresp_def,hresp_s1,hresp_s2,hresp_s3;
wire [31:0]hrdata_slv_def,hrdata_slv_s1,hrdata_slv_s2,hrdata_slv_s3;

first_slave fs1(.haddr_s1(haddr_slv),.hclk_s1(hclk_slv),.hrst_s1(hrst_slv),.hwrite_s1(hwrite_slv),.hmaster_s1(hmaster_slv),.htrans_s1(htrans_slv),.hsize_s1(hsize_slv),
            .hsel_s1(hsel_s1),.hburst_s1(hburst_slv),.hready_s1(hready_slv),.hwdata_s1(hwdata_slv),.hready_out_s1(hreadyout_s1),.hrdata_s1(hrdata_slv_s1),.hresp_s1(hresp_s1)); 

second_slave ss1(.haddr_s2(haddr_slv),.hclk_s2(hclk_slv),.hrst_s2(hrst_slv),.hwrite_s2(hwrite_slv),.hmaster_s2(hmaster_slv),.htrans_s2(htrans_slv),.hsize_s2(hsize_slv),
            .hsel_s2(hsel_s2),.hburst_s2(hburst_slv),.hready_s2(hready_slv),.hwdata_s2(hwdata_slv),.hready_out_s2(hreadyout_s2),.hrdata_s2(hrdata_slv_s2),.hresp_s2(hresp_s2));

third_slave ts1(.haddr_s3(haddr_slv),.hclk_s3(hclk_slv),.hrst_s3(hrst_slv),.hwrite_s3(hwrite_slv),.hmaster_s3(hmaster_slv),.htrans_s3(htrans_slv),.hsize_s3(hsize_slv),
            .hsel_s3(hsel_s3),.hburst_s3(hburst_slv),.hready_s3(hready_slv),.hwdata_s3(hwdata_slv),.hready_out_s3(hreadyout_s3),.hrdata_s3(hrdata_slv_s3),.hresp_s3(hresp_s3));

def_slave ds1(.haddr_def(haddr_slv),.hclk_def(hclk_slv),.hrst_def(hrst_slv),.hwrite_def(hwrite_slv),.hmaster_def(hmaster_slv),.htrans_def(htrans_slv),.hsize_def(hsize_slv),
            .hsel_def(hsel_def),.hburst_def(hburst_slv),.hready_def(hready_slv),.hwdata_def(hwdata_slv),.hready_out_def(hreadyout_def),.hrdata_def(hrdata_slv_def),.hresp_def(hresp_def));

slave_to_master_mux smmux(.hready_out_def(hreadyout_def),.hrdata_def(hrdata_slv_def),.hresp_def(hresp_def),.mux_select(mux_select_slv),.hready_out(hready_out_slv),.hrdata(hrdata_slv),.hresp(hresp_slv),
			   .hready_out_s1(hreadyout_s1),.hrdata_s1(hrdata_slv_s1),.hresp_s1(hresp_s1),
			   .hready_out_s2(hreadyout_s2),.hrdata_s2(hrdata_slv_s2),.hresp_s2(hresp_s2),
			   .hready_out_s3(hreadyout_s3),.hrdata_s3(hrdata_slv_s3),.hresp_s3(hresp_s3));

slave_select_decoder ssdec(.haddr(haddr_slv),.hsel_def(hsel_def),.hsel_1(hsel_s1),.hsel_2(hsel_s2),.hsel_3(hsel_s3),.mux_select(mux_select_slv));


endmodule 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// FULL - MASTER - FULL - SLAVE - MODULE ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module ahb_proj(hclk_a,hrst_a,hmaster_a,hbusreq_def_a,hbusreq_m1_a,hbusreq_m2_a,hgrant_def_a,hgrant_m1_a,hgrant_m2_a,
		hbusreq_m3_a,hgrant_m3_a);
input hclk_a,hrst_a;
input [3:0]hmaster_a;
input hgrant_def_a,hgrant_m1_a,hgrant_m2_a,hgrant_m3_a;
output hbusreq_def_a,hbusreq_m1_a,hbusreq_m2_a,hbusreq_m3_a;

wire [31:0]hadder_w,hrdata_w,hwdata_w;
wire hwrite_w,hready_w;
wire [2:0]hsize_w,hburst_w;
wire[1:0]htrans_w,hresp_w;

full_master full_mstr1(.haddr_mast(hadder_w),.hwdata_mast(hwdata_w),.hwrite_mast(hwrite_w),.hbusreq_def_mast(hbusreq_def_a),.hbusreq_m1_mast(hbusreq_m1_a),.hbusreq_m2_mast(hbusreq_m2_a),
                       .hsize_mast(hsize_w),.hburst_mast(hburst_w),.htrans_mast(htrans_w),.hrst_mast(hrst_a),.hclk_mast(hclk_a),.hready_mast(hready_w),
                       .hgrant_def_mast(hgrant_def_a),.hgrant_m1_mast(hgrant_m1_a),.hgrant_m2_mast(hgrant_m2_a),.hresp_mast(hresp_w),.hrdata_mast(hrdata_w),.hmaster_arb(hmaster_a),
		       .hbusreq_m3_mast(hbusreq_m3_a),.hgrant_m3_mast(hgrant_m3_a));

full_slave full_slv1(.hready_out_slv(hready_w),.hresp_slv(hresp_w),.hrdata_slv(hrdata_w),.haddr_slv(hadder_w),.hwdata_slv(hwdata_w),.hsize_slv(hsize_w),
               .hburst_slv(hburst_w),.hwrite_slv(hwrite_w),.hclk_slv(hclk_a),.hrst_slv(hrst_a),.hready_slv(hready_w),.htrans_slv(htrans_w),.hmaster_slv(hmaster_a)); //check hready_out

endmodule

//testbench
module tb_ahb_proj;
reg hclk_a,hrst_a;
reg [3:0]hmaster_a;
reg hgrant_def_a,hgrant_m1_a,hgrant_m2_a,hgrant_m3_a;
wire hbusreq_def_a,hbusreq_m1_a,hbusreq_m2_a,hbusreq_m3_a;

ahb_proj full_mast_slv(hclk_a,hrst_a,hmaster_a,hbusreq_def_a,hbusreq_m1_a,hbusreq_m2_a,hgrant_def_a,hgrant_m1_a,hgrant_m2_a,
			hbusreq_m3_a,hgrant_m3_a);

always #2 hclk_a = ~hclk_a;

initial begin
#12 hmaster_a = 4'b0001;
end

initial
begin
hgrant_def_a = 0;hgrant_m1_a = 0;hgrant_m2_a = 0; hgrant_m3_a = 0;
#8 hgrant_m1_a = 1;
end

initial
begin
hclk_a = 0;
hrst_a = 0;
#1 hrst_a = 1;
end
endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// ARBITER FINAL ///////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

module lfsr_general #(parameter seed=4'b0001)(rand_out,clk,reset);
output reg [3:0]rand_out;
input clk,reset;
reg [3:0]temp;
always @(posedge clk, negedge reset)
begin
if(!reset)
temp <= seed;
else begin
temp <= {temp[2:0],temp[3]^temp[2]};  //[3:0] 4 ^ 3
rand_out <= temp;
end
end
endmodule

//LFSR General Stage
module lfsr_general_stage #(parameter seed=5'b00101)(rand_out,clk,reset);
output reg [4:0]rand_out;
input clk,reset;
reg [4:0]temp;
always @(posedge clk, negedge reset)
begin
if(!reset)
temp <= seed;
else begin
temp <= {temp[3:0],temp[4]^temp[3]};
rand_out <= temp;
end
end
endmodule

////////////// Ticket Generator //////////////
module ticket_generator(clk,reset,tkt1,tkt2,tkt3,tkt4);
input clk,reset;
output reg[3:0]tkt1,tkt2,tkt3,tkt4;

lfsr_general #(4'b0001)lfsr_gen1(tkt1,clk,reset); //def_tkt
lfsr_general #(4'b0010)lfsr_gen2(tkt2,clk,reset);
lfsr_general #(4'b0011)lfsr_gen3(tkt3,clk,reset);
lfsr_general #(4'b0101)lfsr_gen4(tkt4,clk,reset);

endmodule

////////////////// LOTTERY MANAGER /////////////////////////////////
module lottery_manager(clk,reset,req1,req2,req3,req4,tkt1,tkt2,tkt3,tkt4,sum1,sum2,sum3,sum4);
input clk,reset,req1,req2,req3,req4;
input [3:0]tkt1,tkt2,tkt3,tkt4;
output reg [5:0]sum1,sum2,sum3,sum4;

reg [5:0]tmp[1:4];

always_comb
begin
tmp[1] = req1*tkt1;
tmp[2] = req2*tkt2;
tmp[3] = req3*tkt3;
tmp[4] = req4*tkt4;
end
always_comb
begin
sum1 = tmp[1];
sum2 = tmp[1] + tmp[2];
sum3 = tmp[1] + tmp[2] + tmp[3];
sum4 = tmp[1] + tmp[2] + tmp[3] + tmp[4];
end
endmodule

////////////////////////////////////////////
///////////////// COMPARATOR ///////////////
////////////////////////////////////////////

module comparator(clk,reset,req1,req2,req3,req4,grantt1,grantt2,grantt3,grantt4);
input clk,reset,req1,req2,req3,req4;
output reg grantt1,grantt2,grantt3,grantt4;

wire [5:0]sum[1:4];
wire [3:0]tkt[1:4];
wire [4:0]rand_no;

ticket_generator tkt_gen(clk,reset,tkt[1],tkt[2],tkt[3],tkt[4]);

lottery_manager lot_man(clk,reset,req1,req2,req3,req4,tkt[1],tkt[2],tkt[3],tkt[4],sum[1],sum[2],sum[3],sum[4]);

lfsr_general_stage lfsr_gen_st(rand_no,clk,reset);


always_comb
begin
if(req1 && rand_no >=0 && rand_no < sum[1])
begin
	grantt1 = 1; grantt2 = 0; grantt3 = 0; grantt4 = 0;
end
else if(req2 && rand_no >= sum[1] && rand_no < sum[2])
begin
	grantt1 = 0; grantt2 = 1; grantt3 = 0; grantt4 = 0;
end
else if(req3 && rand_no >= sum[2] && rand_no < sum[3])
begin
	grantt1 = 0; grantt2 = 0; grantt3 = 1; grantt4 = 0;
end
else if(req4 && rand_no >= sum[3] && rand_no < sum[4])
begin
	grantt1 = 0; grantt2 = 0; grantt3 = 0; grantt4 = 1;
end
else
begin
	grantt1 = 0; grantt2 = 0; grantt3 = 0; grantt4 = 0;
end
end
endmodule

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// ARBITER - FINAL //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

module arbiter_final(clk,reset,req1,req2,req3,req4,hready,htrans,hresp,hburst,haddr,hmaster,grant1,grant2,grant3,grant4,grant_def);
input clk,reset,req1,req2,req3,req4,hready;
input [1:0]htrans,hresp;
input [2:0]hburst;
input [31:0]haddr;
output reg[3:0]hmaster;
output reg grant1,grant2,grant3,grant4,grant_def;

wire grantt1,grantt2,grantt3,grantt4;

comparator comp(clk,reset,req1,req2,req3,req4,grantt1,grantt2,grantt3,grantt4);

//grant signal generator
always_ff @(posedge clk)
begin
if(grant1)
hmaster <= 4'b0001;
else if(grant2)
hmaster <= 4'b0010;
else if(grant3)
hmaster <= 4'b0011;
else if(grant4)
hmaster <= 4'b0100;
else
hmaster <= 4'b0000;
end

//master signal generator
always_ff @(posedge clk, negedge reset)
begin
if(!reset)
begin
grant1 <= 0;
grant2 <= 0;
grant3 <= 0;
grant4 <= 0;
grant_def <= 0;
end
else if ((htrans == 2'b00) && (hresp == 2'b00) && hready == 1)
begin
	if(req1 && !req2 && !req3 && !req4)
	begin
	grant1 <= 1;
	grant2 <= 0;
	grant3 <= 0;
	grant4 <= 0;
	grant_def <= 0;
	end

	else if(!req1 && req2 && !req3 && !req4)
	begin
	grant1 <= 0;
	grant2 <= 1;
	grant3 <= 0;
	grant4 <= 0;
	grant_def <= 0;
	end

	else if(!req1 && !req2 && req3 && !req4)
	begin
	grant1 <= 0;
	grant2 <= 0;
	grant3 <= 1;
	grant4 <= 0;
	grant_def <= 0;
	end

	else if(!req1 && !req2 && !req3 && req4)
	begin
	grant1 <= 0;
	grant2 <= 0;
	grant3 <= 0;
	grant4 <= 1;
	grant_def <= 0;
	end

	else if(!req1 && !req2 && !req3 && !req4)
	begin
	grant1 <= 1;
	grant2 <= 0;
	grant3 <= 0;
	grant4 <= 0;
	grant_def <= 1;
	end

	else
	begin
	grant1 <= grantt1;
	grant2 <= grantt2;
	grant3 <= grantt3;
	grant4 <= grantt4;
	grant_def <= 0;
	end
end
end
endmodule


//testbench arbiter_final
module tb_arbiter_final;
reg clk,reset,req1,req2,req3,req4,hready;
reg [1:0]htrans,hresp;
reg [2:0]hburst;
reg [31:0]haddr;
wire[3:0]hmaster;
wire grant1,grant2,grant3,grant4,grant_def;

arbiter_final arb_f(clk,reset,req1,req2,req3,req4,hready,htrans,hresp,hburst,haddr,hmaster,grant1,grant2,grant3,grant4,grant_def);

always #5 clk = ~clk;
initial begin
clk = 0; 
hburst = 2'b00;
hresp = 2'b00;
htrans = 2'b00;
hready = 1;
haddr = 32'h0000_0010;
reset = 0;
req1 = 0; req2 = 0; req3 = 0; req4 = 0;
#18 reset = 1;
req1 = 1; req2 = 1; req3 = 1; req4 = 0;
#38 htrans = 2'b10;
end
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////// FULL MASTER SLAVE ARBITER FINAL AHB MODULE //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module my_ahb_project_final(hclk_a,hrst_a,hbusreq_m4_a);
input hclk_a,hrst_a,hbusreq_m4_a;
wire [3:0]hmaster_a;
wire hgrant_def_a,hgrant_m1_a,hgrant_m2_a,hgrant_m3_a;
wire hbusreq_def_a,hbusreq_m1_a,hbusreq_m2_a,hbusreq_m3_a;

wire [31:0]hadder_w,hrdata_w,hwdata_w;
wire hwrite_w,hready_w;
wire [2:0]hsize_w,hburst_w;
wire[1:0]htrans_w,hresp_w;

full_master full_master1(.haddr_mast(hadder_w),.hwdata_mast(hwdata_w),.hwrite_mast(hwrite_w),.hbusreq_def_mast(hbusreq_def_a),.hbusreq_m1_mast(hbusreq_m1_a),.hbusreq_m2_mast(hbusreq_m2_a),
                       .hsize_mast(hsize_w),.hburst_mast(hburst_w),.htrans_mast(htrans_w),.hrst_mast(hrst_a),.hclk_mast(hclk_a),.hready_mast(hready_w),
                       .hgrant_def_mast(hgrant_def_a),.hgrant_m1_mast(hgrant_m1_a),.hgrant_m2_mast(hgrant_m2_a),.hresp_mast(hresp_w),.hrdata_mast(hrdata_w),.hmaster_arb(hmaster_a),
		       .hbusreq_m3_mast(hbusreq_m3_a),.hgrant_m3_mast(hgrant_m3_a));

full_slave full_slave1(.hready_out_slv(hready_w),.hresp_slv(hresp_w),.hrdata_slv(hrdata_w),.haddr_slv(hadder_w),.hwdata_slv(hwdata_w),.hsize_slv(hsize_w),
               .hburst_slv(hburst_w),.hwrite_slv(hwrite_w),.hclk_slv(hclk_a),.hrst_slv(hrst_a),.hready_slv(hready_w),.htrans_slv(htrans_w),.hmaster_slv(hmaster_a)); //check hready_out

arbiter_final arb_final(.clk(hclk_a),.reset(hrst_a),.req1(hbusreq_m1_a),.req2(hbusreq_m2_a),.req3(hbusreq_m3_a),.req4(hbusreq_m4_a),.hready(hready_w),.htrans(htrans_w),.hresp(hresp_w),.hburst(hburst_w),
                        .haddr(hadder_w),.hmaster(hmaster_a),.grant1(hgrant_m1_a),.grant2(hgrant_m2_a),.grant3(hgrant_m3_a),.grant_def(hgrant_def_a));
endmodule

