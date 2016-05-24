module SingleProtocolTransmitter(
	clk
	, reset
	
	, out_cmd
	, out_data
	
	, tran_interface_ready
	, tran_interface_start
	, tran_interface_out_byte
	
	, flush
	
	, start_packet_tran
	, tran_complete
);

// -------------------------------------------------------------------------------

/*!
	[const] BYTE_LENGHT - length of a byte measured in bits
	[const] BUFFER_LENGHT - buffer for incoming data in bytes without command byte
	[const] TOTAL_REQUARED_BITS - total bits in data buffer
	[const] COUNTER_SIZE - bits in recv_data_cnt and recv_data_size
 */

parameter BYTE_LENGTH = 8;
parameter BUFFER_LENGTH = 24;
parameter TOTAL_REQUARED_BITS = BYTE_LENGTH * BUFFER_LENGTH;
parameter COUNTER_SIZE = 8;

// -------------------------------------------------------------------------------

/*!
	[in] clk - clock
	[in] reset - reset on fall edge
	
	[in] form_packet - command for save "cmd" and "data" into internal buffers, 
		affect on flag "complete", if "complete" is setted than new "form_packet" 
		command will be ignored, if you need in save new "cmd" and "data" in 
		"cmd_buf" and "data_buf" at first step you must flush SingleProtocolTransmitter
	[in] out_cmd - wire to internal buffer "cmd"
	[in] out_data - wire to internal buffer "data"
	
	[in] get_out_byte - command for shifting new byte from internal buffers
	[out] out_byte - shifted byte from internal buffer
	
	[in] flush - command flush "SingleProtocolTransmitter"
	
	[out] busy - flag indicates that "get_out_byte" command is performed
	[out] complete - flag indicates that packet is fully transmitted
 */
input wire clk;
input wire reset;

input wire [7:0] out_cmd;
input wire [TOTAL_REQUARED_BITS - 1:0] out_data;

input wire tran_interface_ready;
output reg tran_interface_start;
output reg [7:0] tran_interface_out_byte;
	
input wire flush;
	
input wire start_packet_tran;
output reg tran_complete;
// -------------------------------------------------------------------------------

reg flag_cmd_tran;
reg flag_all_data_tran;
reg flag_crc_tran;
reg [COUNTER_SIZE - 1:0] tran_data_cnt;
reg [COUNTER_SIZE - 1:0] tran_data_size;

reg flag_need_tran_packet;
reg flag_need_tran_crc;
reg flag_wait_tran_interface;

reg prev_state_start_packet_tran;
wire start_packet_tran_posedge = ~prev_state_start_packet_tran & start_packet_tran;

reg prev_state_flush;
wire flush_posedge = ~prev_state_flush & flush;

reg prev_state_tran_interface_ready;
wire tran_interface_ready_posedge = ~prev_state_tran_interface_ready & tran_interface_ready;

// CRC8
reg crc_reset;
reg crc_start;
reg [7:0] crc_data_size;
wire crc_complete;
wire [7:0] crc_value;
calc_CRC8 #(BUFFER_LENGTH + 1) crc(
	.clk(clk),
	.reset(crc_reset),
	
	.data({out_data, out_cmd}),
	.data_size(crc_data_size),
	
	.start(crc_start),
	.complete(crc_complete),
	.value(crc_value)
);

// -------------------------------------------------------------------------------

always @(posedge clk or negedge reset) begin
	if (~reset) begin
		flag_cmd_tran <= 1'd0;
		flag_all_data_tran <= 1'd0;
		flag_crc_tran <= 1'd0;
		tran_data_cnt <= 1'd0;
		tran_data_size <= 1'd0;
		
		tran_interface_start <= 1'd0;
		tran_complete <= 1'd0;
		
		crc_start <= 1'd0;
		crc_reset <= 1'd0;
		crc_data_size <= 0;
		
		flag_need_tran_packet <= 1'd0;
		flag_need_tran_crc <= 1'd0;
		flag_wait_tran_interface <= 1'd0;
	end else begin
		// rising edge of start_packet_tran
		prev_state_start_packet_tran <= start_packet_tran;
		// rising edge of flush
		prev_state_flush <= flush;
		// rising edge of tran_interface_ready
		prev_state_tran_interface_ready <= tran_interface_ready;
		
		if (start_packet_tran_posedge) begin
			flag_need_tran_packet <= 1'd1;
			tran_complete <= 1'd0;
		end
		
		// packet transmission
		if (flag_need_tran_packet && ~flag_wait_tran_interface) begin
			// transmite "cmd"
			if (~flag_cmd_tran) begin
				tran_interface_out_byte <= out_cmd;
				flag_cmd_tran <= 1'd1;
				
				/*!--------------------------------->
				   set data size, change here if need
				  ---------------------------------->*/
				case(out_cmd)
					8'hD0 : tran_data_size <= 8'd1; // test leds
					8'h21 : tran_data_size <= 8'd1; // test buttons click
					
					8'hF2 : tran_data_size <= 8'd6; // frequency relation meter: get measure interval
					8'hF4 : tran_data_size <= 8'd1; // frequency relation meter: is measure complete?
					8'hF5 : tran_data_size <= 8'd24; // frequency relation meter: transmit measured data
					
					8'hE1 : tran_data_size <= 8'd1; // frequency meters: get meter type
					8'hE5 : tran_data_size <= 8'd6; // frequency meters: get measure time or period amount
					8'hE6 : tran_data_size <= 8'd1; // frequency meters: is complete
					8'hE7 : tran_data_size <= 8'd12; // frequency meters: get full period measure data
					8'hE8 : tran_data_size <= 8'd12; // frequency meters: get rise to fall measure data
					
					8'hC4 : tran_data_size <= 8'd1; // pulse meter: get edge type
					8'hC6 : tran_data_size <= 8'd6; // pulse meter: get measure time
					8'hC7 : tran_data_size <= 8'd1; // pulse meter: is complete
					8'hC8 : tran_data_size <= 8'd6; // pulse meter: get measured pulses value
					
					8'hB3 : tran_data_size <= 8'd1; // time interval meter: get signal type
					8'hB4 : tran_data_size <= 8'd1; // time interval meter: is complete
					8'hB5 : tran_data_size <= 8'd6; // time interval meter: get measured value
					
					8'h34 : tran_data_size <= 8'd1;
					
					default : tran_data_size <= 8'd0; // unknown command (you must use only known command)
				endcase
				/*!<---------------------------------
				   set data size, change here if need
				   <---------------------------------*/
			end
			
			// transmite "data"
			if (flag_cmd_tran && (tran_data_cnt != tran_data_size)) begin
				tran_interface_out_byte[0] <= out_data[8'd8 * tran_data_cnt + 8'd0]; // 0 bit
				tran_interface_out_byte[1] <= out_data[8'd8 * tran_data_cnt + 8'd1]; // 1 bit
				tran_interface_out_byte[2] <= out_data[8'd8 * tran_data_cnt + 8'd2]; // 2 bit
				tran_interface_out_byte[3] <= out_data[8'd8 * tran_data_cnt + 8'd3]; // 3 bit
				tran_interface_out_byte[4] <= out_data[8'd8 * tran_data_cnt + 8'd4]; // 4 bit
				tran_interface_out_byte[5] <= out_data[8'd8 * tran_data_cnt + 8'd5]; // 5 bit
				tran_interface_out_byte[6] <= out_data[8'd8 * tran_data_cnt + 8'd6]; // 6 bit
				tran_interface_out_byte[7] <= out_data[8'd8 * tran_data_cnt + 8'd7]; // 7 bit
				tran_data_cnt <= tran_data_cnt + 1'd1;
			end
			
			// start transmite "CRC8"
			if (flag_all_data_tran) begin
				flag_need_tran_crc <= 1'd1;
				crc_data_size <= tran_data_size + 1'd1;
				crc_start <= 1'd1;
			end
			
			tran_interface_start <= 1'd1;
			flag_wait_tran_interface <= 1'd1;
		end
		
		// drop "flag_wait_tran_interface"
		if (flag_wait_tran_interface && tran_interface_ready_posedge) begin
			flag_wait_tran_interface <= 1'd0;
		end
		
		// drop "tran_interface_start"
		if (tran_interface_start && ~tran_interface_ready) begin
			tran_interface_start <= 1'd0;
		end
		
		// all data is transmitted
		if (flag_cmd_tran && (tran_data_cnt == tran_data_size) && flag_wait_tran_interface) begin
			flag_all_data_tran <= 1'd1;
		end
		
		// CRC8 transmission
		if (flag_need_tran_crc && crc_complete &&~flag_crc_tran) begin
			tran_interface_out_byte <= crc_value;
			flag_crc_tran <= 1'd1;
			
			tran_interface_start <= 1'd1;
			flag_wait_tran_interface <= 1'd1;
		end
		
		// end of transmission
		if (/*flag_cmd_tran && flag_all_data_tran*/flag_crc_tran && tran_interface_ready_posedge) begin
			flag_need_tran_packet <= 1'd0;
			flag_need_tran_crc <= 1'd0;
			tran_complete <= 1'd1;
			
			flag_cmd_tran <= 1'd0;
			flag_all_data_tran <= 1'd0;
			flag_crc_tran <= 1'd0;
			tran_data_cnt <= 1'd0;
			
			crc_reset <= 1'd0;
		end
		
		// flush
		if (flush_posedge) begin
			flag_need_tran_packet <= 1'd0;
			flag_need_tran_crc <= 1'd0;
			tran_complete <= 1'd1;
			
			flag_cmd_tran <= 1'd0;
			flag_all_data_tran <= 1'd0;
			flag_crc_tran <= 1'd0;
			tran_data_cnt <= 1'd0;
			
			crc_reset <= 1'd0;
		end
		
		// flush "crc_start"
		if (crc_start) begin
			crc_start <= 1'd0;
		end
		// flush "crc_reset"
		if (~crc_reset) begin
			crc_reset <= 1'd1;
		end
	end
end

endmodule
