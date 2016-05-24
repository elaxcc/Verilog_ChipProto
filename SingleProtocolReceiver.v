module SingleProtocolReceiver(
	clk
	, reset
	
	, process_in_byte
	, in_byte
	
	, in_cmd
	, in_data
	, in_crc_valid
	
	, busy
	, complete
	, flush
);

// -------------------------------------------------------------------------------

/*!
	[const] BYTE_LENGHT - length of a byte measured in bits
	[const] BUFFER_LENGHT - buffer for incoming data in bytes without command byte
	[const] TOTAL_REQUARED_BITS - total bits in data buffer
	[const] COUNTER_SIZE - bits in recv_data_cnt and recv_data_size
	[const] TIMEOUT_COUNTER_SIZE - size of the timeout counter
	[const] TIMEOUT_MAX_VAL - value for reset, time T for reset can be find using equation
		T = TIMEOUT_MAX_VAL / clk_freq, where clk_freq - clock frequency
 */

parameter BYTE_LENGTH = 8;
parameter BUFFER_LENGTH = 6;
parameter TOTAL_REQUARED_BITS = BYTE_LENGTH * BUFFER_LENGTH;
parameter COUNTER_SIZE = 8;
parameter TIMEOUT_COUNTER_SIZE = 32;
parameter TIMEOUT_MAX_VAL = 10000000;

// -------------------------------------------------------------------------------

/*!
	[in] clk - clock
	[in] reset - reset on fall edge
	
	[in] process_in_data - flag for new incoming byte processing, processing starts
		on rising edge
	[in] in_data - incoming byte
	
	[out] in_cmd - incoming command
	[out] in_data - incoming data
	
	[out] busy - output busy flag, sets to "1" if incoming byte is processed
	[out] complete - flag of the end of a packet parsing
 */
input wire clk;
input wire reset;

input wire process_in_byte;
input wire [7:0] in_byte;

output reg [7:0] in_cmd;
output reg [TOTAL_REQUARED_BITS - 1:0] in_data;
output reg in_crc_valid;

output wire busy;
output reg complete;
input wire flush;

// -------------------------------------------------------------------------------

reg flag_got_command;
reg flag_got_all_data;
reg flag_got_crc;
reg flag_need_to_process_byte;
reg flag_need_to_flush;
reg [COUNTER_SIZE - 1:0] recv_data_cnt;
reg [COUNTER_SIZE - 1:0] recv_data_size;
reg [7:0] in_crc;

reg prev_state_process_in_byte; // find rising edge of "process_in_byte"
wire process_in_byte_posedge = ~prev_state_process_in_byte & process_in_byte;

reg prev_state_flush; // find rising edge of "flush"
wire flush_posedge = ~prev_state_flush & flush;

assign busy = flag_need_to_process_byte;

reg [TIMEOUT_COUNTER_SIZE - 1:0] timeout_cnt;
reg timeout_enable;

// CRC8
reg crc_reset;
reg crc_start;
reg [7:0] crc_data_size;
wire crc_complete;
wire [7:0] crc_value;
calc_CRC8 #(BUFFER_LENGTH + 1) crc(
	.clk(clk),
	.reset(crc_reset),
	
	.data({in_data, in_cmd}),
	.data_size(crc_data_size),
	
	.start(crc_start),
	.complete(crc_complete),
	.value(crc_value)
);

// -------------------------------------------------------------------------------

always @(posedge clk or negedge reset) begin
	if (~reset) begin
		complete <= 1'd0;
		
		flag_got_command <= 1'd0;
		flag_got_all_data <= 1'd0;
		flag_got_crc <= 1'd0;
		flag_need_to_process_byte <= 1'd0;
		flag_need_to_flush <= 1'd0;
		
		recv_data_cnt <= 8'd0;
		in_crc_valid <= 1'd0;
		in_crc <= 8'd0;
		
		crc_start <= 1'd0;
		crc_reset <= 1'd0;
		crc_data_size <= 0;
		
		// timeout
		timeout_cnt <= 32'd0;
		timeout_enable <= 1'd0;
	end else begin
	
		// try to find rising edge of "process_in_byte"
		prev_state_process_in_byte <= process_in_byte;
		// try to find rising edge of "flush"
		prev_state_flush <= flush;
	
		if (process_in_byte_posedge) begin
			flag_need_to_process_byte <= 1'd1;
		end
		
		if (flush_posedge) begin
			flag_need_to_flush <= 1'd1;
		end
		
		if (flag_need_to_process_byte) begin
		
			// get command
			if (~flag_got_command) begin
				in_cmd <= in_byte;
				/*!--------------------------------->
				   set data size, change here if need
				   --------------------------------->*/
				case(in_byte)
					8'h11 : recv_data_size <= 8'd1; // test leds
					
					8'h12 : recv_data_size <= 8'd1; // frequency switcher: set clock source
					
					8'hF1 : recv_data_size <= 8'd6; // frequency relation meter: set measure interval
					8'hF2 : recv_data_size <= 8'd0; // frequency relation meter: get measure interval
					8'hF3 : recv_data_size <= 8'd0; // frequency relation meter: start measure
					8'hF4 : recv_data_size <= 8'd0; // frequency relation meter: is measure complete?
					8'hF5 : recv_data_size <= 8'd0; // frequency relation meter: transmit measured data
					
					8'hE0 : recv_data_size <= 8'd1; // frequency meters: set meter type
					8'hE1 : recv_data_size <= 8'd0; // frequency meters: get meter type
					8'hE2 : recv_data_size <= 8'd0; // frequency meters: reset
					8'hE3 : recv_data_size <= 8'd0; // frequency meters: start
					8'hE4 : recv_data_size <= 8'd6; // frequency meters: set measure time or period amount
					8'hE5 : recv_data_size <= 8'd0; // frequency meters: get measure time or period amount
					8'hE6 : recv_data_size <= 8'd0; // frequency meters: is complete
					8'hE7 : recv_data_size <= 8'd0; // frequency meters: get full period measure data
					8'hE8 : recv_data_size <= 8'd0; // frequency meters: get rise to fall measure data
					
					8'hC0 : recv_data_size <= 8'd0; // pulse meter: reset
					8'hC1 : recv_data_size <= 8'd0; // pulse meter: start
					8'hC2 : recv_data_size <= 8'd0; // pulse meter: pause
					8'hC3 : recv_data_size <= 8'd1; // pulse meter: set edge type
					8'hC4 : recv_data_size <= 8'd0; // pulse meter: get edge type
					8'hC5 : recv_data_size <= 8'd6; // pulse meter: set measure time
					8'hC6 : recv_data_size <= 8'd0; // pulse meter: get measure time
					8'hC7 : recv_data_size <= 8'd0; // pulse meter: is complete
					8'hC8 : recv_data_size <= 8'd0; // pulse meter: get measured pulses value
					
					8'hB0 : recv_data_size <= 8'd0; // time interval meter: reset
					8'hB1 : recv_data_size <= 8'd0; // time interval meter: start
					8'hB2 : recv_data_size <= 8'd1; // time interval meter: set signal type
					8'hB3 : recv_data_size <= 8'd0; // time interval meter: get signal type
					8'hB4 : recv_data_size <= 8'd0; // time interval meter: is complete
					8'hB5 : recv_data_size <= 8'd0; // time interval meter: get measured value
					
					8'h34 : recv_data_size <= 8'd1;
					
					default : recv_data_size <= 8'd0; // unknown command
				endcase
				/*!<---------------------------------
				   set data size, change here if need
				   <---------------------------------*/
				   
				// reset flags
				flag_got_command <= 1'd1;
				flag_need_to_process_byte <= 1'd0;
				
				// enable timeout
				timeout_cnt <= 32'd0;
				timeout_enable <= 1'd1;
			end
			
			// get data
			if (flag_got_command && (recv_data_cnt != recv_data_size)) begin
				in_data[8'd8 * recv_data_cnt + 8'd0] <= in_byte[0]; // 0 bit
				in_data[8'd8 * recv_data_cnt + 8'd1] <= in_byte[1]; // 1 bit
				in_data[8'd8 * recv_data_cnt + 8'd2] <= in_byte[2]; // 2 bit
				in_data[8'd8 * recv_data_cnt + 8'd3] <= in_byte[3]; // 3 bit
				in_data[8'd8 * recv_data_cnt + 8'd4] <= in_byte[4]; // 4 bit
				in_data[8'd8 * recv_data_cnt + 8'd5] <= in_byte[5]; // 5 bit
				in_data[8'd8 * recv_data_cnt + 8'd6] <= in_byte[6]; // 6 bit
				in_data[8'd8 * recv_data_cnt + 8'd7] <= in_byte[7]; // 7 bit
				recv_data_cnt <= recv_data_cnt + 1'd1;
				
				flag_need_to_process_byte <= 1'd0;
			end
			
			// get CRC8
			if (flag_got_all_data && ~flag_got_crc) begin
				in_crc <= in_byte;
				flag_got_crc <= 1'd1;
				crc_data_size <= recv_data_size + 1'd1;
				crc_start <= 1'd1;
		
				// disable timeout
				timeout_enable <= 1'd0;
			end
		end
		
		// got all command data
		if (flag_got_command && ~flag_got_all_data && (recv_data_cnt == recv_data_size)) begin
			flag_got_all_data <= 1'd1;
		end
		
		// end of parsing
		if (flag_got_crc && crc_complete) begin
			in_crc_valid <= (crc_value == in_crc) ? 1'd1 : 1'd0;
			complete <= 1'd1;
		end
		
		// timeout count
		if (timeout_enable) begin
			timeout_cnt <= timeout_cnt + 1'd1;
		end
		
		// if flush or timeout
		if (flag_need_to_flush || (timeout_enable && (timeout_cnt == TIMEOUT_MAX_VAL))) begin
			flag_need_to_process_byte <= 1'd0;
		
			complete <= 1'd0;
			flag_got_command <= 1'd0;
			flag_got_all_data <= 1'd0;
			flag_got_crc <= 1'd0;
			flag_need_to_flush <= 1'd0;
			
			recv_data_cnt <= 8'd0;
			in_crc_valid <= 1'd0;
			in_crc <= 8'd0;
			crc_reset <= 1'd0;
			crc_data_size <= 0;
			
			// disable timeout
			timeout_enable <= 1'd0;
			timeout_cnt <= 32'd0;
		end
		
		// drop "crc_start"
		if (crc_start) begin
			crc_start <= 1'd0;
		end
		// drop "crc_reset"
		if (~crc_reset) begin
			crc_reset <= 1'd1;
		end
	end
end

endmodule
