module calc_CRC8 #(data_bytes_len = 6, data_size_bits_len = 8) (
	clk
	, reset
	
	, data
	, data_size
	
	, start
	, complete
	, value
);

input clk;
input reset;

input [data_bytes_len*8 - 1:0] data;
input [data_size_bits_len - 1:0] data_size;

input start;
output reg complete;
output reg [7:0] value;

reg flag_start;
reg [data_size_bits_len - 1:0] current_byte;

integer i;

always @(posedge clk or negedge reset) begin
	if (~reset) begin
		complete <= 1'd0;
		value <= 8'hFF;
		
		flag_start <= 1'd0;
		current_byte <= 0;
	end else begin
		if (start && ~flag_start) begin
			flag_start <= 1'd1;
		end
	
		if (flag_start && ~complete) begin
			if (current_byte != data_size) begin
				value[0] = value[0] ^ data [8'd8 * current_byte + 8'd0];
				value[1] = value[1] ^ data [8'd8 * current_byte + 8'd1];
				value[2] = value[2] ^ data [8'd8 * current_byte + 8'd2];
				value[3] = value[3] ^ data [8'd8 * current_byte + 8'd3];
				value[4] = value[4] ^ data [8'd8 * current_byte + 8'd4];
				value[5] = value[5] ^ data [8'd8 * current_byte + 8'd5];
				value[6] = value[6] ^ data [8'd8 * current_byte + 8'd6];
				value[7] = value[7] ^ data [8'd8 * current_byte + 8'd7];
				
				for (i = 0; i < 8; i=i+1) begin
					value = value & 8'h80 ? (value << 1'd1) ^ 8'h31 : value << 1'd1;
				end
				
				current_byte <= current_byte + 1'd1;
			end else begin
				complete <= 1'd1;
			end
		end
	end
end

endmodule
