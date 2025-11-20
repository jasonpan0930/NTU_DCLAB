module SevenHexDecoder (
	input        [3:0] i_hex,
	output logic [6:0] o_seven_ten,
	output logic [6:0] o_seven_one
);

/* The layout of seven segment display, 1: dark
 *    00
 *   5  1
 *    66
 *   4  2
 *    33
 */
parameter D0 = 7'b1000000;
parameter D1 = 7'b1111001;
parameter D2 = 7'b0100100;
parameter D3 = 7'b0110000;
parameter D4 = 7'b0011001;
parameter D5 = 7'b0010010;
parameter D6 = 7'b0000010;
parameter D7 = 7'b1011000;
parameter D8 = 7'b0000000;
parameter D9 = 7'b0010000;
always_comb begin
	case(i_hex)
		4'h0: begin o_seven_ten = D0; o_seven_one = D0; end
		4'h1: begin o_seven_ten = D0; o_seven_one = D1; end
		4'h2: begin o_seven_ten = D0; o_seven_one = D2; end
		4'h3: begin o_seven_ten = D0; o_seven_one = D3; end
		4'h4: begin o_seven_ten = D0; o_seven_one = D4; end
		4'h5: begin o_seven_ten = D0; o_seven_one = D5; end
		4'h6: begin o_seven_ten = D0; o_seven_one = D6; end
		4'h7: begin o_seven_ten = D0; o_seven_one = D7; end
		4'h8: begin o_seven_ten = D0; o_seven_one = D8; end
		4'h9: begin o_seven_ten = D0; o_seven_one = D9; end
		4'ha: begin o_seven_ten = D1; o_seven_one = D0; end
		4'hb: begin o_seven_ten = D1; o_seven_one = D1; end
		4'hc: begin o_seven_ten = D1; o_seven_one = D2; end
		4'hd: begin o_seven_ten = D1; o_seven_one = D3; end
		4'he: begin o_seven_ten = D1; o_seven_one = D4; end
		4'hf: begin o_seven_ten = D1; o_seven_one = D5; end
	endcase
end

endmodule

// Extended decoder: display a 13-bit number (0..8191) on four 7-seg digits
// Outputs are ordered as thousands, hundreds, tens, ones
module SevenDec13 (
	input        [12:0] i_val,
	output logic [6:0]  o_hex3,
	output logic [6:0]  o_hex2,
	output logic [6:0]  o_hex1,
	output logic [6:0]  o_hex0
);

	// 7-seg codes (1 means dark)
	localparam D0 = 7'b1000000;
	localparam D1 = 7'b1111001;
	localparam D2 = 7'b0100100;
	localparam D3 = 7'b0110000;
	localparam D4 = 7'b0011001;
	localparam D5 = 7'b0010010;
	localparam D6 = 7'b0000010;
	localparam D7 = 7'b1011000;
	localparam D8 = 7'b0000000;
	localparam D9 = 7'b0010000;

	function automatic [6:0] digit_to_seg(input [3:0] d);
		case (d)
			4'd0: digit_to_seg = D0;
			4'd1: digit_to_seg = D1;
			4'd2: digit_to_seg = D2;
			4'd3: digit_to_seg = D3;
			4'd4: digit_to_seg = D4;
			4'd5: digit_to_seg = D5;
			4'd6: digit_to_seg = D6;
			4'd7: digit_to_seg = D7;
			4'd8: digit_to_seg = D8;
			4'd9: digit_to_seg = D9;
			default: digit_to_seg = 7'b1111111; // blank
		endcase
	endfunction

	logic [3:0] thousands, hundreds, tens, ones;
	integer tmp;

	always_comb begin
		// Clamp to 13-bit range just in case
		tmp = i_val & 13'h1FFF;
		thousands = tmp / 1000;
		tmp = tmp % 1000;
		hundreds = tmp / 100;
		tmp = tmp % 100;
		tens = tmp / 10;
		ones = tmp % 10;

		o_hex3 = digit_to_seg(thousands);
		o_hex2 = digit_to_seg(hundreds);
		o_hex1 = digit_to_seg(tens);
		o_hex0 = digit_to_seg(ones);
	end

endmodule
