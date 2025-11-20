module BRAM_Reader (
	input logic i_clk,
	input logic i_rst_n,
	input logic i_keydown,  // Debounced key press
	// BRAM interface
	output logic [7:0] o_bram_addr,
	output logic o_bram_wren,
	// BRAM data output (from BRAM module)
	input logic [15:0] i_bram_data,
	// Output data
	output logic [15:0] o_palette_data
);

	// Address counter (8-bit address for 256 entries)
	logic [7:0] addr;
	
	// BRAM control signals
	assign o_bram_addr = addr;
	assign o_bram_wren = 1'b0;  // Always read mode
	
	// Register BRAM output data
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			addr <= 8'd0;
			o_palette_data <= 16'd0;
		end else begin
			// Increment address on keydown
			if (i_keydown) begin
				addr <= addr + 8'd1;
			end
			
			// Read data from BRAM (BRAM has registered output)
			o_palette_data <= i_bram_data;
		end
	end

endmodule

