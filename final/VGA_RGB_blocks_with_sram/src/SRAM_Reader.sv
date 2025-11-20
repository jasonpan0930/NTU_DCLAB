module SRAM_Reader (
	input logic i_clk,
	input logic i_rst_n,
	input logic i_keydown,  // Debounced key press
	// SRAM interface
	output logic [19:0] o_sram_addr,
	output logic o_sram_ce_n,
	inout [15:0] io_sram_dq,
	output logic o_sram_lb_n,
	output logic o_sram_oe_n,
	output logic o_sram_ub_n,
	output logic o_sram_we_n,
	// Output data
	output logic [7:0] o_byte_data
);

	// Address counter (byte address, starting from 0)
	// SRAM has 20-bit address for words (16-bit), so 2^20 * 2 = 2^21 bytes total
	logic [20:0] byte_addr;
	
	// SRAM control signals
	assign o_sram_ce_n = 1'b0;  // Always enable chip
	assign o_sram_oe_n = 1'b0;  // Always enable output
	assign o_sram_we_n = 1'b1;  // Always read mode
	assign o_sram_addr = byte_addr[20:1];  // Word address (16-bit word)
	
	// Byte select based on LSB of byte address
	assign o_sram_lb_n = byte_addr[0];  // Low byte: 0 when byte_addr[0] == 0
	assign o_sram_ub_n = ~byte_addr[0]; // High byte: 0 when byte_addr[0] == 1
	
	// Data bus: SRAM drives the bus during read
	assign io_sram_dq = 16'hZZZZ;  // High-Z for read mode
	
	// Read data from SRAM based on byte address LSB
	// Use registered byte address for reading (one cycle delay for SRAM access)
	logic [20:0] byte_addr_reg;
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			byte_addr <= 21'd0;
			byte_addr_reg <= 21'd0;
			o_byte_data <= 8'd0;
		end else begin
			// Register current address for reading
			byte_addr_reg <= byte_addr;
			
			// Increment address on keydown
			if (i_keydown) begin
				byte_addr <= byte_addr + 21'd1;
			end
			
			// Read data from SRAM (use registered address)
			if (byte_addr_reg[0] == 1'b0) begin
				// Read low byte
				o_byte_data <= io_sram_dq[7:0];
			end else begin
				// Read high byte
				o_byte_data <= io_sram_dq[15:8];
			end
		end
	end

endmodule

