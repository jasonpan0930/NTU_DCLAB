// VGA Image Display Module
// Reads pixel data from SRAM, looks up color from BRAM palette, and outputs to VGA 720p
module VGA_Image_Display (
	input logic i_clk,              // 74.25MHz VGA clock
	input logic i_rst_n,
	input logic [10:0] i_h_count,   // Horizontal position (0-1279)
	input logic [9:0] i_v_count,    // Vertical position (0-719)
	input logic i_active_video,     // Active video region
	
	// SRAM interface for image data
	output logic [19:0] o_sram_addr,
	output logic o_sram_ce_n,
	inout [15:0] io_sram_dq,
	output logic o_sram_lb_n,
	output logic o_sram_oe_n,
	output logic o_sram_ub_n,
	output logic o_sram_we_n,
	
	// BRAM interface for palette
	output logic [7:0] o_bram_addr,
	output logic o_bram_wren,
	input logic [15:0] i_bram_data,
	
	// VGA RGB output
	output logic [7:0] o_vga_r,
	output logic [7:0] o_vga_g,
	output logic [7:0] o_vga_b
);

	// Image dimensions
	localparam IMAGE_WIDTH = 1280;
	localparam IMAGE_HEIGHT = 720;
	
	// Calculate byte address in SRAM: y * 1280 + x
	// Since SRAM is 16-bit wide, word address = byte_addr[20:1]
	logic [20:0] pixel_byte_addr;
	logic [7:0] pixel_index;  // Palette index from SRAM
	logic [15:0] rgb565;       // RGB565 color from BRAM
	
	// Pipeline stages for timing
	logic [7:0] pixel_index_reg;
	logic [15:0] rgb565_reg;
	
	// Calculate pixel byte address
	assign pixel_byte_addr = (i_v_count * IMAGE_WIDTH) + i_h_count;
	
	// SRAM control signals
	assign o_sram_ce_n = 1'b0;
	assign o_sram_oe_n = 1'b0;
	assign o_sram_we_n = 1'b1;  // Read mode
	assign o_sram_addr = pixel_byte_addr[20:1];  // Word address
	assign o_sram_lb_n = pixel_byte_addr[0];     // Low byte select
	assign o_sram_ub_n = ~pixel_byte_addr[0];    // High byte select
	assign io_sram_dq = 16'hZZZZ;  // High-Z for read
	
	// BRAM control signals
	assign o_bram_wren = 1'b0;  // Read mode
	
	// Pipeline stage 1: Read pixel index from SRAM
	// Use registered address for reading (SRAM has access delay)
	logic [20:0] pixel_byte_addr_reg;
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			pixel_byte_addr_reg <= 21'd0;
			pixel_index_reg <= 8'd0;
		end else begin
			// Register address for reading
			pixel_byte_addr_reg <= pixel_byte_addr;
			
			// Read byte from SRAM based on registered byte address LSB
			if (pixel_byte_addr_reg[0] == 1'b0) begin
				pixel_index_reg <= io_sram_dq[7:0];   // Low byte
			end else begin
				pixel_index_reg <= io_sram_dq[15:8];  // High byte
			end
		end
	end
	
	// Pipeline stage 2: Use pixel index to read palette from BRAM
	// BRAM address needs to be registered (BRAM expects registered address)
	logic [7:0] bram_addr_reg;
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			bram_addr_reg <= 8'd0;
			rgb565_reg <= 16'd0;
		end else begin
			bram_addr_reg <= pixel_index_reg;
			rgb565_reg <= i_bram_data;  // BRAM has registered output (1 cycle delay)
		end
	end
	
	assign o_bram_addr = bram_addr_reg;
	
	// RGB565 to RGB888 conversion
	// RGB565 format: R[15:11] (5 bits), G[10:5] (6 bits), B[4:0] (5 bits)
	// RGB888: R[7:0], G[7:0], B[7:0]
	logic [7:0] r_8bit, g_8bit, b_8bit;
	
	// Red: 5 bits -> 8 bits (multiply by 8.225, approximate as {R[4:0], R[4:2]})
	assign r_8bit = {rgb565_reg[15:11], rgb565_reg[15:13]};
	
	// Green: 6 bits -> 8 bits (multiply by 4.05, approximate as {G[5:0], G[5:4]})
	assign g_8bit = {rgb565_reg[10:5], rgb565_reg[10:9]};
	
	// Blue: 5 bits -> 8 bits (multiply by 8.225, approximate as {B[4:0], B[4:2]})
	assign b_8bit = {rgb565_reg[4:0], rgb565_reg[4:2]};
	
	// Output RGB values
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			o_vga_r <= 8'd0;
			o_vga_g <= 8'd0;
			o_vga_b <= 8'd0;
		end else begin
			if (i_active_video) begin
				o_vga_r <= r_8bit;
				o_vga_g <= g_8bit;
				o_vga_b <= b_8bit;
			end else begin
				o_vga_r <= 8'd0;
				o_vga_g <= 8'd0;
				o_vga_b <= 8'd0;
			end
		end
	end

endmodule

