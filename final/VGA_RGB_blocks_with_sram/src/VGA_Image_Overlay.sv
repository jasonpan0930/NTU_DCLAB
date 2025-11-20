// VGA Image Overlay Module
// Overlays zombie sprite on top of background with transparency support
module VGA_Image_Overlay (
	input logic i_clk,              // 74.25MHz VGA clock
	input logic i_rst_n,
	input logic [10:0] i_h_count,   // Horizontal position (0-1279)
	input logic [9:0] i_v_count,    // Vertical position (0-719)
	input logic i_active_video,     // Active video region
	
	// Background RGB input (from background display)
	input logic [7:0] i_bg_r,
	input logic [7:0] i_bg_g,
	input logic [7:0] i_bg_b,
	
	// Zombie position and size
	input logic [10:0] i_zombie_x1,  // Zombie top-left X position
	input logic [9:0] i_zombie_y1,   // Zombie top-left Y position
	input logic [10:0] i_zombie_size_x,  // Zombie width (102)
	input logic [9:0] i_zombie_size_y,   // Zombie height (149)
	
	// Zombie BRAM interface (image data)
	output logic [13:0] o_zombie_bram_addr,
	output logic o_zombie_bram_wren,
	input logic [7:0] i_zombie_bram_data,
	
	// Zombie palette BRAM interface
	output logic [7:0] o_zombie_palette_addr,
	output logic o_zombie_palette_wren,
	input logic [15:0] i_zombie_palette_data,
	
	// VGA RGB output
	output logic [7:0] o_vga_r,
	output logic [7:0] o_vga_g,
	output logic [7:0] o_vga_b
);

	// Check if current pixel is within zombie sprite area
	logic in_zombie_area;
	logic [10:0] zombie_local_x;  // X position relative to zombie sprite
	logic [9:0] zombie_local_y;   // Y position relative to zombie sprite
	
	assign in_zombie_area = (i_h_count >= i_zombie_x1) && 
	                        (i_h_count < (i_zombie_x1 + i_zombie_size_x)) &&
	                        (i_v_count >= i_zombie_y1) && 
	                        (i_v_count < (i_zombie_y1 + i_zombie_size_y));
	
	assign zombie_local_x = i_h_count - i_zombie_x1;
	assign zombie_local_y = i_v_count - i_zombie_y1;
	
	// Calculate zombie BRAM address: y * width + x
	// Zombie image is 102x149, stored in BRAM with 14-bit address
	logic [13:0] zombie_pixel_addr;
	assign zombie_pixel_addr = in_zombie_area ? 
	                          ((zombie_local_y * i_zombie_size_x) + zombie_local_x) : 
	                          14'd0;
	
	// Zombie BRAM control signals
	assign o_zombie_bram_wren = 1'b0;  // Read mode
	assign o_zombie_bram_addr = zombie_pixel_addr;
	
	// Pipeline stages
	logic in_zombie_area_reg;
	logic [7:0] zombie_pixel_index;
	logic [7:0] zombie_pixel_index_reg;
	logic [15:0] zombie_rgb565;
	logic [15:0] zombie_rgb565_reg;
	logic [7:0] bg_r_reg, bg_g_reg, bg_b_reg;
	
	// Pipeline stage 1: Read zombie pixel index from BRAM
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			in_zombie_area_reg <= 1'b0;
			zombie_pixel_index_reg <= 8'd0;
			bg_r_reg <= 8'd0;
			bg_g_reg <= 8'd0;
			bg_b_reg <= 8'd0;
		end else begin
			in_zombie_area_reg <= in_zombie_area;
			zombie_pixel_index_reg <= i_zombie_bram_data;  // BRAM has registered output
			bg_r_reg <= i_bg_r;
			bg_g_reg <= i_bg_g;
			bg_b_reg <= i_bg_b;
		end
	end
	
	// Pipeline stage 2: Read zombie palette
	assign o_zombie_palette_wren = 1'b0;  // Read mode
	assign o_zombie_palette_addr = zombie_pixel_index_reg;
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			zombie_rgb565_reg <= 16'd0;
		end else begin
			zombie_rgb565_reg <= i_zombie_palette_data;  // BRAM has registered output
		end
	end
	
	// RGB565 to RGB888 conversion for zombie
	logic [7:0] zombie_r_8bit, zombie_g_8bit, zombie_b_8bit;
	
	assign zombie_r_8bit = {zombie_rgb565_reg[15:11], zombie_rgb565_reg[15:13]};
	assign zombie_g_8bit = {zombie_rgb565_reg[10:5], zombie_rgb565_reg[10:9]};
	assign zombie_b_8bit = {zombie_rgb565_reg[4:0], zombie_rgb565_reg[4:2]};
	
	// Output RGB: if in zombie area and pixel is not transparent (0x00), use zombie color; otherwise use background
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			o_vga_r <= 8'd0;
			o_vga_g <= 8'd0;
			o_vga_b <= 8'd0;
		end else begin
			if (i_active_video) begin
				// If in zombie area and pixel is not transparent, use zombie color
				if (in_zombie_area_reg && (zombie_pixel_index_reg != 8'h00)) begin
					o_vga_r <= zombie_r_8bit;
					o_vga_g <= zombie_g_8bit;
					o_vga_b <= zombie_b_8bit;
				end else begin
					// Use background color
					o_vga_r <= bg_r_reg;
					o_vga_g <= bg_g_reg;
					o_vga_b <= bg_b_reg;
				end
			end else begin
				o_vga_r <= 8'd0;
				o_vga_g <= 8'd0;
				o_vga_b <= 8'd0;
			end
		end
	end

endmodule

