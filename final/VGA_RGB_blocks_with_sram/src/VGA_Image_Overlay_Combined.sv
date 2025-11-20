// VGA Image Overlay Module (Combined)
// Handles both background and zombie overlay with transparency support
// Uses sequential (_r) and combinational (_w) logic separation
module VGA_Image_Overlay_Combined (
	input logic i_clk,              // 74.25MHz VGA clock
	input logic i_rst_n,
	input logic [10:0] i_h_count,   // Horizontal position (0-1279) - already 1 cycle ahead
	input logic [9:0] i_v_count,   // Vertical position (0-719) - already 1 cycle ahead
	input logic i_active_video,     // Active video region
	
	// Zombie position and size
	input logic [10:0] i_zombie_x1,  // Zombie top-left X position
	input logic [9:0] i_zombie_y1,   // Zombie top-left Y position
	input logic [10:0] i_zombie_size_x,  // Zombie width (102)
	input logic [9:0] i_zombie_size_y,   // Zombie height (149)
	
	// Background SRAM interface
	output logic [19:0] o_bg_sram_addr,
	output logic o_bg_sram_ce_n,
	inout [15:0] io_bg_sram_dq,
	output logic o_bg_sram_lb_n,
	output logic o_bg_sram_oe_n,
	output logic o_bg_sram_ub_n,
	output logic o_bg_sram_we_n,
	
	// Background palette BRAM interface
	output logic [7:0] o_bg_palette_addr,
	output logic o_bg_palette_wren,
	input logic [15:0] i_bg_palette_data,
	
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

	// Image dimensions
	localparam BG_WIDTH = 1280;
	localparam BG_HEIGHT = 720;
	
	// ============================================================
	// Combinational Logic: Address Calculation (_w)
	// ============================================================
	
	// Background pixel byte address calculation
	logic [20:0] bg_pixel_byte_addr_w;
	assign bg_pixel_byte_addr_w = (i_v_count * BG_WIDTH) + i_h_count;
	
	// Zombie area check and local coordinates
	logic in_zombie_area_w;
	logic [10:0] zombie_local_x_w;
	logic [9:0] zombie_local_y_w;
	
	assign in_zombie_area_w = (i_h_count >= i_zombie_x1) && 
	                          (i_h_count < (i_zombie_x1 + i_zombie_size_x)) &&
	                          (i_v_count >= i_zombie_y1) && 
	                          (i_v_count < (i_zombie_y1 + i_zombie_size_y));
	
	assign zombie_local_x_w = i_h_count - i_zombie_x1;
	assign zombie_local_y_w = i_v_count - i_zombie_y1;
	
	// Zombie pixel address calculation
	logic [13:0] zombie_pixel_addr_w;
	assign zombie_pixel_addr_w = in_zombie_area_w ? 
	                            ((zombie_local_y_w * i_zombie_size_x) + zombie_local_x_w) : 
	                            14'd0;
	
	// ============================================================
	// Sequential Logic: Pipeline Stage 1 - Read Image Data (_r)
	// ============================================================
	
	// Background SRAM address and control (registered)
	logic [20:0] bg_pixel_byte_addr_r;
	logic [7:0] bg_pixel_index_r;
	logic in_zombie_area_r;
	logic [7:0] zombie_pixel_index_r;
	logic [7:0] bg_palette_addr_w;
	logic [7:0] zombie_palette_addr_w;
	
	// Background SRAM control signals
	assign o_bg_sram_ce_n = 1'b0;
	assign o_bg_sram_oe_n = 1'b0;
	assign o_bg_sram_we_n = 1'b1;  // Read mode
	assign o_bg_sram_addr = bg_pixel_byte_addr_w[20:1];  // Word address
	assign o_bg_sram_lb_n = bg_pixel_byte_addr_w[0];     // Low byte select
	assign o_bg_sram_ub_n = ~bg_pixel_byte_addr_w[0];    // High byte select
	assign io_bg_sram_dq = 16'hZZZZ;  // High-Z for read
	
	// Zombie BRAM control signals
	assign o_zombie_bram_wren = 1'b0;  // Read mode
	assign o_zombie_bram_addr = zombie_pixel_addr_w;
	
	// Pipeline stage 1: Register addresses and read pixel indices
	logic active_video_r1;  // First stage pipeline for active_video
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			bg_pixel_byte_addr_r <= 21'd0;
			bg_pixel_index_r <= 8'd0;
			in_zombie_area_r <= 1'b0;
			zombie_pixel_index_r <= 8'd0;
			active_video_r1 <= 1'b0;
		end else begin
			// Register background address
			bg_pixel_byte_addr_r <= bg_pixel_byte_addr_w;
			
			// Read background pixel index from SRAM
			if (bg_pixel_byte_addr_r[0] == 1'b0) begin
				bg_pixel_index_r <= io_bg_sram_dq[7:0];   // Low byte
			end else begin
				bg_pixel_index_r <= io_bg_sram_dq[15:8];  // High byte
			end
			
			// Register zombie area flag
			in_zombie_area_r <= in_zombie_area_w;
			
			// Read zombie pixel index from BRAM (BRAM has registered output)
			zombie_pixel_index_r <= i_zombie_bram_data;
			
			// Pipeline active video signal (stage 1)
			active_video_r1 <= i_active_video;
		end
	end
	
	// Calculate palette addresses (combinational)
	assign bg_palette_addr_w = bg_pixel_index_r;
	assign zombie_palette_addr_w = zombie_pixel_index_r;
	
	// ============================================================
	// Sequential Logic: Pipeline Stage 2 - Read Palette Data (_r)
	// ============================================================
	
	// Background palette BRAM control
	assign o_bg_palette_wren = 1'b0;  // Read mode
	assign o_bg_palette_addr = bg_palette_addr_w;
	
	// Zombie palette BRAM control
	assign o_zombie_palette_wren = 1'b0;  // Read mode
	assign o_zombie_palette_addr = zombie_palette_addr_w;
	
	// Registered palette addresses and data
	logic [7:0] bg_palette_addr_r;
	logic [7:0] zombie_palette_addr_r;
	logic [15:0] bg_rgb565_r;
	logic [15:0] zombie_rgb565_r;
	logic in_zombie_area_r2;  // Pipeline zombie area flag
	logic [7:0] zombie_pixel_index_r2;  // Pipeline zombie pixel index for transparency check
	logic active_video_r;  // Pipeline active video signal
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			bg_palette_addr_r <= 8'd0;
			zombie_palette_addr_r <= 8'd0;
			bg_rgb565_r <= 16'd0;
			zombie_rgb565_r <= 16'd0;
			in_zombie_area_r2 <= 1'b0;
			zombie_pixel_index_r2 <= 8'd0;
			active_video_r <= 1'b0;
		end else begin
			// Register palette addresses
			bg_palette_addr_r <= bg_palette_addr_w;
			zombie_palette_addr_r <= zombie_palette_addr_w;
			
			// Read palette data (BRAM has registered output, 1 cycle delay)
			bg_rgb565_r <= i_bg_palette_data;
			zombie_rgb565_r <= i_zombie_palette_data;
			
			// Pipeline zombie area flag and pixel index
			in_zombie_area_r2 <= in_zombie_area_r;
			zombie_pixel_index_r2 <= zombie_pixel_index_r;
			
			// Pipeline active video signal (stage 2, total 2 cycles delay to match RGB data)
			active_video_r <= active_video_r1;
		end
	end
	
	// ============================================================
	// Combinational Logic: RGB565 to RGB888 Conversion
	// ============================================================
	
	logic [7:0] bg_r_8bit, bg_g_8bit, bg_b_8bit;
	logic [7:0] zombie_r_8bit, zombie_g_8bit, zombie_b_8bit;
	
	// Background RGB565 to RGB888
	assign bg_r_8bit = {bg_rgb565_r[15:11], bg_rgb565_r[15:13]};
	assign bg_g_8bit = {bg_rgb565_r[10:5], bg_rgb565_r[10:9]};
	assign bg_b_8bit = {bg_rgb565_r[4:0], bg_rgb565_r[4:2]};
	
	// Zombie RGB565 to RGB888
	assign zombie_r_8bit = {zombie_rgb565_r[15:11], zombie_rgb565_r[15:13]};
	assign zombie_g_8bit = {zombie_rgb565_r[10:5], zombie_rgb565_r[10:9]};
	assign zombie_b_8bit = {zombie_rgb565_r[4:0], zombie_rgb565_r[4:2]};
	
	// ============================================================
	// Combinational Logic: Color Selection
	// ============================================================
	
	// Check if zombie pixel is transparent (0x00)
	// Use pipelined zombie pixel index from stage 2
	logic use_zombie_color_w;
	assign use_zombie_color_w = in_zombie_area_r2 && (zombie_pixel_index_r2 != 8'h00);
	
	// Final RGB selection (combinational)
	logic [7:0] final_r_w, final_g_w, final_b_w;
	
	assign final_r_w = use_zombie_color_w ? zombie_r_8bit : bg_r_8bit;
	assign final_g_w = use_zombie_color_w ? zombie_g_8bit : bg_g_8bit;
	assign final_b_w = use_zombie_color_w ? zombie_b_8bit : bg_b_8bit;
	
	// ============================================================
	// Sequential Logic: Output RGB Values
	// ============================================================
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			o_vga_r <= 8'd0;
			o_vga_g <= 8'd0;
			o_vga_b <= 8'd0;
		end else begin
			if (active_video_r) begin
				o_vga_r <= final_r_w;
				o_vga_g <= final_g_w;
				o_vga_b <= final_b_w;
			end else begin
				o_vga_r <= 8'd0;
				o_vga_g <= 8'd0;
				o_vga_b <= 8'd0;
			end
		end
	end

endmodule

