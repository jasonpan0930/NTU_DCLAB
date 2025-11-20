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
	
	// Background palette BRAM - instantiated internally
	// Zombie BRAM - instantiated internally
	// Zombie palette BRAM - instantiated internally
	
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
	// BRAM Instantiations (must be before pipeline stages)
	// ============================================================
	
	// Background palette BRAM
	logic [7:0] bg_palette_addr_reg;
	logic [15:0] bg_palette_data;
	
	background_palette bg_palette_bram(
		.address(bg_palette_addr_reg),
		.clock(i_clk),
		.data(16'd0),
		.wren(1'b0),
		.q(bg_palette_data)
	);
	
	// Zombie image BRAM
	logic [13:0] zombie_bram_addr_reg;
	logic [7:0] zombie_bram_data;
	
	zombie_1x zombie_bram(
		.address(zombie_bram_addr_reg),
		.clock(i_clk),
		.data(8'd0),
		.wren(1'b0),
		.q(zombie_bram_data)
	);
	
	// Zombie palette BRAM
	logic [7:0] zombie_palette_addr_reg;
	logic [15:0] zombie_palette_data;
	
	zombie_1x_palette zombie_palette_bram(
		.address(zombie_palette_addr_reg),
		.clock(i_clk),
		.data(16'd0),
		.wren(1'b0),
		.q(zombie_palette_data)
	);
	
	// ============================================================
	// Sequential Logic: Pipeline Stage 0 - Register Addresses (_r)
	// ============================================================
	
	// Background SRAM control signals (combinational, output immediately)
	assign o_bg_sram_ce_n = 1'b0;
	assign o_bg_sram_oe_n = 1'b0;
	assign o_bg_sram_we_n = 1'b1;  // Read mode
	assign o_bg_sram_addr = bg_pixel_byte_addr_w[20:1];  // Word address
	assign o_bg_sram_lb_n = bg_pixel_byte_addr_w[0];     // Low byte select
	assign o_bg_sram_ub_n = ~bg_pixel_byte_addr_w[0];    // High byte select
	assign io_bg_sram_dq = 16'hZZZZ;  // High-Z for read
	
	// Register addresses for SRAM and BRAM (Stage 0)
	// SRAM needs 2 cycles, so we register the address twice
	logic [20:0] bg_pixel_byte_addr_r0;  // First cycle: register address
	logic [20:0] bg_pixel_byte_addr_r1;  // Second cycle: SRAM data ready
	logic [13:0] zombie_pixel_addr_r0;
	logic in_zombie_area_r0;
	logic active_video_r0;
	
	// Register zombie BRAM address (BRAM requires registered address, 1 cycle delay)
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			zombie_bram_addr_reg <= 14'd0;
			bg_pixel_byte_addr_r0 <= 21'd0;
			bg_pixel_byte_addr_r1 <= 21'd0;
			zombie_pixel_addr_r0 <= 14'd0;
			in_zombie_area_r0 <= 1'b0;
			active_video_r0 <= 1'b0;
		end else begin
			// Register addresses for pipeline
			bg_pixel_byte_addr_r0 <= bg_pixel_byte_addr_w;  // Stage 0: register SRAM address
			bg_pixel_byte_addr_r1 <= bg_pixel_byte_addr_r0;  // Stage 1: SRAM data will be ready next cycle
			zombie_bram_addr_reg <= zombie_pixel_addr_w;    // BRAM address (1 cycle delay)
			zombie_pixel_addr_r0 <= zombie_pixel_addr_w;
			in_zombie_area_r0 <= in_zombie_area_w;
			active_video_r0 <= i_active_video;
		end
	end
	
	// ============================================================
	// Sequential Logic: Pipeline Stage 1 - Read Image Data (_r)
	// ============================================================
	
	// SRAM has 2-cycle delay, so we latch SRAM data here
	// bg_pixel_byte_addr_r1 was set 2 cycles ago, so SRAM data is ready now
	logic [15:0] sram_dq_r;  // Latch SRAM data
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			sram_dq_r <= 16'd0;
		end else begin
			sram_dq_r <= io_bg_sram_dq;  // Latch SRAM output (after 2 cycles)
		end
	end
	
	logic [7:0] bg_pixel_index_r;
	logic in_zombie_area_r;
	logic [7:0] zombie_pixel_index_r;
	logic active_video_r1;  // First stage pipeline for active_video
	
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			bg_pixel_index_r <= 8'd0;
			in_zombie_area_r <= 1'b0;
			zombie_pixel_index_r <= 8'd0;
			active_video_r1 <= 1'b0;
		end else begin
			// Read background pixel index from SRAM (using latched data after 2 cycles)
			// bg_pixel_byte_addr_r1 corresponds to the address we sent 2 cycles ago
			if (bg_pixel_byte_addr_r1[0] == 1'b0) begin
				bg_pixel_index_r <= sram_dq_r[7:0];   // Low byte
			end else begin
				bg_pixel_index_r <= sram_dq_r[15:8];  // High byte
			end
			
			// Pipeline zombie area flag (sync with background)
			in_zombie_area_r <= in_zombie_area_r0;
			
			// Read zombie pixel index from BRAM (BRAM has registered output, 1 cycle delay)
			// zombie_bram_addr_reg was set 1 cycle ago, so data is ready now
			zombie_pixel_index_r <= zombie_bram_data;
			
			// Pipeline active video signal (stage 1)
			active_video_r1 <= active_video_r0;
		end
	end
	
	// Calculate palette addresses (combinational)
	logic [7:0] bg_palette_addr_w;
	logic [7:0] zombie_palette_addr_w;
	assign bg_palette_addr_w = bg_pixel_index_r;
	assign zombie_palette_addr_w = zombie_pixel_index_r;
	
	// Register palette BRAM addresses (BRAM requires registered addresses)
	always_ff @(posedge i_clk) begin
		if (~i_rst_n) begin
			bg_palette_addr_reg <= 8'd0;
			zombie_palette_addr_reg <= 8'd0;
		end else begin
			bg_palette_addr_reg <= bg_palette_addr_w;
			zombie_palette_addr_reg <= zombie_palette_addr_w;
		end
	end
	
	// ============================================================
	// Sequential Logic: Pipeline Stage 2 - Read Palette Data (_r)
	// ============================================================
	
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
			bg_rgb565_r <= bg_palette_data;
			zombie_rgb565_r <= zombie_palette_data;
			
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

