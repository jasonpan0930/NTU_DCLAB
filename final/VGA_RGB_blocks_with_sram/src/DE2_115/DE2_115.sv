module DE2_115 (
	input CLOCK_50,
	input CLOCK2_50,
	input CLOCK3_50,
	input ENETCLK_25,
	input SMA_CLKIN,
	output SMA_CLKOUT,
	output [8:0] LEDG,
	output [17:0] LEDR,
	input [3:0] KEY,
	input [17:0] SW,
	output [6:0] HEX0,
	output [6:0] HEX1,
	output [6:0] HEX2,
	output [6:0] HEX3,
	output [6:0] HEX4,
	output [6:0] HEX5,
	output [6:0] HEX6,
	output [6:0] HEX7,
	output LCD_BLON,
	inout [7:0] LCD_DATA,
	output LCD_EN,
	output LCD_ON,
	output LCD_RS,
	output LCD_RW,
	output UART_CTS,
	input UART_RTS,
	input UART_RXD,
	output UART_TXD,
	inout PS2_CLK,
	inout PS2_DAT,
	inout PS2_CLK2,
	inout PS2_DAT2,
	output SD_CLK,
	inout SD_CMD,
	inout [3:0] SD_DAT,
	input SD_WP_N,
	output [7:0] VGA_B,
	output VGA_BLANK_N,
	output VGA_CLK,
	output [7:0] VGA_G,
	output VGA_HS,
	output [7:0] VGA_R,
	output VGA_SYNC_N,
	output VGA_VS,
	input AUD_ADCDAT,
	inout AUD_ADCLRCK,
	inout AUD_BCLK,
	output AUD_DACDAT,
	inout AUD_DACLRCK,
	output AUD_XCK,
	output EEP_I2C_SCLK,
	inout EEP_I2C_SDAT,
	output I2C_SCLK,
	inout I2C_SDAT,
	output ENET0_GTX_CLK,
	input ENET0_INT_N,
	output ENET0_MDC,
	input ENET0_MDIO,
	output ENET0_RST_N,
	input ENET0_RX_CLK,
	input ENET0_RX_COL,
	input ENET0_RX_CRS,
	input [3:0] ENET0_RX_DATA,
	input ENET0_RX_DV,
	input ENET0_RX_ER,
	input ENET0_TX_CLK,
	output [3:0] ENET0_TX_DATA,
	output ENET0_TX_EN,
	output ENET0_TX_ER,
	input ENET0_LINK100,
	output ENET1_GTX_CLK,
	input ENET1_INT_N,
	output ENET1_MDC,
	input ENET1_MDIO,
	output ENET1_RST_N,
	input ENET1_RX_CLK,
	input ENET1_RX_COL,
	input ENET1_RX_CRS,
	input [3:0] ENET1_RX_DATA,
	input ENET1_RX_DV,
	input ENET1_RX_ER,
	input ENET1_TX_CLK,
	output [3:0] ENET1_TX_DATA,
	output ENET1_TX_EN,
	output ENET1_TX_ER,
	input ENET1_LINK100,
	input TD_CLK27,
	input [7:0] TD_DATA,
	input TD_HS,
	output TD_RESET_N,
	input TD_VS,
	inout [15:0] OTG_DATA,
	output [1:0] OTG_ADDR,
	output OTG_CS_N,
	output OTG_WR_N,
	output OTG_RD_N,
	input OTG_INT,
	output OTG_RST_N,
	input IRDA_RXD,
	output [12:0] DRAM_ADDR,
	output [1:0] DRAM_BA,
	output DRAM_CAS_N,
	output DRAM_CKE,
	output DRAM_CLK,
	output DRAM_CS_N,
	inout [31:0] DRAM_DQ,
	output [3:0] DRAM_DQM,
	output DRAM_RAS_N,
	output DRAM_WE_N,
	output [19:0] SRAM_ADDR,
	output SRAM_CE_N,
	inout [15:0] SRAM_DQ,
	output SRAM_LB_N,
	output SRAM_OE_N,
	output SRAM_UB_N,
	output SRAM_WE_N,
	output [22:0] FL_ADDR,
	output FL_CE_N,
	inout [7:0] FL_DQ,
	output FL_OE_N,
	output FL_RST_N,
	input FL_RY,
	output FL_WE_N,
	output FL_WP_N,
	inout [35:0] GPIO,
	input HSMC_CLKIN_P1,
	input HSMC_CLKIN_P2,
	input HSMC_CLKIN0,
	output HSMC_CLKOUT_P1,
	output HSMC_CLKOUT_P2,
	output HSMC_CLKOUT0,
	inout [3:0] HSMC_D,
	input [16:0] HSMC_RX_D_P,
	output [16:0] HSMC_TX_D_P,
	inout [6:0] EX_IO
);

logic keydown;
logic [12:0] random_value;
assign random_value = 13'd0;

// VGA signals
logic [9:0] vga_h_count;
logic [9:0] vga_v_count;
logic vga_active_video;
logic vga_clock_25;   // 25MHz clock from PLL
logic vga_clock_74_25; // 74.25MHz clock from PLL

// VGA 720p signals
logic [10:0] vga_720p_h_count;
logic [9:0] vga_720p_v_count;
logic vga_720p_active_video;

// SRAM Reader signals (for debugging)
logic [7:0] sram_byte_data;
logic [19:0] sram_reader_addr;
logic sram_reader_ce_n, sram_reader_lb_n, sram_reader_oe_n, sram_reader_ub_n, sram_reader_we_n;

// BRAM Reader signals (for debugging)
logic [7:0] bram_debug_addr;
logic bram_debug_wren;
logic [15:0] bram_data_out;
logic [15:0] palette_data;

// VGA Image Display signals
logic [19:0] vga_sram_addr;
logic vga_sram_ce_n, vga_sram_lb_n, vga_sram_oe_n, vga_sram_ub_n, vga_sram_we_n;
logic [7:0] vga_bram_addr;
logic vga_bram_wren;
logic [7:0] bg_r, bg_g, bg_b;  // Background RGB output

// Zombie position and size
logic [10:0] zombie_x1;
logic [9:0] zombie_y1;
logic [10:0] zombie_size_x;
logic [9:0] zombie_size_y;

// Initialize zombie position and size
assign zombie_x1 = 11'd0;
assign zombie_y1 = 10'd0;
assign zombie_size_x = 11'd102;  // Zombie width
assign zombie_size_y = 10'd149;   // Zombie height

// Zombie BRAM signals
logic [13:0] zombie_bram_addr;
logic zombie_bram_wren;
logic [7:0] zombie_bram_data_out;

// Zombie palette BRAM signals
logic [7:0] zombie_palette_addr;
logic zombie_palette_wren;
logic [15:0] zombie_palette_data_out;

// VGA Overlay output
logic [7:0] vga_r, vga_g, vga_b;

// Mode selection: SW[17] = 0 for VGA display, SW[17] = 1 for debug mode
logic debug_mode;
assign debug_mode = SW[17];

Debounce deb0(
	.i_in(KEY[0]),
	.i_rst_n(KEY[1]),
	.i_clk(CLOCK_50),
	.o_neg(keydown)
);

// SRAM Reader module (for debugging, only active when debug_mode = 1)
SRAM_Reader sram_reader0(
	.i_clk(CLOCK_50),
	.i_rst_n(KEY[1]),
	.i_keydown(keydown),
	.o_sram_addr(sram_reader_addr),
	.o_sram_ce_n(sram_reader_ce_n),
	.io_sram_dq(SRAM_DQ),
	.o_sram_lb_n(sram_reader_lb_n),
	.o_sram_oe_n(sram_reader_oe_n),
	.o_sram_ub_n(sram_reader_ub_n),
	.o_sram_we_n(sram_reader_we_n),
	.o_byte_data(sram_byte_data)
);

// SRAM address/control multiplexer: VGA display has priority
assign SRAM_ADDR = debug_mode ? sram_reader_addr : vga_sram_addr;
assign SRAM_CE_N = debug_mode ? sram_reader_ce_n : vga_sram_ce_n;
assign SRAM_LB_N = debug_mode ? sram_reader_lb_n : vga_sram_lb_n;
assign SRAM_OE_N = debug_mode ? sram_reader_oe_n : vga_sram_oe_n;
assign SRAM_UB_N = debug_mode ? sram_reader_ub_n : vga_sram_ub_n;
assign SRAM_WE_N = debug_mode ? sram_reader_we_n : vga_sram_we_n;

// Display byte data on HEX0 and HEX1 using HexTo7Seg
HexTo7Seg hex_decoder_low(
	.i_hex(sram_byte_data[3:0]),  // Low 4 bits
	.o_seg(HEX0)
);

HexTo7Seg hex_decoder_high(
	.i_hex(sram_byte_data[7:4]),  // High 4 bits
	.o_seg(HEX1)
);

// BRAM module - need two instances: one for VGA (74.25MHz) and one for debug (50MHz)
// VGA BRAM instance
logic [7:0] vga_bram_addr_reg;
logic [15:0] vga_bram_data_out;

background_palette bram_vga(
	.address(vga_bram_addr_reg),
	.clock(vga_clock_74_25),  // VGA clock domain
	.data(16'd0),
	.wren(1'b0),
	.q(vga_bram_data_out)
);

// Debug BRAM instance
background_palette bram_debug(
	.address(bram_debug_addr),
	.clock(CLOCK_50),  // Debug clock domain
	.data(16'd0),
	.wren(bram_debug_wren),
	.q(bram_data_out)
);

// BRAM Reader module (for debugging)
BRAM_Reader bram_reader0(
	.i_clk(CLOCK_50),
	.i_rst_n(KEY[1]),
	.i_keydown(keydown),
	.o_bram_addr(bram_debug_addr),
	.o_bram_wren(bram_debug_wren),
	.i_bram_data(bram_data_out),
	.o_palette_data(palette_data)
);

// Display palette data on HEX7~HEX4 (16-bit value, 4 hex digits)
HexTo7Seg hex_palette_0(
	.i_hex(palette_data[3:0]),   // Least significant 4 bits
	.o_seg(HEX4)
);

HexTo7Seg hex_palette_1(
	.i_hex(palette_data[7:4]),
	.o_seg(HEX5)
);

HexTo7Seg hex_palette_2(
	.i_hex(palette_data[11:8]),
	.o_seg(HEX6)
);

HexTo7Seg hex_palette_3(
	.i_hex(palette_data[15:12]), // Most significant 4 bits
	.o_seg(HEX7)
);



// VGA Clock PLL (50MHz -> 25MHz and 74.25MHz)
vga_clock vga_clk_pll(
	.clk_clk(CLOCK_50),
	.reset_reset_n(KEY[1]),
	.vga_clock_25_clk(vga_clock_25),
	.vga_clock_74_25_clk(vga_clock_74_25)
);

// // VGA Controller
// VGA_Controller vga_ctrl0(
// 	.i_clk(vga_clock_25),  // Use PLL-generated 25MHz clock
// 	.i_rst_n(KEY[1]),
// 	.o_vga_clk(VGA_CLK),
// 	.o_hsync(VGA_HS),
// 	.o_vsync(VGA_VS),
// 	.o_blank_n(VGA_BLANK_N),
// 	.o_sync_n(VGA_SYNC_N),
// 	.o_h_count(vga_h_count),
// 	.o_v_count(vga_v_count),
// 	.o_active_video(vga_active_video)
// );

// // VGA Pattern Generator (640x480)
// VGA_Pattern vga_pattern0(
// 	.i_clk(CLOCK_50),
// 	.i_rst_n(KEY[1]),
// 	.i_h_count(vga_h_count),
// 	.i_v_count(vga_v_count),
// 	.i_active_video(vga_active_video),
// 	.i_rgb_enable({SW[2], SW[1], SW[0]}), // SW[0]=R, SW[1]=G, SW[2]=B
// 	.o_vga_r(VGA_R),
// 	.o_vga_g(VGA_G),
// 	.o_vga_b(VGA_B)
// );

// VGA Controller 720p
VGA_Controller_720p vga_ctrl_720p(
	.i_clk(vga_clock_74_25),  // Use PLL-generated 74.25MHz clock
	.i_rst_n(KEY[1]),
	.o_vga_clk(VGA_CLK),      // Connected to external VGA_CLK pin
	.o_hsync(VGA_HS),         // Connected to external VGA_HS pin
	.o_vsync(VGA_VS),         // Connected to external VGA_VS pin
	.o_blank_n(VGA_BLANK_N),  // Connected to external VGA_BLANK_N pin
	.o_sync_n(VGA_SYNC_N),    // Connected to external VGA_SYNC_N pin
	.o_h_count(vga_720p_h_count),
	.o_v_count(vga_720p_v_count),
	.o_active_video(vga_720p_active_video)
);

// VGA Image Display module (for background)
VGA_Image_Display vga_image_display(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.i_h_count(vga_720p_h_count),
	.i_v_count(vga_720p_v_count),
	.i_active_video(vga_720p_active_video),
	.o_sram_addr(vga_sram_addr),
	.o_sram_ce_n(vga_sram_ce_n),
	.io_sram_dq(SRAM_DQ),
	.o_sram_lb_n(vga_sram_lb_n),
	.o_sram_oe_n(vga_sram_oe_n),
	.o_sram_ub_n(vga_sram_ub_n),
	.o_sram_we_n(vga_sram_we_n),
	.o_bram_addr(vga_bram_addr),
	.o_bram_wren(vga_bram_wren),
	.i_bram_data(vga_bram_data_out),
	.o_vga_r(bg_r),
	.o_vga_g(bg_g),
	.o_vga_b(bg_b)
);

// Zombie BRAM instance
zombie_1x zombie_bram(
	.address(zombie_bram_addr),
	.clock(vga_clock_74_25),
	.data(8'd0),
	.wren(1'b0),
	.q(zombie_bram_data_out)
);

// Zombie palette BRAM instance
zombie_1x_palette zombie_palette_bram(
	.address(zombie_palette_addr),
	.clock(vga_clock_74_25),
	.data(16'd0),
	.wren(1'b0),
	.q(zombie_palette_data_out)
);

// VGA Image Overlay module (overlays zombie on background)
VGA_Image_Overlay vga_image_overlay(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.i_h_count(vga_720p_h_count),
	.i_v_count(vga_720p_v_count),
	.i_active_video(vga_720p_active_video),
	.i_bg_r(bg_r),
	.i_bg_g(bg_g),
	.i_bg_b(bg_b),
	.i_zombie_x1(zombie_x1),
	.i_zombie_y1(zombie_y1),
	.i_zombie_size_x(zombie_size_x),
	.i_zombie_size_y(zombie_size_y),
	.o_zombie_bram_addr(zombie_bram_addr),
	.o_zombie_bram_wren(zombie_bram_wren),
	.i_zombie_bram_data(zombie_bram_data_out),
	.o_zombie_palette_addr(zombie_palette_addr),
	.o_zombie_palette_wren(zombie_palette_wren),
	.i_zombie_palette_data(zombie_palette_data_out),
	.o_vga_r(vga_r),
	.o_vga_g(vga_g),
	.o_vga_b(vga_b)
);

// Register VGA BRAM address (BRAM needs registered address)
always_ff @(posedge vga_clock_74_25) begin
	if (~KEY[1]) begin
		vga_bram_addr_reg <= 8'd0;
	end else begin
		vga_bram_addr_reg <= vga_bram_addr;
	end
end

// VGA RGB output multiplexer: use image display when not in debug mode
assign VGA_R = debug_mode ? 8'd0 : vga_r;
assign VGA_G = debug_mode ? 8'd0 : vga_g;
assign VGA_B = debug_mode ? 8'd0 : vga_b;

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule
