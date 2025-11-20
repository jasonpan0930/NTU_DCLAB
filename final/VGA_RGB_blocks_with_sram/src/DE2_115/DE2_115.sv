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

// VGA signals
logic vga_clock_74_25; // 74.25MHz clock from PLL

// VGA 720p signals
logic [10:0] vga_720p_h_count;
logic [9:0] vga_720p_v_count;
logic vga_720p_active_video;

// VGA Image Overlay signals (SRAM interface)
logic [19:0] vga_sram_addr;
logic vga_sram_ce_n, vga_sram_lb_n, vga_sram_oe_n, vga_sram_ub_n, vga_sram_we_n;

// Zombie position and size
logic [10:0] zombie_x1;
logic [9:0] zombie_y1;
logic [10:0] zombie_size_x;
logic [9:0] zombie_size_y;

// Initialize zombie size
assign zombie_size_x = 11'd102;  // Zombie width
assign zombie_size_y = 10'd149;   // Zombie height

// Zombie Position Test Controller
// SW[0]: move right (increase x1)
// SW[1]: move left (decrease x1)
// SW[17]: move down (increase y1)
// SW[16]: move up (decrease y1)
Zombie_Position_Test_Controller zombie_pos_test_ctrl(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.i_sw_right(SW[0]),      // SW[0]: move right
	.i_sw_left(SW[1]),       // SW[1]: move left
	.i_sw_down(SW[17]),      // SW[17]: move down
	.i_sw_up(SW[16]),        // SW[16]: move up
	.i_zombie_size_x(zombie_size_x),
	.i_zombie_size_y(zombie_size_y),
	.i_screen_width(11'd1280),
	.i_screen_height(10'd720),
	.o_zombie_x1(zombie_x1),
	.o_zombie_y1(zombie_y1)
);

// VGA output
logic [7:0] vga_r, vga_g, vga_b;



// VGA Clock PLL (50MHz -> 74.25MHz)
vga_clock vga_clk_pll(
	.clk_clk(CLOCK_50),
	.reset_reset_n(KEY[1]),
	.vga_clock_25_clk(),  // Not used
	.vga_clock_74_25_clk(vga_clock_74_25)
);

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

// VGA Image Overlay Combined module (handles background + zombie overlay)
VGA_Image_Overlay_Combined vga_image_overlay_combined(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.i_h_count(vga_720p_h_count),
	.i_v_count(vga_720p_v_count),
	.i_active_video(vga_720p_active_video),
	.i_zombie_x1(zombie_x1),
	.i_zombie_y1(zombie_y1),
	.i_zombie_size_x(zombie_size_x),
	.i_zombie_size_y(zombie_size_y),
	.o_bg_sram_addr(vga_sram_addr),
	.o_bg_sram_ce_n(vga_sram_ce_n),
	.io_bg_sram_dq(SRAM_DQ),
	.o_bg_sram_lb_n(vga_sram_lb_n),
	.o_bg_sram_oe_n(vga_sram_oe_n),
	.o_bg_sram_ub_n(vga_sram_ub_n),
	.o_bg_sram_we_n(vga_sram_we_n),
	.o_vga_r(vga_r),
	.o_vga_g(vga_g),
	.o_vga_b(vga_b)
);

// SRAM interface connections (for background image data)
assign SRAM_ADDR = vga_sram_addr;
assign SRAM_CE_N = vga_sram_ce_n;
assign SRAM_LB_N = vga_sram_lb_n;
assign SRAM_OE_N = vga_sram_oe_n;
assign SRAM_UB_N = vga_sram_ub_n;
assign SRAM_WE_N = vga_sram_we_n;

// VGA RGB output
assign VGA_R = vga_r;
assign VGA_G = vga_g;
assign VGA_B = vga_b;

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule
