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

Debounce deb0(
	.i_in(KEY[0]),
	.i_rst_n(KEY[1]),
	.i_clk(CLOCK_50),
	.o_neg(keydown)
);

 SevenDec13 seven_dec0(
	.i_val(random_value),
	.o_hex0(HEX0),
	.o_hex1(HEX1),
	.o_hex2(HEX2),
	.o_hex3(HEX3)
);



// VGA Clock PLL (50MHz -> 25MHz and 74.25MHz)
vga_clock vga_clk_pll(
	.clk_clk(CLOCK_50),
	.reset_reset_n(KEY[1]),
	.clock_25_clk(vga_clock_25),
	.clock_74_25_clk(vga_clock_74_25)
);

// // VGA Controller
VGA_Controller vga_ctrl0(
	.i_clk(vga_clock_25),  // Use PLL-generated 25MHz clock
	.i_rst_n(KEY[1]),
	.o_vga_clk(VGA_CLK),
	.o_hsync(VGA_HS),
	.o_vsync(VGA_VS),
	.o_blank_n(VGA_BLANK_N),
	.o_sync_n(VGA_SYNC_N),
	.o_h_count(vga_h_count),
	.o_v_count(vga_v_count),
	.o_active_video(vga_active_video)
);

// VGA Pattern Generator (640x480)
VGA_Pattern vga_pattern0(
	.i_clk(CLOCK_50),
	.i_rst_n(KEY[1]),
	.i_h_count(vga_h_count),
	.i_v_count(vga_v_count),
	.i_active_video(vga_active_video),
	.i_rgb_enable({SW[2], SW[1], SW[0]}), // SW[0]=R, SW[1]=G, SW[2]=B
	.o_vga_r(VGA_R),
	.o_vga_g(VGA_G),
	.o_vga_b(VGA_B)
);

// VGA Controller 720p
// VGA_Controller_720p vga_ctrl_720p(
// 	.i_clk(vga_clock_74_25),  // Use PLL-generated 74.25MHz clock
// 	.i_rst_n(KEY[1]),
// 	.o_vga_clk(VGA_CLK),      // Connected to external VGA_CLK pin
// 	.o_hsync(VGA_HS),         // Connected to external VGA_HS pin
// 	.o_vsync(VGA_VS),         // Connected to external VGA_VS pin
// 	.o_blank_n(VGA_BLANK_N),  // Connected to external VGA_BLANK_N pin
// 	.o_sync_n(VGA_SYNC_N),    // Connected to external VGA_SYNC_N pin
// 	.o_h_count(vga_720p_h_count),
// 	.o_v_count(vga_720p_v_count),
// 	.o_active_video(vga_720p_active_video)
// );

// // VGA Pattern Generator 720p
// VGA_Pattern_720p vga_pattern_720p(
// 	.i_clk(vga_clock_74_25),
// 	.i_rst_n(KEY[1]),
// 	.i_h_count(vga_720p_h_count),
// 	.i_v_count(vga_720p_v_count),
// 	.i_active_video(vga_720p_active_video),
// 	.i_rgb_enable({SW[2], SW[1], SW[0]}), // SW[0]=R, SW[1]=G, SW[2]=B
// 	.o_vga_r(VGA_R),  // Connected to external VGA_R pins
// 	.o_vga_g(VGA_G),  // Connected to external VGA_G pins
// 	.o_vga_b(VGA_B)   // Connected to external VGA_B pins
// );

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule
