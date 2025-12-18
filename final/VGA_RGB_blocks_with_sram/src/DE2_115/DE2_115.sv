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

// Zombie position arrays (for multiple zombies)
logic [10:0] zombie_x [0:9];  // Array of X positions (MAX_ZOMBIES = 10)
logic [9:0] zombie_y [0:9];   // Array of Y positions
logic zombie_valid [0:9];     // Array of valid flags
logic [31:0] zombie_distance [0:9];  // Distance to target for each zombie
logic [3:0] active_zombie_count;     // Number of active zombies

// Zombie size
logic [10:0] zombie_size_x;
logic [9:0] zombie_size_y;

// Initialize zombie size
assign zombie_size_x = 11'd102;  // Zombie width
assign zombie_size_y = 10'd149;   // Zombie height

// Aim position (controlled by switches for testing)
logic [10:0] aim_x;
logic [9:0] aim_y;

// Aim size
localparam AIM_SIZE_X = 200;
localparam AIM_SIZE_Y = 200;

// I2C and IMU signals
logic i2c_start;
logic i2c_rw;
logic [6:0] i2c_dev_addr;
logic [7:0] i2c_reg_addr;
logic [7:0] i2c_wr_data;
logic [7:0] i2c_rd_data;
logic i2c_busy;
logic i2c_done;
logic i2c_ack_error;
logic i2c_scl;
wire i2c_sda;  // inout signal should be wire

// IMU output signals
logic signed [31:0] imu_v_x;
logic signed [31:0] imu_v_y;
logic signed [31:0] imu_v_z;
logic signed [15:0] imu_gyro_x;
logic signed [15:0] imu_gyro_y;
logic signed [15:0] imu_gyro_z;
logic imu_data_valid;

/*******************************test code*******************************/
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

// // Clock divider for aim movement (74.25MHz / 580078 ≈ 128 Hz)

localparam AIM_MOVEMENT_DIVIDER = 26'd580078;  // ~128 movements per second
logic [25:0] aim_movement_counter;
logic aim_movement_tick;

// Clock divider: generate movement tick
always_ff @(posedge vga_clock_74_25 or negedge KEY[1]) begin
	if (!KEY[1]) begin
		aim_movement_counter <= 26'd0;
		aim_movement_tick <= 1'b0;
	end else begin
		if (aim_movement_counter >= AIM_MOVEMENT_DIVIDER - 1) begin
			aim_movement_counter <= 26'd0;
			aim_movement_tick <= 1'b1;
		end else begin
			aim_movement_counter <= aim_movement_counter + 26'd1;
			aim_movement_tick <= 1'b0;
		end
	end
end

// Aim position control with switches (DISABLED - now controlled by Position_Controller)
// SW[0]: move right
// SW[1]: move left
// SW[2]: move down
// SW[3]: move up
// Commented out because aim_x and aim_y are now driven by Position_Controller
// always_ff @(posedge vga_clock_74_25 or negedge KEY[1]) begin
// 	if (!KEY[1]) begin
// 		// aim_x, aim_y is the center of the aim
// 		aim_x <= 11'd640;  // Center X: 1280/2 = 640
// 		aim_y <= 10'd360;  // Center Y: 720/2 = 360
// 	end else if (aim_movement_tick) begin
// 		// X position control (SW[0] = right, SW[1] = left)
// 		if (SW[0] && !SW[1]) begin
// 			// Move right (increase x)
// 			if (aim_x < (11'd1280 - 100)) begin
// 				aim_x <= aim_x + 11'd1;
// 			end
// 		end else if (SW[1] && !SW[0]) begin
// 			// Move left (decrease x)
// 			if (aim_x > 100) begin
// 				aim_x <= aim_x - 11'd1;
// 			end
// 		end
// 		
// 		// Y position control (SW[2] = down, SW[3] = up)
// 		if (SW[2] && !SW[3]) begin
// 			// Move down (increase y)
// 			if (aim_y < (10'd720 - 100)) begin
// 				aim_y <= aim_y + 10'd1;
// 			end
// 		end else if (SW[3] && !SW[2]) begin
// 			// Move up (decrease y)
// 			if (aim_y > 100) begin
// 				aim_y <= aim_y - 10'd1;
// 			end
// 		end
// 	end
// end

// Show current X on HEX[3:0] and current Y on HEX[7:4] with decimal display
// HEX0: X ones digit
// HEX1: X tens digit
// HEX2: X hundreds digit
// HEX3: X thousands digit
// HEX4: Y ones digit
// HEX5: Y tens digit
// HEX6: Y hundreds digit
// HEX7: blank (Y is 0-719, so no thousands)

// Extract decimal digits from aim_x (0-1279)
logic [3:0] aim_x_ones, aim_x_tens, aim_x_hundreds, aim_x_thousands;
logic [3:0] aim_y_ones, aim_y_tens, aim_y_hundreds;

// X position digit extraction (combinational)
always_comb begin
	aim_x_ones = (aim_x % 10);
	aim_x_tens = ((aim_x / 10) % 10);
	aim_x_hundreds = ((aim_x / 100) % 10);
	aim_x_thousands = (aim_x / 1000);
end

// Y position digit extraction (combinational)
always_comb begin
	aim_y_ones = (aim_y % 10);
	aim_y_tens = ((aim_y / 10) % 10);
	aim_y_hundreds = (aim_y / 100);
end

// 7-segment decoders for X position
HexTo7Seg hex_x0(.i_hex(aim_x_ones), .o_seg(HEX0));
HexTo7Seg hex_x1(.i_hex(aim_x_tens), .o_seg(HEX1));
HexTo7Seg hex_x2(.i_hex(aim_x_hundreds), .o_seg(HEX2));
HexTo7Seg hex_x3(.i_hex(aim_x_thousands), .o_seg(HEX3));

// 7-segment decoders for Y position
HexTo7Seg hex_y0(.i_hex(aim_y_ones), .o_seg(HEX4));
HexTo7Seg hex_y1(.i_hex(aim_y_tens), .o_seg(HEX5));
HexTo7Seg hex_y2(.i_hex(aim_y_hundreds), .o_seg(HEX6));
assign HEX7 = 7'b1111111;  // Blank (Y position is 0-719, no thousands digit)

/****************************test code end*****************************/
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/




// Random Position Generator - DISABLED FOR TESTING
// Assign all zombie positions to 0 for testing
Random_Position_Generator #(
	.MAX_POSITIONS(10),  // Must match MAX_ZOMBIES in VGA_Image_Overlay_Combined
	.CLOCK_FREQ(32'd74250000)  // 74.25MHz
) zombie_gen(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.o_x(zombie_x),
	.o_y(zombie_y),
	.o_valid(zombie_valid),
	.o_distance(zombie_distance),
	.o_active_count(active_zombie_count)
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

// VGA Image Overlay Combined module (handles background + multiple zombie overlays)
VGA_Image_Overlay_Combined #(
	.MAX_ZOMBIES(10)  // Must match MAX_POSITIONS in Random_Position_Generator
) vga_image_overlay_combined(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.i_h_count(vga_720p_h_count),
	.i_v_count(vga_720p_v_count),
	.i_active_video(vga_720p_active_video),
	.i_zombie_x(zombie_x),          // Array of X positions
	.i_zombie_y(zombie_y),          // Array of Y positions
	.i_zombie_valid(zombie_valid),  // Array of valid flags
	.i_zombie_size_x(zombie_size_x),
	.i_zombie_size_y(zombie_size_y),
	.i_aim_x(aim_x - 100),                   // Aim X position
	.i_aim_y(aim_y - 100),                   // Aim Y position
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

// I2C Master module instantiation
i2c_master #(
	.CLK_HZ(50_000_000),  // 50MHz system clock
	.I2C_HZ(100_000)      // 100kHz I2C clock
) i2c_master_inst(
	.clk(CLOCK_50),
	.rst_n(KEY[1]),
	.start(i2c_start),
	.rw(i2c_rw),
	.dev_addr(i2c_dev_addr),
	.reg_addr(i2c_reg_addr),
	.wr_data(i2c_wr_data),
	.rd_data(i2c_rd_data),
	.busy(i2c_busy),
	.done(i2c_done),
	.ack_error(i2c_ack_error),
	.scl(i2c_scl),
	.sda(i2c_sda)  // inout wire connection
);

// MPU6050 IMU Controller module instantiation
mpu6050_ctrl #(
	.CLK_HZ(50_000_000),  // 50MHz system clock
	.I2C_ADDR(7'h68)      // MPU6050 I2C address (AD0=0)
) mpu6050_ctrl_inst(
	.clk(CLOCK_50),
	.rst_n(KEY[1]),
	.i2c_start(i2c_start),
	.i2c_rw(i2c_rw),
	.i2c_dev_addr(i2c_dev_addr),
	.i2c_reg_addr(i2c_reg_addr),
	.i2c_wr_data(i2c_wr_data),
	.i2c_rd_data(i2c_rd_data),
	.i2c_busy(i2c_busy),
	.i2c_done(i2c_done),
	.i2c_ack_error(i2c_ack_error),
	.v_x(imu_v_x),
	.v_y(imu_v_y),
	.v_z(imu_v_z),
	.gyro_x(imu_gyro_x),
	.gyro_y(imu_gyro_y),
	.gyro_z(imu_gyro_z),
	.data_valid(imu_data_valid)
);

// Synchronize IMU data from 50MHz domain to 74.25MHz domain for Position_Controller
// This prevents metastability issues when crossing clock domains
logic signed [15:0] imu_gyro_x_sync, imu_gyro_y_sync, imu_gyro_z_sync;
logic signed [15:0] imu_gyro_x_sync1, imu_gyro_y_sync1, imu_gyro_z_sync1;
logic signed [15:0] imu_gyro_x_sync2, imu_gyro_y_sync2, imu_gyro_z_sync2;

always_ff @(posedge vga_clock_74_25 or negedge KEY[1]) begin
	if (!KEY[1]) begin
		imu_gyro_x_sync1 <= 16'd0;
		imu_gyro_y_sync1 <= 16'd0;
		imu_gyro_z_sync1 <= 16'd0;
		imu_gyro_x_sync2 <= 16'd0;
		imu_gyro_y_sync2 <= 16'd0;
		imu_gyro_z_sync2 <= 16'd0;
		imu_gyro_x_sync <= 16'd0;
		imu_gyro_y_sync <= 16'd0;
		imu_gyro_z_sync <= 16'd0;
	end else begin
		// Two-stage synchronizer to safely cross clock domains
		imu_gyro_x_sync1 <= imu_gyro_x;
		imu_gyro_y_sync1 <= imu_gyro_y;
		imu_gyro_z_sync1 <= imu_gyro_z;
		imu_gyro_x_sync2 <= imu_gyro_x_sync1;
		imu_gyro_y_sync2 <= imu_gyro_y_sync1;
		imu_gyro_z_sync2 <= imu_gyro_z_sync1;
		imu_gyro_x_sync <= imu_gyro_x_sync2;
		imu_gyro_y_sync <= imu_gyro_y_sync2;
		imu_gyro_z_sync <= imu_gyro_z_sync2;
	end
end

// Position Controller module instantiation (for IMU-based aim control)
Position_Controller #(
	.SCREEN_WIDTH(1280),
	.SCREEN_HEIGHT(720),
	.KX(32'd100000),          // Horizontal sensitivity (adjustable)
	.KY(32'd100000),          // Vertical sensitivity (adjustable)
	.UPDATE_DIVIDER(24'd74250), // ~1000Hz update rate at 74.25MHz
	.USE_LINEAR_VELOCITY(0)    // Disable linear velocity for now
) position_controller_inst(
	.i_clk(vga_clock_74_25),   // Use VGA clock (74.25MHz)
	.i_rst_n(KEY[1]),
	.i_wx(imu_gyro_x_sync),    // Angular velocity X (pitch) → controls Y (synchronized)
	.i_wy(imu_gyro_y_sync),    // Angular velocity Y (roll) → not used (synchronized)
	.i_wz(imu_gyro_z_sync),    // Angular velocity Z (yaw) → controls X (synchronized)
	.i_vx(32'd0),              // Linear velocity X (not used)
	.i_vy(32'd0),              // Linear velocity Y (not used)
	.i_vz(32'd0),              // Linear velocity Z (not used)
	.o_x(aim_x),               // Output X position to aim_x
	.o_y(aim_y)                // Output Y position to aim_y
);

// I2C physical pin connections
// SCL connected to GPIO[0], SDA connected to GPIO[2]
assign GPIO[0] = i2c_scl;
assign GPIO[2] = i2c_sda;

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule
