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

	// =========================================================
	// 時鐘和重置
	// =========================================================
	logic clk;
	logic rst_n;
	
	assign clk = CLOCK_50;
	assign rst_n = KEY[1];  // KEY[0] 為重置按鈕（低電平有效）


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
logic [10:0] zombie_x [0:19];  // Array of X positions (MAX_ZOMBIES = 20)
logic [9:0] zombie_y [0:19];   // Array of Y positions
logic zombie_valid [0:19];     // Array of valid flags
logic [31:0] zombie_distance [0:19];  // Distance to target for each zombie
logic [3:0] active_zombie_count;     // Number of active zombies

// Zombie size
logic [10:0] zombie_size_x;
logic [9:0] zombie_size_y;

// Initialize zombie size
assign zombie_size_x = 11'd102;  // Zombie width
assign zombie_size_y = 10'd149;   // Zombie height

// Shared zombie sprite BRAM arbitration signals
logic [13:0] zombie_addr_overlay;     // Address from VGA overlay module
logic [13:0] zombie_addr_trans;       // Address from transparency builder
logic [13:0] zombie_bram_addr;        // Address driven into zombie_1x
logic [7:0]  zombie_bram_q;           // Pixel data from zombie_1x

// Transparency bounds and control from set_trans_arr
// For each X, [7:0] = min transparent Y, [15:8] = max transparent Y
logic [15:0] zombie_trans_bounds [0:102-1];
logic        trans_done;
logic        trans_busy;

// Aim position (controlled by IMU / position controller)
logic [10:0] aim_x;
logic [9:0] aim_y;

// Transparency bounds display index counter
logic [6:0] trans_bounds_index;  // Index 0-101 for zombie_trans_bounds array
logic key0_debounced;
logic key0_posedge;  // One-clock pulse when KEY[0] is pressed

// Debounce KEY[0] for transparency bounds index increment
Debounce #(
    .CNT_N(250000) // ~5ms at 50MHz
) key0_debounce_inst (
    .i_in       (~KEY[0]),  // KEY[0] is active low
    .i_clk      (clk),
    .i_rst_n    (KEY[1]),
    .o_debounced(key0_debounced),
    .o_neg      (),
    .o_pos      (key0_posedge)
);



// Trigger (板機) input from external button on GPIO[4] with debouncing
logic trigger_raw;           // raw GPIO input
logic trigger_debounced;     // stable level after debounce
logic trigger_posedge;       // one-clock pulse when button is pressed (rising edge)
logic trigger_negedge;       // one-clock pulse when button is released (falling edge)

// Trigger press counter (counts how many times the trigger is pulled)
logic [3:0] trigger_count;   // 0~15, will be shown on HEX4

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

// =========================================================
// Debounce for external trigger button on GPIO[4]
// 5002A 按鈕接到 GPIO[4]，按下時視為「扣下板機」
// =========================================================
// 將 GPIO[4] 作為輸入使用
// 若 5002A 這類按鈕模組為「按下輸出變低 (active-low)」，在這裡取反，讓按下時 trigger_raw = 1
assign trigger_raw = ~GPIO[4];

// 使用 50MHz 主時鐘 clk 與 KEY[1] 作為 active-low reset 進行去彈跳
// 調整 CNT_N 讓按鍵需要穩定一段時間才算一次，避免一次按下被彈跳當成多次
Debounce #(
    .CNT_N(250000) // 約 5ms 在 50MHz 下，可依需要再調整
) trigger_debounce_inst (
    .i_in       (trigger_raw),
    .i_clk      (clk),
    .i_rst_n    (KEY[1]),
    .o_debounced(trigger_debounced),
    .o_neg      (trigger_negedge),
    .o_pos      (trigger_posedge)   // 按鈕由未按 → 按下 時產生一個 clock 週期的脈波，可當作板機觸發
);



// Random Position Generator - DISABLED FOR TESTING
// Assign all zombie positions to 0 for testing
Random_Position_Generator #(
	.MAX_POSITIONS(20),  // Must match MAX_ZOMBIES in VGA_Image_Overlay_Combined
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
	.MAX_ZOMBIES(20)  // Must match MAX_POSITIONS in Random_Position_Generator
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
	.o_zombie_addr(zombie_addr_overlay),     // BRAM address when not busy building transparency
	.i_zombie_pixel(zombie_bram_q),          // Shared BRAM pixel data
	.i_trans_bounds(zombie_trans_bounds),    // Transparency bounds from set_trans_arr
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

// Transparency table builder for the shared zombie sprite
// set_trans_arr #(
// 	.ZOMBIE_SIZE_X(102),
// 	.ZOMBIE_SIZE_Y(149)
// ) u_set_trans_arr (
// 	.i_clk(vga_clock_74_25),
// 	.i_rst_n(KEY[1]),
// 	.o_trans_bounds(zombie_trans_bounds),
// 	.o_done(trans_done),
// 	.o_busy(trans_busy),
// 	.o_zombie_addr(zombie_addr_trans),
// 	.i_zombie_pixel(zombie_bram_q)
// );

// Simple arbiter for shared zombie_1x BRAM:
// - While trans_busy = 1, set_trans_arr owns the BRAM.
// - When trans_busy = 0, VGA_Image_Overlay_Combined owns the BRAM.
assign zombie_bram_addr = trans_busy ? zombie_addr_trans : zombie_addr_overlay;

// Shared zombie sprite BRAM instance
zombie_1x u_zombie_1x (
	.address(zombie_bram_addr),
	.clock  (vga_clock_74_25),
	.data   (8'd0),
	.wren   (1'b0),
	.q      (zombie_bram_q)
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


// Position Controller module instantiation (for IMU-based aim control)
// Intermediate wires for shifted signals (preserve 32-bit signed width)
wire signed [31:0] gx_cal_shifted = gx_cal >>> 1;  // Signed arithmetic shift (preserves sign)
wire signed [31:0] gy_cal_shifted = gy_cal >>> 1;
wire signed [31:0] gz_cal_shifted = gz_cal >>> 1;
wire signed [31:0] vx_shifted = vx >>> 1;
wire signed [31:0] vy_shifted = vy >>> 1;
wire signed [31:0] vz_shifted = vz >>> 1;

Position_Controller #(
	.SCREEN_WIDTH(1280),
	.SCREEN_HEIGHT(720),
	.KX(-32'd1),          // Horizontal sensitivity (adjustable)
	.KY(-32'd1),          // Vertical sensitivity (adjustable)
	.UPDATE_DIVIDER(24'd742500), // ~1000Hz update rate at 74.25MHz
	.USE_LINEAR_VELOCITY(0)    // Disable linear velocity for now
) position_controller_inst(
	.i_clk(vga_clock_74_25),   // Use VGA clock (74.25MHz)
	.i_rst_n(KEY[1]),
	.i_wx(gx_cal_shifted),    // Angular velocity X (pitch) → controls Y (synchronized, fast divide by 2)
	.i_wy(gy_cal_shifted),    // Angular velocity Y (roll) → not used (synchronized, fast divide by 2)
	.i_wz(gz_cal_shifted),    // Angular velocity Z (yaw) → controls X (synchronized, fast divide by 2)
	.i_vy(vy_shifted),              // Linear velocity Y (not used, fast divide by 2)
	.i_vx(vx_shifted),              // Linear velocity X (not used, fast divide by 2)
	.i_vz(vz_shifted),              // Linear velocity Z (not used, fast divide by 2)
	.o_x(aim_x),               // Output X position to aim_x
	.o_y(aim_y)                // Output Y position to aim_y
);

	// =========================================================
	// UART Receiver 實例化
	// =========================================================
    wire rx_dv;
    wire [7:0] rx_byte;
    wire rx_input_signal;
    
    // 將 GPIO[0] 用作 UART RX 輸入
    // 請將 STM32 TX 接到 GPIO[0]
    assign rx_input_signal = GPIO[0];
    // 如果使用 DB9 接口，請改用: assign rx_input_signal = UART_RXD;

    uart_rx #(
        .CLKS_PER_BIT(434) // 50MHz / 115200 = 434
    ) u_rx (
        .i_Clock(clk),
        .i_Rx_Serial(rx_input_signal),
        .o_Rx_DV(rx_dv),
        .o_Rx_Byte(rx_byte)
    );

	// =========================================================
	// IMU Parser 實例化
	// =========================================================
    wire signed [31:0] ax, ay, az, gx, gy, gz;
    wire parser_data_ready;

    imu_parser u_parser (
        .clk(clk),
        .reset_n(rst_n),
        .rx_dv(rx_dv),
        .rx_byte(rx_byte),
        .ax(ax),
        .ay(ay),
        .az(az),
        .gx(gx),
        .gy(gy),
        .gz(gz),
        .data_ready(parser_data_ready)
    );

	// =========================================================
	// Velocity Integrator 實例化
	// =========================================================
    wire signed [31:0] vx, vy, vz;
    wire signed [31:0] gx_cal, gy_cal, gz_cal;
    wire calibrated;

    velocity_integrator u_integrator (
        .clk(clk),
        .reset_n(rst_n),
        .en(parser_data_ready), // Update when new data arrives
        .ax(ax),
        .ay(ay),
        .az(az),
        .gx(gx),
        .gy(gy),
        .gz(gz),
        .vx(vx),
        .vy(vy),
        .vz(vz),
        .gx_out(gx_cal),
        .gy_out(gy_cal),
        .gz_out(gz_cal),
        .calibrated(calibrated)
    );

	// =========================================================
	// LED 狀態顯示
	// =========================================================
	assign LEDG[0] = parser_data_ready; // 收到完整封包時閃爍
	
    // 讓 LEDG[1] 隨著每次收到 Byte 閃爍
    reg byte_led;
    always @(posedge clk) if(rx_dv) byte_led <= ~byte_led;
    assign LEDG[1] = byte_led;
    
    // LEDG[2] 顯示 Calibration 是否完成
    assign LEDG[2] = calibrated;

    // LEDG[3] 顯示板機觸發「一次事件」
    // 使用 trigger_posedge：按鈕由未按→按下時產生一個 clock 週期的脈波（長按只算扣下一發）
    assign LEDG[3] = trigger_posedge;

    // 觸發次數計數器：每次 trigger_posedge +1
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            trigger_count <= 4'd0;
        end else if (trigger_posedge) begin
            trigger_count <= trigger_count + 4'd1; // 0~15 循環
        end
    end

    // 其餘高位 LEDG[8:4] 先關閉
	assign LEDG[8:4] = 5'b0;

	// LEDR 顯示接收到的原始 Byte (除錯用)
    reg [7:0] last_rx_byte;
    always @(posedge clk) if(rx_dv) last_rx_byte <= rx_byte;
	assign LEDR[17:10] = last_rx_byte;
    assign LEDR[9:0] = 10'b0;

	// =========================================================
	// 7段顯示器：顯示 IMU 數據
	// =========================================================
	// 使用 SW[2:0] 選擇顯示內容：
	// 000: VX (Velocity X)
	// 001: VY (Velocity Y)
	// 010: VZ (Velocity Z)
	// 011: GX
	// 100: GY
	// 101: GZ

	logic signed [31:0] display_value_32bit;
	logic [31:0] display_value_abs;
	logic is_negative;

	always_comb begin
		case (SW[2:0])
			3'b000: display_value_32bit = vx;
			3'b001: display_value_32bit = vy;
			3'b010: display_value_32bit = vz;
			3'b011: display_value_32bit = gx_cal;
			3'b100: display_value_32bit = gy_cal;
			3'b101: display_value_32bit = gz_cal;
			default: display_value_32bit = vx;
		endcase
		
		if (display_value_32bit[31]) begin
			display_value_abs = (~display_value_32bit + 1);
			is_negative = 1'b1;
		end else begin
			display_value_abs = display_value_32bit;
			is_negative = 1'b0;
		end
	end

	// 提取十進制數字 (僅顯示低4位十進制，範圍 0-9999)
	logic [3:0] thousands, hundreds, tens, ones;
	logic [15:0] temp_val;

	always_comb begin
		// 限制顯示範圍 9999
		temp_val = (display_value_abs > 9999) ? 16'd9999 : display_value_abs[15:0];
		
		thousands = temp_val / 1000;
		temp_val = temp_val % 1000;
		
		hundreds = temp_val / 100;
		temp_val = temp_val % 100;
		
		tens = temp_val / 10;
		ones = temp_val % 10;
	end


	// test for transparency bounds display
// Increment trans_bounds_index when KEY[0] is pressed
always_ff @(posedge clk or negedge KEY[1]) begin
    if (!KEY[1]) begin
        trans_bounds_index <= 7'd0;
    end else if (key0_posedge) begin
        if (trans_bounds_index >= 7'd101) begin
            trans_bounds_index <= 7'd0;  // Wrap around
        end else begin
            trans_bounds_index <= trans_bounds_index + 7'd1;
        end
    end
end
	// Transparency bounds display signals
	logic [7:0] trans_bounds_display_value;
	logic [3:0] trans_bounds_ones, trans_bounds_tens;
	logic [3:0] index_ones, index_tens;
	
	// Select which byte to display based on SW[10]
	assign trans_bounds_display_value = SW[10] ? zombie_trans_bounds[trans_bounds_index][15:8] 
	                                             : zombie_trans_bounds[trans_bounds_index][7:0];
	
	// Extract decimal digits for transparency bounds value (0-255)
	always_comb begin
		trans_bounds_ones = trans_bounds_display_value % 10;
		trans_bounds_tens = (trans_bounds_display_value / 10) % 10;
	end
	
	// Extract decimal digits for index (0-101)
	always_comb begin
		index_ones = trans_bounds_index % 10;
		index_tens = (trans_bounds_index / 10) % 10;
	end

	// 7段顯示器解碼
	HexTo7Seg hex0(.i_hex(ones),      .o_seg(HEX0));
	HexTo7Seg hex1(.i_hex(tens),      .o_seg(HEX1));
	HexTo7Seg hex2(.i_hex(hundreds),  .o_seg(HEX2));
	HexTo7Seg hex3(.i_hex(thousands), .o_seg(HEX3));
	
    // Display transparency bounds:
    // HEX[7:6] = index i (0-101)
    // HEX[5:4] = transparency bounds value (controlled by SW[10]: 0=low byte [7:0], 1=high byte [15:8])
    HexTo7Seg hex6_index(.i_hex(index_ones), .o_seg(HEX6));
    HexTo7Seg hex7_index(.i_hex(index_tens), .o_seg(HEX7));
    HexTo7Seg hex4_trans(.i_hex(trans_bounds_ones), .o_seg(HEX4));
    HexTo7Seg hex5_trans(.i_hex(trans_bounds_tens), .o_seg(HEX5));
    
    // Original HEX4-HEX5 displays replaced by transparency bounds display above
    // assign HEX4 = is_negative ? 7'b0111111 : 7'b1111111; // '-' or OFF
    // HexTo7Seg hex5(.i_hex(trigger_count), .o_seg(HEX5));

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule


