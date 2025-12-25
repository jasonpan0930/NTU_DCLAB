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

	// =========================================================
	// 音訊時鐘（從vga_clock PLL獲取12MHz和100kHz）
	// =========================================================
	logic clk_12m;   // 12MHz 音訊主時鐘
	logic clk_100k;  // 100kHz I2C時鐘
	
	// 時鐘從vga_clock PLL模組輸出（已在下面實例化）
	assign AUD_XCK = clk_12m;


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
logic [2:0]  zombie_size_sel_overlay; // Size selector from VGA overlay module (0=0.5x, 1=0.6x, 2=0.7x, 3=0.8x, 4=0.9x, 5=1.0x)
logic [13:0] zombie_addr_trans;       // Address from transparency builder
logic [13:0] zombie_bram_addr;        // Address driven into zombie BRAM
logic [13:0] zombie_bram_05x_addr, zombie_bram_06x_addr, zombie_bram_07x_addr;
logic [13:0] zombie_bram_08x_addr, zombie_bram_09x_addr, zombie_bram_1x_addr;
logic [7:0]  zombie_bram_q;           // Pixel data from zombie BRAM (routed)
logic [7:0]  zombie_bram_05x_q, zombie_bram_06x_q, zombie_bram_07x_q;
logic [7:0]  zombie_bram_08x_q, zombie_bram_09x_q, zombie_bram_1x_q;

// Transparency bounds and control from set_trans_arr
// For each X, [7:0] = min transparent Y, [15:8] = max transparent Y
logic [15:0] zombie_trans_bounds [0:102-1];
logic        trans_done;
logic        trans_busy = 1'b0;  // Initialize to 0 (trans module is disabled)

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
assign trigger_raw = ~GPIO[6];

// 使用 50MHz 主時鐘 clk 與 KEY[1] 作為 active-low reset 進行去彈跳
// 調整 CNT_N 讓按鍵需要穩定一段時間才算一次，避免一次按下被彈跳當成多次
Debounce #(
    .CNT_N(250000) // 約 5ms 在 50MHz 下，可依需要再調整
) trigger_debounce_inst (
    .i_in       (trigger_raw),
    .i_clk      (vga_clock_74_25),
    .i_rst_n    (KEY[1]),
    .o_debounced(trigger_debounced),
    .o_neg      (trigger_negedge),
    .o_pos      (trigger_posedge)   // 按鈕由未按 → 按下 時產生一個 clock 週期的脈波，可當作板機觸發
);

wire kill_en;
wire [4:0] kill_index;

// Start Game Detector - detects when game should start (aim in box + trigger pressed)
logic game_started;  // Output from Start_Game_Detector

Start_Game_Detector #(
    .START_BOX_X_MIN(11'd494), 
    .START_BOX_X_MAX(11'd778),  
    .START_BOX_Y_MIN(10'd571),  
    .START_BOX_Y_MAX(10'd630) 
) start_game_detector (
    .i_clk(vga_clock_74_25),     // Use VGA clock (aim and trigger are in VGA domain)
    .i_rst_n(KEY[1]),
    .i_aim_x(aim_x),              // Aim X position
    .i_aim_y(aim_y),              // Aim Y position
    .i_trigger_posedge(trigger_posedge),  // Trigger button pulse
    .i_key3(KEY[3]),              // KEY[3] for manual reset (active low)
    .o_started(game_started)      // Game started signal (latched until reset)
);

// Zombie kill counter (for HEX6 & HEX7 display)
logic [7:0] zombie_kill_count;  // 0-100 counter for kill count display

// Life counter (for HEX4 & HEX5 display)
logic [7:0] life_count;  // 0-99 life counter, starts at 10
logic zombie_hit;  // Hit signal from zombie generator

// Win/Lose signals
logic win_signal;  // High when kill_count reaches 100
logic lose_signal;  // High when life_count reaches 0

// Random Position Generator - DISABLED FOR TESTING
// Assign all zombie positions to 0 for testing
Random_Position_Generator #(
	.MAX_POSITIONS(20),  // Must match MAX_ZOMBIES in VGA_Image_Overlay_Combined
	.CLOCK_FREQ(32'd74250000)  // 74.25MHz
) zombie_gen(
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.i_game_started(game_started),  // Zombies only move when game is started
	.i_win_signal(win_signal),  // Win signal: hide all zombies
	.i_lose_signal(lose_signal),  // Lose signal: stop zombie movement
	.i_kill_count(zombie_kill_count),  // Kill count for speed scaling
	.o_x(zombie_x),
	.o_y(zombie_y),
	.o_valid(zombie_valid),
	.o_distance(zombie_distance),
	.o_active_count(active_zombie_count),
	.o_hit(zombie_hit),  // Hit signal when zombie arrives at destination

	.i_kill(kill_en),
	.i_kill_index(kill_index)
);


// // VGA output
logic [7:0] vga_r, vga_g, vga_b;



// VGA Clock PLL (50MHz -> 74.25MHz)
vga_clock vga_clk_pll(
	.clk_clk(CLOCK_50),
	.reset_reset_n(KEY[1]),
	.vga_clock_25_clk(),  // Not used
	.vga_clock_74_25_clk(vga_clock_74_25),
	.clock_100k_clk(clk_100k),
	.clock_12m_clk(clk_12m)
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
	.i_aim_x(aim_x),                   // Aim X position
	.i_aim_y(aim_y),                   // Aim Y position
	.i_started(game_started),                // Game started signal (0 = start screen, 1 = game playing)
	.o_zombie_addr(zombie_addr_overlay),     // BRAM address when not busy building transparency
	.o_zombie_size_sel(zombie_size_sel_overlay), // Size selector: 0=0.5x, 1=0.6x, 2=0.7x, 3=0.8x, 4=0.9x, 5=1.0x
	.i_zombie_pixel(zombie_bram_q),          // Shared BRAM pixel data (routed)
	
	.o_bg_sram_addr(vga_sram_addr),
	.o_bg_sram_ce_n(vga_sram_ce_n),
	.io_bg_sram_dq(SRAM_DQ),
	.o_bg_sram_lb_n(vga_sram_lb_n),
	.o_bg_sram_oe_n(vga_sram_oe_n),
	.o_bg_sram_ub_n(vga_sram_ub_n),
	.o_bg_sram_we_n(vga_sram_we_n),
	.o_vga_r(vga_r),
	.o_vga_g(vga_g),
	.o_vga_b(vga_b),

	.i_killing(trigger_posedge),
	.o_kill_idx(kill_index),
	.o_kill_en(kill_en)
);

// Transparency table builder for the shared zombie sprite
set_trans_arr #(
	.ZOMBIE_SIZE_X(102),
	.ZOMBIE_SIZE_Y(149)
) u_set_trans_arr (
	.i_clk(vga_clock_74_25),
	.i_rst_n(KEY[1]),
	.o_trans_bounds(zombie_trans_bounds),
	.o_done(trans_done),
	.o_busy(trans_busy),
	.o_zombie_addr(zombie_addr_trans),
	.i_zombie_pixel(zombie_bram_q)
);

// Simple arbiter for shared zombie BRAM:
// - While trans_busy = 1, set_trans_arr owns the BRAM (always uses 1x).
// - When trans_busy = 0, VGA_Image_Overlay_Combined owns the BRAM.
assign zombie_bram_addr = trans_busy ? zombie_addr_trans : zombie_addr_overlay;

// Route address to appropriate BRAM based on size selection
// For now, trans always uses 1x (can be extended later if needed)
always_comb begin
    if (trans_busy) begin
        // Transparency builder always uses 1x
        zombie_bram_05x_addr = 14'd0;
        zombie_bram_06x_addr = 14'd0;
        zombie_bram_07x_addr = 14'd0;
        zombie_bram_08x_addr = 14'd0;
        zombie_bram_09x_addr = 14'd0;
        zombie_bram_1x_addr = zombie_bram_addr;
    end else begin
        // Route based on size selector
        zombie_bram_05x_addr = (zombie_size_sel_overlay == 3'd0) ? zombie_bram_addr : 14'd0;
        zombie_bram_06x_addr = (zombie_size_sel_overlay == 3'd1) ? zombie_bram_addr : 14'd0;
        zombie_bram_07x_addr = (zombie_size_sel_overlay == 3'd2) ? zombie_bram_addr : 14'd0;
        zombie_bram_08x_addr = (zombie_size_sel_overlay == 3'd3) ? zombie_bram_addr : 14'd0;
        zombie_bram_09x_addr = (zombie_size_sel_overlay == 3'd4) ? zombie_bram_addr : 14'd0;
        zombie_bram_1x_addr = (zombie_size_sel_overlay == 3'd5) ? zombie_bram_addr : 14'd0;
    end
end

// Shared zombie sprite BRAM instances
zombie_05x u_zombie_05x (
	.address(zombie_bram_05x_addr),
	.clock  (vga_clock_74_25),
	.q      (zombie_bram_05x_q)
);

zombie_06x u_zombie_06x (
	.address(zombie_bram_06x_addr),
	.clock  (vga_clock_74_25),
	.q      (zombie_bram_06x_q)
);

zombie_07x u_zombie_07x (
	.address(zombie_bram_07x_addr),
	.clock  (vga_clock_74_25),
	.q      (zombie_bram_07x_q)
);

zombie_08x u_zombie_08x (
	.address(zombie_bram_08x_addr),
	.clock  (vga_clock_74_25),
	.q      (zombie_bram_08x_q)
);

zombie_09x u_zombie_09x (
	.address(zombie_bram_09x_addr),
	.clock  (vga_clock_74_25),
	.q      (zombie_bram_09x_q)
);

zombie_1x u_zombie_1x (
	.address(zombie_bram_1x_addr),
	.clock  (vga_clock_74_25),
	.data   (8'd0),
	.wren   (1'b0),
	.q      (zombie_bram_1x_q)
);

// Route pixel data based on size selection
always_comb begin
    if (trans_busy) begin
        zombie_bram_q = zombie_bram_1x_q;  // Trans always uses 1x
    end else begin
        case (zombie_size_sel_overlay)
            3'd0: zombie_bram_q = zombie_bram_05x_q;
            3'd1: zombie_bram_q = zombie_bram_06x_q;
            3'd2: zombie_bram_q = zombie_bram_07x_q;
            3'd3: zombie_bram_q = zombie_bram_08x_q;
            3'd4: zombie_bram_q = zombie_bram_09x_q;
            3'd5: zombie_bram_q = zombie_bram_1x_q;
            default: zombie_bram_q = zombie_bram_1x_q;
        endcase
    end
end

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
	.i_clk(CLOCK_50),
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

    // LEDG[3] 顯示板機觸發「一次事件」（已改為LEDG[8]以避免與音訊系統衝突）
    // 使用 trigger_posedge：按鈕由未按→按下時產生一個 clock 週期的脈波（長按只算扣下一發）
    // assign LEDG[3] = trigger_posedge;  // 已移除，改用LEDG[8]
    assign LEDG[8] = trigger_posedge;

    // 觸發次數計數器：每次 trigger_posedge +1
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            trigger_count <= 4'd0;
        end else if (trigger_posedge) begin
            trigger_count <= trigger_count + 4'd1; // 0~15 循環
        end
    end

    // 殭屍擊殺計數器：每次 kill_en 為高時 +1 (0-100，限制為三位數)
    logic kill_en_prev;  // 用來檢測 kill_en 的上升緣
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            zombie_kill_count <= 8'd0;
            kill_en_prev <= 1'b0;
        end else begin
            kill_en_prev <= kill_en;
            // 檢測 kill_en 的上升緣
            if (kill_en && !kill_en_prev) begin
                if (zombie_kill_count < 8'd100) begin
                    zombie_kill_count <= zombie_kill_count + 8'd1;  // 0-100
                end else begin
                    zombie_kill_count <= 8'd100;  // 最大顯示 100
                end
            end
        end
    end

    // 生命計數器：從 10 開始，每次 zombie_hit 為高時 -1
    logic zombie_hit_prev;  // 用來檢測 zombie_hit 的上升緣
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            life_count <= 8'd10;  // 初始生命值為 10
            zombie_hit_prev <= 1'b0;
            lose_signal <= 1'b0;
        end else begin
            zombie_hit_prev <= zombie_hit;
            // 檢測 zombie_hit 的上升緣
            if (zombie_hit && !zombie_hit_prev) begin
                if (life_count > 8'd0) begin
                    life_count <= life_count - 8'd1;  // 減少生命值
                end else begin
                    life_count <= 8'd0;  // 最小為 0
                end
            end
            // 檢測輸掉條件：生命值為 0
            if (life_count == 8'd0) begin
                lose_signal <= 1'b1;  // 保持高電平
            end
        end
    end

    // 勝利信號：當擊殺數達到 100 時
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            win_signal <= 1'b0;
        end else begin
            if (zombie_kill_count >= 8'd100) begin
                win_signal <= 1'b1;  // 保持高電平
            end
        end
    end

    // 將擊殺計數轉換為十進制的百位、十位和個位
    logic [3:0] kill_count_hundreds;  // 百位數 (0-1)
    logic [3:0] kill_count_tens;      // 十位數 (0-9)
    logic [3:0] kill_count_ones;       // 個位數 (0-9)
    logic [7:0] temp_kill_count;
    logic [7:0] temp_remainder;
    
    always_comb begin
        temp_kill_count = zombie_kill_count;
        kill_count_hundreds = temp_kill_count / 8'd100;
        temp_remainder = temp_kill_count % 8'd100;
        kill_count_tens = temp_remainder / 8'd10;
        kill_count_ones = temp_remainder % 8'd10;
    end

    // 將生命計數轉換為十進制的十位和個位
    logic [3:0] life_count_tens;   // 十位數 (0-9)
    logic [3:0] life_count_ones;    // 個位數 (0-9)
    logic [7:0] temp_life_count;
    logic [7:0] temp_life_remainder;
    
    always_comb begin
        temp_life_count = life_count;
        temp_life_remainder = temp_life_count % 8'd100;
        life_count_tens = temp_life_remainder / 8'd10;
        life_count_ones = temp_life_remainder % 8'd10;
    end

    // LEDG[7] 顯示遊戲是否已開始 (game_started)
    assign LEDG[7] = game_started;
    
    // LEDG[6] 顯示勝利信號 (win_signal) - 保持高電平
    assign LEDG[6] = win_signal;
    
    // LEDG[5] 顯示失敗信號 (lose_signal) - 保持高電平
    assign LEDG[5] = lose_signal;
    
    // LEDG[4] 顯示音訊播放狀態
    assign LEDG[4] = audio_playing;
    
    // LEDG[3] 顯示I2C初始化完成
    assign LEDG[3] = i2c_init_done;
    
    // LEDR[0] 顯示I2C初始化錯誤
    assign LEDR[0] = i2c_ledr_nack;

	// LEDR 顯示接收到的原始 Byte (除錯用)
    reg [7:0] last_rx_byte;
    always @(posedge clk) if(rx_dv) last_rx_byte <= rx_byte;
	assign LEDR[17:10] = last_rx_byte;
    assign LEDR[9:1] = 9'b0;  // LEDR[0] 保留給I2C錯誤指示
    
    // =========================================================
    // 音訊播放系統
    // =========================================================
    
    // 音訊狀態機
    typedef enum logic [2:0] {
        AUD_IDLE,
        AUD_I2C_INIT,
        AUD_PLAY_BGM,
        AUD_PLAY_KILLED,
        AUD_RESET_ADDR  // 用於重置地址的中間狀態
    } audio_state_t;
    
    audio_state_t audio_state, audio_state_next;
    
    // I2C初始化完成標誌（避免每次按鍵都重新初始化）
    logic i2c_init_done;
    
    // 擊殺信號檢測（用於觸發killed音效）
    logic kill_en_prev_audio;  // 用於檢測kill_en的上升緣
    
    // 按鍵去抖動（KEY2和KEY3）
    logic key2_debounced, key3_debounced;
    logic key2_posedge, key3_posedge;
    
    Debounce #(
        .CNT_N(250000) // ~5ms at 50MHz
    ) key2_debounce (
        .i_in(~KEY[2]),  // KEY[2] is active low
        .i_clk(clk),
        .i_rst_n(rst_n),
        .o_debounced(key2_debounced),
        .o_neg(),
        .o_pos(key2_posedge)
    );
    
    Debounce #(
        .CNT_N(250000) // ~5ms at 50MHz
    ) key3_debounce (
        .i_in(~KEY[3]),  // KEY[3] is active low
        .i_clk(clk),
        .i_rst_n(rst_n),
        .o_debounced(key3_debounced),
        .o_neg(),
        .o_pos(key3_posedge)
    );
    
    // I2C 初始化信號
    logic i2c_start, i2c_finished;
    logic i2c_sdat, i2c_oen;
    logic i2c_ledr_nack;
    
    // 音訊播放控制信號
    logic audio_playing;
    logic [19:0] bram_addr;  // 地址計數器
    logic signed [15:0] bram_data;
    logic signed [15:0] dac_data;
    
    // DACLRCK邊緣檢測
    logic daclrck_prev;
    logic daclrck_posedge;
    
    // BRAM 選擇信號（根據按鍵選擇bgm或killed）
    logic [15:0] bgm_data;
    logic [15:0] killed_data;
    logic [16:0] bgm_addr;  // bgm使用17位地址（117520個樣本）
    logic [15:0] killed_addr;
    
    // 根據當前狀態選擇BRAM地址和數據
    // KEY2按下時讀取bgm BRAM，KEY3按下時讀取killed BRAM
    always_comb begin
        if (audio_state == AUD_PLAY_BGM) begin
            // 播放BGM時，從bgm BRAM讀取，地址從0開始
            bgm_addr = bram_addr[16:0];  // 使用低17位（bgm使用17位地址）
            killed_addr = 16'd0;  // killed BRAM不使用
            bram_data = bgm_data;  // 使用bgm BRAM的數據
        end else if (audio_state == AUD_PLAY_KILLED) begin
            // 播放KILLED時，從killed BRAM讀取，地址從0開始
            bgm_addr = 17'd0;  // bgm BRAM不使用
            killed_addr = bram_addr[15:0];  // 使用低16位（killed使用16位地址）
            bram_data = killed_data;  // 使用killed BRAM的數據
        end else begin
            // 其他狀態時，兩個BRAM都不使用
            bgm_addr = 17'd0;
            killed_addr = 16'd0;
            bram_data = 16'd0;
        end
    end
    
    // 擊殺信號檢測（用於觸發killed音效）
    // 直接在74.25MHz的vga_clock_74_25域檢測kill_en的上升緣
    logic kill_en_prev_vga;
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            kill_en_prev_vga <= 1'b0;
        end else begin
            kill_en_prev_vga <= kill_en;
        end
    end
    
    // 在VGA時鐘域中檢測上升緣
    logic kill_en_posedge_vga;
    assign kill_en_posedge_vga = kill_en && !kill_en_prev_vga;
    
    // 產生持續兩個cycle的脈衝
    logic kill_en_posedge_pulse;
    logic [1:0] kill_en_pulse_counter;
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            kill_en_posedge_pulse <= 1'b0;
            kill_en_pulse_counter <= 2'd0;
        end else begin
            if (kill_en_posedge_vga) begin
                kill_en_posedge_pulse <= 1'b1;
                kill_en_pulse_counter <= 2'd2;  // 持續兩個cycle
            end else if (kill_en_pulse_counter > 2'd0) begin
                // counter遞減，當遞減後的counter > 0時保持脈衝為1
                logic [1:0] next_counter;
                next_counter = kill_en_pulse_counter - 2'd1;
                kill_en_pulse_counter <= next_counter;
                kill_en_posedge_pulse <= (next_counter > 2'd0);
            end else begin
                kill_en_posedge_pulse <= 1'b0;
            end
        end
    end
    
    // 將kill_en_posedge_pulse同步到clk域（用於狀態機控制）
    logic kill_en_posedge_pulse_sync1, kill_en_posedge_pulse_sync2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kill_en_posedge_pulse_sync1 <= 1'b0;
            kill_en_posedge_pulse_sync2 <= 1'b0;
        end else begin
            kill_en_posedge_pulse_sync1 <= kill_en_posedge_pulse;
            kill_en_posedge_pulse_sync2 <= kill_en_posedge_pulse_sync1;
        end
    end
    
    // 檢測同步後的上升緣（用於狀態機）
    logic kill_en_posedge;
    assign kill_en_posedge = kill_en_posedge_pulse_sync2;
    
    // I2C初始化完成標誌
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_init_done <= 1'b0;
        end else begin
            if (i2c_finished && (audio_state == AUD_I2C_INIT)) begin
                i2c_init_done <= 1'b1;
            end
        end
    end
    
    // 音訊狀態機
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            audio_state <= AUD_IDLE;
        end else begin
            audio_state <= audio_state_next;
        end
    end
    
    // 狀態機組合邏輯
    always_comb begin
        audio_state_next = audio_state;
        i2c_start = 1'b0;
        audio_playing = 1'b0;
        
        case (audio_state)
            AUD_IDLE: begin
                // 系統啟動後自動開始播放BGM
                if (i2c_init_done) begin
                    audio_state_next = AUD_PLAY_BGM;  // 直接開始播放BGM
                end else begin
                    audio_state_next = AUD_I2C_INIT;
                end
            end
            
            AUD_I2C_INIT: begin
                // 保持i2c_start為高直到初始化完成
                i2c_start = 1'b1;
                if (i2c_finished) begin
                    // I2C初始化完成，自動開始播放BGM
                    audio_state_next = AUD_PLAY_BGM;
                end
            end
            
            AUD_PLAY_BGM: begin
                audio_playing = 1'b1;
                // 擊殺發生時，切換到播放killed音效（打斷BGM）
                if (kill_en_posedge) begin
                    audio_state_next = AUD_PLAY_KILLED;  // 切換到播放killed
                end
                // BGM播放完成後循環播放（重新開始）
                else if (bram_addr >= 20'd117519) begin  // bgm有117520個樣本（0-117519）
                    audio_state_next = AUD_PLAY_BGM;  // 循環播放，地址會重置
                end
            end
            
            AUD_PLAY_KILLED: begin
                audio_playing = 1'b1;
                // 如果播放killed時再次擊殺，重置地址重新播放killed
                if (kill_en_posedge) begin
                    audio_state_next = AUD_PLAY_KILLED;  // 保持KILLED狀態，但會觸發地址重置
                end
                // killed播放完成後，恢復播放BGM
                else if (bram_addr >= 20'd41519) begin  // killed有41520個樣本（0-41519）
                    audio_state_next = AUD_PLAY_BGM;  // 恢復播放BGM，地址會重置
                end
            end
            
            AUD_RESET_ADDR: begin
                // 這個狀態不再需要，直接轉換到播放狀態
                audio_state_next = AUD_PLAY_BGM;
            end
            
            default: audio_state_next = AUD_IDLE;
        endcase
    end
    
    // I2C 初始化器
    I2cInitializer i2c_init (
        .i_rst_n(rst_n),
        .i_clk(clk_100k),
        .i_start(i2c_start),
        .i_sdat(I2C_SDAT),
        .o_finished(i2c_finished),
        .o_sclk(I2C_SCLK),
        .o_sdat(i2c_sdat),
        .o_oen(i2c_oen),
        .o_ledr(i2c_ledr_nack)
    );
    
    // I2C SDA 開漏輸出
    assign I2C_SDAT = (i2c_oen) ? i2c_sdat : 1'bz;
    
    // =========================================================
    // 簡單音訊播放邏輯（直接在DE2_115中實現）
    // =========================================================
    
    // DACLRCK邊緣檢測（用於遞增地址）
    always_ff @(posedge AUD_BCLK or negedge rst_n) begin
        if (!rst_n) begin
            daclrck_prev <= 1'b0;
        end else begin
            daclrck_prev <= AUD_DACLRCK;
        end
    end
    
    assign daclrck_posedge = AUD_DACLRCK && !daclrck_prev;
    
    // 地址計數器（根據DACLRCK上升緣遞增）
    // 記錄上一個狀態，用於檢測狀態切換
    audio_state_t audio_state_prev;
    // 將kill_en_posedge同步到AUD_BCLK時鐘域（用於地址重置）
    logic kill_en_posedge_sync_bclk1, kill_en_posedge_sync_bclk2;
    always_ff @(posedge AUD_BCLK or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr <= 20'd0;
            audio_state_prev <= AUD_IDLE;
            kill_en_posedge_sync_bclk1 <= 1'b0;
            kill_en_posedge_sync_bclk2 <= 1'b0;
        end else begin
            audio_state_prev <= audio_state;
            // 同步kill_en_posedge到AUD_BCLK時鐘域
            kill_en_posedge_sync_bclk1 <= kill_en_posedge;
            kill_en_posedge_sync_bclk2 <= kill_en_posedge_sync_bclk1;
            
            // 狀態切換時重置地址
            if (audio_state != audio_state_prev) begin
                bram_addr <= 20'd0;
            end
            // 在AUD_PLAY_KILLED狀態時，如果kill_en_posedge為true，重置地址
            else if (audio_state == AUD_PLAY_KILLED && kill_en_posedge_sync_bclk2) begin
                bram_addr <= 20'd0;  // 重置地址，重新播放killed
            end
            // 正常播放時遞增地址
            else if (audio_playing && daclrck_posedge) begin
                // 根據當前狀態決定最大地址
                if (audio_state == AUD_PLAY_BGM) begin
                    if (bram_addr >= 20'd117519) begin
                        bram_addr <= 20'd0;  // BGM循環播放
                    end else begin
                        bram_addr <= bram_addr + 20'd1;
                    end
                end else if (audio_state == AUD_PLAY_KILLED) begin
                    if (bram_addr >= 20'd41519) begin
                        bram_addr <= 20'd0;  // Killed播放完成，會切換回BGM
                    end else begin
                        bram_addr <= bram_addr + 20'd1;
                    end
                end
            end
        end
    end
    
    // 直接將BRAM數據輸出到DAC（簡單播放，無特殊處理）
    assign dac_data = bram_data;
    
    // 音訊播放器
    AudPlayer audio_player (
        .i_rst_n(rst_n),
        .i_bclk(AUD_BCLK),
        .i_daclrck(AUD_DACLRCK),
        .i_en(audio_playing),
        .i_dac_data(dac_data),
        .o_aud_dacdat(AUD_DACDAT)
    );
    
    // BGM BRAM 實例化
    bgm bgm_rom (
        .address(bgm_addr),
        .clock(clk_12m),
        .q(bgm_data)
    );
    
    // Killed BRAM 實例化
    killed killed_rom (
        .address(killed_addr),
        .clock(clk_12m),
        .q(killed_data)
    );

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


	// 7段顯示器解碼
	HexTo7Seg hex0(.i_hex(ones),      .o_seg(HEX0));
	HexTo7Seg hex1(.i_hex(tens),      .o_seg(HEX1));
	HexTo7Seg hex2(.i_hex(hundreds),  .o_seg(HEX2));
	HexTo7Seg hex3(.i_hex(thousands), .o_seg(HEX3));
    
    // HEX4, HEX5 display life count (0-99, starts at 10)
    HexTo7Seg hex4(.i_hex(life_count_ones), .o_seg(HEX4));   // 顯示生命值的個位數 (0-9)
    HexTo7Seg hex5(.i_hex(life_count_tens), .o_seg(HEX5));   // 顯示生命值的十位數 (0-9)
    
    // HEX6, HEX7 display zombie kill count (0-100)
    HexTo7Seg hex6(.i_hex(kill_count_ones), .o_seg(HEX6));      // 顯示擊殺數量的個位數 (0-9)
    HexTo7Seg hex7(.i_hex(kill_count_tens), .o_seg(HEX7));      // 顯示擊殺數量的十位數 (0-9)

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule


