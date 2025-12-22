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
	assign rst_n = KEY[0];  // KEY[0] 為重置按鈕（低電平有效）

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
    
	assign LEDG[8:3] = 6'b0;

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

	// 7段顯示器解碼
	HexTo7Seg hex0(.i_hex(ones),      .o_seg(HEX0));
	HexTo7Seg hex1(.i_hex(tens),      .o_seg(HEX1));
	HexTo7Seg hex2(.i_hex(hundreds),  .o_seg(HEX2));
	HexTo7Seg hex3(.i_hex(thousands), .o_seg(HEX3));
	
    // 負號顯示在 HEX4
    assign HEX4 = is_negative ? 7'b0111111 : 7'b1111111; // '-' or OFF
	
    // 其他 HEX 關閉
	assign HEX5 = 7'b1111111;
	assign HEX6 = 7'b1111111;
	assign HEX7 = 7'b1111111;

	// =========================================================
	// 未使用的輸出設定為安全預設值
	// =========================================================
	assign VGA_R = 8'h00;
	assign VGA_G = 8'h00;
	assign VGA_B = 8'h00;
	assign VGA_HS = 1'b0;
	assign VGA_VS = 1'b0;
	assign VGA_CLK = 1'b0;
	assign VGA_BLANK_N = 1'b0;
	assign VGA_SYNC_N = 1'b0;

	assign SRAM_ADDR = 20'h00000;
	assign SRAM_CE_N = 1'b1;
	assign SRAM_LB_N = 1'b1;
	assign SRAM_OE_N = 1'b1;
	assign SRAM_UB_N = 1'b1;
	assign SRAM_WE_N = 1'b1;
	assign SRAM_DQ = 16'hZZZZ;

	assign SMA_CLKOUT = 1'b0;
	assign LCD_BLON = 1'b0;
	assign LCD_ON = 1'b0;
	assign LCD_EN = 1'b0;
	assign LCD_RS = 1'b0;
	assign LCD_RW = 1'b0;
	assign LCD_DATA = 8'hZZ;
	assign UART_CTS = 1'b0;
	assign UART_TXD = 1'b1;
	assign SD_CLK = 1'b0;
	assign EEP_I2C_SCLK = 1'b0;
	assign I2C_SCLK = 1'b0;
	assign ENET0_RST_N = 1'b0;
	assign ENET0_GTX_CLK = 1'b0;
	assign ENET0_MDC = 1'b0;
	assign ENET0_TX_DATA = 4'h0;
	assign ENET0_TX_EN = 1'b0;
	assign ENET0_TX_ER = 1'b0;
	assign ENET1_RST_N = 1'b0;
	assign ENET1_GTX_CLK = 1'b0;
	assign ENET1_MDC = 1'b0;
	assign ENET1_TX_DATA = 4'h0;
	assign ENET1_TX_EN = 1'b0;
	assign ENET1_TX_ER = 1'b0;
	assign TD_RESET_N = 1'b0;
	assign OTG_RST_N = 1'b0;
	assign OTG_CS_N = 1'b1;
	assign OTG_WR_N = 1'b1;
	assign OTG_RD_N = 1'b1;
	assign OTG_ADDR = 2'h0;
	assign OTG_DATA = 16'hZZZZ;
	assign DRAM_ADDR = 13'h0000;
	assign DRAM_BA = 2'h0;
	assign DRAM_CAS_N = 1'b1;
	assign DRAM_CKE = 1'b0;
	assign DRAM_CLK = 1'b0;
	assign DRAM_CS_N = 1'b1;
	assign DRAM_DQM = 4'h0;
	assign DRAM_RAS_N = 1'b1;
	assign DRAM_WE_N = 1'b1;
	assign DRAM_DQ = 32'hZZZZZZZZ;
	assign FL_ADDR = 23'h000000;
	assign FL_CE_N = 1'b1;
	assign FL_OE_N = 1'b1;
	assign FL_RST_N = 1'b0;
	assign FL_WE_N = 1'b1;
	assign FL_WP_N = 1'b0;
	assign FL_DQ = 8'hZZ;
	assign HSMC_CLKOUT_P1 = 1'b0;
	assign HSMC_CLKOUT_P2 = 1'b0;
	assign HSMC_CLKOUT0 = 1'b0;
	assign HSMC_TX_D_P = 17'h00000;

`ifdef DUT_LAB1
	initial begin
		$fsdbDumpfile("LAB1.fsdb");
		$fsdbDumpvars(0, DE2_115, "+mda");
	end
`endif

endmodule

// =========================================================
// HexTo7Seg 模組
// =========================================================
module HexTo7Seg (
	input  [3:0] i_hex,
	output [6:0] o_seg
);
	localparam D0 = 7'b1000000;
	localparam D1 = 7'b1111001;
	localparam D2 = 7'b0100100;
	localparam D3 = 7'b0110000;
	localparam D4 = 7'b0011001;
	localparam D5 = 7'b0010010;
	localparam D6 = 7'b0000010;
	localparam D7 = 7'b1111000;
	localparam D8 = 7'b0000000;
	localparam D9 = 7'b0010000;
	localparam DA = 7'b0001000;
	localparam DB = 7'b0000011;
	localparam DC = 7'b1000110;
	localparam DD = 7'b0100001;
	localparam DE = 7'b0000110;
	localparam DF = 7'b0001110;

	always_comb begin
		case (i_hex)
			4'h0: o_seg = D0;
			4'h1: o_seg = D1;
			4'h2: o_seg = D2;
			4'h3: o_seg = D3;
			4'h4: o_seg = D4;
			4'h5: o_seg = D5;
			4'h6: o_seg = D6;
			4'h7: o_seg = D7;
			4'h8: o_seg = D8;
			4'h9: o_seg = D9;
			4'ha: o_seg = DA;
			4'hb: o_seg = DB;
			4'hc: o_seg = DC;
			4'hd: o_seg = DD;
			4'he: o_seg = DE;
			4'hf: o_seg = DF;
			default: o_seg = 7'b1111111;
		endcase
	end
endmodule
