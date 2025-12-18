// top.sv - IMU 測試模組
// 用於測試 MPU6050 IMU 硬體是否正常工作

module top (
    input  CLOCK_50,
    input  [3:0] KEY,
    input  [17:0] SW,
    output [17:0] LEDR,
    output [8:0] LEDG,
    output [6:0] HEX0,
    output [6:0] HEX1,
    output [6:0] HEX2,
    output [6:0] HEX3,
    output [6:0] HEX4,
    output [6:0] HEX5,
    output [6:0] HEX6,
    output [6:0] HEX7,
    output I2C_SCLK,
    inout  I2C_SDAT
);

    // =========================================================
    // 時鐘和重置
    // =========================================================
    logic clk;
    logic rst_n;
    
    assign clk = CLOCK_50;
    assign rst_n = KEY[0];  // KEY[0] 為重置按鈕（低電平有效）

    // =========================================================
    // I2C Master 實例化
    // =========================================================
    logic i2c_start;
    logic i2c_rw;
    logic [6:0] i2c_dev_addr;
    logic [7:0] i2c_reg_addr;
    logic [7:0] i2c_wr_data;
    logic [7:0] i2c_rd_data;
    logic i2c_busy;
    logic i2c_done;
    logic i2c_ack_error;

    i2c_master #(
        .CLK_HZ(50_000_000),
        .I2C_HZ(100_000)
    ) i2c_master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(i2c_start),
        .rw(i2c_rw),
        .dev_addr(i2c_dev_addr),
        .reg_addr(i2c_reg_addr),
        .wr_data(i2c_wr_data),
        .rd_data(i2c_rd_data),
        .busy(i2c_busy),
        .done(i2c_done),
        .ack_error(i2c_ack_error),
        .scl(I2C_SCLK),
        .sda(I2C_SDAT)
    );

    // =========================================================
    // MPU6050 控制器實例化
    // =========================================================
    logic signed [31:0] v_x, v_y, v_z;
    logic signed [15:0] gyro_x, gyro_y, gyro_z;
    logic data_valid;

    mpu6050_ctrl #(
        .CLK_HZ(50_000_000),
        .I2C_ADDR(7'h68)  // MPU6050 I2C 位址 (AD0=0)
    ) mpu6050_ctrl_inst (
        .clk(clk),
        .rst_n(rst_n),
        .i2c_start(i2c_start),
        .i2c_rw(i2c_rw),
        .i2c_dev_addr(i2c_dev_addr),
        .i2c_reg_addr(i2c_reg_addr),
        .i2c_wr_data(i2c_wr_data),
        .i2c_rd_data(i2c_rd_data),
        .i2c_busy(i2c_busy),
        .i2c_done(i2c_done),
        .i2c_ack_error(i2c_ack_error),
        .v_x(v_x),
        .v_y(v_y),
        .v_z(v_z),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .data_valid(data_valid)
    );

    // =========================================================
    // LED 狀態顯示
    // =========================================================
    assign LEDG[0] = data_valid;        // 資料有效指示
    assign LEDG[1] = i2c_busy;          // I2C 忙碌指示
    assign LEDG[2] = i2c_ack_error;     // I2C 錯誤指示
    assign LEDG[3] = ~rst_n;            // 重置狀態
    assign LEDG[8:4] = 5'b0;

    // LEDR 顯示加速度計的絕對值（取高8位）
    assign LEDR[17:10] = (v_x[31]) ? (~v_x[22:15] + 1) : v_x[22:15];  // X軸加速度
    assign LEDR[9:2] = (v_y[31]) ? (~v_y[22:15] + 1) : v_y[22:15];      // Y軸加速度
    assign LEDR[1:0] = 2'b0;

    // =========================================================
    // 7段顯示器：顯示加速度和陀螺儀數據
    // =========================================================
    // 使用 SW[17:16] 選擇顯示模式：
    // 00: 顯示加速度 X 軸 (v_x)
    // 01: 顯示加速度 Y 軸 (v_y)
    // 10: 顯示加速度 Z 軸 (v_z)
    // 11: 顯示陀螺儀 X 軸 (gyro_x)

    logic signed [31:0] display_value;
    logic [15:0] display_value_abs;
    logic [3:0] sign_digit;
    logic signed [15:0] display_value_16bit;

    always_comb begin
        case (SW[17:16])
            2'b00: display_value = v_x;
            2'b01: display_value = v_y;
            2'b10: display_value = v_z;
            2'b11: display_value = {{16{gyro_x[15]}}, gyro_x};
            default: display_value = v_x;
        endcase
        
        // 取低16位用於顯示
        display_value_16bit = display_value[15:0];
        
        // 取絕對值（用於顯示）
        if (display_value_16bit[15]) begin
            display_value_abs = (~display_value_16bit + 1);
            sign_digit = 4'd10;  // 顯示負號（用 'A' 表示）
        end else begin
            display_value_abs = display_value_16bit;
            sign_digit = 4'd11;  // 顯示正號（用 'b' 表示）
        end
    end

    // 提取十進制數字
    logic [3:0] thousands, hundreds, tens, ones;
    logic [15:0] temp_val;

    always_comb begin
        // 限制在 0-9999 範圍內
        temp_val = (display_value_abs > 9999) ? 16'd9999 : display_value_abs;
        thousands = temp_val / 1000;
        temp_val = temp_val % 1000;
        hundreds = temp_val / 100;
        temp_val = temp_val % 100;
        tens = temp_val / 10;
        ones = temp_val % 10;
    end

    // 7段顯示器解碼器
    HexTo7Seg hex0(.i_hex(ones), .o_seg(HEX0));
    HexTo7Seg hex1(.i_hex(tens), .o_seg(HEX1));
    HexTo7Seg hex2(.i_hex(hundreds), .o_seg(HEX2));
    HexTo7Seg hex3(.i_hex(thousands), .o_seg(HEX3));
    
    // HEX4-7 顯示符號和模式指示
    HexTo7Seg hex4(.i_hex(sign_digit), .o_seg(HEX4));
    
    // HEX5-7 顯示模式（00, 01, 10, 11）
    HexTo7Seg hex5(.i_hex(SW[16] ? 4'd1 : 4'd0), .o_seg(HEX5));
    HexTo7Seg hex6(.i_hex(SW[17] ? 4'd1 : 4'd0), .o_seg(HEX6));
    assign HEX7 = 7'b1111111;  // 空白

endmodule

// =========================================================
// HexTo7Seg 模組：將 4-bit 十六進制數轉換為 7段顯示
// =========================================================
module HexTo7Seg (
    input  [3:0] i_hex,
    output [6:0] o_seg
);

    // 7段顯示器編碼（1=亮，0=暗）
    // 段順序：g f e d c b a
    localparam D0 = 7'b1000000;  // 0
    localparam D1 = 7'b1111001;  // 1
    localparam D2 = 7'b0100100;  // 2
    localparam D3 = 7'b0110000;  // 3
    localparam D4 = 7'b0011001;  // 4
    localparam D5 = 7'b0010010;  // 5
    localparam D6 = 7'b0000010;  // 6
    localparam D7 = 7'b1111000;  // 7
    localparam D8 = 7'b0000000;  // 8
    localparam D9 = 7'b0010000;  // 9
    localparam DA = 7'b0001000;  // A (用於負號)
    localparam DB = 7'b0000011;  // b (用於正號)
    localparam DC = 7'b1000110;  // C
    localparam DD = 7'b0100001;  // d
    localparam DE = 7'b0000110;  // E
    localparam DF = 7'b0001110;  // F

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
            default: o_seg = 7'b1111111;  // 空白
        endcase
    end

endmodule

