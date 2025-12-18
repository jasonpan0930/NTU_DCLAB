// IMU_ctrl.sv
// 專門搭配 i2c_master.sv 使用
// 週期性讀取 MPU6050 的 6軸資料 (acc_x/y/z, gyro_x/y/z)

module mpu6050_ctrl #(
    parameter CLK_HZ     = 50_000_000,
    parameter I2C_ADDR   = 7'h68          // AD0=0 時 MPU6050 I2C位址
)(
    input  logic       clk,
    input  logic       rst_n,

    // 連到 i2c_master 的控制介面
    output logic       i2c_start,
    output logic       i2c_rw,           // 0=write, 1=read
    output logic [6:0] i2c_dev_addr,
    output logic [7:0] i2c_reg_addr,
    output logic [7:0] i2c_wr_data,
    input  logic [7:0] i2c_rd_data,
    input  logic       i2c_busy,
    input  logic       i2c_done,
    input  logic       i2c_ack_error,

    // 給外部(遊戲邏輯/VGA)使用的感測資料
    output logic signed [31:0] v_x,
    output logic signed [31:0] v_y,
    output logic signed [31:0] v_z,
    output logic signed [15:0] gyro_x,
    output logic signed [15:0] gyro_y,
    output logic signed [15:0] gyro_z,
    output logic              data_valid,   // 每次成功更新一整組資料時拉高1拍
    output logic [4:0]        fsm_state    // 將內部狀態輸出，方便在外部顯示/除錯
);

    // =========================================================
    //  MPU6050 重要暫存器位址 (只列這次會用到的)
    // =========================================================
    localparam [7:0]
        REG_PWR_MGMT_1   = 8'h6B,   // Power Management 1
        REG_SMPLRT_DIV   = 8'h19,   // Sample Rate Divider
        REG_CONFIG       = 8'h1A,   // DLPF config
        REG_GYRO_CONFIG  = 8'h1B,   // Gyro full-scale
        REG_ACCEL_CONFIG = 8'h1C,   // Accel full-scale
        REG_INT_ENABLE   = 8'h38,   // Interrupt Enable
        REG_ACCEL_XOUT_H = 8'h3B;   // 從這裡開始連續 14 bytes

    // =========================================================
    //  簡單的取樣頻率分頻 (控制多久讀一次資料)
    // =========================================================
    // 降低取樣頻率以便觀察 FSM 運作（調試用）
    localparam int SAMPLE_HZ = 1000;                 // 1次/秒 => 1000ms（調試用，正常可設為 200）
    localparam int SAMPLE_CNT_MAX = CLK_HZ / SAMPLE_HZ;

    logic [$clog2(SAMPLE_CNT_MAX)-1:0] sample_cnt;
    logic                              sample_tick;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_cnt  <= '0;
            sample_tick <= 1'b0;
        end else begin
            if (sample_cnt == SAMPLE_CNT_MAX-1) begin
                sample_cnt  <= '0;
                sample_tick <= 1'b1;
            end else begin
                sample_cnt  <= sample_cnt + 1;
                sample_tick <= 1'b0;
            end
        end
    end

    // =========================================================
    //  狀態機控制 I2C 流程
    // =========================================================

    typedef enum logic [4:0] {
        ST_RESET      = 5'd0,
        ST_INIT_0     = 5'd1,
        ST_INIT_1     = 5'd2,
        ST_INIT_2     = 5'd3,
        ST_INIT_3     = 5'd4,
        ST_INIT_4     = 5'd5,
        ST_IDLE       = 5'd6,
        ST_READ_BYTE  = 5'd7,
        ST_DONE_FRAME = 5'd8,
        ST_ERROR      = 5'd9
    } state_t;

    state_t state;

    // 用來控制「目前在做第幾次I2C動作」
    logic [3:0] byte_index_reg;    // 0..13  共14 bytes（內部使用）
    logic [7:0] data_buf [0:13];    // 暫存加速度/溫度/陀螺資料

    //加速度暫存
    logic signed [15:0] acc_x;
    logic signed [15:0] acc_y;
    logic signed [15:0] acc_z;

    // I2C 輸出暫存
    always_comb begin
        // 預設值
        i2c_start    = 1'b0;
        i2c_rw       = 1'b0;
        i2c_dev_addr = I2C_ADDR;
        i2c_reg_addr = 8'h00;
        i2c_wr_data  = 8'h00;


        case (state)
            // -------------------------------------------------
            // 初始化流程：寫幾個重要暫存器
            ST_INIT_0: begin
                // 解除睡眠：PWR_MGMT_1=0x00
                i2c_rw       = 1'b0;         // write
                i2c_reg_addr = REG_PWR_MGMT_1;
                i2c_wr_data  = 8'h00;
                if (!i2c_busy) begin
                    i2c_start   = 1'b1;
                end
            end

            ST_INIT_1: begin
                // 設定 SMPLRT_DIV，例如 7 => 1kHz / (1+7) = 125Hz
                i2c_rw       = 1'b0;
                i2c_reg_addr = REG_SMPLRT_DIV;
                i2c_wr_data  = 8'd7;
                if (!i2c_busy) begin
                    i2c_start   = 1'b1;
                end
            end

            ST_INIT_2: begin
                // 設定 DLPF CONFIG (這裡寫0) => 260Hz BW
                i2c_rw       = 1'b0;
                i2c_reg_addr = REG_CONFIG;
                i2c_wr_data  = 8'h00;
                if (!i2c_busy) begin
                    i2c_start   = 1'b1;
                end
            end

            ST_INIT_3: begin
                // 設定 GYRO_CONFIG (這裡選 ±250deg/s, 寫0x00)
                i2c_rw       = 1'b0;
                i2c_reg_addr = REG_GYRO_CONFIG;
                i2c_wr_data  = 8'h00;
                if (!i2c_busy) begin
                    i2c_start   = 1'b1;
                end
            end

            ST_INIT_4: begin
                // 設定 ACCEL_CONFIG (這裡選 ±2g, 寫0x00)
                i2c_rw       = 1'b0;
                i2c_reg_addr = REG_ACCEL_CONFIG;
                i2c_wr_data  = 8'h00;
                if (!i2c_busy) begin
                    i2c_start   = 1'b1;
                end
            end

            // -------------------------------------------------
            // 開始一次取樣：先告訴MPU6050「我要從0x3B開始讀」

            // -------------------------------------------------
            // 之後每次讀 1 byte，總共讀 14 bytes
            ST_READ_BYTE: begin
                i2c_rw       = 1'b1;   // read
                i2c_reg_addr = REG_ACCEL_XOUT_H + byte_index_reg;
                if (!i2c_busy) begin
                    i2c_start   = 1'b1;
                end
            end

            default: begin
                // 其他狀態不做新的start
            end
        endcase
    end

    // =========================================================
    //  主狀態機：控制初始化 + 按固定頻率讀取14 bytes
    // =========================================================

    logic data_valid_reg;
    assign data_valid = data_valid_reg;
    assign fsm_state = state;  // 輸出目前 FSM 狀態

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_RESET;
            byte_index_reg    <= 4'd0;
            data_valid_reg<= 1'b0;
            v_x           <= '0;
            v_y           <= '0;
            v_z           <= '0;
            gyro_x        <= '0;
            gyro_y        <= '0;
            gyro_z        <= '0;
        end else begin
            data_valid_reg <= 1'b0; // 預設0，有更新時才拉1拍

            case (state)
                // ---------------------------------------------
                ST_RESET: begin
                    // 等系統穩定一下再開始 INIT
                    state <= ST_INIT_0;
                end

                // ---------------------------------------------
                ST_INIT_0: begin
                    // 已在 combinational 產生 start, 等 done
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ST_ERROR;
                        else
                            state <= ST_INIT_1;
                    end
                end

                ST_INIT_1: begin
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ST_ERROR;
                        else
                            state <= ST_INIT_2;
                    end
                end

                ST_INIT_2: begin
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ST_ERROR;
                        else
                            state <= ST_INIT_3;
                    end
                end

                ST_INIT_3: begin
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ST_ERROR;
                        else
                            state <= ST_INIT_4;
                    end
                end

                ST_INIT_4: begin
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ST_ERROR;
                        else
                            state <= ST_IDLE;
                    end
                end

                // ---------------------------------------------
                // 閒置：等 sample_tick 再啟動一次讀取流程
                ST_IDLE: begin
                    if (sample_tick) begin
                        byte_index_reg <= 4'd0;
                        state      <= ST_READ_BYTE;
                    end
                end

                // ---------------------------------------------
                // 觸發讀 1 byte
                ST_READ_BYTE: begin
                    data_buf[byte_index_reg] <= i2c_rd_data;
                    if (i2c_done) begin
                        if (i2c_ack_error) begin
                            // 這一輪讀壞了，不更新 buf，直接進 ERROR
                            state <= ST_ERROR;
                        end else begin
                            // 讀結束後，i2c_rd_data 就是一個byte
                            if (byte_index_reg == 4'd13) begin
                                state <= ST_DONE_FRAME;
                            end else begin
                                byte_index_reg <= byte_index_reg + 1;
                                state      <= ST_READ_BYTE; // 再讀下一個byte
                            end
                        end
                    end
                end

                // ---------------------------------------------
                // 一整組14 bytes讀完，組合成 16-bit 變數
                ST_DONE_FRAME: begin
                    // data_buf[0]~data_buf[13] 對應：
                    // 0: ACCEL_XOUT_H, 1: ACCEL_XOUT_L
                    // 2: ACCEL_YOUT_H, 3: ACCEL_YOUT_L
                    // 4: ACCEL_ZOUT_H, 5: ACCEL_ZOUT_L
                    // 6: TEMP_OUT_H,   7: TEMP_OUT_L
                    // 8: GYRO_XOUT_H,  9: GYRO_XOUT_L
                    // 10: GYRO_YOUT_H, 11: GYRO_YOUT_L
                    // 12: GYRO_ZOUT_H, 13: GYRO_ZOUT_L

                    acc_x  <= $signed({data_buf[0], data_buf[1]});
                    acc_y  <= $signed({data_buf[2], data_buf[3]});
                    acc_z  <= $signed({data_buf[4], data_buf[5]});
                    gyro_x <= $signed({data_buf[8], data_buf[9]});
                    gyro_y <= $signed({data_buf[10], data_buf[11]});
                    gyro_z <= $signed({data_buf[12], data_buf[13]});

                    v_x <=  acc_x;
                    v_y <=  acc_y;
                    v_z <=  acc_z;

                    data_valid_reg <= 1'b1;  // 告訴外面：新數據OK了
                    state          <= ST_IDLE;
                end

                // ---------------------------------------------
                // I2C 發生 ACK 錯誤時的處理：丟棄本輪資料，重新初始化
                ST_ERROR: begin
                    // 不更新 acc_x/gyro_x 等輸出，維持上一筆成功的資料
                    data_valid_reg <= 1'b0;   // 這一輪沒有有效新資料
                    state          <= ST_RESET; // 回到 RESET，重新跑 INIT_0~4
                end

                default: state <= ST_RESET;
            endcase
        end
    end

endmodule
