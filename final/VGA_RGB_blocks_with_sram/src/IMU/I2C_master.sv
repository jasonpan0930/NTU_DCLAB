// i2c_master.sv
// 簡化版：支援一次對某個 dev_addr / reg_addr 做 1 byte READ 或 WRITE
// I2C標準模式約 100kHz（需依實際clk_hz與DIV設計調整）

module i2c_master #(
    parameter CLK_HZ = 50_000_000, // FPGA系統時脈
    parameter I2C_HZ = 100_000 // 目標I2C頻率（調試用，正常為 100_000）
    )(
    input logic clk,
    input logic rst_n,

    // 上層控制介面
    input logic start, // 拉高1拍開始一筆交易
    input logic rw, // 0=WRITE, 1=READ
    input logic [6:0] dev_addr, // 7-bit I2C器件位址，例如MPU6050=7'h68
    input logic [7:0] reg_addr, // 要存取的暫存器位址，例如0x3B
    input logic [7:0] wr_data, // 要寫入的資料(WRITE時使用)
    output logic [7:0] rd_data, // 讀回資料(READ時有效)
    output logic busy, // 交易進行中
    output logic done, // 交易完成(拉高1拍)
    output logic ack_error, // 任一步沒有收到ACK則=1

    // I2C 實體腳位
    output logic scl,
    inout wire sda
);

    // =========================================================
    // I2C 時脈產生 (簡單分頻，給 SCL 使用)
    // =========================================================

    localparam int DIV = (CLK_HZ / (I2C_HZ * 4));
    // *4 是因為一bit用4個clk phase: low準備 -> low保持 -> high保持 -> high準備

    logic [$clog2(DIV)-1:0] clk_cnt;
    logic [1:0] phase; // 0~3 對應上面四個phase
    logic i2c_clk_en; // 每個phase切換一次

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= '0;
            phase <= 2'd0;
            i2c_clk_en <= 1'b0;
        end else begin
            if (clk_cnt == DIV-2) begin
                clk_cnt <= clk_cnt + 1;
                i2c_clk_en <= 1'b1;
            end else if (clk_cnt == DIV-1) begin
                clk_cnt <= '0;
                phase <= phase + 2'd1;
                i2c_clk_en <= 1'b0;
            end else begin
                clk_cnt <= clk_cnt + 1;
                i2c_clk_en <= 1'b0;
            end
        end
    end

    // SCL 由 state 控制：這裡先準備一個暫存
    logic scl_o; // 我們內部控制的SCL狀態
    assign scl = scl_o;

    // =========================================================
    // SDA 開漏控制 (inout)
    // =========================================================

    logic sda_o; // 想要輸出的值(0/1)
    logic sda_oe; // 輸出enable；1=主動拉低或拉高，0=高阻由pull-up決定

    assign sda = sda_oe ? sda_o : 1'bz; // 開漏：通常只拉低(0)，高電平交給pull-up
    wire sda_i = sda; // 讀回線上實際電位

    // 規範上Master應只主動拉低(0)，拉高靠上拉電阻，
    // 本簡化範例中sda_o只會輸出0，拉高就是 sda_oe=0 (高阻->被上拉為1)

    // =========================================================
    // 狀態機定義
    // =========================================================

    typedef enum logic [3:0] {
        ST_IDLE = 4'd0,
        ST_START = 4'd1,
        ST_DEV_ADDR = 4'd2,
        ST_DEV_ACK = 4'd3,
        ST_REG_ADDR = 4'd4,
        ST_REG_ACK = 4'd5,
        ST_RESTART = 4'd6, // 只在READ時使用
        ST_DEV_ADDR_R = 4'd7,
        ST_DEV_ACK_R = 4'd8,
        ST_WRITE_DATA = 4'd9,
        ST_WRITE_ACK = 4'd10,
        ST_READ_DATA = 4'd11,
        ST_READ_NACK = 4'd12,
        ST_STOP = 4'd13
    } state_t;

    state_t state;

    logic [7:0] shifter;
    logic [2:0] bit_cnt;
    logic rw_reg;
    logic [7:0] rd_data_reg;
    logic ack_err_reg;
    logic done_reg;
    logic busy_reg;

    assign rd_data = rd_data_reg;
    assign ack_error = ack_err_reg;
    assign done = done_reg;
    assign busy = busy_reg;

    // =========================================================
    // 主狀態機 (同步邏輯)
    // =========================================================

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            shifter <= 8'h00;
            bit_cnt <= 3'd7;
            rw_reg <= 1'b0;
            rd_data_reg<= 8'h00;
            ack_err_reg<= 1'b0;
            done_reg <= 1'b0;
            busy_reg <= 1'b0;
            scl_o <= 1'b1;
            sda_o <= 1'b1;
            sda_oe <= 1'b0; // 高阻，靠pull-up為1
        end else begin
            done_reg <= 1'b0; // 預設為0，有完成事才拉1拍

            if (i2c_clk_en) begin
                case (state)
                    // -------------------------------------------------
                    ST_IDLE: begin
                        scl_o <= 1'b1; // bus idle: SCL/SDA都在高電位
                        sda_oe <= 1'b0; // 高阻，靠pull-up拉高
                        busy_reg<= 1'b0;
                        if (start && phase == 2'd3) begin
                            // 鎖存操作類型
                            rw_reg <= rw;
                            ack_err_reg <= 1'b0;
                            busy_reg <= 1'b1;
                            sda_oe <= 1'b1;

                            // 進入Start條件
                            state <= ST_START;
                        end
                    end

                    // -------------------------------------------------
                    ST_START: begin
                        // 產生 START：SCL=1 時 SDA 從1->0
                        case (phase)
                            2'd0: begin
                                scl_o <= 1'b1;
                                sda_o <= 1'b1;
                            end
                            2'd1: begin
                                sda_o <= 1'b0; // 拉低 SDA
                            end
                            2'd3: begin
                                scl_o <= 1'b0; // SCL 拉低，開始bit傳輸
                                // 準備送 dev_addr + R/W(第0次為write)
                                shifter <= {dev_addr, 1'b0}; // 先以寫模式送裝置位址
                                bit_cnt <= 3'd7;
                                state <= ST_DEV_ADDR;
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 傳送第一個: dev_addr + write(0)
                    ST_DEV_ADDR: begin
                        case (phase)
                            2'd0: begin
                                sda_o <= shifter[7]; // 送MSB
                            end
                            2'd1: begin
                                scl_o <= 1'b1; // SCL高，對方會讀SDA
                            end
                            2'd3: begin
                                // 一bit完成，移位
                                shifter <= {shifter[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    bit_cnt <= 3'd7;
                                    // 下一拍進入ACK階段
                                    state <= ST_DEV_ACK;
                                    scl_o <= 1'b0;
                                    sda_oe <= 1'b0;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                    scl_o <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 檢查裝置地址 ACK
                    ST_DEV_ACK: begin
                        case (phase)
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                if (sda_i == 1'b1) begin
                                    ack_err_reg <= 1'b1; // 沒收到ACK
                                end
                                // 接下來送寄存器位址
                                shifter <= reg_addr;
                                bit_cnt <= 3'd7;
                                state <= ST_REG_ADDR;
                                scl_o <= 1'b0;
                                sda_oe <= 1'b1;
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 傳送寄存器位址
                    ST_REG_ADDR: begin
                        case (phase)
                            2'd0: begin
                                sda_o <= shifter[7];
                            end
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                shifter <= {shifter[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    bit_cnt <= 3'd7;
                                    state <= ST_REG_ACK;
                                    scl_o <= 1'b0;
                                    sda_oe <= 1'b0;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                    scl_o <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 檢查reg地址ACK
                    ST_REG_ACK: begin
                        case (phase)
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                if (sda_i == 1'b1) begin
                                    ack_err_reg <= 1'b1;
                                end
                                sda_oe <= 1'b1;
                                if (rw_reg == 1'b0) begin
                                    // WRITE: 下一步直接送資料
                                    shifter <= wr_data;
                                    bit_cnt <= 3'd7;
                                    state <= ST_WRITE_DATA;
                                    scl_o <= 1'b0;
                                end else begin
                                    // READ: 產生 repeated START,之後換 read方向送addr
                                    state <= ST_RESTART;
                                end
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // repeated START (READ 用)
                    ST_RESTART: begin
                        case (phase)
                            2'd0: begin
                                // 先確保 SDA高
                                scl_o <= 1'b1;
                                sda_o <= 1'b1; // 先釋放 SDA
                            end
                            2'd1: begin
                                sda_o <= 1'b0; // 拉低 SDA
                            end
                            2'd3: begin
                                // 準備送 dev_addr + read(1)
                                scl_o <= 1'b0;
                                shifter <= {dev_addr, 1'b1};
                                bit_cnt <= 3'd7;
                                state <= ST_DEV_ADDR_R;
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 傳送 dev_addr + read
                    ST_DEV_ADDR_R: begin
                        case (phase)
                            2'd0: begin
                                sda_o <= shifter[7];
                            end
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                shifter <= {shifter[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    bit_cnt <= 3'd7;
                                    state <= ST_DEV_ACK_R;
                                    scl_o <= 1'b0;
                                    sda_oe <= 1'b0;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                    scl_o <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 檢查 read 地址 ACK
                    ST_DEV_ACK_R: begin
                        case (phase)
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                if (sda_i == 1'b1) begin
                                    ack_err_reg <= 1'b1;
                                end
                                // 接著讀1 byte
                                rd_data_reg <= 8'h00;
                                state <= ST_READ_DATA;
                                scl_o <= 1'b0;
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 讀1 byte資料
                    ST_READ_DATA: begin
                        case (phase)
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd2: begin
                                // 在 phase 2 採樣 SDA（SCL 高電平期間）
                                rd_data_reg <= {rd_data_reg[6:0], sda_i};
                            end
                            2'd3: begin
                                if (bit_cnt == 0) begin
                                    bit_cnt <= 3'd7;
                                    state <= ST_READ_NACK;
                                    scl_o <= 1'b0;
                                    sda_oe <= 1'b1;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                    scl_o <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 發NACK，表示只讀這1 byte就好
                    ST_READ_NACK: begin
                        case (phase)
                            2'd0: begin
                                sda_o <= 1'b1; // NACK => SDA保持高
                            end
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                state <= ST_STOP;
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // WRITE: 傳送1 byte wr_data
                    ST_WRITE_DATA: begin
                        case (phase)
                            2'd0: begin
                                sda_o <= shifter[7];
                            end
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                shifter <= {shifter[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    bit_cnt <= 3'd7;
                                    state <= ST_WRITE_ACK;
                                    scl_o <= 1'b0;
                                    sda_oe <= 1'b0;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                    scl_o <= 1'b0;
                                end
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // 檢查 WRITE 資料的 ACK
                    ST_WRITE_ACK: begin
                        case (phase)
                            2'd1: begin
                                scl_o <= 1'b1;
                            end
                            2'd3: begin
                                if (sda_i == 1'b1) begin
                                    ack_err_reg <= 1'b1;
                                end
                                state <= ST_STOP;
                                sda_oe <= 1'b1;
                            end
                        endcase
                    end

                    // -------------------------------------------------
                    // STOP條件：SCL=1時，SDA由0->1
                    ST_STOP: begin
                        case (phase)
                            2'd0: begin
                                scl_o <= 1'b1;
                                sda_o <= 1'b0; // 確保SDA先拉低
                            end
                            2'd1: begin
                                sda_o <= 1'b1;
                            end
                            2'd3: begin
                                sda_oe <= 1'b0; // 釋放 SDA -> 被上拉為1
                                busy_reg <= 1'b0;
                                done_reg <= 1'b1; // 交易完成一拍
                                state <= ST_IDLE;
                            end
                        endcase
                    end

                    default: state <= ST_IDLE;
                endcase
            end // if (i2c_clk_en)

        end
    end

endmodule