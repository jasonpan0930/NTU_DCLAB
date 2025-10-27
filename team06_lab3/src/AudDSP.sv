module AudDSP (
    input logic i_rst_n,
    input logic i_clk,
    input logic i_start,
    input logic i_pause,
    input logic i_stop,
    input logic [2:0] i_speed,      // 速度選擇: 0=1x, 1=2x, 2=3x, ..., 7=8x
    input logic i_fast,              // 快速播放模式
    input logic i_slow_0,            // 慢速播放 - 零次內插 (constant)
    input logic i_slow_1,            // 慢速播放 - 一次內插 (linear)
    input logic i_daclrck,           // DAC Left/Right Clock
    input logic signed [15:0] i_sram_data,  // 從SRAM讀取的音訊資料
    output logic signed [15:0] o_dac_data,  // 輸出到DAC的音訊資料
    output logic [19:0] o_sram_addr         // SRAM讀取位址
);

    // 狀態定義
    typedef enum logic [1:0] {
        S_IDLE,
        S_PLAY,
        S_PAUSE
    } state_t;

    state_t state, state_next;

    // 內部暫存器
    logic [19:0] addr_counter, addr_counter_next;
    logic [2:0] sample_counter, sample_counter_next;  // 用於慢速播放的取樣計數
    logic signed [15:0] prev_sample, prev_sample_next;  // 前一筆資料，用於內插
    logic signed [15:0] curr_sample, curr_sample_next;  // 當前資料
    logic signed [15:0] dac_data_r, dac_data_next;
    logic daclrck_prev;

    // 邊緣偵測
    wire daclrck_pos_edge = i_daclrck && !daclrck_prev;

    // 狀態機 - Sequential logic
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= S_IDLE;
            addr_counter <= 20'd0;
            sample_counter <= 3'd0;
            prev_sample <= 16'd0;
            curr_sample <= 16'd0;
            dac_data_r <= 16'd0;
            daclrck_prev <= 1'b0;
        end else begin
            state <= state_next;
            addr_counter <= addr_counter_next;
            sample_counter <= sample_counter_next;
            prev_sample <= prev_sample_next;
            curr_sample <= curr_sample_next;
            dac_data_r <= dac_data_next;
            daclrck_prev <= i_daclrck;
        end
    end

    // 狀態機 - Combinational logic
    always_comb begin
        // 預設值
        state_next = state;
        addr_counter_next = addr_counter;
        sample_counter_next = sample_counter;
        prev_sample_next = prev_sample;
        curr_sample_next = curr_sample;
        dac_data_next = dac_data_r;

        case (state)
            S_IDLE: begin
                if (i_start) begin
                    state_next = S_PLAY;
                    addr_counter_next = 20'd0;
                    sample_counter_next = 3'd0;
                end
            end

            S_PLAY: begin
                if (i_stop) begin
                    state_next = S_IDLE;
                    addr_counter_next = 20'd0;
                    sample_counter_next = 3'd0;
                end else if (i_pause) begin
                    state_next = S_PAUSE;
                end else if (daclrck_pos_edge) begin
                    // 快速播放模式
                    if (i_fast) begin
                        case (i_speed)
                            3'd0: addr_counter_next = addr_counter + 20'd1;  // 1x
                            3'd1: addr_counter_next = addr_counter + 20'd2;  // 2x
                            3'd2: addr_counter_next = addr_counter + 20'd3;  // 3x
                            3'd3: addr_counter_next = addr_counter + 20'd4;  // 4x
                            3'd4: addr_counter_next = addr_counter + 20'd5;  // 5x
                            3'd5: addr_counter_next = addr_counter + 20'd6;  // 6x
                            3'd6: addr_counter_next = addr_counter + 20'd7;  // 7x
                            3'd7: addr_counter_next = addr_counter + 20'd8;  // 8x
                            default: addr_counter_next = addr_counter + 20'd1;
                        endcase
                        curr_sample_next = i_sram_data;
                        dac_data_next = i_sram_data;
                        sample_counter_next = 3'd0;
                    end 
                    // 慢速播放模式
                    else if (i_slow_0 || i_slow_1) begin
                        sample_counter_next = sample_counter + 3'd1;
                        
                        // 根據速度判斷是否需要讀取新資料
                        case (i_speed)
                            3'd1: begin  // 1/2x: 每2個cycle讀一次
                                if (sample_counter >= 3'd1) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd2: begin  // 1/3x: 每3個cycle讀一次
                                if (sample_counter >= 3'd2) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd3: begin  // 1/4x
                                if (sample_counter >= 3'd3) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd4: begin  // 1/5x
                                if (sample_counter >= 3'd4) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd5: begin  // 1/6x
                                if (sample_counter >= 3'd5) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd6: begin  // 1/7x
                                if (sample_counter >= 3'd6) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd7: begin  // 1/8x
                                if (sample_counter >= 3'd7) begin
                                    addr_counter_next = addr_counter + 20'd1;
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            default: begin
                                addr_counter_next = addr_counter + 20'd1;
                                prev_sample_next = curr_sample;
                                curr_sample_next = i_sram_data;
                                sample_counter_next = 3'd0;
                            end
                        endcase

                        // 內插計算
                        if (i_slow_0) begin
                            // 零次內插 (constant): 保持前一個值
                            dac_data_next = curr_sample;
                        end else if (i_slow_1) begin
                            // 一次內插 (linear): 線性內插
                            logic signed [15:0] diff;
                            logic signed [31:0] interpolated;
                            
                            diff = curr_sample - prev_sample;
                            
                            case (i_speed)
                                3'd1: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 2;
                                3'd2: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 3;
                                3'd3: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 4;
                                3'd4: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 5;
                                3'd5: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 6;
                                3'd6: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 7;
                                3'd7: interpolated = $signed(prev_sample) + ($signed(diff) * ($signed(sample_counter) + 3'd1)) / 8;
                                default: interpolated = curr_sample;
                            endcase
                            
                            dac_data_next = interpolated[15:0];
                        end
                    end
                    // 正常播放
                    else begin
                        addr_counter_next = addr_counter + 20'd1;
                        curr_sample_next = i_sram_data;
                        dac_data_next = i_sram_data;
                    end
                end
            end

            S_PAUSE: begin
                if (i_stop) begin
                    state_next = S_IDLE;
                    addr_counter_next = 20'd0;
                    sample_counter_next = 3'd0;
                end else if (i_start) begin
                    state_next = S_PLAY;
                end
                // 暫停時保持當前輸出
            end

            default: state_next = S_IDLE;
        endcase
    end

    // 輸出指定
    assign o_sram_addr = addr_counter;
    assign o_dac_data = dac_data_r;

endmodule
