// AudDSP_BRAM.sv
// Modified version of AudDSP to read from BRAM instead of SRAM
// This module reads audio data from BRAM (MIF files) instead of SRAM

module AudDSP_BRAM (
    input logic i_rst_n,
    input logic i_clk,
    input logic i_start,
    input logic i_pause,
    input logic i_stop,
    input logic [2:0] i_speed,      // 速度選擇: 0=1x, 1=2x, 2=3x, ..., 7=8x
    input logic i_fast,              // 快速播放模式
    input logic i_slow_0,            // 慢速播放 - 零次內插 (constant)
    input logic i_slow_1,            // 慢速播放 - 一次內插 (linear)
    input logic i_voice_robot,       // 聲音變換：機器人（全波整流）
    input logic i_play_backward,     // 反向播放
    input logic [19:0] i_max_addr,   // 最大位址（用於判斷播放結束）
    input logic i_daclrck,           // DAC Left/Right Clock
    input logic signed [15:0] i_bram_data,  // 從BRAM讀取的音訊資料
    output logic signed [15:0] o_dac_data,  // 輸出到DAC的音訊資料
    output logic [19:0] o_bram_addr         // BRAM讀取位址
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
    // Robot effect LFO (square wave for ring modulation)
    logic [11:0] robot_cnt, robot_cnt_next;
    logic robot_sq, robot_sq_next;
    localparam int ROBOT_TOGGLE_TH = 12'd120;
    // Robot envelope follower
    logic [17:0] robot_env, robot_env_next;
    localparam int ROBOT_ENV_DECAY_SHIFT  = 8;
    localparam int ROBOT_ENV_ATTACK_SHIFT = 6;

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
            robot_cnt <= '0;
            robot_sq <= 1'b0;
            robot_env <= '0;
        end else begin
            state <= state_next;
            addr_counter <= addr_counter_next;
            sample_counter <= sample_counter_next;
            prev_sample <= prev_sample_next;
            curr_sample <= curr_sample_next;
            dac_data_r <= dac_data_next;
            daclrck_prev <= i_daclrck;
            robot_cnt <= robot_cnt_next;
            robot_sq <= robot_sq_next;
            robot_env <= robot_env_next;
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
        robot_cnt_next = robot_cnt;
        robot_sq_next = robot_sq;
        robot_env_next = robot_env;

        case (state)
            S_IDLE: begin
                if (i_start) begin
                    state_next = S_PLAY;
                    addr_counter_next = i_play_backward ? i_max_addr : 20'd0;
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
                    // Update robot square LFO at LRCK rate
                    if (robot_cnt >= ROBOT_TOGGLE_TH) begin
                        robot_cnt_next = 12'd0;
                        robot_sq_next = ~robot_sq;
                    end else begin
                        robot_cnt_next = robot_cnt + 12'd1;
                    end
                    // Envelope follower update
                    begin
                        logic [15:0] abs_curr;
                        logic [17:0] env_decay;
                        logic [17:0] env_attack;
                        abs_curr = dac_data_next[15] ? (-dac_data_next) : dac_data_next;
                        env_decay = robot_env - (robot_env >> ROBOT_ENV_DECAY_SHIFT);
                        env_attack = env_decay + (abs_curr >> ROBOT_ENV_ATTACK_SHIFT);
                        robot_env_next = env_attack;
                    end
                    // 快速播放模式
                    if (i_fast) begin
                        logic [19:0] step_val;
                        case (i_speed)
                            3'd0: step_val = 20'd1;
                            3'd1: step_val = 20'd2;
                            3'd2: step_val = 20'd3;
                            3'd3: step_val = 20'd4;
                            3'd4: step_val = 20'd5;
                            3'd5: step_val = 20'd6;
                            3'd6: step_val = 20'd7;
                            3'd7: step_val = 20'd8;
                            default: step_val = 20'd1;
                        endcase
                        if (i_play_backward) begin
                            addr_counter_next = (addr_counter > (step_val - 20'd1)) ? (addr_counter - step_val) : 20'd0;
                        end else begin
                            addr_counter_next = addr_counter + step_val;
                        end
                        curr_sample_next = i_bram_data;
                        dac_data_next = i_bram_data;
                        sample_counter_next = 3'd0;
                    end 
                    // 慢速播放模式
                    else if (i_slow_0 || i_slow_1) begin
                        sample_counter_next = sample_counter + 3'd1;
                        
                        // 根據速度判斷是否需要讀取新資料
                        case (i_speed)
                            3'd1: begin  // 1/2x
                                if (sample_counter >= 3'd1) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd2: begin  // 1/3x
                                if (sample_counter >= 3'd2) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd3: begin  // 1/4x
                                if (sample_counter >= 3'd3) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd4: begin  // 1/5x
                                if (sample_counter >= 3'd4) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd5: begin  // 1/6x
                                if (sample_counter >= 3'd5) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd6: begin  // 1/7x
                                if (sample_counter >= 3'd6) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd7: begin  // 1/8x
                                if (sample_counter >= 3'd7) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_bram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            default: begin
                                addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                    : (addr_counter + 20'd1);
                                prev_sample_next = curr_sample;
                                curr_sample_next = i_bram_data;
                                sample_counter_next = 3'd0;
                            end
                        endcase

                        // 內插計算
                        if (i_slow_0) begin
                            dac_data_next = curr_sample;
                        end else if (i_slow_1) begin
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
                        addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                            : (addr_counter + 20'd1);
                        curr_sample_next = i_bram_data;
                        dac_data_next = i_bram_data;
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
            end

            default: state_next = S_IDLE;
        endcase
    end

    // 機器人音效處理
    localparam logic signed [15:0] ROBOT_ENV_THRESH = 16'sd256;
    logic signed [15:0] env16;
    logic signed [15:0] wet_robot;
    logic signed [15:0] mixed_out;
    logic signed [15:0] post_robot;
    assign env16 = robot_env[17:2];
    assign wet_robot = (robot_sq ? env16 : -env16);
    assign mixed_out = (wet_robot >>> 1) + (wet_robot >>> 2) + (wet_robot >>> 3) + (wet_robot >>> 5)
                     + (dac_data_r >>> 4) + (dac_data_r >>> 5);
    
    // Hadamard transform buffers
    logic [1:0] h4_in_idx, h4_out_idx;
    logic signed [15:0] h4_in_buf [0:3];
    logic signed [15:0] h4_out_buf[0:3];
    logic h4_out_valid;
    logic signed [15:0] h4_curr_sample;

    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            h4_in_idx <= 2'd0;
            h4_out_idx <= 2'd0;
            h4_out_valid <= 1'b0;
        end else if (daclrck_pos_edge) begin
            if (i_voice_robot) begin
                if (h4_out_valid) begin
                    h4_curr_sample <= h4_out_buf[h4_out_idx];
                    h4_out_idx <= h4_out_idx + 2'd1;
                    if (h4_out_idx == 2'd3) begin
                        h4_out_valid <= 1'b0;
                        h4_out_idx <= 2'd0;
                    end
                end else begin
                    h4_in_buf[h4_in_idx] <= curr_sample;
                    h4_in_idx <= h4_in_idx + 2'd1;
                    if (h4_in_idx == 2'd3) begin
                        logic signed [16:0] s0, s1, s2, s3;
                        logic signed [17:0] c0, c1, c2, c3;
                        logic signed [17:0] m0, m1, m2, m3;
                        logic signed [18:0] t0, t1, t2, t3;
                        s0 = h4_in_buf[0];
                        s1 = h4_in_buf[1];
                        s2 = h4_in_buf[2];
                        s3 = h4_in_buf[3];
                        c0 = s0 + s1 + s2 + s3;
                        c1 = s0 - s1 + s2 - s3;
                        c2 = s0 + s1 - s2 - s3;
                        c3 = s0 - s1 - s2 + s3;
                        m0 = c0;
                        m1 = c1 >>> 1;
                        m2 = (c2 * 3) >>> 2;
                        m3 = (c3 * 5) >>> 2;
                        t0 = m0 + m1 + m2 + m3;
                        t1 = m0 - m1 + m2 - m3;
                        t2 = m0 + m1 - m2 - m3;
                        t3 = m0 - m1 - m2 + m3;
                        h4_out_buf[0] <= t0 >>> 2;
                        h4_out_buf[1] <= t1 >>> 2;
                        h4_out_buf[2] <= t2 >>> 2;
                        h4_out_buf[3] <= t3 >>> 2;
                        h4_out_valid <= 1'b1;
                        h4_in_idx <= 2'd0;
                    end
                end
            end else begin
                h4_in_idx <= 2'd0;
                h4_out_idx <= 2'd0;
                h4_out_valid <= 1'b0;
                h4_curr_sample <= 16'sd0;
            end
        end
    end

    wire signed [15:0] robot_env_mix = (i_voice_robot && (env16 >= ROBOT_ENV_THRESH)) ? mixed_out : dac_data_r;
    wire signed [15:0] robot_path_out = (i_voice_robot && h4_out_valid) ? h4_curr_sample : robot_env_mix;
    assign post_robot = robot_path_out;

    // 輸出指定
    assign o_bram_addr = addr_counter;
    assign o_dac_data = post_robot;

endmodule

