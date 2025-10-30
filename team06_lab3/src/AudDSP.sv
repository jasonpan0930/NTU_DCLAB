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
    input logic i_voice_robot,       // 聲音變換：機器人（全波整流）
    input logic i_play_backward,     // 反向播放
    input logic [19:0] i_max_addr,   // 錄音的最大位址（反向起點）
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
    // Robot effect LFO (square wave for ring modulation)
    logic [11:0] robot_cnt, robot_cnt_next; // enough for ~48k/2/N up to ~2^12
    logic robot_sq, robot_sq_next;
    // 48 kHz sample rate: toggle every 120 samples -> ~200 Hz square (48000/(2*120))
    localparam int ROBOT_TOGGLE_TH = 12'd120;
    // Robot envelope follower
    logic [17:0] robot_env, robot_env_next; // headroom for accumulation
    localparam int ROBOT_ENV_DECAY_SHIFT  = 8; // larger -> slower decay
    localparam int ROBOT_ENV_ATTACK_SHIFT = 6; // smaller -> faster attack

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
            // （移除頻移 NCO 初始化）
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
                    // Envelope follower update (leaky integrator on |sample|)
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
                            3'd0: step_val = 20'd1;  // 1x
                            3'd1: step_val = 20'd2;  // 2x
                            3'd2: step_val = 20'd3;  // 3x
                            3'd3: step_val = 20'd4;  // 4x
                            3'd4: step_val = 20'd5;  // 5x
                            3'd5: step_val = 20'd6;  // 6x
                            3'd6: step_val = 20'd7;  // 7x
                            3'd7: step_val = 20'd8;  // 8x
                            default: step_val = 20'd1;
                        endcase
                        if (i_play_backward) begin
                            addr_counter_next = (addr_counter > (step_val - 20'd1)) ? (addr_counter - step_val) : 20'd0;
                        end else begin
                            addr_counter_next = addr_counter + step_val;
                        end
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
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd2: begin  // 1/3x: 每3個cycle讀一次
                                if (sample_counter >= 3'd2) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd3: begin  // 1/4x
                                if (sample_counter >= 3'd3) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd4: begin  // 1/5x
                                if (sample_counter >= 3'd4) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd5: begin  // 1/6x
                                if (sample_counter >= 3'd5) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd6: begin  // 1/7x
                                if (sample_counter >= 3'd6) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            3'd7: begin  // 1/8x
                                if (sample_counter >= 3'd7) begin
                                    addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                        : (addr_counter + 20'd1);
                                    prev_sample_next = curr_sample;
                                    curr_sample_next = i_sram_data;
                                    sample_counter_next = 3'd0;
                                end
                            end
                            default: begin
                                addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                                    : (addr_counter + 20'd1);
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
                        addr_counter_next = i_play_backward ? ((addr_counter > 20'd0) ? (addr_counter - 20'd1) : 20'd0)
                                                            : (addr_counter + 20'd1);
                        curr_sample_next = i_sram_data;
                        // Default output is raw sample; may be replaced by transform below
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

    // 機器人音效：以語音包絡(robot_env)乘上固定頻率carrier（方波），並與原音混合（dry/wet）
    localparam logic signed [15:0] ROBOT_ENV_THRESH = 16'sd256; // 低於此包絡則直接乾聲，抑制底噪
    logic signed [15:0] env16;
    logic signed [15:0] wet_robot;
    logic signed [15:0] mixed_out;
    logic signed [15:0] post_robot;
    assign env16 = robot_env[17:2]; // scale envelope to 16-bit
    assign wet_robot = (robot_sq ? env16 : -env16);
    // 約 90% 濕聲 + 10% 乾聲（以移位加法近似：濕=0.90625，乾=0.09375）
    assign mixed_out = (wet_robot >>> 1) + (wet_robot >>> 2) + (wet_robot >>> 3) + (wet_robot >>> 5)
                     + (dac_data_r >>> 4) + (dac_data_r >>> 5);
    // 簡易 4 點 Hadamard 變換以改變音色（矩陣變換），僅在啟用時覆蓋輸出
    // 緩衝 4 筆輸入與 4 筆輸出
    logic [1:0] h4_in_idx, h4_out_idx;
    logic signed [15:0] h4_in_buf [0:3];
    logic signed [15:0] h4_out_buf[0:3];
    logic h4_out_valid;
    logic signed [15:0] h4_curr_sample;

    // （移除頻移功能，保留原有訊號定義）

    

    // 在 LRCK 正緣時收樣/輸出
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            h4_in_idx <= 2'd0;
            h4_out_idx <= 2'd0;
            h4_out_valid <= 1'b0;
        end else if (daclrck_pos_edge) begin
            // 當啟用機器人音效時，執行 Hadamard 區塊處理
            if (i_voice_robot) begin
                // 如果有已產生的輸出，先送出下一筆
                if (h4_out_valid) begin
                    h4_curr_sample <= h4_out_buf[h4_out_idx];
                    h4_out_idx <= h4_out_idx + 2'd1;
                    if (h4_out_idx == 2'd3) begin
                        h4_out_valid <= 1'b0;
                        h4_out_idx <= 2'd0;
                    end
                end else begin
                    // 累積 4 筆輸入
                    h4_in_buf[h4_in_idx] <= curr_sample; // 使用已緩存的當前樣本
                    h4_in_idx <= h4_in_idx + 2'd1;
                    if (h4_in_idx == 2'd3) begin
                        // 已收滿 4 筆，計算 H4、調整係數、反變換
                        logic signed [16:0] s0, s1, s2, s3;
                        logic signed [17:0] c0, c1, c2, c3;
                        logic signed [17:0] m0, m1, m2, m3; // 調整後係數
                        logic signed [18:0] t0, t1, t2, t3;
                        s0 = h4_in_buf[0];
                        s1 = h4_in_buf[1];
                        s2 = h4_in_buf[2];
                        s3 = h4_in_buf[3];
                        // H4 係數
                        c0 = s0 + s1 + s2 + s3;
                        c1 = s0 - s1 + s2 - s3;
                        c2 = s0 + s1 - s2 - s3;
                        c3 = s0 - s1 - s2 + s3;
                        // 係數調整（矩陣領域塑形）：保留 c0、削弱 c1、輕度增強 c3
                        m0 = c0;
                        m1 = c1 >>> 1;         // 0.5x
                        m2 = (c2 * 3) >>> 2;   // 0.75x
                        m3 = (c3 * 5) >>> 2;   // 1.25x
                        // 反變換（H4）並除以 4
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
                // 未啟用時由上游路徑決定輸出（不改寫）
                h4_in_idx <= 2'd0;
                h4_out_idx <= 2'd0;
                h4_out_valid <= 1'b0;
                h4_curr_sample <= 16'sd0;
            end

            // （移除頻移運算）
        end
    end

    // 若未使用矩陣路徑，使用先前的 envelope 混合；否則以矩陣輸出覆蓋
    wire signed [15:0] robot_env_mix = (i_voice_robot && (env16 >= ROBOT_ENV_THRESH)) ? mixed_out : dac_data_r;
    wire signed [15:0] robot_path_out = (i_voice_robot && h4_out_valid) ? h4_curr_sample : robot_env_mix;
    assign post_robot = robot_path_out;

    // 輸出指定
    assign o_sram_addr = addr_counter;
    assign o_dac_data = post_robot;

endmodule
