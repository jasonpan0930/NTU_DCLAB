module audio_controller (
    // 時鐘和重置
    input  logic clk,              // 50MHz 系統時鐘
    input  logic clk_12m,          // 12MHz 音訊主時鐘
    input  logic clk_100k,         // 100kHz I2C時鐘
    input  logic vga_clock_74_25,  // 74.25MHz VGA時鐘（用於檢測kill_en）
    input  logic rst_n,
    
    // 擊殺信號（來自VGA模組）
    input  logic kill_en,
    
    // I2C接口
    output logic I2C_SCLK,
    inout  logic I2C_SDAT,
    
    // 音訊接口
    input  logic AUD_BCLK,
    input  logic AUD_DACLRCK,
    output logic AUD_DACDAT,
    
    // 狀態輸出（用於LED顯示）
    output logic audio_playing,
    output logic i2c_init_done,
    output logic i2c_ledr_nack
);

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
    logic i2c_init_done_internal;
    
    // I2C 初始化信號
    logic i2c_start, i2c_finished;
    logic i2c_sdat, i2c_oen;
    logic i2c_ledr_nack_internal;
    
    // 音訊播放控制信號
    logic audio_playing_internal;
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
    logic [1:0] next_counter;
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
    
    // 用於狀態機的kill_en_posedge（直接使用同步後的脈衝）
    logic kill_en_posedge;
    assign kill_en_posedge = kill_en_posedge_pulse_sync2;
    
    // I2C初始化完成標誌
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i2c_init_done_internal <= 1'b0;
        end else begin
            if (i2c_finished && (audio_state == AUD_I2C_INIT)) begin
                i2c_init_done_internal <= 1'b1;
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
        audio_playing_internal = 1'b0;
        
        case (audio_state)
            AUD_IDLE: begin
                // 系統啟動後自動開始播放BGM
                if (i2c_init_done_internal) begin
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
                audio_playing_internal = 1'b1;
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
                audio_playing_internal = 1'b1;
                // 如果播放killed時再次擊殺，保持KILLED狀態（地址會重置）
                if (kill_en_posedge) begin
                    audio_state_next = AUD_PLAY_KILLED;  // 保持KILLED狀態，地址會重置
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
        .o_ledr(i2c_ledr_nack_internal)
    );
    
    // I2C SDA 開漏輸出
    assign I2C_SDAT = (i2c_oen) ? i2c_sdat : 1'bz;
    
    // =========================================================
    // 簡單音訊播放邏輯
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
                if (audio_state == AUD_PLAY_KILLED) begin
                    bram_addr <= 20'd7000;  // killed從地址1000開始播放
                end else begin
                    bram_addr <= 20'd0;
                end
            end
            // 在AUD_PLAY_KILLED狀態時，如果kill_en_posedge為true，重置地址
            else if (audio_state == AUD_PLAY_KILLED && kill_en_posedge_sync_bclk2) begin
                bram_addr <= 20'd7000;  // 重置地址到1000，重新播放killed
            end
            // 正常播放時遞增地址
            else if (audio_playing_internal && daclrck_posedge) begin
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
        .i_en(audio_playing_internal),
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
    
    // 輸出信號
    assign audio_playing = audio_playing_internal;
    assign i2c_init_done = i2c_init_done_internal;
    assign i2c_ledr_nack = i2c_ledr_nack_internal;
    
endmodule

