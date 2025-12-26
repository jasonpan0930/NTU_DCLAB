module audio_control (
    // 時鐘和重置
    input  logic clk,              // 50MHz 系統時鐘
    input  logic clk_12m,          // 12MHz 音訊主時鐘
    input  logic clk_100k,         // 100kHz I2C時鐘
    input  logic vga_clock_74_25,  // 74.25MHz VGA時鐘
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
            // 播放KILLED時，從killed BRAM讀取
            // killed從7000開始，所以killed BRAM的地址是 bram_addr - 7000
            bgm_addr = 17'd0;  // bgm BRAM不使用
            if (bram_addr >= 20'd7000) begin
                // 減去7000後使用低16位
                killed_addr = (bram_addr - 20'd7000) & 20'hFFFF;  // 使用位運算確保16位
            end else begin
                killed_addr = 16'd0;  // 安全機制：如果地址小於7000，使用0
            end
            bram_data = killed_data;  // 使用killed BRAM的數據
        end else begin
            // 其他狀態時，兩個BRAM都不使用，默認輸出0
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
    
    // 產生持續800個cycle的脈衝
    logic kill_en_posedge_pulse;
    logic [9:0] kill_en_pulse_counter;  // 需要10位來存儲800
    logic [9:0] next_counter;
    always_ff @(posedge vga_clock_74_25 or negedge rst_n) begin
        if (!rst_n) begin
            kill_en_posedge_pulse <= 1'b0;
            kill_en_pulse_counter <= 10'd0;
        end else begin
            if (kill_en_posedge_vga) begin
                kill_en_posedge_pulse <= 1'b1;
                kill_en_pulse_counter <= 10'd800;  // 持續800個cycle
            end else if (kill_en_pulse_counter > 10'd0) begin
                // counter遞減，當遞減後的counter > 0時保持脈衝為1
                next_counter = kill_en_pulse_counter - 10'd1;
                kill_en_pulse_counter <= next_counter;
                kill_en_posedge_pulse <= (next_counter > 10'd0);
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
    
    // 將bram_addr同步到clk域（用於狀態機判斷）
    logic [19:0] bram_addr_sync1, bram_addr_sync2;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr_sync1 <= 20'd0;
            bram_addr_sync2 <= 20'd0;
        end else begin
            bram_addr_sync1 <= bram_addr;
            bram_addr_sync2 <= bram_addr_sync1;
        end
    end
    
    // 狀態機組合邏輯
    always_comb begin
        // 默認狀態：如果沒有特殊情況，就進入BGM
        audio_state_next = AUD_PLAY_BGM;
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
                end else begin
                    audio_state_next = AUD_I2C_INIT;  // 等待初始化完成
                end
            end
            
            AUD_PLAY_BGM: begin
                audio_playing_internal = 1'b1;
                // 擊殺發生時，切換到播放killed音效（打斷BGM）
                if (kill_en_posedge) begin
                    audio_state_next = AUD_PLAY_KILLED;  // 切換到播放killed
                end else begin
                    // 沒有擊殺時，保持播放BGM（默認行為）
                    audio_state_next = AUD_PLAY_BGM;
                end
            end
            
            AUD_PLAY_KILLED: begin
                audio_playing_internal = 1'b1;
                // 如果播放killed時再次擊殺，重置地址重新播放killed
                if (kill_en_posedge) begin
                    audio_state_next = AUD_PLAY_KILLED;  // 保持KILLED狀態，但會觸發地址重置
                end
                // killed播放完成後，恢復播放BGM
                // 注意：killed從7000開始，所以結束地址應該是7000+41519=48519
                else if (bram_addr_sync2 >= 20'd48519) begin  // killed從7000開始，到48519結束
                    audio_state_next = AUD_PLAY_BGM;  // 恢復播放BGM，地址會重置
                end else begin
                    // 還在播放killed，保持狀態
                    audio_state_next = AUD_PLAY_KILLED;
                end
            end
            
            AUD_RESET_ADDR: begin
                // 這個狀態不再需要，直接轉換到播放狀態
                audio_state_next = AUD_PLAY_BGM;
            end
            
            default: begin
                // 任何未知狀態都回到BGM（安全機制）
                audio_state_next = AUD_PLAY_BGM;
            end
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
    // 簡單音訊播放邏輯（直接在audio_control中實現）
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
    // 將audio_state同步到AUD_BCLK時鐘域
    audio_state_t audio_state_sync1, audio_state_sync2;
    // 將kill_en_posedge同步到AUD_BCLK時鐘域（用於地址重置）
    logic kill_en_posedge_sync_bclk1, kill_en_posedge_sync_bclk2;
    // 將audio_playing_internal同步到AUD_BCLK時鐘域
    logic audio_playing_sync1, audio_playing_sync2;
    always_ff @(posedge AUD_BCLK or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr <= 20'd0;
            audio_state_prev <= AUD_IDLE;
            audio_state_sync1 <= AUD_IDLE;
            audio_state_sync2 <= AUD_IDLE;
            kill_en_posedge_sync_bclk1 <= 1'b0;
            kill_en_posedge_sync_bclk2 <= 1'b0;
            audio_playing_sync1 <= 1'b0;
            audio_playing_sync2 <= 1'b0;
        end else begin
            // 同步audio_state到AUD_BCLK域
            audio_state_sync1 <= audio_state;
            audio_state_sync2 <= audio_state_sync1;
            audio_state_prev <= audio_state_sync2;
            // 同步kill_en_posedge到AUD_BCLK時鐘域
            kill_en_posedge_sync_bclk1 <= kill_en_posedge;
            kill_en_posedge_sync_bclk2 <= kill_en_posedge_sync_bclk1;
            // 同步audio_playing_internal到AUD_BCLK時鐘域
            audio_playing_sync1 <= audio_playing_internal;
            audio_playing_sync2 <= audio_playing_sync1;
            
            // 優先級1：狀態切換時重置地址（最高優先級）
            if (audio_state_sync2 != audio_state_prev) begin
                if (audio_state_sync2 == AUD_PLAY_KILLED) begin
                    bram_addr <= 20'd7000;  // killed從地址7000開始播放（立即）
                end else if (audio_state_sync2 == AUD_PLAY_BGM) begin
                    bram_addr <= 20'd0;  // BGM從地址0開始播放
                end else begin
                    // 其他狀態也重置為0（安全機制，確保不會卡住）
                    bram_addr <= 20'd0;
                end
            end
            // 優先級2：在AUD_PLAY_KILLED狀態時，如果kill_en_posedge為true，重置地址
            else if (audio_state_sync2 == AUD_PLAY_KILLED && kill_en_posedge_sync_bclk2) begin
                bram_addr <= 20'd7000;  // 重置地址到7000，重新播放killed（立即）
            end
            // 優先級3：確保在AUD_PLAY_KILLED狀態時，地址永遠不小於7000（安全檢查）
            else if (audio_state_sync2 == AUD_PLAY_KILLED && bram_addr < 20'd7000) begin
                bram_addr <= 20'd7000;  // 如果地址小於7000，立即設為7000（防止從0開始）
            end
            // 優先級4：正常播放時遞增地址
            else if (audio_playing_sync2 && daclrck_posedge) begin
                // 根據當前狀態決定最大地址
                if (audio_state_sync2 == AUD_PLAY_BGM) begin
                    if (bram_addr >= 20'd117519) begin
                        bram_addr <= 20'd0;  // BGM循環播放，重置到0
                    end else begin
                        bram_addr <= bram_addr + 20'd1;
                    end
                end else if (audio_state_sync2 == AUD_PLAY_KILLED) begin
                    // killed從7000開始，到48519結束（7000+41519）
                    // 確保地址至少為7000
                    if (bram_addr < 20'd7000) begin
                        bram_addr <= 20'd7000;  // 如果地址小於7000，設為7000
                    end else if (bram_addr >= 20'd48519) begin
                        bram_addr <= 20'd0;  // Killed播放完成，會切換回BGM，地址重置為0
                    end else begin
                        bram_addr <= bram_addr + 20'd1;
                    end
                end else begin
                    // 其他狀態不遞增地址，重置為0（安全機制）
                    bram_addr <= 20'd0;
                end
            end
            // 如果audio_playing為0但應該播放，重置地址（安全機制）
            else if (!audio_playing_sync2 && (audio_state_sync2 == AUD_PLAY_BGM || audio_state_sync2 == AUD_PLAY_KILLED)) begin
                // 如果應該播放但audio_playing為0，重置地址（防止卡住）
                if (audio_state_sync2 == AUD_PLAY_KILLED) begin
                    bram_addr <= 20'd7000;
                end else begin
                    bram_addr <= 20'd0;
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

