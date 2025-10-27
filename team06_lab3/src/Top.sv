// Top.sv
// Complete implementation with AudDSP integration
// Controls recording, playback, speed adjustment via buttons

module Top (
    input  i_rst_n,
    input  i_clk,

    // Buttons
    input  i_key_0,  // Start/Pause toggle
    input  i_key_1,  // Record/Play mode switch
    input  i_key_2,  // Stop/Reset
	input  [17:0] i_sw, //Speed Control

    // ==== Clocks from PLL/Qsys ====
    input  i_clk_100k,   // for I2C bit engine
    input  i_clk_12m,    // 12 MHz master clock for WM8731

    // ==== SRAM ====
    output [19:0] o_SRAM_ADDR,
    inout  [15:0] io_SRAM_DQ,
    output        o_SRAM_WE_N, //(Write Enable)控制是否執行寫入，當此訊號為 0 時，允許將資料寫入 SRAM
    output        o_SRAM_CE_N, //(Chip Enable)控制是否啟用這顆 SRAM，為 0 時 SRAM 可以被讀或寫，為 1 時記憶體對外部不作用
    output        o_SRAM_OE_N, //(Output Enable)控制 SRAM 是否驅動數據到資料匯流排（讀取時），為 0 時可以將資料從 SRAM 輸出至資料線
    output        o_SRAM_LB_N, //(Lower Byte Enable)若 SRAM 是 16bit 資料，這個訊號控制是否允許低 8 bit 的資料存取
    output        o_SRAM_UB_N, //(Upper Byte Enable)若 SRAM 是 16bit 資料，這個訊號控制是否允許高 8 bit 的資料存取

    // ==== I2C ====
    output o_I2C_SCLK,
    inout  io_I2C_SDAT,

    // ==== WM8731 / I2S ====
    input  i_AUD_ADCDAT,
    inout  i_AUD_ADCLRCK,
    inout  i_AUD_BCLK,
    inout  i_AUD_DACLRCK,
    output o_AUD_DACDAT,

	// ==== LED ====
	output o_LEDG_recording,
	output o_LEDR_rec_overflow,
	output o_LEDR_ack_in_i2C_init,
	output [5:0] o_LEDG_state  // State indicator LEDs
);


    // ----------------------------------------------------------------
    // State machine
    // ----------------------------------------------------------------
    localparam S_IDLE       = 3'd0;
    localparam S_I2C        = 3'd1;
    localparam S_RECD       = 3'd2;
    localparam S_RECD_PAUSE = 3'd3;
    localparam S_PLAY       = 3'd4;
    localparam S_PLAY_PAUSE = 3'd5;

    // I2C wires
    logic i2c_oen, i2c_sdat;
    logic i2c_start, i2c_finished;
    logic i2c_ledr_nack;  // NACK detection LED from I2C initializer

    // SRAM / Audio buffers
    logic [19:0] addr_record, addr_play;
    logic [15:0] data_record, data_play, dac_data;

    // FSM regs
    logic [2:0] state_r, state_next;
    
    // LED state indicators
    logic [5:0] ledg_state;

    // DSP control signals
    logic dsp_start, dsp_pause, dsp_stop;
    logic dsp_fast, dsp_slow_0, dsp_slow_1;
	logic [2:0] dsp_speed;

    // Keys are already debounced and pulsed in DE2_115.sv
    // Use them directly without additional edge detection

    // Recorder control
    logic rec_start, rec_pause, rec_stop;
	logic rec_overflow;

    // ---- Open-drain SDA ----
    assign io_I2C_SDAT = (i2c_oen) ? i2c_sdat : 1'bz;

    // ---- SRAM wiring ----
    assign o_SRAM_ADDR = (state_r == S_RECD || state_r == S_RECD_PAUSE) ? addr_record : addr_play;
    assign io_SRAM_DQ  = (state_r == S_RECD) ? data_record : 16'hzzzz;
    assign data_play   = io_SRAM_DQ;

    assign o_SRAM_WE_N = (state_r == S_RECD) ? 1'b0 : 1'b1;  // Write during record
    assign o_SRAM_CE_N = 1'b0;
    assign o_SRAM_OE_N = 1'b0;
    assign o_SRAM_LB_N = 1'b0;
    assign o_SRAM_UB_N = 1'b0;

    assign o_LEDG_recording = rec_start;
    assign o_LEDR_rec_overflow = rec_overflow;
    assign o_LEDR_ack_in_i2C_init = i2c_ledr_nack;
    
    // State indicator LEDs
    // LEDG[0]: S_IDLE - Idle state
    // LEDG[1]: S_I2C - I2C initialization
    // LEDG[2]: S_RECD - Recording
    // LEDG[3]: S_RECD_PAUSE - Recording paused
    // LEDG[4]: S_PLAY - Playback
    // LEDG[5]: S_PLAY_PAUSE - Playback paused
    assign ledg_state = (state_r == S_IDLE)       ? 6'b000001 :
                        (state_r == S_I2C)         ? 6'b000010 :
                        (state_r == S_RECD)        ? 6'b000100 :
                        (state_r == S_RECD_PAUSE)  ? 6'b001000 :
                        (state_r == S_PLAY)        ? 6'b010000 :
                        (state_r == S_PLAY_PAUSE)  ? 6'b100000 : 6'b000000;
    
    assign o_LEDG_state = ledg_state;

    // ----------------------------------------------------------------
    // I2C Initializer
    // ----------------------------------------------------------------
    I2cInitializer init0 (
        .i_rst_n    (i_rst_n),
        .i_clk      (i_clk_100k),
        .i_start    (i2c_start),
        .i_sdat     (io_I2C_SDAT),  // Read SDA line to detect ACK/NACK
        .o_finished (i2c_finished),
        .o_sclk     (o_I2C_SCLK),
        .o_sdat     (i2c_sdat),
        .o_oen      (i2c_oen),
        .o_ledr     (i2c_ledr_nack) // LED: lit when NACK detected
    );

    // ----------------------------------------------------------------
    // AudDSP - Digital Signal Processing & Playback Control
    // ----------------------------------------------------------------
    AudDSP dsp0 (
        .i_rst_n     (i_rst_n),
        .i_clk       (i_AUD_BCLK),       // Use bit clock for DSP
        .i_start     (dsp_start),
        .i_pause     (dsp_pause),
        .i_stop      (dsp_stop),
        .i_speed     (dsp_speed),        // Speed control (0=1x, 1=2x, ..., 7=8x)
        .i_fast      (dsp_fast),         // Fast playback mode
        .i_slow_0    (dsp_slow_0),       // Slow with constant interpolation
        .i_slow_1    (dsp_slow_1),       // Slow with linear interpolation
        .i_daclrck   (i_AUD_DACLRCK),
        .i_sram_data (data_play),
        .o_dac_data  (dac_data),
        .o_sram_addr (addr_play)
    );

    // ----------------------------------------------------------------
    // AudPlayer - I2S TX (DAC output)
    // ----------------------------------------------------------------
    AudPlayer player0 (
        .i_rst_n      (i_rst_n),
        .i_bclk       (i_AUD_BCLK),
        .i_daclrck    (i_AUD_DACLRCK),
        .i_en         ((state_r == S_PLAY)),  // Enable during playback
        .i_dac_data   (dac_data),
        .o_aud_dacdat (o_AUD_DACDAT)
    );

    // ----------------------------------------------------------------
    // AudRecorder - I2S RX (ADC input)
    // ----------------------------------------------------------------
    AudRecorder recorder0 (
        .i_rst_n   (i_rst_n),
        .i_bclk    (i_AUD_BCLK),
        .i_lrc     (i_AUD_ADCLRCK),
        .i_start   (rec_start),
        .i_pause   (rec_pause),
        .i_stop    (rec_stop),
        .i_data    (i_AUD_ADCDAT),
        .o_address (addr_record),
        .o_data    (data_record),
        .o_overflow(rec_overflow)
    );

    // ----------------------------------------------------------------
    // FSM Control Logic
    // ----------------------------------------------------------------
    always_comb begin
        // Default values
        state_next = state_r;
        i2c_start  = 1'b0;
        
        // DSP controls
        dsp_start  = 1'b0;
        dsp_pause  = 1'b0;
        dsp_stop   = 1'b0;
        dsp_fast   = 1'b0;
        dsp_slow_0 = 1'b0;
        dsp_slow_1 = 1'b0;
        dsp_speed  = 3'd0;  // Default 1x speed
        
        // Recorder controls
        rec_start  = 1'b0;
        rec_pause  = 1'b0;
        rec_stop   = 1'b0;

        case (state_r)
            S_IDLE: begin
                if (i_key_0) begin
                    state_next = S_I2C;  // Initialize codec
                end
            end

            S_I2C: begin
                i2c_start = 1'b1;
                if (i2c_finished) begin
                    state_next = S_RECD;  // Start recording after init
                end
            end

            S_RECD: begin
                rec_start = 1'b1;
                
                if (i_key_2) begin
                    state_next = S_IDLE;
                    rec_stop = 1'b1;
                end else if (i_key_0) begin
                    state_next = S_RECD_PAUSE;
                    rec_pause = 1'b1;
                end else if (i_key_1) begin
                    state_next = S_PLAY;  // Switch to playback
                    rec_stop = 1'b1;
                end
            end

            S_RECD_PAUSE: begin
                rec_pause = 1'b1;
                
                if (i_key_2) begin
                    state_next = S_IDLE;
                    rec_stop = 1'b1;
                end else if (i_key_0) begin
                    state_next = S_RECD;  // Resume recording
                    rec_start = 1'b1;
                end else if (i_key_1) begin
                    state_next = S_PLAY;
                    rec_stop = 1'b1;
                end
            end

            S_PLAY: begin
                dsp_start = 1'b1;
                
                // Example: Use switches or additional buttons for speed control
                // Here we default to normal playback (1x speed)
                // You can extend this with more inputs
                dsp_speed = i_sw[2:0];  
                dsp_fast = i_sw[17];  
                dsp_slow_0 = i_sw[16]; 
				dsp_slow_1 = i_sw[15];
                
                if (i_key_2) begin
                    state_next = S_IDLE;
                    dsp_stop = 1'b1;
                end else if (i_key_0) begin
                    state_next = S_PLAY_PAUSE;
                    dsp_pause = 1'b1;
                end else if (i_key_1) begin
                    state_next = S_RECD;  // Switch back to recording
                    dsp_stop = 1'b1;
                end
            end

            S_PLAY_PAUSE: begin
                dsp_pause = 1'b1;
                
                if (i_key_2) begin
                    state_next = S_IDLE;
                    dsp_stop = 1'b1;
                end else if (i_key_0) begin
                    state_next = S_PLAY;  // Resume playback
                    dsp_start = 1'b1;
                end else if (i_key_1) begin
                    state_next = S_RECD;
                    dsp_stop = 1'b1;
                end
            end

            default: state_next = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------------
    // State register
    // ----------------------------------------------------------------
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state_r <= S_IDLE;
        end else begin
            state_r <= state_next;
        end
    end

endmodule
