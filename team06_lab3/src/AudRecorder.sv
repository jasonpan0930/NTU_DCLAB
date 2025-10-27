module AudRecorder(
    input i_rst_n,
    input i_bclk,            // BCLK from WM8731
    input i_lrc,            // ADCLRCK (Left/Right Channel Clock)
    input i_start,          // Start recording signal
    input i_pause,          // Pause recording signal  
    input i_stop,           // Stop recording signal
    input i_data,           // ADCDAT from WM8731
    output [19:0] o_address, // SRAM address output
    output [15:0] o_data,   // SRAM data output
    output o_overflow       // Address overflow indicator (1 = overflow, 0 = normal)
);

// Recording state machine
typedef enum logic [1:0] {
    IDLE,
    RECORDING,
    PAUSED
} rec_state_t;

rec_state_t state_r, state_w;

// Internal signals - _r for registered, _w for combinational
logic [15:0] shift_reg_r, shift_reg_w;     // Shift register to collect 16-bit audio data
logic [4:0] bit_counter_r, bit_counter_w;    // Bit counter (0-15)
logic prev_lrc_w, prev_lrc_r;                           // Previous LRC state for edge detection
logic [19:0] address_counter_r, address_counter_w; // Address counter for SRAM
logic overflow_r, overflow_w;                // Overflow flag register

// Recording enable signal
logic recording_en;

// Edge detection
logic lrc_rising;  // Rising edge for right channel (LRC=1)

// Continuous assignments
assign recording_en = (state_r == RECORDING);
assign lrc_rising = !prev_lrc_r && i_lrc;

// --------------------------------------------------------------------------------
// Combinational Logic - All the decision making happens here
// --------------------------------------------------------------------------------
always_comb begin
    // Default: hold values
    state_w = state_r;
    shift_reg_w = shift_reg_r;
    bit_counter_w = bit_counter_r;
    address_counter_w = address_counter_r;
    overflow_w = overflow_r;
    prev_lrc_w = i_lrc;
    // State machine
    case (state_r)
        IDLE: begin
            if (i_start) begin
                state_w = RECORDING;
                shift_reg_w = 16'd0;
                bit_counter_w = 5'd0;
            end
        end
        
        RECORDING: begin
            if (i_stop) begin
                state_w = IDLE;
            end
            else if (i_pause) begin
                state_w = PAUSED;
            end
            
            // I2S receiver logic - only process RIGHT channel (i_lrc == 1)
            if (i_lrc) begin
                if (lrc_rising) begin
                    // LRC rising edge - start of right channel
                    
                    // If we just finished a word (bit_counter_r is 16), increment address
                    if (bit_counter_r == 16) begin
                        // Increment address for next word
                        if (address_counter_r == 20'hFFFFF) begin
                            overflow_w = 1'b1;
                            state_w = IDLE;
                        end
                        else begin
                            address_counter_w = address_counter_r + 1;
                        end
                    end
                    
                    bit_counter_w = 5'd0;  // Reset counter for new word
                end
                else begin
                    if (bit_counter_r <= 15) begin
                        shift_reg_w = {shift_reg_r[14:0], i_data};
                        bit_counter_w = bit_counter_r + 1;
                    end
                end
            end
        end
        
        PAUSED: begin
            if (i_stop) begin
                state_w = IDLE;
            end
            else if (i_start) begin
                state_w = RECORDING;
            end
        end
        
        default: state_w = IDLE;
    endcase
    
    // Reset counters when not recording
    if (!recording_en) begin
        shift_reg_w = 16'd0;
        bit_counter_w = 5'd0;
        
        // Reset address counter only when entering IDLE state
        if (state_w == IDLE) begin
            address_counter_w = 20'd0;
            overflow_w = 1'b0;
        end
    end
end

// --------------------------------------------------------------------------------
// Sequential Logic - Only stores values in registers
// --------------------------------------------------------------------------------
// BCLK operates on NEGATIVE edge per WM8731 spec
always_ff @(negedge i_bclk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        state_r <= IDLE;
        shift_reg_r <= 16'd0;
        bit_counter_r <= 5'd0;
        prev_lrc_r <= 1'b0;
        address_counter_r <= 20'd0;
        overflow_r <= 1'b0;
    end
    else begin
        state_r <= state_w;
        shift_reg_r <= shift_reg_w;
        bit_counter_r <= bit_counter_w;
        prev_lrc_r <= prev_lrc_w;
        address_counter_r <= address_counter_w;
        overflow_r <= overflow_w;
    end
end

// Output assignments
assign o_address = address_counter_r;
assign o_data = shift_reg_r;
assign o_overflow = overflow_r;

endmodule
