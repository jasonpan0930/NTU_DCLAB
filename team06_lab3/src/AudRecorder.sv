module AudRecorder(
    input i_rst_n,
    input i_clk,            // BCLK from WM8731
    input i_lrc,            // ADCLRCK (Left/Right Channel Clock)
    input i_start,          // Start recording signal
    input i_pause,          // Pause recording signal  
    input i_stop,           // Stop recording signal
    input i_data,           // ADCDAT from WM8731
    output [19:0] o_address, // SRAM address output
    output [15:0] o_data,   // SRAM data output
    output o_overflow       // Address overflow indicator (1 = overflow, 0 = normal)
);

// Internal signals
logic [15:0] shift_reg_r;     // Shift register to collect 16-bit audio data
logic [4:0] bit_counter_r;    // Bit counter (0-15)
logic prev_lrc_r;             // Previous LRC state for edge detection
logic recording_en_w;         // Recording enable signal
logic [19:0] address_counter_r; // Address counter for SRAM
logic overflow_r;             // Overflow flag register

// Recording state machine
typedef enum logic [1:0] {
    IDLE,
    RECORDING,
    PAUSED
} rec_state_t;

rec_state_t state_r, state_w;

// State machine combinational logic
always_comb begin
    case (state_r)
        IDLE: begin
            if (i_start) begin
                state_w = RECORDING;
            end
            else begin
                state_w = IDLE;
            end
        end
        
        RECORDING: begin
            if (i_stop) begin
                state_w = IDLE;
            end
            else if (i_pause) begin
                state_w = PAUSED;
            end
            else begin
                state_w = RECORDING;
            end
        end
        
        PAUSED: begin
            if (i_stop) begin
                state_w = IDLE;
            end
            else if (i_start) begin
                state_w = RECORDING;
            end
            else begin
                state_w = PAUSED;
            end
        end
        
        default: begin
            state_w = IDLE;
        end
    endcase
end

// Recording enable signal
assign recording_en_w = (state_r == RECORDING);

// State machine and I2S receiver logic
always_ff @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        // Reset all registers
        state_r <= IDLE;
        shift_reg_r <= 16'd0;
        bit_counter_r <= 5'd0;
        prev_lrc_r <= 1'b0;
        address_counter_r <= 20'd0;
        overflow_r <= 1'b0;
    end
    else begin
        // Update state machine
        state_r <= state_w;
        
        // I2S receiver logic
        if (recording_en_w) begin
            // Detect LRC edge (channel change)
            if (i_lrc != prev_lrc_r) begin
                // LRC changed, reset bit counter
                bit_counter_r <= 5'd0;
                // Don't increment address here, wait for complete word
            end
            else begin
                // Shift in data bit by bit
                if (bit_counter_r < 16) begin
                    shift_reg_r <= {shift_reg_r[14:0], i_data};
                    
                    // When we've collected 15 bits (about to collect 16th), increment address
                    if (bit_counter_r == 15) begin
                        // Check for overflow before incrementing
                        if (address_counter_r == 20'hFFFFF) begin  // Max 20-bit address
                            overflow_r <= 1'b1;
                        end
                        else begin
                            address_counter_r <= address_counter_r + 1;
                        end
                    end
                    
                    bit_counter_r <= bit_counter_r + 1;
                end
            end
            prev_lrc_r <= i_lrc;
        end
        else begin
            // Reset shift register and bit counter when not recording
            shift_reg_r <= 16'd0;
            bit_counter_r <= 5'd0;
            prev_lrc_r <= 1'b0;
            
            // Reset address counter only when entering IDLE state
            if (state_w == IDLE) begin
                address_counter_r <= 20'd0;
                overflow_r <= 1'b0;  // Clear overflow flag when resetting
            end
            // PAUSED state preserves address_counter_r and overflow_r (no reset)
        end
    end
end

// Output assignments
assign o_address = address_counter_r;
assign o_data = shift_reg_r;
assign o_overflow = overflow_r;

endmodule
