// Start Game Detector Module
// Detects when the game should start based on aim position and trigger button
// Output: started = 1 when start condition is met, stays 1 until reset
module Start_Game_Detector #(
    parameter START_BOX_X_MIN = 11'd494,  // 11 bits to match i_aim_x[10:0]
    parameter START_BOX_X_MAX = 11'd778,  // 11 bits to match i_aim_x[10:0]
    parameter START_BOX_Y_MIN = 10'd571,  // 10 bits to match i_aim_y[9:0]
    parameter START_BOX_Y_MAX = 10'd630   // 10 bits to match i_aim_y[9:0]
)(
    input logic i_clk,
    input logic i_rst_n,
    
    // Aim position (in VGA clock domain)
    input logic [10:0] i_aim_x,
    input logic [9:0] i_aim_y,
    
    // Trigger button (debounced, in VGA clock domain)
    input logic i_trigger_posedge,  // One-clock pulse when trigger is pressed
    
    // Manual reset button (KEY[3], active low, synchronized to VGA clock domain)
    input logic i_key3,  // KEY[3] input (active low)
    
    // Enable signal (when low, detector is disabled)
    input logic i_enable,  // Enable signal: 1 = enabled, 0 = disabled
    
    // Output
    output logic o_started  // 0 = not started, 1 = started (latched until reset)
);

    // Synchronize KEY[3] to VGA clock domain and detect press (active low)
    logic key3_sync1, key3_sync2, key3_sync3;
    logic key3_pressed;  // One-clock pulse when KEY[3] is pressed
    
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            key3_sync1 <= 1'b1;  // KEY is active low, so default to 1 (not pressed)
            key3_sync2 <= 1'b1;
            key3_sync3 <= 1'b1;
        end else begin
            key3_sync1 <= i_key3;
            key3_sync2 <= key3_sync1;
            key3_sync3 <= key3_sync2;
        end
    end
    
    // Detect falling edge: KEY[3] pressed (1 -> 0)
    assign key3_pressed = key3_sync2 && !key3_sync3;  // Falling edge detection
    
    // Check if aim is in start box area using parameters
    logic aim_in_start_box;
    always_comb begin
        aim_in_start_box = (i_aim_x >= START_BOX_X_MIN) && (i_aim_x <= START_BOX_X_MAX) &&
                           (i_aim_y >= START_BOX_Y_MIN) && (i_aim_y <= START_BOX_Y_MAX);
    end
    
    // Combined condition: aim in box AND trigger pressed AND enabled
    // Only allow start condition when NOT in reset AND enabled
    logic start_condition_met;
    assign start_condition_met = i_rst_n && i_enable && aim_in_start_box && i_trigger_posedge;
    
    // Internal started signal: latch the started signal, once it becomes 1, it stays 1 until reset
    logic started_internal;
    
    // Initialize to 0 to prevent unknown state
    initial begin
        started_internal = 1'b0;
    end
    
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            started_internal <= 1'b0;
        end else begin
            // Reset when KEY[3] is pressed
            if (key3_pressed) begin
                started_internal <= 1'b0;
            end
            // Once started becomes 1, it stays 1 (latched) - unless KEY[3] resets it
            else if (!started_internal && start_condition_met) begin
                started_internal <= 1'b1;
            end
            // Otherwise, keep current value
        end
    end
    
    // Output flip-flop: register the internal signal before output
    logic started_output;
    initial begin
        started_output = 1'b0;
    end
    
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            started_output <= 1'b0;
        end else begin
            started_output <= started_internal;
        end
    end
    
    assign o_started = started_output;

endmodule

