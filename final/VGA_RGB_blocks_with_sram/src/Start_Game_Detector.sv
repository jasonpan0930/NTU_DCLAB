// Start Game Detector Module
// Detects when the game should start based on aim position and trigger button
// Output: started = 1 when start condition is met, stays 1 until reset
module Start_Game_Detector (
    input logic i_clk,
    input logic i_rst_n,
    
    // Aim position (in VGA clock domain)
    input logic [10:0] i_aim_x,
    input logic [9:0] i_aim_y,
    
    // Trigger button (debounced, in VGA clock domain)
    input logic i_trigger_posedge,  // One-clock pulse when trigger is pressed
    
    // Start box coordinates (parameters)
    input logic [10:0] i_start_box_x_min,
    input logic [10:0] i_start_box_x_max,
    input logic [9:0] i_start_box_y_min,
    input logic [9:0] i_start_box_y_max,
    
    // Output
    output logic o_started  // 0 = not started, 1 = started (latched until reset)
);

    // Check if aim is in start box area
    logic aim_in_start_box;
    always_comb begin
        aim_in_start_box = (i_aim_x >= i_start_box_x_min) && (i_aim_x <= i_start_box_x_max) &&
                           (i_aim_y >= i_start_box_y_min) && (i_aim_y <= i_start_box_y_max);
    end
    
    // Combined condition: aim in box AND trigger pressed
    logic start_condition_met;
    assign start_condition_met = aim_in_start_box && i_trigger_posedge;
    
    // Latch the started signal: once it becomes 1, it stays 1 until reset
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_started <= 1'b0;
        end else begin
            // Once started becomes 1, it stays 1 (latched)
            if (!o_started && start_condition_met) begin
                o_started <= 1'b1;
            end
            // Otherwise, keep current value
        end
    end

endmodule

