module set_trans_arr #(
    parameter int ZOMBIE_SIZE_X = 102,
    parameter int ZOMBIE_SIZE_Y = 149
)(
    input  logic                             i_clk,
    input  logic                             i_rst_n,
    output logic [15:0]                      o_trans_bounds [0:ZOMBIE_SIZE_X-1],
    output logic                             o_done,
    output logic                             o_busy,
    output logic [13:0]                      o_zombie_addr,
    input  logic [7:0]                       i_zombie_pixel
);

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    logic [15:0] trans_bounds [0:ZOMBIE_SIZE_X-1];

    typedef enum logic [1:0] { S_IDLE, S_SCAN, S_DONE } state_t;
    state_t state;

    int unsigned scan_x, scan_y;
    int unsigned pixel_x, pixel_y;
    logic        pixel_valid;

    // Registers for current column accumulation
    logic [7:0] temp_min_y;
    logic [7:0] temp_max_y;
    
    // NEW: Flag to detect the "First" transparent pixel in a column
    logic       first_trans_found;

    // -------------------------------------------------------------------------
    // Main Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            state             <= S_IDLE;
            scan_x            <= 0;
            scan_y            <= 0;
            o_zombie_addr     <= 14'd0;
            o_done            <= 1'b0;
            o_busy            <= 1'b0;
            pixel_valid       <= 1'b0;
            
            // Initialization
            temp_min_y        <= 8'hFF;
            temp_max_y        <= 8'h00;
            first_trans_found <= 1'b0; // Start fresh

            for (int xx = 0; xx < ZOMBIE_SIZE_X; xx++) begin
                trans_bounds[xx] <= 16'h00FF; 
            end

        end else begin
            case (state)
                S_IDLE: begin
                    state             <= S_SCAN;
                    o_busy            <= 1'b1;
                    scan_x            <= 0;
                    scan_y            <= 0;
                    pixel_valid       <= 1'b0;
                    temp_min_y        <= 8'hFF;
                    temp_max_y        <= 8'h00;
                    first_trans_found <= 1'b0;
                end

                S_SCAN: begin
                    // --- 1. Address Request (No change) ---
                    o_zombie_addr <= scan_y * ZOMBIE_SIZE_X + scan_x;
                    pixel_x       <= scan_x;
                    pixel_y       <= scan_y;
                    pixel_valid   <= 1'b1;

                    if (scan_y < ZOMBIE_SIZE_Y - 1) begin
                        scan_y <= scan_y + 1;
                    end else begin
                        scan_y <= 0;
                        if (scan_x < ZOMBIE_SIZE_X - 1) scan_x <= scan_x + 1;
                        else begin
                            state       <= S_DONE;
                            pixel_valid <= 1'b0;
                        end
                    end

                    // --- 2. Data Processing (Optimized) ---
                    if (pixel_valid) begin
                        // Check transparency (0x00 is transparent)
                        if (i_zombie_pixel == 8'h00) begin
                            
                            // OPTIMIZATION 1: No "if (y > max)" check needed.
                            // Since scan goes 0..148, current is always the largest seen so far.
                            temp_max_y <= pixel_y[7:0];

                            // OPTIMIZATION 2: No "if (y < min)" check needed.
                            // The first one we find is automatically the smallest.
                            if (!first_trans_found) begin
                                temp_min_y        <= pixel_y[7:0];
                                first_trans_found <= 1'b1; // Lock it!
                            end
                        end

                        // End of Column Handling
                        if (pixel_y == ZOMBIE_SIZE_Y - 1) begin
                            logic [7:0] final_min;
                            logic [7:0] final_max;
                            
                            // Special check for the very last pixel of the column
                            if (i_zombie_pixel == 8'h00) begin
                                final_max = pixel_y[7:0]; // Last pixel is trans, so it's the max
                                // If this is the FIRST time seeing trans (column was all opaque until now), it's also min
                                final_min = (!first_trans_found) ? pixel_y[7:0] : temp_min_y;
                            end else begin
                                final_max = temp_max_y;
                                final_min = temp_min_y;
                            end

                            // Write result
                            trans_bounds[pixel_x] <= {final_max, final_min};

                            // Reset for NEXT column
                            temp_min_y        <= 8'hFF;
                            temp_max_y        <= 8'h00;
                            first_trans_found <= 1'b0; // Reset flag
                        end
                    end
                end

                S_DONE: begin
                    o_done <= 1'b1;
                    o_busy <= 1'b0;
                end
            endcase
        end
    end

    always_comb begin
        for (int xx = 0; xx < ZOMBIE_SIZE_X; xx++) begin
            o_trans_bounds[xx] = trans_bounds[xx];
        end
    end

endmodule