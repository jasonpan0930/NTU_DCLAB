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
    
    // Flag to detect the "First" NON-TRANSPARENT (Opaque) pixel in a column
    logic       first_opaque_found; // Renamed for clarity

    // -------------------------------------------------------------------------
    // Main Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (!i_rst_n) begin
            state              <= S_IDLE;
            scan_x             <= 0;
            scan_y             <= 0;
            o_zombie_addr      <= 14'd0;
            o_done             <= 1'b0;
            o_busy             <= 1'b0;
            pixel_valid        <= 1'b0;
            
            // Initialization
            temp_min_y         <= 8'hFF;
            temp_max_y         <= 8'h00;
            first_opaque_found <= 1'b0; 

            for (int xx = 0; xx < ZOMBIE_SIZE_X; xx++) begin
                trans_bounds[xx] <= 16'h00FF; 
            end

        end else begin
            case (state)
                S_IDLE: begin
                    state              <= S_SCAN;
                    o_busy             <= 1'b1;
                    scan_x             <= 0;
                    scan_y             <= 0;
                    pixel_valid        <= 1'b0;
                    temp_min_y         <= 8'hFF;
                    temp_max_y         <= 8'h00;
                    first_opaque_found <= 1'b0;
                end

                S_SCAN: begin
                    // --- 1. Address Request ---
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

                    // --- 2. Data Processing ---
                    if (pixel_valid) begin
                        
                        // [關鍵修改] Check if pixel is NOT transparent (Looking for Object)
                        // 假設 0x00 是透明，非 0x00 就是實體
                        if (i_zombie_pixel != 8'h00) begin 
                            
                            // 更新 Max: 只要是實體，它一定是目前為止最下面的 (因為掃描順序是 Y 遞增)
                            // 這就達成了「找下面數來第一個非 transparent」的目的
                            temp_max_y <= pixel_y[7:0];

                            // 更新 Min: 如果這是這一列第一次遇到實體，它就是最上面的
                            // 這就達成了「找上面數來第一個非 transparent」的目的
                            if (!first_opaque_found) begin
                                temp_min_y         <= pixel_y[7:0];
                                first_opaque_found <= 1'b1; // Lock it!
                            end
                        end

                        // End of Column Handling
                        if (pixel_y == ZOMBIE_SIZE_Y - 1) begin
                            logic [7:0] final_min;
                            logic [7:0] final_max;
                            
                            // [關鍵修改] Handle the very last pixel of the column
                            if (i_zombie_pixel != 8'h00) begin
                                final_max = pixel_y[7:0]; // Last pixel is opaque, so it is the absolute Max
                                
                                // Check if this is also the Min (i.e., the whole column was transparent until this last pixel)
                                final_min = (!first_opaque_found) ? pixel_y[7:0] : temp_min_y;
                            end else begin
                                // Last pixel is transparent, retain previous results
                                final_max = temp_max_y;
                                final_min = temp_min_y;
                            end

                            // Write result
                            trans_bounds[pixel_x] <= {final_max, final_min};

                            // Reset for NEXT column
                            temp_min_y         <= 8'hFF;
                            temp_max_y         <= 8'h00;
                            first_opaque_found <= 1'b0; 
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