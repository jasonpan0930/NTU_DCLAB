// VGA Image Overlay Module (Combined & Layered)
// Architecture: 5-Stage Pipeline
// Layer 0: Candidate Selection & Address Calculation
// Layer 1: Bounds Request & SRAM Address Out
// Layer 2: Bounds Check & ID Select & SRAM Data Latch
// Layer 3: Pixel Index Fetch (Pixel BRAM & SRAM Parse)
// Layer 4: Palette Fetch
// Layer 5: RGB Conversion & Mixing
module VGA_Image_Overlay_Combined #(
    parameter MAX_ZOMBIES = 5,
    parameter ZOMBIE_SIZE_X = 102,
    parameter ZOMBIE_SIZE_Y = 149
)(
    input logic i_clk,
    input logic i_rst_n,
    input logic [10:0] i_h_count,
    input logic [9:0] i_v_count,
    input logic i_active_video,
    
    // Inputs
    input logic [10:0] i_zombie_x [0:MAX_ZOMBIES-1],
    input logic [9:0] i_zombie_y [0:MAX_ZOMBIES-1],
    input logic i_zombie_valid [0:MAX_ZOMBIES-1],
    input logic [10:0] i_zombie_size_x,
    input logic [9:0] i_zombie_size_y,
    input logic [10:0] i_aim_x,
    input logic [9:0] i_aim_y,

    // Shared Zombie Pixel BRAM (Stage 3 -> 4)
    output logic [13:0] o_zombie_addr,
    input  logic [7:0]  i_zombie_pixel,

    // Background SRAM (Stage 1 -> 2)
    output logic [19:0] o_bg_sram_addr,
    output logic o_bg_sram_ce_n,
    inout  wire [15:0]  io_bg_sram_dq,
    output logic o_bg_sram_lb_n,
    output logic o_bg_sram_oe_n,
    output logic o_bg_sram_ub_n,
    output logic o_bg_sram_we_n,
    
    // Output
    output logic [7:0] o_vga_r,
    output logic [7:0] o_vga_g,
    output logic [7:0] o_vga_b,

	// killing part
	input logic i_killing,
	output logic o_kill_en,
	output logic [4:0] o_kill_idx
);

    // Constants
    localparam BG_WIDTH = 1280;
    localparam AIM_SIZE_X = 200;
    localparam AIM_SIZE_Y = 200;

    // ============================================================
    //  Killing Logic: Capture Target Position
    // ============================================================
    logic [10:0] kill_target_x;
    logic [9:0] kill_target_y;
    logic kill_target_valid;
    
    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            kill_target_x <= 0;
            kill_target_y <= 0;
            kill_target_valid <= 0;
        end else begin
            if (i_killing) begin
                // Capture aim center position when killing signal goes high
                kill_target_x <= i_aim_x;
                kill_target_y <= i_aim_y;
                kill_target_valid <= 1'b1;
            end else if (kill_target_valid && 
                         (i_h_count == kill_target_x) && 
                         (i_v_count == kill_target_y)) begin
                // Clear valid flag after we've processed this position
                kill_target_valid <= 1'b0;
            end
        end
    end

    // ============================================================
    //  LAYER 0: Logic Calculation & Candidate Selection
    // ============================================================
    
    // --- 0.1 Background Address Calc ---
    logic [20:0] bg_addr_s0;
    assign bg_addr_s0 = (i_v_count * BG_WIDTH) + i_h_count;
    
    // --- 0.1.5 Kill Target Detection ---
    logic at_kill_target_s0;
    assign at_kill_target_s0 = kill_target_valid && 
                                (i_h_count == kill_target_x) && 
                                (i_v_count == kill_target_y);

    // --- 0.2 Aim Calc ---
    // Convert center coordinates to top-left corner (use signed arithmetic)
    logic signed [11:0] aim_top_left_x_signed;
    logic signed [10:0] aim_top_left_y_signed;
    assign aim_top_left_x_signed = $signed({1'b0, i_aim_x}) - 12'sd100; // Center - (AIM_SIZE_X/2)
    assign aim_top_left_y_signed = $signed({1'b0, i_aim_y}) - 11'sd100; // Center - (AIM_SIZE_Y/2)
    
    logic in_aim_s0;
    logic [7:0] aim_lx_s0, aim_ly_s0;
    logic [15:0] aim_addr_s0;
    
    // Signed comparison for boundaries that may be negative
    logic signed [11:0] h_count_signed, h_count_plus_aim_signed;
    logic signed [10:0] v_count_signed, v_count_plus_aim_signed;
    assign h_count_signed = $signed({1'b0, i_h_count});
    assign v_count_signed = $signed({1'b0, i_v_count});
    assign h_count_plus_aim_signed = aim_top_left_x_signed + 12'sd200;
    assign v_count_plus_aim_signed = aim_top_left_y_signed + 11'sd200;

    assign in_aim_s0 = (h_count_signed >= aim_top_left_x_signed) && (h_count_signed < h_count_plus_aim_signed) &&
                       (v_count_signed >= aim_top_left_y_signed) && (v_count_signed < v_count_plus_aim_signed);
    assign aim_lx_s0 = i_h_count - aim_top_left_x_signed[10:0];
    assign aim_ly_s0 = i_v_count - aim_top_left_y_signed[9:0];
    // Optimize: 200*y + x
    assign aim_addr_s0 = in_aim_s0 ? ((aim_ly_s0 << 7) + (aim_ly_s0 << 6) + (aim_ly_s0 << 3) + aim_lx_s0) : 16'd0;

    // --- 0.3 Zombie Top-4 Selection ---
    logic [10:0] top_zombies_x_s0 [0:3];
    logic [9:0]  top_zombies_y_s0 [0:3];
    logic [5:0]  top_zombies_idx_s0 [0:3];
    logic        top_zombies_valid_s0 [0:3];

    always_comb begin
        int found_count; // Declare at top
        
        // Init
        for (int k = 0; k < 4; k++) begin
            top_zombies_x_s0[k]     = 11'd0;
            top_zombies_y_s0[k]     = 10'd0;
            top_zombies_idx_s0[k]   = 6'd0;
            top_zombies_valid_s0[k] = 1'b0;
        end
        
        found_count = 0;

        for (int z = 0; z < MAX_ZOMBIES; z++) begin
            logic [10:0] cur_x;
            logic [9:0]  cur_y;
            logic        hit;

            cur_x = i_h_count - i_zombie_x[z];
            cur_y = i_v_count - i_zombie_y[z];

            if (i_zombie_valid[z] && 
                (i_h_count >= i_zombie_x[z]) && (i_h_count < (i_zombie_x[z] + i_zombie_size_x)) &&
                (i_v_count >= i_zombie_y[z]) && (i_v_count < (i_zombie_y[z] + i_zombie_size_y))) 
                hit = 1'b1;
            else 
                hit = 1'b0;

            if (hit && (found_count < 4)) begin
                top_zombies_idx_s0[found_count] = z[5:0];
                top_zombies_x_s0[found_count]     = cur_x;
                top_zombies_y_s0[found_count]     = cur_y;
                top_zombies_valid_s0[found_count] = 1'b1;
                found_count++;
            end
        end
    end

    // ------------------------------------------------------------
    // === PIPELINE REGISTER PASSING (Layer 0 -> Layer 1) ===
    // ------------------------------------------------------------
    logic [10:0] z_x_s1 [0:3];
    logic [9:0]  z_y_s1 [0:3];
    logic [5:0]  z_idx_s1 [0:3];
    logic        z_valid_s1 [0:3];
    logic [20:0] bg_addr_s1;
    logic [15:0] aim_addr_s1;
    logic        in_aim_s1;
    logic        active_video_s1;
    logic        at_kill_target_s1;

    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            bg_addr_s1 <= 0; active_video_s1 <= 0; in_aim_s1 <= 0; at_kill_target_s1 <= 0;
        end else begin
            // Zombie Data
            for(int k=0; k<4; k++) begin
                z_x_s1[k] <= top_zombies_x_s0[k];
                z_y_s1[k] <= top_zombies_y_s0[k];
                z_idx_s1[k] <= top_zombies_idx_s0[k];
                z_valid_s1[k] <= top_zombies_valid_s0[k];
            end
            // BG & Aim & Control
            bg_addr_s1      <= bg_addr_s0;
            aim_addr_s1     <= aim_addr_s0;
            in_aim_s1       <= in_aim_s0;
            active_video_s1 <= i_active_video;
            at_kill_target_s1 <= at_kill_target_s0;
        end
    end


    // ============================================================
    //  LAYER 1: Memory Request (Bounds & Aim & BG Addr)
    // ============================================================

    // --- 1.1 Bounds BRAM Request ---
    logic [15:0] bounds_data_0, bounds_data_1; // Returns in Layer 2
    logic [15:0] bounds_data_2, bounds_data_3;

    zombie_1x_bound ram_bounds_1 (
        .clock(i_clk),
        .address_a(z_x_s1[0][6:0]), .q_a(bounds_data_0),
        .address_b(z_x_s1[1][6:0]), .q_b(bounds_data_1)
    );
    zombie_1x_bound2 ram_bounds_2 (
        .clock(i_clk),
        .address_a(z_x_s1[2][6:0]), .q_a(bounds_data_2),
        .address_b(z_x_s1[3][6:0]), .q_b(bounds_data_3)
    );

    // --- 1.2 Aim BRAM Request ---
    logic [1:0] aim_pixel_data_s2; // Returns in Layer 2
    aim aim_bram (
        .clock(i_clk),
        .address(aim_addr_s1),
        .q(aim_pixel_data_s2)
    );

    // --- 1.3 SRAM Control Output ---
    // Output address immediately to pins (Combinational from Register S1)
    assign o_bg_sram_ce_n = 1'b0;
    assign o_bg_sram_oe_n = 1'b0;
    assign o_bg_sram_we_n = 1'b1;
    assign o_bg_sram_addr = bg_addr_s1[20:1]; // Word Addr
    assign o_bg_sram_lb_n = bg_addr_s1[0];
    assign o_bg_sram_ub_n = ~bg_addr_s1[0];
    assign io_bg_sram_dq  = 16'hZZZZ;

    // ------------------------------------------------------------
    // === PIPELINE REGISTER PASSING (Layer 1 -> Layer 2) ===
    // ------------------------------------------------------------
    // Note: BRAM data is implicit register inside BRAM IP
    logic [10:0] z_x_s2 [0:3];
    logic [9:0]  z_y_s2 [0:3];
    logic [5:0]  z_idx_s2 [0:3];
    logic        z_valid_s2 [0:3];
    logic [20:0] bg_addr_s2; // Keep tracking for parsing low/high byte later
    logic        in_aim_s2;
    logic        active_video_s2;
    logic        at_kill_target_s2;

    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            active_video_s2 <= 0; at_kill_target_s2 <= 0;
        end else begin
            // Pass Zombie Info for Checking
            for(int k=0; k<4; k++) begin
                z_x_s2[k] <= z_x_s1[k];
                z_y_s2[k] <= z_y_s1[k];
                z_idx_s2[k] <= z_idx_s1[k];
                z_valid_s2[k] <= z_valid_s1[k];
            end
            // Pass BG Info
            bg_addr_s2 <= bg_addr_s1;
            // Pass Aim & Control
            in_aim_s2 <= in_aim_s1;
            active_video_s2 <= active_video_s1;
            at_kill_target_s2 <= at_kill_target_s1;
        end
    end


    // ============================================================
    //  LAYER 2: Bounds Check & ID Select & SRAM Data Latch
    // ============================================================

    // --- 2.1 Bounds Check & Priority Select ---
    logic [3:0] is_opaque_s2;
    logic [5:0] final_id_s2;
    logic [10:0] final_x_s2;
    logic [9:0]  final_y_s2;
	logic use_zombie;

    always_comb begin
        // Check Opacity using Data returned from Layer 1
        is_opaque_s2[0] = (z_valid_s2[0] && (z_y_s2[0] >= bounds_data_0[7:0]) && (z_y_s2[0] <= bounds_data_0[15:8]));
        is_opaque_s2[1] = (z_valid_s2[1] && (z_y_s2[1] >= bounds_data_1[7:0]) && (z_y_s2[1] <= bounds_data_1[15:8]));
        is_opaque_s2[2] = (z_valid_s2[2] && (z_y_s2[2] >= bounds_data_2[7:0]) && (z_y_s2[2] <= bounds_data_2[15:8]));
        is_opaque_s2[3] = (z_valid_s2[3] && (z_y_s2[3] >= bounds_data_3[7:0]) && (z_y_s2[3] <= bounds_data_3[15:8]));

        // Priority Select
        if (is_opaque_s2[0]) begin
            final_id_s2 = z_idx_s2[0]; final_x_s2 = z_x_s2[0]; final_y_s2 = z_y_s2[0]; use_zombie = 1'b1;
        end else if (is_opaque_s2[1]) begin
            final_id_s2 = z_idx_s2[1]; final_x_s2 = z_x_s2[1]; final_y_s2 = z_y_s2[1]; use_zombie = 1'b1;
        end else if (is_opaque_s2[2]) begin
            final_id_s2 = z_idx_s2[2]; final_x_s2 = z_x_s2[2]; final_y_s2 = z_y_s2[2]; use_zombie = 1'b1;
        end else if (is_opaque_s2[3]) begin
            final_id_s2 = z_idx_s2[3]; final_x_s2 = z_x_s2[3]; final_y_s2 = z_y_s2[3]; use_zombie = 1'b1;
        end else begin
			use_zombie = 1'b0;
            final_x_s2 = 0; final_y_s2 = 0; final_id_s2 = 6'd0;
        end
    end

    // --- 2.2 Calculate Zombie Pixel Address ---
    logic [13:0] z_pixel_addr_s2;
    logic        use_zombie_s2;
    assign use_zombie_s2 = use_zombie;
    assign z_pixel_addr_s2 = use_zombie_s2 ? ((final_y_s2 * i_zombie_size_x) + final_x_s2) : 14'd0;

    // --- 2.3 Kill Detection Logic ---
    // Check if we're at the kill target position and there's a zombie
    logic kill_detected_s2;
    assign kill_detected_s2 = at_kill_target_s2 && use_zombie_s2;
    
    // Hold kill outputs for 10 cycles
    logic [3:0] kill_hold_counter; // 0-10 counter
    logic kill_en_reg;
    logic [4:0] kill_idx_reg;
    
    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            kill_hold_counter <= 4'd0;
            kill_en_reg <= 1'b0;
            kill_idx_reg <= 5'd0;
        end else begin
            if (kill_detected_s2 && (kill_hold_counter == 4'd0)) begin
                // New kill detected, latch values and start counter
                kill_en_reg <= 1'b1;
                kill_idx_reg <= final_id_s2[4:0];
                kill_hold_counter <= 4'd1; // Start counting
            end else if (kill_hold_counter > 4'd0) begin
                // Counting down
                if (kill_hold_counter < 4'd10) begin
                    kill_hold_counter <= kill_hold_counter + 4'd1;
                end else begin
                    // Reached 10 cycles, clear outputs
                    kill_hold_counter <= 4'd0;
                    kill_en_reg <= 1'b0;
                    kill_idx_reg <= 5'd0;
                end
            end
        end
    end
    
    assign o_kill_en = kill_en_reg;
    assign o_kill_idx = kill_idx_reg;

    // ------------------------------------------------------------
    // === PIPELINE REGISTER PASSING (Layer 2 -> Layer 3) ===
    // ------------------------------------------------------------
    logic [15:0] sram_data_latched_s3;
    logic        bg_low_byte_s3;
    logic [1:0]  aim_data_s3;
    logic        in_aim_s3;
    logic        use_zombie_s3;
    logic        active_video_s3;
    
    // NOTE: o_zombie_addr connects directly here to go to BRAM
    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            o_zombie_addr <= 0; active_video_s3 <= 0;
        end else begin
            // 1. Zombie Request (Send to BRAM)
            o_zombie_addr <= z_pixel_addr_s2;
            use_zombie_s3 <= use_zombie_s2;

            // 2. BG Data Latch (SRAM Data is ready now, after 2 cycles)
            sram_data_latched_s3 <= io_bg_sram_dq; 
            bg_low_byte_s3       <= bg_addr_s2[0]; // Needed to split 16-bit to 8-bit

            // 3. Aim Data Delay
            aim_data_s3 <= aim_pixel_data_s2;
            in_aim_s3   <= in_aim_s2;

            // 4. Control
            active_video_s3 <= active_video_s2;
        end
    end


    // ============================================================
    //  LAYER 3: Pixel Index Fetch (Palette Req & SRAM Parse)
    // ============================================================

    // --- 3.1 Parse SRAM Data to Index ---
    logic [7:0] bg_pixel_index_s3;
    assign bg_pixel_index_s3 = (bg_low_byte_s3 == 1'b0) ? sram_data_latched_s3[7:0] : sram_data_latched_s3[15:8];

    // --- 3.2 Zombie BRAM Data ---
    // i_zombie_pixel is valid at the END of this cycle (Layer 3) / Start of Layer 4

    // ------------------------------------------------------------
    // === PIPELINE REGISTER PASSING (Layer 3 -> Layer 4) ===
    // ------------------------------------------------------------
    // NOTE: Palette BRAM address inputs connect here
    logic [1:0] aim_data_s4;
    logic       in_aim_s4;
    logic       use_zombie_s4;
    logic       active_video_s4;
	logic [7:0] zombie_pixel_index_s4;
	logic [7:0] bg_pixel_index_s4;

    // Internal wires for Palette BRAM connection
    // logic [7:0] bg_pal_addr;
    // logic [7:0] z_pal_addr;

    // assign bg_pal_addr = bg_pixel_index_s3;
    // assign z_pal_addr  = i_zombie_pixel; // Data from BRAM returning in S3

    always_ff @(posedge i_clk) begin
        if(~i_rst_n) begin 
            active_video_s4 <= 0; 
        end else begin
            // Delay Control & Aim to match Palette Latency
            aim_data_s4     <= aim_data_s3;
            in_aim_s4       <= in_aim_s3;
            use_zombie_s4   <= use_zombie_s3;
            active_video_s4 <= active_video_s3;
			zombie_pixel_index_s4 <= i_zombie_pixel;
			bg_pixel_index_s4 <= bg_pixel_index_s3;
        end
    end


    // ============================================================
    //  LAYER 4: Palette Fetch (BRAM Access)
    // ============================================================

    // --- 4.1 Palette BRAM Instantiation ---
    logic [15:0] bg_rgb565_s5; // Returns in Layer 5
    logic [15:0] z_rgb565_s5;  // Returns in Layer 5

    // Note: BRAMs latch address on rising edge of Layer 4 input, data ready Layer 5
    background_palette bg_pal (
        .clock(i_clk), .address(bg_pixel_index_s4), 
        .q(bg_rgb565_s5), .data(0), .wren(0)
    );
    zombie_1x_palette z_pal (
        .clock(i_clk), .address(zombie_pixel_index_s4), 
        .q(z_rgb565_s5), .data(0), .wren(0)
    );

    // ------------------------------------------------------------
    // === PIPELINE REGISTER PASSING (Layer 4 -> Layer 5) ===
    // ------------------------------------------------------------
    logic [1:0] aim_data_s5;
    logic       in_aim_s5;
    logic       use_zombie_s5;
    logic       active_video_s5;

    always_ff @(posedge i_clk) begin
        if(~i_rst_n) begin
            active_video_s5 <= 0;
        end else begin
            // Delay again for Palette Latency
            aim_data_s5     <= aim_data_s4;
            in_aim_s5       <= in_aim_s4;
            use_zombie_s5   <= use_zombie_s4 && (zombie_pixel_index_s4 != 8'h00);
            active_video_s5 <= active_video_s4;
        end
    end


    // ============================================================
    //  LAYER 5: RGB Conversion & Mixing & Output
    // ============================================================

    // --- 5.1 Convert RGB565 to RGB888 ---
    logic [7:0] bg_r, bg_g, bg_b;
    logic [7:0] z_r, z_g, z_b;

    assign bg_r = {bg_rgb565_s5[15:11], bg_rgb565_s5[15:13]};
    assign bg_g = {bg_rgb565_s5[10:5],  bg_rgb565_s5[10:9]};
    assign bg_b = {bg_rgb565_s5[4:0],   bg_rgb565_s5[4:2]};

    assign z_r = {z_rgb565_s5[15:11], z_rgb565_s5[15:13]};
    assign z_g = {z_rgb565_s5[10:5],  z_rgb565_s5[10:9]};
    assign z_b = {z_rgb565_s5[4:0],   z_rgb565_s5[4:2]};

    // --- 5.2 Aim Color ---
    logic use_aim;
    logic [7:0] aim_r, aim_g, aim_b;

    assign use_aim = in_aim_s5 && (aim_data_s5 != 2'b00);
    assign aim_r = (aim_data_s5 == 2'b10) ? 8'd255 : 8'd0; // Red or Black
    assign aim_g = 8'd0;
    assign aim_b = 8'd0;

    // --- 5.3 Final Mixer ---
    logic [7:0] fin_r, fin_g, fin_b;

    always_comb begin
        if (use_aim) begin
            fin_r = aim_r; fin_g = aim_g; fin_b = aim_b;
        end else if (use_zombie_s5) begin
            fin_r = z_r;   fin_g = z_g;   fin_b = z_b;
        end else begin
            fin_r = bg_r;  fin_g = bg_g;  fin_b = bg_b;
        end
    end

    // ------------------------------------------------------------
    // === PIPELINE OUTPUT REGISTER (Layer 5 -> Output Pins) ===
    // ------------------------------------------------------------
    always_ff @(posedge i_clk) begin
        if (active_video_s5) begin
            o_vga_r <= fin_r;
            o_vga_g <= fin_g;
            o_vga_b <= fin_b;
        end else begin
            o_vga_r <= 0;
            o_vga_g <= 0;
            o_vga_b <= 0;
        end
    end

endmodule