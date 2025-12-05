// VGA Image Overlay Module (Fully Pipelined & Parallelized)
// Optimization: Uses Speculative Execution in Stage 0 to calculate all 
// possible zombie addresses in parallel, removing math from the critical decision path.

module VGA_Image_Overlay_Combined #(
    parameter MAX_ZOMBIES = 3,       // Must match Random_Position_Generator
    parameter ZOMBIE_SIZE_X = 102,   // Zombie width
    parameter ZOMBIE_SIZE_Y = 149    // Zombie height
)(
    input logic i_clk,              // 74.25MHz VGA clock
    input logic i_rst_n,
    input logic [10:0] i_h_count,   // Horizontal position
    input logic [9:0] i_v_count,    // Vertical position
    input logic i_active_video,     // Active video region
    
    // Zombie positions inputs
    input logic [10:0] i_zombie_x [0:MAX_ZOMBIES-1],
    input logic [9:0] i_zombie_y [0:MAX_ZOMBIES-1],
    input logic i_zombie_valid   [0:MAX_ZOMBIES-1],
    input logic [10:0] i_zombie_size_x,
    input logic [9:0] i_zombie_size_y,
    
    // Background SRAM interface
    output logic [19:0] o_bg_sram_addr,
    output logic o_bg_sram_ce_n,
    inout  [15:0] io_bg_sram_dq,
    output logic o_bg_sram_lb_n,
    output logic o_bg_sram_oe_n,
    output logic o_bg_sram_ub_n,
    output logic o_bg_sram_we_n,
    
    // VGA RGB output
    output logic [7:0] o_vga_r,
    output logic [7:0] o_vga_g,
    output logic [7:0] o_vga_b
);

    localparam BG_WIDTH = 1280;

    // ============================================================
    // PIPELINE STAGE 0: Combinational Calculations (Parallel)
    // ============================================================
    // We calculate the address for EVERY zombie here combinationaly,
    // regardless of whether it is selected. This isolates the Multiplier 
    // from the sequential pipeline stage.

    // Combinational calculations (_w suffix)
    logic [20:0] bg_addr_w;
    logic [13:0] z_speculative_addr_w [0:MAX_ZOMBIES-1];
    logic        z_in_area_w          [0:MAX_ZOMBIES-1];
    
    // Background address calculation (combinational)
    assign bg_addr_w = (i_v_count * BG_WIDTH) + i_h_count;
    
    // Parallel zombie calculations (combinational)
    genvar z;
    generate
        for (z = 0; z < MAX_ZOMBIES; z++) begin : zombie_calc
            // Calculate local coordinates (combinational)
            logic [10:0] loc_x_w;
            logic [9:0]  loc_y_w;
            logic [10:0] zombie_x2_w;
            logic [9:0]  zombie_y2_w;
            
            assign loc_x_w = i_h_count - i_zombie_x[z];
            assign loc_y_w = i_v_count - i_zombie_y[z];
            
            // Calculate bounds (combinational)
            assign zombie_x2_w = i_zombie_x[z] + i_zombie_size_x;
            assign zombie_y2_w = i_zombie_y[z] + i_zombie_size_y;
            
            // Bounds check (combinational)
            assign z_in_area_w[z] = i_zombie_valid[z] && 
                                    (i_h_count >= i_zombie_x[z]) && 
                                    (i_h_count < zombie_x2_w) &&
                                    (i_v_count >= i_zombie_y[z]) && 
                                    (i_v_count < zombie_y2_w);
            
            // SPECULATIVE MATH: Calculate Address (combinational)
            // Doing this combinationaly removes the multiplier from the pipeline stage
            assign z_speculative_addr_w[z] = (loc_y_w * i_zombie_size_x) + loc_x_w;
        end
    endgenerate

    // Pipeline Stage 0: Register combinational outputs
    logic [20:0] bg_addr_p0;
    logic [13:0] z_speculative_addr_p0 [0:MAX_ZOMBIES-1];
    logic        z_in_area_p0          [0:MAX_ZOMBIES-1];
    logic        active_video_p0;

    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            bg_addr_p0 <= 21'd0;
            active_video_p0 <= 1'b0;
            for(int k=0; k<MAX_ZOMBIES; k++) begin
                z_speculative_addr_p0[k] <= 14'd0;
                z_in_area_p0[k] <= 1'b0;
            end
        end else begin
            // Register all combinational calculations
            bg_addr_p0 <= bg_addr_w;
            active_video_p0 <= i_active_video;
            for(int k=0; k<MAX_ZOMBIES; k++) begin
                z_speculative_addr_p0[k] <= z_speculative_addr_w[k];
                z_in_area_p0[k] <= z_in_area_w[k];
            end
        end
    end

    // ============================================================
    // PIPELINE STAGE 1: Selection (Priority Mux)
    // ============================================================
    // Now we just select the winner. No heavy math here.
    
    logic [13:0] zombie_bram_addr_p1;
    logic [20:0] bg_addr_p1;
    logic        z_hit_p1;
    logic        active_video_p1;
    
    logic [13:0] z_addr_selected_w; // Combinational Mux Output
    logic        z_hit_w;           // Combinational Hit Output

    always_comb begin
        // Priority Encoder (Lowest index = Highest Priority)
        z_addr_selected_w = 14'd0;
        z_hit_w = 1'b0;

        if (z_in_area_p0[0]) begin
            z_hit_w = 1'b1;
            z_addr_selected_w = z_speculative_addr_p0[0];
        end else if (z_in_area_p0[1]) begin
            z_hit_w = 1'b1;
            z_addr_selected_w = z_speculative_addr_p0[1];
        end else if (z_in_area_p0[2]) begin
            z_hit_w = 1'b1;
            z_addr_selected_w = z_speculative_addr_p0[2];
        end
    end

    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            zombie_bram_addr_p1 <= 14'd0;
            bg_addr_p1 <= 21'd0;
            z_hit_p1 <= 1'b0;
            active_video_p1 <= 1'b0;
        end else begin
            zombie_bram_addr_p1 <= z_addr_selected_w;
            z_hit_p1            <= z_hit_w;
            bg_addr_p1          <= bg_addr_p0;
            active_video_p1     <= active_video_p0;
        end
    end

    // ============================================================
    // MEMORY INTERFACE (Driven by Stage 1 Registers)
    // ============================================================
    
    // SRAM Controls
    assign o_bg_sram_ce_n = 1'b0;
    assign o_bg_sram_oe_n = 1'b0;
    assign o_bg_sram_we_n = 1'b1; // Read Mode
    assign o_bg_sram_addr = bg_addr_p1[20:1]; // Word address
    assign o_bg_sram_lb_n = bg_addr_p1[0];    // Low byte active low
    assign o_bg_sram_ub_n = ~bg_addr_p1[0];   // High byte active low
    assign io_bg_sram_dq  = 16'hZZZZ;         // High-Z for reading

    // Internal BRAM Wires
    logic [15:0] bg_palette_data;
    logic [7:0]  zombie_pixel_index;
    logic [15:0] zombie_palette_data;
    logic [7:0]  bg_palette_addr_reg;
    logic [7:0]  zombie_palette_addr_reg;

    // Zombie Image BRAM Instance
    // Reads address from Stage 1, Data available in Stage 2
    zombie_1x zombie_bram(
        .address(zombie_bram_addr_p1),
        .clock(i_clk),
        .data(8'd0),
        .wren(1'b0),
        .q(zombie_pixel_index)
    );

    // ============================================================
    // PIPELINE STAGE 2: Latch Data & Prepare Palette Lookup
    // ============================================================
    
    logic [15:0] sram_dq_p2;
    logic        bg_addr_lsb_p2;
    logic        z_hit_p2;
    logic        active_video_p2;
    
    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            sram_dq_p2 <= 16'd0;
            bg_addr_lsb_p2 <= 1'b0;
            z_hit_p2 <= 1'b0;
            active_video_p2 <= 1'b0;
        end else begin
            // Latch SRAM Data (Data valid 1 cycle after address update in Stage 1)
            sram_dq_p2 <= io_bg_sram_dq; 
            
            // Store LSB to select upper/lower byte later
            bg_addr_lsb_p2 <= bg_addr_p1[0];

            // Pipeline controls
            z_hit_p2 <= z_hit_p1;
            active_video_p2 <= active_video_p1;
        end
    end

    // Extract Background Index (Combinational logic before Stage 3 Register)
    logic [7:0] bg_pixel_index_w;
    assign bg_pixel_index_w = (bg_addr_lsb_p2) ? sram_dq_p2[15:8] : sram_dq_p2[7:0];

    // ============================================================
    // PIPELINE STAGE 3: Palette Address Setup
    // ============================================================
    
    logic z_hit_p3;
    logic active_video_p3;
    logic [7:0] zombie_index_p3; // Need to keep index for transparency check (0x00)

    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            bg_palette_addr_reg <= 8'd0;
            zombie_palette_addr_reg <= 8'd0;
            z_hit_p3 <= 1'b0;
            active_video_p3 <= 1'b0;
            zombie_index_p3 <= 8'd0;
        end else begin
            // Register addresses for Palette BRAMs
            bg_palette_addr_reg <= bg_pixel_index_w;
            zombie_palette_addr_reg <= zombie_pixel_index; // Output from Zombie BRAM

            // Pipeline controls
            z_hit_p3 <= z_hit_p2;
            active_video_p3 <= active_video_p2;
            zombie_index_p3 <= zombie_pixel_index;
        end
    end

    // Palette BRAM Instances
    // Read address from Stage 3, Data available in Stage 4
    background_palette bg_palette_bram(
        .address(bg_palette_addr_reg),
        .clock(i_clk),
        .data(16'd0),
        .wren(1'b0),
        .q(bg_palette_data)
    );
    
    zombie_1x_palette zombie_palette_bram(
        .address(zombie_palette_addr_reg),
        .clock(i_clk),
        .data(16'd0),
        .wren(1'b0),
        .q(zombie_palette_data)
    );

    // ============================================================
    // PIPELINE STAGE 4: Final Output (RGB Conversion)
    // ============================================================
    
    always_ff @(posedge i_clk) begin
        if (~i_rst_n) begin
            o_vga_r <= 8'd0;
            o_vga_g <= 8'd0;
            o_vga_b <= 8'd0;
        end else begin
            if (active_video_p3) begin
                // Check: 1. Are we in a zombie box? 2. Is the pixel NOT transparent (0x00)?
                if (z_hit_p3 && (zombie_index_p3 != 8'h00)) begin
                    // Output Zombie Color (RGB565 -> RGB888)
                    o_vga_r <= {zombie_palette_data[15:11], zombie_palette_data[15:13]};
                    o_vga_g <= {zombie_palette_data[10:5],  zombie_palette_data[10:9]};
                    o_vga_b <= {zombie_palette_data[4:0],   zombie_palette_data[4:2]};
                end else begin
                    // Output Background Color (RGB565 -> RGB888)
                    o_vga_r <= {bg_palette_data[15:11], bg_palette_data[15:13]};
                    o_vga_g <= {bg_palette_data[10:5],  bg_palette_data[10:9]};
                    o_vga_b <= {bg_palette_data[4:0],   bg_palette_data[4:2]};
                end
            end else begin
                // Blanking region
                o_vga_r <= 8'd0;
                o_vga_g <= 8'd0;
                o_vga_b <= 8'd0;
            end
        end
    end

endmodule