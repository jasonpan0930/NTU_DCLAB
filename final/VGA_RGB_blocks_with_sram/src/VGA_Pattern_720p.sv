// VGA Pattern Generator for 720p (1280x720)
// Divides screen into 3x3 grid with RGB pattern
// SW[0,1,2] control R, G, B enable
module VGA_Pattern_720p (
	input i_clk,
	input i_rst_n,
	input [10:0] i_h_count,  // Horizontal position (0-1279)
	input [9:0] i_v_count,   // Vertical position (0-719)
	input i_active_video,    // Active video region
	input [2:0] i_rgb_enable, // SW[0]=R, SW[1]=G, SW[2]=B
	
	output [7:0] o_vga_r,
	output [7:0] o_vga_g,
	output [7:0] o_vga_b
);

// Screen dimensions
parameter H_DISPLAY = 1280;
parameter V_DISPLAY = 720;

// Grid dimensions (3x3)
parameter GRID_H = H_DISPLAY / 3;  // 426 pixels per grid horizontally (approx)
parameter GRID_V = V_DISPLAY / 3;  // 240 pixels per grid vertically

// Grid calculation
wire [1:0] grid_col_accurate;
wire [1:0] grid_row_accurate;

assign grid_col_accurate = (i_h_count < GRID_H) ? 2'd0 : 
                          (i_h_count < (2*GRID_H)) ? 2'd1 : 2'd2;
                          
assign grid_row_accurate = (i_v_count < GRID_V) ? 2'd0 : 
                          (i_v_count < (2*GRID_V)) ? 2'd1 : 2'd2;

// Color pattern:
// Row 0: R, G, B
// Row 1: G, B, R
// Row 2: B, R, G
reg [7:0] r_val, g_val, b_val;

always_comb begin
	r_val = 8'd0;
	g_val = 8'd0;
	b_val = 8'd0;
	
	if (i_active_video) begin
		case (grid_row_accurate)
			2'd0: begin // First row: RGB
				case (grid_col_accurate)
					2'd0: begin r_val = 8'd255; g_val = 8'd0;   b_val = 8'd0;   end // Red
					2'd1: begin r_val = 8'd0;   g_val = 8'd255; b_val = 8'd0;   end // Green
					2'd2: begin r_val = 8'd0;   g_val = 8'd0;   b_val = 8'd255; end // Blue
				endcase
			end
			2'd1: begin // Second row: GBR
				case (grid_col_accurate)
					2'd0: begin r_val = 8'd0;   g_val = 8'd255; b_val = 8'd0;   end // Green
					2'd1: begin r_val = 8'd0;   g_val = 8'd0;   b_val = 8'd255; end // Blue
					2'd2: begin r_val = 8'd255; g_val = 8'd0;   b_val = 8'd0;   end // Red
				endcase
			end
			2'd2: begin // Third row: BRG
				case (grid_col_accurate)
					2'd0: begin r_val = 8'd0;   g_val = 8'd0;   b_val = 8'd255; end // Blue
					2'd1: begin r_val = 8'd255; g_val = 8'd0;   b_val = 8'd0;   end // Red
					2'd2: begin r_val = 8'd0;   g_val = 8'd255; b_val = 8'd0;   end // Green
				endcase
			end
		endcase
	end
end

// Apply RGB enable switches
assign o_vga_r = r_val & {8{i_rgb_enable[0]}};
assign o_vga_g = g_val & {8{i_rgb_enable[1]}};
assign o_vga_b = b_val & {8{i_rgb_enable[2]}};

endmodule

