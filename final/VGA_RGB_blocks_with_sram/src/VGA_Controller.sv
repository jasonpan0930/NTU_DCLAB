// VGA Controller for 640x480@60Hz
// Uses external 25MHz clock from PLL
module VGA_Controller (
	input i_clk,      // 25MHz VGA clock (from PLL)
	input i_rst_n,
	output o_vga_clk, // 25MHz VGA clock (pass through)
	output o_hsync,
	output o_vsync,
	output o_blank_n,
	output o_sync_n,
	output [9:0] o_h_count, // Horizontal counter (0-799)
	output [9:0] o_v_count, // Vertical counter (0-524)
	output o_active_video   // Active video region
);

// VGA 640x480@60Hz timing parameters
parameter H_DISPLAY = 640;
parameter H_FRONT_PORCH = 16;
parameter H_SYNC_PULSE = 96;
parameter H_BACK_PORCH = 48;
parameter H_TOTAL = H_DISPLAY + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH; // 800

parameter V_DISPLAY = 480;
parameter V_FRONT_PORCH = 10;
parameter V_SYNC_PULSE = 2;
parameter V_BACK_PORCH = 33;
parameter V_TOTAL = V_DISPLAY + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH; // 525

// Pass through the VGA clock
assign o_vga_clk = i_clk;

// Horizontal counter
reg [9:0] h_count;
reg [9:0] v_count;

always_ff @(posedge i_clk or negedge i_rst_n) begin
	if (!i_rst_n) begin
		h_count <= 10'd0;
		v_count <= 10'd0;
	end else begin
		if (h_count == H_TOTAL - 1) begin
			h_count <= 10'd0;
			if (v_count == V_TOTAL - 1)
				v_count <= 10'd0;
			else
				v_count <= v_count + 10'd1;
		end else begin
			h_count <= h_count + 10'd1;
		end
	end
end

// Horizontal sync signal
assign o_hsync = (h_count >= H_DISPLAY + H_FRONT_PORCH) && 
                 (h_count < H_DISPLAY + H_FRONT_PORCH + H_SYNC_PULSE) ? 1'b0 : 1'b1;

// Vertical sync signal
assign o_vsync = (v_count >= V_DISPLAY + V_FRONT_PORCH) && 
                 (v_count < V_DISPLAY + V_FRONT_PORCH + V_SYNC_PULSE) ? 1'b0 : 1'b1;

// Active video region
assign o_active_video = (h_count < H_DISPLAY) && (v_count < V_DISPLAY);

// Blank and sync signals
assign o_blank_n = o_active_video;
assign o_sync_n = 1'b0; // Low for standard VGA

// Output pixel coordinates
assign o_h_count = h_count;
assign o_v_count = v_count;

endmodule

