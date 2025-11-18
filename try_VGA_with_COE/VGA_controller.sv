module VGA_controller (
	input logic vga_clk,
	input logic reset,
	output logic [9:0] DrawX, DrawY,
	output logic blank,
	output logic vga_hs, vga_vs
);

// VGA 640x480 @ 60Hz timing parameters
localparam H_VISIBLE = 10'd640;
localparam H_FRONT_PORCH = 10'd16;
localparam H_SYNC_PULSE = 10'd96;
localparam H_BACK_PORCH = 10'd48;
localparam H_TOTAL = H_VISIBLE + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH; // 800

localparam V_VISIBLE = 10'd480;
localparam V_FRONT_PORCH = 10'd10;
localparam V_SYNC_PULSE = 10'd2;
localparam V_BACK_PORCH = 10'd33;
localparam V_TOTAL = V_VISIBLE + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH; // 525

logic [9:0] h_count, v_count;

always_ff @ (posedge vga_clk) begin
	if (reset) begin
		h_count <= 10'd0;
		v_count <= 10'd0;
	end else begin
		if (h_count == H_TOTAL - 1) begin
			h_count <= 10'd0;
			if (v_count == V_TOTAL - 1) begin
				v_count <= 10'd0;
			end else begin
				v_count <= v_count + 10'd1;
			end
		end else begin
			h_count <= h_count + 10'd1;
		end
	end
end

// Horizontal sync: active low during sync pulse
assign vga_hs = ~((h_count >= H_VISIBLE + H_FRONT_PORCH) && 
                  (h_count < H_VISIBLE + H_FRONT_PORCH + H_SYNC_PULSE));

// Vertical sync: active low during sync pulse
assign vga_vs = ~((v_count >= V_VISIBLE + V_FRONT_PORCH) && 
                  (v_count < V_VISIBLE + V_FRONT_PORCH + V_SYNC_PULSE));

// Draw coordinates (only valid during visible area)
assign DrawX = (h_count < H_VISIBLE) ? h_count : 10'd0;
assign DrawY = (v_count < V_VISIBLE) ? v_count : 10'd0;

// Blank signal: active high when in visible area
assign blank = (h_count < H_VISIBLE) && (v_count < V_VISIBLE);

endmodule

