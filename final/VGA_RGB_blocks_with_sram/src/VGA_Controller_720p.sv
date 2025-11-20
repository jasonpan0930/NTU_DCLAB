// VGA Controller for 1280x720@60Hz (720p)
// Uses external 74.25MHz clock from PLL
module VGA_Controller_720p (
    input i_clk,      // 74.25MHz VGA clock (from PLL)
    input i_rst_n,
    output o_vga_clk, // 74.25MHz VGA clock (pass through)
    output o_hsync,
    output o_vsync,
    output o_blank_n,
    output o_sync_n,
    output [10:0] o_h_count, // Horizontal pixel coordinate (0-1279)
    output [9:0] o_v_count,  // Vertical pixel coordinate (0-719)
    output o_active_video    // Active video region
);

// VGA 1280x720@60Hz timing parameters
parameter H_DISPLAY    = 1280;
parameter H_FRONT_PORCH = 110;
parameter H_SYNC_PULSE  = 40;
parameter H_BACK_PORCH  = 220;
parameter H_TOTAL       = H_DISPLAY + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH; // 1650

parameter V_DISPLAY    = 720;
parameter V_FRONT_PORCH = 5;
parameter V_SYNC_PULSE  = 5;
parameter V_BACK_PORCH  = 20;
parameter V_TOTAL       = V_DISPLAY + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH; // 750

// --- 輔助時序點參數 ---
parameter H_SYNC_START = H_BACK_PORCH + H_DISPLAY + H_FRONT_PORCH; // 1610
parameter V_SYNC_START = V_BACK_PORCH + V_DISPLAY + V_FRONT_PORCH; // 745

// Pass through the VGA clock
assign o_vga_clk = i_clk;

// Sequential logic: registers (current values, r)
reg [10:0] h_count_r;
reg [9:0] v_count_r;
reg [10:0] h_active_count_r;
reg [9:0] v_active_count_r;

// Combinational logic: next state calculation (write values, w)
logic [10:0] h_count_w;
logic [9:0] v_count_w;
logic [10:0] h_active_count_w; // BRAM/Active Address Request
logic [9:0] v_active_count_w;  // BRAM/Active Address Request

// Calculate next cycle's counters (combinational)
always_comb begin
    // 1. 計算下一週期 H 總計數 (h_count_w)
    if (h_count_r == H_TOTAL - 1) begin
        h_count_w = 11'd0;
    end else begin
        h_count_w = h_count_r + 11'd1;
    end
    
    // 2. 計算下一週期 V 總計數 (v_count_w)
    if (h_count_r == H_TOTAL - 1) begin
        if (v_count_r == V_TOTAL - 1) begin
            v_count_w = 10'd0;
        end else begin
            v_count_w = v_count_r + 10'd1;
        end
    end else begin
        v_count_w = v_count_r;
    end
    
    // 3. BRAM 地址請求/Active 座標 (h_active_count_w, v_active_count_w)
    //    *** 關鍵修正：基於當前值 h_count_r 計算，實現 BRAM 延遲補償 ***
    //    T 時計算 T 週期座標 (h_active_count_w)，在 T+1 週期輸出給 VGA (h_active_count_r)
    
    // H 座標計算
    if (h_count_r >= H_BACK_PORCH && h_count_r < H_BACK_PORCH + H_DISPLAY) begin
        h_active_count_w = h_count_r - H_BACK_PORCH;
    end else begin
        h_active_count_w = 11'd0;
    end
    
    // V 座標計算
    if (v_count_r >= V_BACK_PORCH && v_count_r < V_BACK_PORCH + V_DISPLAY) begin
        v_active_count_w = v_count_r - V_BACK_PORCH;
    end else begin
        v_active_count_w = 10'd0;
    end
end

// Sequential logic: register updates
always_ff @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        h_count_r <= 11'd0;
        v_count_r <= 10'd0;
        h_active_count_r <= 11'd0;
        v_active_count_r <= 10'd0;
    end else begin
        h_count_r <= h_count_w;
        v_count_r <= v_count_w;
        
        // BRAM Address / Active Coordinate Pipeline
        h_active_count_r <= h_active_count_w; // T+1 輸出的 VGA 座標 = T 時計算出的地址
        v_active_count_r <= v_active_count_w;
    end
end

// --- VGA 輸出訊號 ---

// Horizontal sync signal (使用標準的 H_SYNC_START)
assign o_hsync = (h_count_r >= H_SYNC_START) && 
                 (h_count_r < H_SYNC_START + H_SYNC_PULSE) ? 1'b0 : 1'b1;

// Vertical sync signal (使用標準的 V_SYNC_START)
assign o_vsync = (v_count_r >= V_SYNC_START) && 
                 (v_count_r < V_SYNC_START + V_SYNC_PULSE) ? 1'b0 : 1'b1;

// Active video region
assign o_active_video = (h_count_r >= H_BACK_PORCH) && (h_count_r < H_BACK_PORCH + H_DISPLAY) && (v_count_r >= V_BACK_PORCH) && (v_count_r < V_BACK_PORCH + V_DISPLAY);

// Blank and sync signals
assign o_blank_n = o_active_video;
assign o_sync_n = 1'b0; // Low for standard VGA

// Output pixel coordinates
assign o_h_count = h_active_count_r; // 此訊號應連接到 BRAM 的 Address 埠
assign o_v_count = v_active_count_r; // 此訊號應連接到 BRAM 的 Address 埠

endmodule