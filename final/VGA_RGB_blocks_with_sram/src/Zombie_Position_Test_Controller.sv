// Zombie Position Test Controller
// Controls zombie position using switches for testing
// SW[0]: move right (increase x1)
// SW[1]: move left (decrease x1)
// SW[17]: move down (increase y1)
// SW[16]: move up (decrease y1)
// Movement speed: ~128 pixels per second
module Zombie_Position_Test_Controller (
	input logic i_clk,              // Clock (74.25MHz VGA clock)
	input logic i_rst_n,            // Reset (active low)
	
	// Control switches
	input logic i_sw_right,         // SW[0]: move right
	input logic i_sw_left,          // SW[1]: move left
	input logic i_sw_down,          // SW[17]: move down
	input logic i_sw_up,            // SW[16]: move up
	
	// Zombie size (for boundary checking)
	input logic [10:0] i_zombie_size_x,  // Zombie width
	input logic [9:0] i_zombie_size_y,   // Zombie height
	
	// Screen dimensions (for boundary checking)
	input logic [10:0] i_screen_width,   // Screen width (1280)
	input logic [9:0] i_screen_height,  // Screen height (720)
	
	// Output position
	output logic [10:0] o_zombie_x1,     // Zombie top-left X position
	output logic [9:0] o_zombie_y1       // Zombie top-left Y position
);

	// Clock divider for movement (74.25MHz / 580078 ≈ 128 Hz, so 128 movements per second)
	// 74,250,000 / 128 ≈ 580,078 cycles per movement
	localparam MOVEMENT_DIVIDER = 26'd580078;  // Approximately 128 movements per second
	
	logic [25:0] movement_counter;
	logic movement_tick;
	
	// Clock divider: generate movement tick
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			movement_counter <= 26'd0;
			movement_tick <= 1'b0;
		end else begin
			if (movement_counter >= MOVEMENT_DIVIDER - 1) begin
				movement_counter <= 26'd0;
				movement_tick <= 1'b1;
			end else begin
				movement_counter <= movement_counter + 26'd1;
				movement_tick <= 1'b0;
			end
		end
	end
	
	// Zombie position registers
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			o_zombie_x1 <= 11'd0;
			o_zombie_y1 <= 10'd0;
		end else if (movement_tick) begin
			// X position control (i_sw_right and i_sw_left)
			if (i_sw_right && !i_sw_left) begin
				// Move right (increase x1)
				if (o_zombie_x1 < (i_screen_width - i_zombie_size_x)) begin
					o_zombie_x1 <= o_zombie_x1 + 11'd1;
				end
			end else if (i_sw_left && !i_sw_right) begin
				// Move left (decrease x1)
				if (o_zombie_x1 > 11'd0) begin
					o_zombie_x1 <= o_zombie_x1 - 11'd1;
				end
			end
			
			// Y position control (i_sw_down and i_sw_up)
			if (i_sw_down && !i_sw_up) begin
				// Move down (increase y1)
				if (o_zombie_y1 < (i_screen_height - i_zombie_size_y)) begin
					o_zombie_y1 <= o_zombie_y1 + 10'd1;
				end
			end else if (i_sw_up && !i_sw_down) begin
				// Move up (decrease y1)
				if (o_zombie_y1 > 10'd0) begin
					o_zombie_y1 <= o_zombie_y1 - 10'd1;
				end
			end
		end
	end

endmodule

