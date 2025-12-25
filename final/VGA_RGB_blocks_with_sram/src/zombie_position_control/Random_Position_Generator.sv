// Random Position Generator (Optimized / Timing Safe / Syntax Fixed)
module Random_Position_Generator #(
	parameter SCREEN_WIDTH = 1280,
	parameter SCREEN_HEIGHT = 720,
	parameter TARGET_X = 640,
	parameter TARGET_Y = 719,
	parameter MAX_POSITIONS = 20,
	parameter VELOCITY = 16'd2,
	parameter CLOCK_FREQ = 32'd74250000,
	// Base update rate for zombie movement (120 Hz allows more granular speed steps)
	parameter UPDATE_RATE = 32'd120,
	parameter GENERATION_RATE = 32'd1
)(
	input logic i_clk,
	input logic i_rst_n,
	
	// Game state: zombies only move when game is started
	input logic i_game_started,
	
	// Kill count: used to adjust zombie base speed (0-100)
	input logic [7:0] i_kill_count,
	
	// Kill signal: when i_kill=1, zombie at i_kill_index dies
	input logic i_kill,
	input logic [4:0] i_kill_index,
	
	output logic [10:0] o_x [0:MAX_POSITIONS-1],
	output logic [9:0] o_y [0:MAX_POSITIONS-1],
	output logic o_valid [0:MAX_POSITIONS-1],
	output logic [31:0] o_distance [0:MAX_POSITIONS-1],
	output logic [3:0] o_active_count
);

	// ============================================================
	// Internal State & Signals
	// ============================================================
	logic signed [31:0] pos_x [0:MAX_POSITIONS-1];
	logic signed [31:0] pos_y [0:MAX_POSITIONS-1];
	logic position_valid [0:MAX_POSITIONS-1];
	
	// Error Accumulator for Bresenham movement
	logic signed [31:0] error_acc [0:MAX_POSITIONS-1];
	
	// Per-zombie speed control based on Y position
	// Use 4-bit divider to support more speed steps (2-15 divider range)
	logic [3:0] move_div_cnt [0:MAX_POSITIONS-1];
	logic [3:0] move_div_val [0:MAX_POSITIONS-1];
	logic move_enable [0:MAX_POSITIONS-1]; 

	// Timing Counters
	// Base update rate: 60 Hz (can be divided to get 15, 20, 30 Hz per zombie)
	localparam BASE_UPDATE_RATE = 32'd60;
	localparam UPDATE_DIVIDER = CLOCK_FREQ / BASE_UPDATE_RATE;
	localparam GEN_DIVIDER = CLOCK_FREQ / GENERATION_RATE;
	logic [23:0] update_counter;
	logic update_tick;
	logic [31:0] gen_counter;
	logic gen_tick;

	// Random Number Generator (LFSR)
	logic [31:0] lfsr_state;
	
	// Helper signals
	logic [3:0] available_slot;
	logic slot_found;

	// ============================================================
	// Clock Dividers & LFSR
	// ============================================================
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			update_counter <= 24'd0;
			update_tick <= 1'b0;
			gen_counter <= 32'd0;
			gen_tick <= 1'b0;
			lfsr_state <= 32'hACE1;
		end else begin
			// Update Tick (60Hz)
			if (update_counter >= UPDATE_DIVIDER - 1) begin
				update_counter <= 24'd0;
				update_tick <= 1'b1;
			end else begin
				update_counter <= update_counter + 24'd1;
				update_tick <= 1'b0;
			end

			// Generation Tick (1Hz)
			if (gen_counter >= GEN_DIVIDER - 1) begin
				gen_counter <= 32'd0;
				gen_tick <= 1'b1;
			end else begin
				gen_counter <= gen_counter + 32'd1;
				gen_tick <= 1'b0;
			end

			// LFSR Advance
			if (gen_tick || update_tick) begin
				lfsr_state <= {lfsr_state[30:0], lfsr_state[31] ^ lfsr_state[21] ^ lfsr_state[1] ^ lfsr_state[0]};
			end
		end
	end

	// Find available slot
	always_comb begin
		slot_found = 1'b0;
		available_slot = 4'd0;
		for (int k = 0; k < MAX_POSITIONS; k++) begin
			if (!slot_found && !position_valid[k]) begin
				slot_found = 1'b1;
				available_slot = k[3:0];
			end
		end
	end

	// ============================================================
	// Main Movement Logic
	// ============================================================
	
	genvar i;
	generate
		for (i = 0; i < MAX_POSITIONS; i++) begin : pos_logic
			
			// Temporary variables declarations must be OUTSIDE the procedural block or at the top
			logic signed [31:0] dx, dy, abs_dx, abs_dy;
			logic signed [31:0] target_x_fixed;
			logic signed [31:0] target_y_fixed;
			
			// Use assign for constants inside generate to be safe
			assign target_x_fixed = TARGET_X;
			assign target_y_fixed = TARGET_Y;
			
			// Speed multiplier based on kill count thresholds
			// Thresholds: 20, 30, 50, 70, 90
			// Smaller multiplier = faster speed (reduces divider value)
			logic [3:0] base_speed_multiplier;
			logic [3:0] base_div_val;
			logic [7:0] scaled_div_val;
			
			always_comb begin
				dx = target_x_fixed - pos_x[i];
				dy = target_y_fixed - pos_y[i];
				abs_dx = (dx >= 0) ? dx : -dx;
				abs_dy = (dy >= 0) ? dy : -dy;
				
				// Determine base speed multiplier based on kill count
				// 0-19 kills: multiplier = 10 (normal speed)
				// 20-29 kills: multiplier = 9 (10% faster)
				// 30-49 kills: multiplier = 8 (20% faster)
				// 50-69 kills: multiplier = 7 (30% faster)
				// 70-89 kills: multiplier = 6 (40% faster)
				// 90+ kills: multiplier = 5 (50% faster)
				if (i_kill_count < 8'd20) begin
					base_speed_multiplier = 4'd10;
				end else if (i_kill_count < 8'd30) begin
					base_speed_multiplier = 4'd9;
				end else if (i_kill_count < 8'd50) begin
					base_speed_multiplier = 4'd8;
				end else if (i_kill_count < 8'd70) begin
					base_speed_multiplier = 4'd7;
				end else if (i_kill_count < 8'd90) begin
					base_speed_multiplier = 4'd6;
				end else begin
					base_speed_multiplier = 4'd5;
				end
				
				// Calculate base divider value based on Y position
				// Base rate: 120 Hz
				// Zombies spawn at Y=250, target at Y=719
				// SLOW at beginning (small Y), FAST as they approach target (large Y)
				// Larger divider = slower speed (120/divider = Hz)
				if (pos_y[i] < 32'd280) begin
					base_div_val = 4'd10; // 120/10 = 12 Hz (slowest - just spawned)
				end else if (pos_y[i] < 32'd320) begin
					base_div_val = 4'd8;  // 120/8 = 15 Hz
				end else if (pos_y[i] < 32'd360) begin
					base_div_val = 4'd6;  // 120/6 = 20 Hz
				end else if (pos_y[i] < 32'd400) begin
					base_div_val = 4'd5;  // 120/5 = 24 Hz
				end else if (pos_y[i] < 32'd480) begin
					base_div_val = 4'd4;  // 120/4 = 30 Hz
				end else if (pos_y[i] < 32'd560) begin
					base_div_val = 4'd3;  // 120/3 = 40 Hz (faster)
				end else begin
					base_div_val = 4'd2;  // 120/2 = 60 Hz (fastest - near target)
				end
				
				// Apply speed multiplier: multiply base_div_val by multiplier/10
				// This scales the speed based on kill count
				// Result is clamped to minimum of 2 (fastest) and maximum of 10 (slowest)
				scaled_div_val = (base_div_val * base_speed_multiplier) / 4'd10;
				if (scaled_div_val < 4'd2) begin
					move_div_val[i] = 4'd2;  // Minimum divider (fastest)
				end else if (scaled_div_val > 4'd10) begin
					move_div_val[i] = 4'd10; // Maximum divider (slowest)
				end else begin
					move_div_val[i] = scaled_div_val[3:0];
				end
				
				// Enable movement when counter matches divider value AND game is started
				move_enable[i] = update_tick && position_valid[i] && i_game_started &&
				                 (move_div_cnt[i] == move_div_val[i]);
			end

			always_ff @(posedge i_clk or negedge i_rst_n) begin
				if (!i_rst_n) begin
					pos_x[i] <= 32'(SCREEN_WIDTH / 2);
					pos_y[i] <= 32'(SCREEN_HEIGHT / 2);
					position_valid[i] <= 1'b0;
					error_acc[i] <= 32'd0;
					move_div_cnt[i] <= 4'd0;
				end else begin
					
					// Update per-zombie divider counter (independent update, runs every update_tick)
					// Skip counter update if zombie is being killed, spawned, or moved
					// Also skip if game is not started (zombies don't move in start screen)
					if (update_tick && position_valid[i] && i_game_started &&
					    !(i_kill && (i_kill_index == i[4:0])) &&
					    !(gen_tick && (available_slot == i) && slot_found) &&
					    !move_enable[i]) begin
						if (move_div_cnt[i] >= move_div_val[i]) begin
							move_div_cnt[i] <= 4'd0;
						end else begin
							move_div_cnt[i] <= move_div_cnt[i] + 4'd1;
						end
					end
					
					// 0. KILL ZOMBIE (highest priority: when kill signal arrives, zombie dies)
					//    Generate next spawn position when killed (same as arrival behavior)
					if (i_kill && (i_kill_index == i[4:0])) begin
						position_valid[i] <= 1'b0; // Zombie dies - disappears
						pos_x[i] <= (lfsr_state[10:0] % SCREEN_WIDTH); // Generate and store random X position for next spawn
						pos_y[i] <= 32'd300; // Set spawn Y position
						error_acc[i] <= 32'd0;
						move_div_cnt[i] <= 4'd0;
					end
					// 1. SPAWN NEW ZOMBIE (priority: when gen_tick arrives, make zombie visible)
					//    Position was already generated and fixed when previous zombie disappeared/killed
					//    (or will be generated here if first spawn after reset)
					//    Now we just make it visible (position_valid=1) - position is already stable
					else if (gen_tick && (available_slot == i) && slot_found) begin
						// If position is at reset value (screen center), generate new random position
						// Otherwise, position was pre-generated when zombie disappeared - use it
						if (pos_x[i] == 32'(SCREEN_WIDTH / 2) && pos_y[i] == 32'(SCREEN_HEIGHT / 2)) begin
							// First spawn after reset - generate random position now
							pos_x[i] <= (lfsr_state[10:0] % SCREEN_WIDTH);
							pos_y[i] <= 32'd270;
						end
						// Position is already set (either pre-generated or just set above) - make visible
						position_valid[i] <= 1'b1;  // Zombie becomes visible at stable position
						error_acc[i] <= 32'd0;
						move_div_cnt[i] <= 4'd0;  // Reset counter: ensures no immediate movement,
						//                         zombie stays at spawn position until move_div_cnt reaches move_div_val
					end 
					// 2. MOVE ZOMBIE (Bresenham-like Logic with variable speed)
					//    Only executes if not spawning in this cycle
					else if (move_enable[i]) begin
						// Reset counter after movement is enabled
						move_div_cnt[i] <= 4'd0;
						
						// Check arrival
						if ((abs_dx + abs_dy) <= VELOCITY) begin
							// Zombie arrives at destination: disappear and generate next spawn position
							// Generate random position NOW (when zombie disappears) and store it
							// This ensures position is fixed and stable before zombie becomes visible
							position_valid[i] <= 1'b0; // Disappear - slot becomes available for next gen_tick
							pos_x[i] <= (lfsr_state[10:0] % SCREEN_WIDTH); // Generate and store random X position
							pos_y[i] <= 32'd270; // Set spawn Y position
							// Position is now fixed and waiting - won't change until zombie becomes valid
						end else begin
							if (abs_dx >= abs_dy) begin
								// Move X (Major)
								pos_x[i] <= (dx > 0) ? (pos_x[i] + VELOCITY) : (pos_x[i] - VELOCITY);
								// Check Y error
								if ((error_acc[i] + abs_dy) >= abs_dx) begin
									pos_y[i] <= (dy > 0) ? (pos_y[i] + VELOCITY) : (pos_y[i] - VELOCITY);
									error_acc[i] <= (error_acc[i] + abs_dy) - abs_dx;
								end else begin
									error_acc[i] <= error_acc[i] + abs_dy;
								end
							end else begin
								// Move Y (Major)
								pos_y[i] <= (dy > 0) ? (pos_y[i] + VELOCITY) : (pos_y[i] - VELOCITY);
								// Check X error
								if ((error_acc[i] + abs_dx) >= abs_dy) begin
									pos_x[i] <= (dx > 0) ? (pos_x[i] + VELOCITY) : (pos_x[i] - VELOCITY);
									error_acc[i] <= (error_acc[i] + abs_dx) - abs_dy;
								end else begin
									error_acc[i] <= error_acc[i] + abs_dx;
								end
							end
						end
					end
				end
			end
		end
	endgenerate

	// ============================================================
	// Outputs
	// ============================================================
	
	// Active Count
	always_comb begin
		o_active_count = 4'd0;
		for (int k = 0; k < MAX_POSITIONS; k++) begin
			if (position_valid[k]) o_active_count = o_active_count + 4'd1;
		end
	end

	// Position and Distance Output
	generate
		for (i = 0; i < MAX_POSITIONS; i++) begin : pos_out
			always_comb begin
				// FIX: Declarations moved to the very top of the block
				logic signed [31:0] d_out_x;
				logic signed [31:0] d_out_y;
				
				// Initialize them to avoid latch warnings (good practice)
				d_out_x = 32'd0;
				d_out_y = 32'd0;

				if (!position_valid[i]) begin
					o_x[i] = TARGET_X[10:0];
					o_y[i] = TARGET_Y[9:0];
					o_valid[i] = 1'b0;
					o_distance[i] = 32'd0;
				end else begin
					// Clamp X
					if (pos_x[i] < 0) o_x[i] = 11'd0;
					else if (pos_x[i] >= SCREEN_WIDTH) o_x[i] = SCREEN_WIDTH[10:0] - 11'd1;
					else o_x[i] = pos_x[i][10:0];

					// Clamp Y
					if (pos_y[i] < 0) o_y[i] = 10'd0;
					else if (pos_y[i] >= SCREEN_HEIGHT) o_y[i] = SCREEN_HEIGHT[9:0] - 10'd1;
					else o_y[i] = pos_y[i][9:0];

					o_valid[i] = 1'b1;
					
					// Recalculate distance for output
					d_out_x = TARGET_X - pos_x[i];
					d_out_y = TARGET_Y - pos_y[i];
					o_distance[i] = ((d_out_x>=0)?d_out_x:-d_out_x) + ((d_out_y>=0)?d_out_y:-d_out_y);
				end
			end
		end
	endgenerate

endmodule