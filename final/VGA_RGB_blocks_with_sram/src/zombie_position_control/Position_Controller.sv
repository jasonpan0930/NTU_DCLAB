// Position Controller
// Integrates angular velocity inputs to compute screen position (x, y) for aim/cursor display
//
// Coordinate system (when facing the screen):
//   +y = forward (toward screen, along gun barrel)
//   +x = to the right
//   +z = up
//
// Angular velocity mapping:
//   ω_z (rotation around +z, vertical axis) → yaw → controls screen X (horizontal)
//   ω_x (rotation around +x, right axis) → pitch → controls screen Y (vertical)
//   ω_y (rotation around +y, along barrel) → roll → ignored for 2D cursor
//
// Formulas per time step Δt:
//   Δx = kx * ωz * dt
//   Δy = -ky * ωx * dt  (negative because pitch up moves cursor up, which is smaller y)
//
// Inputs:
//   - wx, wy, wz: Angular velocities (signed)
//   - vx, vy, vz: Linear velocities (optional, can be used for additional effects)
// Outputs:
//   - x, y: Screen position coordinates
//
module Position_Controller #(
	// Screen dimensions (parameters, can be overridden)
	parameter SCREEN_WIDTH = 1280,
	parameter SCREEN_HEIGHT = 720,
	
	// Sensitivity constants (pixels per (angular_velocity_unit * update_period))
	// These already incorporate the time step dt, so KX/KY = kx * dt from the formula
	// Adjust these to tune aiming sensitivity
	// Example: For 1000Hz update rate (dt = 0.001s) and sensitivity of 100 pixels/(rad/s):
	//   KX = 100 * 0.001 * 1000 = 100 (scaled by 1000 to avoid decimals)
	//   Then divide by 1000 in the calculation to get back to pixels
	parameter KX = 32'd100000,  // Horizontal sensitivity (for ωz → screen X), scaled by 1000
	parameter KY = 32'd100000,  // Vertical sensitivity (for ωx → screen Y), scaled by 1000
	
	// Update rate divider (how many clock cycles per position update)
	// For 74.25MHz clock and 1000Hz update rate: 74,250,000 / 1000 = 74,250
	// For 60Hz update rate: 74,250,000 / 60 = 1,237,500
	parameter UPDATE_DIVIDER = 24'd74250,  // Default: ~1000Hz update rate
	
	// Optional: Enable linear velocity contributions
	parameter USE_LINEAR_VELOCITY = 0,
	parameter LINEAR_SCALE = 32'd10  // Scaling for linear velocities if enabled
)(
	input logic i_clk,              // Clock (typically VGA clock, e.g., 74.25MHz)
	input logic i_rst_n,            // Reset (active low)
	
	// Angular velocity inputs (signed) - PRIMARY INPUTS FOR AIMING
	input logic signed [15:0] i_wx, // Angular velocity X (pitch around X axis, right)
	input logic signed [15:0] i_wy, // Angular velocity Y (yaw around Y axis, forward) - ignored for 2D cursor
	input logic signed [15:0] i_wz, // Angular velocity Z (yaw around Z axis, up) - controls horizontal aim
	
	// Linear velocity inputs (signed) - OPTIONAL
	input logic signed [31:0] i_vx, // Linear velocity X (right/left)
	input logic signed [31:0] i_vy, // Linear velocity Y (forward/backward toward screen)
	input logic signed [31:0] i_vz, // Linear velocity Z (up/down)
	
	// Output position
	output logic [10:0] o_x,         // Screen X position (0 to SCREEN_WIDTH-1)
	output logic [9:0] o_y           // Screen Y position (0 to SCREEN_HEIGHT-1)
);

	// Internal position registers (signed, with extra bits for accumulation)
	// Using 32-bit signed for position to allow large accumulated values
	logic signed [31:0] pos_x;
	logic signed [31:0] pos_y;
	
	// Clock divider for position updates
	logic [23:0] update_counter;
	logic update_tick;
	
	// Position deltas per update (computed from angular velocities)
	// Δx = kx * ωz * dt
	// Δy = -ky * ωx * dt
	logic signed [31:0] delta_x;
	logic signed [31:0] delta_y;
	
	// Intermediate new position values
	logic signed [31:0] new_pos_x;
	logic signed [31:0] new_pos_y;
	
	// Optional linear velocity contributions
	logic signed [31:0] linear_delta_x;
	logic signed [31:0] linear_delta_y;
	
	// Clock divider: generate update tick
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			update_counter <= 24'd0;
			update_tick <= 1'b0;
		end else begin
			if (update_counter >= UPDATE_DIVIDER - 1) begin
				update_counter <= 24'd0;
				update_tick <= 1'b1;
			end else begin
				update_counter <= update_counter + 24'd1;
				update_tick <= 1'b0;
			end
		end
	end
	
	// Calculate position deltas from angular velocities
	// Core aiming formulas:
	//   Δx = kx * ωz * dt
	//   Δy = -ky * ωx * dt
	//
	// Where:
	//   - ωz (yaw around +z) controls horizontal aim (screen X)
	//   - ωx (pitch around +x) controls vertical aim (screen Y)
	//   - The negative sign in Δy is because pitch up (positive ωx) moves cursor up (smaller y)
	//
	// KX and KY are scaled by 1000 to allow integer arithmetic, so we divide by 1000 at the end
	always_comb begin
		// Horizontal movement: ωz → screen X
		// Δx = (KX/1000) * ωz = (KX * ωz) / 1000
		delta_x = (KX * i_wz) / 32'd1000;
		
		// Vertical movement: ωx → screen Y (negative because pitch up = cursor up = smaller y)
		// Δy = -(KY/1000) * ωx = -(KY * ωx) / 1000
		delta_y = -(KY * i_wx) / 32'd1000;
	end
	
	// Optional: Add linear velocity contributions if enabled
	generate
		if (USE_LINEAR_VELOCITY) begin
			always_comb begin
				// Linear velocities can add small corrections
				// vx (right) → screen X
				// vz (up) → screen Y (negative because screen Y increases downward)
				linear_delta_x = (i_vx * LINEAR_SCALE) / 32'd1000;
				linear_delta_y = (-i_vz * LINEAR_SCALE) / 32'd1000;
			end
		end else begin
			always_comb begin
				linear_delta_x = 32'd0;
				linear_delta_y = 32'd0;
			end
		end
	endgenerate
	
	// Calculate new position (combinational)
	// Combine angular velocity deltas with optional linear velocity contributions
	always_comb begin
		new_pos_x = pos_x + delta_x + linear_delta_x;
		new_pos_y = pos_y + delta_y + linear_delta_y;
	end
	
	// Position integration and clamping
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			// Initialize to center of screen
			pos_x <= 32'(SCREEN_WIDTH / 2);
			pos_y <= 32'(SCREEN_HEIGHT / 2);
		end else if (update_tick) begin
			// Update position with clamping
			if (new_pos_x < 0) begin
				pos_x <= 32'd0;
			end else if (new_pos_x >= SCREEN_WIDTH) begin
				pos_x <= 32'(SCREEN_WIDTH - 1);
			end else begin
				pos_x <= new_pos_x;
			end
			
			if (new_pos_y < 0) begin
				pos_y <= 32'd0;
			end else if (new_pos_y >= SCREEN_HEIGHT) begin
				pos_y <= 32'(SCREEN_HEIGHT - 1);
			end else begin
				pos_y <= new_pos_y;
			end
		end
	end
	
	// Output position (clamped to screen bounds)
	always_comb begin
		// Clamp final output
		if (pos_x < 0) begin
			o_x = 11'd0;
		end else if (pos_x >= SCREEN_WIDTH) begin
			o_x = SCREEN_WIDTH[10:0] - 11'd1;
		end else begin
			o_x = pos_x[10:0];
		end
		
		if (pos_y < 0) begin
			o_y = 10'd0;
		end else if (pos_y >= SCREEN_HEIGHT) begin
			o_y = SCREEN_HEIGHT[9:0] - 10'd1;
		end else begin
			o_y = pos_y[9:0];
		end
	end

endmodule

