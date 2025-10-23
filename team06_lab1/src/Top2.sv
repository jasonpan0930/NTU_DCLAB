module Top (
	input        i_clk,
	input        i_rst_n,
	input        i_start,
	input        i_previous,
	input		 i_even,
	input		 i_odd,
	input  [12:0] i_range,
	input		 i_stop,
	output [12:0] o_random_out,
	output o_LEDG,
	output o_LEDR
);

// ===== States =====
parameter S_IDLE = 1'b0;
parameter S_RUN  = 1'b1;

// ===== Output Buffers =====
logic [12:0] o_random_out_r, o_random_out_w;

// ===== Registers & Wires =====
logic state_r, state_w;

// Free-running counter to introduce entropy for seeding LFSR
logic [31:0] free_counter_r, free_counter_w;

// 16-bit LFSR for randomness
logic [15:0] lfsr_r, lfsr_w;

// Slowdown timing
logic [31:0] tick_counter_r, tick_counter_w;
logic [31:0] current_period_r, current_period_w;
logic [7:0]  steps_left_r, steps_left_w;

logic [12:0] range_max;

// previous function
logic [15:0] previous_r, previous_w;

localparam INITIAL_PERIOD = 32'd1000000;  // ~20 ms at 50 MHz (fast start)
localparam PERIOD_STEP    = 32'd80000;    // unused when using percentage growth
localparam MAX_STEPS      = 8'd27;        // ~10s total with ~3% growth per tick


// ===== Output Assignments =====
assign o_random_out = o_random_out_r;
assign range_max = i_range;

// ===== Combinational Circuits =====
always_comb begin
	// Default Values
	o_random_out_w   = o_random_out_r;
	state_w          = state_r;
	free_counter_w   = free_counter_r + 32'd1;
	lfsr_w           = lfsr_r;
	tick_counter_w   = tick_counter_r + 32'd1;
	current_period_w = current_period_r;
	steps_left_w     = steps_left_r;
	previous_w       = previous_r;  // 添加默认值

	// detect even odd sw error
	o_LEDR = i_odd && i_even;
	o_LEDG = ~o_LEDR;
	// if(i_odd && i_even) begin
	// 	o_LEDG = 0;
	// 	o_LEDR = 1;
	// end
	// else begin
	// 	o_LEDG = 1;
	// 	o_LEDR = 0;
	// end

	// FSM
	if (state_r == S_IDLE) begin
		if (i_start) begin
			// Start: seed LFSR and begin rolling
			lfsr_w           = (free_counter_r[15:0] ^ tick_counter_r[15:0] ^ 16'hBEEF);
			lfsr_w           = (lfsr_w == 16'd0) ? 16'h0001 : lfsr_w;
			current_period_w = INITIAL_PERIOD;
			tick_counter_w   = 32'd0;
			steps_left_w     = MAX_STEPS;
			state_w          = S_RUN;
		end
	end else begin // S_RUN
		if (i_start) begin
			// Restart: re-seed and begin new round
			lfsr_w           = (free_counter_r[15:0] ^ tick_counter_r[15:0] ^ 16'hBEEF);
			lfsr_w           = (lfsr_w == 16'd0) ? 16'h0001 : lfsr_w;
			current_period_w = INITIAL_PERIOD;
			tick_counter_w   = 32'd0;
			steps_left_w     = MAX_STEPS;
			state_w          = S_RUN;
		end else if (tick_counter_r >= current_period_r) begin
			// Tick: advance LFSR and update display
			logic feedback;
			if(i_stop) begin
				state_w = S_IDLE; // Stop and return to idle
				previous_w = o_random_out_w;
			end else begin
				feedback = lfsr_r[15] ^ lfsr_r[13] ^ lfsr_r[12] ^ lfsr_r[10];
				lfsr_w = {lfsr_r[14:0], feedback};
				o_random_out_w = lfsr_w[12:0];
				// tackle even and odd specification
				if(i_even) begin
					o_random_out_w[0] = 0;
				end
				else if(i_odd) begin
					o_random_out_w[0] = 1;
				end
				// apply range limit using modulus
				if (range_max != 13'd0) begin
					o_random_out_w = lfsr_w % range_max;
				end
				// reset tick counter
				tick_counter_w = 32'd0;
				// Increase period to slow down gradually (~3% growth per tick)
				current_period_w = current_period_r + (current_period_r >> 3);
				// Count down remaining steps and stop when done
				steps_left_w = steps_left_r - 8'd1;
				if (steps_left_r == 8'd0) begin
					state_w = S_IDLE; // Stop and return to idle
					previous_w = o_random_out_w;
				end
			end
		end
	end
end

// ===== Sequential Circuits =====
always_ff @(posedge i_clk or negedge i_rst_n or posedge i_previous) begin
	if (!i_rst_n) begin
		o_random_out_r   <= 13'd0;
		state_r          <= S_IDLE;
		lfsr_r           <= 16'h1;
		tick_counter_r   <= 32'd0;
		current_period_r <= INITIAL_PERIOD;
		steps_left_r     <= MAX_STEPS;
		previous_r		 <= 16'h0;
	end else if (i_previous) begin 
		o_random_out_r   <= previous_w;
		state_r          <= S_IDLE;
		previous_r		 <= previous_w;
		lfsr_r			 <= previous_w;
	end else begin
		o_random_out_r   <= o_random_out_w;
		state_r          <= state_w;
		lfsr_r           <= lfsr_w;
		tick_counter_r   <= tick_counter_w;
		current_period_r <= current_period_w;
		steps_left_r     <= steps_left_w;
		previous_r       <= previous_w;
	end
end

// Free counter runs continuously across resets to add true timing variability
always_ff @(posedge i_clk) begin
	free_counter_r <= free_counter_w;
end

endmodule
