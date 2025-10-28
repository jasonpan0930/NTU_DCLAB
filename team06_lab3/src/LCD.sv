module LCD(
	input			 i_rst_n,
	input			 i_clk,
    input [1:0] i_mode,
	input [19:0] i_addr_current,
	input [19:0] i_addr_max,
	output		 o_LCD_BLON,
	output	[7:0] o_LCD_DATA,
	output		 o_LCD_EN,
	output		 o_LCD_ON,
	output		 o_LCD_RS,
	output		 o_LCD_RW
);

	// ----------------------------------------------------------------
	// Parameters
	// ----------------------------------------------------------------
	parameter int CLK_HZ = 12_000_000; // Default 12 MHz

	// LCD timing (conservative)
	localparam int T_POWERON_US   = 15_000; // 15 ms
	localparam int T_CMD_US       = 100;    // standard command wait
	localparam int T_CLEAR_US     = 2_000;  // clear/home needs >1.52ms
	localparam int T_FUNC1_US     = 4_100;  // function set first wait
	localparam int T_PULSE_EN_NS  = 1000;   // EN high > 450ns (use 1us)

	localparam int CYCLES_PER_US  = CLK_HZ / 1_000_000;
	localparam int EN_PULSE_CYC   = (CLK_HZ + 999_999) / 1_000_000; // ~1us pulse

	// ----------------------------------------------------------------
	// Outputs (constant/defaults)
	// ----------------------------------------------------------------
	assign o_LCD_ON   = 1'b1;  // Turn LCD power on
	assign o_LCD_BLON = 1'b1;  // Backlight on
	assign o_LCD_RW   = 1'b0;  // Always write

	// ----------------------------------------------------------------
	// Internal registers
	// ----------------------------------------------------------------
	typedef enum logic [4:0] {
		S_PWR_WAIT,
		S_FUNC_SET_1,
		S_FUNC_SET_2,
		S_FUNC_SET_3,
		S_DISP_ON,
		S_CLEAR,
		S_ENTRY_MODE,
		S_SET_LINE1,
		S_WRITE_DATA,
		S_SET_LINE2,
		S_WRITE_ADDR_DATA,
		S_DONE
	} state_t;

	state_t state_r, state_w;

	logic [31:0] delay_cnt_r, delay_cnt_w;
	logic [7:0]  data_bus_r, data_bus_w;
	logic        rs_r, rs_w;
	logic        en_r, en_w;
	logic [7:0]  text_rom_record [0:31];
	logic [7:0]  text_rom_play [0:31];
	logic [5:0]  text_idx_r, text_idx_w; // supports up to 32 chars
	logic [5:0]  addr_text_idx_r, addr_text_idx_w; // index for address text
	logic [7:0]  addr_text_rom [0:31]; // buffer for address text on line 2
	logic [7:0]  current_byte;
	logic        start_pulse_r, start_pulse_w; // begin an EN pulse
	logic [15:0] en_pulse_cnt_r, en_pulse_cnt_w;
	logic        en_pulsing;
	logic [1:0]  i_mode_r, i_mode_w; // track i_mode changes
	logic  [1:0]      last_mode_r, last_mode_w; // track if text was written
	logic [4:0]  last_addr_current_r, last_addr_current_w; // track address changes
	logic [4:0]  last_addr_max_r, last_addr_max_w;
	logic [15:0] refresh_timer_r, refresh_timer_w; // Timer to limit refresh rate

	assign o_LCD_DATA = data_bus_r;
	assign o_LCD_RS   = rs_r;
	assign o_LCD_EN   = en_r;

	assign en_pulsing = (en_pulse_cnt_r != 16'd0);

	// ----------------------------------------------------------------
	// Text ROM: "Record time:" and "Play time:"
	// ----------------------------------------------------------------
	// Record time: R e c o r d _ t i m e :
	// Play time:    P l a y _ t i m e :
	initial begin
		// Record time
		text_rom_record[0]  = "R";
		text_rom_record[1]  = "e";
		text_rom_record[2]  = "c";
		text_rom_record[3]  = "o";
		text_rom_record[4]  = "r";
		text_rom_record[5]  = "d";
		text_rom_record[6]  = 8'h20; // space
		text_rom_record[7]  = "t";
		text_rom_record[8]  = "i";
		text_rom_record[9]  = "m";
		text_rom_record[10] = "e";
		text_rom_record[11] = ":";
		
		// Play time
		text_rom_play[0]  = "P";
		text_rom_play[1]  = "l";
		text_rom_play[2]  = "a";
		text_rom_play[3]  = "y";
		text_rom_play[4]  = 8'h20; // space
		text_rom_play[5]  = "t";
		text_rom_play[6]  = "i";
		text_rom_play[7]  = "m";
		text_rom_play[8]  = "e";
		text_rom_play[9]  = ":";
		
		// rest unused
		for (int i = 12; i < 32; i++) begin
			text_rom_record[i] = 8'h20;
			text_rom_play[i] = 8'h20;
		end
	end

	// Helper to start a data/command write with EN pulse and then wait
	task automatic issue_byte(
		input logic rs,
		input logic [7:0] data_byte,
		input int wait_us,
		output logic [7:0] data_bus_next,
		output logic rs_next,
		output logic start_pulse_next,
		output logic [31:0] delay_cnt_next
	);
		begin
			data_bus_next     = data_byte;
			rs_next           = rs;
			start_pulse_next  = 1'b1; // begin EN pulse
			delay_cnt_next    = wait_us * CYCLES_PER_US;
		end
		endtask

	// Helper to convert number (0-99) to 2 decimal ASCII chars
	function automatic logic [15:0] num_to_decimal_ascii(input logic [7:0] num);
		begin
			// Extract tens and ones digits
			automatic logic [3:0] tens = num / 10;
			automatic logic [3:0] ones = num % 10;
			num_to_decimal_ascii = {8'h30 + tens, 8'h30 + ones};
		end
	endfunction

	// ----------------------------------------------------------------
	// Combinational FSM
	// ----------------------------------------------------------------
	always_comb begin
		state_w        = state_r;
		delay_cnt_w    = delay_cnt_r;
		data_bus_w     = data_bus_r;
		rs_w           = rs_r;
		en_w           = en_r;
		text_idx_w     = text_idx_r;
		addr_text_idx_w = addr_text_idx_r;
		start_pulse_w  = 1'b0;
		en_pulse_cnt_w = en_pulse_cnt_r;
		i_mode_w       = i_mode;
		last_mode_w    = last_mode_r;
		// Default: keep previous address values unless updated by state
		last_addr_current_w = last_addr_current_r;
		last_addr_max_w = last_addr_max_r;
		refresh_timer_w = refresh_timer_r;
		
		// Prepare address text for line 2: (current>>15) "/" (max>>15)
		// Format like "12/34"
		begin
			automatic logic [7:0] current_val = i_addr_current[19:15];
			automatic logic [7:0] max_val = i_addr_max[19:15];
			automatic logic [15:0] current_ascii = num_to_decimal_ascii(current_val);
			automatic logic [15:0] max_ascii = num_to_decimal_ascii(max_val);
			
			addr_text_rom[0] = current_ascii[15:8]; // tens digit of current
			addr_text_rom[1] = current_ascii[7:0];  // ones digit of current
			addr_text_rom[2] = "/"; // 0x2F
			addr_text_rom[3] = max_ascii[15:8];    // tens digit of max
			addr_text_rom[4] = max_ascii[7:0];     // ones digit of max
		end

		// Default: EN low unless we are pulsing
		en_w = (en_pulsing) ? 1'b1 : 1'b0;

		// EN pulse countdown
		if (start_pulse_r) begin
			// latch request handled in sequential
		end
		else if (en_pulsing) begin
			// keep EN high until counter hits zero (sequential dec)
		end
		else begin
			// when not pulsing, manage delays
			if (delay_cnt_r != 0) begin
				// wait
			end else begin
				case (state_r)
					S_PWR_WAIT: begin
						// Function set 0x38 (8-bit, 2 lines, 5x8 font)
						issue_byte(1'b0, 8'h38, T_FUNC1_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						state_w = S_FUNC_SET_1;
					end
					S_FUNC_SET_1: begin
						issue_byte(1'b0, 8'h38, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						state_w = S_FUNC_SET_2;
					end
					S_FUNC_SET_2: begin
						issue_byte(1'b0, 8'h38, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						state_w = S_FUNC_SET_3;
					end
					S_FUNC_SET_3: begin
						// Display ON, Cursor OFF, Blink OFF
						issue_byte(1'b0, 8'h0C, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						state_w = S_DISP_ON;
					end
					S_DISP_ON: begin
						// Clear Display
						issue_byte(1'b0, 8'h01, T_CLEAR_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						state_w = S_CLEAR;
					end
					S_CLEAR: begin
						// After clearing, check if we came from S_DONE (mode change)
						// If mode is 0, just go back to done, otherwise continue to entry mode
						if (i_mode_r == 2'b00 && last_mode_r == i_mode_r) begin
							state_w = S_DONE;
						end else begin
							// Entry Mode: Increment, no shift
							issue_byte(1'b0, 8'h06, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
							state_w = S_ENTRY_MODE;
						end
					end
					S_ENTRY_MODE: begin
						// Set DDRAM to line 1 start (0x80)
						issue_byte(1'b0, 8'h80, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						state_w = S_SET_LINE1;
					end
					S_SET_LINE1: begin
						// Check i_mode before writing text
						if (i_mode_r == 2'b00) begin
							// Skip writing and go to done
							state_w = S_DONE;
						end else begin
							// Begin writing text - select first char based on mode
							text_idx_w = 6'd0;
							if (i_mode_r == 2'b01) begin
								issue_byte(1'b1, text_rom_record[0], T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
							end else begin // i_mode_r == 2'b10
								issue_byte(1'b1, text_rom_play[0], T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
							end
							state_w = S_WRITE_DATA;
							last_mode_w = i_mode_r; // remember we wrote text for this mode
						end
					end
					S_WRITE_DATA: begin
						// Check how many chars to write based on mode
						logic [5:0] max_idx;
						if (i_mode_r == 2'b01) begin
							max_idx = 6'd11; // "Record time:" has 11 chars
						end else begin
							max_idx = 6'd9; // "Play time:" has 9 chars
						end
						
						if (text_idx_r < max_idx) begin
							text_idx_w = text_idx_r + 6'd1;
							if (i_mode_r == 2'b01) begin
								issue_byte(1'b1, text_rom_record[text_idx_w], T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
							end else begin
								issue_byte(1'b1, text_rom_play[text_idx_w], T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
							end
						end else begin
							// Move to line 2 to show address info
							state_w = S_SET_LINE2;
						end
					end
					S_SET_LINE2: begin
						// Set DDRAM to line 2 start (0xC0)
						issue_byte(1'b0, 8'hC0, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						addr_text_idx_w = 6'd0;
						state_w = S_WRITE_ADDR_DATA;
					end
					S_WRITE_ADDR_DATA: begin
						// Write address info: 5 chars (e.g., "12/34")
						if (addr_text_idx_r < 6'd4) begin
							addr_text_idx_w = addr_text_idx_r + 6'd1;
							issue_byte(1'b1, addr_text_rom[addr_text_idx_w], T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
						end else begin
							// Update last_addr registers to prevent immediate re-trigger
							last_addr_current_w = i_addr_current[19:15];
							last_addr_max_w = i_addr_max[19:15];
							state_w = S_DONE;
						end
					end
					S_DONE: begin
						// Monitor i_mode changes
						if (i_mode_r != last_mode_r) begin
							// Mode changed, reload
							if (i_mode_r == 2'b00) begin
								// Mode is now 0, clear display
								issue_byte(1'b0, 8'h01, T_CLEAR_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
								state_w = S_CLEAR;
								last_mode_w = i_mode_r;
							end else begin
								// Mode is now != 0, write text
								issue_byte(1'b0, 8'h80, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
								state_w = S_SET_LINE1;
								last_mode_w = i_mode_r;
							end
						end else if (i_mode_r != 2'b00) begin
							// In record/play mode, check if addresses changed (throttled by timer)
							if (refresh_timer_r == 16'd0) begin
								logic [4:0] current_val, max_val;
								current_val = i_addr_current[19:15];
								max_val = i_addr_max[19:15];
								if (current_val != last_addr_current_r || max_val != last_addr_max_r) begin
									// Address changed, refresh line 2
									issue_byte(1'b0, 8'hC0, T_CMD_US, data_bus_w, rs_w, start_pulse_w, delay_cnt_w);
									addr_text_idx_w = 6'd0;
									state_w = S_WRITE_ADDR_DATA;
									// Reload timer (~0.5ms at 12MHz = 6000 cycles)
									refresh_timer_w = 16'd6000;
								end
							end
						end
						// else idle
					end
					default: state_w = S_DONE;
				endcase
			end
		end
	end

	// ----------------------------------------------------------------
	// Sequential
	// ----------------------------------------------------------------
	always_ff @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			state_r        <= S_PWR_WAIT;
			delay_cnt_r    <= T_POWERON_US * CYCLES_PER_US;
			data_bus_r     <= 8'h00;
			rs_r           <= 1'b0;
			en_r           <= 1'b0;
			text_idx_r     <= 6'd0;
			addr_text_idx_r <= 6'd0;
			start_pulse_r  <= 1'b0;
			en_pulse_cnt_r <= 16'd0;
			i_mode_r       <= 2'b00;
			last_mode_r    <= 2'b00;
			last_addr_current_r <= 5'd0;
			last_addr_max_r <= 5'd0;
			refresh_timer_r <= 16'd6000; // Start timer
		end else begin
			state_r        <= state_w;
			data_bus_r      <= data_bus_w;
			rs_r            <= rs_w;
			en_r            <= en_w;
			text_idx_r      <= text_idx_w;
			addr_text_idx_r <= addr_text_idx_w;
			start_pulse_r   <= start_pulse_w;
			i_mode_r        <= i_mode_w;
			last_mode_r     <= last_mode_w;
			last_addr_current_r <= last_addr_current_w;
			last_addr_max_r <= last_addr_max_w;

			// Delay counter
			if (delay_cnt_r != 0)
				delay_cnt_r <= delay_cnt_r - 1;
			else
				delay_cnt_r <= delay_cnt_w; // reloaded when issuing next byte

			// Refresh timer - decrement when in DONE state, otherwise use combo value
			if (state_r == S_DONE && refresh_timer_r > 0)
				refresh_timer_r <= refresh_timer_r - 1;
			else
				refresh_timer_r <= refresh_timer_w;

			// Start EN pulse when requested
			if (start_pulse_w) begin
				en_pulse_cnt_r <= EN_PULSE_CYC[15:0];
				en_r           <= 1'b1;
			end else if (en_pulsing) begin
				// continue pulse
				if (en_pulse_cnt_r > 0)
					en_pulse_cnt_r <= en_pulse_cnt_r - 1;
				else begin
					en_pulse_cnt_r <= 16'd0;
					en_r           <= 1'b0;
				end
			end
		end
	end

endmodule

