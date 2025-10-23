module Rsa256Core (
	input          i_clk,
	input          i_rst,
	input          i_start,
	input  [255:0] i_a, // cipher text y
	input  [255:0] i_d, // private key
	input  [255:0] i_n, // N
	output [255:0] o_a_pow_d, // plain text x (message to be sent)
	output         o_finished
);

// Controller to compute y^d (mod N) using Montgomery method.
// 1) Precompute t = y * 2^256 (mod N) via 256 doublings with reduction.
// 2) m <- 1.
// 3) For i=0..255 (LSB first of d):
//      if d[i]==1: m <- Mont(N, m, t)
//      t <- Mont(N, t, t)

typedef enum logic [2:0] {
	S_IDLE   = 3'd0,
	S_PREP   = 3'd1,
	S_MBIT   = 3'd2,
	S_SQR    = 3'd3,
	S_DONE   = 3'd4
} state_t;

state_t state_r, state_w;

logic [255:0] t_r, t_w;
logic [255:0] m_r, m_w;
logic [7:0]   cnt_r, cnt_w;
logic         finished_r, finished_w;

logic         d_bit;
assign d_bit = i_d[cnt_r];

// Montgomery module 1 (for m = m * t when d_bit == 1)
logic         mont1_start_w, mont1_start_r;
logic [255:0] mont1_a_w, mont1_a_r;
logic [255:0] mont1_b_w, mont1_b_r;
logic [255:0] mont1_ans;
logic         mont1_done;

// Montgomery module 2 (for t = t * t, always runs)
logic         mont2_start_w, mont2_start_r;
logic [255:0] mont2_a_w, mont2_a_r;
logic [255:0] mont2_b_w, mont2_b_r;
logic [255:0] mont2_ans;
logic         mont2_done;

Montgomery_algorithm #(.WIDTH(256)) u_mont1 (
	.i_clk   (i_clk),
	.i_rst   (i_rst),
	.i_start (mont1_start_r),
	.i_a     (mont1_a_r),
	.i_b     (mont1_b_r),
	.i_n     (i_n),
	.o_ans   (mont1_ans),
	.o_done  (mont1_done)
);

Montgomery_algorithm #(.WIDTH(256)) u_mont2 (
	.i_clk   (i_clk),
	.i_rst   (i_rst),
	.i_start (mont2_start_r),
	.i_a     (mont2_a_r),
	.i_b     (mont2_b_r),
	.i_n     (i_n),
	.o_ans   (mont2_ans),
	.o_done  (mont2_done)
);

assign o_a_pow_d = m_r;
assign o_finished = finished_r;

always_comb begin
	state_w       = state_r;
	t_w           = t_r;
	m_w           = m_r;
	cnt_w         = cnt_r;
	finished_w    = finished_r;
	mont1_start_w  = 1'b0;
	mont1_a_w      = mont1_a_r;
	mont1_b_w      = mont1_b_r;
	mont2_start_w  = 1'b0;
	mont2_a_w      = mont2_a_r;
	mont2_b_w      = mont2_b_r;

	unique case (state_r)
		S_IDLE: begin
			if (i_start) begin
				finished_w = 1'b0;
				t_w    = i_a;
				m_w    = 256'd1;
				cnt_w  = 8'd0;
				state_w = S_PREP;
			end else begin
				finished_w = 1'b0;
			end
		end
		S_PREP: begin
			// done t = y * 2^256 mod N
			logic [256:0] dbl;
			dbl = {1'b0, t_r} + {1'b0, t_r};
			if (dbl[255:0] >= i_n || dbl[256]) begin
				dbl = dbl - {1'b0, i_n};
				t_w = dbl[255:0];
			end else begin
				t_w = dbl[255:0];
			end
			cnt_w = cnt_r + 8'd1;
			if (cnt_r == 8'd255) begin
				cnt_w  = 8'd0;
				state_w = S_MBIT;
			end
		end
		S_MBIT: begin
			// Run both Montgomery modules in parallel
			// mont1: m = m * t (only if d_bit == 1)
			// mont2: t = t * t (always)
			
			if (!mont1_start_r && !mont2_start_r) begin
				// Start both operations
				if (d_bit) begin
					// Start mont1 for m = m * t
					mont1_a_w     = m_r;
					mont1_b_w     = t_r;
					mont1_start_w = 1'b1;
				end
				// Always start mont2 for t = t * t
				mont2_a_w     = t_r;
				mont2_b_w     = t_r;
				mont2_start_w = 1'b1;
			end else if (mont2_done && (mont1_done || !d_bit)) begin
				// Both done (or mont1 wasn't needed)
				if (d_bit) begin
					m_w = mont1_ans;
				end
				t_w = mont2_ans;
				
				if (cnt_r == 8'd255) begin
					state_w = S_DONE;
				end else begin
					cnt_w   = cnt_r + 8'd1;
					state_w = S_MBIT;
				end
			end
			// Otherwise wait for both to complete
		end
		S_SQR: begin
			// This state is no longer used - merged into S_MBIT
			state_w = S_MBIT;
		end
		S_DONE: begin
			finished_w = 1'b1;
			if (i_start) begin
				finished_w = 1'b0;
				t_w    = i_a;
				m_w    = 256'd1;
				cnt_w  = 8'd0;
				state_w = S_PREP;
			end else begin
				state_w       = S_IDLE;
				t_w           = 256'd0;
				cnt_w         = 8'd0;
				mont1_start_w = 1'b0;
				mont1_a_w     = 256'd0;
				mont1_b_w     = 256'd0;
				mont2_start_w = 1'b0;
				mont2_a_w     = 256'd0;
				mont2_b_w     = 256'd0;
			end
		end
	endcase
end

always_ff @(posedge i_clk or posedge i_rst) begin
	if (i_rst) begin
		state_r       <= S_IDLE;
		t_r           <= 256'd0;
		m_r           <= 256'd0;
		cnt_r         <= 8'd0;
		finished_r    <= 1'b0;
		mont1_start_r <= 1'b0;
		mont1_a_r     <= 256'd0;
		mont1_b_r     <= 256'd0;
		mont2_start_r <= 1'b0;
		mont2_a_r     <= 256'd0;
		mont2_b_r     <= 256'd0;
	end else begin
		state_r       <= state_w;
		t_r           <= t_w;
		m_r           <= m_w;
		cnt_r         <= cnt_w;
		finished_r    <= finished_w;
		mont1_start_r <= mont1_start_w;
		mont1_a_r     <= mont1_a_w;
		mont1_b_r     <= mont1_b_w;
		mont2_start_r <= mont2_start_w;
		mont2_a_r     <= mont2_a_w;
		mont2_b_r     <= mont2_b_w;
	end
end

endmodule



// this module is used to calculate a * b mod n with Montgomery algorithm
module Montgomery_algorithm#(
    parameter int WIDTH = 256
)(
    input i_clk,
    input i_rst,
    input i_start,
    input [WIDTH-1:0] i_a,
    input [WIDTH-1:0] i_b, 
    input [WIDTH-1:0] i_n,
    output [WIDTH-1:0] o_ans,
    output o_done
);

typedef enum logic [2:0] {
    IDLE  = 3'b000,
    PREP  = 3'b001,
    CALC1 = 3'b010,
    CALC2 = 3'b011,
    DONE  = 3'b100
} state_t;

state_t state_r, state_w;
logic [WIDTH-1:0] m_r, m_w;
logic [7:0] counter_r, counter_w;
logic [WIDTH-1:0] saved_a_w, saved_b_w;
logic [WIDTH-1:0] saved_a_r, saved_b_r;
assign o_done = (state_r == DONE);
assign o_ans = m_r;

always_comb begin
    state_w = state_r;
    m_w = m_r;
    counter_w = counter_r;
    saved_a_w = saved_a_r;
    saved_b_w = saved_b_r;
    unique case (state_r)
        IDLE: begin
            if (i_start) begin
                state_w = PREP;
                m_w = 0;
                counter_w = 0;
                saved_a_w = i_a;
                saved_b_w = i_b;
            end
        end
        PREP: begin
            if(saved_a_r >= i_n) begin
                saved_a_w = saved_a_r - i_n;
                state_w = PREP;  // Stay in PREP to continue reduction
            end else if(saved_b_r >= i_n) begin
                saved_b_w = saved_b_r - i_n;
                state_w = PREP;  // Stay in PREP to continue reduction
            end else begin
                // Both are < i_n, ready for calculation
                state_w = CALC1;
                m_w = 0;
                counter_w = 0;
            end
        end
        CALC1: begin
            // First half: conditional add based on saved_a_r[0]
            if(saved_a_r[0]) begin
                logic [WIDTH:0] tmp;
                tmp = {1'b0, m_r} + {1'b0, saved_b_r};
                if(tmp >= {1'b0, i_n}) tmp = tmp - {1'b0, i_n};
                m_w = tmp[WIDTH-1:0];
            end
            state_w = CALC2;
        end
        CALC2: begin
            // Second half: conditional add and shift based on m_r[0]
            if(m_r[0]) begin
                logic [WIDTH:0] tmp;
                tmp = {1'b0, m_r} + {1'b0, i_n};
                tmp = tmp >> 1;
                m_w = tmp[WIDTH-1:0];
            end
            else m_w = m_r >> 1;

            saved_a_w = saved_a_r >> 1;
            counter_w = counter_r + 1;
            if(counter_r == WIDTH-1) begin
                // Always complete all WIDTH iterations for correctness
                if(m_w >= i_n) begin
                    m_w = m_w - i_n;
                end
                state_w = DONE;
            end else begin
                state_w = CALC1;
            end
        end
        DONE: begin
            state_w = IDLE;
        	m_w = 0;
        	counter_w = 0;
			saved_a_w = 0;
			saved_b_w = 0;
        end
        default: begin
            state_w = IDLE;
            m_w = 0;
            counter_w = 0;
        end
    endcase
end

always_ff@(posedge i_clk or posedge i_rst) begin
    if(i_rst) begin
        state_r <= IDLE;
        m_r <= 0;
        counter_r <= 0;
        saved_a_r <= 0;
        saved_b_r <= 0;
    end else begin
        state_r <= state_w;
        m_r <= m_w;
        counter_r <= counter_w;
        saved_a_r <= saved_a_w;
        saved_b_r <= saved_b_w;
    end
end
endmodule