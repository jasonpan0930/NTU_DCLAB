module Clean_sram(
    input  i_rst_n,
    input  i_clk,
    input  i_start,

    // SRAM write interface
    output logic [19:0] o_address,
    output logic [15:0] o_data,

    // Status
    output logic        o_done
);

    // Parameters
    localparam logic [19:0] ADDR_MAX = 20'hFFFFF; // 1M x 16 SRAM (20-bit address space)

    // Internal registers
    typedef enum logic [0:0] {
        S_IDLE  = 1'd0,
        S_WRITE = 1'd1
    } state_t;

    state_t state_r, state_w;
    logic [19:0] addr_r, addr_w;
    logic done_r, done_w;
    logic start_r; // for edge detect

    // Outputs that are static/derived
    // Always write zeros when cleaning
    assign o_data = 16'd0;
    // o_done is generated as a one-cycle pulse in combinational logic
    assign o_done = done_r;
    // Start pulse detection
    wire start_pulse = i_start & ~start_r;

    // Combinational next-state logic
    always_comb begin
        state_w  = state_r;
        addr_w   = addr_r;
        // Default outputs
        o_address= addr_r;
        done_w = done_r;
        unique case (state_r)
            S_IDLE: begin
                done_w = 1'b0;
                if (start_pulse) begin
                    state_w = S_WRITE;
                    addr_w  = 20'd0;
                end
            end

            S_WRITE: begin
                // Address increments every cycle
                if (addr_r == ADDR_MAX) begin
                    // Last write cycle: assert done pulse and go back to IDLE
                    done_w  = 1'b1;
                    state_w = S_IDLE;
                end else begin
                    addr_w = addr_r + 20'd1;
                end
            end

            default: begin
                state_w = S_IDLE;
            end
        endcase
    end

    // Sequential logic
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state_r <= S_IDLE;
            addr_r  <= 20'd0;
            start_r <= 1'b0;
            done_r <= 1'b0;
        end else begin
            state_r <= state_w;
            addr_r  <= addr_w;
            start_r <= i_start;
            done_r <= done_w;
        end
    end

endmodule
    