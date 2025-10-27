// I2cInitializer.sv
// WM8731 I2C init: open-drain correct, proper START/STOP,
// 24-bit control word framing, ACK release on every 9th clock.

module I2cInitializer (
    input  logic i_rst_n,
    input  logic i_clk,        // bit engine clock; SCL ~= i_clk/2
    input  logic i_start,      // level or pulse; starts the whole sequence
    output logic o_finished,   // high when all config words sent
    output logic o_sclk,       // I2C SCL (master drives)
    output logic o_sdat,       // I2C SDA data value when driving (always 0)
    output logic o_oen         // SDA output enable: 1=drive low, 0=release (Z)
);

    // ---- Codec I2C 7-bit address (CSB=0 on DE2-115 boards) ----
    localparam logic [6:0] WM8731_ADDR = 7'h1A; // 0011010

    // ---- WM8731 lab-recommended configuration sequence ----
    // Each record is { reg_addr[6:0], reg_data[8:0] }.
    typedef struct packed {
        logic [6:0] reg_addr;
        logic [8:0] reg_data;
    } cfg_t;

    // Matches lab slides: 16-bit, I2S, 32kHz, Active
    localparam int N_CFG = 6;
    localparam cfg_t CFG [N_CFG] = '{
        '{7'h04, 9'b0_0001_0101}, // R4  Analog Audio Path Control
        '{7'h05, 9'b0_0000_0000}, // R5  Digital Audio Path Control
        '{7'h06, 9'b0_0000_0000}, // R6  Power Down Control (normal)
        '{7'h07, 9'b0_0100_0010}, // R7  Digital Audio IF: I2S, 16-bit (0x042)
        '{7'h08, 9'b0_0001_1001}, // R8  Sampling Control: 32k (needs 12MHz MCLK)
        '{7'h09, 9'b0_0000_0001}  // R9  Active Control: Active
    };

    // We send 3 bytes per register:
    // B0 = {WM8731_ADDR, 1'b0}      // address + W
    // B1 = {reg_addr[6:0], data[8]} // reg addr + MSB of data
    // B2 =  data[7:0]
    logic [7:0] b0, b1, b2;

    // State machine
    typedef enum logic [3:0] {
        ST_IDLE,
        ST_START,
        ST_LOAD_B0, ST_BIT_B0_LOW, ST_BIT_B0_HIGH, ST_ACK0_LOW, ST_ACK0_HIGH,
        ST_LOAD_B1, ST_BIT_B1_LOW, ST_BIT_B1_HIGH, ST_ACK1_LOW, ST_ACK1_HIGH,
        ST_LOAD_B2, ST_BIT_B2_LOW, ST_BIT_B2_HIGH, ST_ACK2_LOW, ST_ACK2_HIGH,
        ST_STOP_A, ST_STOP_B,
        ST_NEXT,
        ST_DONE
    } st_t;

    st_t  st, st_n;
    logic sclk, sclk_n;
    logic sda_drive_low, sda_drive_low_n;     // 1=drive SDA low, 0=release (Z)
    logic [3:0] bit_cnt, bit_cnt_n;
    logic [2:0] cfg_idx, cfg_idx_n;           // enough for N_CFG <= 8
    logic [7:0] shreg, shreg_n;
    logic       finished, finished_n;

    // Outputs (open-drain: we *only* ever drive SDA low)
    assign o_sclk     = sclk;
    assign o_sdat     = 1'b0;                 // value when driving (low)
    assign o_oen      = sda_drive_low;        // 1 = drive low, 0 = Z (pulled high)
    assign o_finished = finished;

    // Compose the three bytes for current entry
    always_comb begin
        b0 = {WM8731_ADDR, 1'b0};  // WRITE
        b1 = {CFG[cfg_idx].reg_addr, CFG[cfg_idx].reg_data[8]};
        b2 =  CFG[cfg_idx].reg_data[7:0];
    end

    // Sequential
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            st            <= ST_IDLE;
            sclk          <= 1'b1;    // bus idle: SCL high
            sda_drive_low <= 1'b0;    // bus idle: SDA released (Z->pull-up)
            bit_cnt       <= '0;
            cfg_idx       <= '0;
            shreg         <= '0;
            finished      <= 1'b0;
        end else begin
            st            <= st_n;
            sclk          <= sclk_n;
            sda_drive_low <= sda_drive_low_n;
            bit_cnt       <= bit_cnt_n;
            cfg_idx       <= cfg_idx_n;
            shreg         <= shreg_n;
            finished      <= finished_n;
        end
    end

    // Combinational FSM
    always_comb begin
        // defaults
        st_n            = st;
        sclk_n          = sclk;
        sda_drive_low_n = sda_drive_low;
        bit_cnt_n       = bit_cnt;
        cfg_idx_n       = cfg_idx;
        shreg_n         = shreg;
        finished_n      = finished;

        case (st)
            ST_IDLE: begin
                sclk_n          = 1'b1;
                sda_drive_low_n = 1'b0;
                finished_n      = 1'b0;
                if (i_start) st_n = ST_START;
            end

            // START: SDA low while SCL high
            ST_START: begin
                sclk_n          = 1'b1;
                sda_drive_low_n = 1'b1; // pull SDA low
                st_n            = ST_LOAD_B0;
            end

            // --- Byte 0 ---
            ST_LOAD_B0: begin
                shreg_n   = b0;
                bit_cnt_n = 4'd7;
                sclk_n    = 1'b0;            // low phase to set first bit
                st_n      = ST_BIT_B0_LOW;
            end
            ST_BIT_B0_LOW: begin
                sda_drive_low_n = (shreg[7] == 1'b0); // 0: drive low, 1: release
                sclk_n          = 1'b1;               // clock high: data valid
                st_n            = ST_BIT_B0_HIGH;
            end
            ST_BIT_B0_HIGH: begin
                sclk_n  = 1'b0;                        // back low, shift next
                shreg_n = {shreg[6:0], 1'b0};
                if (bit_cnt == 0) st_n = ST_ACK0_LOW;
                else begin
                    bit_cnt_n = bit_cnt - 1;
                    st_n      = ST_BIT_B0_LOW;
                end
            end
            ST_ACK0_LOW: begin
                sda_drive_low_n = 1'b0;    // release for ACK
                sclk_n          = 1'b1;    // slave pulls SDA low for ACK=0
                st_n            = ST_ACK0_HIGH;
            end
            ST_ACK0_HIGH: begin
                sclk_n = 1'b0;
                st_n   = ST_LOAD_B1;
            end

            // --- Byte 1 ---
            ST_LOAD_B1: begin
                shreg_n   = b1;
                bit_cnt_n = 4'd7;
                sclk_n    = 1'b0;
                st_n      = ST_BIT_B1_LOW;
            end
            ST_BIT_B1_LOW: begin
                sda_drive_low_n = (shreg[7] == 1'b0);
                sclk_n          = 1'b1;
                st_n            = ST_BIT_B1_HIGH;
            end
            ST_BIT_B1_HIGH: begin
                sclk_n  = 1'b0;
                shreg_n = {shreg[6:0], 1'b0};
                if (bit_cnt == 0) st_n = ST_ACK1_LOW;
                else begin
                    bit_cnt_n = bit_cnt - 1;
                    st_n      = ST_BIT_B1_LOW;
                end
            end
            ST_ACK1_LOW: begin
                sda_drive_low_n = 1'b0;
                sclk_n          = 1'b1;
                st_n            = ST_ACK1_HIGH;
            end
            ST_ACK1_HIGH: begin
                sclk_n = 1'b0;
                st_n   = ST_LOAD_B2;
            end

            // --- Byte 2 ---
            ST_LOAD_B2: begin
                shreg_n   = b2;
                bit_cnt_n = 4'd7;
                sclk_n    = 1'b0;
                st_n      = ST_BIT_B2_LOW;
            end
            ST_BIT_B2_LOW: begin
                sda_drive_low_n = (shreg[7] == 1'b0);
                sclk_n          = 1'b1;
                st_n            = ST_BIT_B2_HIGH;
            end
            ST_BIT_B2_HIGH: begin
                sclk_n  = 1'b0;
                shreg_n = {shreg[6:0], 1'b0};
                if (bit_cnt == 0) st_n = ST_ACK2_LOW;
                else begin
                    bit_cnt_n = bit_cnt - 1;
                    st_n      = ST_BIT_B2_LOW;
                end
            end
            ST_ACK2_LOW: begin
                sda_drive_low_n = 1'b0;
                sclk_n          = 1'b1;
                st_n            = ST_ACK2_HIGH;
            end
            ST_ACK2_HIGH: begin
                sclk_n = 1'b0;
                st_n   = ST_STOP_A;
            end

            // STOP: SDA goes high while SCL high
            ST_STOP_A: begin
                sda_drive_low_n = 1'b1; // ensure SDA low
                sclk_n          = 1'b1; // SCL high
                st_n            = ST_STOP_B;
            end
            ST_STOP_B: begin
                sda_drive_low_n = 1'b0; // release SDA -> STOP
                st_n            = ST_NEXT;
            end

            // Next register
            ST_NEXT: begin
                if (cfg_idx == N_CFG-1) st_n = ST_DONE;
                else begin
                    cfg_idx_n = cfg_idx + 1;
                    st_n      = ST_START;   // repeated START
                end
            end

            ST_DONE: begin
                sclk_n     = 1'b1;
                sda_drive_low_n = 1'b0;
                finished_n = 1'b1;
            end

            default: st_n = ST_IDLE;
        endcase
    end

endmodule
