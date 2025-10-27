// I2cInitializer.sv
// WM8731 I2C initialization based on lab specifications
// Protocol: SCL=0: change data, SCL=1: read data
// Format: {ADDR[6:0], RW, REG_ADDR[6:0], DATA[8:0]}
// CSB=0: slave address 0011010 (0x1A)

module I2cInitializer (
    input  logic i_rst_n,
    input  logic i_clk,        // I2C clock (100kHz)
    input  logic i_start,      // start initialization
    input  logic i_sdat,       // I2C SDA input (to read ACK from slave)
    output logic o_finished,   // initialization complete
    output logic o_sclk,       // I2C serial clock
    output logic o_sdat,       // I2C serial data value
    output logic o_oen,        // SDA output enable (1=drive low, 0=release/Z)
    output logic o_ledr        // Red LED: lit when NACK detected (ACK=1)
);

    // I2C slave address (CSB=0 on DE2-115)
    localparam logic [6:0] SLAVE_ADDR = 7'b0011010; // 0x1A

    // Configuration structure: {reg_addr[6:0], reg_data[8:0]}
    typedef struct packed {
        logic [6:0] reg_addr;
        logic [8:0] reg_data;
    } cfg_t;

    // Initialization sequence from lab notes
    localparam int N_CFG = 6;
    localparam cfg_t CFG [N_CFG] = '{
        '{7'h04, 9'b0_0001_0101}, // R4: Analog Audio Path Control
        '{7'h05, 9'b0_0000_0000}, // R5: Digital Audio Path Control (disable soft mute)
        '{7'h06, 9'b0_0000_0000}, // R6: Power Down Control (power on, all enabled)
        '{7'h07, 9'b0_0100_0010}, // R7: Digital Audio Interface Format (I2S, 16-bit)
        '{7'h08, 9'b0_0001_1001}, // R8: Sampling Control (USB mode, 32kHz)
        '{7'h09, 9'b0_0000_0001}  // R9: Active Control (interface active)
    };

    // I2C data bytes: {ADDR[6:0], RW}, {REG_ADDR[6:0], DATA[8]}, {DATA[7:0]}
    logic [7:0] b0, b1, b2;

    // I2C state machine
    typedef enum logic [4:0] {
        ST_IDLE,
        ST_START_A, ST_START_B,
        ST_LOAD_B0, ST_BIT_B0_SET, ST_BIT_B0_CLK, ST_ACK0_CHECK, ST_ACK0_READ, ST_ACK0_DONE,
        ST_LOAD_B1, ST_BIT_B1_SET, ST_BIT_B1_CLK, ST_ACK1_CHECK, ST_ACK1_READ, ST_ACK1_DONE,
        ST_LOAD_B2, ST_BIT_B2_SET, ST_BIT_B2_CLK, ST_ACK2_CHECK, ST_ACK2_READ, ST_ACK2_DONE,
        ST_STOP_A, ST_STOP_B,
        ST_NEXT,
        ST_DONE
    } st_t;

    st_t  state, state_n;
    logic sclk, sclk_n;
    logic sda_drive, sda_drive_n;     // 1=drive SDA low, 0=release (Z)
    logic [3:0] bit_cnt, bit_cnt_n;
    logic [2:0] cfg_idx, cfg_idx_n;
    logic [7:0] shreg, shreg_n;
    logic finished, finished_n;
    logic nack_detected, nack_detected_n;  // NACK error tracking

    // Output assignments
    assign o_sclk      = sclk;
    assign o_sdat      = 1'b0;                 // only drive low (open-drain)
    assign o_oen       = sda_drive;             // 1=drive low, 0=Z (pulled high)
    assign o_finished  = finished;
    assign o_ledr      = nack_detected;         // Red LED on NACK error

    // Byte composition for each register
    always_comb begin
        b0 = {SLAVE_ADDR, 1'b0};           // {ADDR[6:0], R/W=0 (write)}
        b1 = {CFG[cfg_idx].reg_addr, CFG[cfg_idx].reg_data[8]}; // {REG_ADDR[6:0], DATA[8]}
        b2 = CFG[cfg_idx].reg_data[7:0];   // DATA[7:0]
    end

    // Sequential state machine
    always_ff @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state          <= ST_IDLE;
            sclk           <= 1'b1;    // idle: SCL high
            sda_drive      <= 1'b0;    // idle: SDA released (high-Z)
            bit_cnt        <= '0;
            cfg_idx        <= '0;
            shreg          <= '0;
            finished       <= 1'b0;
            nack_detected  <= 1'b0;
        end else begin
            state          <= state_n;
            sclk           <= sclk_n;
            sda_drive      <= sda_drive_n;
            bit_cnt        <= bit_cnt_n;
            cfg_idx        <= cfg_idx_n;
            shreg          <= shreg_n;
            finished       <= finished_n;
            nack_detected  <= nack_detected_n;
        end
    end

    // Combinational state machine
    always_comb begin
        // Defaults: hold values
        state_n          = state;
        sclk_n           = sclk;
        sda_drive_n      = sda_drive;
        bit_cnt_n        = bit_cnt;
        cfg_idx_n        = cfg_idx;
        shreg_n          = shreg;
        finished_n       = finished;
        nack_detected_n  = nack_detected;

        case (state)
            ST_IDLE: begin
                sclk_n           = 1'b1;    // SCL high (idle)
                sda_drive_n      = 1'b0;    // SDA high-Z (idle)
                finished_n       = 1'b0;
                nack_detected_n  = 1'b0;    // clear NACK on idle
                if (i_start) state_n = ST_START_A;
            end

            // START condition: SDA transitions from high to low while SCL=1
            ST_START_A: begin
                sclk_n           = 1'b1;    // keep SCL high
                sda_drive_n      = 1'b1;    // pull SDA low (start condition)
                nack_detected_n  = 1'b0;    // clear NACK at start of new transaction
                state_n          = ST_START_B;
            end
            ST_START_B: begin
                sclk_n      = 1'b0;    // SCL low, ready for data transfer
                sda_drive_n = 1'b1;    // keep SDA low
                state_n     = ST_LOAD_B0;
            end

            // Byte 0: Send slave address + write bit
            ST_LOAD_B0: begin
                shreg_n   = b0;
                bit_cnt_n = 4'd7;
                state_n   = ST_BIT_B0_SET;
            end
            ST_BIT_B0_SET: begin
                sclk_n      = 1'b0;    // SCL low: data can change
                sda_drive_n = (shreg[7] == 1'b0) ? 1'b1 : 1'b0; // 0=drive low, 1=release
                state_n     = ST_BIT_B0_CLK;
            end
            ST_BIT_B0_CLK: begin
                sclk_n  = 1'b1;    // SCL high: data valid
                state_n = ST_ACK0_CHECK;
            end
            ST_ACK0_CHECK: begin
                sclk_n  = 1'b0;    // SCL low, shift register
                shreg_n = {shreg[6:0], 1'b0};
                if (bit_cnt == 0) state_n = ST_ACK0_READ;  // Read ACK after all bits sent
                else begin
                    bit_cnt_n = bit_cnt - 1;
                    state_n   = ST_BIT_B0_SET;
                end
            end
            ST_ACK0_READ: begin
                sda_drive_n = 1'b0;    // release SDA for slave ACK
                sclk_n      = 1'b1;    // SCL high: read ACK
                state_n     = ST_ACK0_DONE;
            end
            ST_ACK0_DONE: begin
                sclk_n      = 1'b0;    // Lower SCL
                // Check for NACK: if i_sdat is high, slave didn't acknowledge
                if (i_sdat) nack_detected_n = 1'b1;  // NACK detected!
                state_n     = ST_LOAD_B1;
            end

            // Byte 1: Send register address + data MSB
            ST_LOAD_B1: begin
                shreg_n   = b1;
                bit_cnt_n = 4'd7;
                sclk_n    = 1'b0;
                state_n   = ST_BIT_B1_SET;
            end
            ST_BIT_B1_SET: begin
                sclk_n      = 1'b0;
                sda_drive_n = (shreg[7] == 1'b0) ? 1'b1 : 1'b0;
                state_n     = ST_BIT_B1_CLK;
            end
            ST_BIT_B1_CLK: begin
                sclk_n  = 1'b1;
                state_n = ST_ACK1_CHECK;
            end
            ST_ACK1_CHECK: begin
                sclk_n  = 1'b0;
                shreg_n = {shreg[6:0], 1'b0};
                if (bit_cnt == 0) state_n = ST_ACK1_READ;
                else begin
                    bit_cnt_n = bit_cnt - 1;
                    state_n   = ST_BIT_B1_SET;
                end
            end
            ST_ACK1_READ: begin
                sda_drive_n = 1'b0;
                sclk_n      = 1'b1;
                state_n     = ST_ACK1_DONE;
            end
            ST_ACK1_DONE: begin
                sclk_n      = 1'b0;
                // Check for NACK: if i_sdat is high, slave didn't acknowledge
                if (i_sdat) nack_detected_n = 1'b1;  // NACK detected!
                state_n     = ST_LOAD_B2;
            end

            // Byte 2: Send data [7:0]
            ST_LOAD_B2: begin
                shreg_n   = b2;
                bit_cnt_n = 4'd7;
                sclk_n    = 1'b0;
                state_n   = ST_BIT_B2_SET;
            end
            ST_BIT_B2_SET: begin
                sclk_n      = 1'b0;
                sda_drive_n = (shreg[7] == 1'b0) ? 1'b1 : 1'b0;
                state_n     = ST_BIT_B2_CLK;
            end
            ST_BIT_B2_CLK: begin
                sclk_n  = 1'b1;
                state_n = ST_ACK2_CHECK;
            end
            ST_ACK2_CHECK: begin
                sclk_n  = 1'b0;
                shreg_n = {shreg[6:0], 1'b0};
                if (bit_cnt == 0) state_n = ST_ACK2_READ;
                else begin
                    bit_cnt_n = bit_cnt - 1;
                    state_n   = ST_BIT_B2_SET;
                end
            end
            ST_ACK2_READ: begin
                sda_drive_n = 1'b0;
                sclk_n      = 1'b1;
                state_n     = ST_ACK2_DONE;
            end
            ST_ACK2_DONE: begin
                sclk_n      = 1'b0;
                // Check for NACK: if i_sdat is high, slave didn't acknowledge
                if (i_sdat) nack_detected_n = 1'b1;  // NACK detected!
                state_n     = ST_STOP_A;
            end

            // STOP condition: SDA transitions from low to high while SCL=1
            ST_STOP_A: begin
                sda_drive_n = 1'b1;    // keep SDA low
                sclk_n      = 1'b1;    // SCL high while SDA low
                state_n     = ST_STOP_B;
            end
            ST_STOP_B: begin
                sda_drive_n = 1'b0;    // release SDA -> STOP (SDA goes high while SCL=1)
                sclk_n      = 1'b1;    // keep SCL high
                state_n     = ST_NEXT;
            end

            // Prepare for next register or finish
            ST_NEXT: begin
                sclk_n      = 1'b1;    // idle: SCL high
                sda_drive_n = 1'b0;    // idle: SDA high-Z
                if (cfg_idx == N_CFG-1) state_n = ST_DONE;
                else begin
                    cfg_idx_n = cfg_idx + 1;
                    state_n   = ST_START_A;    // repeated START
                end
            end

            ST_DONE: begin
                sclk_n      = 1'b1;
                sda_drive_n = 1'b0;
                finished_n  = 1'b1;
            end

            default: state_n = ST_IDLE;
        endcase
    end

endmodule
