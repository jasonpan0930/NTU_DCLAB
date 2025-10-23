module Rsa256Wrapper (
    input         avm_rst,
    input         avm_clk,
    output  [4:0] avm_address, // =0: read; =4: write; =8: check status
    output        avm_read,
    input  [31:0] avm_readdata,
    output        avm_write,
    output [31:0] avm_writedata,
    input         avm_waitrequest,
    // Password protection inputs
    input  [15:0] i_switches,    // SW0~SW15 for password input
    input         i_sw_enable,   // SW17 to enable password protection
    input         i_sw_change    // SW16 to change password
);

localparam RX_BASE     = 0*4; // read received bytes from PC, the received bytes are in avm_readdata
localparam TX_BASE     = 1*4; // write bytes to send to PC, written bytes are stored in avm_writedata
localparam STATUS_BASE = 2*4; // access status register, check if read/write is ready before doing
localparam TX_OK_BIT   = 6; // avm_readdata[6] TX ready
localparam RX_OK_BIT   = 7; // avm_readdata[7] RX ready

// Feel free to design your own FSM!
localparam S_GET_KEY = 0;
localparam S_GET_COUNT = 1;
localparam S_GET_DATA = 2;
localparam S_WAIT_CALCULATE = 3;
localparam S_SEND_DATA = 4;

logic [255:0] n_r, n_w, d_r, d_w, enc_r, enc_w, dec_r, dec_w; // N,d, ciphertext, decrypted result
logic [2:0] state_r, state_w; // 3 bits for 5 states
logic [7:0] package_count_r, package_count_w; // Number of packages to process 
logic [6:0] bytes_counter_r, bytes_counter_w; // counts how many bytes left to process
logic [4:0] avm_address_r, avm_address_w; //
logic avm_read_r, avm_read_w, avm_write_r, avm_write_w;

logic rsa_start_r, rsa_start_w; // start to rsa core
logic rsa_finished; // done from rsa core
logic [255:0] rsa_dec; // result from rsa core

// Password controller signals
logic authenticated;
logic password_mode;

assign avm_address = avm_address_r;
assign avm_read = avm_read_r;
assign avm_write = avm_write_r;

// Only check authentication in S_SEND_DATA state
// If not authenticated (and password enabled), send zeros; otherwise send real data
assign avm_writedata = (state_r == S_SEND_DATA && i_sw_enable && !authenticated) ? 
                        8'd0 : rsa_dec[(bytes_counter_r*8)+7-:8];

Rsa256Core rsa256_core(
    .i_clk(avm_clk),
    .i_rst(avm_rst),
    .i_start(rsa_start_r),
    .i_a(enc_r),
    .i_d(d_r),
    .i_n(n_r),
    .o_a_pow_d(rsa_dec),
    .o_finished(rsa_finished)
);

// Password Controller Instance
PasswordController passwd_ctrl (
    .i_clk(avm_clk),
    .i_rst(avm_rst),
    .i_switches(i_switches),
    .i_sw_enable(i_sw_enable),
    .i_sw_change(i_sw_change),
    .o_authenticated(authenticated),
    .o_password_mode(password_mode)
);

task StartRead;
    input [4:0] addr;
    begin
        avm_read_w = 1;
        avm_write_w = 0;
        avm_address_w = addr;
    end
endtask
task StartWrite;
    input [4:0] addr;
    begin
        avm_read_w = 0;
        avm_write_w = 1;
        avm_address_w = addr;
    end
endtask

always_comb begin
    // Default values - keep current values unless changed
    n_w = n_r;
    d_w = d_r;
    enc_w = enc_r;
    dec_w = dec_r;
    state_w = state_r;
    package_count_w = package_count_r;
    bytes_counter_w = bytes_counter_r;
    avm_address_w = avm_address_r;
    avm_read_w = avm_read_r;
    avm_write_w = avm_write_r;
    rsa_start_w = rsa_start_r;
    
    case (state_r)
        S_GET_KEY: begin
            // Reading key data: first 32 bytes = N, next 32 bytes = d
            if (bytes_counter_r >= 32) begin
                // Reading N (first 32 bytes)
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[RX_OK_BIT]) begin
                            // RX ready, read data
                            StartRead(RX_BASE);
                        end else begin
                            // Wait for RX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Reading from RX_BASE, shift in byte
                        n_w = {n_r[247:0], avm_readdata[7:0]};
                        bytes_counter_w = bytes_counter_r - 1;
                        StartRead(STATUS_BASE);
                    end
                end
            end else if (bytes_counter_r >= 1) begin
                // Reading d (next 32 bytes)
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[RX_OK_BIT]) begin
                            // RX ready, read data
                            StartRead(RX_BASE);
                        end else begin
                            // Wait for RX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Reading from RX_BASE, shift in byte
                        d_w = {d_r[247:0], avm_readdata[7:0]};
                        bytes_counter_w = bytes_counter_r - 1;
                        StartRead(STATUS_BASE);
                    end
                end
            end else begin
                // Last byte of d - read it then move to get data
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[RX_OK_BIT]) begin
                            // RX ready, read last byte
                            StartRead(RX_BASE);
                        end else begin
                            // Wait for RX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Reading from RX_BASE, shift in last byte
                        d_w = {d_r[247:0], avm_readdata[7:0]};
                        state_w = S_GET_COUNT;
                        StartRead(STATUS_BASE); // Prepare to read package count
                    end
                end
            end
        end
        
        S_GET_COUNT: begin
            // Read package count (1 byte)
            if (!avm_waitrequest) begin
                if (avm_address_r == STATUS_BASE) begin
                    // Checking status
                    if (avm_readdata[RX_OK_BIT]) begin
                        // RX ready, read package count
                        StartRead(RX_BASE);
                    end else begin
                        // Wait for RX ready
                        StartRead(STATUS_BASE);
                    end
                end else begin
                    // Reading from RX_BASE, get package count
                    package_count_w = avm_readdata[7:0];
                    state_w = S_GET_DATA;
                    bytes_counter_w = 31; // 32 bytes for encrypted data
                    StartRead(STATUS_BASE); // Prepare for next read
                end
            end
        end
        
        S_GET_DATA: begin
            // Reading encrypted data (32 bytes)
            if (bytes_counter_r >= 1) begin
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[RX_OK_BIT]) begin
                            // RX ready, read data
                            StartRead(RX_BASE);
                        end else begin
                            // Wait for RX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Reading from RX_BASE, shift in byte
                        enc_w = {enc_r[247:0], avm_readdata[7:0]};
                        bytes_counter_w = bytes_counter_r - 1;
                        StartRead(STATUS_BASE);
                    end
                end
            end else begin
                // Last byte - read it and start RSA calculation
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[RX_OK_BIT]) begin
                            // RX ready, read last byte
                            StartRead(RX_BASE);
                        end else begin
                            // Wait for RX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Reading from RX_BASE, shift in last byte and start calculation
                        enc_w = {enc_r[247:0], avm_readdata[7:0]};
                        state_w = S_WAIT_CALCULATE;
                        rsa_start_w = 1'b1;
                    end
                end
            end
        end
        
        S_WAIT_CALCULATE: begin
            // Wait for RSA calculation to finish
            rsa_start_w = 1'b0;
            if (rsa_finished) begin
                state_w = S_SEND_DATA;
                bytes_counter_w = 30; // 31 bytes to send
                StartRead(STATUS_BASE); // Prepare for sending
            end
        end
        
        S_SEND_DATA: begin
            // Send decrypted data (31 bytes)
            if (bytes_counter_r >= 1) begin
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[TX_OK_BIT]) begin
                            // TX ready, write data
                            StartWrite(TX_BASE);
                        end else begin
                            // Wait for TX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Just finished writing to TX_BASE, move to next byte
                        bytes_counter_w = bytes_counter_r - 1;
                        StartRead(STATUS_BASE);
                    end
                end
            end else begin
                // Last byte - send it and go back to get next data
                if (!avm_waitrequest) begin
                    if (avm_address_r == STATUS_BASE) begin
                        // Checking status
                        if (avm_readdata[TX_OK_BIT]) begin
                            // TX ready, write last byte
                            StartWrite(TX_BASE);
                        end else begin
                            // Wait for TX ready
                            StartRead(STATUS_BASE);
                        end
                    end else begin
                        // Just finished writing last byte
                        package_count_w = package_count_r - 1;
                        if (package_count_r == 1) begin
                            // Last package sent, go back to get new key
                            state_w = S_GET_KEY;
                            bytes_counter_w = 63; // 64 bytes for key
                        end else begin
                            // More packages to process
                            state_w = S_GET_DATA;
                            bytes_counter_w = 31; // 32 bytes for next encrypted data
                        end
                        StartRead(STATUS_BASE); // Prepare for next read
                    end
                end
            end
        end
    endcase
end

always_ff @(posedge avm_clk or posedge avm_rst) begin
    if (avm_rst) begin
        n_r <= 0;
        d_r <= 0;
        enc_r <= 0;
        dec_r <= 0;
        avm_address_r <= STATUS_BASE;
        avm_read_r <= 1;
        avm_write_r <= 0;
        state_r <= S_GET_KEY;
        package_count_r <= 0;
        bytes_counter_r <= 63;
        rsa_start_r <= 0;
    end else begin
        n_r <= n_w;
        d_r <= d_w;
        enc_r <= enc_w;
        dec_r <= dec_w;
        avm_address_r <= avm_address_w;
        avm_read_r <= avm_read_w;
        avm_write_r <= avm_write_w;
        state_r <= state_w;
        package_count_r <= package_count_w;
        bytes_counter_r <= bytes_counter_w;
        rsa_start_r <= rsa_start_w;
    end
end

endmodule

module PasswordController (
    input         i_clk,
    input         i_rst,
    // Switch inputs
    input  [15:0] i_switches,      // SW0~SW15 for password input
    input         i_sw_enable,     // SW17 to enable password protection
    input         i_sw_change,     // SW16 to change password
    // Output
    output logic  o_authenticated, // Authentication status
    output logic  o_password_mode  // 1 = in password change mode, 0 = normal operation
);

localparam DEFAULT_PASSWORD = 16'b0000_0000_0000_0000;

typedef enum logic [0:0] {
    NORMAL        = 1'b0,
    CHANGE_PASSWD = 1'b1
} passwd_state_t;

passwd_state_t state_r, state_w;
logic [15:0] password_r, password_w;

assign o_password_mode = (state_r == CHANGE_PASSWD);

// Continuous authentication check
always_comb begin
    if (!i_sw_enable) begin
        // Password protection disabled → always authenticated
        o_authenticated = 1'b1;
    end else begin
        // Password protection enabled → check if current switches match password
        o_authenticated = (i_switches == password_r);
    end
end

always_comb begin
    state_w = state_r;
    password_w = password_r;
    
    // Check for password change mode entry
    if (i_sw_enable && i_sw_change && (i_switches == password_r) && (state_r != CHANGE_PASSWD)) begin
        state_w = CHANGE_PASSWD;
        password_w = i_switches; // Start tracking new password
    end else begin
        case (state_r)
            NORMAL: begin
                // Just wait for password change request
            end
            
            CHANGE_PASSWD: begin
                if (!i_sw_change) begin
                    // SW16 released → save new password
                    password_w = i_switches;
                    state_w = NORMAL;
                end else begin
                    // SW16 still high → keep updating password preview
                    password_w = i_switches;
                    state_w = CHANGE_PASSWD;
                end
            end
            
            default: state_w = NORMAL;
        endcase
    end
end

always_ff @(posedge i_clk or posedge i_rst) begin
    if (i_rst) begin
        state_r <= NORMAL;
        password_r <= DEFAULT_PASSWORD;
    end else begin
        state_r <= state_w;
        password_r <= password_w;
    end
end

endmodule
