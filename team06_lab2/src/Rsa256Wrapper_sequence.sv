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

// Password protection
localparam DEFAULT_PASSWORD = 16'b0000_0000_0000_0000; // 預設密碼 (SW15~SW0 = 全部0)

// Feel free to design your own FSM!
localparam S_CHECK_PASSWORD = 0;
localparam S_CHANGE_PASSWORD = 1;
localparam S_GET_KEY = 2;
localparam S_GET_COUNT = 3;
localparam S_GET_DATA = 4;
localparam S_WAIT_CALCULATE = 5;
localparam S_SEND_DATA = 6;

logic [255:0] n_r, n_w, d_r, d_w, enc_r, enc_w, dec_r, dec_w; // N,d, ciphertext, decrypted result
logic [2:0] state_r, state_w; // 3 bits for 7 states
logic [7:0] package_count_r, package_count_w; // Number of packages to process 
logic [6:0] bytes_counter_r, bytes_counter_w; // counts how many bytes left to process
logic [4:0] avm_address_r, avm_address_w; //
logic avm_read_r, avm_read_w, avm_write_r, avm_write_w;
logic [15:0] password_r, password_w; // Current password
logic authenticated_r, authenticated_w; // Track if password was correct

logic rsa_start_r, rsa_start_w; // start to rsa core
logic rsa_finished; // done from rsa core
logic [255:0] rsa_dec; // result from rsa core

assign avm_address = avm_address_r;
assign avm_read = avm_read_r;
assign avm_write = avm_write_r;
// If authenticated or password not enabled, send real data; otherwise send 0
assign avm_writedata = (authenticated_r || !i_sw_enable) ? rsa_dec[(bytes_counter_r*8)+7-:8] : 8'd0;

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
    password_w = password_r;
    authenticated_w = authenticated_r;
    
    // Global check: if password is enabled, correct, and change switch is on, enter password change mode
    if (i_sw_enable && i_sw_change && (i_switches == password_r) && (state_r != S_CHANGE_PASSWORD)) begin
        state_w = S_CHANGE_PASSWORD;
        password_w = i_switches; // Start tracking the new password
        avm_read_w = 0;
        avm_write_w = 0;
        avm_address_w = STATUS_BASE;
        rsa_start_w = 0; // Stop any ongoing RSA calculation
    end else begin
        case (state_r)
                S_CHECK_PASSWORD: begin
                    // 檢查密碼保護是否啟用（key已經在S_GET_KEY讀取完畢）
                    if (!i_sw_enable) begin
                        // SW17 沒有被拉起，直接進入正常流程
                        authenticated_w = 1'b1; // Password not required, mark as authenticated
                        state_w = S_GET_COUNT;
                        StartRead(STATUS_BASE);
                    end else begin
                        // SW17 被拉起，需要檢查密碼
                        if (i_switches == password_r) begin
                            // 密碼正確
                            authenticated_w = 1'b1; // Mark as authenticated
                            if (i_sw_change) begin
                                // SW16 被拉起，進入修改密碼狀態
                                state_w = S_CHANGE_PASSWORD;
                                avm_read_w = 0;
                                avm_write_w = 0;
                                avm_address_w = STATUS_BASE;
                            end else begin
                                // SW16 沒有被拉起，進入正常流程
                                state_w = S_GET_COUNT;
                                StartRead(STATUS_BASE);
                            end
                        end else begin
                            // 密碼錯誤，但仍然進入正常流程（只是會傳送0而不是真實資料）
                            authenticated_w = 1'b0; // Mark as NOT authenticated
                            state_w = S_GET_COUNT;
                            StartRead(STATUS_BASE);
                        end
                    end
                end
                
                S_CHANGE_PASSWORD: begin
                    // 修改密碼狀態 - 等待 SW16 釋放
                    if (!i_sw_change) begin
                        // SW16 釋放，保存新密碼並返回
                        password_w = i_switches;
                        state_w = S_GET_KEY; // 回到讀取金鑰狀態重新開始
                        bytes_counter_w = 63;
                        StartRead(STATUS_BASE);
                    end else begin
                        // SW16 仍然被拉起，保持在修改密碼狀態，持續更新密碼預覽
                        state_w = S_CHANGE_PASSWORD;
                        password_w = i_switches; // 實時更新新密碼
                        avm_read_w = 0;
                        avm_write_w = 0;
                        avm_address_w = STATUS_BASE;
                    end
                end
                
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
                        // Last byte of d - read it then move to check password
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
                                state_w = S_CHECK_PASSWORD; // Go to password check after getting key
                                avm_read_w = 0;
                                avm_write_w = 0;
                                avm_address_w = STATUS_BASE;
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
                                // Last package sent, go back to S_GET_KEY
                                state_w = S_GET_KEY;
                                authenticated_w = 1'b0; // Reset authentication status
                                bytes_counter_w = 63; // 64 bytes for key
                                StartRead(STATUS_BASE);
                            end else begin
                                    // More packages to process
                                    state_w = S_GET_DATA;
                                    bytes_counter_w = 31; // 32 bytes for next encrypted data
                                    StartRead(STATUS_BASE); // Prepare for next read
                                end
                            end
                        end
                end
            end
        endcase
    end
end

always_ff @(posedge avm_clk or posedge avm_rst) begin
    if (avm_rst) begin
        n_r <= 0;
        d_r <= 0;
        enc_r <= 0;
        dec_r <= 0;
        avm_address_r <= STATUS_BASE;
        avm_read_r <= 1;  // 初始化為1，準備讀取
        avm_write_r <= 0;
        state_r <= S_GET_KEY;  // 從讀取金鑰狀態開始
        package_count_r <= 0;
        bytes_counter_r <= 63;  // 初始化為63，為S_GET_KEY狀態做準備
        rsa_start_r <= 0;
        password_r <= DEFAULT_PASSWORD; // 初始化為預設密碼
        authenticated_r <= 0; // Not authenticated by default
    end else begin
        // if(i_sw_change && i_sw_enable && (i_switches == password_r)) state_r <= S_CHANGE_PASSWORD;
        // else state_r <= state_w;
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
        password_r <= password_w;
        authenticated_r <= authenticated_w;
    end
end

endmodule
