module imu_parser (
    input wire clk,
    input wire reset_n,
    input wire rx_dv,
    input wire [7:0] rx_byte,
    
    output reg signed [31:0] ax,
    output reg signed [31:0] ay,
    output reg signed [31:0] az,
    output reg signed [31:0] gx,
    output reg signed [31:0] gy,
    output reg signed [31:0] gz,
    output reg data_ready
);

    typedef enum logic [2:0] {
        IDLE,
        READ_NUM,
        WAIT_SEP
    } state_t;

    state_t state = IDLE;
    
    reg signed [31:0] current_val;
    reg [2:0] val_index; // 0=ax, 1=ay, ... 5=gz
    reg is_negative;

    // Temporary storage for incomplete packet
    reg signed [31:0] temp_ax, temp_ay, temp_az, temp_gx, temp_gy, temp_gz;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= IDLE;
            ax <= 0; ay <= 0; az <= 0;
            gx <= 0; gy <= 0; gz <= 0;
            data_ready <= 0;
            val_index <= 0;
            current_val <= 0;
            is_negative <= 0;
        end else begin
            data_ready <= 0; // Pulse only
            
            if (rx_dv) begin
                case (state)
                    IDLE: begin
                        if (rx_byte == "$") begin
                            state <= READ_NUM;
                            val_index <= 0;
                            current_val <= 0;
                            is_negative <= 0;
                        end
                    end

                    READ_NUM: begin
                        if (rx_byte == "-") begin
                            is_negative <= 1;
                        end 
                        else if (rx_byte >= "0" && rx_byte <= "9") begin
                            current_val <= (current_val * 10) + (rx_byte - "0");
                        end 
                        else if (rx_byte == "," || rx_byte == 10 || rx_byte == 13) begin // Comma or Newline
                            // Store to temp with sign applied
                            case (val_index)
                                0: temp_ax <= is_negative ? -current_val : current_val;
                                1: temp_ay <= is_negative ? -current_val : current_val;
                                2: temp_az <= is_negative ? -current_val : current_val;
                                3: temp_gx <= is_negative ? -current_val : current_val;
                                4: temp_gy <= is_negative ? -current_val : current_val;
                                5: temp_gz <= is_negative ? -current_val : current_val;
                            endcase

                            // Prepare for next
                            if (rx_byte == ",") begin
                                current_val <= 0;
                                is_negative <= 0;
                                val_index <= val_index + 1;
                                state <= READ_NUM; 
                            end else if (rx_byte == 10 || rx_byte == 13) begin // LF or CR
                                // Packet complete, update outputs
                                ax <= temp_ax;
                                ay <= temp_ay;
                                az <= temp_az;
                                gx <= temp_gx;
                                gy <= temp_gy;
                                // For the last value (gz), we use the one calculated right now
                                gz <= is_negative ? -current_val : current_val;
                                
                                current_val <= 0;
                                is_negative <= 0;
                                data_ready <= 1;
                                state <= IDLE;
                            end
                        end
                    end
                endcase
            end
        end
    end

endmodule
