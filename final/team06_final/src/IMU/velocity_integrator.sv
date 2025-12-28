module velocity_integrator (
    input wire clk,
    input wire reset_n,
    input wire en, // data_ready pulse
    input wire signed [31:0] ax,
    input wire signed [31:0] ay,
    input wire signed [31:0] az,
    input wire signed [31:0] gx,
    input wire signed [31:0] gy,
    input wire signed [31:0] gz,
    
    output reg signed [31:0] vx,
    output reg signed [31:0] vy,
    output reg signed [31:0] vz,
    output reg signed [31:0] gx_out,
    output reg signed [31:0] gy_out,
    output reg signed [31:0] gz_out,
    output reg calibrated
);

    // Calibration settings
    localparam CALIB_SAMPLES = 2048; // ~10s at 200Hz
    localparam CALIB_SHIFT = 11;   // 2^11 = 2048
    localparam THRESHOLD = 8;     // Small deadzone to stop "crawling" drift
    localparam STABILITY_PERIOD = 40;  // 0.2s at 200Hz

    // Internal state
    typedef enum logic [1:0] {
        ST_CALIB,
        ST_RUN
    } state_t;
    
    state_t state;
    
    // Accumulators for calibration
    reg signed [63:0] acc_x, acc_y, acc_z;
    reg signed [63:0] acc_gx, acc_gy, acc_gz;
    reg [15:0] sample_count;
    reg [15:0] stability_timer;
    
    // Calculated Offsets (Gravity + Bias)
    reg signed [31:0] offset_x, offset_y, offset_z;
    reg signed [31:0] offset_gx, offset_gy, offset_gz;
    
    // Velocity accumulators (Internal high precision)
    reg signed [63:0] v_acc_x, v_acc_y, v_acc_z;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= ST_CALIB;
            sample_count <= 0;
            acc_x <= 0; acc_y <= 0; acc_z <= 0;
            acc_gx <= 0; acc_gy <= 0; acc_gz <= 0;
            offset_x <= 0; offset_y <= 0; offset_z <= 0;
            offset_gx <= 0; offset_gy <= 0; offset_gz <= 0;
            vx <= 0; vy <= 0; vz <= 0;
            gx_out <= 0; gy_out <= 0; gz_out <= 0;
            v_acc_x <= 0; v_acc_y <= 0; v_acc_z <= 0;
            stability_timer <= 0;
            calibrated <= 0;
        end else begin
            if (en) begin
                case (state)
                    ST_CALIB: begin
                        acc_x <= acc_x + ax;
                        acc_y <= acc_y + ay;
                        acc_z <= acc_z + az;
                        acc_gx <= acc_gx + gx;
                        acc_gy <= acc_gy + gy;
                        acc_gz <= acc_gz + gz;
                        sample_count <= sample_count + 1;
                        
                        if (sample_count == CALIB_SAMPLES - 1) begin
                            // Calculate average
                            offset_x <= acc_x >>> CALIB_SHIFT;
                            offset_y <= acc_y >>> CALIB_SHIFT;
                            offset_z <= acc_z >>> CALIB_SHIFT;
                            offset_gx <= acc_gx >>> CALIB_SHIFT;
                            offset_gy <= acc_gy >>> CALIB_SHIFT;
                            offset_gz <= acc_gz >>> CALIB_SHIFT;
                            
                            v_acc_x <= 0;
                            v_acc_y <= 0;
                            v_acc_z <= 0;
                            stability_timer <= 0;
                            calibrated <= 1;
                            state <= ST_RUN;
                        end
                    end
                    
                    ST_RUN: begin
                        // Difference from offset
                        logic signed [31:0] dx, dy, dz;
                        dx = ax - offset_x;
                        dy = ay - offset_y;
                        dz = az - offset_z;

                        // Integrate with tiny deadzone
                        if (dx > THRESHOLD || dx < -THRESHOLD)
                            v_acc_x <= v_acc_x + dx;
                        
                        if (dy > THRESHOLD || dy < -THRESHOLD)
                            v_acc_y <= v_acc_y + dy;
                            
                        if (dz > THRESHOLD || dz < -THRESHOLD)
                            v_acc_z <= v_acc_z + dz;
                        
                        // Smart Reset: If stationary for 20 seconds, reset velocity to zero
                        if (dx <= THRESHOLD && dx >= -THRESHOLD &&
                            dy <= THRESHOLD && dy >= -THRESHOLD &&
                            dz <= THRESHOLD && dz >= -THRESHOLD) 
                        begin
                            if (stability_timer >= STABILITY_PERIOD - 1) begin
                                v_acc_x <= 0;
                                v_acc_y <= 0;
                                v_acc_z <= 0;
                                stability_timer <= 0;
                            end else begin
                                stability_timer <= stability_timer + 1;
                            end
                        end else begin
                            stability_timer <= 0; // Reset timer if movement detected
                        end
                        
                        // Output (lower 32 bits)
                        vx <= v_acc_x[31:0];
                        vy <= v_acc_y[31:0];
                        vz <= v_acc_z[31:0];

                        gx_out <= gx - offset_gx;
                        gy_out <= gy - offset_gy;
                        gz_out <= gz - offset_gz;
                    end
                endcase
            end
        end
    end

endmodule
