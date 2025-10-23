module AudPlayer(
    input i_rst_n,
    input i_bclk,           // 位時鐘
    input i_daclrck,        // 左右聲道時鐘
    input i_en,             // 使能信號
    input [15:0] i_dac_data, // 來自DSP的音頻數據 
    output o_aud_dacdat     // 輸出到CODEC的數據
);

logic [15:0] shift_reg;     // 移位寄存器
logic [4:0] bit_counter;    // 位計數器
logic prev_daclrck;         // 前一個DACLRCK狀態

always_ff @(posedge i_bclk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        shift_reg <= 16'd0;
        bit_counter <= 5'd0;
        prev_daclrck <= 1'b0;
    end
    else if (i_en) begin
        // 檢測DACLRCK的變化
        if (i_daclrck != prev_daclrck) begin
            // DACLRCK改變時，載入新的數據
            shift_reg <= i_dac_data;
            bit_counter <= 5'd0;
        end
        else begin
            // 在BCLK的每個時鐘週期移位輸出
            if (bit_counter < 16) begin
                shift_reg <= {shift_reg[14:0], 1'b0};
                bit_counter <= bit_counter + 1;
            end
        end
        prev_daclrck <= i_daclrck;
    end
end

// 輸出MSB
assign o_aud_dacdat = shift_reg[15];

endmodule