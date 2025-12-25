
module vga_clock (
	clk_clk,
	reset_reset_n,
	vga_clock_25_clk,
	vga_clock_74_25_clk,
	clock_100k_clk,
	clock_12m_clk);	

	input		clk_clk;
	input		reset_reset_n;
	output		vga_clock_25_clk;
	output		vga_clock_74_25_clk;
	output		clock_100k_clk;
	output		clock_12m_clk;
endmodule
