	vga_clock u0 (
		.clk_clk             (<connected-to-clk_clk>),             //             clk.clk
		.reset_reset_n       (<connected-to-reset_reset_n>),       //           reset.reset_n
		.vga_clock_25_clk    (<connected-to-vga_clock_25_clk>),    //    vga_clock_25.clk
		.vga_clock_74_25_clk (<connected-to-vga_clock_74_25_clk>), // vga_clock_74_25.clk
		.clock_100k_clk      (<connected-to-clock_100k_clk>),      //      clock_100k.clk
		.clock_12m_clk       (<connected-to-clock_12m_clk>)        //       clock_12m.clk
	);

