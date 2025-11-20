	component vga_clock is
		port (
			clk_clk             : in  std_logic := 'X'; -- clk
			reset_reset_n       : in  std_logic := 'X'; -- reset_n
			vga_clock_25_clk    : out std_logic;        -- clk
			vga_clock_74_25_clk : out std_logic         -- clk
		);
	end component vga_clock;

	u0 : component vga_clock
		port map (
			clk_clk             => CONNECTED_TO_clk_clk,             --             clk.clk
			reset_reset_n       => CONNECTED_TO_reset_reset_n,       --           reset.reset_n
			vga_clock_25_clk    => CONNECTED_TO_vga_clock_25_clk,    --    vga_clock_25.clk
			vga_clock_74_25_clk => CONNECTED_TO_vga_clock_74_25_clk  -- vga_clock_74_25.clk
		);

