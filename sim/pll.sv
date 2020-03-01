module pll
  (
   input wire   refclk,
   input wire   rst,
   output logic outclk_0,
   output logic pll_locked
   );

  initial begin
    outclk_0  = '0;
    forever outclk_0 = #8 ~outclk_0;
  end

  initial begin
    pll_locked  = '0;
    repeat (10) @(posedge outclk_0);
    pll_locked = '1;
  end
endmodule // pll
