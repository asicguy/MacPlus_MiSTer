module tb;
  logic                clk_sys;
  logic                pll_locked;
  logic                RESET;

  tri [45:0]           HPS_BUS;
  logic                CLK_VIDEO;

   //Multiple resolutions are supported using different CE_PIXEL rates.
   //Must be based on CLK_VIDEO
  logic                CE_PIXEL;

   //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
  logic [7:0]          VIDEO_ARX;
  logic [7:0]          VIDEO_ARY;

  logic [7:0]          VGA_R;
  logic [7:0]          VGA_G;
  logic [7:0]          VGA_B;
  logic                VGA_HS;
  logic                VGA_VS;
  logic                VGA_DE; // = ~(VBlank | HBlank)
  logic                VGA_F1;
  logic [1:0]          VGA_SL;

  logic                LED_USER; // 1 - ON; 0 - OFF.

  // b[1]: 0 - LED status is system status OR'd with b[0]
  //       1 - LED status is controled solely by b[0]
  // hint: supply 2'b00 to let the system control the LED.
  logic [1:0]          LED_POWER;
  logic [1:0]          LED_DISK;

  // I/O board button press simulation (active high)
  // b[1]: user button
  // b[0]: osd button
  logic [1:0]          BUTTONS;

  logic [15:0]         AUDIO_L;
  logic [15:0]         AUDIO_R;
  logic                AUDIO_S; // 1 - signed audio samples; 0 - unsigned
  logic [1:0]          AUDIO_MIX; // 0 - no mix; 1 - 25%; 2 - 50%; 3 - 100% (mono)

  //ADC
  tri [3:0]            ADC_BUS;

  //SD-SPI
  logic                SD_SCK;
  logic                SD_MOSI;
  logic                SD_MISO;
  logic                SD_CS;
  logic                SD_CD;

  //SDRAM interface with lower latency - cpu interface
  logic [24:0]         sdram_addr;
  logic [15:0]         sdram_din;
  logic [1:0]          sdram_ds;
  logic                sdram_we;
  logic                sdram_oe;
  logic [15:0]         sdram_out;
  logic                cep;

  logic                UART_CTS;
  logic                UART_RTS;
  logic                UART_RXD;
  logic                UART_TXD;
  logic                UART_DTR;
  logic                UART_DSR;

  // Memory -- probably not the best way to do this.
  bit [7:0] memory[67108864];

  // Open-drain User port.
  // 0 - D+/RX
  // 1 - D-/TX
  // 2..6 - USR2..USR6
  // Set USER_OUT to 1 to read from USER_IN.
  logic [6:0]          USER_IN;
  logic [6:0]          USER_OUT;

  logic                OSD_STATUS;

  assign HPS_BUS = 'z;
  assign ADC_BUS = 'z;
  assign SDRAM_DQ = 'z;

  initial begin
    clk_sys          = '0;
    forever clk_sys  = #7.576 ~clk_sys;
  end

  initial begin
    pll_locked  = '0;
    RESET       = '0;

    repeat (100) @(posedge clk_sys) pll_locked = '1;
    repeat (100) @(posedge clk_sys) RESET = '1;
    repeat (100) @(posedge clk_sys) RESET = '0;

  end

  MacPlus_subsys
    #
    (
     .DELAY      (20'hFF),
     .RESET_BITS (8)
     )
  u_emu
    (
     .*
     );

  int r1, c1;
  // SDRAM model
  initial begin
    r1 = $fopen("boot.rom","rb");
    c1 = $fread(memory,r1,2097152*2); // map rom to sdram word address $200000 - $20ffff
  end

  always @(posedge clk_sys) begin
    if (sdram_we) begin
      if (sdram_ds[0]) memory[{sdram_addr, 1'b0}] <= sdram_din[7:0];
      if (sdram_ds[1]) memory[{sdram_addr, 1'b1}] <= sdram_din[15:8];
    end
    //if (sdram_oe) begin
      sdram_out <= {memory[{sdram_addr, 1'b1}], memory[{sdram_addr, 1'b0}]};
    //end
  end


endmodule // tb
