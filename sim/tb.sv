module tb;
  logic CLK_50M;
  logic RESET;

  tri   [45:0] HPS_BUS;
  logic CLK_VIDEO;

   //Multiple resolutions are supported using different CE_PIXEL rates.
   //Must be based on CLK_VIDEO
   logic        CE_PIXEL;

   //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
   logic [7:0]  VIDEO_ARX;
   logic [7:0]  VIDEO_ARY;

   logic [7:0]  VGA_R;
   logic [7:0]  VGA_G;
   logic [7:0]  VGA_B;
   logic        VGA_HS;
   logic        VGA_VS;
   logic        VGA_DE; // = ~(VBlank | HBlank)
   logic        VGA_F1;
   logic [1:0]  VGA_SL;

   logic        LED_USER; // 1 - ON; 0 - OFF.

   // b[1]: 0 - LED status is system status OR'd with b[0]
   //       1 - LED status is controled solely by b[0]
   // hint: supply 2'b00 to let the system control the LED.
   logic [1:0]  LED_POWER;
   logic [1:0]  LED_DISK;

   // I/O board button press simulation (active high)
   // b[1]: user button
   // b[0]: osd button
   logic [1:0]  BUTTONS;

   logic [15:0] AUDIO_L;
   logic [15:0] AUDIO_R;
   logic        AUDIO_S; // 1 - signed audio samples; 0 - unsigned
   logic [1:0]  AUDIO_MIX; // 0 - no mix; 1 - 25%; 2 - 50%; 3 - 100% (mono)

   //ADC
   tri [3:0]   ADC_BUS;

   //SD-SPI
   logic        SD_SCK;
   logic        SD_MOSI;
   logic         SD_MISO;
   logic        SD_CS;
   logic         SD_CD;

   //High latency DDR3 RAM interface
   //Use for non-critical time purposes
   logic        DDRAM_CLK;
   logic         DDRAM_BUSY;
   logic [7:0]  DDRAM_BURSTCNT;
   logic [28:0] DDRAM_ADDR;
   logic [63:0]  DDRAM_DOUT;
   logic         DDRAM_DOUT_READY;
   logic        DDRAM_RD;
   logic [63:0] DDRAM_DIN;
   logic [7:0]  DDRAM_BE;
   logic        DDRAM_WE;

   //SDRAM interface with lower latency
   logic        SDRAM_CLK;
   logic        SDRAM_CKE;
   logic [12:0] SDRAM_A;
   logic [1:0]  SDRAM_BA;
   tri [15:0]  SDRAM_DQ;
   logic        SDRAM_DQML;
   logic        SDRAM_DQMH;
   logic        SDRAM_nCS;
   logic        SDRAM_nCAS;
   logic        SDRAM_nRAS;
   logic        SDRAM_nWE;

   logic         UART_CTS;
   logic        UART_RTS;
   logic         UART_RXD;
   logic        UART_TXD;
   logic        UART_DTR;
   logic         UART_DSR;

   // Open-drain User port.
   // 0 - D+/RX
   // 1 - D-/TX
   // 2..6 - USR2..USR6
   // Set USER_OUT to 1 to read from USER_IN.
   logic [6:0]   USER_IN;
   logic [6:0]  USER_OUT;

  logic         OSD_STATUS;

  assign HPS_BUS = 'z;
  assign ADC_BUS = 'z;
  assign SDRAM_DQ = 'z;

  initial begin
    CLK_50M          = '0;
    forever CLK_50M  = #10 ~CLK_50M;
  end

  initial begin
    RESET       = '0;

    repeat (100) @(posedge CLK_50M) RESET = '1;
    repeat (100) @(posedge CLK_50M) RESET = '0;

  end

  emu
    #
    (
     .DELAY      (20'hFF),
     .RESET_BITS (8)
     )
  u_emu
    (
     .*
     );

endmodule // tb
