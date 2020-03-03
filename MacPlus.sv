//============================================================================
//  Macintosh Plus
//
//  Port to MiSTer
//  Copyright (C) 2017-2019 Sorgelig
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

module emu
  #
  (
   parameter     DELAY      = 20'hFFFFF,
   parameter     RESET_BITS = 16
   )
 (
   //Master input clock
   input         CLK_50M,

   //Async reset from top-level module.
   //Can be used as initial reset.
   input         RESET,

   //Must be passed to hps_io module
   inout [45:0]  HPS_BUS,

   //Base video clock. Usually equals to CLK_SYS.
   output        CLK_VIDEO,

   //Multiple resolutions are supported using different CE_PIXEL rates.
   //Must be based on CLK_VIDEO
   output        CE_PIXEL,

   //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
   output [7:0]  VIDEO_ARX,
   output [7:0]  VIDEO_ARY,

   output [7:0]  VGA_R,
   output [7:0]  VGA_G,
   output [7:0]  VGA_B,
   output        VGA_HS,
   output        VGA_VS,
   output        VGA_DE, // = ~(VBlank | HBlank)
   output        VGA_F1,
   output [1:0]  VGA_SL,

   output        LED_USER, // 1 - ON, 0 - OFF.

   // b[1]: 0 - LED status is system status OR'd with b[0]
   //       1 - LED status is controled solely by b[0]
   // hint: supply 2'b00 to let the system control the LED.
   output [1:0]  LED_POWER,
   output [1:0]  LED_DISK,

   // I/O board button press simulation (active high)
   // b[1]: user button
   // b[0]: osd button
   output [1:0]  BUTTONS,

   output [15:0] AUDIO_L,
   output [15:0] AUDIO_R,
   output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
   output [1:0]  AUDIO_MIX, // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

   //ADC
   inout [3:0]   ADC_BUS,

   //SD-SPI
   output        SD_SCK,
   output        SD_MOSI,
   input         SD_MISO,
   output        SD_CS,
   input         SD_CD,

   //High latency DDR3 RAM interface
   //Use for non-critical time purposes
   output        DDRAM_CLK,
   input         DDRAM_BUSY,
   output [7:0]  DDRAM_BURSTCNT,
   output [28:0] DDRAM_ADDR,
   input [63:0]  DDRAM_DOUT,
   input         DDRAM_DOUT_READY,
   output        DDRAM_RD,
   output [63:0] DDRAM_DIN,
   output [7:0]  DDRAM_BE,
   output        DDRAM_WE,

   //SDRAM interface with lower latency
   output        SDRAM_CLK,
   output        SDRAM_CKE,
   output [12:0] SDRAM_A,
   output [1:0]  SDRAM_BA,
   inout [15:0]  SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nCS,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nWE,

   input         UART_CTS,
   output        UART_RTS,
   input         UART_RXD,
   output        UART_TXD,
   output        UART_DTR,
   input         UART_DSR,

   // Open-drain User port.
   // 0 - D+/RX
   // 1 - D-/TX
   // 2..6 - USR2..USR6
   // Set USER_OUT to 1 to read from USER_IN.
   input [6:0]   USER_IN,
   output [6:0]  USER_OUT,

   input         OSD_STATUS
   );

  logic          clk_sys;
  logic          pll_locked;
  logic [24:0]   sdram_addr;
  logic [15:0]   sdram_din;
  logic [1:0]    sdram_ds;
  logic          sdram_we;
  logic          sdram_oe;
  logic [15:0]   sdram_out;
  logic          cep;

  // Clocks
  pll pll
    (
     .refclk        (CLK_50M),
     .rst           (0),
     .outclk_0      (clk_sys),
     //.outclk_1    (clk_sys_2),
     .locked        (pll_locked)
     );

  MacPlus_subsys
    #
    (
     .DELAY           (DELAY),
     .RESET_BITS      (RESET_BITS)
     )
  u0
    (
     //Master input clock -- From PLL at the top level
     .clk_sys         (clk_sys),
     .pll_locked      (pll_locked),

     //Async reset from top-level module.
     //Can be used as initial reset.
     .RESET           (RESET),

     //Must be passed to hps_io module
     .HPS_BUS         (HPS_BUS),

     //Base video clock. Usually equals to CLK_SYS.
     .CLK_VIDEO       (CLK_VIDEO),

     //Multiple resolutions are supported using different CE_PIXEL rates.
     //Must be based on CLK_VIDEO
     .CE_PIXEL        (CE_PIXEL),

     //Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
     .VIDEO_ARX       (VIDEO_ARX),
     .VIDEO_ARY       (VIDEO_ARY),

     .VGA_R           (VGA_R),
     .VGA_G           (VGA_G),
     .VGA_B           (VGA_B),
     .VGA_HS          (VGA_HS),
     .VGA_VS          (VGA_VS),
     .VGA_DE          (VGA_DE), // = ~(VBlank | HBlank)
     .VGA_F1          (VGA_F1),
     .VGA_SL          (VGA_SL),

     .LED_USER        (LED_USER), // 1 - ON, 0 - OFF.

     // b[1]: 0 - LED status is system status OR'd with b[0]
     //       1 - LED status is controled solely by b[0]
     // hint: supply 2'b00 to let the system control the LED.
     .LED_POWER       (LED_POWER),
     .LED_DISK        (LED_DISK),

     // I/O board button press simulation (active high)
     // b[1]: user button
     // b[0]: osd button
     .BUTTONS         (BUTTONS),

     .AUDIO_L         (AUDIO_L),
     .AUDIO_R         (AUDIO_R),
     .AUDIO_S         (AUDIO_S), // 1 - signed audio samples, 0 - unsigned
     .AUDIO_MIX       (AUDIO_MIX), // 0 - no mix, 1 - 25%, 2 - 50%, 3 - 100% (mono)

     //ADC
     .ADC_BUS         (ADC_BUS),

     //SD-SPI
     .SD_SCK          (SD_SCK),
     .SD_MOSI         (SD_MOSI),
     .SD_MISO         (SD_MISO),
     .SD_CS           (SD_CS),
     .SD_CD           (SD_CD),

     //High latency DDR3 RAM interface
     //Use for non-critical time purposes
     .DDRAM_CLK       (DDRAM_CLK),
     .DDRAM_BUSY      (DDRAM_BUSY),
     .DDRAM_BURSTCNT  (DDRAM_BURSTCNT),
     .DDRAM_ADDR      (DDRAM_ADDR),
     .DDRAM_DOUT      (DDRAM_DOUT),
     .DDRAM_DOUT_READY(DDRAM_DOUT_READY),
     .DDRAM_RD        (DDRAM_RD),
     .DDRAM_DIN       (DDRAM_DIN),
     .DDRAM_BE        (DDRAM_BE),
     .DDRAM_WE        (DDRAM_WE),

     //SDRAM interface with lower latency
     .sdram_addr      (sdram_addr),
     .sdram_din       (sdram_din),
     .sdram_ds        (sdram_ds),
     .sdram_we        (sdram_we),
     .sdram_oe        (sdram_oe),
     .sdram_out       (sdram_out),
     .cep             (cep),

     .UART_CTS        (UART_CTS),
     .UART_RTS        (UART_RTS),
     .UART_RXD        (UART_RXD),
     .UART_TXD        (UART_TXD),
     .UART_DTR        (UART_DTR),
     .UART_DSR        (UART_DSR),

     // Open-drain User port.
     // 0 - D+/RX
     // 1 - D-/TX
     // 2..6 - USR2..USR6
     // Set USER_OUT to 1 to read from USER_IN.
     .USER_IN         (USER_IN),
     .USER_OUT        (USER_OUT),

     .OSD_STATUS      (OSD_STATUS)
     );

  assign SDRAM_CKE = 1;

  // Need to pull out the ddr IO
  sdram sdram
    (
     // system interface
     .init    ( !pll_locked ),
     .clk     ( clk_sys     ),
     .sync    ( cep         ),

     .sd_clk  ( SDRAM_CLK   ),
     .sd_data ( SDRAM_DQ    ),
     .sd_addr ( SDRAM_A     ),
     .sd_dqm  ( {SDRAM_DQMH, SDRAM_DQML} ),
     .sd_cs   ( SDRAM_nCS   ),
     .sd_ba   ( SDRAM_BA    ),
     .sd_we   ( SDRAM_nWE   ),
     .sd_ras  ( SDRAM_nRAS  ),
     .sd_cas  ( SDRAM_nCAS  ),

        // cpu/chipset interface
     // map rom to sdram word address $200000 - $20ffff
     .din     ( sdram_din   ),
     .addr    ( sdram_addr  ),
     .ds      ( sdram_ds    ),
     .we      ( sdram_we    ),
     .oe      ( sdram_oe    ),
     .dout    ( sdram_out   )
     );

endmodule // emu
