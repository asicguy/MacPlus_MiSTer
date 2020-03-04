module video
  (
   input        clk,
   input        ce,

   input [14:0] addr,
   input [15:0] dataIn,
   input [1:0]  wr,

   output reg   hsync,
   output reg   vsync,
   output       _hblank,
   output       _vblank,

   output       video_en,
   output       pixelOut
   );

  localparam kVisibleWidth       = 128;
  localparam kTotalWidth         = 176;
  localparam kVisibleHeightStart = 21;
  localparam kVisibleHeightEnd   = 362;
  localparam kTotalHeight        = 370;
  localparam kHsyncStart         = 135;
  localparam kHsyncEnd           = 152;
  localparam kVsyncStart         = 365;
  localparam kVsyncEnd           = 369;
  localparam kPixelLatency       = 1; // number of clk8 cycles from xpos==0 to when pixel data actually exits the video shift register

  localparam videoStart          = 'hA700 - (kVisibleHeightStart * kVisibleWidth/2);

  logic [7:0]   xpos;
  logic [9:0]   ypos;

  assign _hblank = ~(xpos >= kVisibleWidth);
  assign _vblank = ~(ypos < kVisibleHeightStart || ypos > kVisibleHeightEnd);

  logic [15:0]  scrData;
  logic         ram_wr;
  logic         old_wr;


  vram buff
    (
     .clock        (clk),

     .data         (dataIn),
     .wraddress    (addr[13:0]),
     .byteena_a    (wr),
     .wren         (ram_wr),

     .rdaddress    (videoStart[14:1] + {ypos[8:0], xpos[6:2]}),
     .q            (scrData)
     );

  always @(posedge clk) begin
    old_wr <= |wr;
    ram_wr <= (~old_wr & |wr);
  end

  logic [3:0] cycle;
  initial begin
    cycle  = '0;
    xpos   = '0;
    ypos   = '0;
  end

  always @(posedge clk) if(ce) cycle <= cycle + 1'd1;

  always @(posedge clk) begin
    if(ce && !cycle[1:0]) begin
      if (xpos == kTotalWidth-1) begin
        xpos <= 0;
        if (ypos == kTotalHeight-1) ypos <= 0;
        else ypos <= ypos + 1'b1;
      end else if (!xpos && cycle) begin
        xpos <= 0;
      end else begin
        xpos <= xpos + 1'b1;
      end

      if(xpos == kHsyncStart+kPixelLatency) begin
        hsync <= 1;
        vsync <= (ypos >= kVsyncStart && ypos <= kVsyncEnd);
      end
      if(xpos == kHsyncEnd+kPixelLatency) hsync <= 0;
    end
  end

  // a 0 bit is white, and a 1 bit is black
  // data is shifted out MSB first
  logic [15:0] shiftRegister;
  logic [15:0] paper;
  assign pixelOut = ~shiftRegister[15];
  assign video_en = paper[15];

  always @(posedge clk) begin
    if(ce) begin
      if(_vblank && _hblank && cycle == 2) begin
        shiftRegister <= scrData;
        paper <= 16'hFFFF;
      end
      else begin
        shiftRegister <= { shiftRegister[14:0], 1'b1 };
        paper <= { paper[14:0], 1'b0 };
      end
    end
  end

endmodule

module vram
  #
  (
   parameter INFER_RAM = 1
   )
  (
   input         clock,

   input [15:0]  data,
   input [13:0]  wraddress,
   input [1:0]   byteena_a,
   input         wren,

   input [13:0]  rdaddress,
   output logic [15:0] q
   );

  generate
    if (INFER_RAM == 1) begin : g_INFER_RAM
      // use a multi-dimensional packed array
      //to model individual bytes within the word
      logic [1:0][7:0]    ram[0:16384];// # words = 1 << address width

      // Infer RAM from Altera Recommended HDL coding styles
      always_ff @(posedge clock) begin
        if (wren) begin
          if (byteena_a[0]) ram[wraddress][0] <= data[7:0];
          if (byteena_a[1]) ram[wraddress][1] <= data[15:8];
        end
        q <= ram[rdaddress];
      end

    end else begin : g_INSTANTIATE

      altsyncram	altsyncram_component
        (
         .address_a (wraddress),
         .address_b (rdaddress),
         .byteena_a (byteena_a),
         .clock0 (clock),
         .data_a (data),
         .wren_a (wren),
         .q_b (q),
         .aclr0 (1'b0),
         .aclr1 (1'b0),
         .addressstall_a (1'b0),
         .addressstall_b (1'b0),
         .byteena_b (1'b1),
         .clock1 (1'b1),
         .clocken0 (1'b1),
         .clocken1 (1'b1),
         .clocken2 (1'b1),
         .clocken3 (1'b1),
         .data_b ({16{1'b1}}),
         .eccstatus (),
         .q_a (),
         .rden_a (1'b1),
         .rden_b (1'b1),
         .wren_b (1'b0));
      defparam
        altsyncram_component.address_aclr_b = "NONE",
        altsyncram_component.address_reg_b = "CLOCK0",
        altsyncram_component.byte_size = 8,
        altsyncram_component.clock_enable_input_a = "BYPASS",
        altsyncram_component.clock_enable_input_b = "BYPASS",
        altsyncram_component.clock_enable_output_b = "BYPASS",
        altsyncram_component.intended_device_family = "Cyclone V",
        altsyncram_component.lpm_type = "altsyncram",
        altsyncram_component.numwords_a = 16384,
        altsyncram_component.numwords_b = 16384,
        altsyncram_component.operation_mode = "DUAL_PORT",
        altsyncram_component.outdata_aclr_b = "NONE",
        altsyncram_component.outdata_reg_b = "UNREGISTERED",
        altsyncram_component.power_up_uninitialized = "FALSE",
        altsyncram_component.read_during_write_mode_mixed_ports = "DONT_CARE",
        altsyncram_component.widthad_a = 14,
        altsyncram_component.widthad_b = 14,
        altsyncram_component.width_a = 16,
        altsyncram_component.width_b = 16,
        altsyncram_component.width_byteena_a = 2;
    end // block: g_INSTANTIATE
  endgenerate

endmodule
