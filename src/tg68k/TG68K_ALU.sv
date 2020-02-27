//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
// Copyright (c) 2009-2011 Tobias Gubener                                   //
// SystemVerilog Conversion Copyright (c) 2020 Francis Bruno                //
// Subdesign fAMpIGA by TobiFlex                                            //
//                                                                          //
// This source file is free software: you can redistribute it and/or modify //
// it under the terms of the GNU General Public License as published        //
// by the Free Software Foundation, either version 3 of the License, or     //
// (at your option) any later version.                                      //
//                                                                          //
// This source file is distributed in the hope that it will be useful,      //
// but WITHOUT ANY WARRANTY; without even the implied warranty of           //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            //
// GNU General Public License for more details.                             //
//                                                                          //
// You should have received a copy of the GNU General Public License        //
// along with this program.  If not, see <http://www.gnu.org/licenses/>.    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

import TG68K_Pack::*;

module TG68K_ALU
  #
  (
   parameter MUL_Mode = 0,	// 0=>16Bit,  1=>32Bit,  2=>switchable with CPU(1),  3=>no MUL,
   parameter DIV_Mode = 0	// 0=>16Bit,  1=>32Bit,  2=>switchable with CPU(1),  3=>no DIV,
   )
  (
   input  wire                     clk,
   input  wire                     Reset,
   input  wire                     clkena_lw, // = '1,
   input  wire                     execOPC,
   input  wire                     exe_condition,
   input  wire                     exec_tas,
   input  wire                     long_start,
   input  wire                     movem_presub,
   input  wire                     set_stop,
   input  wire                     Z_error,
   input  wire [1:0]               rot_bits,
   input  wire [lastOpcBit:0]      exec,
   input  wire [31:0]              OP1out,
   input  wire [31:0]              OP2out,
   input  wire [31:0]              reg_QA,
   input  wire [31:0]              reg_QB,
   input  wire [15:0]              opcode,
   input  wire [1:0]               datatype,
   input  wire [15:0]              exe_opcode,
   input  wire [1:0]               exe_datatype,
   input  wire [15:0]              sndOPC,
   input  wire [15:0]              last_data_read,
   input  wire [15:0]              data_read,
   input  wire [7:0]               FlagsSR,
   input  micro_states             micro_state,
   input  wire [7:0]               bf_ext_in,
   output logic [7:0]              bf_ext_out,
   input  wire [5:0]               bf_shift,
   input  wire [5:0]               bf_width,
   input  wire [4:0]               bf_loffset,

   output logic                    set_V_Flag,
   output logic [7:0]              Flags,
   output logic [2:0]              c_out,
   output logic [31:0]             addsub_q,
   output logic [31:0]             ALUout
   );

  //////////////////////////////////////////////////////////////////////////
  // ALU and more
  //////////////////////////////////////////////////////////////////////////
  logic [31:0]                     OP1in;
  logic [31:0]                     addsub_a;
  logic [31:0]                     addsub_b;
  logic [33:0]                     notaddsub_b;
  logic [33:0]                     add_result;
  logic [2:0]                      addsub_ofl;
  logic                            opaddsub;
  logic [3:0]                      c_in;
  logic [2:0]                      flag_z;
  logic [3:0]                      set_Flags;	//NZVC
  logic [7:0]                      CCRin;

  logic [5:0]                      niba_l;
  logic [5:0]                      niba_h;
  logic                            niba_lc;
  logic                            niba_hc;
  logic                            bcda_lc;
  logic                            bcda_hc;
  logic [5:0]                      nibs_l;
  logic [5:0]                      nibs_h;
  logic                            nibs_lc;
  logic                            nibs_hc;

  logic [8:0]                      bcd_a;
  logic [8:0]                      bcd_s;
  logic [15:0]                     pack_out;
  logic [15:0]                     pack_a;
  logic [63:0]                     result_mulu;
  logic [63:0]                     result_div;
  logic                            set_mV_Flag;
  logic                            V_Flag;

  logic                            rot_rot;
  logic                            rot_lsb;
  logic                            rot_msb;
  logic                            rot_X;
  logic                            rot_C;
  logic [31:0]                     rot_out;
  logic                            asl_VFlag;
  logic [1:0]                      bit_bits;
  logic [4:0]                      bit_number;
  logic [31:0]                     bits_out;
  logic                            one_bit_in;
  logic                            bchg;
  logic                            bset;

  logic                            mulu_sign;
  logic [16:0]                     mulu_signext;
  logic                            muls_msb;
  logic [63:0]                     mulu_reg;
  logic                            FAsign;
  logic [31:0]                     faktorA;
  logic [31:0]                     faktorB;

  logic [63:0]                     div_reg;
  logic [63:0]                     div_quot;
  logic                            div_ovl;
  logic                            div_neg;
  logic                            div_bit;
  logic [32:0]                     div_sub;
  logic [32:0]                     div_over;
  logic                            nozero;
  logic                            div_qsign;
  logic [63:0]                     divisor;
  logic                            divs;
  logic                            signedOP;
  logic                            OP1_sign;
  logic                            OP2_sign;
  logic [15:0]                     OP2outext;

  logic [5:0]                      in_offset;
  //    logic [5:0]                in_width;
  logic [31:0]                     datareg;
  logic [31:0]                     insert;
  //    logic [31:0]               bf_result;
  //    logic [5:0]                bf_offset;
  //    logic [5:0]                bf_width;
  //    logic [5:0]                bf_firstbit;
  logic [31:0]                     bf_datareg;
  //    logic [31:0]               bf_out;
  logic [39:0]                     result;
  logic [39:0]                     result_tmp;
  logic [31:0]                     sign;
  logic [39:0]                     bf_set1;
  logic [39:0]                     inmux0;
  logic [39:0]                     inmux1;
  logic [39:0]                     inmux2;
  logic [31:0]                     inmux3;
  logic [39:0]                     copymux0;
  logic [39:0]                     copymux1;
  logic [39:0]                     copymux2;
  logic [31:0]                     copymux3;
  logic [31:0]                     bf_set2;
  //    logic [31:0]               bf_set3;
  logic [39:0]                     shift;
  logic [39:0]                     copy;
  //    logic [5:0]                offset;
  //    logic [5:0]                width;
  logic [5:0]                      bf_firstbit;
  logic [3:0]                      mux;
  logic [4:0]                      bitnr;
  logic [31:0]                     mask;
  logic                            bf_bset;
  logic                            bf_NFlag;
  logic                            bf_bchg;
  logic                            bf_ins;
  logic                            bf_exts;
  logic                            bf_fffo;
  logic                            bf_d32;
  logic                            bf_s32;
  logic [4:0]                      index;
  //    logic [4:0]                i;
  //    logic [5:0]                i;

  ////////////////////////////////////////////////////////////////////////////-
  // set OP1in
  ////////////////////////////////////////////////////////////////////////////-
  always_comb begin
    ALUout    = OP1in;
    ALUout[7] = OP1in[7] | exec_tas;

    if (exec[opcBFwb]) begin
      ALUout = result[31:0];
      if (bf_fffo) begin
        ALUout      = '0;
        ALUout[5:0] = bf_firstbit + bf_shift;
      end
    end

    OP1in = addsub_q;

    if (exec[opcABCD]) begin
      OP1in[7:0] = bcd_a[7:0];
    end else if (exec[opcSBCD]) begin
      OP1in[7:0] = bcd_s[7:0];
    end else if (exec[opcMULU] && (MUL_Mode != 3)) begin
      if (exec[write_lowlong] && ((MUL_Mode == 1) || (MUL_Mode == 2))) begin
        OP1in = result_mulu[31:0];
      end else begin
        OP1in = result_mulu[63:32];
      end
    end else if (exec[opcDIVU] && (DIV_Mode != 3)) begin
      if (exe_opcode[15] || (DIV_Mode == 0)) begin
        OP1in = {result_div[47:32], result_div[15:0]};
      end else begin	//64bit
        if (exec[write_reminder]) begin
          OP1in = result_div[63:32];
        end else begin
          OP1in  = result_div[31:0];
        end
      end
    end else if (exec[opcOR]) begin
      OP1in = OP2out | OP1out;
    end else if (exec[opcAND]) begin
      OP1in = OP2out & OP1out;
    end else if (exec[opcScc]) begin
      OP1in[7:0] = {8{exe_condition}};
    end else if (exec[opcEOR]) begin
      OP1in = OP2out ^ OP1out;
    end else if (exec[opcMOVE] || exec[exg]) begin
      // OP1in = {OP2out[31:8],(OP2out[7] | exec_tas), OP2out[6:0]};
      OP1in = OP2out;
    end else if (exec[opcROT]) begin
      OP1in = rot_out;
    end else if (exec[opcSWAP]) begin
      OP1in = {OP1out[15:0], OP1out[31:16]};
    end else if (exec[opcBITS]) begin
      OP1in = bits_out;
    end else if (exec[opcBF]) begin
      OP1in = bf_datareg;
    end else if (exec[opcMOVESR]) begin
      OP1in[7:0] = Flags;
      if (~|exe_datatype) begin
        OP1in[15:8] = '0;
      end else begin
        OP1in[15:8] = FlagsSR;
      end
    end else if (exec[opcPACK]) begin
      OP1in[15:0] = pack_out;
    end
  end // always_comb

  ////////////////////////////////////////////////////////////////////////////-
  // addsub
  ////////////////////////////////////////////////////////////////////////////-
  always_comb begin
    addsub_a = OP1out;
    if (exec[get_bfoffset]) begin
      if (sndOPC[11]) begin
        addsub_a = {{3{OP1out[31]}}, OP1out[31:3]};;
      end else begin
        addsub_a = {30'b0, sndOPC[10:9]};
      end
    end

    opaddsub = exec[subidx];

    c_in[0]  = '0;
    addsub_b = OP2out;
    if ((execOPC == 0) && ~exec[OP2out_one] && ~exec[get_bfoffset]) begin
      if ((long_start == 0) && ~|datatype && ~exec[use_SP]) begin
        addsub_b = 32'b1;
      end else if ((long_start == 0) && (exe_datatype == 2'b10) && (exec[presub] || exec[postadd] || movem_presub)) begin
        if (exec[movem_action]) begin
          addsub_b = 32'b00000000000000000000000000000110;
        end else begin
          addsub_b = 32'b00000000000000000000000000000100;
        end
      end else begin
        addsub_b = 32'b00000000000000000000000000000010;
      end
    end else begin
      if ((exec[use_XZFlag] && Flags[4]) || exec[opcCHK]) begin
        c_in[0] = '1;
      end
      opaddsub = exec[addsub];
    end

    if (~opaddsub || long_start) begin		//ADD
      notaddsub_b = {1'b0, addsub_b, c_in[0]};
    end else begin
      notaddsub_b = ~{1'b0, addsub_b, c_in[0]};
    end

    add_result     = ({1'b0, addsub_a, notaddsub_b[0]} + notaddsub_b);
    c_in[1]        = add_result[9]  ^ addsub_a[8]  ^ addsub_b[8];
    c_in[2]        = add_result[17] ^ addsub_a[16] ^ addsub_b[16];
    c_in[3]        = add_result[33];
    addsub_q       = add_result[32:1];
    addsub_ofl[0]  = (c_in[1] ^ add_result[8]  ^ addsub_a[7]  ^ addsub_b[7]);	// V Byte
    addsub_ofl[1]  = (c_in[2] ^ add_result[16] ^ addsub_a[15] ^ addsub_b[15]);	// V Word
    addsub_ofl[2]  = (c_in[3] ^ add_result[32] ^ addsub_a[31] ^ addsub_b[31]);	// V Long
    c_out          = c_in[3:1];
  end // always_comb

  //////////////////////////////////////////////////////////////////////////////
  //ALU
  //////////////////////////////////////////////////////////////////////////////
  always_comb begin
    if (exe_opcode[7:6] == 2'b01) begin
      // PACK
      pack_a   = $unsigned(OP1out[15:0]) + $unsigned(OP2out[15:0]);
      pack_out = {8'b0, pack_a[11:8], pack_a[3:0]};
    end else begin
      // UNPK
      pack_a   = {4'b0, OP2out[7:4], 4'b0, OP2out[3:0]};
      pack_out = $unsigned(OP1out[15:0]) + $unsigned(pack_a);
    end

    //BCD_ARITH//////////////////////////////////////////////////////////////////-
    //ADC
    bcd_a   = {niba_hc, (niba_h[4:1] + {1'b0, niba_hc, niba_hc, 1'b0}), (niba_l[4:1] + {1'b0, niba_lc, niba_lc, 1'b0})};
    niba_l  = {1'b0, OP1out[3:0], 1'b1} + {1'b0, OP2out[3:0], Flags[4]};
    niba_lc = niba_l[5] | (niba_l[4] & niba_l[3]) | (niba_l[4] & niba_l[2]);

    niba_h  = {1'b0, OP1out[7:4], 1'b1} + {1'b0, OP2out[7:4], niba_lc};
    niba_hc = niba_h[5] | (niba_h[4] & niba_h[3]) | (niba_h[4] & niba_h[2]);
    //SBC
    bcd_s   = {nibs_hc, (nibs_h[4:1] - {1'b0, nibs_hc, nibs_hc, 1'b0}), (nibs_l[4:1] - {1'b0, nibs_lc, nibs_lc, 1'b0})};
    nibs_l  = {1'b0, OP1out[3:0], 1'b0} - {1'b0, OP2out[3:0], Flags[4]};
    nibs_lc = nibs_l[5];

    nibs_h  = {1'b0, OP1out[7:4], 1'b0} - {1'b0, OP2out[7:4], nibs_lc};
    nibs_hc = nibs_h[5];
  end // always_comb

  ////////////////////////////////////////////////////////////////////////////-
  // Bits
  ////////////////////////////////////////////////////////////////////////////-
  always_ff @(posedge clk) begin
    if (clkena_lw) begin
      bchg <= '0;
      bset <= '0;
      case (opcode[7:6])
        2'b01: begin // bchg
          bchg <= '1;
        end
        2'b11: begin //bset
          bset <= '1;
        end
      endcase // case (opcode[7:6])
    end // if (clkena_lw)
  end // always_ff @ (posedge clk)

  always_comb begin
    if (~exe_opcode[8]) begin
      if (exe_opcode[5:4] == 2'b00) begin
        bit_number = sndOPC[4:0];
      end else begin
        bit_number = {2'b0, sndOPC[2:0]};
      end
    end else begin
      if (exe_opcode[5:4] == 2'b00) begin
        bit_number  = reg_QB[4:0];
      end else begin
        bit_number  = {2'b0, reg_QB[2:0]};
      end
    end // else: !if(~exe_opcode[8])

    one_bit_in = OP1out[bit_number];
    bits_out   = OP1out;
    bits_out[bit_number] = (bchg & ~one_bit_in) | bset;
  end // always_comb

  ////////////////////////////////////////////////////////////////////////////-
  // Bit Field
  ////////////////////////////////////////////////////////////////////////////-
  always_ff @(posedge clk) begin
    if (clkena_lw) begin
      bf_bset <= '0;
      bf_bchg <= '0;
      bf_ins  <= '0;
      bf_exts <= '0;
      bf_fffo <= '0;
      bf_d32  <= '0;
      bf_s32  <= '0;
      case (opcode[10:8])
        3'b010: bf_bchg <= '1;				//BFCHG
        3'b011: bf_exts <= '1;				//BFEXTS
        //3'b100: insert <= (OTHERS =>'0');		//BFCLR
        3'b101: bf_fffo <= '1;				//BFFFO
        3'b110: bf_bset <= '1;				//BFSET
        3'b111: begin
          bf_ins <= '1;					//BFINS
          bf_s32 <= '1;
        end
      endcase
      if (opcode[4:3] == 2'b00) begin
        bf_d32 <= '1;
      end
      bf_ext_out <= result[39:32];
    end // if (clkena_lw)
  end

  always_comb begin
    shift = {bf_ext_in, OP2out};
    if (bf_s32) shift[39:32] = OP2out[7:0];
    inmux0          = bf_shift[0]   ? {shift[0],       shift[39:1]}     : shift;
    inmux1          = bf_shift[1]   ? {inmux0[1:0],    inmux0[39:2]}    : inmux0;
    inmux2          = bf_shift[2]   ? {inmux1[3:0],    inmux1[39:4]}    : inmux1;
    inmux3          = bf_shift[3]   ? {inmux2[7:0],    inmux2[31:8]}    : inmux2[31:0];
    bf_set2         = bf_shift[4]   ? {inmux3[15:0],   inmux3[31:16]}   : inmux3;
    copymux3        = bf_loffset[4] ? {sign[15:0],     sign[31:16]}     : sign;
    copymux2[31:0]  = bf_loffset[3] ? {copymux3[23:0], copymux3[31:24]} : copymux3;
    copymux2[39:32] = bf_d32        ? copymux3[7:0]                     : '1;
    copymux1        = bf_loffset[2] ? {copymux2[35:0], copymux2[39:36]} : copymux2;
    copymux0        = bf_loffset[1] ? {copymux1[37:0], copymux1[39:38]} : copymux1;
    copy            = bf_loffset[0] ? {copymux0[38:0], copymux0[39]}    : copymux0;

    result_tmp      = {bf_ext_in, OP1out};
    datareg         = bf_ins        ? reg_QB                            : bf_set2;

    if (bf_ins) begin
      result[31:0]  = bf_set2;
      result[39:32] = bf_set2[7:0];
    end else if (bf_bchg) begin
      result[31:0]  = ~OP1out;
      result[39:32] = ~bf_ext_in;
    end else begin
      result        = '0;
    end
    if (bf_bset) result = '1;

    sign = '0;
    bf_NFlag = datareg[bf_width[4:0]]; // FB, check me
    for (int i = 0; i < 32; i++) begin
      if (i > bf_width[4:0]) begin
        datareg[i] = '0;
        sign[i]    = '1;
      end
    end
    for (int i = 0; i < 40; i++) begin
      if (copy[i]) result[i] = result_tmp[i];
    end
    if (bf_exts && bf_NFlag) begin
      bf_datareg = datareg | sign;
    end else begin
      bf_datareg = datareg;
    end

    //	bf_datareg <= copy(31:0);
    //	result(31:0)<=datareg;
    //BFFFO
    mask        = datareg;
    bf_firstbit = {1'b0, bitnr};
    bitnr       = '1;
    if (mask[31:28] == 4'b0000) begin
      if (mask[27:24] == 4'b0000) begin
        if (mask[23:20] == 4'b0000) begin
          if (mask[19:16] == 4'b0000) begin
            bitnr[4] = '0;
            if (mask[15:12] == 4'b0000) begin
              if (mask[11:8] == 4'b0000) begin
                bitnr[3] = '0;
                if (mask[7:4] == 4'b0000) begin
                  bitnr[2] = '0;
                  mux      = mask[3:0];
                end else begin
                  mux      = mask[7:4];
                end
              end else begin
                mux = mask[11:8];
                bitnr[2] = '0;
              end // else: !if(mask[11:8] == 4'b0000)
            end else begin
              mux = mask[15:12];
            end // else: !if(mask[15:12] == 4'b0000)
          end else begin
            mux      = mask[19:16];
            bitnr[3] = '0;
            bitnr[2] = '0;
          end // else: !if(mask[19:16] == 4'b0000)
        end else begin
          mux      = mask[23:20];
          bitnr[3] = '0;
        end // else: !if(mask[23:20] == 4'b0000)
      end else begin
        mux      = mask[27:24];
        bitnr[2] = '0;
      end // else: !if(mask[27:24] == 4'b0000)
    end else begin // if (mask[31:28] == 4'b0000)
      mux = mask[31:28];
    end // else: !if(mask[31:28] == 4'b0000)

    if (mux[3:2] == 2'b00) begin
      bitnr[1] = '0;
      if (~mux[1]) begin
        bitnr[0] = '0;
      end
    end else begin
      if (~mux[3]) begin
        bitnr[0]  = '0;
      end
    end
  end // always_comb

  ////////////////////////////////////////////////////////////////////////////-
  // Rotation
  ////////////////////////////////////////////////////////////////////////////-
  always_comb begin
    casez (exe_opcode[7:6])
      2'b00: rot_rot = OP1out[7];  // Byte
      2'bz1: rot_rot = OP1out[15]; // Word
      2'b10: rot_rot = OP1out[31]; // Long
    endcase
    case (rot_bits)
      2'b00: begin // ASL, ASR
        rot_lsb = '0;
        rot_msb = rot_rot;
      end
      2'b01: begin // LSL, LSR
        rot_lsb = '0;
        rot_msb = '0;
      end
      2'b10: begin // ROXL, ROXR
        rot_lsb = Flags[4];
        rot_msb = Flags[4];
      end
      2'b11: begin // ROL, ROR
        rot_lsb = rot_rot;
        rot_msb = OP1out[0];
      end
    endcase

    if (exec[rot_nop]) begin
      rot_out = OP1out;
      rot_X   = Flags[4];
      if (rot_bits == 2'b10) // ROXL, ROXR
        rot_C = Flags[4];
      else
        rot_C = '0;
    end else begin
      if (exe_opcode[8]) begin // left
        rot_out = {OP1out[30:0], rot_lsb};
        rot_X   = rot_rot;
        rot_C   = rot_rot;
      end else begin
        rot_out = {rot_msb, OP1out[31:1]};
        rot_X   = OP1out[0];
        rot_C   = OP1out[0];
        casez (exe_opcode[7:6])
          2'b00: rot_out[7]  = rot_msb; // Byte
          2'bz1: rot_out[15] = rot_msb; // Word
        endcase // casez (exe_opcode[7:6])
      end // else: !if(exe_opcode[8])
    end // else: !if(exec[rot_nop])
  end // always_comb

  always_comb begin
    //////////////////////////////////////////////////////////////////////////////
    //CCR op
    //////////////////////////////////////////////////////////////////////////////
    if      (exec[andiSR]) CCRin = Flags & last_data_read[7:0];
    else if (exec[eoriSR]) CCRin = Flags ^ last_data_read[7:0];
    else if (exec[oriSR])  CCRin = Flags | last_data_read[7:0];
    else                   CCRin = OP2out[7:0];

    //////////////////////////////////////////////////////////////////////////////
    //Flags
    //////////////////////////////////////////////////////////////////////////////
    flag_z = '0;
    if (exec[use_XZFlag] && ~Flags[2]) begin
      flag_z = '0;
    end else begin
      flag_z[0]  = ~|OP1in[7:0];
      flag_z[1]  = ~|OP1in[15:0];
      flag_z[2]  = ~|OP1in[31:0];
    end

    // Flags NZVC
    if (exe_datatype == 2'b00) begin // Byte
      set_Flags = {OP1in[7], flag_z[0], addsub_ofl[0], c_out[0]};
      if (exec[opcABCD]) begin
        set_Flags[0] = bcd_a[8];
      end else if (exec[opcSBCD]) begin
        set_Flags[0] = bcd_s[8];
      end
    end else if ((exe_datatype == 2'b10) || exec[opcCPMAW]) begin // Long
      set_Flags = {OP1in[31], flag_z[2], addsub_ofl[2], c_out[2]};
    end else begin // Word
      set_Flags = {OP1in[15], flag_z[1], addsub_ofl[1], c_out[1]};
    end
  end // always_comb

  always_ff @(posedge clk) begin
    if (clkena_lw) begin
      if (exec[directSR] || set_stop) begin
        Flags[7:0] <= data_read[7:0];
      end
      if (exec[directCCR]) begin
        Flags[7:0] <= data_read[7:0];
      end

      if (exec[opcROT]) begin
        asl_VFlag <= ((set_Flags[3] ^ rot_rot) | asl_VFlag);
      end else begin
        asl_VFlag <= '0;
      end
      if (exec[to_CCR]) begin
        Flags[7:0] <= CCRin[7:0];			//CCR
      end else if (Z_error) begin
        Flags[3:0] <= ~exe_opcode[8] ? {reg_QA[31], 3'b000} : 4'b0100;
      end else if (~exec[no_Flags]) begin
        if (exec[opcADD]) begin
          Flags[4] <= set_Flags[0];
        end else if (exec[opcROT] && (rot_bits != 2'b11) && ~exec[rot_nop]) begin
          Flags[4] <= rot_X;
        end

        if (exec[opcADD] || exec[opcCMP]) begin
          Flags[3:0] <= set_Flags;
        end else if (exec[opcDIVU] && (DIV_Mode != 3)) begin
          if (V_Flag) begin
            Flags[3:0] <= 4'b1010;
          end else begin
            Flags[3:0] <= {OP1in[15], flag_z[1], 2'b00};
          end
        end else if (exec[write_reminder] && (MUL_Mode != 3)) begin // z-flag MULU.l
          Flags[3] <= set_Flags[3];
          Flags[2] <= set_Flags[2] & Flags[2];
          Flags[1] <= '0;
          Flags[0] <= '0;
        end else if (exec[write_lowlong] && ((MUL_Mode == 1) || (MUL_Mode == 2))) begin  // flag MULU.l
          Flags[3] <= set_Flags[3];
          Flags[2] <= set_Flags[2];
          Flags[1] <= set_mV_Flag;	//V
          Flags[0] <= '0;
        end else if (exec[opcOR]    || exec[opcAND]  || exec[opcEOR] || exec[opcMOVE] ||
                     exec[opcMOVEQ] || exec[opcSWAP] || exec[opcBF]  ||
                     (exec[opcMULU] && (MUL_Mode != 3))) begin
          Flags[1:0] <= '0;
          Flags[3:2] <= set_Flags[3:2];
          if (exec[opcBF]) Flags[3] <= bf_NFlag;
        end else if (exec[opcROT]) begin
          Flags[0]   <= rot_C;
          Flags[1]   <= ((rot_bits == 2'b00) && ((set_Flags[3] ^ rot_rot) || asl_VFlag)); // ASL/ASR
          Flags[3:2] <= set_Flags[3:2];
        end else if (exec[opcBITS]) begin
          Flags[2]   <= ~one_bit_in;
        end else if (exec[opcCHK]) begin
          if (exe_datatype == 2'b01) begin // Word
            Flags[3] <= OP1out[15];
          end else begin
            Flags[3] <= OP1out[31];
          end
          if ((OP1out[15:0] == 16'b0) && ((exe_datatype == 2'b01) || (OP1out[31:16] == 16'b0))) begin
            Flags[2] <= '1;
          end else begin
            Flags[2] <= '0;
          end
          Flags[1:0] <= '0;
        end // if (exec[opcCHK])
      end // if (~exec[no_Flags])
    end // if (clkena_lw)
    Flags[7:5] <= '0;
  end // always_ff @ (posedge clk)

  //////////////////////////////////////////////////////////////////////////////-
  //// MULU/MULS
  //////////////////////////////////////////////////////////////////////////////-
  always_comb begin
    if ((signedOP && faktorB[31]) || FAsign) begin
      muls_msb = mulu_reg[63];
    end else begin
      muls_msb = '0;
    end

    mulu_sign <= signedOP && faktorB[31];

    if (MUL_Mode == 0) begin // 16 Bit
      result_mulu[63:32] = {muls_msb, mulu_reg[63:33]};
      //result_mulu[15:0]  = {mulu_reg[15], mulu_reg[15:1]};
      result_mulu[15:0]  = {1'bx, mulu_reg[15:1]};
      if (mulu_reg[0]) begin
        if (FAsign) begin
          //result_mulu[63:47] = {muls_msb, (mulu_reg[63:48] - {mulu_sign, faktorB[31:16]})};
          result_mulu[63:47] = {muls_msb, mulu_reg[63:48]} - {mulu_sign, faktorB[31:16]};
        end else begin
          //result_mulu[63:47] = {muls_msb, (mulu_reg[63:48] + {mulu_sign, faktorB[31:16]})};
          result_mulu[63:47] = {muls_msb, mulu_reg[63:48]} + {mulu_sign, faktorB[31:16]};
        end
      end
    end else begin // 32 Bit
      result_mulu = {muls_msb, mulu_reg[63:1]};
      if (mulu_reg[0]) begin
        if (FAsign) begin
          result_mulu[63:31] = {muls_msb, (mulu_reg[63:32] - {mulu_sign, faktorB})};
        end else begin
          result_mulu[63:31] = {muls_msb, (mulu_reg[63:32] + {mulu_sign, faktorB})};
        end
      end
    end // else: !if(MUL_Mode == 0)

    if (exe_opcode[15] || (MUL_Mode == 0)) begin
      faktorB[31:16] = OP2out[15:0];
      faktorB[15:0]  = '0;
    end else begin
      faktorB = OP2out;
    end
    if ((~|result_mulu[63:32] && (~signedOP || ~result_mulu[31])) ||
      (&result_mulu[63:32] && signedOP && result_mulu[31])) begin
      set_mV_Flag = '0;
    end else begin
      set_mV_Flag = '1;
    end
  end // always_comb

  always_ff @(posedge clk) begin
    if (clkena_lw) begin
      if (micro_state == mul1) begin
        mulu_reg[63:32] <= '0;
        if (divs && ((exe_opcode[15] && reg_QA[15]) || (~exe_opcode[15] && reg_QA[31]))) begin
          FAsign         <= '1;
          mulu_reg[31:0] <= ~reg_QA + 1;
        end else begin
          FAsign         <= '0;
          mulu_reg[31:0] <= reg_QA;
        end
      end else if (~exec[opcMULU]) begin
        mulu_reg <= result_mulu;
      end
    end // if (clkena_lw)
  end // always_ff @ (posedge clk)

  //////////////////////////////////////////////////////////////////////////////-
  //// DIVU/DIVS
  //////////////////////////////////////////////////////////////////////////////-
  always_comb begin
    divs           = (opcode[15] && opcode[8]) || (~opcode[15] && sndOPC[11]);
    divisor[15:0]  = '0;
    divisor[63:32] = {32{divs & reg_QA[31]}};
    if (exe_opcode[15] || (DIV_Mode == 0)) begin
      divisor[47:16] = reg_QA;
    end else begin
      divisor[31:0]  = reg_QA;
      if (exe_opcode[14] && sndOPC[10]) begin
        divisor[63:32] = reg_QB;
      end
    end
    if (signedOP || ~opcode[15]) begin
      OP2outext = OP2out[31:16];
    end else begin
      OP2outext = '0;
    end
    if (signedOP && OP2out[31]) begin
      div_sub = div_reg[63:31] + {1'b1, OP2out[31:0]};
    end else begin
      div_sub = div_reg[63:31] - {1'b0, OP2outext[15:0], OP2out[15:0]};
    end
    if (DIV_Mode == 0) begin
      div_bit = div_sub[16];
    end else begin
      div_bit = div_sub[32];
    end
    if (div_bit) begin
      div_quot[63:32] = div_reg[62:31];
    end else begin
      div_quot[63:32] = div_sub[31:0];
    end
    div_quot[31:0] = {div_reg[30:0], ~div_bit};

    if (((nozero && signedOP && (OP2out[31] ^ OP1_sign ^ div_neg ^ div_qsign))	//Overflow DIVS
         || (~signedOP && ~div_over[32])) && (DIV_Mode != 3)) begin	//Overflow DIVU
      set_V_Flag = '1;
    end else begin
      set_V_Flag = '0;
    end
  end // always_comb

  always_ff @(posedge clk) begin
    if (clkena_lw) begin
      V_Flag   <= set_V_Flag;
      signedOP <= divs;
      if (micro_state == div1) begin
        nozero <= '0;
        if (divs && divisor[63]) begin // Neg divisor
          OP1_sign <= '1;
          div_reg  <= 0 - divisor;
        end else begin
          OP1_sign <= '0;
          div_reg  <= divisor;
        end
      end else begin
        div_reg <= div_quot;
        nozero  <= ~div_bit | nozero;
      end
      if (micro_state == div2) begin
        div_qsign <= ~div_bit;
        div_neg   <= signedOP & (OP2out[31] ^ OP1_sign);
        if (DIV_Mode == 0) begin
          div_over[32:16] <= {1'b0, div_reg[47:32]} - {1'b0, OP2out[15:0]};
        end else begin
          div_over        <= {1'b0, div_reg[63:32]} - {1'b0, OP2out};
        end
      end
      if (~exec[write_reminder]) begin
        //				IF exec_DIVU='0' THEN
        if (div_neg) begin
          result_div[31:0] <= 0-div_quot[31:0];
        end else begin
          result_div[31:0] <= div_quot[31:0];
        end

        if (OP1_sign) begin
          result_div[63:32] <= 0 - div_quot[63:32];
        end else begin
          result_div[63:32] <= div_quot[63:32];
        end
      end // if (~exec[write_reminder])
    end // if (clkena_lw)
  end // always_ff @ (posedge clk)
endmodule // TG68K_ALU
