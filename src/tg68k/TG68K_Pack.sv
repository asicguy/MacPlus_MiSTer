//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//                                                                          --
// Copyright (c) 2009-2013 Tobias Gubener                                   --
// SystemVerilog Conversion Copyright (c) 2020 Francis Bruno                --
// Subdesign fAMpIGA by TobiFlex                                            --
//                                                                          --
// This source file is free software: you can redistribute it and/or modify --
// it under the terms of the GNU General Public License as published        --
// by the Free Software Foundation, either version 3 of the License, or     --
// (at your option) any later version.                                      --
//                                                                          --
// This source file is distributed in the hope that it will be useful,      --
// but WITHOUT ANY WARRANTY; without even the implied warranty of           --
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            --
// GNU General Public License for more details.                             --
//                                                                          --
// You should have received a copy of the GNU General Public License        --
// along with this program.  If not, see <http://www.gnu.org/licenses/>.    --
//                                                                          --
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
`ifndef TG68K_PACK
`define TG68K_PACK

package TG68K_Pack;

  typedef enum logic [6:0]
               {
                idle, nop, ld_nn, st_nn, ld_dAn1, ld_AnXn1, ld_AnXn2, st_dAn1, ld_AnXnbd1, ld_AnXnbd2, ld_AnXnbd3,
                ld_229_1, ld_229_2, ld_229_3, ld_229_4, st_229_1, st_229_2, st_229_3, st_229_4,
                st_AnXn1, st_AnXn2, bra1, bsr1, bsr2, nopnop, dbcc1, movem1, movem2, movem3,
                andi, op_AxAy, cmpm, link1, link2, unlink1, unlink2, int1, int2, int3, int4, rte1, rte2, rte3, trap0, trap1, trap2, trap3,
                trap4, trap5, trap6, movec1, movep1, movep2, movep3, movep4, movep5, rota1, bf1,
                mul1, mul2, mul_end1,  mul_end2, div1, div2, div3, div4, div_end1, div_end2, pack1, pack2, pack3
                } micro_states;

  const logic [6:0] opcMOVE        = 7'd0;
  const logic [6:0] opcMOVEQ       = 7'd1; //
  const logic [6:0] opcMOVESR      = 7'd2; //
  const logic [6:0] opcADD         = 7'd3; //
  const logic [6:0] opcADDQ        = 7'd4; //
  const logic [6:0] opcOR          = 7'd5; //
  const logic [6:0] opcAND         = 7'd6; //
  const logic [6:0] opcEOR         = 7'd7;     //
  const logic [6:0] opcCMP         = 7'd8; //
  const logic [6:0] opcROT         = 7'd9; //
  const logic [6:0] opcCPMAW       = 7'd10;
  const logic [6:0] opcEXT         = 7'd11; //
  const logic [6:0] opcABCD        = 7'd12; //
  const logic [6:0] opcSBCD        = 7'd13; //
  const logic [6:0] opcBITS        = 7'd14; //
  const logic [6:0] opcSWAP        = 7'd15; //
  const logic [6:0] opcScc         = 7'd16; //
  const logic [6:0] andiSR         = 7'd17; //
  const logic [6:0] eoriSR         = 7'd18; //
  const logic [6:0] oriSR          = 7'd19; //
  const logic [6:0] opcMULU        = 7'd20; //
  const logic [6:0] opcDIVU        = 7'd21; //
  const logic [6:0] dispouter      = 7'd22; //
  const logic [6:0] rot_nop        = 7'd23; //
  const logic [6:0] ld_rot_cnt     = 7'd24; //
  const logic [6:0] writePC_add    = 7'd25; //
  const logic [6:0] ea_data_OP1    = 7'd26; //
  const logic [6:0] ea_data_OP2    = 7'd27; //
  const logic [6:0] use_XZFlag     = 7'd28; //
  const logic [6:0] get_bfoffset   = 7'd29; //
  const logic [6:0] save_memaddr   = 7'd30; //
  const logic [6:0] opcCHK         = 7'd31; //
  const logic [6:0] movec_rd       = 7'd32; //
  const logic [6:0] movec_wr       = 7'd33; //
  const logic [6:0] Regwrena       = 7'd34; //
  const logic [6:0] update_FC      = 7'd35; //
  const logic [6:0] linksp         = 7'd36; //
  const logic [6:0] movepl         = 7'd37; //
  const logic [6:0] update_ld      = 7'd38; //
  const logic [6:0] OP1addr        = 7'd39; //
  const logic [6:0] write_reg      = 7'd40; //
  const logic [6:0] changeMode     = 7'd41; //
  const logic [6:0] ea_build       = 7'd42; //
  const logic [6:0] trap_chk       = 7'd43; //
  const logic [6:0] store_ea_data  = 7'd44; //
  const logic [6:0] addrlong       = 7'd45; //
  const logic [6:0] postadd        = 7'd46; //
  const logic [6:0] presub         = 7'd47; //
  const logic [6:0] subidx         = 7'd48; //
  const logic [6:0] no_Flags       = 7'd49; //
  const logic [6:0] use_SP         = 7'd50; //
  const logic [6:0] to_CCR         = 7'd51; //
  const logic [6:0] to_SR          = 7'd52; //
  const logic [6:0] OP2out_one     = 7'd53; //
  const logic [6:0] OP1out_zero    = 7'd54; //
  const logic [6:0] mem_addsub     = 7'd55; //
  const logic [6:0] addsub         = 7'd56; //
  const logic [6:0] directPC       = 7'd57; //
  const logic [6:0] direct_delta   = 7'd58; //
  const logic [6:0] directSR       = 7'd59; //
  const logic [6:0] directCCR      = 7'd60; //
  const logic [6:0] exg            = 7'd61; //
  const logic [6:0] get_ea_now     = 7'd62; //
  const logic [6:0] ea_to_pc       = 7'd63; //
  const logic [6:0] hold_dwr       = 7'd64; //
  const logic [6:0] to_USP         = 7'd65; //
  const logic [6:0] from_USP       = 7'd66; //
  const logic [6:0] write_lowlong  = 7'd67; //
  const logic [6:0] write_reminder = 7'd68; //
  const logic [6:0] movem_action   = 7'd69; //
  const logic [6:0] briefext       = 7'd70; //
  const logic [6:0] get_2ndOPC     = 7'd71; //
  const logic [6:0] mem_byte       = 7'd72; //
  const logic [6:0] longaktion     = 7'd73; //
  const logic [6:0] opcRESET       = 7'd74; //
  const logic [6:0] opcBF          = 7'd75; //
  const logic [6:0] opcBFwb        = 7'd76; //
  const logic [6:0] s2nd_hbits     = 7'd77; //
  const logic [6:0] opcPACK        = 7'd77; //
  //const logic [6:0] s2nd_hbits   = 7'd77; //

  const logic [6:0] lastOpcBit     = 7'd77;

  typedef struct packed
                 {
                   bit opcMOVE;
                   bit opcMOVEQ;
                   bit opcMOVESR;
                   bit opcADD;
                   bit opcADDQ;
                   bit opcOR;
                   bit opcAND;
                   bit opcEOR;
                   bit opcCMP;
                   bit opcROT;
                   bit opcCPMAW;
                   bit opcEXT;
                   bit opcABCD;
                   bit opcSBCD;
                   bit opcBITS;
                   bit opcSWAP;
                   bit opcScc;
                   bit andiSR;
                   bit eoriSR;
                   bit oriSR;
                   bit opcMULU;
                   bit opcDIVU;
                   bit dispouter;
                   bit rot_nop;
                   bit ld_rot_cnt;
                   bit writePC_add;
                   bit ea_data_OP1;
                   bit ea_data_OP2;
                   bit use_XZFlag;
                   bit get_bfoffset;
                   bit save_memaddr;
                   bit opcCHK;
                   bit movec_rd;
                   bit movec_wr;
                   bit Regwrena;
                   bit update_FC;
                   bit linksp;
                   bit movepl;
                   bit update_ld;
                   bit OP1addr;
                   bit write_reg;
                   bit changeMode;
                   bit ea_build;
                   bit trap_chk;
                   bit store_ea_data;
                   bit addrlong;
                   bit postadd;
                   bit presub;
                   bit subidx;
                   bit no_Flags;
                   bit use_SP;
                   bit to_CCR;
                   bit to_SR;
                   bit OP2out_one;
                   bit OP1out_zero;
                   bit mem_addsub;
                   bit addsub;
                   bit directPC;
                   bit direct_delta;
                   bit directSR;
                   bit directCCR;
                   bit exg;
                   bit get_ea_now;
                   bit ea_to_pc;
                   bit hold_dwr;
                   bit to_USP;
                   bit from_USP;
                   bit write_lowlong;
                   bit write_reminder;
                   bit movem_action;
                   bit briefext;
                   bit get_2ndOPC;
                   bit mem_byte;
                   bit longaktion;
                   bit opcRESET;
                   bit opcBF;
                   bit opcBFwb;
                   bit s2nd_hbits;
                 }  rTG68K_opc;

endpackage // TG68K_Pack
`endif
