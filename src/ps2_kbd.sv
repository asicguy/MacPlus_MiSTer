`timescale 1ns / 100ps

/*
 * PS2 Keyboard to Mac interface module
 */
module ps2_kbd
(
        input	clk,
        input	ce,

        input                    reset,

        input [10:0] ps2_key,
        output reg   capslock,

        input  [7:0] data_out,
        input                    strobe_out,

        output [7:0] data_in,
        output           strobe_in
);

reg   [8:0] keymac;
reg			key_pending;
reg  [21:0] pacetimer;
reg			inquiry_active;
reg                     cmd_inquiry;
reg                     cmd_instant;
reg                     cmd_model;
reg                     cmd_test;
  logic                 tick_long;
  logic                 tick_short;
  logic [8:0]           key_code;
  logic [8:0]           key_addr;


/* Latch commands from Mac */
always@(posedge clk or posedge reset) begin
        if (reset) begin
                cmd_inquiry <= 0;
                cmd_instant <= 0;
                cmd_model <= 0;
                cmd_test <= 0;
        end else if(ce) begin
                if (strobe_out) begin
                        cmd_inquiry <= 0;
                        cmd_instant <= 0;
                        cmd_model <= 0;
                        cmd_test <= 0;
                        case(data_out)
                                8'h10: cmd_inquiry <= 1;
                                8'h14: cmd_instant <= 1;
                                8'h16: cmd_model   <= 1;
                                8'h36: cmd_test    <= 1;
                        endcase
                end
        end
end

/* Divide our clock to pace our responses to the Mac. tick_short ticks
 * when we can respond to a command, and tick_long ticks when an inquiry
 * command shall timeout
 */
always@(posedge clk or posedge reset) begin
        if (reset)
                pacetimer <= 0;
        else if(ce) begin
                /* reset counter on command from Mac */
                if (strobe_out)
                        pacetimer <= 0;
                else if (!tick_long)
                        pacetimer <= pacetimer + 1'd1;
        end
end

assign tick_long  = pacetimer == 22'h3fffff;
assign tick_short = pacetimer == 22'h000fff;

/* Delay inquiry responses to after tick_short */
always@(posedge clk or posedge reset) begin
        if (reset)
                inquiry_active <= 0;
        else if(ce) begin
                if (strobe_out | strobe_in)
                        inquiry_active <= 0;
                else if (tick_short)
                        inquiry_active <= cmd_inquiry;
        end
end

wire pop_key = (cmd_instant & tick_short) | (inquiry_active & tick_long) | (inquiry_active & key_pending);

/* Reply to Mac */
assign strobe_in = ((cmd_model | cmd_test) & tick_short) | pop_key;

/* Data to Mac */
assign data_in = cmd_test       ? 8'h7d :
                 cmd_model	? 8'h03 :
                 key_pending ? (keymac[8] ? 8'h79 : keymac[7:0]) :
                 8'h7b;

wire depress  = ~ps2_key[9];
wire extended = ps2_key[8];
wire ignore_capslock = {extended,ps2_key[7:0]} == 9'h058 && capslock;

/* Handle key_pending, and multi-byte keypad responses */
always @(posedge clk) begin
        reg old_stb;

        if (reset) begin
                key_pending <= 0;
                old_stb <= ps2_key[10];
                capslock <= 0;
        end
        else if(ce) begin
                if (cmd_model | cmd_test) key_pending <= 0;
                else if (pop_key) begin
                        if (keymac[8]) keymac[8] <= 0;
                        else key_pending <= 0;
                end else begin

                        old_stb <= ps2_key[10];
                        if(old_stb != ps2_key[10]) begin

                                /* Capslock handling */
                                if(ps2_key[7:0] == 8'h58 && !extended && !depress) capslock <= ~capslock;

                                if(!key_pending && !ignore_capslock) begin
                                        key_pending <= 1;
                                        keymac <= {key_code[8],depress,key_code[6:0]};
                                end
                        end
                end
        end
end


  logic [8:0] code[512];
  initial begin
    code[9'h000] = 9'h07b;
    code[9'h001] = 9'h07b;  // F9
    code[9'h002] = 9'h07b;
    code[9'h003] = 9'h07b;  // F5
    code[9'h004] = 9'h07b;  // F3
    code[9'h005] = 9'h07b;  // F1
    code[9'h006] = 9'h07b;  // F2
    code[9'h007] = 9'h07b;  // F12 <OSD>
    code[9'h008] = 9'h07b;
    code[9'h009] = 9'h07b;  // F10
    code[9'h00A] = 9'h07b;  // F8
    code[9'h00B] = 9'h07b;  // F6
    code[9'h00C] = 9'h07b;  // F4
    code[9'h00D] = 9'h061;  // TAB
    code[9'h00E] = 9'h065;  // ~ (`)
    code[9'h00F] = 9'h07b;

    code[9'h010] = 9'h07b;
    code[9'h011] = 9'h06f;  // LEFT ALT (command)
    code[9'h012] = 9'h071;  // LEFT SHIFT
    code[9'h013] = 9'h07b;
    code[9'h014] = 9'h07b;  // CTRL (not mapped)
    code[9'h015] = 9'h019;  // q
    code[9'h016] = 9'h025;  // 1
    code[9'h017] = 9'h07b;
    code[9'h018] = 9'h019;
    code[9'h019] = 9'h025;
    code[9'h01A] = 9'h00d;  // z
    code[9'h01B] = 9'h003;  // s
    code[9'h01C] = 9'h001;  // a
    code[9'h01D] = 9'h01b;  // w
    code[9'h01E] = 9'h027;  // 2
    code[9'h01F] = 9'h07b;

    code[9'h020] = 9'h07b;
    code[9'h021] = 9'h011;  // c
    code[9'h022] = 9'h00f;  // x
    code[9'h023] = 9'h005;  // d
    code[9'h024] = 9'h01d;  // e
    code[9'h025] = 9'h02b;  // 4
    code[9'h026] = 9'h029;  // 3
    code[9'h027] = 9'h07b;
    code[9'h028] = 9'h07b;
    code[9'h029] = 9'h063;  // SPACE
    code[9'h02A] = 9'h013;  // v
    code[9'h02B] = 9'h007;  // f
    code[9'h02C] = 9'h023;  // t
    code[9'h02D] = 9'h01f;  // r
    code[9'h02E] = 9'h02f;  // 5
    code[9'h02F] = 9'h07b;

    code[9'h030] = 9'h07b;
    code[9'h031] = 9'h05b;  // n
    code[9'h032] = 9'h017;  // b
    code[9'h033] = 9'h009;  // h
    code[9'h034] = 9'h00b;  // g
    code[9'h035] = 9'h021;  // y
    code[9'h036] = 9'h02d;  // 6
    code[9'h037] = 9'h07b;
    code[9'h038] = 9'h07b;
    code[9'h039] = 9'h07b;
    code[9'h03A] = 9'h05d;  // m
    code[9'h03B] = 9'h04d;  // j
    code[9'h03C] = 9'h041;  // u
    code[9'h03D] = 9'h035;  // 7
    code[9'h03E] = 9'h039;  // 8
    code[9'h03F] = 9'h07b;

    code[9'h040] = 9'h07b;
    code[9'h041] = 9'h057;  // <,
    code[9'h042] = 9'h051;  // k
    code[9'h043] = 9'h045;  // i
    code[9'h044] = 9'h03f;  // o
    code[9'h045] = 9'h03b;  // 0
    code[9'h046] = 9'h033;  // 9
    code[9'h047] = 9'h07b;
    code[9'h048] = 9'h07b;
    code[9'h049] = 9'h05f;  // >.
    code[9'h04A] = 9'h059;  // FORWARD SLASH
    code[9'h04B] = 9'h04b;  // l
    code[9'h04C] = 9'h053;  // ;
    code[9'h04D] = 9'h047;  // p
    code[9'h04E] = 9'h037;  // -
    code[9'h04F] = 9'h07b;

    code[9'h050] = 9'h07b;
    code[9'h051] = 9'h07b;
    code[9'h052] = 9'h04f;  // '"
    code[9'h053] = 9'h07b;
    code[9'h054] = 9'h043;  // [
    code[9'h055] = 9'h031;  // =
    code[9'h056] = 9'h07b;
    code[9'h057] = 9'h07b;
    code[9'h058] = 9'h073;  // CAPSLOCK
    code[9'h059] = 9'h071;  // RIGHT SHIFT
    code[9'h05A] = 9'h049;  // ENTER
    code[9'h05B] = 9'h03d;  // ]
    code[9'h05C] = 9'h07b;
    code[9'h05D] = 9'h055;  // BACKSLASH
    code[9'h05E] = 9'h07b;
    code[9'h05F] = 9'h07b;

    code[9'h060] = 9'h07b;
    code[9'h061] = 9'h071;  //international left shift cut out (German '<>' key), 0x56 Set#1 code
    code[9'h062] = 9'h07b;
    code[9'h063] = 9'h07b;
    code[9'h064] = 9'h07b;
    code[9'h065] = 9'h07b;
    code[9'h066] = 9'h067;  // BACKSPACE
    code[9'h067] = 9'h07b;
    code[9'h068] = 9'h07b;
    code[9'h069] = 9'h127;  // KP 1
    code[9'h06A] = 9'h07b;
    code[9'h06B] = 9'h12d;  // KP 4
    code[9'h06C] = 9'h133;  // KP 7
    code[9'h06D] = 9'h07b;
    code[9'h06E] = 9'h07b;
    code[9'h06F] = 9'h07b;

    code[9'h070] = 9'h125;  // KP 0
    code[9'h071] = 9'h103;  // KP .
    code[9'h072] = 9'h129;  // KP 2
    code[9'h073] = 9'h12f;  // KP 5
    code[9'h074] = 9'h131;  // KP 6
    code[9'h075] = 9'h137;  // KP 8
    code[9'h076] = 9'h07b;  // ESCAPE
    code[9'h077] = 9'h07b;  // NUMLOCK (Mac keypad clear?)
    code[9'h078] = 9'h07b;  // F11 <OSD>
    code[9'h079] = 9'h10d;  // KP +
    code[9'h07A] = 9'h12b;  // KP 3
    code[9'h07B] = 9'h11d;  // KP -
    code[9'h07C] = 9'h105;  // KP *
    code[9'h07D] = 9'h139;  // KP 9
    code[9'h07E] = 9'h07b;  // SCROLL LOCK / KP )
    code[9'h07F] = 9'h07b;

    code[9'h080] = 9'h07b;
    code[9'h081] = 9'h07b;
    code[9'h082] = 9'h07b;
    code[9'h083] = 9'h07b;  // F7
    code[9'h084] = 9'h07b;
    code[9'h085] = 9'h07b;
    code[9'h086] = 9'h07b;
    code[9'h087] = 9'h07b;
    code[9'h088] = 9'h07b;
    code[9'h089] = 9'h07b;
    code[9'h08A] = 9'h07b;
    code[9'h08B] = 9'h07b;
    code[9'h08C] = 9'h07b;
    code[9'h08D] = 9'h07b;
    code[9'h08E] = 9'h07b;
    code[9'h08F] = 9'h07b;

    code[9'h090] = 9'h07b;
    code[9'h091] = 9'h07b;
    code[9'h092] = 9'h07b;
    code[9'h093] = 9'h07b;
    code[9'h094] = 9'h07b;
    code[9'h095] = 9'h07b;
    code[9'h096] = 9'h07b;
    code[9'h097] = 9'h07b;
    code[9'h098] = 9'h07b;
    code[9'h099] = 9'h07b;
    code[9'h09A] = 9'h07b;
    code[9'h09B] = 9'h07b;
    code[9'h09C] = 9'h07b;
    code[9'h09D] = 9'h07b;
    code[9'h09E] = 9'h07b;
    code[9'h09F] = 9'h07b;

    code[9'h0A0] = 9'h07b;
    code[9'h0A1] = 9'h07b;
    code[9'h0A2] = 9'h07b;
    code[9'h0A3] = 9'h07b;
    code[9'h0A4] = 9'h07b;
    code[9'h0A5] = 9'h07b;
    code[9'h0A6] = 9'h07b;
    code[9'h0A7] = 9'h07b;
    code[9'h0A8] = 9'h07b;
    code[9'h0A9] = 9'h07b;
    code[9'h0AA] = 9'h07b;
    code[9'h0AB] = 9'h07b;
    code[9'h0AC] = 9'h07b;
    code[9'h0AD] = 9'h07b;
    code[9'h0AE] = 9'h07b;
    code[9'h0AF] = 9'h07b;

    code[9'h0B0] = 9'h07b;
    code[9'h0B1] = 9'h07b;
    code[9'h0B2] = 9'h07b;
    code[9'h0B3] = 9'h07b;
    code[9'h0B4] = 9'h07b;
    code[9'h0B5] = 9'h07b;
    code[9'h0B6] = 9'h07b;
    code[9'h0B7] = 9'h07b;
    code[9'h0B8] = 9'h07b;
    code[9'h0B9] = 9'h07b;
    code[9'h0BA] = 9'h07b;
    code[9'h0BB] = 9'h07b;
    code[9'h0BC] = 9'h07b;
    code[9'h0BD] = 9'h07b;
    code[9'h0BE] = 9'h07b;
    code[9'h0BF] = 9'h07b;

    code[9'h0C0] = 9'h07b;
    code[9'h0C1] = 9'h07b;
    code[9'h0C2] = 9'h07b;
    code[9'h0C3] = 9'h07b;
    code[9'h0C4] = 9'h07b;
    code[9'h0C5] = 9'h07b;
    code[9'h0C6] = 9'h07b;
    code[9'h0C7] = 9'h07b;
    code[9'h0C8] = 9'h07b;
    code[9'h0C9] = 9'h07b;
    code[9'h0CA] = 9'h07b;
    code[9'h0CB] = 9'h07b;
    code[9'h0CC] = 9'h07b;
    code[9'h0CD] = 9'h07b;
    code[9'h0CE] = 9'h07b;
    code[9'h0CF] = 9'h07b;

    code[9'h0D0] = 9'h07b;
    code[9'h0D1] = 9'h07b;
    code[9'h0D2] = 9'h07b;
    code[9'h0D3] = 9'h07b;
    code[9'h0D4] = 9'h07b;
    code[9'h0D5] = 9'h07b;
    code[9'h0D6] = 9'h07b;
    code[9'h0D7] = 9'h07b;
    code[9'h0D8] = 9'h07b;
    code[9'h0D9] = 9'h07b;
    code[9'h0DA] = 9'h07b;
    code[9'h0DB] = 9'h07b;
    code[9'h0DC] = 9'h07b;
    code[9'h0DD] = 9'h07b;
    code[9'h0DE] = 9'h07b;
    code[9'h0DF] = 9'h07b;

    code[9'h0E0] = 9'h07b;  // ps2 extended key
    code[9'h0E1] = 9'h07b;
    code[9'h0E2] = 9'h07b;
    code[9'h0E3] = 9'h07b;
    code[9'h0E4] = 9'h07b;
    code[9'h0E5] = 9'h07b;
    code[9'h0E6] = 9'h07b;
    code[9'h0E7] = 9'h07b;
    code[9'h0E8] = 9'h07b;
    code[9'h0E9] = 9'h07b;
    code[9'h0EA] = 9'h07b;
    code[9'h0EB] = 9'h07b;
    code[9'h0EC] = 9'h07b;
    code[9'h0ED] = 9'h07b;
    code[9'h0EE] = 9'h07b;
    code[9'h0EF] = 9'h07b;

    code[9'h0F0] = 9'h07b;  // ps2 release code
    code[9'h0F1] = 9'h07b;
    code[9'h0F2] = 9'h07b;
    code[9'h0F3] = 9'h07b;
    code[9'h0F4] = 9'h07b;
    code[9'h0F5] = 9'h07b;
    code[9'h0F6] = 9'h07b;
    code[9'h0F7] = 9'h07b;
    code[9'h0F8] = 9'h07b;
    code[9'h0F9] = 9'h07b;
    code[9'h0FA] = 9'h07b;  // ps2 ack code
    code[9'h0FB] = 9'h07b;
    code[9'h0FC] = 9'h07b;
    code[9'h0FD] = 9'h07b;
    code[9'h0FE] = 9'h07b;
    code[9'h0FF] = 9'h07b;

    code[9'h100] = 9'h07b;
    code[9'h101] = 9'h07b;
    code[9'h102] = 9'h07b;
    code[9'h103] = 9'h07b;
    code[9'h104] = 9'h07b;
    code[9'h105] = 9'h07b;
    code[9'h106] = 9'h07b;
    code[9'h107] = 9'h07b;
    code[9'h108] = 9'h07b;
    code[9'h109] = 9'h07b;
    code[9'h10A] = 9'h07b;
    code[9'h10B] = 9'h07b;
    code[9'h10C] = 9'h07b;
    code[9'h10D] = 9'h07b;
    code[9'h10E] = 9'h07b;
    code[9'h10F] = 9'h07b;

    code[9'h110] = 9'h07b;
    code[9'h111] = 9'h06f;  // RIGHT ALT (command)
    code[9'h112] = 9'h07b;
    code[9'h113] = 9'h07b;
    code[9'h114] = 9'h07b;
    code[9'h115] = 9'h07b;
    code[9'h116] = 9'h07b;
    code[9'h117] = 9'h07b;
    code[9'h118] = 9'h07b;
    code[9'h119] = 9'h07b;
    code[9'h11A] = 9'h07b;
    code[9'h11B] = 9'h07b;
    code[9'h11C] = 9'h07b;
    code[9'h11D] = 9'h07b;
    code[9'h11E] = 9'h07b;
    code[9'h11F] = 9'h075;  // WINDOWS OR APPLICATION KEY (option)

    code[9'h120] = 9'h07b;
    code[9'h121] = 9'h07b;
    code[9'h122] = 9'h07b;
    code[9'h123] = 9'h07b;
    code[9'h124] = 9'h07b;
    code[9'h125] = 9'h07b;
    code[9'h126] = 9'h07b;
    code[9'h127] = 9'h07b;
    code[9'h128] = 9'h07b;
    code[9'h129] = 9'h07b;
    code[9'h12A] = 9'h07b;
    code[9'h12B] = 9'h07b;
    code[9'h12C] = 9'h07b;
    code[9'h12D] = 9'h07b;
    code[9'h12E] = 9'h07b;
    code[9'h12F] = 9'h07b;

    code[9'h130] = 9'h07b;
    code[9'h131] = 9'h07b;
    code[9'h132] = 9'h07b;
    code[9'h133] = 9'h07b;
    code[9'h134] = 9'h07b;
    code[9'h135] = 9'h07b;
    code[9'h136] = 9'h07b;
    code[9'h137] = 9'h07b;
    code[9'h138] = 9'h07b;
    code[9'h139] = 9'h07b;
    code[9'h13A] = 9'h07b;
    code[9'h13B] = 9'h07b;
    code[9'h13C] = 9'h07b;
    code[9'h13D] = 9'h07b;
    code[9'h13E] = 9'h07b;
    code[9'h13F] = 9'h07b;

    code[9'h140] = 9'h07b;
    code[9'h141] = 9'h07b;
    code[9'h142] = 9'h07b;
    code[9'h143] = 9'h07b;
    code[9'h144] = 9'h07b;
    code[9'h145] = 9'h07b;
    code[9'h146] = 9'h07b;
    code[9'h147] = 9'h07b;
    code[9'h148] = 9'h07b;
    code[9'h149] = 9'h07b;
    code[9'h14A] = 9'h11b;  // KP /
    code[9'h14B] = 9'h07b;
    code[9'h14C] = 9'h07b;
    code[9'h14D] = 9'h07b;
    code[9'h14E] = 9'h07b;
    code[9'h14F] = 9'h07b;

    code[9'h150] = 9'h07b;
    code[9'h151] = 9'h07b;
    code[9'h152] = 9'h07b;
    code[9'h153] = 9'h07b;
    code[9'h154] = 9'h07b;
    code[9'h155] = 9'h07b;
    code[9'h156] = 9'h07b;
    code[9'h157] = 9'h07b;
    code[9'h158] = 9'h07b;
    code[9'h159] = 9'h07b;
    code[9'h15A] = 9'h119;  // KP ENTER
    code[9'h15B] = 9'h07b;
    code[9'h15C] = 9'h07b;
    code[9'h15D] = 9'h07b;
    code[9'h15E] = 9'h07b;
    code[9'h15F] = 9'h07b;

    code[9'h160] = 9'h07b;
    code[9'h161] = 9'h07b;
    code[9'h162] = 9'h07b;
    code[9'h163] = 9'h07b;
    code[9'h164] = 9'h07b;
    code[9'h165] = 9'h07b;
    code[9'h166] = 9'h07b;
    code[9'h167] = 9'h07b;
    code[9'h168] = 9'h07b;
    code[9'h169] = 9'h07b;  // END
    code[9'h16A] = 9'h07b;
    code[9'h16B] = 9'h10d;  // ARROW LEFT
    code[9'h16C] = 9'h07b;  // HOME
    code[9'h16D] = 9'h07b;
    code[9'h16E] = 9'h07b;
    code[9'h16F] = 9'h07b;

    code[9'h170] = 9'h07b;  // INSERT = HELP
    code[9'h171] = 9'h10f;  // DELETE (KP clear?)
    code[9'h172] = 9'h111;  // ARROW DOWN
    code[9'h173] = 9'h07b;
    code[9'h174] = 9'h105;  // ARROW RIGHT
    code[9'h175] = 9'h11b;  // ARROW UP
    code[9'h176] = 9'h07b;
    code[9'h177] = 9'h07b;
    code[9'h178] = 9'h07b;
    code[9'h179] = 9'h07b;
    code[9'h17A] = 9'h07b;  // PGDN <OSD>
    code[9'h17B] = 9'h07b;
    code[9'h17C] = 9'h07b;  // PRTSCR <OSD>
    code[9'h17D] = 9'h07b;  // PGUP <OSD>
    code[9'h17E] = 9'h07b;  // ctrl+break
    code[9'h17F] = 9'h07b;

    code[9'h180] = 9'h07b;
    code[9'h181] = 9'h07b;
    code[9'h182] = 9'h07b;
    code[9'h183] = 9'h07b;
    code[9'h184] = 9'h07b;
    code[9'h185] = 9'h07b;
    code[9'h186] = 9'h07b;
    code[9'h187] = 9'h07b;
    code[9'h188] = 9'h07b;
    code[9'h189] = 9'h07b;
    code[9'h18A] = 9'h07b;
    code[9'h18B] = 9'h07b;
    code[9'h18C] = 9'h07b;
    code[9'h18D] = 9'h07b;
    code[9'h18E] = 9'h07b;
    code[9'h18F] = 9'h07b;

    code[9'h190] = 9'h07b;
    code[9'h191] = 9'h07b;
    code[9'h192] = 9'h07b;
    code[9'h193] = 9'h07b;
    code[9'h194] = 9'h07b;
    code[9'h195] = 9'h07b;
    code[9'h196] = 9'h07b;
    code[9'h197] = 9'h07b;
    code[9'h198] = 9'h07b;
    code[9'h199] = 9'h07b;
    code[9'h19A] = 9'h07b;
    code[9'h19B] = 9'h07b;
    code[9'h19C] = 9'h07b;
    code[9'h19D] = 9'h07b;
    code[9'h19E] = 9'h07b;
    code[9'h19F] = 9'h07b;

    code[9'h1A0] = 9'h07b;
    code[9'h1A1] = 9'h07b;
    code[9'h1A2] = 9'h07b;
    code[9'h1A3] = 9'h07b;
    code[9'h1A4] = 9'h07b;
    code[9'h1A5] = 9'h07b;
    code[9'h1A6] = 9'h07b;
    code[9'h1A7] = 9'h07b;
    code[9'h1A8] = 9'h07b;
    code[9'h1A9] = 9'h07b;
    code[9'h1AA] = 9'h07b;
    code[9'h1AB] = 9'h07b;
    code[9'h1AC] = 9'h07b;
    code[9'h1AD] = 9'h07b;
    code[9'h1AE] = 9'h07b;
    code[9'h1AF] = 9'h07b;

    code[9'h1B0] = 9'h07b;
    code[9'h1B1] = 9'h07b;
    code[9'h1B2] = 9'h07b;
    code[9'h1B3] = 9'h07b;
    code[9'h1B4] = 9'h07b;
    code[9'h1B5] = 9'h07b;
    code[9'h1B6] = 9'h07b;
    code[9'h1B7] = 9'h07b;
    code[9'h1B8] = 9'h07b;
    code[9'h1B9] = 9'h07b;
    code[9'h1BA] = 9'h07b;
    code[9'h1BB] = 9'h07b;
    code[9'h1BC] = 9'h07b;
    code[9'h1BD] = 9'h07b;
    code[9'h1BE] = 9'h07b;
    code[9'h1BF] = 9'h07b;

    code[9'h1C0] = 9'h07b;
    code[9'h1C1] = 9'h07b;
    code[9'h1C2] = 9'h07b;
    code[9'h1C3] = 9'h07b;
    code[9'h1C4] = 9'h07b;
    code[9'h1C5] = 9'h07b;
    code[9'h1C6] = 9'h07b;
    code[9'h1C7] = 9'h07b;
    code[9'h1C8] = 9'h07b;
    code[9'h1C9] = 9'h07b;
    code[9'h1CA] = 9'h07b;
    code[9'h1CB] = 9'h07b;
    code[9'h1CC] = 9'h07b;
    code[9'h1CD] = 9'h07b;
    code[9'h1CE] = 9'h07b;
    code[9'h1CF] = 9'h07b;

    code[9'h1D0] = 9'h07b;
    code[9'h1D1] = 9'h07b;
    code[9'h1D2] = 9'h07b;
    code[9'h1D3] = 9'h07b;
    code[9'h1D4] = 9'h07b;
    code[9'h1D5] = 9'h07b;
    code[9'h1D6] = 9'h07b;
    code[9'h1D7] = 9'h07b;
    code[9'h1D8] = 9'h07b;
    code[9'h1D9] = 9'h07b;
    code[9'h1DA] = 9'h07b;
    code[9'h1DB] = 9'h07b;
    code[9'h1DC] = 9'h07b;
    code[9'h1DD] = 9'h07b;
    code[9'h1DE] = 9'h07b;
    code[9'h1DF] = 9'h07b;

    code[9'h1E0] = 9'h07b;  // ps2 extended key(duplicate, see $e0)
    code[9'h1E1] = 9'h07b;
    code[9'h1E2] = 9'h07b;
    code[9'h1E3] = 9'h07b;
    code[9'h1E4] = 9'h07b;
    code[9'h1E5] = 9'h07b;
    code[9'h1E6] = 9'h07b;
    code[9'h1E7] = 9'h07b;
    code[9'h1E8] = 9'h07b;
    code[9'h1E9] = 9'h07b;
    code[9'h1EA] = 9'h07b;
    code[9'h1EB] = 9'h07b;
    code[9'h1EC] = 9'h07b;
    code[9'h1ED] = 9'h07b;
    code[9'h1EE] = 9'h07b;
    code[9'h1EF] = 9'h07b;

    code[9'h1F0] = 9'h07b;  // ps2 release code(duplicate, see $f0)
    code[9'h1F1] = 9'h07b;
    code[9'h1F2] = 9'h07b;
    code[9'h1F3] = 9'h07b;
    code[9'h1F4] = 9'h07b;
    code[9'h1F5] = 9'h07b;
    code[9'h1F6] = 9'h07b;
    code[9'h1F7] = 9'h07b;
    code[9'h1F8] = 9'h07b;
    code[9'h1F9] = 9'h07b;
    code[9'h1FA] = 9'h07b;  // ps2 ack code(duplicate see $fa)
    code[9'h1FB] = 9'h07b;
    code[9'h1FC] = 9'h07b;
    code[9'h1FD] = 9'h07b;
    code[9'h1FE] = 9'h07b;
    code[9'h1FF] = 9'h07b;
  end // initial begin

  //use BRAM for table
  assign key_code = code[key_addr];
  always @(posedge clk) key_addr <= {extended,ps2_key[7:0]};

endmodule
