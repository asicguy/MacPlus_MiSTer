/* verilator lint_off UNUSED */
/* verilator lint_off SYNCASYNCNET */

// scsi.v
// implements a target only scsi device

module scsi
(
        input      clk,

        // scsi interface
        input     rst, // bus reset from initiator
        input     sel,
        input     atn, // initiator requests to send a message
        output    bsy, // target holds bus

        output    msg,
        output    cd,
        output    io,

        output    req,
        input     ack, // initiator acknowledges a request

        input   [7:0] din, // data from initiator to target
        output  [7:0] dout, // data from target to initiator

        // interface to io controller
        output [31:0] io_lba,
        output reg        io_rd,
        output reg        io_wr,
        input        io_ack,

        input      [8:0] sd_buff_addr,
        input      [7:0] sd_buff_dout,
        output reg [7:0] sd_buff_din,
        input            sd_buff_wr
);


// SCSI device id
parameter [7:0] ID = 0;

`define PHASE_IDLE        3'd0
`define PHASE_CMD_IN      3'd1
`define PHASE_DATA_OUT    3'd2
`define PHASE_DATA_IN     3'd3
`define PHASE_STATUS_OUT  3'd4
`define PHASE_MESSAGE_OUT 3'd5
reg [2:0]  phase;
  wire [7:0] cmd_dout;
// https://en.wikipedia.org/wiki/SCSI_command
  wire       cmd_read;
  wire       cmd_read6;
  wire       cmd_read10;
  wire       cmd_write;
  wire       cmd_write6;
  wire       cmd_write10;
  wire       cmd_inquiry;
  wire       cmd_format;
  wire       cmd_mode_select;
  wire       cmd_mode_sense;
  wire       cmd_test_unit_ready;
  wire       cmd_read_capacity;
  reg [7:0]  buffer_dout;
  wire [7:0] inquiry_dout;
  wire [7:0] read_capacity_dout;
  wire [7:0] mode_sense_dout;

// count data bytes. don't increase counter while we are waiting for data from
// the io controller
reg [31:0] data_cnt;
reg        data_complete;

// latch parameters once command is complete
reg [31:0] lba;
reg [15:0] tlen;

// parse commands
  wire [7:0] op_code;
  wire [2:0] cmd_group;

// check if a complete command has been received
  wire       cmd_cpl;
  wire       cmd6_cpl;
  wire       cmd10_cpl;

// logical block address
  wire [7:0] cmd1;
  wire [20:0] lba6;
  wire [31:0] lba10;

// transfer length
  wire [8:0]  tlen6;
  wire [15:0] tlen10;

// ---------------- buffer read engine -----------------------
// the buffer itself. Can hold one sector
reg [7:0] buffer_out [512];
always @(posedge clk) sd_buff_din <= buffer_out[sd_buff_addr];

// ---------------- buffer write engine ----------------------
// the buffer itself. Can hold one sector
reg [7:0] buffer_in [512];
always @(posedge clk) if(sd_buff_wr) buffer_in[sd_buff_addr] <= sd_buff_dout;

// -----------------------------------------------------------

// status replies
reg [7:0]  status;
`define STATUS_OK 8'h00
`define STATUS_CHECK_CONDITION 8'h02

// message codes
`define MSG_CMD_COMPLETE 8'h00

// drive scsi signals according to phase
assign msg = (phase == `PHASE_MESSAGE_OUT);
assign cd = (phase == `PHASE_CMD_IN) || (phase == `PHASE_STATUS_OUT) || (phase == `PHASE_MESSAGE_OUT);
assign io = (phase == `PHASE_DATA_OUT) || (phase == `PHASE_STATUS_OUT) || (phase == `PHASE_MESSAGE_OUT);
assign req = (phase != `PHASE_IDLE) && !ack && !io_rd && !io_wr && !io_ack;
assign bsy = (phase != `PHASE_IDLE);

assign dout = (phase == `PHASE_STATUS_OUT)?status:
         (phase == `PHASE_MESSAGE_OUT)?`MSG_CMD_COMPLETE:
         (phase == `PHASE_DATA_OUT)?cmd_dout:
         8'h00;

// de-multiplex different data sources
assign cmd_dout =
                cmd_read?buffer_dout:
                cmd_inquiry?inquiry_dout:
                cmd_read_capacity?read_capacity_dout:
                cmd_mode_sense?mode_sense_dout:
                8'h00;

// output of inquiry command, identify as "SEAGATE ST225N"
  assign inquiry_dout =
                (data_cnt == 32'd4 )?8'd32:  // length

                (data_cnt == 32'd8 )?" ":(data_cnt == 32'd9 )?"S":
                (data_cnt == 32'd10)?"E":(data_cnt == 32'd11)?"A":
                (data_cnt == 32'd12)?"G":(data_cnt == 32'd13)?"A":
                (data_cnt == 32'd14)?"T":(data_cnt == 32'd15)?"E":
                (data_cnt == 32'd16)?" ":(data_cnt == 32'd17)?" ":
                (data_cnt == 32'd18)?" ":(data_cnt == 32'd19)?" ":
                (data_cnt == 32'd20)?" ":(data_cnt == 32'd21)?" ":
                (data_cnt == 32'd22)?" ":(data_cnt == 32'd23)?" ":
                (data_cnt == 32'd24)?" ":(data_cnt == 32'd25)?" ":

                (data_cnt == 32'd26)?"S":(data_cnt == 32'd27)?"T":
                (data_cnt == 32'd28)?"2":(data_cnt == 32'd29)?"2":
                (data_cnt == 32'd30)?"5":(data_cnt == 32'd31)?"N" + ID:	// TESTING. ElectronAsh.
                8'h00;

// output of read capacity command
//wire [31:0] capacity = 32'd41056;   // 40960 + 96 blocks = 20MB
wire [31:0] capacity = 32'd1024096;   // 1024000 + 96 blocks = 500MB
wire [31:0] capacity_m1 = capacity - 32'd1;
  assign read_capacity_dout =
                (data_cnt == 32'd0 )?capacity_m1[31:24]:
                (data_cnt == 32'd1 )?capacity_m1[23:16]:
                (data_cnt == 32'd2 )?capacity_m1[15:8]:
                (data_cnt == 32'd3 )?capacity_m1[7:0]:
                (data_cnt == 32'd6 )?8'd2:             // 512 bytes per sector
                8'h00;

  assign mode_sense_dout =
                (data_cnt == 32'd3 )?8'd8:
                (data_cnt == 32'd5 )?capacity[23:16]:
                (data_cnt == 32'd6 )?capacity[15:8]:
                (data_cnt == 32'd7 )?capacity[7:0]:
                (data_cnt == 32'd10 )?8'd2:
                8'h00;

// clock data out of buffer to allow for embedded ram
always @(posedge clk) buffer_dout <= buffer_in[data_cnt];

// buffer to store incoming commands
reg [3:0]  cmd_cnt;
reg [7:0]  cmd [9:0];

/* ----------------------- request data from/to io controller ----------------------- */

// base address of current block. Subtract one when writing since the writing happens
// after a block has been transferred and data_cnt has thus already been increased by 512
assign io_lba = lba + { 9'd0, data_cnt[31:9] } -
                (cmd_write ? 32'd1 : 32'd0);

wire req_rd = ((phase == `PHASE_DATA_OUT) && cmd_read && (data_cnt[8:0] == 0) && !data_complete);
wire req_wr = ((((phase == `PHASE_DATA_IN) && (data_cnt[8:0] == 0) && (data_cnt != 0)) || (phase == `PHASE_STATUS_OUT)) && cmd_write);
always @(posedge clk) begin
        reg old_rd, old_wr;

        old_rd <= req_rd;
        old_wr <= req_wr;

        if(io_ack) begin
                io_rd <= 1'b0;
                io_wr <= 1'b0;
        end else begin
                // generate an io_rd signal whenever the first byte of a 512 byte block is required and io_wr whenever
                // the last byte of a 512 byte block has been revceived
                if(~old_rd & req_rd) io_rd <= 1;

                // generate an io_wr signal whenever a 512 byte block has been received or when the status
                // phase of a write command has been reached
                if(~old_wr & req_wr) io_wr <= 1;
        end
end

reg  stb_ack;
reg  stb_adv;
always @(posedge clk) begin
        reg old_ack;

        old_ack <= ack;
        stb_ack <= (~old_ack & ack);
        stb_adv <= stb_ack;
end

// store data on rising edge of ack, ...
always @(posedge clk) begin
        if(stb_ack) begin
                if(phase == `PHASE_CMD_IN)  cmd[cmd_cnt] <= din;
                if(phase == `PHASE_DATA_IN) buffer_out[data_cnt] <= din;
        end
end

// ... advance counter on falling edge
always @(posedge clk) begin
        if(phase == `PHASE_IDLE) cmd_cnt <= 4'd0;
        else if(stb_adv && (phase == `PHASE_CMD_IN) && (cmd_cnt != 15)) cmd_cnt <= cmd_cnt + 4'd1;
end

// For block transfers tlen contains the number of 512 bytes blocks to transfer.
// Most other commands have the bytes length stored in the transfer length field.
// And some have a fixed length idependent from any header field.
// The data transfer has finished once the data counter reaches this
// number.
wire [31:0] data_len =
                 cmd_read_capacity?32'd8:
                 cmd_read?{ 7'd0, tlen, 9'd0 }:   // read command length is in 512 bytes blocks
                 cmd_write?{ 7'd0, tlen, 9'd0 }:  // write command length is in 512 bytes blocks
                 { 16'd0, tlen };                 // inquiry etc have length in bytes

always @(posedge clk) begin
        if((phase != `PHASE_DATA_OUT) && (phase != `PHASE_DATA_IN) && (phase != `PHASE_STATUS_OUT) && (phase != `PHASE_MESSAGE_OUT)) begin
                data_cnt <= 0;
                data_complete <= 0;
        end else begin
                if(stb_adv)begin
                        if(!data_complete) data_cnt <= data_cnt + 1'd1;
                        data_complete <= (data_len - 1'd1) == data_cnt;
                end
        end
end

// check whether status byte has been sent
reg status_sent;
always @(posedge clk) begin
        if(phase != `PHASE_STATUS_OUT) status_sent <= 0;
        else if(stb_adv) status_sent <= 1;
end

// check whether message byte has been sent
reg message_sent;
always @(posedge clk) begin
        if(phase != `PHASE_MESSAGE_OUT) message_sent <= 0;
        else if(stb_adv) message_sent <= 1;
end

/* ----------------------- command decoding ------------------------------- */


// parse commands
  assign op_code = cmd[0];
  assign cmd_group = op_code[7:5];

// check if a complete command has been received
assign       cmd_cpl = cmd6_cpl || cmd10_cpl;
assign       cmd6_cpl = (cmd_group == 3'b000) && (cmd_cnt == 6);
  assign      cmd10_cpl = ((cmd_group == 3'b010) || (cmd_group == 3'b001)) && (cmd_cnt == 10);

// https://en.wikipedia.org/wiki/SCSI_command
assign       cmd_read = cmd_read6 || cmd_read10;
assign       cmd_read6 = (op_code == 8'h08);
assign       cmd_read10 = (op_code == 8'h28);
assign       cmd_write = cmd_write6 || cmd_write10;
assign       cmd_write6 = (op_code == 8'h0a);
assign       cmd_write10 = (op_code == 8'h2a);
assign       cmd_inquiry = (op_code == 8'h12);
assign       cmd_format = (op_code == 8'h04);
assign       cmd_mode_select = (op_code == 8'h15);
assign       cmd_mode_sense = (op_code == 8'h1a);
assign       cmd_test_unit_ready = (op_code == 8'h00);
assign       cmd_read_capacity = (op_code == 8'h25);

// valid command in buffer? TODO: check for valid command parameters
wire  cmd_ok = cmd_read || cmd_write || cmd_inquiry || cmd_test_unit_ready ||
                  cmd_read_capacity || cmd_mode_select || cmd_format || cmd_mode_sense;

always @(posedge clk) begin
        if(cmd_cpl && (phase == `PHASE_CMD_IN)) begin
                lba <= cmd6_cpl?{11'd0, lba6}:lba10;
                tlen <= cmd6_cpl?{7'd0, tlen6}:tlen10;
        end
end

// logical block address
assign cmd1 = cmd[1];
assign lba6 = { cmd1[4:0], cmd[2], cmd[3] };
assign lba10 = { cmd[2], cmd[3], cmd[4], cmd[5] };

// transfer length
assign  tlen6 = (cmd[4] == 0)?9'd256:{1'b0,cmd[4]};
assign tlen10 = { cmd[7], cmd[8] };


// the 5380 changes phase in the falling edge, thus we monitor it
// on the rising edge
always @(posedge clk) begin
        if(rst) begin
                phase <= `PHASE_IDLE;
        end else begin
                if(phase == `PHASE_IDLE) begin
                        if(sel && din[ID])  // own id on bus during selection?
                                phase <= `PHASE_CMD_IN;
                end

                else if(phase == `PHASE_CMD_IN) begin
                        // check if a full command is in the buffer
                        if(cmd_cpl) begin
                                // is this a supported and valid command?
                                if(cmd_ok) begin
                                        // yes, continue
                                        status <= `STATUS_OK;

                                        // continue according to command

                                        // these commands return data
                                        if(cmd_read || cmd_inquiry || cmd_read_capacity || cmd_mode_sense) phase <= `PHASE_DATA_OUT;
                                        // these commands receive dataa
                                        else if(cmd_write || cmd_mode_select) phase <= `PHASE_DATA_IN;
                                        // and all other valid commands are just "ok"
                                        else phase <= `PHASE_STATUS_OUT;
                                end else begin
                                        // no, report failure
                                        status <= `STATUS_CHECK_CONDITION;
                                        phase <= `PHASE_STATUS_OUT;
                                end
                        end
                end

                else if(phase == `PHASE_DATA_OUT) begin
                        if(data_complete) phase <= `PHASE_STATUS_OUT;
                end

                else if(phase == `PHASE_DATA_IN) begin
                        if(data_complete) phase <= `PHASE_STATUS_OUT;
                end

                else if(phase == `PHASE_STATUS_OUT) begin
                        if(status_sent) phase <= `PHASE_MESSAGE_OUT;
                end

                else if(phase == `PHASE_MESSAGE_OUT) begin
                        if(message_sent) phase <= `PHASE_IDLE;
                end

                else
                        phase <= `PHASE_IDLE;  // should never happen
        end
end


endmodule
