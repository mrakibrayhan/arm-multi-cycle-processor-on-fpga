//====================================================================
// regfile_arm_mc.v   (ARMv4 multi-cycle CPU, step 3)  -  register file with SYNCHRONOUS read (two M9K copies)
//
//   Harris & Harris multi-cycle datapath (section 7.4.1): register file with read ports RA1 -> RD1 and RA2 -> RD2,
//   one write port (A3, WD3 = Result, RegWrite), and the non-architectural registers A <= RD1, WriteData <= RD2 loaded
//   every cycle. Here each read port is an M9K copy whose ADDRESS register is loaded every cycle, so the M9K outputs
//   ARE the registers A and WriteData (value of the register addressed in the previous cycle). Both copies are written
//   together (one write port), so they always hold the same r0..r14.
//
//   R15 = PC: not stored. [HH] connects Result to the R15 input; in DECODE Result = PC + 4 = instruction address + 8.
//   The datapath loads that value into the R15 LATCH (r15_we = 1 in DECODE), and a read of address 15 on either port
//   returns the latch for the rest of the instruction (a synchronous M9K cannot be read "combinationally" in a later
//   state, so the R15 value is held instead). A write to address 15 is ignored here (it is a PC write, datapath).
//
//   zero2 = 1 : port 2 returns 0 (the non-accumulating long multiplies use the UMLAL states with a zero accumulator).
//   The output select (is15 / zero / M9K bit) is one 4-input LUT per bit: no extra logic levels.
//
//   Read during write: a read issued in the cycle in which the same register is written is never used by the CPU
//   (spec section 6), so the M9K mixed-port read-during-write behaviour is don't care (no_rw_check: no bypass logic).
//
//   RESET: rst = 1 clears r0..r14 through the write port, one register per cycle (counter 'clr'). rst must be held at
//   least 16 cycles (the board holds it for milliseconds; the testbenches for 20 cycles). The single-cycle core cleared
//   its flip-flop register file in one cycle; an M9K cannot be cleared in one cycle.
//====================================================================
module regfile_arm_mc (
    input         clk,
    input         rst,
    // read ports (addresses captured at the rising edge, data in the next cycle)
    input  [3:0]  ra1,
    input  [3:0]  ra2,
    input         zero2,
    output [31:0] a,            // A         = R[ra1 of the previous cycle]
    output [31:0] wd,           // WriteData = R[ra2 of the previous cycle] (or 0)
    // R15 latch
    input         r15_we,       // DECODE
    input  [31:0] r15_in,       // Result (= PC + 8 in DECODE)
    // write port
    input         we,
    input  [3:0]  wa,
    input  [31:0] wdat
);
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf1 [0:15];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf2 [0:15];
    integer i;
    initial for (i = 0; i < 16; i = i + 1) begin rf1[i] = 32'd0; rf2[i] = 32'd0; end

    reg  [3:0]  clr = 4'd0;
    wire        wen = rst | (we & (wa != 4'd15));
    wire [3:0]  wad = rst ? clr : wa;
    wire [31:0] wdd = rst ? 32'd0 : wdat;

    reg [31:0] q1, q2, r15 = 32'd0;
    reg        is15_1 = 1'b0, is15_2 = 1'b0, z2 = 1'b0;
    always @(posedge clk) begin
        if (wen) rf1[wad] <= wdd;
        q1 <= rf1[ra1];
    end
    always @(posedge clk) begin
        if (wen) rf2[wad] <= wdd;
        q2 <= rf2[ra2];
    end
    always @(posedge clk) begin
        clr    <= rst ? clr + 4'd1 : 4'd0;
        is15_1 <= (ra1 == 4'd15);
        is15_2 <= (ra2 == 4'd15);
        z2     <= zero2;
        if (r15_we) r15 <= r15_in;
    end
    assign a  = is15_1 ? r15 : q1;
    assign wd = z2 ? 32'd0 : is15_2 ? r15 : q2;
endmodule
