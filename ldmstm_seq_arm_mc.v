//====================================================================
// ldmstm_seq_arm_mc.v   (ARMv4 multi-cycle CPU, step 11)  -  LDM / STM register-list sequencer (spec section 6.2)
//
//   Same list scan as the board-proven single-cycle sequencer (ldmstm_seq_arm.v, no copy of the list):
//       cand = start ? list : list bits beyond lastpos (above it for U = 1, below it for U = 0)
//       one  = U ? lowest set bit of cand : highest     (AND/OR chains)      pos = index of 'one'
//       last = no other bit left in cand
//   but it runs ONE CYCLE AHEAD of the memory beats, because a synchronous register read has to be issued one cycle
//   before its data is used:
//       DECODE ('start')        : pos = r0          -> RA2 (STM data of beat 0);    reg_q <= r0
//       beat k ('step', LSMBEAT): pos = r(k+1)      -> RA2 (STM data of beat k+1);  reg_q <= r(k+1), reg_qq <= r(k)
//   so in beat k:  reg_q = r(k) (the register stored / loaded by this beat), last_q = 1 if r(k) is the last one,
//                  reg_qq = r(k-1) (LDM: its data arrives in beat k and is written there; LDMLAST writes r(N-1)).
//   beat0 = 1 in the first beat (the ALU then starts from A = Rn instead of ALUOut).
//   The running address is NOT here (the single-cycle sequencer had a 32-bit pointer and adder): the multi-cycle core
//   computes it in the ALU (ALUOut <= ALUOut +/- 4), spec 6.2.
//
//   EN = 0 : not built (FEATURES[10] = 0): constant outputs, the decoder rejects LDM/STM.
//====================================================================
module ldmstm_seq_arm_mc #(
    parameter EN = 1
) (
    input         clk,
    input         rst,
    input         start,         // DECODE of an LDM/STM (the list comes straight from the memory output)
    input         step,          // LSMBEAT
    input  [15:0] list,          // Instr[15:0]
    input         U,             // Instr[23]
    output [3:0]  pos,           // register to read next (RA2)
    output [3:0]  reg_q,         // register of the current beat
    output [3:0]  reg_qq,        // register of the previous beat
    output        last_q,        // the current beat is the last one
    output        beat0          // the current beat is the first one
);
    generate
        if (!EN) begin : g_none
            assign pos = 4'd0; assign reg_q = 4'd0; assign reg_qq = 4'd0; assign last_q = 1'b1; assign beat0 = 1'b1;
        end else begin : g_seq
            reg [3:0] lastpos = 4'd0, rq = 4'd0, rqq = 4'd0;
            reg       lq = 1'b0, b0 = 1'b0;

            wire [15:0] beyond;
            genvar g;
            for (g = 0; g < 16; g = g + 1) begin : g_bey
                assign beyond[g] = U ? (lastpos < g) : (lastpos > g);
            end
            wire [15:0] cand = start ? list : (list & beyond);

            reg [15:0] lo_bit, hi_bit;
            reg        seen_lo, seen_hi;
            integer    i;
            always @(*) begin
                seen_lo = 1'b0;
                for (i = 0; i < 16; i = i + 1) begin lo_bit[i] = cand[i] & ~seen_lo; seen_lo = seen_lo | cand[i]; end
                seen_hi = 1'b0;
                for (i = 15; i >= 0; i = i - 1) begin hi_bit[i] = cand[i] & ~seen_hi; seen_hi = seen_hi | cand[i]; end
            end
            wire [15:0] one = U ? lo_bit : hi_bit;
            reg  [3:0]  p;
            integer     k;
            always @(*) begin
                p = 4'd0;
                for (k = 0; k < 16; k = k + 1) if (one[k]) p = k[3:0];
            end
            wire last = ((cand & ~one) == 16'd0);
            assign pos = p;

            always @(posedge clk) begin
                if (rst) begin
                    b0 <= 1'b0; lq <= 1'b0;
                end else if (start | step) begin
                    lastpos <= p;
                    lq      <= last;
                    rq      <= p;
                    rqq     <= rq;
                    b0      <= start;
                end
            end
            assign reg_q = rq; assign reg_qq = rqq; assign last_q = lq; assign beat0 = b0;
        end
    endgenerate
endmodule
