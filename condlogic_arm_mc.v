//====================================================================
// condlogic_arm_mc.v   (ARMv4 multi-cycle CPU, step 5)  -  condition check, NZCV flags, delayed CondEx
//
//   Harris & Harris multi-cycle "Conditional Logic" (section 7.4.2): the condition is evaluated on the current flags
//   and CondEx is DELAYED by a flip-flop (CondExR), because the flags of the instruction itself may change in its
//   Execute state before its write-back state uses CondEx. Here the condition is evaluated in DECODE (Instr comes
//   straight from the memory output) and held in CondExR for all later states of the instruction.
//
//   CondEx  : combinational, condition of 'cond' on the current flags (used in DECODE only: SWI halt).
//   CondExR : loaded in DECODE (cond_ld), gates every write of the instruction (RegWrite, MemWrite, PC write of a branch,
//             FlagWrite, halt) in the controller. It never changes the state sequence (H&H rule).
//   Flags are written at the rising edge when FlagW[i] & CondExR:  FlagW[2] N and Z, FlagW[1] C, FlagW[0] V.
//   cond = 1111 -> CondEx = 0 (illegal encoding, the decoder reports it).
//====================================================================
module condlogic_arm_mc (
    input        clk,
    input        rst,
    input  [3:0] cond,          // Instr[31:28] (DECODE: from the memory output)
    input        cond_ld,       // DECODE
    input  [3:0] nzcv_in,       // {N, Z, C, V} produced in this state
    input  [2:0] FlagW,         // from the controller (not yet gated)
    output       CondEx,
    output       CondExR,
    output [3:0] flags,
    output [3:0] flags_next     // value after the coming edge (trace only)
);
    reg [3:0] nzcv = 4'b0000;
    reg       cx = 1'b0;
    wire N = nzcv[3], Z = nzcv[2], C = nzcv[1], V = nzcv[0];
    reg ex;
    always @(*) begin
        case (cond)
            4'b0000: ex = Z;              4'b0001: ex = ~Z;
            4'b0010: ex = C;              4'b0011: ex = ~C;
            4'b0100: ex = N;              4'b0101: ex = ~N;
            4'b0110: ex = V;              4'b0111: ex = ~V;
            4'b1000: ex = C & ~Z;         4'b1001: ex = ~C | Z;
            4'b1010: ex = (N == V);       4'b1011: ex = (N != V);
            4'b1100: ex = ~Z & (N == V);  4'b1101: ex = Z | (N != V);
            4'b1110: ex = 1'b1;           default: ex = 1'b0;
        endcase
    end
    assign CondEx  = ex;
    assign CondExR = cx;
    assign flags   = nzcv;
    wire [2:0] fw = FlagW & {3{cx}};
    assign flags_next = rst ? 4'b0000 : {fw[2] ? nzcv_in[3:2] : nzcv[3:2], fw[1] ? nzcv_in[1] : nzcv[1], fw[0] ? nzcv_in[0] : nzcv[0]};
    always @(posedge clk) begin
        if (rst) begin
            nzcv <= 4'b0000; cx <= 1'b0;
        end else begin
            if (cond_ld) cx <= ex;
            if (fw[2]) nzcv[3:2] <= nzcv_in[3:2];
            if (fw[1]) nzcv[1]   <= nzcv_in[1];
            if (fw[0]) nzcv[0]   <= nzcv_in[0];
        end
    end
endmodule
