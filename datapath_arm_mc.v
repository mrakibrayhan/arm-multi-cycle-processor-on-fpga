//====================================================================
// datapath_arm_mc.v   (ARMv4 multi-cycle CPU, step 4)  -  multi-cycle datapath (spec section 5)
//
//   Harris & Harris ARM multi-cycle datapath (section 7.4.1): PC, unified memory port (Adr), Instr register,
//   register file with A / WriteData, Extend unit, ALU with SrcA / SrcB muxes, ALUOut register, Result mux whose
//   output goes to the PC, to the register file write port and to the R15 input.
//   Extensions for the full ARMv4 subset (all marked in the spec): barrel shifter on the WriteData path, ShAmt latch
//   (shift by register), multiplier with MulHi / Creg / Zlo (long multiplies), LDM/STM sequencer hooks, two more
//   inputs on AdrSrc and ALUSrcA, product input on ALUSrcB.
//
//   The memory itself is outside (cpu_core_arm_mc.v): this module gives 'adr' / 'wdata' and takes the raw word
//   'q_word' and the aligned load data 'rdata' (both synchronous: valid in the cycle AFTER the address).
//
//   ins = the instruction seen by the controller and by every instruction-field user:
//         DECODE: the memory output (the fetched word);  other states: the Instr register (loaded at the end of DECODE).
//
//   Select encodings (spec section 5):
//     AdrSrc   0 PC   1 ALUOut   2 A   3 ALUResult          ALUSrcA 0 A   1 PC   2 ALUOut   3 MulHi
//     ALUSrcB  0 shifted WriteData   1 ExtImm   2 4   3 product[31:0]        ResultSrc 0 ALUOut   1 Data   2 ALUResult
//     RA1Sel   0 Rn   1 R15   2 Rm                         RA2Sel  0 Rm  1 Rs  2 Rd  3 Instr[19:16]  4 sequencer
//     A3Sel    0 Rd   1 Rn   2 R14   3 sequencer (LDM)
//====================================================================
module datapath_arm_mc #(
    parameter ROT_EN   = 1,        // rotated immediates      (FEATURES[1])
    parameter SH_EN    = 1,        // barrel shifter          (FEATURES[1])
    parameter SHREG_EN = 1,        // shift amount from Rs    (FEATURES[2])
    parameter MUL_EN   = 1,        // MUL / MLA               (FEATURES[8] | FEATURES[9])
    parameter MULL_EN  = 1,        // long multiplies         (FEATURES[9])
    parameter LDM_EN   = 1         // LDM / STM               (FEATURES[10])
) (
    input         clk,
    input         rst,
    input         en,              // 0 = hold everything (CPU stalled by a freeze in FETCH, or stopped)
    // memory port
    output [31:0] adr,
    output [31:0] wdata,
    input  [31:0] q_word,
    input  [31:0] rdata,
    output [31:0] ins,
    output [31:0] pc,
    // control (from the controller; RegWrite / PCWrite already gated with CondExR)
    input         IRWrite,
    input         R15W,
    input         PCWrite,
    input         RegWrite,
    input  [1:0]  AdrSrc,
    input  [1:0]  ALUSrcA,
    input  [1:0]  ALUSrcB,
    input  [1:0]  ResultSrc,
    input  [3:0]  ALUControl,
    input         CinSel,
    input  [1:0]  ImmSrc,
    input         ShDis,
    input         ShByReg,
    input         ShAmtW,
    input  [1:0]  RA1Sel,
    input  [2:0]  RA2Sel,
    input         Zero2,
    input  [1:0]  A3Sel,
    input         ALUOutW,
    input         MulHiW,
    input         CregW,
    input         ZloUse,
    input         CSrc,
    input         ImmCarry,       // C of a rotated immediate (EXECI) instead of the shifter carry
    input  [3:0]  seq_pos,
    input  [3:0]  seq_reg,        // register of the previous LDM beat (write target)
    // flags
    input  [3:0]  flags,
    output [3:0]  nzcv_in,
    output        res0,           // bit 0 of the ALU result (BX target)
    output [3:0]  a3,             // write address (trace)
    output [31:0] result,         // Result (trace)
    // board view (cpu_core_arm_mc DBG_EN): RA1 = dbg_ra, adr = dbg_adr
    input         view,
    input  [3:0]  dbg_ra,
    input  [31:0] dbg_adr,
    output [31:0] a_out
);
    // ---------------- PC, Instr ----------------
    reg [31:0] pc_r = 32'd0, instr_r = 32'd0;
    assign pc  = pc_r;
    assign ins = IRWrite ? q_word : instr_r;

    // ---------------- register file ----------------
    wire [3:0] rn = ins[19:16], rd = ins[15:12], rs = ins[11:8], rm = ins[3:0];
    wire [3:0] ra1 = view ? dbg_ra : (RA1Sel == 2'd0) ? rn : (RA1Sel == 2'd1) ? 4'd15 : rm;
    wire [3:0] ra2 = (RA2Sel == 3'd0) ? rm : (RA2Sel == 3'd1) ? rs : (RA2Sel == 3'd2) ? rd : (RA2Sel == 3'd3) ? rn : seq_pos;
    assign a3      = (A3Sel == 2'd0) ? rd : (A3Sel == 2'd1) ? rn : (A3Sel == 2'd2) ? 4'd14 : seq_reg;
    wire [31:0] A, WD;
    regfile_arm_mc rf (.clk(clk), .rst(rst), .ra1(ra1), .ra2(ra2), .zero2(Zero2), .a(A), .wd(WD),
                       .r15_we(R15W & en), .r15_in(result), .we(RegWrite & en), .wa(a3), .wdat(result));
    assign a_out = A;
    assign wdata = WD;

    // ---------------- extend unit, barrel shifter ----------------
    wire [31:0] ext;
    ext_arm #(.ROT_EN(ROT_EN)) ex (.instr24(ins[23:0]), .ImmSrc(ImmSrc), .ext(ext));

    reg  [7:0]  shamt = 8'd0;                                     // ShAmt latch [FUR]: Rs[7:0], loaded in RSREAD
    always @(posedge clk) if (en & ShAmtW) shamt <= WD[7:0];
    wire [31:0] shifted;
    wire        sh_c;
    shifter_arm #(.SH_EN(SH_EN), .REG_EN(SHREG_EN)) shf (
        .din(WD), .typ(ins[6:5]), .amt_imm(ins[11:7]), .amt_reg(shamt), .by_reg(ShByReg), .dis(ShDis),
        .cin(flags[1]), .dout(shifted), .cout(sh_c));
    wire imm_c = (ROT_EN != 0 && ins[11:8] != 4'd0) ? ext[31] : flags[1];

    // ---------------- multiplier ----------------
    wire [63:0] prod;
    mul_arm_mc #(.MUL_EN(MUL_EN), .LONG_EN(MULL_EN)) mul (.a(A), .b(WD), .sgn(ins[22]), .p(prod));
    reg [31:0] mulhi = 32'd0;
    reg        creg = 1'b0, zlo = 1'b0;

    // ---------------- ALU ----------------
    reg  [31:0] aluout = 32'd0;
    wire [31:0] srca = (ALUSrcA == 2'd0) ? A : (ALUSrcA == 2'd1) ? pc_r : (ALUSrcA == 2'd2) ? aluout : mulhi;
    wire [31:0] srcb = (ALUSrcB == 2'd0) ? shifted : (ALUSrcB == 2'd1) ? ext : (ALUSrcB == 2'd2) ? 32'd4 : prod[31:0];
    wire [31:0] alu_res;
    wire        alu_n, alu_z, alu_c, alu_v;
    alu_arm alu (.A(srca), .B(srcb), .ALUControl(ALUControl), .Cin(CinSel ? creg : flags[1]), .Result(alu_res),
                 .N(alu_n), .Z(alu_z), .Cadd(alu_c), .Vadd(alu_v));
    assign res0 = alu_res[0];
    assign nzcv_in = {alu_n, ZloUse ? (alu_z & zlo) : alu_z, CSrc ? (ImmCarry ? imm_c : sh_c) : alu_c, alu_v};

    // ---------------- result, address ----------------
    assign result = (ResultSrc == 2'd0) ? aluout : (ResultSrc == 2'd1) ? rdata : alu_res;
    assign adr    = view ? dbg_adr : (AdrSrc == 2'd0) ? pc_r : (AdrSrc == 2'd1) ? aluout : (AdrSrc == 2'd2) ? A : alu_res;

    // ---------------- registers ----------------
    generate
        if (MULL_EN) begin : g_mull
            always @(posedge clk) if (en) begin
                if (MulHiW) mulhi <= prod[63:32];
                if (CregW) begin creg <= alu_c; zlo <= alu_z; end
            end
        end
    endgenerate
    always @(posedge clk) begin
        if (rst) pc_r <= 32'd0;
        else if (en) begin
            if (PCWrite) pc_r <= {result[31:2], 2'b00};
            if (IRWrite) instr_r <= q_word;
            if (ALUOutW) aluout <= alu_res;
        end
    end
endmodule
