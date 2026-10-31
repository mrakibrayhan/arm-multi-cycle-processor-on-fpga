//====================================================================
// cpu_core_arm_mc.v   (ARMv4 multi-cycle CPU, step 5)  -  processor core: controller + datapath + memory
//
//   controller_arm_mc (main FSM) + condlogic_arm_mc (flags, CondExR) + datapath_arm_mc + ldmstm_seq_arm_mc
//   + mem_arm_mc (unified synchronous-read memory: ROM and RAM in M9K). Harris & Harris multi-cycle organisation.
//
//   BUILT : classes whose hardware is enabled in this build step (FEATURES bits, see decoder_arm.v). FEATURES is ANDed
//           with BUILT, so a class that is not built yet decodes as illegal (same mechanism as the single-cycle core).
//           Step 6: 0 (the H&H base set)   step 7: bits 0,1,2   step 8: 3,4   step 9: 5,6,7   step 10: 8,9   step 11: 10.
//   Run control: HALT state (SWI or "B ." with the condition true), FAULT state (illegal encoding, BX to an odd address).
//   Either one stops everything (the CPU keeps showing its last state). rst returns to FETCH at PC 0 and clears r0..r14
//   (rst must be held >= 16 cycles, regfile_arm_mc.v).
//
//   Board view (DBG_EN = 1): 'freeze' stops the CPU at the next instruction boundary (FETCH); when the CPU is stopped
//   (frozen, halted or faulted) 'view_rdy' = 1 and the view borrows the register-file port 1 and the memory port:
//   dbg_idx 0..14 = r0..r14, 15 = PC, 16.. = RAM word (idx - 16). SYNCHRONOUS read: dbg_val belongs to the dbg_idx of
//   the PREVIOUS cycle (one cycle of latency, the snapshot unit allows for it).
//====================================================================
module cpu_core_arm_mc #(
    parameter [10:0] FEATURES  = 11'h7FF,
    parameter        IMEM_AW   = 7,
    parameter        DMEM_AW   = 5,
    parameter        INIT_FILE = "program.hex",
    parameter        DATA_FILE = "data.hex",       // starting values of the 32 RAM words (mem_arm_mc.v)
    parameter [10:0] BUILT     = 11'h7FF,
    parameter        DBG_EN    = 0
) (
    input         clk,
    input         rst,
    output        halted,
    output        fault,
    output        retire,           // 1 in the last cycle of every instruction (condition-failed ones included)
    output [31:0] pc,               // PC register (= address of the current instruction + 4 after FETCH)
    output [31:0] instr,            // instruction being executed (Instr register; the fetched word in DECODE)
    output [3:0]  flags,
    output [4:0]  state,
    // board view
    input         freeze,
    output        view_rdy,
    input  [5:0]  dbg_idx,
    output [31:0] dbg_val,
    // trace (testbench)
    output        tr_regw,  output [3:0] tr_wa,  output [31:0] tr_wd,
    output        tr_memw,  output [31:0] tr_adr, output [31:0] tr_mdata, output [1:0] tr_msize
);
    localparam [10:0] FEAT_RAW = FEATURES & BUILT;
    localparam [10:0] FEAT     = {FEAT_RAW[10:3], FEAT_RAW[2] & FEAT_RAW[1], FEAT_RAW[1:0]};   // SHREG needs SHIFT

    wire [31:0] adr, wdata, q_word, rdata, ins, result, a_out;
    wire        IRWrite, R15W, CondLd, PCWrite, RegWrite, MemWrite, CinSel, ShDis, ShByReg, ShAmtW, Zero2;
    wire        ALUOutW, MulHiW, CregW, ZloUse, CSrc, ImmCarry, MemSigned, SeqStart, SeqStep, stall, illegal;
    wire [2:0]  FlagW, RA2Sel;
    wire [1:0]  AdrSrc, ALUSrcA, ALUSrcB, ResultSrc, ImmSrc, RA1Sel, A3Sel, MemSize;
    wire [3:0]  ALUControl, nzcv_in, a3, seq_pos, seq_reg, seq_regq;
    wire        CondEx, CondExR, res0, seq_last, seq_beat0;

    // ---------------- control ----------------
    controller_arm_mc #(.FEATURES(FEAT)) ctl (
        .clk(clk), .rst(rst), .freeze(DBG_EN ? freeze : 1'b0), .ins(ins), .CondEx(CondEx), .CondExR(CondExR), .res0(res0),
        .seq_last(seq_last), .seq_beat0(seq_beat0), .state(state), .stall(stall), .halted(halted), .fault(fault),
        .retire(retire), .IRWrite(IRWrite), .R15W(R15W), .CondLd(CondLd), .PCWrite(PCWrite), .RegWrite(RegWrite),
        .MemWrite(MemWrite), .FlagW(FlagW), .AdrSrc(AdrSrc), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ResultSrc(ResultSrc),
        .ALUControl(ALUControl), .CinSel(CinSel), .ImmSrc(ImmSrc), .ShDis(ShDis), .ShByReg(ShByReg), .ShAmtW(ShAmtW),
        .RA1Sel(RA1Sel), .RA2Sel(RA2Sel), .Zero2(Zero2), .A3Sel(A3Sel), .ALUOutW(ALUOutW), .MulHiW(MulHiW), .CregW(CregW),
        .ZloUse(ZloUse), .CSrc(CSrc), .ImmCarry(ImmCarry), .MemSize(MemSize), .MemSigned(MemSigned),
        .SeqStart(SeqStart), .SeqStep(SeqStep), .illegal_o(illegal));

    wire run = ~halted & ~fault & ~stall;
    condlogic_arm_mc cl (.clk(clk), .rst(rst), .cond(ins[31:28]), .cond_ld(CondLd & run), .nzcv_in(nzcv_in),
                         .FlagW(FlagW & {3{run}}), .CondEx(CondEx), .CondExR(CondExR), .flags(flags), .flags_next());

    ldmstm_seq_arm_mc #(.EN(FEAT[10])) seq (.clk(clk), .rst(rst), .start(SeqStart & run), .step(SeqStep & run),
        .list(ins[15:0]), .U(ins[23]), .pos(seq_pos), .reg_q(seq_regq), .reg_qq(seq_reg), .last_q(seq_last), .beat0(seq_beat0));

    // ---------------- board view ----------------
    assign view_rdy = DBG_EN & (halted | fault | stall);
    wire       view = view_rdy;
    wire [5:0] dbg_ridx = dbg_idx - 6'd16;
    wire [31:0] dbg_adr = {4'h1, 20'd0, dbg_ridx, 2'b00};
    reg  [5:0] idx_q = 6'd0;
    always @(posedge clk) idx_q <= dbg_idx;
    generate
        if (DBG_EN) begin : g_dbg
            assign dbg_val = (idx_q < 6'd15) ? a_out : (idx_q == 6'd15) ? pc :
                             ({1'b0, idx_q - 6'd16} < (7'd1 << DMEM_AW)) ? q_word : 32'd0;
        end else begin : g_nodbg
            assign dbg_val = 32'd0;
        end
    endgenerate

    // ---------------- datapath ----------------
    wire to_pc = RegWrite & (a3 == 4'd15);                 // a register write to R15 is a PC write
    datapath_arm_mc #(.ROT_EN(FEAT[1]), .SH_EN(FEAT[1]), .SHREG_EN(FEAT[2]), .MUL_EN(FEAT[8] | FEAT[9]), .MULL_EN(FEAT[9]),
                      .LDM_EN(FEAT[10])) dp (
        .clk(clk), .rst(rst), .en(run), .adr(adr), .wdata(wdata), .q_word(q_word), .rdata(rdata), .ins(ins), .pc(pc),
        .IRWrite(IRWrite), .R15W(R15W), .PCWrite(PCWrite | to_pc), .RegWrite(RegWrite), .AdrSrc(AdrSrc), .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB), .ResultSrc(ResultSrc), .ALUControl(ALUControl), .CinSel(CinSel), .ImmSrc(ImmSrc), .ShDis(ShDis),
        .ShByReg(ShByReg), .ShAmtW(ShAmtW), .RA1Sel(RA1Sel), .RA2Sel(RA2Sel), .Zero2(Zero2), .A3Sel(A3Sel),
        .ALUOutW(ALUOutW), .MulHiW(MulHiW), .CregW(CregW), .ZloUse(ZloUse), .CSrc(CSrc), .ImmCarry(ImmCarry),
        .seq_pos(seq_pos), .seq_reg(seq_reg), .flags(flags), .nzcv_in(nzcv_in), .res0(res0), .a3(a3), .result(result),
        .view(view), .dbg_ra(dbg_idx[3:0]), .dbg_adr(dbg_adr), .a_out(a_out));

    // ---------------- memory ----------------
    mem_arm_mc #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW), .INIT_FILE(INIT_FILE), .DATA_FILE(DATA_FILE)) mem (
        .clk(clk), .adr(adr), .we(MemWrite), .size(view ? 2'd0 : MemSize), .sgn(view ? 1'b0 : MemSigned), .wdata(wdata),
        .q_word(q_word), .rdata(rdata));

    assign instr = ins;

    // ---------------- trace ----------------
    assign tr_regw = RegWrite & (a3 != 4'd15); assign tr_wa = a3; assign tr_wd = result;
    assign tr_memw = MemWrite; assign tr_adr = adr; assign tr_mdata = wdata; assign tr_msize = MemSize;
endmodule
