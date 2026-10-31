//====================================================================
// controller_arm_mc.v   (ARMv4 multi-cycle CPU, step 5)  -  main FSM of the multi-cycle core (spec section 6)
//
//   Harris & Harris multi-cycle control unit (section 7.4.2): main FSM (states FETCH, DECODE, MEMADR, MEMRD, MEMWB,
//   MEMWR, EXECR, EXECI, ALUWB, BRANCH) plus the states of the extensions (RSREAD, BLLR, BXEX, MUL1, MLAADD, MULWB,
//   MULLLO, MULLHI, MULLWB, LSMBEAT, LDMLAST, LDMWB) and the stop states HALT / FAULT. The ALU decoder is folded in
//   (ALUControl = data-processing opcode in EXECR/EXECI, ADD/SUB/MOV/ADC elsewhere).
//   Legality comes from the unchanged single-cycle decoder (decoder_arm.v, FEATURES), so both cores run exactly the
//   same instruction subset; the instruction class is recognised here from the same fields.
//
//   'ins' = memory output in DECODE, Instr register afterwards (datapath). Every write is gated with CondExR here
//   (RegWrite, MemWrite, the PC write of B/BX and of a register write to R15, FlagW in condlogic). The state sequence
//   never depends on the condition (H&H rule): a condition-failed instruction takes its normal number of cycles.
//
//   Freeze (board snapshot): 'freeze' is accepted only in FETCH (an instruction boundary: no M9K output value is in use);
//   the FSM then waits in FETCH doing nothing ('stall'). HALT / FAULT do nothing anyway.
//====================================================================
module controller_arm_mc #(
    parameter [10:0] FEATURES = 11'h7FF
) (
    input             clk,
    input             rst,
    input             freeze,
    input      [31:0] ins,
    input             CondEx,
    input             CondExR,
    input             res0,          // ALU result bit 0 (BX target)
    input             seq_last,      // LDM/STM: current beat is the last
    input             seq_beat0,     // LDM/STM: current beat is the first
    output reg [4:0]  state = 5'd0,
    output            stall,         // frozen in FETCH
    output            halted,
    output            fault,
    output            retire,
    // datapath control
    output reg        IRWrite, R15W, CondLd,
    output            PCWrite,
    output            RegWrite,
    output            MemWrite,
    output reg [2:0]  FlagW,
    output reg [1:0]  AdrSrc, ALUSrcA, ALUSrcB, ResultSrc,
    output reg [3:0]  ALUControl,
    output reg        CinSel,
    output reg [1:0]  ImmSrc,
    output reg        ShDis, ShByReg, ShAmtW,
    output reg [1:0]  RA1Sel,
    output reg [2:0]  RA2Sel,
    output reg        Zero2,
    output reg [1:0]  A3Sel,
    output reg        ALUOutW, MulHiW, CregW, ZloUse, CSrc, ImmCarry,
    output reg [1:0]  MemSize,
    output reg        MemSigned,
    output reg        SeqStart, SeqStep,
    output            illegal_o
);
    localparam [4:0] FETCH = 5'd0, DECODE = 5'd1, MEMADR = 5'd2, MEMRD = 5'd3, MEMWB = 5'd4, MEMWR = 5'd5,
                     EXECR = 5'd6, EXECI = 5'd7, ALUWB = 5'd8, BRANCH = 5'd9, RSREAD = 5'd10, BLLR = 5'd11,
                     BXEX = 5'd12, MUL1 = 5'd13, MLAADD = 5'd14, MULWB = 5'd15, MULLLO = 5'd16, MULLHI = 5'd17,
                     MULLWB = 5'd18, LSMBEAT = 5'd19, LDMLAST = 5'd20, LDMWB = 5'd21, HALT = 5'd22, FAULT = 5'd23;
    localparam [3:0] ADD = 4'b0100, SUB = 4'b0010, MOV = 4'b1101, ADC = 4'b0101;

    // ---------------- legality (unchanged single-cycle decoder; only 'illegal' is used) ----------------
    wire illegal;
    decoder_arm #(.FEATURES(FEATURES)) dec (.instr(ins), .illegal(illegal),
        .RA1sel(), .RA3sel(), .WA1sel(), .BSrc(), .ImmSrc(), .ShAmtSel(), .ShDis(), .ALUControl(), .MemAddrSel(), .MemR(),
        .MemW(), .MemSize(), .MemSigned(), .ResSrc(), .Res2Src(), .RegW1(), .RegW2(), .FlagW(), .CSrc(), .PCS(), .PCTSel(),
        .Mul(), .MulLong(), .MulSigned(), .MulAcc(), .Halt(), .Seq(), .BX());
    assign illegal_o = illegal;

    // ---------------- instruction class (same field tests as decoder_arm.v) ----------------
    wire [2:0] op = ins[27:25];
    wire P = ins[24], U = ins[23], Bb = ins[22], W = ins[21], L = ins[20];
    wire is_mulspace = (op == 3'b000) & (ins[7:4] == 4'b1001);
    wire is_mul  = is_mulspace & (ins[27:22] == 6'b000000);
    wire is_mull = is_mulspace & (ins[27:23] == 5'b00001);
    wire is_half = (op == 3'b000) & ins[7] & ins[4] & ~is_mulspace;
    wire is_bx   = (op == 3'b000) & (ins[27:4] == 24'h12FFF1);
    wire is_dpi  = (op == 3'b001);
    wire is_dpr  = (op == 3'b000) & ~is_mulspace & ~is_half & ~is_bx;
    wire is_mem  = (op == 3'b010) | (op == 3'b011);
    wire is_ldm  = (op == 3'b100);
    wire is_br   = (op == 3'b101);
    wire is_swi  = (op == 3'b111);
    wire regshift = is_dpr & ins[4];
    wire [3:0] opc = ins[24:21];
    wire compare = (opc[3:2] == 2'b10);                    // TST TEQ CMP CMN
    wire arith   = (opc == 4'd2) | (opc == 4'd3) | (opc == 4'd4) | (opc == 4'd5) | (opc == 4'd6) | (opc == 4'd7) |
                   (opc == 4'd10) | (opc == 4'd11);
    wire wb      = ~P | W;                                 // load/store write-back
    wire bself   = (ins[23:0] == 24'hFFFFFE);              // "B ."

    assign stall = freeze & (state == FETCH);
    wire run = (state != HALT) & (state != FAULT) & ~stall;

    reg [4:0] nxt;
    reg       regw, memw, pcw_br, pcw_next, go_halt, go_fault;

    always @(*) begin
        // defaults
        nxt = state;
        IRWrite = 1'b0; R15W = 1'b0; CondLd = 1'b0; pcw_next = 1'b0; pcw_br = 1'b0; regw = 1'b0; memw = 1'b0;
        FlagW = 3'b000; AdrSrc = 2'd0; ALUSrcA = 2'd0; ALUSrcB = 2'd0; ResultSrc = 2'd0; ALUControl = ADD; CinSel = 1'b0;
        ImmSrc = is_dpi ? 2'd0 : is_br ? 2'd2 : is_half ? 2'd3 : 2'd1;
        ShDis = 1'b0; ShByReg = 1'b0; ShAmtW = 1'b0;
        RA1Sel = is_br ? 2'd1 : (is_mul | is_mull) ? 2'd2 : 2'd0;
        RA2Sel = 3'd0; Zero2 = 1'b0; A3Sel = 2'd0;
        ALUOutW = 1'b1; MulHiW = 1'b0; CregW = 1'b0; ZloUse = 1'b0; CSrc = 1'b0; ImmCarry = 1'b0;
        MemSize = 2'd0; MemSigned = 1'b0;
        if (is_mem) MemSize = Bb ? 2'd2 : 2'd0;
        if (is_half) begin MemSize = (ins[6:5] == 2'b10) ? 2'd2 : 2'd1; MemSigned = ins[6]; end
        SeqStart = 1'b0; SeqStep = 1'b0; go_halt = 1'b0; go_fault = 1'b0;

        case (state)
            FETCH: begin
                AdrSrc = 2'd0; ALUSrcA = 2'd1; ALUSrcB = 2'd2; ALUControl = ADD; ResultSrc = 2'd2; pcw_next = 1'b1;
                nxt = DECODE;
            end
            DECODE: begin
                IRWrite = 1'b1; CondLd = 1'b1; R15W = 1'b1;
                ALUSrcA = 2'd1; ALUSrcB = 2'd2; ALUControl = ADD; ResultSrc = 2'd2;          // Result = PC + 4 = PC_instr + 8
                if (is_dpr)              RA2Sel = regshift ? 3'd1 : 3'd0;
                else if (is_mul|is_mull) RA2Sel = 3'd1;
                else if (is_ldm)         RA2Sel = 3'd4;
                SeqStart = is_ldm & ~illegal;
                if (illegal)                   begin nxt = FAULT; go_fault = 1'b1; end
                else if (is_swi)               begin nxt = CondEx ? HALT : FETCH; go_halt = CondEx; end
                else if (is_dpi)               nxt = EXECI;
                else if (is_dpr)               nxt = regshift ? RSREAD : EXECR;
                else if (is_mem | is_half)     nxt = MEMADR;
                else if (is_br)                nxt = BRANCH;
                else if (is_bx)                nxt = BXEX;
                else if (is_mul | is_mull)     nxt = MUL1;
                else                           nxt = LSMBEAT;                               // is_ldm
            end
            RSREAD: begin                                                                    // [FUR] Rs -> ShAmt, read Rm
                ShAmtW = 1'b1; RA2Sel = 3'd0;
                nxt = EXECR;
            end
            EXECR: begin
                ALUSrcA = 2'd0; ALUSrcB = 2'd0; ShByReg = regshift; ALUControl = opc;
                FlagW = L ? {2'b11, arith} : 3'b000; CSrc = ~arith;
                nxt = ALUWB;
            end
            EXECI: begin
                ALUSrcA = 2'd0; ALUSrcB = 2'd1; ALUControl = opc;
                FlagW = L ? {2'b11, arith} : 3'b000; CSrc = ~arith; ImmCarry = 1'b1;
                nxt = ALUWB;
            end
            ALUWB: begin
                ResultSrc = 2'd0; regw = ~compare; A3Sel = 2'd0;
                nxt = FETCH;
            end
            MEMADR: begin
                ALUSrcA = 2'd0; ALUControl = U ? ADD : SUB;
                ALUSrcB = is_mem ? (ins[25] ? 2'd0 : 2'd1) : (Bb ? 2'd1 : 2'd0);
                ShDis = is_half;                                                             // halfword register offset: no shift
                RA2Sel = 3'd2;                                                               // Rd: store data for MEMWR
                nxt = L ? MEMRD : MEMWR;
            end
            MEMRD: begin
                AdrSrc = P ? 2'd1 : 2'd2;
                ResultSrc = 2'd0; regw = wb; A3Sel = 2'd1;                                   // write-back Rn <- ALUOut
                nxt = MEMWB;
            end
            MEMWB: begin
                ResultSrc = 2'd1; regw = 1'b1; A3Sel = 2'd0;
                nxt = FETCH;
            end
            MEMWR: begin
                AdrSrc = P ? 2'd1 : 2'd2; memw = 1'b1;
                ResultSrc = 2'd0; regw = wb; A3Sel = 2'd1;
                nxt = FETCH;
            end
            BRANCH: begin
                ALUSrcA = 2'd0; ALUSrcB = 2'd1; ALUControl = ADD; ResultSrc = 2'd2; pcw_br = 1'b1;
                RA1Sel = 2'd1;                                                               // keep A = R15 for BLLR
                if (~P & bself & CondExR) begin nxt = HALT; go_halt = 1'b1; end
                else nxt = P ? BLLR : FETCH;
            end
            BLLR: begin                                                                      // LR = (PC + 8) - 4
                ALUSrcA = 2'd0; ALUSrcB = 2'd2; ALUControl = SUB; ResultSrc = 2'd2; regw = 1'b1; A3Sel = 2'd2;
                nxt = FETCH;
            end
            BXEX: begin
                ALUSrcB = 2'd0; ShDis = 1'b1; ALUControl = MOV; ResultSrc = 2'd2;
                if (CondExR & res0) begin nxt = FAULT; go_fault = 1'b1; end
                else begin pcw_br = 1'b1; nxt = FETCH; end
            end
            MUL1: begin
                ALUSrcB = 2'd3; ALUControl = MOV; MulHiW = 1'b1;
                if (is_mul & ~W) FlagW = L ? 3'b100 : 3'b000;
                RA2Sel = 3'd2; Zero2 = is_mull & ~W;                                         // Ra (MLA) / RdLo (or 0)
                nxt = is_mull ? MULLLO : (W ? MLAADD : MULWB);
            end
            MLAADD: begin
                ALUSrcA = 2'd2; ALUSrcB = 2'd0; ShDis = 1'b1; ALUControl = ADD; FlagW = L ? 3'b100 : 3'b000;
                nxt = MULWB;
            end
            MULWB: begin
                ResultSrc = 2'd0; regw = 1'b1; A3Sel = 2'd1;
                nxt = FETCH;
            end
            MULLLO: begin
                ALUSrcA = 2'd2; ALUSrcB = 2'd0; ShDis = 1'b1; ALUControl = ADD; CregW = 1'b1;
                RA2Sel = 3'd3; Zero2 = ~W;                                                   // RdHi (or 0)
                nxt = MULLHI;
            end
            MULLHI: begin
                ResultSrc = 2'd0; regw = 1'b1; A3Sel = 2'd0;                                 // RdLo <- lo
                ALUSrcA = 2'd3; ALUSrcB = 2'd0; ShDis = 1'b1; ALUControl = ADC; CinSel = 1'b1;
                FlagW = L ? 3'b100 : 3'b000; ZloUse = 1'b1;
                nxt = MULLWB;
            end
            MULLWB: begin
                ResultSrc = 2'd0; regw = 1'b1; A3Sel = 2'd1;                                 // RdHi <- hi
                nxt = FETCH;
            end
            LSMBEAT: begin
                SeqStep = 1'b1; RA2Sel = 3'd4;
                ALUSrcA = seq_beat0 ? 2'd0 : 2'd2; ALUSrcB = 2'd2; ALUControl = U ? ADD : SUB;
                AdrSrc  = P ? 2'd3 : (seq_beat0 ? 2'd2 : 2'd1);
                if (L) begin
                    ResultSrc = 2'd1; regw = ~seq_beat0; A3Sel = 2'd3;                       // data of the previous beat
                end else begin
                    memw = 1'b1;
                    ResultSrc = 2'd2; regw = W & seq_last; A3Sel = 2'd1;                     // STM write-back on the last beat
                end
                nxt = seq_last ? (L ? LDMLAST : FETCH) : LSMBEAT;
            end
            LDMLAST: begin
                ResultSrc = 2'd1; regw = 1'b1; A3Sel = 2'd3; ALUOutW = 1'b0;
                nxt = W ? LDMWB : FETCH;
            end
            LDMWB: begin
                ResultSrc = 2'd0; regw = 1'b1; A3Sel = 2'd1;
                nxt = FETCH;
            end
            HALT:    nxt = HALT;
            FAULT:   nxt = FAULT;
            default: nxt = FAULT;
        endcase
    end

    // ---------------- write gating ----------------
    // (a register write to address 15 is a PC write, [HH]: R15 is the PC. The core adds it to PCWrite.)
    wire       wr_ok    = run & CondExR;
    assign RegWrite = regw & wr_ok;
    assign MemWrite = memw & wr_ok;
    assign PCWrite  = run & (pcw_next | (pcw_br & CondExR));     // + RegWrite to R15 (core)

    always @(posedge clk) begin
        if (rst) state <= FETCH;
        else if (run) state <= nxt;
    end
    assign halted = (state == HALT);
    assign fault  = (state == FAULT);
    assign retire = run & (state != FETCH) & ((nxt == FETCH) | (nxt == HALT));
endmodule
