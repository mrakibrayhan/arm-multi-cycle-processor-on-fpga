//====================================================================
// mem_arm_mc.sv  (ARMv4 multi-cycle CPU, step 2; SystemVerilog file because of the byte-enable RAM)  -  unified memory port with SYNCHRONOUS-read M9K blocks
//
//   Harris & Harris multi-cycle processor (section 7.4): ONE memory for instructions and data, one address 'adr'
//   (AdrSrc mux in the datapath). Here the memory is two M9K-based regions behind that single port:
//       adr[31:28] = 0 : ROM, 2**IMEM_AW words (program + constants, $readmemh(INIT_FILE)), read only
//       adr[31:28] = 1 : RAM, 2**DMEM_AW words, byte-enable writes
//       anything else  : reads 0, writes ignored          (same map as the single-cycle core, spec section 3)
//
//   SYNCHRONOUS READ: 'adr' is captured by the M9K address registers at the rising edge; the word appears on q_word in
//   the NEXT cycle and stays until the next edge. This is exactly the Harris & Harris 'Data' register
//   (Data <= Mem[Adr] every cycle), so no extra flip-flops are needed:
//       FETCH  : adr = PC       -> DECODE sees the instruction on q_word
//       MEMRD  : adr = ALUOut/A -> MEMWB sees the load data on rdata (after the aligner)
//   q_word = raw word (instruction fetch).  rdata = aligned load data: byte / halfword select by the registered adr[1:0],
//   zero or sign extension by size / sgn (these come from the held Instr register, so they are valid in MEMWB).
//
//   STORE ('we' = MemWrite already gated by CondEx): data written at the rising edge that ends MEMWR.
//       size 0 word, 1 halfword, 2 byte. The store data is copied onto every lane (STRB: 4 x byte, STRH: 2 x half) and the
//       byte enables select the addressed lane(s), so no store shifter and no read-modify-write is needed.
//       The RAM is one 32-bit M9K with byte enables, written with the SystemVerilog byte-enable RAM template of the Quartus
//       coding-style guide (packed [3:0][7:0] words), so that $readmemh(data.hex) initialises it directly.
//   RAM STARTING VALUES: DATA_FILE (data.hex, $readmemh: one 8-digit hex word per line, line 1 = RAM word 0 at 0x10000000,
//   2**DMEM_AW lines) is loaded at configuration time, like INIT_FILE for the ROM. The reset (KEY3) does not reload it:
//   program the FPGA again to restore the data.hex values.
//   Word accesses ignore adr[1:0], halfword accesses ignore adr[0] (unaligned accesses are outside the subset).
//   A read in the same cycle as a write to the same word is never used by the CPU (no_rw_check: no bypass logic).
//====================================================================
module mem_arm_mc #(
    parameter IMEM_AW   = 7,                  // 128 words of ROM (board build)
    parameter DMEM_AW   = 5,                  // 32 words of RAM (board build)
    parameter INIT_FILE = "program.hex",
    parameter DATA_FILE = "data.hex"              // starting values of the RAM (all zeros = the old behaviour)
) (
    input         clk,
    input  [31:0] adr,                        // byte address (AdrSrc mux)
    input         we,                         // MemWrite & CondEx
    input  [1:0]  size,                       // 0 word, 1 half, 2 byte   (store and load)
    input         sgn,                        // sign-extend a load (LDRSB, LDRSH)
    input  [31:0] wdata,                      // store data (WriteData = Rd)
    output [31:0] q_word,                     // raw word read with the address of the PREVIOUS cycle (instruction)
    output [31:0] rdata                       // aligned / extended load data (Data)
);
    wire sel_rom = (adr[31:28] == 4'h0);
    wire sel_ram = (adr[31:28] == 4'h1);
    wire [IMEM_AW-1:0] ridx = adr[IMEM_AW+1:2];
    wire [DMEM_AW-1:0] widx = adr[DMEM_AW+1:2];

    // ---------------- ROM (one M9K) ----------------
    (* romstyle = "M9K" *) reg [31:0] rom [0:(1 << IMEM_AW) - 1];
    initial $readmemh(INIT_FILE, rom);
    reg [31:0] rom_q;
    always @(posedge clk) rom_q <= rom[ridx];

    // ---------------- store lanes and byte enables ----------------
    reg [31:0] sdata;
    reg [3:0]  be;
    always @(*) begin
        case (size)
            2'd2:    begin sdata = {4{wdata[7:0]}};  be = 4'b0001 << adr[1:0];        end   // byte
            2'd1:    begin sdata = {2{wdata[15:0]}}; be = adr[1] ? 4'b1100 : 4'b0011; end   // halfword
            default: begin sdata = wdata;            be = 4'b1111;                      end   // word
        endcase
    end
    wire wram = we & sel_ram;

    // ---------------- RAM: one 32-bit M9K with byte enables (Intel's SystemVerilog byte-enable RAM template) ----------------
    // Starting values: $readmemh(DATA_FILE) straight into the RAM array (data.hex, line 1 = word 0), like the ROM.
    (* ramstyle = "M9K, no_rw_check" *) logic [3:0][7:0] ram [0:(1 << DMEM_AW) - 1];
    initial $readmemh(DATA_FILE, ram);
    logic [31:0] ram_q;
    always @(posedge clk) begin
        if (wram) begin
            if (be[0]) ram[widx][0] <= sdata[7:0];
            if (be[1]) ram[widx][1] <= sdata[15:8];
            if (be[2]) ram[widx][2] <= sdata[23:16];
            if (be[3]) ram[widx][3] <= sdata[31:24];
        end
        ram_q <= ram[widx];
    end

    // ---------------- registered region and byte offset (they belong to the word on the outputs) ----------------
    reg rom_r = 1'b1, ram_r = 1'b0;
    reg [1:0] a10_r = 2'd0;
    always @(posedge clk) begin
        rom_r <= sel_rom;
        ram_r <= sel_ram;
        a10_r <= adr[1:0];
    end
    assign q_word = ram_r ? ram_q : rom_r ? rom_q : 32'd0;

    // ---------------- load aligner (Data) ----------------
    reg [7:0] b;
    always @(*) begin
        case (a10_r)
            2'd0:    b = q_word[7:0];
            2'd1:    b = q_word[15:8];
            2'd2:    b = q_word[23:16];
            default: b = q_word[31:24];
        endcase
    end
    wire [15:0] h = a10_r[1] ? q_word[31:16] : q_word[15:0];
    assign rdata = (size == 2'd2) ? {{24{sgn & b[7]}}, b} :
                   (size == 2'd1) ? {{16{sgn & h[15]}}, h} : q_word;
endmodule
