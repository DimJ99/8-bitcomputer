module cpu (
  input  logic       clk,
  input  logic       reset,
  output logic [7:0] addr_bus,
  output logic       c_ri,
  output logic       c_ro,
  output logic       mem_clk,
  output logic       mem_io,
  inout  tri   [7:0] bus
);


  // ─────────────────────────────────────────────────────────────
  //  Opcode, state, ALU‑mode, jump‑condition, and register codes
  // ─────────────────────────────────────────────────────────────
  localparam [7:0]
    OP_NOP  = 8'b00_000_000,
    OP_CALL = 8'b00_000_001,
    OP_RET  = 8'b00_000_010,
    OP_OUT  = 8'b00_000_011,
    OP_IN   = 8'b00_000_100,
    OP_HLT  = 8'b00_000_101,
    OP_CMP  = 8'b00_000_110,
    OP_LDI  = 8'b00_010_000,
    OP_JMP  = 8'b00_011_000,
    OP_PUSH = 8'b00_100_000,
    OP_POP  = 8'b00_101_000,
    OP_ALU  = 8'b01_000_000,
    OP_MOV  = 8'b10_000_000;

  localparam [7:0]
    STATE_NEXT       = 8'h00,
    STATE_FETCH_PC   = 8'h01,
    STATE_FETCH_INST = 8'h02,
    STATE_HALT       = 8'h03,
    STATE_JUMP       = 8'h04,
    STATE_OUT        = 8'h05,
    STATE_ALU_OUT    = 8'h06,
    STATE_ALU_EXEC   = 8'h07,
    STATE_MOV_STORE  = 8'h08,
    STATE_MOV_FETCH  = 8'h09,
    STATE_MOV_LOAD   = 8'h0A,
    STATE_FETCH_SP   = 8'h0C,
    STATE_PC_STORE   = 8'h0D,
    STATE_TMP_JUMP   = 8'h0E,
    STATE_RET        = 8'h0F,
    STATE_INC_SP     = 8'h10,
    STATE_SET_ADDR   = 8'h11,
    STATE_IN         = 8'h12,
    STATE_REG_STORE  = 8'h13,
    STATE_SET_REG    = 8'h14;

  localparam [2:0]
    ALU_ADD = 3'b000,
    ALU_SUB = 3'b001,
    ALU_INC = 3'b010,
    ALU_DEC = 3'b011,
    ALU_AND = 3'b100,
    ALU_OR  = 3'b101,
    ALU_XOR = 3'b110,
    ALU_ADC = 3'b111;

  localparam [2:0]
    JMP_JMP = 3'b000,
    JMP_JZ  = 3'b001,
    JMP_JNZ = 3'b010,
    JMP_JC  = 3'b011,
    JMP_JNC = 3'b100;

  localparam [2:0] REG_A = 3'b000,
                   REG_T = 3'b111;

  // ─────────────────────────────────────────────────────────────
  //  Internal shared‑bus framework (one driver per source)
  // ─────────────────────────────────────────────────────────────
  logic [7:0] bus_from_pc,  bus_from_sp,  bus_from_alu, bus_from_reg;
  logic       bus_drive_pc, bus_drive_sp, bus_drive_alu, bus_drive_reg;

  // ─────────────────────────────────────────────────────────────
  //  Status flags, clocks and cycle generator
  // ─────────────────────────────────────────────────────────────
  logic flag_zero, flag_carry;

  logic cycle_clk    = 0;
  logic internal_clk = 0;
  logic [2:0] cnt    = 3'b100;
  logic halted       = 0;
  logic c_halt;

  always_ff @(posedge clk) begin
    if (!halted) begin
      {cycle_clk, mem_clk, internal_clk} <= cnt;
      case (cnt)
        3'b100: cnt <= 3'b010;
        3'b010: cnt <= 3'b001;
        3'b001: cnt <= 3'b100;
      endcase
    end
  end

  // ─────────────────────────────────────────────────────────────
  //  General‑purpose registers
  // ─────────────────────────────────────────────────────────────
  logic [2:0] sel_in, sel_out;
  logic [7:0] rega_out, regb_out, regs_out;
  logic       c_rfi, c_rfo;

  cpu_reg m_registers (
    .clk           (internal_clk),
           .reset          (reset),
    .data_in       (bus),// read from bus
    .sel_in        (sel_in),
    .sel_out       (sel_out),
    .enable_write  (c_rfi),
    .output_enable (c_rfo),
    .data_out      (regs_out),     // **internal**, not directly on bus
    .rega          (rega_out),
    .regb          (regb_out)
  );

  // ─────────────────────────────────────────────────────────────
  //  Instruction register
  // ─────────────────────────────────────────────────────────────
  logic [7:0] regi_out;
  logic       c_ii;

  register m_regi (
    .in    (bus),
    .clk   (internal_clk),
    .enable(c_ii),
    .reset (reset),
    .out   (regi_out)
  );

  // ─────────────────────────────────────────────────────────────
  //  Memory‑address register
  // ─────────────────────────────────────────────────────────────
  logic c_mi;
  register m_mar (
    .in    (bus),
    .clk   (internal_clk),
    .enable(c_mi),
    .reset (reset),
    .out   (addr_bus)
  );

  // ─────────────────────────────────────────────────────────────
  //  Program counter
  // ─────────────────────────────────────────────────────────────
  logic [7:0] pc_out;
  logic       c_co, c_ci, c_j;

  counter m_pc (
    .clk    (c_ci & internal_clk),
    .in     (bus),
    .sel_in (c_j),
    .reset  (reset),
    .down   (1'b0),
    .out    (pc_out)
  );

  // ─────────────────────────────────────────────────────────────
  //  Stack pointer
  // ─────────────────────────────────────────────────────────────
  logic [7:0] sp_out;
  logic       c_si, c_sd, c_so;

  counter m_sp (
    .clk    (reset | (c_si & internal_clk)),
    .in     (8'hFF),
    .sel_in (reset),
    .reset  (1'b0),
    .down   (c_sd),
    .out    (sp_out)
  );

  // ─────────────────────────────────────────────────────────────
  //  ALU (combinational + flags)
  // ─────────────────────────────────────────────────────────────
  logic       c_eo, c_ee;
  logic [7:0] alu_out;
  logic [2:0] alu_op;

  alu m_alu (
    .clk        (internal_clk),
    .enable     (c_ee),
    .reset      (reset),
    .in_a       (rega_out),
    .in_b       (regb_out),
    .out        (alu_out),
    .op         (alu_op),
    .flag_zero  (flag_zero),
    .flag_carry (flag_carry)
  );
