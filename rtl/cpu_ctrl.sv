`default_nettype none
// ============================================================
// cpu_ctrl.sv - CONTROL FSM (masked families + exact opcodes)
// ------------------------------------------------------------
// High-level:
// • This module is the microcoded sequencer. It looks at the live IR byte
//   (`instruction`), a simple micro-cycle counter (`cycle`), and handshakes
//   like `bus_ready`, then emits control strobes that the datapath uses
//   (register file enables, PC motion, etc.).
// • Opcodes are grouped into "families" via their upper bits (e.g., ALU/MOV)
//   or are exact values (e.g., HLT/RET/PUSH/POP). Family predicates decode
//   straight off the live IR so the controller always follows what the
//   datapath latched.
// • Some instructions need multi-cycle sub-flows (e.g., LDI). We mark those
//   with `ldi_in_progress` and temporarily “freeze” the general `cycle`
//   stepping so we can step through dedicated states for the sub-flow.
// • `opcode` is exported for visibility: we forward the *live* IR (`instruction`)
//   to downstream debug; `latched_opcode` is just a local trace of what we saw
//   at STATE_NEXT.
// ============================================================
module cpu_ctrl (
    input  logic       clk,
    input  logic       reset_cycle,     // sync reset of controller/cycle
    input  logic [7:0] instruction,     // IR value (raw byte, not immediate)
    output logic [7:0] state,           // current FSM state (for datapath+debug)
    output logic [3:0] cycle,           // micro-cycle stepper (0..7)
    input  logic       bus_ready,       // memory has put valid data on bus
    output logic       c_eo,            // "drive enable" hint for datapath bus sources
    output logic [7:0] opcode,          // exported view of current opcode (live IR)
    output logic       c_rfi,           // register-file write enable (bus sink)
    output logic       c_rfo,           // register-file read enable  (bus source)
    output logic       pc_inc,          // program counter increment
    output logic       pc_load,         // program counter load (jumps/returns)
    output logic       pc_dec,          // program counter decrement (unused here)
    input  logic       jump_allowed     // condition result from datapath flags
);

  // ─── Opcodes (base encodings) ─────────────────────────────
  // Families vs exact-opcodes. Families are matched using masks/prefix bits;
  // exact ops are compared for equality.
  localparam [7:0]
    OP_NOP   = 8'b00_000_000, // 0x00
    OP_CALL  = 8'b00_000_001, // 0x01
    OP_RET   = 8'b00_000_010, // 0x02
    OP_OUT   = 8'b00_000_011, // 0x03
    OP_IN    = 8'b00_000_100, // 0x04
    OP_HLT   = 8'b00_000_101, // 0x05
    OP_CMP   = 8'b00_000_110, // 0x06 (exact)
    OP_LDI   = 8'b00_010_000, // 0x10..0x17 (family: rr in [2:0])
    OP_JMP   = 8'b00_011_000, // 0x18..0x1F (family: cond in [2:0])
    OP_PUSH  = 8'b00_100_000, // 0x20 (exact)
    OP_POP   = 8'b00_101_000, // 0x28 (exact)
    OP_ALU   = 8'b01_000_000, // 0x40..0x7F (family)
    OP_MOV   = 8'b10_000_000; // 0x80..0xBF (family)

  // ─── States ───────────────────────────────────────────────
  // Each state corresponds to a micro-step in the instruction flow.
  // Some are generic (FETCH_*), others serve specific ops (e.g., stack paths).
  localparam [7:0]
    STATE_NEXT          = 8'h00,
    STATE_FETCH_PC      = 8'h01, // put PC on bus → MAR in datapath
    STATE_FETCH_INST    = 8'h02, // capture IR when bus_ready
    STATE_HALT          = 8'h03, // terminal idle
    STATE_JUMP          = 8'h04, // PC load if condition allows
    STATE_OUT           = 8'h05, // IO write cycle setup
    STATE_ALU_OUT       = 8'h06, // ALU result drives bus (datapath derives)
    STATE_ALU_EXEC      = 8'h07, // ALU computes; flags update
    STATE_DEC_SP        = 8'h1D, // stack pre-decrement (PUSH)
    STATE_MOV_STORE     = 8'h08, // MOV reg→mem data write
    STATE_MOV_FETCH     = 8'h09, // MOV mem→reg address phase
    STATE_MOV_LOAD      = 8'h0A, // MOV mem→reg data phase
    STATE_FETCH_SP      = 8'h0C, // SP → bus
    STATE_PC_STORE      = 8'h0D, // CALL writes return PC
    STATE_TMP_JUMP      = 8'h0E, // helper for multi-step jumps
    STATE_RET           = 8'h0F, // PC <= [SP]
    STATE_INC_SP        = 8'h10, // stack post-increment (POP/RET)
    STATE_SET_ADDR      = 8'h11, // latch address (IO/mem helpers)
    STATE_IN            = 8'h12, // IO read cycle setup
    STATE_REG_STORE     = 8'h13, // PUSH: register value write to RAM
    STATE_SET_REG       = 8'h14, // writeback to register file
    STATE_LOAD_IMM      = 8'h15, // LDI: immediate placed on bus for writeback
    STATE_WAIT_FOR_RAM  = 8'h16, // wait until memory drives bus (read path)
    STATE_ALU_WRITEBACK = 8'h17, // ALU → REG_A architectural write
    STATE_REG_READ      = 8'h18, // source register read before ALU
    STATE_FETCH_IMM     = 8'h19, // fetch immediate operand byte
    STATE_WAIT_IMM      = 8'h1A, // wait for immediate byte from memory
    STATE_MOV_MEM_ADDR  = 8'h1C; // (reserved helper; not used in this flow)

  // ---- Internal regs ---------------------------------------------------------------
  logic [7:0] latched_opcode;   // snapshot of IR at STATE_NEXT (debug visibility)
  logic       ldi_in_progress;  // gate to pause general cycle during LDI sub-flow

  // ---- Cycle Counter (freeze during LDI sub-flow) ---------------------------------
  // `cycle` advances 0→7 repeatedly, driving the "normal" fetch/decode cadence.
  // When an LDI is detected, we set `ldi_in_progress` and stop incrementing
  // so we can walk dedicated LDI states without interference.
  initial cycle = 0;
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)       cycle <= 0;
    else if (!ldi_in_progress)
      cycle <= (cycle == 7) ? 0 : cycle + 1;
  end
  localparam [7:0] STATE_MEM_WRITE = 8'h1B; // write-hold to meet memory timing

  // Latch instruction at NEXT (when IR is stable) unless we’re in LDI sub-flow.
  // This is for tracing/diagnostics; the exported `opcode` comes from live IR.
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      latched_opcode <= 8'h00;
    end else if (state == STATE_NEXT && !ldi_in_progress) begin
      latched_opcode <= instruction;
      $display("[INST LATCH] New instruction: %02h, latched_opcode: %02h",
               instruction, instruction);
    end
  end

  // ---- Predicates (families vs exact) ------------------------------------------------
  // Decode straight off the live IR from datapath so controller and datapath
  // always agree on the current instruction semantics.
  wire [7:0] ir = instruction;

  // Predicates
  wire is_LDI = (ir[7:3] == OP_LDI[7:3]);  // family match (mask upper 5 bits)
  wire is_JMP = (ir[7:3] == OP_JMP[7:3]);  // family match
  wire is_MOV = (ir[7:6] == 2'b10);        // class 10
  wire is_ALU = (ir[7:6] == 2'b01);        // class 01
  wire is_HLT = (ir == OP_HLT);            // exact
  wire is_RET = (ir == OP_RET);            // exact
  wire is_POP = (ir == OP_POP);            // exact
  wire is_PUSH= (ir == OP_PUSH);           // exact
  wire is_CALL= (ir == OP_CALL);           // exact
  wire is_IN  = (ir == OP_IN);             // exact
  wire is_OUT = (ir == OP_OUT);            // exact
  wire is_CMP = (ir == OP_CMP);            // exact (ALU SUB without writeback)
  wire is_NOP = (ir == OP_NOP);            // exact

  // Export the same opcode the datapath (IR) sees:
  assign opcode = ir;

  // ---- FSM --------------------------------------------------------------------------
  // Priority:
  // 1) Reset/HALT handling.
  // 2) Dedicated sub-flows (LDI, MOV, ALU, stack ops) with explicit next states.
  // 3) Otherwise “normal” stepping driven by `cycle`.
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      state           <= STATE_FETCH_PC;   // start by forming address from PC
      ldi_in_progress <= 1'b0;
    end else if (state == STATE_HALT) begin
      state <= STATE_HALT;                 // sticky halt

    // LDI sub-flow (3 steps): FETCH_IMM -> WAIT_IMM -> LOAD_IMM -> back to fetch
    end else if (state == STATE_FETCH_IMM) begin
      state <= STATE_WAIT_IMM;
    end else if (state == STATE_WAIT_IMM) begin
      state <= bus_ready ? STATE_LOAD_IMM : STATE_WAIT_IMM;
    end else if (state == STATE_LOAD_IMM) begin
      state           <= STATE_FETCH_PC;   // datapath performs writeback this cycle
      ldi_in_progress <= 1'b0;
      $display("[LDI FSM] LDI complete, returning to FETCH_PC");

    // Other multi-cycle handoffs (MOV/ALU/stack flows)
    end else if (state == STATE_MOV_FETCH)      state <= STATE_MOV_LOAD;
    else if (state == STATE_MOV_LOAD)       state <= STATE_MOV_STORE;
    else if (state == STATE_MOV_STORE)      state <= STATE_FETCH_PC;
    else if (state == STATE_REG_READ)       state <= STATE_ALU_EXEC;
    else if (state == STATE_ALU_EXEC)       state <= STATE_ALU_WRITEBACK;
    else if (state == STATE_ALU_WRITEBACK)  state <= STATE_FETCH_PC;
    else if (state == STATE_PC_STORE)       state <= STATE_FETCH_PC;

    // --- Stack paths ---
    else if (state == STATE_DEC_SP)         state <= STATE_FETCH_SP;

    else if (state == STATE_FETCH_SP) begin
      if (is_PUSH)                          state <= STATE_REG_STORE;    // write path
      else if (is_POP || is_RET)            state <= STATE_WAIT_FOR_RAM; // read path
    end

    else if (state == STATE_WAIT_FOR_RAM && !bus_ready) state <= STATE_WAIT_FOR_RAM;
    else if ((is_POP || is_RET) && state == STATE_WAIT_FOR_RAM && bus_ready) begin
      if (is_POP)  state <= STATE_SET_REG;  // Rn <= [SP]
      else         state <= STATE_RET;      // PC <= [SP]
    end

    else if (state == STATE_SET_REG) begin
      if (is_POP)                           state <= STATE_INC_SP;       // now SP++
      else                                  state <= STATE_FETCH_PC;
    end

    else if (state == STATE_REG_STORE)      state <= STATE_MEM_WRITE;    // PUSH write hold
    else if (state == STATE_MEM_WRITE)      state <= STATE_FETCH_PC;

    else if (state == STATE_RET)            state <= STATE_INC_SP;       // bump SP after RET
    else if (state == STATE_INC_SP)         state <= STATE_FETCH_PC;

    // Normal cycle stepping (when not in an LDI sub-flow)
    else if (!ldi_in_progress) begin
      case (cycle)
        4'd0: state <= STATE_FETCH_PC;          // drive PC → address
        4'd1: state <= STATE_WAIT_FOR_RAM;      // wait for memory (1)
        4'd2: state <= STATE_WAIT_FOR_RAM;      // wait for memory (2)
        4'd3: state <= STATE_FETCH_INST;        // IR <= bus (when bus_ready)
        4'd4: state <= STATE_NEXT;              // decode/prepare
        4'd5: state <= STATE_NEXT;              // decode/prepare (continued)

        // Issue phase: choose the appropriate flow for this opcode
        4'd6: begin
          unique case (1'b1)
            is_HLT:                 state <= STATE_HALT;
            is_MOV:                 state <= STATE_MOV_FETCH;
            (is_ALU || is_CMP):     state <= STATE_REG_READ;
            (is_RET || is_POP):     state <= STATE_FETCH_SP;   // read first
            is_PUSH:                state <= STATE_DEC_SP;     // pre-decrement
            is_CALL:                state <= STATE_SET_REG;
            is_LDI: begin
              state           <= STATE_FETCH_IMM;
              ldi_in_progress <= 1'b1;
              $display("[FSM] LDI detected, entering FETCH_IMM");
            end
            is_JMP:                 state <= STATE_JUMP;
            (is_IN || is_OUT):      state <= STATE_SET_ADDR;
            default:                state <= STATE_NEXT;       // NOP/unknown safe no-op
          endcase
        end

        // Cleanups/finalization for multi-step ops
        4'd7: begin
          unique case (1'b1)
            is_ALU:                 state <= STATE_ALU_WRITEBACK;
            is_CALL:                state <= STATE_PC_STORE;
            is_JMP:                 state <= STATE_FETCH_PC;
            (is_IN || is_OUT):      state <= STATE_NEXT;
            default:                state <= STATE_NEXT;
          endcase
        end
        default: state <= STATE_NEXT;
      endcase
    end
  end

  // Export RAW opcode (datapath does its own predicates)
  // (Already assigned above: `assign opcode = ir;`)

  // ---- Control signals ---------------------------------------------------------------
  // Purely combinational control decode from current `state` (+ some predicates).
  // Keep defaults to 0 and selectively assert per-state to avoid latches.
  always_comb begin
    c_rfi   = 1'b0;  // default: no reg write
    c_rfo   = 1'b0;  // default: reg file not driving bus
    c_eo    = 1'b0;  // default: no generic "drive enable" hint

    // PC control:
    // • Increment when we successfully accept an instruction byte or immediate.
    // • Load PC during jumps/returns (datapath provides `jump_allowed`).
    pc_inc  = ((state == STATE_FETCH_INST) && bus_ready) ||
              ((state == STATE_WAIT_IMM)   && bus_ready);

    pc_load = (state == STATE_JUMP) && jump_allowed
           || (state == STATE_RET)
           || (state == STATE_TMP_JUMP);

    pc_dec = 1'b0; // not used by this controller (SP dec handled in datapath)

    // Per-state enables:
    unique case (state)
      // ALU writeback drives result and writes REG_A (datapath maps REG_A)
      STATE_ALU_WRITEBACK: begin
        c_eo  = 1'b1;
        c_rfi = 1'b1;
      end

      // LDI: after WAIT_IMM, LOAD_IMM performs the actual register write
      STATE_LOAD_IMM:      if (is_LDI) c_rfi = 1'b1;

      // IN path: captured input byte is written into REG_A (datapath maps)
      STATE_IN:            c_rfi = 1'b1;

      // Addressing helpers may deposit a value into a register (e.g., CALL temp)
      STATE_SET_ADDR:      c_rfi = 1'b1;

      // Generic register writeback state (MOV/POP/etc.)
      STATE_SET_REG:       c_rfi = 1'b1;

      // MOV reg↔mem:
      // • In STORE: source register must drive bus (c_rfo), and if destination
      //   is a register (dst != T/mem), allow register write (c_rfi).
      STATE_MOV_STORE: begin
        if (instruction[2:0] != 3'b111) c_rfi = 1'b1; // reg dst
        if (instruction[5:3] != 3'b111) c_rfo = 1'b1; // reg src
      end

      // PUSH data phase: keep source register driving across write & hold
      STATE_REG_STORE:     c_rfo = 1'b1;
      STATE_MEM_WRITE:     c_rfo = 1'b1;

      default: begin
        c_rfi = 1'b0;
        c_rfo = 1'b0;
      end
    endcase
  end

endmodule
