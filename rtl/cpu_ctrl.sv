`default_nettype none
// ============================================================
// cpu_ctrl.sv - CONTROL FSM (masked families + exact opcodes)
// ============================================================
module cpu_ctrl (
    input  logic       clk,
    input  logic       reset_cycle,
    input  logic [7:0] instruction,  // IR value (raw byte, not immediate)
    output logic [7:0] state,
    output logic [3:0] cycle,
    input  logic       bus_ready,
    output logic       c_eo,
    output logic [7:0] opcode,
    output logic       c_rfi,
    output logic       c_rfo,
    output logic       pc_inc,
    output logic       pc_load,
    output logic       pc_dec,
    input  logic       jump_allowed
);

  // ─── Opcodes (base encodings) ─────────────────────────────
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
  localparam [7:0]
    STATE_NEXT          = 8'h00,
    STATE_FETCH_PC      = 8'h01,
    STATE_FETCH_INST    = 8'h02,
    STATE_HALT          = 8'h03,
    STATE_JUMP          = 8'h04,
    STATE_OUT           = 8'h05,
    STATE_ALU_OUT       = 8'h06,
    STATE_ALU_EXEC      = 8'h07,
    STATE_DEC_SP       = 8'h1D,
    STATE_MOV_STORE     = 8'h08,
    STATE_MOV_FETCH     = 8'h09,
    STATE_MOV_LOAD      = 8'h0A,
    STATE_FETCH_SP      = 8'h0C,
    STATE_PC_STORE      = 8'h0D,
    STATE_TMP_JUMP      = 8'h0E,
    STATE_RET           = 8'h0F,
    STATE_INC_SP        = 8'h10,
    STATE_SET_ADDR      = 8'h11,
    STATE_IN            = 8'h12,
    STATE_REG_STORE     = 8'h13,
    STATE_SET_REG       = 8'h14,
    STATE_LOAD_IMM      = 8'h15,
    STATE_WAIT_FOR_RAM  = 8'h16,
    STATE_ALU_WRITEBACK = 8'h17,
    STATE_REG_READ      = 8'h18,
    STATE_FETCH_IMM     = 8'h19,
    STATE_WAIT_IMM      = 8'h1A,
    STATE_MOV_MEM_ADDR  = 8'h1C;

  // ---- Internal regs ---------------------------------------------------------------
  logic [7:0] latched_opcode;
  logic       ldi_in_progress;

  // ---- Cycle Counter (freeze during LDI sub-flow) ---------------------------------
  initial cycle = 0;
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)       cycle <= 0;
    else if (!ldi_in_progress)
      cycle <= (cycle == 7) ? 0 : cycle + 1;
  end
  localparam [7:0] STATE_MEM_WRITE = 8'h1B;

  
  // Latch instruction at NEXT (when IR stable) if not in LDI sub-flow
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
// Use the live IR coming from datapath:
wire [7:0] ir = instruction;

// Predicates
wire is_LDI = (ir[7:3] == OP_LDI[7:3]);
wire is_JMP = (ir[7:3] == OP_JMP[7:3]);
wire is_MOV = (ir[7:6] == 2'b10);
wire is_ALU = (ir[7:6] == 2'b01);
wire is_HLT = (ir == OP_HLT);
wire is_RET = (ir == OP_RET);
wire is_POP = (ir == OP_POP);
wire is_PUSH= (ir == OP_PUSH);
wire is_CALL= (ir == OP_CALL);
wire is_IN  = (ir == OP_IN);
wire is_OUT = (ir == OP_OUT);
wire is_CMP = (ir == OP_CMP);
wire is_NOP = (ir == OP_NOP);

// Export the same opcode the datapath (IR) sees:
assign opcode = ir;

  // ---- FSM ---------------------------------------------------------------
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      state           <= STATE_FETCH_PC;
      ldi_in_progress <= 1'b0;
    end else if (state == STATE_HALT) begin
      state <= STATE_HALT;

    // LDI sub-flow
    end else if (state == STATE_FETCH_IMM) begin
      state <= STATE_WAIT_IMM;
    end else if (state == STATE_WAIT_IMM) begin
      state <= bus_ready ? STATE_LOAD_IMM : STATE_WAIT_IMM;
    end else if (state == STATE_LOAD_IMM) begin
      state           <= STATE_FETCH_PC; // writeback happens in datapath this cycle
      ldi_in_progress <= 1'b0;
      $display("[LDI FSM] LDI complete, returning to FETCH_PC");

    // Other multi-cycle handoffs
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
  if (is_PUSH)                          state <= STATE_REG_STORE;   // write path
  else if (is_POP || is_RET)            state <= STATE_WAIT_FOR_RAM;// read path
end

else if (state == STATE_WAIT_FOR_RAM && !bus_ready) state <= STATE_WAIT_FOR_RAM;
else if ((is_POP || is_RET) && state == STATE_WAIT_FOR_RAM && bus_ready) begin
  if (is_POP)  state <= STATE_SET_REG;   // Rn <= [SP]
  else         state <= STATE_RET;       // PC <= [SP]
end

else if (state == STATE_SET_REG) begin
  if (is_POP)                           state <= STATE_INC_SP;      // now SP++
  else                                  state <= STATE_FETCH_PC;
end

else if (state == STATE_REG_STORE)      state <= STATE_MEM_WRITE;   // PUSH write hold
else if (state == STATE_MEM_WRITE)      state <= STATE_FETCH_PC;

else if (state == STATE_RET)            state <= STATE_INC_SP;      // bump SP after RET
else if (state == STATE_INC_SP)         state <= STATE_FETCH_PC;
    // Normal cycle stepping
    else if (!ldi_in_progress) begin
      case (cycle)
        4'd0: state <= STATE_FETCH_PC;
        4'd1: state <= STATE_WAIT_FOR_RAM;
        4'd2: state <= STATE_WAIT_FOR_RAM;
        4'd3: state <= STATE_FETCH_INST;
        4'd4: state <= STATE_NEXT;
        4'd5: state <= STATE_NEXT;
4'd6: begin
  unique case (1'b1)
    is_HLT:                 state <= STATE_HALT;
    is_MOV:                 state <= STATE_MOV_FETCH;
    (is_ALU || is_CMP):     state <= STATE_REG_READ;
    (is_RET || is_POP):     state <= STATE_FETCH_SP; // read first
    is_PUSH:                state <= STATE_DEC_SP;   // pre-decrement
    is_CALL:                state <= STATE_SET_REG;
    is_LDI: begin
      state           <= STATE_FETCH_IMM;
      ldi_in_progress <= 1'b1;
      $display("[FSM] LDI detected, entering FETCH_IMM");
    end
    is_JMP:                 state <= STATE_JUMP;
    (is_IN || is_OUT):      state <= STATE_SET_ADDR;
    default:                state <= STATE_NEXT;
  endcase
end

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

  // ---- Control signals ---------------------------------------------------------------
  always_comb begin
    c_rfi   = 1'b0;
    c_rfo   = 1'b0;
    c_eo    = 1'b0;

    // PC control: inc when instruction or immediate is accepted
    pc_inc  = ((state == STATE_FETCH_INST) && bus_ready) ||
              ((state == STATE_WAIT_IMM)   && bus_ready);

    pc_load = (state == STATE_JUMP) && jump_allowed
           || (state == STATE_RET)
           || (state == STATE_TMP_JUMP);

    pc_dec = 1'b0;

    unique case (state)
  STATE_ALU_WRITEBACK: begin
    c_eo  = 1'b1;
    c_rfi = 1'b1;
  end
  STATE_LOAD_IMM:      if (is_LDI) c_rfi = 1'b1;
  STATE_IN:            c_rfi = 1'b1;
  STATE_SET_ADDR:      c_rfi = 1'b1;
  STATE_SET_REG:       c_rfi = 1'b1;

  STATE_MOV_STORE: begin
    if (instruction[2:0] != 3'b111) c_rfi = 1'b1; // reg dst
    if (instruction[5:3] != 3'b111) c_rfo = 1'b1; // reg src
  end

  // PUSH: drive source register during write & hold
  STATE_REG_STORE:     c_rfo = 1'b1;
  STATE_MEM_WRITE:     c_rfo = 1'b1;
      default: begin
        c_rfi = 1'b0;
        c_rfo = 1'b0;
      end
    endcase
  end

endmodule