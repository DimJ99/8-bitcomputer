module cpu_ctrl (
    input  logic       clk,
    input  logic       reset_cycle,
    input  logic [7:0] instruction,
    output logic [7:0] state,
    output logic [3:0] cycle,
    input  logic       bus_ready,
    output logic [7:0] opcode,
    output logic       c_rfi,
    output logic       c_rfo,
    output logic       pc_inc,    // counter.inc  
    output logic       pc_load,   // counter.load 
    output logic       pc_dec,    // counter.dec  
    input  logic       jump_allowed
);

  // ─── Opcodes ─────────────────────────────────────────────
  localparam [7:0]
    OP_NOP   = 8'b00_000_000,
    OP_CALL  = 8'b00_000_001,
    OP_RET   = 8'b00_000_010,
    OP_OUT   = 8'b00_000_011,
    OP_IN    = 8'b00_000_100,
    OP_HLT   = 8'b00_000_101,
    OP_CMP   = 8'b00_000_110,
    OP_LDI   = 8'b00_010_000,
    OP_JMP   = 8'b00_011_000,
    OP_PUSH  = 8'b00_100_000,
    OP_POP   = 8'b00_101_000,
    OP_ALU   = 8'b01_000_000,
    OP_MOV   = 8'b10_000_000;
    
  // ─── States ──────────────────────────────────────────────
  localparam [7:0]
    STATE_NEXT          = 8'h00,
    STATE_FETCH_PC      = 8'h01,
    STATE_WAIT_FOR_RAM  = 8'h16,
    STATE_FETCH_INST    = 8'h02,
    STATE_HALT          = 8'h03,
    STATE_JUMP          = 8'h04,
    STATE_OUT           = 8'h05,
    STATE_ALU_OUT       = 8'h06,
    STATE_ALU_EXEC      = 8'h07,
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
    STATE_ALU_WRITEBACK = 8'h17,
    STATE_REG_READ      = 8'h18,
    STATE_FETCH_IMM     = 8'h19,
    STATE_MOV_MEM_ADDR  = 8'h1C;

  logic [7:0] instruction_reg;
  logic [7:0] latched_opcode;
  logic [7:0] immediate_value;
  logic       ldi_in_progress;  // Flag to track LDI execution

  // ─── Cycle Counter ───────────────────────────────────────
  initial cycle = 0;
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)
      cycle <= 0;
    else if (!ldi_in_progress)  // Freeze cycle counter during LDI
      cycle <= (cycle == 7) ? 0 : cycle + 1;
  end

  // ─── Latch Instruction ───────────────────────────────────
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      instruction_reg <= 8'h00;
      immediate_value <= 8'h00;
      latched_opcode  <= 8'h00;
    end 
    // Normal opcode fetch
    else if (state == STATE_FETCH_INST && bus_ready && !ldi_in_progress) begin
      instruction_reg <= instruction;
      latched_opcode  <= instruction;
      $display("[INST LATCH] New instruction: %h, latched_opcode: %h", instruction, instruction);
    end 
    // Immediate fetch for LDI
    else if (state == STATE_FETCH_IMM && bus_ready) begin
      immediate_value <= instruction;
      $display("[IMM LATCH] Immediate value: %h for opcode: %h", instruction, latched_opcode);
    end
  end

  // ─── Control FSM ─────────────────────────────────────────
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      state            <= STATE_FETCH_PC;
      ldi_in_progress  <= 0;
    end 
    else if (state == STATE_HALT) begin
      state <= STATE_HALT;

    // LDI states
    end 
    else if (state == STATE_FETCH_IMM) begin
      if (bus_ready) state <= STATE_LOAD_IMM;
      else           state <= STATE_FETCH_IMM;
    end 
    else if (state == STATE_LOAD_IMM) begin
      state           <= STATE_FETCH_PC;
      ldi_in_progress <= 0;
      $display("[LDI FSM] LDI complete, returning to FETCH_PC");

    // Other multi-cycle
    end 
    else if (state == STATE_MOV_FETCH)      state <= STATE_MOV_LOAD;
    else if (state == STATE_MOV_LOAD)       state <= STATE_MOV_STORE;
    else if (state == STATE_MOV_STORE)      state <= STATE_FETCH_PC;
    else if (state == STATE_ALU_WRITEBACK)  state <= STATE_FETCH_PC;
    else if (state == STATE_PC_STORE)       state <= STATE_FETCH_PC;
    else if (state == STATE_SET_REG)        state <= STATE_FETCH_PC;
    else if (state == STATE_REG_STORE)      state <= STATE_FETCH_PC;
    else if (state == STATE_WAIT_FOR_RAM && !bus_ready) state <= STATE_WAIT_FOR_RAM;
    else if (state == STATE_REG_READ)       state <= STATE_ALU_EXEC;

    // Normal cycle logic only if NOT in LDI
    else if (!ldi_in_progress) begin
      case (cycle)
        4'd0: state <= STATE_FETCH_PC;
        4'd1: state <= STATE_WAIT_FOR_RAM;
        4'd2: state <= STATE_WAIT_FOR_RAM;
        4'd3: state <= STATE_FETCH_INST;
        4'd4: state <= STATE_NEXT;
        4'd5: state <= STATE_NEXT;
        4'd6: begin
          case (latched_opcode)
            OP_HLT: state <= STATE_HALT;
            OP_MOV: state <= STATE_MOV_FETCH;
            OP_ALU, OP_CMP: state <= STATE_REG_READ;
            OP_RET, OP_POP: state <= STATE_INC_SP;
            OP_PUSH: state <= STATE_FETCH_SP;
            OP_CALL: state <= STATE_SET_REG;
            OP_LDI: begin
              state           <= STATE_FETCH_IMM;
              ldi_in_progress <= 1;
              $display("[FSM] LDI detected, entering FETCH_IMM");
            end
            OP_JMP: state <= STATE_JUMP;
            OP_IN, OP_OUT: state <= STATE_SET_ADDR;
            default: state <= STATE_NEXT;
          endcase
        end
        4'd7: begin
          case (latched_opcode)
            OP_ALU:  state <= STATE_ALU_WRITEBACK;
            OP_PUSH: state <= STATE_REG_STORE;
            OP_CALL: state <= STATE_PC_STORE;
            OP_RET, OP_POP: state <= STATE_FETCH_SP;
            OP_JMP:  state <= STATE_FETCH_PC;
            OP_IN, OP_OUT: state <= STATE_NEXT;
            default: state <= STATE_NEXT;
          endcase
        end
        default: state <= STATE_NEXT;
      endcase
    end
  end

  assign opcode = latched_opcode;

  // ─── Control Signal Generation ──────────────────────────
  always_comb begin
    c_rfi = 0;
    c_rfo = 0;

    // PC increments on normal fetch, and also for LDI immediate
    pc_inc = (state == STATE_FETCH_PC) || (state == STATE_FETCH_IMM);

    // PC loads on jumps/calls/returns
    pc_load = (state == STATE_JUMP && jump_allowed)
            || (state == STATE_PC_STORE)
            || (state == STATE_RET)
            || (state == STATE_TMP_JUMP);

    // PC decrements for RET/POP
    pc_dec = (state == STATE_RET) || (state == STATE_FETCH_SP);
            
    case (state)
      STATE_ALU_WRITEBACK: c_rfi = 1;
      STATE_LOAD_IMM: if (latched_opcode == OP_LDI) c_rfi = 1;
      STATE_IN:       c_rfi = 1;
      STATE_SET_ADDR: c_rfi = 1;
      STATE_SET_REG:  c_rfi = 1;
      STATE_MOV_STORE: if (instruction[5:3] != 3'b111) c_rfi = 1;
      STATE_REG_READ: c_rfo = 1;
      default: begin
        c_rfi = 0;
        c_rfo = 0;
      end
    endcase
  end

endmodule
