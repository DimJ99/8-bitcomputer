module cpu_ctrl (
  input  logic       clk,
  input  logic       reset_cycle,
  input  logic [7:0] instruction,
  output logic [7:0] state,
  output logic [3:0] cycle,
  output logic [7:0] opcode
);

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

  localparam [7:0]
    STATE_NEXT          = 8'h00,
    STATE_FETCH_PC      = 8'h01,
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
    STATE_LOAD_IMM      = 8'h15;

  logic [7:0] instruction_reg;
  logic [7:0] latched_opcode;

  initial cycle = 0;

  // Cycle counter
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)
      cycle <= 0;
    else
      cycle <= (cycle == 7) ? 0 : cycle + 1;
  end

  // Latch instruction at cycle 2
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)
      instruction_reg <= 8'h00;
    else if (cycle == 2)
      instruction_reg <= instruction;
  end

  // Decode opcode at cycle 3
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle)
      latched_opcode <= 8'h00;
    else if (cycle == 3) begin
      casez (instruction_reg)
        8'b00_010_???: latched_opcode <= OP_LDI;
        8'b10_???_???: latched_opcode <= OP_MOV;
        8'b01_???_000: latched_opcode <= OP_ALU;
        8'b00_000_110: latched_opcode <= OP_CMP;
        8'b00_011_???: latched_opcode <= OP_JMP;
        8'b00_100_???: latched_opcode <= OP_PUSH;
        8'b00_101_???: latched_opcode <= OP_POP;
        default:        latched_opcode <= instruction_reg;
      endcase
    end
  end

  // Control state machine
  always_ff @(posedge clk or posedge reset_cycle) begin
    if (reset_cycle) begin
      state <= STATE_FETCH_PC;
    end else begin
      case (cycle)
        4'd0: state <= STATE_FETCH_PC;
        4'd1: state <= STATE_FETCH_INST;
        4'd2: state <= STATE_NEXT;
        4'd3: state <= STATE_NEXT;

        4'd4: state <= (latched_opcode == OP_HLT)                             ? STATE_HALT      :
                       (latched_opcode == OP_MOV)                             ? STATE_MOV_FETCH :
                       (latched_opcode == OP_ALU || latched_opcode == OP_CMP) ? STATE_ALU_EXEC  :
                       (latched_opcode == OP_RET || latched_opcode == OP_POP) ? STATE_INC_SP    :
                       (latched_opcode == OP_PUSH)                            ? STATE_FETCH_SP  :
                       (latched_opcode == OP_IN  || latched_opcode == OP_OUT ||
                        latched_opcode == OP_CALL || latched_opcode == OP_LDI ||
                        latched_opcode == OP_JMP)                             ? STATE_FETCH_PC  :
                                                                                STATE_NEXT;

        4'd5: state <= (latched_opcode == OP_JMP)   ? STATE_JUMP        :
                       (latched_opcode == OP_LDI)   ? STATE_NEXT        :  // delay 1 cycle
                       (latched_opcode == OP_MOV)   ? STATE_MOV_LOAD    :
                       (latched_opcode == OP_ALU)   ? STATE_ALU_OUT     :
                       (latched_opcode == OP_OUT ||
                        latched_opcode == OP_IN)    ? STATE_SET_ADDR    :
                       (latched_opcode == OP_PUSH)  ? STATE_REG_STORE   :
                       (latched_opcode == OP_CALL)  ? STATE_SET_REG     :
                       (latched_opcode == OP_RET ||
                        latched_opcode == OP_POP)   ? STATE_FETCH_SP    :
                                                      STATE_NEXT;

        4'd6: state <= (latched_opcode == OP_LDI)   ? STATE_LOAD_IMM    :
                       (latched_opcode == OP_MOV)   ? STATE_MOV_STORE   :
                       (latched_opcode == OP_CALL)  ? STATE_FETCH_SP    :
                       (latched_opcode == OP_RET)   ? STATE_RET         :
                       (latched_opcode == OP_OUT)   ? STATE_OUT         :
                       (latched_opcode == OP_POP)   ? STATE_SET_REG     :
                       (latched_opcode == OP_IN)    ? STATE_IN          :
                                                      STATE_NEXT;

        4'd7: state <= (latched_opcode == OP_LDI)   ? STATE_SET_REG     :
                       (latched_opcode == OP_CALL)  ? STATE_PC_STORE    :
                                                      STATE_NEXT;

        default: state <= STATE_NEXT;
      endcase
    end

    // Debugging
    $display("[CTRL FSM] Cycle=%0d | instruction = %h (%b)", cycle, instruction, instruction);
    $display("[CTRL FSM] instruction_reg = %h | latched_opcode = %h", instruction_reg, latched_opcode);
    $display("[CTRL FSM] state = %h", state);
  end

  assign opcode = latched_opcode;

endmodule
