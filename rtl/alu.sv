// ============================================================================
// alu.sv — 8-bit, clocked ALU with Z/C flags
// ----------------------------------------------------------------------------
// • Sequential (clocked) ALU: results and flags update on clk when `enable=1`.
// • Supported ops: ADD, ADC, SUB, INC, DEC, AND, OR, XOR.
// • Flags:
//    - flag_zero  (Z): 1 when result == 0
//    - flag_carry (C): carry-out for ADD/ADC/INC; for SUB/DEC this is the
//      “no-borrow” bit as produced by {C,result} = A - B in two’s complement.
// • Reset clears result and flags.
// • Note on nonblocking (`<=`) semantics:
//    Z is assigned from `buff_out` in the same clocked block. Because RHS values
//    are read *before* any of the `<=` updates take effect, Z here observes the
//    previous cycle’s `buff_out`. That’s acceptable if you intend flags to be
//    registered alongside results, but it’s easy to trip over in simulation.
//    (Keeping the code as-is per your “comments only” request.)
// ============================================================================

module alu (
  input  logic        enable,     // when 1, perform op on this clock edge
  input  logic        clk,
  input  logic        reset,      // async active-high reset
  input  logic [2:0]  op,         // selects operation (see localparams)
  input  logic [7:0]  in_a,       // operand A (often REG_A)
  input  logic [7:0]  in_b,       // operand B (often REG_B)
  output logic [7:0]  out,        // registered ALU result
  output logic        flag_zero,  // Z flag (1 when result == 0)
  output logic        flag_carry  // C flag (carry/no-borrow)
);

  // ALU operation codes
  localparam ALU_ADD = 3'b000;
  localparam ALU_SUB = 3'b001;
  localparam ALU_INC = 3'b010;
  localparam ALU_DEC = 3'b011;
  localparam ALU_AND = 3'b100;
  localparam ALU_OR  = 3'b101;
  localparam ALU_XOR = 3'b110;
  localparam ALU_ADC = 3'b111;

  // Registered result
  logic [7:0] buff_out;

  // --------------------------------------------------------------------------
  // Clocked ALU core
  // --------------------------------------------------------------------------
  // • On reset: clear result and flags.
  // • When enable=1: compute based on `op`, update result and C flag.
  //   Z flag is updated from `buff_out` (see note above about timing).
  // • When enable=0: hold current result/flags.
  //
  // Carry/borrow details:
  //   - ADD/ADC: `{flag_carry,buff_out} <= in_a + in_b [+ carry_in]`
  //   - SUB/DEC: `{flag_carry,buff_out} <= in_a - in_b/1`
  //              In two’s complement, this bit is “~borrow” (1 means no borrow).
  //   - Logical ops clear C.
  //
  // ADC uses the *previous* cycle’s flag_carry as carry-in (because RHS reads
  // old values before nonblocking updates). That matches typical CPU semantics.
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin // reset to zero
      buff_out   <= 8'd0;
      flag_zero  <= 1'b0;
      flag_carry <= 1'b0;
    end else if (enable) begin
      case (op) // ALU operation
        ALU_ADD: {flag_carry, buff_out} <= in_a + in_b;                  // add
        ALU_ADC: {flag_carry, buff_out} <= in_a + in_b + flag_carry;     // add + carry-in
        ALU_SUB: {flag_carry, buff_out} <= in_a - in_b;                  // C = ~borrow
        ALU_INC: {flag_carry, buff_out} <= in_a + 1;                     // increment A
        ALU_DEC: {flag_carry, buff_out} <= in_a - 1;                     // decrement A
        ALU_AND: begin
          buff_out   <= in_a & in_b;
          flag_carry <= 1'b0;                                            // C cleared on logicals
        end
        ALU_OR: begin
          buff_out   <= in_a | in_b;
          flag_carry <= 1'b0;
        end
        ALU_XOR: begin
          buff_out   <= in_a ^ in_b;
          flag_carry <= 1'b0;
        end
        default: begin
          buff_out   <= 8'hXX;  // drive X to help catch invalid op decodes in sim
          flag_carry <= 1'b0;
        end
      endcase

      // Z flag update (registered)
      // As noted above, this compares the pre-update value of buff_out due to
      // nonblocking semantics; the flag is effectively aligned with result in
      // the next cycle. Left as-is to preserve your behavior.
      flag_zero <= (buff_out == 8'd0);
    end
  end

  // Combinational drive of the registered result
  assign out = buff_out;

endmodule
