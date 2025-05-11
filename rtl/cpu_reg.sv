module cpu_reg (
  input  logic        clk,
  input  logic [7:0]  data_in,
  input  logic [2:0]  sel_in,
  input  logic [2:0]  sel_out,
  input  logic        enable_write,
  input  logic        output_enable,
  output logic [7:0]  data_out,
  output logic [7:0]  rega,
  output logic [7:0]  regb
);

  // MEM array of 8 registers, each 8 bits wide
  logic [7:0] registers [0:7];

// Register initialization
  always_ff @(posedge clk) begin
    if (enable_write)
      registers[sel_in] <= data_in;
  end
  assign data_out = output_enable ? registers[sel_out] : 'bz; //debugging

  // Individual register access (optional debug or named access)
  logic [7:0] regc, regd, rege, regf, regg, regt;

  assign rega = registers[0];
  assign regb = registers[1];
  assign regc = registers[2];
  assign regd = registers[3];
  assign rege = registers[4];
  assign regf = registers[5];
  assign regg = registers[6];
  assign regt = registers[7];

endmodule
