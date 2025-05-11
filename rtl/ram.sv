module ram (
  input  logic       clk,
  input  logic [7:0] addr,
  input  logic       o,  
  input  logic       oe,   
  inout  logic [7:0] data  
);

  // 256 x 8-bit memory
  logic [7:0] mem [0:255];
  logic [7:0] buffer;

  // Internal tri-state driver
  logic [7:0] data_out;
  assign data_out = (oe && !o) ? buffer : 'bz;
  assign data     = data_out;
  
  // Write or buffer on clock edge
  always_ff @(posedge clk) begin
    if (o) begin
      mem[addr] <= data;
      $display("Memory: set [0x%0h] => 0x%0h (%0d)", addr, data, data);
    end else begin
      buffer <= mem[addr];
    end
  end

endmodule
