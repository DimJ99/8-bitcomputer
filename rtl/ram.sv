module ram (
  input  logic       clk,
  input  logic [7:0] addr,
  input  logic       o,   
  input  logic       oe,   
  inout  tri  [7:0] data  
);
  logic [7:0] mem [0:255];
  logic [7:0] buffer;

  initial begin
    $readmemh("memory.list", mem);
    $display("Memory initialized from memory.list");
  end

  // Drive the bus only when reading (oe=1) and NOT writing (o=0)
  assign data = (oe && !o) ? buffer : 8'hZZ;

  always_ff @(posedge clk) begin
    if (o) begin
      // WRITE: sample data from the external bus
      mem[addr] <= data;
      $display("Memory: set [0x%0h] => 0x%0h", addr, data);
    end else begin
      // READ: prefetch into buffer (1-cycle sync read)
      buffer <= mem[addr];
      $display("[RAM] READ   @ 0x%0h => 0x%0h", addr, mem[addr]);
    end
  end
endmodule
