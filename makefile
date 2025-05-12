RTL_SRCS   = $(wildcard rtl/*.sv)
TESTBENCH  = rtl/tb/machine_tb.sv
OUTPUT     = computer
MEMORY     = memory.list
ASM        = tests/multiplication_test.asm

build: $(MEMORY)
	@echo "[Building SystemVerilog sources...]"
	iverilog -g2012 -Wall -o $(OUTPUT) $(RTL_SRCS) $(TESTBENCH)

$(MEMORY): $(ASM)
	@echo "[Assembling: $(ASM) → $(MEMORY)]"
	./asm/asm.py $(ASM) > $(MEMORY)

run: build
	@echo "[Running simulation...]"
	vvp -n $(OUTPUT)

clean:
	@echo "[Cleaning...]"
	rm -rf $(OUTPUT) *.vcd $(MEMORY)

view:
	gtkwave machine.vcd gtkwave/config.gtkw

tests:
	bats tests/tests.bats
