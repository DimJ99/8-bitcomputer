# Collect all RTL source files (no need for separate libraries)
RTL_SRCS   = $(wildcard rtl/*.sv)
TESTBENCH  = rtl/tb/machine_tb.v
OUTPUT     = computer

# Default build rule
build:
	iverilog -g2012 -Wall -o $(OUTPUT) \
		$(RTL_SRCS) \
		$(TESTBENCH)

# Run the simulation
run: build
	vvp -n $(OUTPUT)

# Clean up generated files
clean:
	rm -rf $(OUTPUT) *.vcd

# View waveform
view:
	gtkwave machine.vcd gtkwave/config.gtkw

# Run test suite
tests:
	bats tests/tests.bats

.PHONY: build run clean view tests
