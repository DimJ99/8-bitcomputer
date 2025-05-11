RTL_SRCS   = $(wildcard rtl/*.sv)
TESTBENCH  = rtl/tb/machine_tb.v
OUTPUT     = computer

build:
	iverilog -g2012 -Wall -o $(OUTPUT) \
		$(RTL_SRCS) \
		$(TESTBENCH)

run: build
	vvp -n $(OUTPUT)

clean:
	rm -rf $(OUTPUT) *.vcd


view:
	gtkwave machine.vcd gtkwave/config.gtkw


tests:
	bats tests/tests.bats
