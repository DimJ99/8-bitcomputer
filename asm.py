#!/usr/bin/env python3
import re
import sys

progf = sys.argv[1]

inst = {
    "nop": 0x00,
    "call": 0b00000001,
    "ret": 0b00000010,
    "lda": 0b10000111,
    "out": 0b00000011,
    "in": 0b00000100,
    "hlt": 0b00000101,
    "cmp": 0b00000110,
    "sta": 0b10111000,
    "jmp": 0b00011000,
    "jz":  0b00011001,
    "jnz": 0b00011010,
    "je":  0b00011001,
    "jne": 0b00011010,
    "jc":  0b00011011,
    "jnc": 0b00011100,
    "push": 0b00100000,
    "pop":  0b00101000,
    "add":  0b01000000,
    "sub":  0b01001000,
    "inc":  0b01010000,
    "dec":  0b01011000,
    "and":  0b01100000,
    "or":   0b01101000,
    "xor":  0b01110000,
    "adc":  0b01111000,
    "ldi":  0b00010000,
    "mov":  0b10000000,
}

reg = {
    "A": 0b000,
    "B": 0b001,
    "C": 0b010,
    "D": 0b011,
    "E": 0b100,
    "F": 0b101,
    "G": 0b110,
    "M": 0b111,
}

TEXT, DATA = 0, 1
MEM_SIZE = 256
mem = []
section = None
labels = {}
data = {}
data_addr = {}
symbolic_refs = {}

def rich_int(v):
    v = v.strip()
    if v.startswith("0x"):
        return int(v, 16)
    elif v.startswith("0b"):
        return int(v, 2)
    else:
        return int(v)

with open(progf) as f:
    for line in f:
        line = re.sub(r";.*", "", line).strip()
        if not line:
            continue

        if line == ".text":
            section = TEXT
            continue
        elif line == ".data":
            section = DATA
            continue

        if section == DATA:
            if "=" in line:
                key, val = map(str.strip, line.split("=", 1))
                data[key] = int(val)
        elif section == TEXT:
            tokens = line.split()
            if not tokens:
                continue
            if tokens[0].endswith(":"):
                label = tokens[0][:-1]
                labels[label] = len(mem)
                continue

            op = tokens[0]
            args = tokens[1:]

            if op == "ldi":
                r = reg[args[0]]
                opcode = (inst[op] & 0b11111000) | r
                mem.append(opcode)
                if args[1].startswith("%"):
                    symbolic_refs[len(mem)] = args[1][1:]
                    mem.append(0)
                else:
                    mem.append(rich_int(args[1]))
            elif op in ("push", "pop"):
                r = reg[args[0]]
                opcode = (inst[op] & 0b11111000) | r
                mem.append(opcode)
            elif op == "mov":
                r1 = reg[args[0]]
                r2 = reg[args[1]]
                opcode = (inst[op] & 0b11000111) | (r1 << 3)
                opcode = (opcode & 0b11111000) | r2
                mem.append(opcode)
            elif op in ("sta", "lda", "jmp", "jz", "jnz", "je", "jne", "jc", "jnc"):
                opcode = inst[op]
                mem.append(opcode)
                if args[0].startswith("%"):
                    symbolic_refs[len(mem)] = args[0][1:]
                    mem.append(0)
                else:
                    mem.append(rich_int(args[0]))
            elif op == "out":
                mem.append(inst[op])
                mem.append(rich_int(args[0]))
            else:
                mem.append(inst[op])

# Append .data section and store addresses
for k, v in data.items():
    data_addr[k] = len(mem)
    mem.append(v)

# Add labels to the lookup
data_addr.update(labels)

# Replace symbolic references (%label or %var)
for i, name in symbolic_refs.items():
    if name not in data_addr:
        print(f"Error: undefined symbol %{name}")
        sys.exit(1)
    mem[i] = data_addr[name]

# Output memory in hex format
print(' '.join(f"{b:02x}" for b in mem))
