#!/usr/bin/env python3
import re
import sys
from typing import List, Dict

inst = {
    "nop": 0x00,
    "call": 0b00000001,
    "ret": 0b00000010,
    "lda": 0b10000111,
    "out": 0b00000011,
    "in":  0b00000100,
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

class AsmError(Exception):
    pass

def rich_int(v: str) -> int:
    v = v.strip()
    if v.startswith(("0x", "0X")):
        return int(v, 16)
    if v.startswith(("0b", "0B")):
        return int(v, 2)
    return int(v)

def strip_comment(line: str) -> str:
    return re.sub(r";.*", "", line)

def split_label(line: str):
    """
    Returns (label_or_None, rest_of_line_without_label)
    Handles 'foo:' and 'foo: instruction ...'
    """
    m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$", line)
    if m:
        return m.group(1), m.group(2)
    return None, line

def tokenize(rest: str):
    """
    Split operands on commas and whitespace, drop empties.
    """
    return [t.strip() for t in re.split(r"[,\s]+", rest) if t.strip()]

def is_symbol(x: str) -> bool:
    return x.startswith("%") and len(x) > 1

def expect_args(op, args, n_min=None, n_max=None, exact=None):
    n = len(args)
    if exact is not None:
        if n != exact:
            raise AsmError(f"{op}: expected {exact} arg(s), got {n}")
    else:
        if n_min is not None and n < n_min:
            raise AsmError(f"{op}: expected at least {n_min} arg(s), got {n}")
        if n_max is not None and n > n_max:
            raise AsmError(f"{op}: expected at most {n_max} arg(s), got {n}")

def assemble(progf: str) -> List[int]:
    mem: List[int] = []
    section = None
    labels: Dict[str, int] = {}
    data_kv: Dict[str, int] = {}
    data_addr: Dict[str, int] = {}
    symbolic_refs: Dict[int, str] = {}

    with open(progf, "r", encoding="utf-8") as f:
        for raw in f:
            line = strip_comment(raw).strip()
            if not line:
                continue

            if line.lower() == ".text":
                section = TEXT
                continue
            if line.lower() == ".data":
                section = DATA
                continue
            if section is None:
                raise AsmError("No section selected. Start with .text or .data")

        
            label, rest = split_label(line)
            if label is not None:
                if section != TEXT:
                    raise AsmError(f"Label '{label}' outside .text")
                labels[label] = len(mem)
                line = rest.strip()
                if not line:
                    continue 

            if section == DATA:
                if "=" not in line:
                    raise AsmError(f".data line must be 'name = value', got: {line}")
                key, val = map(str.strip, line.split("=", 1))
                if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", key):
                    raise AsmError(f"Invalid data symbol name: {key}")
                try:
                    data_kv[key] = rich_int(val)
                except ValueError:
                    raise AsmError(f"Invalid numeric literal for {key}: {val}")
                continue

      
            tokens = tokenize(line)
            if not tokens:
                continue

            op = tokens[0].lower()
            args = tokens[1:]

            def reg_code(name: str) -> int:
                n = name.upper()
                if n not in reg:
                    raise AsmError(f"Unknown register '{name}'")
                return reg[n]

            def emit(byte: int):
                if not (0 <= byte <= 0xFF):
                    raise AsmError(f"Byte out of range: {byte}")
                mem.append(byte)

            def emit_addr_or_symbol(token: str):
                if is_symbol(token):
                    symbolic_refs[len(mem)] = token[1:]
                    emit(0)
                else:
                    emit(rich_int(token))

            if op not in inst:
                raise AsmError(f"Unknown instruction '{op}'")

            if op == "ldi":
                expect_args(op, args, exact=2)
                if args[0].upper() == "M":
                    raise AsmError("ldi to M (memory) is not supported")
                r = reg_code(args[0])
                opcode = (inst[op] & 0b11111000) | r
                emit(opcode)
                emit_addr_or_symbol(args[1])

            elif op in ("push", "pop"):
                expect_args(op, args, exact=1)
                r = reg_code(args[0])
                opcode = (inst[op] & 0b11111000) | r
                emit(opcode)

            elif op == "mov":
                expect_args(op, args, exact=2)
                r1 = reg_code(args[0])
                r2 = reg_code(args[1])
                opcode = (inst[op] & 0b11000111) | (r1 << 3)
                opcode = (opcode & 0b11111000) | r2
                emit(opcode)

            elif op in ("sta", "lda", "jmp", "jz", "jnz", "je", "jne", "jc", "jnc"):
                expect_args(op, args, exact=1)
                emit(inst[op])
                emit_addr_or_symbol(args[0])

            elif op in ("out", "in"):
                expect_args(op, args, exact=1)
                emit(inst[op])
                emit(rich_int(args[0]))

            else:
                expect_args(op, args, exact=0)
                emit(inst[op])


    for k, v in data_kv.items():
        data_addr[k] = len(mem)
        if not (0 <= v <= 0xFF):
            raise AsmError(f"Data value for {k} out of 8-bit range: {v}")
        mem.append(v)

  
    data_addr.update(labels)

    for idx, name in symbolic_refs.items():
        if name not in data_addr:
            raise AsmError(f"undefined symbol %{name}")
        addr = data_addr[name]
        if not (0 <= addr <= 0xFF):
            raise AsmError(f"Address for %{name} out of 8-bit range: {addr}")
        mem[idx] = addr


    if len(mem) > MEM_SIZE:
        raise AsmError(f"program too large: {len(mem)} bytes > {MEM_SIZE}")
    while len(mem) < MEM_SIZE:
        mem.append(0)

    return mem

def main():
    if len(sys.argv) != 2:
        print("Usage: asm.py program.asm", file=sys.stderr)
        sys.exit(2)
    progf = sys.argv[1]
    try:
        mem = assemble(progf)
    except AsmError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(" ".join(f"{b:02x}" for b in mem))

if __name__ == "__main__":
    main()
