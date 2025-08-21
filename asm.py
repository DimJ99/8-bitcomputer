#!/usr/bin/env python3
# =============================================================================
# Tiny 8-bit assembler for your microcoded CPU
# -----------------------------------------------------------------------------
# What this script does
# • Reads a simple assembly file with two sections: `.text` (code) and `.data`
# • Supports labels in `.text` and named byte constants in `.data`
# • Encodes your ISA into bytes according to the bit layouts in `inst`/`reg`
# • Allows symbolic immediates using `%name` (resolved after we know addresses)
# • Produces exactly 256 bytes (pads with 0s) printed as hex, space-separated
#
# Assembly syntax overview
# • Sections:     `.text` starts code; `.data` starts constant data
# • Comments:     `;` to end of line
# • Labels:       `start:` defines a code label at that location (only in .text)
# • Data consts:  In `.data`, lines like `FOO = 0x2A` define 1-byte values
# • Symbols:      Use `%FOO` in `.text` to refer to label or data symbol `FOO`
# • Registers:    A,B,C,D,E,F,G,M   (M=memory pseudo-reg, encoded as 111b)
#
# ISA encoding (matches your RTL comments)
# • LDI:  00_010_rrr  imm8                (rrr = dest reg)
# • MOV:  10_sss_rrr                      (sss = src, rrr = dst; M=111 means mem)
# • ALU:  01_op_rrr                       (op in [5:3], dst=rrr (REG_A in datapath))
# • CTRL: exact opcodes (NOP/CALL/RET/OUT/IN/HLT/CMP)
# • Jxx:  00_011_ccc imm8                 (ccc encodes condition; here we expose
#                                          mnemonics jmp/jz/jnz/je/jne/jc/jnc)
# • STA/LDA: assembled as MOV with M involved; here provided as dedicated forms.
# =============================================================================

import re
import sys
from typing import List, Dict

# -----------------------------------------------------------------------------
# Opcode map (base encodings). Some are families (upper bits fixed, low bits carry
# registers/conditions), others are exact bytes.
# -----------------------------------------------------------------------------
inst = {
    "nop": 0x00,
    "call": 0b00000001,
    "ret": 0b00000010,
    "lda": 0b10000111,    # MOV M->A form convenience (family 10 with src=M,dst=A)
    "out": 0b00000011,
    "in":  0b00000100,
    "hlt": 0b00000101,
    "cmp": 0b00000110, 
    "sta": 0b10111000,    # MOV A->M form convenience (family 10 with src=A,dst=M)
    "jmp": 0b00011000,    # family head for Jxx (condition in low 3 bits)
    "jz":  0b00011001,
    "jnz": 0b00011010,
    "je":  0b00011001,    # alias of jz
    "jne": 0b00011010,    # alias of jnz
    "jc":  0b00011011,
    "jnc": 0b00011100,
    "push": 0b00100000,   # family 00_100_rrr
    "pop":  0b00101000,   # family 00_101_rrr
    "add":  0b01000000,   # ALU family 01_op_rrr
    "sub":  0b01001000,
    "inc":  0b01010000,
    "dec":  0b01011000,
    "and":  0b01100000,
    "or":   0b01101000,
    "xor":  0b01110000,
    "adc":  0b01111000,
    "ldi":  0b00010000,   # family 00_010_rrr
    "mov":  0b10000000,   # family 10_sss_rrr
}

# 3-bit register codes (M=111 is the memory pseudo-register used by MOV/LDA/STA)
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

# Section identifiers (simple state machine while reading the source file)
TEXT, DATA = 0, 1

# Output image size is exactly 256 bytes (fits your 8-bit address space)
MEM_SIZE = 256

# so long and good night!!
class AsmError(Exception):
    """Custom exception for clean error messages from the assembler."""
    pass

# -----------------------------------------------------------------------------
# Helpers for parsing
# -----------------------------------------------------------------------------
def rich_int(v: str) -> int:
    """Parses decimal, hex (0x...), or binary (0b...) integers."""
    v = v.strip()
    if v.startswith(("0x", "0X")):
        return int(v, 16)
    if v.startswith(("0b", "0B")):
        return int(v, 2)
    return int(v)

def strip_comment(line: str) -> str:
    """Removes '; ...' comments from a line."""
    return re.sub(r";.*", "", line)

def split_label(line: str):
    """
    If the line starts with a label (`foo:` or `foo: rest`), return (label, rest).
    Otherwise return (None, original_line).
    """
    m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$", line)
    if m:
        return m.group(1), m.group(2)
    return None, line

def tokenize(rest: str):
    """
    Split an instruction tail into tokens by commas/whitespace. Drops empties.
    Example: 'mov A, B' -> ['mov','A','B']
    """
    return [t.strip() for t in re.split(r"[,\s]+", rest) if t.strip()]

def is_symbol(x: str) -> bool:
    """Symbols for immediates are written as %NAME (resolved after assembly)."""
    return x.startswith("%") and len(x) > 1

def expect_args(op, args, n_min=None, n_max=None, exact=None):
    """Small checker to produce helpful arity errors for each mnemonic."""
    n = len(args)
    if exact is not None:
        if n != exact:
            raise AsmError(f"{op}: expected {exact} arg(s), got {n}")
    else:
        if n_min is not None and n < n_min:
            raise AsmError(f"{op}: expected at least {n_min} arg(s), got {n}")
        if n_max is not None and n > n_max:
            raise AsmError(f"{op}: expected at most {n_max} arg(s), got {n}")

# -----------------------------------------------------------------------------
# One-pass assembly with late fix-ups for symbols:
# • We linearize .text into mem[]
# • We collect .data key/value pairs (1 byte each)
# • We note where symbolic immediates appear and patch them after addresses
#   are finalized (after appending .data to the end of .text).
# -----------------------------------------------------------------------------
def assemble(progf: str) -> List[int]:
    mem: List[int] = []
    section = None
    labels: Dict[str, int] = {}        # label -> address (within .text)
    data_kv: Dict[str, int] = {}       # name -> constant byte value
    data_addr: Dict[str, int] = {}     # name -> absolute address (resolved later)
    symbolic_refs: Dict[int, str] = {} # mem index -> symbol name (for fix-up)

    with open(progf, "r", encoding="utf-8") as f:
        for raw in f:
            line = strip_comment(raw).strip()
            if not line:
                continue

            # Section selection (.text or .data)
            if line.lower() == ".text":
                section = TEXT
                continue
            if line.lower() == ".data":
                section = DATA
                continue
            if section is None:
                raise AsmError("No section selected. Start with .text or .data")

            # Label handling (only in .text)
            label, rest = split_label(line)
            if label is not None:
                if section != TEXT:
                    raise AsmError(f"Label '{label}' outside .text")
                labels[label] = len(mem)  # address = current program counter
                line = rest.strip()
                if not line:
                    continue  # pure label line

            # Data section: NAME = value
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

            # From here down: .text parsing
            tokens = tokenize(line)
            if not tokens:
                continue

            op = tokens[0].lower()
            args = tokens[1:]

            # Register code helper (A..G,M). Raises clean error for bad names.
            def reg_code(name: str) -> int:
                n = name.upper()
                if n not in reg:
                    raise AsmError(f"Unknown register '{name}'")
                return reg[n]

            # Emit a single byte into the program image
            def emit(byte: int):
                if not (0 <= byte <= 0xFF):
                    raise AsmError(f"Byte out of range: {byte}")
                mem.append(byte)

            # Emit an 8-bit immediate or record a symbolic placeholder for later
            def emit_addr_or_symbol(token: str):
                if is_symbol(token):
                    symbolic_refs[len(mem)] = token[1:]  # strip leading '%'
                    emit(0)  # placeholder; will patch after layout
                else:
                    emit(rich_int(token))

            if op not in inst:
                raise AsmError(f"Unknown instruction '{op}'")

            # --- Encoding by mnemonic -------------------------------------------------
            if op == "ldi":
                # LDI reg, imm8  →  00_010_rrr imm8
                expect_args(op, args, exact=2)
                if args[0].upper() == "M":
                    raise AsmError("ldi to M (memory) is not supported")
                r = reg_code(args[0])
                opcode = (inst[op] & 0b11111000) | r
                emit(opcode)
                emit_addr_or_symbol(args[1])

            elif op in ("push", "pop"):
                # PUSH/POP reg   →  family 00_100_rrr / 00_101_rrr
                expect_args(op, args, exact=1)
                r = reg_code(args[0])
                opcode = (inst[op] & 0b11111000) | r
                emit(opcode)

            elif op == "mov":
                # MOV dst, src   →  10_sss_rrr  (sss=src in [5:3], rrr=dst in [2:0])
                # M (111) denotes memory on either side (maps to your datapath's REG_T)
                expect_args(op, args, exact=2)
                r1 = reg_code(args[0])  # dst
                r2 = reg_code(args[1])  # src
                opcode = (inst[op] & 0b11000111) | (r1 << 3)  # place dst
                opcode = (opcode & 0b11111000) | r2           # place src
                emit(opcode)

            elif op in ("sta", "lda", "jmp", "jz", "jnz", "je", "jne", "jc", "jnc"):
                # STA/ LDA: helper mnemonics that compile to MOV with M involved,
                # but here we emit a dedicated opcode followed by an 8-bit address.
                # Jxx:      each emits opcode then target byte (symbol or literal).
                expect_args(op, args, exact=1)
                emit(inst[op])
                emit_addr_or_symbol(args[0])

            elif op in ("out", "in"):
                # OUT/IN port# (8-bit port number). For OUT we also expect CPU to
                # present the payload on the bus; for IN, the device will drive bus.
                expect_args(op, args, exact=1)
                emit(inst[op])
                emit(rich_int(args[0]))

            else:
                # All other 0-operand instructions (nop, hlt, call, ret, cmp, add, ...)
                expect_args(op, args, exact=0)
                emit(inst[op])

    # -----------------------------------------------------------------------------
    # Layout `.data` after `.text`. Each symbol is a single byte.
    # We capture the absolute address of each symbol and append its value.
    # -----------------------------------------------------------------------------
    for k, v in data_kv.items():
        data_addr[k] = len(mem)  # absolute address where this byte will live
        if not (0 <= v <= 0xFF):
            raise AsmError(f"Data value for {k} out of 8-bit range: {v}")
        mem.append(v)

    # Labels live in `.text`; data symbols live at the tail. Both are valid
    # destinations for `%name` fix-ups.
    data_addr.update(labels)

    # -----------------------------------------------------------------------------
    # Patch symbolic references now that we know all absolute addresses
    # -----------------------------------------------------------------------------
    for idx, name in symbolic_refs.items():
        if name not in data_addr:
            raise AsmError(f"undefined symbol %{name}")
        addr = data_addr[name]
        if not (0 <= addr <= 0xFF):
            raise AsmError(f"Address for %{name} out of 8-bit range: {addr}")
        mem[idx] = addr

    # -----------------------------------------------------------------------------
    # Finalize image: enforce 256-byte size (error on overflow, pad with zeros)
    # -----------------------------------------------------------------------------
    if len(mem) > MEM_SIZE:
        raise AsmError(f"program too large: {len(mem)} bytes > {MEM_SIZE}")
    while len(mem) < MEM_SIZE:
        mem.append(0)

    return mem

# -----------------------------------------------------------------------------
# CLI entry point
# -----------------------------------------------------------------------------
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
    # Print the 256-byte image as hex pairs separated by spaces
    print(" ".join(f"{b:02x}" for b in mem))

if __name__ == "__main__":
    main()
