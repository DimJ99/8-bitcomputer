#!/usr/bin/env bats

function compile_and_run() {
  local asm_file="$1"
  ./asm/asm.py "./tests/${asm_file}" > ./memory.list
  make clean
  make run
}

