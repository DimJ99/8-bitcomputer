        .text
        ldi B, 4        ; multiplicand
        ldi D, 4        ; multiplier / loop counter
        ldi C, 0        ; result

loop:
        ; C := C + B  (ADD is A:=A+B, so shuttle via A)
        mov A, C
        add
        mov C, A

        ; D := D - 1  (DEC works on A implicitly; shuttle D through A)
        mov A, D
        dec
        mov D, A

        jnz %loop       ; keep going while D != 0 (flags from DEC)
done:
        hlt