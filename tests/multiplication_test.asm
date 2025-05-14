.text

start:
    ldi M %r      ; load address of r into M
    mov A M       ; A ← mem[M] = r

    ldi B 4
    add           ; A ← A + B

    ldi M %r
    mov M A       ; mem[M] ← A (store back to r)

    ldi M %i
    mov A M
    dec
    ldi M %i
    mov M A

    jnz %start

    ldi M %r
    mov A M
    out 0
    hlt

.data
r = 0
i = 4
