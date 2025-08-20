.text
ldi a, 0x3C     ; A = 0x3C
ldi b, 0x00     ; B = 0x00
mov a, b        ; expect B = 0x3C, A unchanged
hlt