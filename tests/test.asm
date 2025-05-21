.text 
ldi A 5       ; A = 5
ldi B 7       ; B = 7
add A B       ; A = A + B = 12
mov M A 0xF0  ; mem[0xF0] = A (12)
hlt           ; stop
