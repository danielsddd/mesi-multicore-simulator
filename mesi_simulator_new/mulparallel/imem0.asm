# Serial Matrix Multiplication - Core 0 only
# C[i][j] = sum(A[i][k] * B[k][j]) for k=0..15
# Matrix A: addresses 0x000-0x0FF (256 elements, 16x16)
# Matrix B: addresses 0x100-0x1FF (256 elements, 16x16)
# Matrix C: addresses 0x200-0x2FF (256 elements, 16x16)
# Row-major order: A[i][j] at address base + i*16 + j
#
# Registers:
#   r2 = i (row index, 0-15)
#   r3 = j (column index, 0-15)
#   r4 = k (inner loop index, 0-15)
#   r5 = sum (accumulator for C[i][j])
#   r6 = A[i][k] value
#   r7 = B[k][j] value
#   r8 = temp address calculation
#   r9 = 16 (loop limit and row size)
#   r10 = 0x100 (base of B)
#   r11 = 0x200 (base of C)
#   r12 = temp product

    add $r9, $zero, $imm, 16        # PC=0: r9 = 16
    add $r10, $zero, $imm, 256      # PC=1: r10 = 0x100 (base of B)
    add $r11, $zero, $imm, 512      # PC=2: r11 = 0x200 (base of C)
    add $r2, $zero, $zero, 0        # PC=3: r2 = i = 0
loop_i:
    add $r3, $zero, $zero, 0        # PC=4: r3 = j = 0
loop_j:
    add $r5, $zero, $zero, 0        # PC=5: r5 = sum = 0
    add $r4, $zero, $zero, 0        # PC=6: r4 = k = 0
loop_k:
    # Calculate A[i][k] address = i*16 + k
    mul $r8, $r2, $r9               # PC=7: r8 = i * 16
    add $r8, $r8, $r4               # PC=8: r8 = i*16 + k
    lw $r6, $r8, $zero, 0           # PC=9: r6 = A[i][k]
    # Calculate B[k][j] address = 0x100 + k*16 + j
    mul $r8, $r4, $r9               # PC=10: r8 = k * 16
    add $r8, $r8, $r3               # PC=11: r8 = k*16 + j
    add $r8, $r8, $r10              # PC=12: r8 = 0x100 + k*16 + j
    lw $r7, $r8, $zero, 0           # PC=13: r7 = B[k][j]
    # sum += A[i][k] * B[k][j]
    mul $r12, $r6, $r7              # PC=14: r12 = A[i][k] * B[k][j]
    add $r5, $r5, $r12              # PC=15: sum += r12
    # k++
    add $r4, $r4, $imm, 1           # PC=16: k++
    blt $imm, $r4, $r9, 7           # PC=17: if k < 16, goto loop_k (PC=7)
    add $zero, $zero, $zero, 0      # PC=18: delay slot
    # Store C[i][j] = sum at address 0x200 + i*16 + j
    mul $r8, $r2, $r9               # PC=19: r8 = i * 16
    add $r8, $r8, $r3               # PC=20: r8 = i*16 + j
    add $r8, $r8, $r11              # PC=21: r8 = 0x200 + i*16 + j
    sw $r5, $r8, $zero, 0           # PC=22: C[i][j] = sum
    # j++
    add $r3, $r3, $imm, 1           # PC=23: j++
    blt $imm, $r3, $r9, 5           # PC=24: if j < 16, goto loop_j (PC=5)
    add $zero, $zero, $zero, 0      # PC=25: delay slot
    # i++
    add $r2, $r2, $imm, 1           # PC=26: i++
    blt $imm, $r2, $r9, 4           # PC=27: if i < 16, goto loop_i (PC=4)
    add $zero, $zero, $zero, 0      # PC=28: delay slot
    halt $zero, $zero, $zero, 0     # PC=29: halt
    halt $zero, $zero, $zero, 0     # PC=30: padding
    halt $zero, $zero, $zero, 0     # PC=31
    halt $zero, $zero, $zero, 0     # PC=32
    halt $zero, $zero, $zero, 0     # PC=33