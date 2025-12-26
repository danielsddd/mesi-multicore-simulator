#  Core 0
# Computes rows 0-3 of result matrix C
# C[i][j] = sum(A[i][k] * B[k][j]) for k=0..15
# Matrix A: 0x000-0x0FF, Matrix B: 0x100-0x1FF, Matrix C: 0x200-0x2FF
#
# Registers:
#   r2 = row start (0 for core 0)
#   r3 = j (column, 0-15)
#   r4 = k (inner loop, 0-15)
#   r5 = sum
#   r6 = A[row][k]
#   r7 = B[k][j]
#   r8 = temp address
#   r9 = 16
#   r10 = 0x100 (base of B)
#   r11 = 0x200 (base of C)
#   r12 = temp product
#   r13 = row end (4 for core 0)
#   r14 = current row

    add $r9, $zero, $imm, 16        # PC=0: r9 = 16
    add $r10, $zero, $imm, 256      # PC=1: r10 = 0x100
    add $r11, $zero, $imm, 512      # PC=2: r11 = 0x200
    add $r2, $zero, $imm, 0         # PC=3: r2 = 0 (start row for core 0)
    add $r13, $zero, $imm, 4        # PC=4: r13 = 4 (end row, exclusive)
    add $r14, $r2, $zero, 0         # PC=5: r14 = current row = start
loop_i:
    add $r3, $zero, $zero, 0        # PC=6: j = 0
loop_j:
    add $r5, $zero, $zero, 0        # PC=7: sum = 0
    add $r4, $zero, $zero, 0        # PC=8: k = 0
loop_k:
    # A[r14][k] at r14*16 + k
    mul $r8, $r14, $r9              # PC=9: r8 = row * 16
    add $r8, $r8, $r4               # PC=10: r8 = row*16 + k
    lw $r6, $r8, $zero, 0           # PC=11: r6 = A[row][k]
    # B[k][j] at 0x100 + k*16 + j
    mul $r8, $r4, $r9               # PC=12: r8 = k * 16
    add $r8, $r8, $r3               # PC=13: r8 = k*16 + j
    add $r8, $r8, $r10              # PC=14: r8 += 0x100
    lw $r7, $r8, $zero, 0           # PC=15: r7 = B[k][j]
    # sum += A * B
    mul $r12, $r6, $r7              # PC=16: r12 = A * B
    add $r5, $r5, $r12              # PC=17: sum += r12
    # k++
    add $r4, $r4, $imm, 1           # PC=18: k++
    blt $imm, $r4, $r9, 9           # PC=19: if k < 16, goto loop_k
    add $zero, $zero, $zero, 0      # PC=20: delay slot
    # Store C[row][j]
    mul $r8, $r14, $r9              # PC=21: r8 = row * 16
    add $r8, $r8, $r3               # PC=22: r8 = row*16 + j
    add $r8, $r8, $r11              # PC=23: r8 += 0x200
    sw $r5, $r8, $zero, 0           # PC=24: C[row][j] = sum
    # j++
    add $r3, $r3, $imm, 1           # PC=25: j++
    blt $imm, $r3, $r9, 7           # PC=26: if j < 16, goto loop_j
    add $zero, $zero, $zero, 0      # PC=27: delay slot
    # row++
    add $r14, $r14, $imm, 1         # PC=28: row++
    blt $imm, $r14, $r13, 6         # PC=29: if row < end, goto loop_i
    add $zero, $zero, $zero, 0      # PC=30: delay slot
    halt $zero, $zero, $zero, 0     # PC=31: halt
    halt $zero, $zero, $zero, 0     # PC=32
    halt $zero, $zero, $zero, 0     # PC=33
    halt $zero, $zero, $zero, 0     # PC=34
    halt $zero, $zero, $zero, 0     # PC=35