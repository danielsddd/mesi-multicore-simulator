# Core 0
# All 4 cores increment counter at address 0, taking turns (0->1->2->3->0...)
# Each core does 128 increments, total = 512
# Address 0: counter (shared)
# Address 1: turn (0-3, indicates which core's turn)
#
# Registers:
#   r2 = core_id (0)
#   r3 = iteration count (0 to 127)
#   r4 = turn value from memory
#   r5 = counter value
#   r6 = temp
#   r7 = 128 (max iterations)
#   r8 = 4 (for modulo check)
#   r9 = 512 (conflict miss address)

    add $r2, $zero, $imm, 0         # PC=0: r2 = 0 (core id)
    add $r3, $zero, $zero, 0        # PC=1: r3 = 0 (iteration count)
    add $r7, $zero, $imm, 128       # PC=2: r7 = 128
    add $r8, $zero, $imm, 4         # PC=3: r8 = 4
    add $r9, $zero, $imm, 512       # PC=4: r9 = 512 (for conflict miss)
wait_turn:
    lw $r4, $zero, $imm, 1          # PC=5: r4 = mem[1] (turn)
    bne $imm, $r4, $r2, 5           # PC=6: if turn != core_id, goto 5
    add $zero, $zero, $zero, 0      # PC=7: delay slot (nop)
    lw $r5, $zero, $imm, 0          # PC=8: r5 = mem[0] (counter)
    add $r5, $r5, $imm, 1           # PC=9: r5 = r5 + 1
    sw $r5, $zero, $imm, 0          # PC=10: mem[0] = r5
    add $r4, $r4, $imm, 1           # PC=11: r4 = turn + 1
    blt $imm, $r4, $r8, 15          # PC=12: if r4 < 4, goto 15 (skip reset)
    add $zero, $zero, $zero, 0      # PC=13: delay slot
    add $r4, $zero, $zero, 0        # PC=14: r4 = 0 (reset turn to 0)
skip_mod:
    sw $r4, $zero, $imm, 1          # PC=15: mem[1] = new turn
    add $r3, $r3, $imm, 1           # PC=16: r3++ (iteration count)
    blt $imm, $r3, $r7, 5           # PC=17: if r3 < 128, goto wait_turn
    add $zero, $zero, $zero, 0      # PC=18: delay slot
    lw $r6, $r9, $zero, 0           # PC=19: read from addr 512 (conflict miss forces writeback)
    halt $zero, $zero, $zero, 0     # PC=20: halt
    halt $zero, $zero, $zero, 0     # PC=21: padding
    halt $zero, $zero, $zero, 0     # PC=22: padding
    halt $zero, $zero, $zero, 0     # PC=23: padding
    halt $zero, $zero, $zero, 0     # PC=24: padding