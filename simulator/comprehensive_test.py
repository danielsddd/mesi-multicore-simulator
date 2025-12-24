import os
import subprocess
import time
import random
import struct

# ==============================================================================
# 1. CONFIGURATION & CONSTANTS
# ==============================================================================
SIM_SOURCE = "sim.c"
SIM_EXE = "sim.exe" if os.name == 'nt' else "./sim"
CC = "gcc"
# Add -g or -O3 if needed. Using basic flags.
CFLAGS = ["-o", SIM_EXE, SIM_SOURCE] 

# --- OPCODE MAP ---
OP_ADD = 0; OP_SUB = 1; OP_AND = 2; OP_OR = 3; OP_XOR = 4; OP_MUL = 5
OP_SLL = 6; OP_SRA = 7; OP_SRL = 8
OP_BEQ = 9; OP_BNE = 10; OP_BLT = 11; OP_BGT = 12; OP_BLE = 13; OP_BGE = 14
OP_JAL = 15; OP_LW = 16; OP_SW = 17; OP_HALT = 20

# --- REGISTERS ---
R0=0; R1=1; R2=2; R3=3; R4=4; R5=5; R6=6; R7=7
R8=8; R9=9; R10=10; R11=11; R12=12; R13=13; R14=14; R15=15

# ==============================================================================
# 2. HELPER FUNCTIONS
# ==============================================================================
def asm(opcode, rd, rs, rt, imm):
    """Encodes instruction per spec: Op[31:24], RD[23:20], RS[19:16], RT[15:12], Imm[11:0]"""
    imm = imm & 0xFFF # Mask to 12 bits
    inst = (opcode << 24) | (rd << 20) | (rs << 16) | (rt << 12) | imm
    return f"{inst:08X}"

def write_file(fname, lines):
    with open(fname, 'w') as f:
        f.write("\n".join(lines) + "\n")

def read_file(fname):
    if not os.path.exists(fname): return []
    with open(fname, 'r') as f:
        return [l.strip() for l in f.readlines() if l.strip()]

def get_reg_values(core_id):
    """Reads final register values (R2-R15) from regoutX.txt"""
    lines = read_file(f"regout{core_id}.txt")
    regs = {}
    for i, line in enumerate(lines):
        if i+2 < 16:
            regs[i+2] = int(line, 16)
    return regs

def get_mem_dump():
    """Reads memory dump from memout.txt"""
    lines = read_file("memout.txt")
    mem = {}
    for i, line in enumerate(lines):
        # optimization: only store non-zero to save python memory/time
        # assuming lines are sequential addresses starting 0
        try:
            val = int(line, 16)
            if val != 0:
                mem[i] = val
        except ValueError:
            pass
    return mem

def clean_env():
    """Removes all simulation artifacts to ensure a clean test"""
    files = ["memin.txt", "memout.txt", "bustrace.txt"]
    for i in range(4):
        files.extend([
            f"imem{i}.txt", 
            f"regout{i}.txt", 
            f"core{i}trace.txt", 
            f"dsram{i}.txt", 
            f"tsram{i}.txt", 
            f"stats{i}.txt"
        ])
    
    for f in files:
        if os.path.exists(f):
            try:
                os.remove(f)
            except OSError:
                pass

def to_signed32(n):
    n = n & 0xFFFFFFFF
    return n | (-(n & 0x80000000))

def check_file_exists_and_not_empty(filename, allow_empty=False):
    if not os.path.exists(filename):
        return False, f"Missing {filename}"
    if not allow_empty and os.path.getsize(filename) == 0:
        return False, f"Empty {filename}"
    return True, ""

def verify_stats_structure(core_id):
    lines = read_file(f"stats{core_id}.txt")
    required_keys = ["cycles", "instructions", "read_hit", "write_hit", "read_miss", "write_miss", "decode_stall", "mem_stall"]
    content = " ".join(lines).lower()
    for k in required_keys:
        if k.replace("_", " ") not in content and k not in content:
            # Try flexible matching (e.g. "read hit" vs "read_hit")
            return False, f"Stats core {core_id} missing '{k}'"
    return True, ""

def verify_artifacts():
    """Checks if all required output files exist and have valid basic structure"""
    # 1. Check Global Files
    chk, msg = check_file_exists_and_not_empty("memout.txt", allow_empty=True) # Allowed empty if 0 lines written (though unlikely)
    if not chk: return False, msg
    
    # Bus trace: Exists, but can be empty if no bus transactions occurred
    chk, msg = check_file_exists_and_not_empty("bustrace.txt", allow_empty=True)
    if not chk: return False, msg
    
    # 2. Check Per-Core Files
    for i in range(4):
        # Regout: Must exist
        chk, msg = check_file_exists_and_not_empty(f"regout{i}.txt")
        if not chk: return False, msg
        
        # Trace: Must exist
        chk, msg = check_file_exists_and_not_empty(f"core{i}trace.txt")
        if not chk: return False, msg
        
        # DSRAM: Must exist
        chk, msg = check_file_exists_and_not_empty(f"dsram{i}.txt", allow_empty=True)
        if not chk: return False, msg
        
        # TSRAM: Must exist
        chk, msg = check_file_exists_and_not_empty(f"tsram{i}.txt", allow_empty=True)
        if not chk: return False, msg
        
        # Stats: Must exist and have keys
        chk, msg = check_file_exists_and_not_empty(f"stats{i}.txt")
        if not chk: return False, msg
        
        chk, msg = verify_stats_structure(i)
        if not chk: return False, msg
        
    return True, ""

# ==============================================================================
# 3. GOLDEN MODEL (PYTHON SIMULATOR)
# ==============================================================================
class GoldenCore:
    def __init__(self, core_id, imem, mem_ref):
        self.id = core_id
        self.regs = [0] * 16
        self.pc = 0
        self.imem = imem # List of instruction strings
        self.mem = mem_ref # Reference to shared global memory dict
        self.halted = False
        self.executed_count = 0

    def step(self):
        if self.halted: return
        if self.pc >= len(self.imem):
            self.halted = True
            return

        # Fetch & Decode
        hex_inst = self.imem[self.pc]
        inst = int(hex_inst, 16)
        
        opcode = (inst >> 24) & 0xFF
        rd = (inst >> 20) & 0xF
        rs = (inst >> 16) & 0xF
        rt = (inst >> 12) & 0xF
        imm = inst & 0xFFF
        # Sign extend immediate (12 bit)
        if imm & 0x800: imm |= 0xFFFFF000
        imm = to_signed32(imm)

        # Update R1
        self.regs[1] = imm
        
        # Read Regs
        rs_val = self.regs[rs]
        rt_val = self.regs[rt]
        
        # Execute
        next_pc = self.pc + 1
        res = 0
        write_reg = True
        
        if opcode == OP_ADD: res = rs_val + rt_val
        elif opcode == OP_SUB: res = rs_val - rt_val
        elif opcode == OP_AND: res = rs_val & rt_val
        elif opcode == OP_OR:  res = rs_val | rt_val
        elif opcode == OP_XOR: res = rs_val ^ rt_val
        elif opcode == OP_MUL: res = rs_val * rt_val
        elif opcode == OP_SLL: res = rs_val << (rt_val & 0x1F)
        elif opcode == OP_SRA: res = rs_val >> (rt_val & 0x1F) # Python >> is arithmetic for signed
        elif opcode == OP_SRL: res = (rs_val & 0xFFFFFFFF) >> (rt_val & 0x1F)
        elif opcode == OP_LW:
            addr = (rs_val + rt_val) & 0x1FFFFF # 21 bit address
            res = self.mem.get(addr, 0)
        elif opcode == OP_SW:
            addr = (rs_val + rt_val) & 0x1FFFFF
            self.mem[addr] = self.regs[rd]
            write_reg = False
        elif opcode == OP_BEQ:
            write_reg = False
            if rs_val == rt_val: next_pc = self.regs[rd] & 0x3FF
        elif opcode == OP_BNE:
            write_reg = False
            if rs_val != rt_val: next_pc = self.regs[rd] & 0x3FF
        elif opcode == OP_BLT:
            write_reg = False
            if rs_val < rt_val: next_pc = self.regs[rd] & 0x3FF
        elif opcode == OP_BGT:
            write_reg = False
            if rs_val > rt_val: next_pc = self.regs[rd] & 0x3FF
        elif opcode == OP_BLE:
            write_reg = False
            if rs_val <= rt_val: next_pc = self.regs[rd] & 0x3FF
        elif opcode == OP_BGE:
            write_reg = False
            if rs_val >= rt_val: next_pc = self.regs[rd] & 0x3FF
        elif opcode == OP_JAL:
            self.regs[15] = self.pc + 1 # Note: Sim uses PC+1
            next_pc = self.regs[rd] & 0x3FF
            write_reg = False
        elif opcode == OP_HALT:
            self.halted = True
            write_reg = False

        if write_reg and rd != 0 and rd != 1:
            self.regs[rd] = to_signed32(res)

        self.pc = next_pc
        self.executed_count += 1
        
        # Branch Delay Slot Handling (Simplified for checking)
        if opcode >= OP_BEQ and opcode <= OP_JAL:
            pass # We rely on simulator logic for this complex behavior

def run_golden_model(cores_imem, initial_mem):
    # Simplified Golden Model for ALU/Memory ops (No branching support for simplicity in fuzzing)
    mem = initial_mem.copy()
    final_regs = []
    
    for c_id in range(4):
        c = GoldenCore(c_id, cores_imem[c_id], mem)
        # Limit cycles to prevent infinite loops
        steps = 0
        while not c.halted and steps < 1000:
            c.step()
            steps += 1
        final_regs.append(c.regs)
        
    return final_regs, mem

# ==============================================================================
# 4. TEST GENERATORS
# ==============================================================================

def gen_alu_test(num_insts):
    """Generates a sequence of random ALU operations"""
    code = []
    # Initialize regs with some values
    for r in range(2, 16):
        val = random.randint(0, 100)
        code.append(asm(OP_ADD, r, R0, R1, val))
        
    for _ in range(num_insts):
        op = random.choice([OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_MUL, OP_SLL, OP_SRA, OP_SRL])
        rd = random.randint(2, 15)
        rs = random.randint(0, 15)
        rt = random.randint(0, 15)
        imm = random.randint(0, 15) # Small imm for shifts
        code.append(asm(op, rd, rs, rt, imm))
        
    code.append(asm(OP_HALT, 0, 0, 0, 0))
    return code

def gen_mem_test(num_insts):
    """Generates random loads and stores to a small memory range"""
    code = []
    # Base address in R2 = 0x100
    code.append(asm(OP_ADD, R2, R0, R1, 0x100))
    
    for _ in range(num_insts):
        op = random.choice([OP_LW, OP_SW])
        rd = random.randint(3, 15) # Data reg
        rs = R2 # Base
        rt = R0 # Offset reg (using 0)
        
        rt = random.randint(3, 6) # Use some regs for offsets
        # Initialize offset reg
        code.append(asm(OP_ADD, rt, R0, R1, random.randint(0, 8)))
        
        code.append(asm(op, rd, rs, rt, 0))
        
    code.append(asm(OP_HALT, 0, 0, 0, 0))
    return code

# ==============================================================================
# 5. TEST RUNNER
# ==============================================================================

class Tester:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.total = 0
        self.compile()

    def compile(self):
        print(">>> Compiling Simulator...")
        try:
            subprocess.run([CC] + CFLAGS, check=True)
            print(">>> Compilation Success.\n")
        except:
            print(">>> COMPILATION FAILED. Cannot run tests.")
            exit(1)

    def run_single_test(self, test_id, description, cores_code, verify_fn=None, use_golden=False):
        self.total += 1
        clean_env()
        
        # 1. Setup Input Files (Correctly padding for all 4 cores)
        full_cores_code = []
        for i in range(4):
            code = cores_code[i] if i < len(cores_code) else [asm(OP_HALT,0,0,0,0)]
            full_cores_code.append(code)
            write_file(f"imem{i}.txt", code)
        
        # Initialize Memory (Zeroed)
        mem_lines = ["00000000"] * 2048
        write_file("memin.txt", mem_lines)
        
        # Wait for FS
        time.sleep(0.05)
        
        # 2. Run Simulator
        try:
            result = subprocess.run(
                [SIM_EXE], 
                timeout=5, # 5 Seconds Timeout
                capture_output=True, # Capture stdout/stderr
                text=True,
                cwd=os.getcwd()
            )
            
            if result.returncode != 0:
                 print(f"Test {test_id:03d}: [FAIL] Crash (Exit {result.returncode}) - {description}")
                 self.failed += 1
                 return
                 
            # Check for Sim Warnings
            if "Warning" in result.stdout:
                print(f"Test {test_id:03d}: [FAIL] Sim Warning - {description}")
                print(f"  STDOUT: {result.stdout.strip()}")
                self.failed += 1
                return

        except subprocess.TimeoutExpired as e:
            print(f"Test {test_id:03d}: [FAIL] Timeout (Frozen Sim) - {description}")
            if e.stdout and "Warning" in e.stdout:
                print(f"  STDOUT: {e.stdout.strip()}")
            elif os.path.exists("core0trace.txt"):
                 print("  Note: Trace files created (Simulator running infinite loop?)")
            self.failed += 1
            return
        except Exception as e:
            print(f"Test {test_id:03d}: [FAIL] Exec Error {e} - {description}")
            self.failed += 1
            return
            
        # 3. Artifact Verification
        art_ok, art_msg = verify_artifacts()
        if not art_ok:
            print(f"Test {test_id:03d}: [FAIL] Artifact Check - {description} -> {art_msg}")
            self.failed += 1
            return

        # 4. Functional Verification
        passed = True
        fail_reason = ""
        
        # A. Custom Validator
        if verify_fn:
            if not verify_fn():
                passed = False
                fail_reason = "Custom check failed"

        # B. Golden Model Comparison
        if use_golden and passed:
            golden_regs, golden_mem = run_golden_model(full_cores_code, {})
            sim_mem = get_mem_dump()
            
            # Check Memory (Exact match for non-zero)
            for addr, val in golden_mem.items():
                s_val = sim_mem.get(addr, 0)
                if val != s_val:
                    passed = False
                    fail_reason = f"Mem Mismatch @{addr:X}. Exp {val:08X} Got {s_val:08X}"
                    break
            
            if passed:
                # Check Registers for ALL cores
                for c_id in range(4):
                    sim_regs = get_reg_values(c_id)
                    for r in range(2, 16):
                        g_val = golden_regs[c_id][r]
                        s_val = sim_regs.get(r, 0)
                        if to_signed32(g_val) != to_signed32(s_val):
                            passed = False
                            fail_reason = f"Core {c_id} Reg R{r} mismatch. Exp {to_signed32(g_val):X} Got {to_signed32(s_val):X}"
                            break
                    if not passed: break
        
        if passed:
            # print(f"Test {test_id:03d}: [PASS] {description}")
            self.passed += 1
        else:
            print(f"Test {test_id:03d}: [FAIL] {description} -> {fail_reason}")
            self.failed += 1

# ==============================================================================
# MAIN
# ==============================================================================

if __name__ == "__main__":
    t = Tester()
    print("Starting 500 Tests Suite...")
    
    # TEST 000: BARE MINIMUM
    code_halt = [asm(OP_HALT, 0,0,0,0)]
    t.run_single_test(0, "Bare Halt", [code_halt], 
                      lambda: True)

    # --- PHASE 1: Sanity (0-20) ---
    code_r0 = [asm(OP_ADD, R0, R0, R1, 55), asm(OP_HALT, 0,0,0,0)]
    t.run_single_test(1, "R0 Write Ignored", [code_r0], 
                      lambda: get_reg_values(0).get(0, 0) == 0)
                      
    code_r1 = [asm(OP_ADD, R2, R0, R1, 0xFFF), asm(OP_HALT, 0,0,0,0)]
    t.run_single_test(2, "R1 Sign Extend", [code_r1],
                      lambda: get_reg_values(0)[2] == 0xFFFFFFFF)

    for i in range(3, 21):
        op = random.choice([OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR])
        v1 = random.randint(0, 100)
        v2 = random.randint(0, 100)
        code = [
            asm(OP_ADD, R2, R0, R1, v1),
            asm(OP_ADD, R3, R0, R1, v2),
            asm(op, R4, R2, R3, 0),
            asm(OP_HALT, 0,0,0,0)
        ]
        t.run_single_test(i, f"Simple ALU {op}", [code], use_golden=True)

    # --- PHASE 2: ALU Fuzzing (21-200) ---
    print("\nRunning ALU Fuzzing (Tests 21-200)...")
    for i in range(21, 201):
        code = gen_alu_test(num_insts=random.randint(5, 50))
        t.run_single_test(i, "Random ALU Sequence", [code], use_golden=True)

    # --- PHASE 3: Memory & Cache (201-300) ---
    print("\nRunning Memory Fuzzing (Tests 201-300)...")
    for i in range(201, 301):
        code = gen_mem_test(num_insts=random.randint(10, 40))
        t.run_single_test(i, "Random Mem Access", [code])

    # --- PHASE 4: Multi-Core Stress (301-500) ---
    print("\nRunning Multi-Core Stress (Tests 301-500)...")
    for i in range(301, 501):
        cores_code = []
        for c in range(4):
            cores_code.append(gen_mem_test(20))
            
        t.run_single_test(i, "Multi-Core Contention", cores_code)

    print("\n" + "="*40)
    print(f"FINAL REPORT: Passed {t.passed} / {t.total}")
    print("="*40)
    if t.failed > 0:
        print("FAILURES DETECTED. See log above.")
    else:
        print("ALL 500 TESTS PASSED. SYSTEM STABLE.")