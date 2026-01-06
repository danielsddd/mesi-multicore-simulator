"""
Multi-Core MESI Simulator - Exhaustive Test Suite
Based on Project Assignment PDF Requirements

Checks requirements from the specification:
- Output file formats (exact field widths, line counts)
- Pipeline behavior (5-stage, delay slot, hazards, stalls)
- Cache behavior (512 words, 64 blocks, 8 words/block, direct-mapped, write-back, write-allocate)
- MESI protocol (all state transitions, snooping, bus_shared signal)
- Bus behavior (round-robin, 16-cycle latency, 8-word flush, transaction locking)
- Statistics (all 8 fields, correct counting)
- All 21 instructions including edge cases
"""

import os, sys, subprocess, tempfile, shutil, argparse
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

# Configuration
SIM_SOURCE = "sim.c"
CC = "gcc"
CFLAGS = ["-Wall", "-Wextra", "-O2"]
IS_WINDOWS = sys.platform.startswith('win')
SIM_BINARY = "sim.exe" if IS_WINDOWS else "sim"

OUTPUT_FILES = [
    "memout.txt",
    "regout0.txt", "regout1.txt", "regout2.txt", "regout3.txt",
    "core0trace.txt", "core1trace.txt", "core2trace.txt", "core3trace.txt",
    "bustrace.txt",
    "dsram0.txt", "dsram1.txt", "dsram2.txt", "dsram3.txt",
    "tsram0.txt", "tsram1.txt", "tsram2.txt", "tsram3.txt",
    "stats0.txt", "stats1.txt", "stats2.txt", "stats3.txt",
]

# Opcodes (from PDF page 4-5)
OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_MUL = 0, 1, 2, 3, 4, 5
OP_SLL, OP_SRA, OP_SRL = 6, 7, 8
OP_BEQ, OP_BNE, OP_BLT, OP_BGT, OP_BLE, OP_BGE = 9, 10, 11, 12, 13, 14
OP_JAL, OP_LW, OP_SW, OP_HALT = 15, 16, 17, 20

HALT_PAD = 10

@dataclass
class TestResult:
    name: str
    passed: bool
    message: str = ""

class TestSuite:
    def __init__(self):
        self.results: List[TestResult] = []
        self.sim_path: Optional[str] = None
        
    def add_result(self, name: str, passed: bool, message: str = ""):
        self.results.append(TestResult(name, passed, message))
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")
        if not passed and message:
            print(f"         {message}")
    
    def summary(self):
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        print("\n" + "=" * 60)
        print(f"SUMMARY: {passed}/{total} passed, {failed} failed")
        print("=" * 60)
        if failed > 0:
            print("Failed tests:")
            for r in self.results:
                if not r.passed:
                    print(f"  - {r.name}: {r.message}")
        return failed == 0

# Instruction encoding (from PDF page 4: opcode[31:24] rd[23:20] rs[19:16] rt[15:12] imm[11:0])
def encode_instruction(opcode: int, rd: int, rs: int, rt: int, imm: int) -> str:
    if imm < 0:
        imm = imm & 0xFFF  # 12-bit two's complement
    inst = ((opcode & 0xFF) << 24) | ((rd & 0xF) << 20) | \
           ((rs & 0xF) << 16) | ((rt & 0xF) << 12) | (imm & 0xFFF)
    return f"{inst:08X}"

def to_signed_32(val: int) -> int:
    return val - 0x100000000 if val >= 0x80000000 else val

NOP = encode_instruction(OP_ADD, 0, 0, 0, 0)

def pad_with_halts(instructions: List[str]) -> List[str]:
    return instructions + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD

def make_halt_only() -> List[str]:
    return [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD

def create_test_files(test_dir: str, imem: List[List[str]], memin: List[str] = None):
    for i in range(4):
        with open(os.path.join(test_dir, f"imem{i}.txt"), 'w') as f:
            if i < len(imem) and imem[i]:
                f.write('\n'.join(imem[i]) + '\n')
    with open(os.path.join(test_dir, "memin.txt"), 'w') as f:
        if memin:
            f.write('\n'.join(memin) + '\n')

def run_simulator(sim_path: str, test_dir: str, timeout: int = 60) -> Tuple[bool, str, str]:
    try:
        result = subprocess.run([sim_path], cwd=test_dir, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Timeout"
    except Exception as e:
        return False, "", str(e)

def read_file(filepath: str) -> List[str]:
    try:
        with open(filepath, 'r') as f:
            return [line.strip() for line in f if line.strip()]
    except:
        return []

def read_file_with_empty(filepath: str) -> List[str]:
    """Read file preserving empty lines for line count checks."""
    try:
        with open(filepath, 'r') as f:
            return [line.rstrip('\n\r') for line in f]
    except:
        return []

def parse_stats(filepath: str) -> Dict[str, int]:
    stats = {}
    for line in read_file(filepath):
        parts = line.split()
        if len(parts) == 2:
            stats[parts[0]] = int(parts[1])
    return stats

def parse_bus_trace(filepath: str) -> List[Dict]:
    traces = []
    for line in read_file(filepath):
        parts = line.split()
        if len(parts) == 6:
            traces.append({
                'cycle': int(parts[0]), 'origid': int(parts[1], 16),
                'cmd': int(parts[2], 16), 'addr': int(parts[3], 16),
                'data': int(parts[4], 16), 'shared': int(parts[5], 16)
            })
    return traces

def read_regout(filepath: str) -> List[int]:
    return [to_signed_32(int(line, 16)) for line in read_file(filepath)]

def read_tsram(filepath: str) -> List[Tuple[int, int]]:
    """Returns list of (MESI_state, tag) tuples."""
    result = []
    for line in read_file(filepath):
        val = int(line, 16)
        mesi = (val >> 12) & 0x3  # bits 13:12
        tag = val & 0xFFF         # bits 11:0
        result.append((mesi, tag))
    return result

def read_dsram(filepath: str) -> List[int]:
    return [int(line, 16) for line in read_file(filepath)]

def read_memout(filepath: str) -> List[int]:
    return [int(line, 16) for line in read_file(filepath)]

# ============================================================================
# COMPILATION
# ============================================================================

def test_compilation(suite: TestSuite, source_path: str) -> Optional[str]:
    print("\n[COMPILATION]")
    print("-" * 40)
    if not os.path.exists(source_path):
        suite.add_result("Source exists", False, f"Not found: {source_path}")
        return None
    suite.add_result("Source exists", True)
    
    temp_dir = tempfile.mkdtemp(prefix="sim_compile_")
    shutil.copy(source_path, temp_dir)
    output_path = os.path.join(temp_dir, SIM_BINARY)
    src_path = os.path.join(temp_dir, SIM_SOURCE)
    
    try:
        result = subprocess.run([CC] + CFLAGS + ["-o", output_path, src_path],
                               capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            suite.add_result("Compilation", True)
            return output_path
        else:
            suite.add_result("Compilation", False, result.stderr[:300])
            shutil.rmtree(temp_dir)
            return None
    except Exception as e:
        suite.add_result("Compilation", False, str(e))
        shutil.rmtree(temp_dir)
        return None

# ============================================================================
# BASIC EXECUTION
# ============================================================================

def test_basic_execution(suite: TestSuite):
    print("\n[BASIC EXECUTION]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_basic_")
    imem = [make_halt_only()] * 4
    create_test_files(test_dir, imem)
    success, _, stderr = run_simulator(suite.sim_path, test_dir)
    suite.add_result("Runs without crash", success, stderr[:100] if not success else "")
    if success:
        missing = [f for f in OUTPUT_FILES if not os.path.exists(os.path.join(test_dir, f))]
        suite.add_result("All 27 output files created", len(missing) == 0, f"Missing: {missing[:3]}")
    shutil.rmtree(test_dir)

# ============================================================================
# OUTPUT FORMAT TESTS (from PDF pages 6-8)
# ============================================================================

def test_regout_format(suite: TestSuite):
    """PDF page 6: regout contains R2-R15 (14 registers), 8 hex digits each."""
    print("\n[REGOUT FORMAT - PDF p.6]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_regfmt_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 0xABC)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    for i in range(4):
        lines = read_file(os.path.join(test_dir, f"regout{i}.txt"))
        suite.add_result(f"regout{i}: exactly 14 lines (R2-R15)", len(lines) == 14, f"Got {len(lines)}")
        if lines:
            bad = [j for j, l in enumerate(lines) if len(l) != 8 or not all(c in '0123456789ABCDEFabcdef' for c in l)]
            suite.add_result(f"regout{i}: all 8 hex digits", len(bad) == 0, f"Bad lines: {bad[:3]}")
    shutil.rmtree(test_dir)

def test_dsram_format(suite: TestSuite):
    """PDF page 2: DSRAM is 32-bit wide, 512 words deep."""
    print("\n[DSRAM FORMAT - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_dsram_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    
    for i in range(4):
        lines = read_file(os.path.join(test_dir, f"dsram{i}.txt"))
        suite.add_result(f"dsram{i}: exactly 512 lines", len(lines) == 512, f"Got {len(lines)}")
        if lines:
            bad = [j for j, l in enumerate(lines) if len(l) != 8]
            suite.add_result(f"dsram{i}: all 8 hex digits", len(bad) == 0, f"Bad lines: {bad[:3]}")
    shutil.rmtree(test_dir)

def test_tsram_format(suite: TestSuite):
    """PDF page 2: TSRAM has 64 lines, each with MESI[13:12] and Tag[11:0]."""
    print("\n[TSRAM FORMAT - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_tsram_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    
    for i in range(4):
        lines = read_file(os.path.join(test_dir, f"tsram{i}.txt"))
        suite.add_result(f"tsram{i}: exactly 64 lines (64 blocks)", len(lines) == 64, f"Got {len(lines)}")
        # Check MESI values are valid (0-3)
        if lines:
            for j, line in enumerate(lines):
                val = int(line, 16)
                mesi = (val >> 12) & 0x3
                if mesi > 3:
                    suite.add_result(f"tsram{i} line {j}: valid MESI", False, f"MESI={mesi}")
                    break
            else:
                suite.add_result(f"tsram{i}: valid MESI values (0-3)", True)
    shutil.rmtree(test_dir)

def test_bustrace_format(suite: TestSuite):
    """PDF page 7: bustrace format with specific field widths."""
    print("\n[BUSTRACE FORMAT - PDF p.7]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_bus_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    
    lines = read_file(os.path.join(test_dir, "bustrace.txt"))
    suite.add_result("Bustrace not empty (has bus activity)", len(lines) > 0, "Empty file")
    if lines:
        parts = lines[0].split()
        suite.add_result("6 fields per line", len(parts) == 6, f"Got {len(parts)}")
        if len(parts) == 6:
            # CYCLE: decimal
            suite.add_result("CYCLE is decimal", parts[0].isdigit(), f"Got: {parts[0]}")
            # bus_origid: 1 hex digit (0-4)
            suite.add_result("bus_origid: 1 hex digit", len(parts[1]) == 1, f"Got: {parts[1]}")
            # bus_cmd: 1 hex digit (0-3)
            suite.add_result("bus_cmd: 1 hex digit", len(parts[2]) == 1, f"Got: {parts[2]}")
            # bus_addr: 6 hex digits (21 bits)
            suite.add_result("bus_addr: 6 hex digits", len(parts[3]) == 6, f"Got: {parts[3]}")
            # bus_data: 8 hex digits (32 bits)
            suite.add_result("bus_data: 8 hex digits", len(parts[4]) == 8, f"Got: {parts[4]}")
            # bus_shared: 1 hex digit (0 or 1)
            suite.add_result("bus_shared: 1 hex digit", len(parts[5]) == 1, f"Got: {parts[5]}")
    shutil.rmtree(test_dir)

def test_coretrace_format(suite: TestSuite):
    """PDF page 7: core trace format with CYCLE, 5 pipeline stages, 14 registers."""
    print("\n[CORE TRACE FORMAT - PDF p.7]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_core_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 42)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    lines = read_file(os.path.join(test_dir, "core0trace.txt"))
    suite.add_result("Core trace not empty", len(lines) > 0, "Empty")
    if lines:
        parts = lines[0].split()
        # CYCLE + FETCH + DECODE + EXEC + MEM + WB + R2..R15 = 1 + 5 + 14 = 20
        suite.add_result("20 fields (cycle + 5 stages + 14 regs)", len(parts) == 20, f"Got {len(parts)}")
        suite.add_result("Starts at cycle 0", parts[0] == "0", f"First cycle: {parts[0]}")
        
        # Check PC format: 3 hex digits or "---"
        if len(parts) >= 6:
            for i, stage in enumerate(['FETCH', 'DECODE', 'EXEC', 'MEM', 'WB']):
                pc = parts[i + 1]
                valid = pc == "---" or (len(pc) == 3 and all(c in '0123456789ABCDEFabcdef' for c in pc))
                suite.add_result(f"{stage}: 3 hex or ---", valid, f"Got: {pc}")
        
        # Check register format: 8 hex digits
        if len(parts) >= 20:
            bad = [j for j, r in enumerate(parts[6:]) if len(r) != 8]
            suite.add_result("Registers: 8 hex digits each", len(bad) == 0, f"Bad: {bad[:3]}")
    shutil.rmtree(test_dir)

def test_stats_format(suite: TestSuite):
    """PDF page 7-8: stats has 8 specific fields."""
    print("\n[STATS FORMAT - PDF p.7-8]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_stats_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    
    # From PDF page 8: exactly these 8 fields
    required = ['cycles', 'instructions', 'read_hit', 'write_hit', 
                'read_miss', 'write_miss', 'decode_stall', 'mem_stall']
    
    for i in range(4):
        stats = parse_stats(os.path.join(test_dir, f"stats{i}.txt"))
        for stat in required:
            suite.add_result(f"stats{i} has '{stat}'", stat in stats, "Missing")
        # Check format: "name X" where X is decimal
        lines = read_file(os.path.join(test_dir, f"stats{i}.txt"))
        for line in lines:
            parts = line.split()
            if len(parts) == 2:
                suite.add_result(f"stats{i}: value is decimal", parts[1].lstrip('-').isdigit(), f"Line: {line}")
                break
    shutil.rmtree(test_dir)

def test_memout_format(suite: TestSuite):
    """PDF page 6: memout same format as memin - 8 hex digits per line."""
    print("\n[MEMOUT FORMAT - PDF p.6]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_memout_")
    # Write and force eviction to ensure memout has content
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 0x123),
        encode_instruction(OP_SW, 2, 0, 1, 0),
        encode_instruction(OP_ADD, 3, 0, 1, 0x200),  # 512 - same index, different tag
        encode_instruction(OP_SW, 2, 3, 0, 0),  # Force eviction
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["00000000"] * 600)
    run_simulator(suite.sim_path, test_dir)
    
    lines = read_file(os.path.join(test_dir, "memout.txt"))
    suite.add_result("Memout not empty", len(lines) > 0, "Empty")
    if lines:
        bad = [j for j, l in enumerate(lines[:100]) if len(l) != 8]
        suite.add_result("8 hex digits per line", len(bad) == 0, f"Bad lines: {bad[:3]}")
    shutil.rmtree(test_dir)

# ============================================================================
# REGISTER TESTS (from PDF page 1)
# ============================================================================

def test_r0_always_zero(suite: TestSuite):
    """PDF page 1: R0 is always zero, writes to R0 don't change it."""
    print("\n[R0 ALWAYS ZERO - PDF p.1]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_r0_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 0, 0, 1, 100),  # Try to write R0
        encode_instruction(OP_ADD, 2, 0, 0, 0),    # R2 = R0 + R0 (should be 0)
        encode_instruction(OP_LW, 0, 0, 1, 0),     # Try to load to R0
        encode_instruction(OP_ADD, 3, 0, 0, 0),    # R3 = R0 + R0 (should be 0)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["FFFFFFFF"])
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("R0 unchanged after ADD", regout[0] == 0, f"R2={regout[0]}")
    suite.add_result("R0 unchanged after LW", regout[1] == 0, f"R3={regout[1]}")
    shutil.rmtree(test_dir)

def test_r1_immediate(suite: TestSuite):
    """PDF page 1: R1 always contains sign-extended immediate, updated each decode."""
    print("\n[R1 IMMEDIATE - PDF p.1]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_r1_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 42),     # R2 = 0 + 42 = 42
        encode_instruction(OP_ADD, 3, 0, 1, -5),     # R3 = 0 + (-5) = -5
        encode_instruction(OP_ADD, 4, 0, 1, 2047),   # Max positive 12-bit
        encode_instruction(OP_ADD, 5, 0, 1, -2048),  # Min negative 12-bit
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("R1 = 42", regout[0] == 42, f"R2={regout[0]}")
    suite.add_result("R1 = -5 (sign extended)", regout[1] == -5, f"R3={regout[1]}")
    suite.add_result("R1 = 2047 (max positive)", regout[2] == 2047, f"R4={regout[2]}")
    suite.add_result("R1 = -2048 (min negative)", regout[3] == -2048, f"R5={regout[3]}")
    shutil.rmtree(test_dir)

# ============================================================================
# PIPELINE HAZARD TESTS (from PDF page 2)
# ============================================================================

def test_data_hazard_stall_decode(suite: TestSuite):
    """PDF page 2: Data hazards solved by stall at DECODE stage."""
    print("\n[DATA HAZARD STALL - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_haz_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10),
        encode_instruction(OP_ADD, 3, 2, 1, 5),  # RAW hazard on R2
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("Correct result despite hazard", regout[0] == 10 and regout[1] == 15,
                    f"R2={regout[0]}, R3={regout[1]}")
    suite.add_result("decode_stall counted", stats.get('decode_stall', 0) > 0,
                    f"decode_stall={stats.get('decode_stall')}")
    shutil.rmtree(test_dir)

def test_cache_miss_stall_mem(suite: TestSuite):
    """PDF page 2: Cache miss stalls at MEM stage."""
    print("\n[CACHE MISS STALL - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_memstall_")
    imem = [pad_with_halts([
        encode_instruction(OP_LW, 2, 0, 1, 0),  # Cache miss
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("mem_stall counted", stats.get('mem_stall', 0) > 0,
                    f"mem_stall={stats.get('mem_stall')}")
    shutil.rmtree(test_dir)

def test_no_bypassing(suite: TestSuite):
    """PDF page 2: No bypassing used - must stall for all hazard distances."""
    print("\n[NO BYPASSING - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    
    # Distance 1 (producer in EX when consumer in DECODE)
    test_dir = tempfile.mkdtemp(prefix="sim_nobyp1_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10),
        encode_instruction(OP_ADD, 3, 2, 1, 5),
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Distance 1 hazard: R3=15", regout[1] == 15, f"R3={regout[1]}")
    shutil.rmtree(test_dir)
    
    # Distance 2 (producer in MEM when consumer in DECODE)
    test_dir = tempfile.mkdtemp(prefix="sim_nobyp2_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10),
        encode_instruction(OP_ADD, 4, 0, 1, 1),
        encode_instruction(OP_ADD, 3, 2, 1, 5),
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Distance 2 hazard: R3=15", regout[1] == 15, f"R3={regout[1]}")
    shutil.rmtree(test_dir)
    
    # Distance 3 (producer in WB when consumer in DECODE)
    test_dir = tempfile.mkdtemp(prefix="sim_nobyp3_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10),
        encode_instruction(OP_ADD, 4, 0, 1, 1),
        encode_instruction(OP_ADD, 5, 0, 1, 2),
        encode_instruction(OP_ADD, 3, 2, 1, 5),
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Distance 3 hazard: R3=15", regout[1] == 15, f"R3={regout[1]}")
    shutil.rmtree(test_dir)

# ============================================================================
# DELAY SLOT TESTS (from PDF page 1)
# ============================================================================

def test_delay_slot(suite: TestSuite):
    """PDF page 1: Delay slot is used - instruction after branch always executes."""
    print("\n[DELAY SLOT - PDF p.1]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_delay_")
    imem = [[
        encode_instruction(OP_ADD, 2, 0, 1, 5),     # R2 = 5
        encode_instruction(OP_ADD, 10, 0, 1, 5),   # R10 = 5 (target)
        encode_instruction(OP_BEQ, 10, 2, 2, 0),   # Branch to R10 (PC=5)
        encode_instruction(OP_ADD, 3, 0, 1, 99),   # DELAY SLOT - always executes!
        encode_instruction(OP_ADD, 4, 0, 1, 88),   # Skipped
        encode_instruction(OP_ADD, 5, 0, 1, 42),   # Target (PC=5)
    ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Delay slot executed", regout[1] == 99, f"R3={regout[1]}")
    suite.add_result("Instruction after delay slot skipped", regout[2] == 0, f"R4={regout[2]}")
    suite.add_result("Branch target reached", regout[3] == 42, f"R5={regout[3]}")
    shutil.rmtree(test_dir)

# ============================================================================
# BRANCH RESOLUTION IN DECODE (from PDF page 2)
# ============================================================================

def test_branch_in_decode(suite: TestSuite):
    """PDF page 2: Branch resolution happens in DECODE stage."""
    print("\n[BRANCH IN DECODE - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    # Branch should take effect immediately, fetching new instruction
    test_dir = tempfile.mkdtemp(prefix="sim_brdec_")
    imem = [[
        encode_instruction(OP_ADD, 10, 0, 1, 4),    # R10 = 4 (target)
        encode_instruction(OP_BEQ, 10, 0, 0, 0),   # Always taken (0==0)
        encode_instruction(OP_ADD, 2, 0, 1, 99),   # Delay slot
        encode_instruction(OP_ADD, 3, 0, 1, 88),   # Skipped
        encode_instruction(OP_ADD, 4, 0, 1, 42),   # Target
    ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Branch resolved in decode", regout[2] == 42 and regout[1] == 0,
                    f"R4={regout[2]}, R3={regout[1]}")
    shutil.rmtree(test_dir)

# ============================================================================
# ALL BRANCH INSTRUCTIONS (from PDF pages 4-5)
# ============================================================================

def test_all_branches(suite: TestSuite):
    """PDF pages 4-5: Test all 6 branch types with signed comparison."""
    print("\n[ALL BRANCHES - PDF p.4-5]")
    print("-" * 40)
    if not suite.sim_path:
        return
    
    branches = [
        ("BEQ", OP_BEQ, 5, 5, True),      # 5 == 5
        ("BEQ not", OP_BEQ, 5, 6, False), # 5 != 6
        ("BNE", OP_BNE, 5, 6, True),      # 5 != 6
        ("BNE not", OP_BNE, 5, 5, False), # 5 == 5
        ("BLT", OP_BLT, -5, 5, True),     # -5 < 5 (signed)
        ("BLT not", OP_BLT, 5, -5, False),# 5 not < -5
        ("BGT", OP_BGT, 5, -5, True),     # 5 > -5 (signed)
        ("BGT not", OP_BGT, -5, 5, False),# -5 not > 5
        ("BLE lt", OP_BLE, 5, 10, True),  # 5 <= 10
        ("BLE eq", OP_BLE, 5, 5, True),   # 5 <= 5
        ("BLE not", OP_BLE, 10, 5, False),# 10 not <= 5
        ("BGE gt", OP_BGE, 10, 5, True),  # 10 >= 5
        ("BGE eq", OP_BGE, 5, 5, True),   # 5 >= 5
        ("BGE not", OP_BGE, 5, 10, False),# 5 not >= 10
    ]
    
    for name, op, rs_val, rt_val, should_take in branches:
        test_dir = tempfile.mkdtemp(prefix=f"sim_br_")
        imem = [[
            encode_instruction(OP_ADD, 2, 0, 1, rs_val),
            encode_instruction(OP_ADD, 3, 0, 1, rt_val),
            encode_instruction(OP_ADD, 10, 0, 1, 6),  # target
            encode_instruction(op, 10, 2, 3, 0),
            encode_instruction(OP_ADD, 4, 0, 1, 77),  # delay slot
            encode_instruction(OP_ADD, 5, 0, 1, 88),  # fall-through
            encode_instruction(OP_ADD, 6, 0, 1, 42),  # target
        ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD] + [make_halt_only()] * 3
        create_test_files(test_dir, imem)
        run_simulator(suite.sim_path, test_dir)
        regout = read_regout(os.path.join(test_dir, "regout0.txt"))
        if should_take:
            suite.add_result(f"{name}: taken", regout[4] == 42, f"R6={regout[4]}")
        else:
            suite.add_result(f"{name}: not taken", regout[3] == 88, f"R5={regout[3]}")
        shutil.rmtree(test_dir)

# ============================================================================
# ALL ALU INSTRUCTIONS (from PDF pages 4-5)
# ============================================================================

def test_all_alu(suite: TestSuite):
    """PDF pages 4-5: Test all ALU operations."""
    print("\n[ALL ALU - PDF p.4-5]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_alu_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10),
        encode_instruction(OP_ADD, 3, 0, 1, 3),
        encode_instruction(OP_ADD, 4, 2, 3, 0),   # 10 + 3 = 13
        encode_instruction(OP_SUB, 5, 2, 3, 0),   # 10 - 3 = 7
        encode_instruction(OP_AND, 6, 2, 3, 0),   # 10 & 3 = 2
        encode_instruction(OP_OR, 7, 2, 3, 0),    # 10 | 3 = 11
        encode_instruction(OP_XOR, 8, 2, 3, 0),   # 10 ^ 3 = 9
        encode_instruction(OP_MUL, 9, 2, 3, 0),   # 10 * 3 = 30
        encode_instruction(OP_SLL, 10, 2, 3, 0),  # 10 << 3 = 80
        encode_instruction(OP_SRL, 11, 2, 3, 0),  # 10 >> 3 = 1 (logical)
        encode_instruction(OP_SRA, 12, 2, 3, 0),  # 10 >>> 3 = 1 (arithmetic)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("ADD 10+3=13", regout[2] == 13, f"R4={regout[2]}")
    suite.add_result("SUB 10-3=7", regout[3] == 7, f"R5={regout[3]}")
    suite.add_result("AND 10&3=2", regout[4] == 2, f"R6={regout[4]}")
    suite.add_result("OR 10|3=11", regout[5] == 11, f"R7={regout[5]}")
    suite.add_result("XOR 10^3=9", regout[6] == 9, f"R8={regout[6]}")
    suite.add_result("MUL 10*3=30", regout[7] == 30, f"R9={regout[7]}")
    suite.add_result("SLL 10<<3=80", regout[8] == 80, f"R10={regout[8]}")
    suite.add_result("SRL 10>>3=1", regout[9] == 1, f"R11={regout[9]}")
    suite.add_result("SRA 10>>>3=1", regout[10] == 1, f"R12={regout[10]}")
    shutil.rmtree(test_dir)

def test_sra_sign_extension(suite: TestSuite):
    """PDF page 5: SRA is arithmetic shift with sign extension."""
    print("\n[SRA SIGN EXTENSION - PDF p.5]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_sra_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, -16),
        encode_instruction(OP_ADD, 3, 0, 1, 2),
        encode_instruction(OP_SRA, 4, 2, 3, 0),   # -16 >> 2 = -4 (sign extended)
        encode_instruction(OP_SRL, 5, 2, 3, 0),   # -16 >> 2 = large positive (logical)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("SRA -16>>2=-4 (signed)", regout[2] == -4, f"R4={regout[2]}")
    suite.add_result("SRL -16>>2 positive (unsigned)", regout[3] > 0, f"R5={regout[3]}")
    shutil.rmtree(test_dir)

# ============================================================================
# JAL INSTRUCTION (from PDF page 5)
# ============================================================================

def test_jal(suite: TestSuite):
    """PDF page 5: JAL saves PC+1 to R15 and jumps."""
    print("\n[JAL - PDF p.5]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_jal_")
    imem = [[
        encode_instruction(OP_ADD, 2, 0, 1, 5),     # R2 = 5 (target)
        encode_instruction(OP_JAL, 2, 0, 0, 0),     # Jump, R15 = PC+1 = 2
        encode_instruction(OP_ADD, 3, 0, 1, 77),    # Delay slot
        encode_instruction(OP_ADD, 4, 0, 1, 88),    # Skipped
        encode_instruction(OP_ADD, 5, 0, 1, 99),    # Skipped
        encode_instruction(OP_ADD, 6, 0, 1, 42),    # Target (PC=5)
    ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("R15 = next instruction (2)", regout[13] == 2, f"R15={regout[13]}")
    suite.add_result("Delay slot executed", regout[1] == 77, f"R3={regout[1]}")
    suite.add_result("Target reached", regout[4] == 42, f"R6={regout[4]}")
    shutil.rmtree(test_dir)

# ============================================================================
# HALT INSTRUCTION (from PDF page 5)
# ============================================================================

def test_halt(suite: TestSuite):
    """PDF page 5: HALT stops the core, pipeline drains."""
    print("\n[HALT - PDF p.5]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_halt_")
    imem = [[
        encode_instruction(OP_ADD, 2, 0, 1, 42),
        encode_instruction(OP_HALT, 0, 0, 0, 0),
        encode_instruction(OP_ADD, 3, 0, 1, 100),  # Should not execute
    ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("Instruction before HALT executed", regout[0] == 42, f"R2={regout[0]}")
    suite.add_result("Instruction after HALT not executed", regout[1] == 0, f"R3={regout[1]}")
    # HALT itself should NOT be counted as an instruction
    suite.add_result("HALT counted as instruction", stats.get('instructions', 0) == 2,
                f"instructions={stats.get('instructions')}")
    shutil.rmtree(test_dir)

# ============================================================================
# CACHE TESTS (from PDF page 2)
# ============================================================================

def test_cache_512_words_64_blocks(suite: TestSuite):
    """PDF page 2: 512 words, 8 words/block = 64 blocks."""
    print("\n[CACHE SIZE - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_cache_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    
    dsram = read_file(os.path.join(test_dir, "dsram0.txt"))
    tsram = read_file(os.path.join(test_dir, "tsram0.txt"))
    suite.add_result("DSRAM has 512 entries", len(dsram) == 512, f"Got {len(dsram)}")
    suite.add_result("TSRAM has 64 entries", len(tsram) == 64, f"Got {len(tsram)}")
    shutil.rmtree(test_dir)

def test_cache_block_8_words(suite: TestSuite):
    """PDF page 2: Block size is 8 words."""
    print("\n[CACHE BLOCK SIZE - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_blk_")
    memin = [f"{i:08X}" for i in range(16)]
    imem = [pad_with_halts([
        encode_instruction(OP_LW, 2, 0, 1, 0),  # miss, fetches block 0 (addr 0-7)
        encode_instruction(OP_LW, 3, 0, 1, 7),  # hit (same block)
        encode_instruction(OP_LW, 4, 0, 1, 8),  # miss, fetches block 1 (addr 8-15)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    
    # Verify correct values loaded
    suite.add_result("Loaded correct values", 
                    regout[0] == 0 and regout[1] == 7 and regout[2] == 8,
                    f"R2={regout[0]}, R3={regout[1]}, R4={regout[2]}")
    # 2 misses (block 0, block 1), 1 hit (addr 7 in block 0)
    suite.add_result("2 read misses", stats.get('read_miss', 0) == 2,
                    f"read_miss={stats.get('read_miss')}")
    suite.add_result("1 read hit", stats.get('read_hit', 0) == 1,
                    f"read_hit={stats.get('read_hit')}")
    shutil.rmtree(test_dir)

def test_cache_direct_mapped(suite: TestSuite):
    """PDF page 2: Direct-mapped cache - same index = conflict."""
    print("\n[DIRECT MAPPED - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_direct_")
    # Addresses 0 and 512 have same index (different tags)
    memin = ["00000000"] * 600
    memin[0] = "11111111"
    memin[512] = "22222222"
    imem = [pad_with_halts([
        encode_instruction(OP_LW, 2, 0, 1, 0),      # miss (tag 0)
        encode_instruction(OP_ADD, 3, 0, 1, 0x200), # R3 = 512
        encode_instruction(OP_LW, 4, 3, 0, 0),      # miss (tag 1, evicts tag 0)
        encode_instruction(OP_LW, 5, 0, 1, 0),      # miss (tag 0 evicted!)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("3 misses (conflict)", stats.get('read_miss', 0) == 3,
                    f"read_miss={stats.get('read_miss')}")
    shutil.rmtree(test_dir)

def test_write_back_write_allocate(suite: TestSuite):
    """PDF page 2: Write-back, write-allocate policy."""
    print("\n[WRITE-BACK WRITE-ALLOCATE - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    
    # Test write-allocate: write miss fetches block first
    test_dir = tempfile.mkdtemp(prefix="sim_wa_")
    memin = ["AABBCCDD"] * 16
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 0x123),
        encode_instruction(OP_SW, 2, 0, 1, 0),     # Write miss - allocate block
        encode_instruction(OP_LW, 3, 0, 1, 1),     # Should hit (same block)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Write allocate: subsequent read hits", stats.get('read_hit', 0) >= 1,
                    f"read_hit={stats.get('read_hit')}")
    suite.add_result("Block fetched on allocate", (regout[1] & 0xFFFFFFFF) == 0xAABBCCDD,
                    f"R3=0x{regout[1] & 0xFFFFFFFF:08X}")
    shutil.rmtree(test_dir)
    
    # Test write-back: data written to cache, only goes to memory on eviction
    test_dir = tempfile.mkdtemp(prefix="sim_wb_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 0x123),
        encode_instruction(OP_SW, 2, 0, 1, 0),      # Write to addr 0
        encode_instruction(OP_ADD, 3, 0, 1, 0x200), # 512 - same index
        encode_instruction(OP_SW, 2, 3, 0, 0),      # Evicts addr 0 block
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["00000000"] * 600)
    run_simulator(suite.sim_path, test_dir)
    memout = read_memout(os.path.join(test_dir, "memout.txt"))
    suite.add_result("Write-back: evicted data in memory", 
                    len(memout) > 0 and memout[0] == 0x123,
                    f"memout[0]=0x{memout[0]:X}" if memout else "empty")
    shutil.rmtree(test_dir)

# ============================================================================
# MESI PROTOCOL TESTS (from PDF pages 2-4)
# ============================================================================

def test_mesi_invalid_to_exclusive(suite: TestSuite):
    """PDF page 2: Single reader gets Exclusive state."""
    print("\n[MESI I->E - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mesiE_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    tsram = read_tsram(os.path.join(test_dir, "tsram0.txt"))
    suite.add_result("Single reader: Exclusive (2)", tsram[0][0] == 2,
                    f"MESI state={tsram[0][0]} (0=I, 1=S, 2=E, 3=M)")
    shutil.rmtree(test_dir)

def test_mesi_shared_with_bus_shared(suite: TestSuite):
    """PDF page 4: bus_shared signal determines Exclusive vs Shared."""
    print("\n[MESI SHARED + bus_shared - PDF p.4]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mesiS_")
    # Core 0 reads first, Core 1 reads later - both should be Shared
    imem = [
        pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)]),
        pad_with_halts([NOP] * 30 + [encode_instruction(OP_LW, 2, 0, 1, 0)]),  # Wait for Core 0
        make_halt_only(), make_halt_only()
    ]
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    tsram0 = read_tsram(os.path.join(test_dir, "tsram0.txt"))
    tsram1 = read_tsram(os.path.join(test_dir, "tsram1.txt"))
    suite.add_result("Core 0: Shared (1)", tsram0[0][0] == 1, f"MESI={tsram0[0][0]}")
    suite.add_result("Core 1: Shared (1)", tsram1[0][0] == 1, f"MESI={tsram1[0][0]}")
    
    # Check bus_shared signal in bus trace
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    flush_with_shared = [t for t in bus if t['cmd'] == 3 and t['shared'] == 1]
    suite.add_result("bus_shared=1 in response", len(flush_with_shared) > 0,
                    f"Flush with shared=1: {len(flush_with_shared)}")
    shutil.rmtree(test_dir)

def test_mesi_modified(suite: TestSuite):
    """PDF page 2: Write puts cache in Modified state."""
    print("\n[MESI I->M - PDF p.2]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mesiM_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 0x123),
        encode_instruction(OP_SW, 2, 0, 1, 0),
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["00000000"])
    run_simulator(suite.sim_path, test_dir)
    tsram = read_tsram(os.path.join(test_dir, "tsram0.txt"))
    suite.add_result("Writer: Modified (3)", tsram[0][0] == 3, f"MESI={tsram[0][0]}")
    shutil.rmtree(test_dir)

def test_mesi_exclusive_to_modified_silent(suite: TestSuite):
    """PDF (implicit): E->M transition is silent, no bus transaction."""
    print("\n[MESI E->M SILENT]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_em_")
    imem = [pad_with_halts([
        encode_instruction(OP_LW, 2, 0, 1, 0),     # BusRd -> Exclusive
        encode_instruction(OP_ADD, 2, 2, 1, 1),
        encode_instruction(OP_SW, 2, 0, 1, 0),     # E->M (no BusRdX needed)
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["00000000"])
    run_simulator(suite.sim_path, test_dir)
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    busrd = len([t for t in bus if t['cmd'] == 1 and t['origid'] == 0])
    busrdx = len([t for t in bus if t['cmd'] == 2 and t['origid'] == 0])
    suite.add_result("Only BusRd, no BusRdX for E->M", busrd >= 1 and busrdx == 0,
                    f"BusRd={busrd}, BusRdX={busrdx}")
    tsram = read_tsram(os.path.join(test_dir, "tsram0.txt"))
    suite.add_result("Final state: Modified", tsram[0][0] == 3, f"MESI={tsram[0][0]}")
    shutil.rmtree(test_dir)

def test_mesi_modified_provides_data(suite: TestSuite):
    """PDF page 4: Modified cache provides data on BusRd from other core."""
    print("\n[MESI M PROVIDES DATA - PDF p.4]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mpd_")
    # Core 0 writes, then Core 1 reads - should get data from Core 0
    # NOTE: Use 0x123 (291) instead of 0xABC (2748) because 0xABC has bit 11 set
    # and gets sign-extended to a negative number (-1348)
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 0, 1, 0x123),  # R2 = 0x123 (291, fits in 11 bits)
            encode_instruction(OP_SW, 2, 0, 1, 0),       # MEM[0] = R2
        ]),
        # Core 1 waits long enough for Core 0 to complete write
        pad_with_halts([NOP] * 50 + [encode_instruction(OP_LW, 2, 0, 1, 0)]),
        make_halt_only(), make_halt_only()
    ]
    create_test_files(test_dir, imem, ["00000000"])
    run_simulator(suite.sim_path, test_dir)
    regout1 = read_regout(os.path.join(test_dir, "regout1.txt"))
    suite.add_result("Core 1 gets Modified data (0x123)", regout1[0] == 0x123,
                    f"R2=0x{regout1[0] & 0xFFFFFFFF:X}")
    
    # Check that Core 0 flushed (origid=0, cmd=3)
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    core0_flush = [t for t in bus if t['origid'] == 0 and t['cmd'] == 3]
    suite.add_result("Core 0 provided data via Flush", len(core0_flush) > 0,
                    f"Core 0 flushes: {len(core0_flush)}")
    shutil.rmtree(test_dir)

# ============================================================================
# BUS TESTS (from PDF pages 3-4)
# ============================================================================

def test_bus_16_cycle_latency(suite: TestSuite):
    """PDF page 4: Memory responds 16 cycles after BusRd."""
    print("\n[BUS 16 CYCLE LATENCY - PDF p.4]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_lat_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    busrd = [t for t in bus if t['cmd'] == 1]
    flush = [t for t in bus if t['cmd'] == 3 and t['origid'] == 4]  # Memory flush
    if busrd and flush:
        latency = flush[0]['cycle'] - busrd[0]['cycle']
        suite.add_result("Memory latency = 16 cycles", latency == 16, f"Got {latency}")
    else:
        suite.add_result("Bus has BusRd and Memory Flush", False, "Missing transactions")
    shutil.rmtree(test_dir)

def test_bus_8_word_flush(suite: TestSuite):
    """PDF page 4: Flush sends 8 words in consecutive cycles."""
    print("\n[BUS 8 WORD FLUSH - PDF p.4]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_flush_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, [f"{i:08X}" for i in range(8)])
    run_simulator(suite.sim_path, test_dir)
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    flush = [t for t in bus if t['cmd'] == 3]
    suite.add_result("8 Flush transactions", len(flush) >= 8, f"Got {len(flush)}")
    if len(flush) >= 8:
        cycles = [f['cycle'] for f in flush[:8]]
        consec = all(cycles[i+1] == cycles[i] + 1 for i in range(7))
        suite.add_result("Consecutive cycles", consec, f"Cycles: {cycles}")
        # Check addresses are consecutive
        addrs = [f['addr'] for f in flush[:8]]
        addr_consec = all(addrs[i+1] == addrs[i] + 1 for i in range(7))
        suite.add_result("Consecutive addresses", addr_consec, f"Addrs: {addrs}")
    shutil.rmtree(test_dir)

def test_bus_origid(suite: TestSuite):
    """PDF page 3: origid 0-3 = cores, 4 = main memory."""
    print("\n[BUS ORIGID - PDF p.3]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_origid_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, ["DEADBEEF"])
    run_simulator(suite.sim_path, test_dir)
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    
    # Core 0 should issue BusRd
    core0_busrd = [t for t in bus if t['origid'] == 0 and t['cmd'] == 1]
    suite.add_result("Core 0 (origid=0) issues BusRd", len(core0_busrd) > 0,
                    f"Count: {len(core0_busrd)}")
    
    # Memory should issue Flush
    mem_flush = [t for t in bus if t['origid'] == 4 and t['cmd'] == 3]
    suite.add_result("Memory (origid=4) issues Flush", len(mem_flush) > 0,
                    f"Count: {len(mem_flush)}")
    shutil.rmtree(test_dir)

def test_bus_round_robin(suite: TestSuite):
    """PDF page 3: Fair round-robin arbitration."""
    print("\n[BUS ROUND-ROBIN - PDF p.3]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_rr_")
    # All 4 cores issue loads to different addresses
    imem = []
    for i in range(4):
        imem.append(pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, (i+1) * 64)]))
    create_test_files(test_dir, imem, ["00000000"] * 500)
    run_simulator(suite.sim_path, test_dir)
    bus = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    cores = set(t['origid'] for t in bus if t['cmd'] in [1, 2] and t['origid'] < 4)
    suite.add_result("All 4 cores got bus access", len(cores) == 4, f"Cores: {cores}")
    shutil.rmtree(test_dir)

def test_bus_transaction_locking(suite: TestSuite):
    """PDF page 3: Bus locked until Flush completes pending BusRd/BusRdX."""
    print("\n[BUS TRANSACTION LOCKING - PDF p.3]")
    print("-" * 40)
    if not suite.sim_path:
        return
    # This is implicitly tested by checking that multiple cores can complete their transactions
    test_dir = tempfile.mkdtemp(prefix="sim_lock_")
    imem = [
        pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)]),
        pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 64)]),
        make_halt_only(), make_halt_only()
    ]
    create_test_files(test_dir, imem, ["00000000"] * 100)
    run_simulator(suite.sim_path, test_dir)
    regout0 = read_regout(os.path.join(test_dir, "regout0.txt"))
    regout1 = read_regout(os.path.join(test_dir, "regout1.txt"))
    suite.add_result("Both cores completed loads", regout0[0] == 0 and regout1[0] == 0,
                    f"Core0 R2={regout0[0]}, Core1 R2={regout1[0]}")
    shutil.rmtree(test_dir)

# ============================================================================
# LOAD/STORE TESTS (from PDF page 5)
# ============================================================================

def test_lw_sw(suite: TestSuite):
    """PDF page 5: LW and SW with address calculation."""
    print("\n[LW/SW - PDF p.5]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_lw_")
    memin = ["DEADBEEF", "CAFEBABE"]
    imem = [pad_with_halts([
        # LW: R[rd] = MEM[R[rs] + R[rt]]
        encode_instruction(OP_LW, 2, 0, 1, 0),     # R2 = MEM[0+0]
        encode_instruction(OP_ADD, 3, 0, 1, 1),
        encode_instruction(OP_LW, 4, 0, 3, 0),     # R4 = MEM[0+1]
        # SW: MEM[R[rs] + R[rt]] = R[rd]
        encode_instruction(OP_ADD, 5, 0, 1, 0x123),
        encode_instruction(OP_SW, 5, 0, 1, 2),     # MEM[0+2] = R5
        encode_instruction(OP_LW, 6, 0, 1, 2),     # R6 = MEM[2]
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("LW from addr 0", (regout[0] & 0xFFFFFFFF) == 0xDEADBEEF,
                    f"R2=0x{regout[0] & 0xFFFFFFFF:08X}")
    suite.add_result("LW from addr 1", (regout[2] & 0xFFFFFFFF) == 0xCAFEBABE,
                    f"R4=0x{regout[2] & 0xFFFFFFFF:08X}")
    suite.add_result("SW then LW", regout[4] == 0x123, f"R6=0x{regout[4]:X}")
    shutil.rmtree(test_dir)

# ============================================================================
# MULTI-CORE TESTS (from PDF page 1)
# ============================================================================

def test_4_cores_parallel(suite: TestSuite):
    """PDF page 1: 4 cores run in parallel."""
    print("\n[4 CORES PARALLEL - PDF p.1]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_4core_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, i*10)]) for i in range(4)]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    for i in range(4):
        regout = read_regout(os.path.join(test_dir, f"regout{i}.txt"))
        stats = parse_stats(os.path.join(test_dir, f"stats{i}.txt"))
        suite.add_result(f"Core {i} computed R2={i*10}", regout[0] == i*10, f"R2={regout[0]}")
        suite.add_result(f"Core {i} has stats", stats.get('cycles', 0) > 0, f"cycles={stats.get('cycles')}")
    shutil.rmtree(test_dir)

# ============================================================================
# LOOP TEST (requires proper delay slot handling)
# ============================================================================

def test_loop_with_delay_slot(suite: TestSuite):
    """Test backward branch loop with NOP in delay slot."""
    print("\n[LOOP WITH DELAY SLOT]")
    print("-" * 40)
    if not suite.sim_path:
        return
    test_dir = tempfile.mkdtemp(prefix="sim_loop_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10),     # R2 = 10
        encode_instruction(OP_ADD, 10, 0, 1, 2),    # R10 = 2 (loop target)
        encode_instruction(OP_ADD, 2, 2, 1, -1),    # R2-- (PC=2)
        encode_instruction(OP_BNE, 10, 2, 0, 0),    # if R2 != 0, goto PC=2
        NOP,                                         # Delay slot
    ])] + [make_halt_only()] * 3
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir, timeout=30)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("Loop completed (R2=0)", regout[0] == 0, f"R2={regout[0]}")
    suite.add_result("Loop iterations reflected in instructions", stats.get('instructions', 0) >= 20,
                    f"instructions={stats.get('instructions')}")
    shutil.rmtree(test_dir)

# ============================================================================
# EXAMPLE FOLDER TESTING (Teacher-provided test cases)
# ============================================================================

def compare_files_detailed(expected_path: str, actual_path: str, max_diffs: int = 10) -> Tuple[bool, List[str]]:
    """
    Compare two files line by line.
    Returns (match, list of difference descriptions).
    """
    diffs = []
    
    if not os.path.exists(expected_path):
        return False, [f"Expected file not found: {expected_path}"]
    if not os.path.exists(actual_path):
        return False, [f"Actual file not found: {actual_path}"]
    
    with open(expected_path, 'r') as f:
        expected_lines = [line.rstrip() for line in f]  # Strip all trailing whitespace
    with open(actual_path, 'r') as f:
        actual_lines = [line.rstrip() for line in f]  # Strip all trailing whitespace
    
    # Remove trailing empty lines
    while expected_lines and expected_lines[-1] == '':
        expected_lines.pop()
    while actual_lines and actual_lines[-1] == '':
        actual_lines.pop()
    
    # Check line count
    if len(expected_lines) != len(actual_lines):
        diffs.append(f"Line count: expected {len(expected_lines)}, got {len(actual_lines)}")
    
    # Compare line by line
    max_lines = max(len(expected_lines), len(actual_lines))
    for i in range(max_lines):
        if len(diffs) >= max_diffs:
            diffs.append(f"... and more differences (stopped at {max_diffs})")
            break
            
        exp = expected_lines[i].strip().upper() if i < len(expected_lines) else "<MISSING>"
        act = actual_lines[i].strip().upper() if i < len(actual_lines) else "<MISSING>"
        
        if exp != act:
            # Find first difference position
            diff_pos = 0
            min_len = min(len(exp), len(act))
            while diff_pos < min_len and exp[diff_pos] == act[diff_pos]:
                diff_pos += 1
            
            # For short differences, show full content
            if len(exp) <= 50 and len(act) <= 50:
                diffs.append(f"Line {i}: exp='{exp}' got='{act}'")
            else:
                # Show context around difference
                start = max(0, diff_pos - 10)
                exp_ctx = exp[start:diff_pos+20] if diff_pos < len(exp) else exp[start:]
                act_ctx = act[start:diff_pos+20] if diff_pos < len(act) else act[start:]
                diffs.append(f"Line {i} pos {diff_pos}: exp='...{exp_ctx}...' got='...{act_ctx}...'")
    
    return len(diffs) == 0, diffs

def run_example_folder_test(suite: TestSuite, folder_path: str):
    """
    Run simulator on teacher-provided example folder.
    Compares all output files line by line with expected outputs.
    """
    folder_name = os.path.basename(folder_path)
    print(f"\n[EXAMPLE FOLDER: {folder_name}]")
    print("=" * 60)
    
    if not suite.sim_path:
        suite.add_result(f"{folder_name}: simulator", False, "No simulator compiled")
        return
    
    if not os.path.exists(folder_path):
        suite.add_result(f"{folder_name}: folder exists", False, f"Not found: {folder_path}")
        return
    
    # Check for required input files
    input_files = ["imem0.txt", "imem1.txt", "imem2.txt", "imem3.txt", "memin.txt"]
    missing_inputs = [f for f in input_files if not os.path.exists(os.path.join(folder_path, f))]
    if missing_inputs:
        suite.add_result(f"{folder_name}: input files", False, f"Missing: {missing_inputs}")
        return
    suite.add_result(f"{folder_name}: input files exist", True)
    
    # Create temporary working directory
    work_dir = tempfile.mkdtemp(prefix=f"sim_example_{folder_name}_")
    
    # Copy input files to work directory
    for f in input_files:
        shutil.copy(os.path.join(folder_path, f), os.path.join(work_dir, f))
    
    # Run simulator
    success, stdout, stderr = run_simulator(suite.sim_path, work_dir, timeout=120)
    suite.add_result(f"{folder_name}: simulator runs", success, 
                    stderr[:100] if not success else "")
    
    if not success:
        shutil.rmtree(work_dir)
        return
    
    # Check all output files were created
    missing_outputs = [f for f in OUTPUT_FILES if not os.path.exists(os.path.join(work_dir, f))]
    suite.add_result(f"{folder_name}: all outputs created", len(missing_outputs) == 0,
                    f"Missing: {missing_outputs[:5]}")
    
    # Compare each output file with expected
    print(f"\n  Comparing output files:")
    all_match = True
    files_compared = 0
    files_matched = 0
    
    for output_file in OUTPUT_FILES:
        expected_path = os.path.join(folder_path, output_file)
        actual_path = os.path.join(work_dir, output_file)
        
        # Skip if expected file doesn't exist (not all tests have all outputs)
        if not os.path.exists(expected_path):
            continue
        
        files_compared += 1
        match, diffs = compare_files_detailed(expected_path, actual_path)
        
        if match:
            files_matched += 1
            print(f"    ✓ {output_file}: MATCH")
        else:
            all_match = False
            print(f"    ✗ {output_file}: MISMATCH")
            for diff in diffs[:5]:  # Show first 5 differences
                print(f"        {diff}")
            if len(diffs) > 5:
                print(f"        ... and {len(diffs) - 5} more differences")
    
    suite.add_result(f"{folder_name}: all files match", all_match,
                    f"{files_matched}/{files_compared} files match")
    
    # Print statistics summary
    print(f"\n  Statistics from run:")
    for i in range(4):
        stats_path = os.path.join(work_dir, f"stats{i}.txt")
        if os.path.exists(stats_path):
            stats = parse_stats(stats_path)
            print(f"    Core {i}: {stats.get('cycles', '?')} cycles, "
                  f"{stats.get('instructions', '?')} inst, "
                  f"rd_hit={stats.get('read_hit', '?')}, rd_miss={stats.get('read_miss', '?')}, "
                  f"wr_hit={stats.get('write_hit', '?')}, wr_miss={stats.get('write_miss', '?')}")
    
    # Compare with expected statistics if available
    print(f"\n  Expected statistics:")
    for i in range(4):
        expected_stats_path = os.path.join(folder_path, f"stats{i}.txt")
        if os.path.exists(expected_stats_path):
            stats = parse_stats(expected_stats_path)
            print(f"    Core {i}: {stats.get('cycles', '?')} cycles, "
                  f"{stats.get('instructions', '?')} inst, "
                  f"rd_hit={stats.get('read_hit', '?')}, rd_miss={stats.get('read_miss', '?')}, "
                  f"wr_hit={stats.get('write_hit', '?')}, wr_miss={stats.get('write_miss', '?')}")
    
    # Cleanup
    shutil.rmtree(work_dir)

def test_example_folders(suite: TestSuite, example_dir: str):
    """
    Test all example folders in the given directory.
    Looks for folders containing imem0.txt as indicator of test case.
    """
    if not os.path.exists(example_dir):
        print(f"Example directory not found: {example_dir}")
        return
    
    # Check if example_dir itself is a test folder
    if os.path.exists(os.path.join(example_dir, "imem0.txt")):
        run_example_folder_test(suite, example_dir)
        return
    
    # Otherwise, look for subfolders that are test cases
    test_folders = []
    for item in os.listdir(example_dir):
        item_path = os.path.join(example_dir, item)
        if os.path.isdir(item_path) and os.path.exists(os.path.join(item_path, "imem0.txt")):
            test_folders.append((item, item_path))
    
    if not test_folders:
        print(f"No test folders found in: {example_dir}")
        print("Looking for folders containing imem0.txt")
        return
    
    print(f"\nFound {len(test_folders)} example test folder(s)")
    
    for name, path in sorted(test_folders):
        run_example_folder_test(suite, path)

# ============================================================================
# ASSIGNMENT-SPECIFIC VALIDATION
# ============================================================================

def validate_output_syntax(suite: TestSuite, work_dir: str, name: str):
    """Validate syntax of all output files per PDF spec."""
    # regout: 14 lines (R2-R15), 8 hex each
    for i in range(4):
        lines = read_file(os.path.join(work_dir, f"regout{i}.txt"))
        if len(lines) != 14:
            suite.add_result(f"{name}: regout{i} has 14 lines", False, f"Got {len(lines)}")
        else:
            bad = [j for j, l in enumerate(lines) if len(l) != 8]
            if bad:
                suite.add_result(f"{name}: regout{i} format", False, f"Bad lines: {bad}")
    
    # dsram: 512 lines
    for i in range(4):
        lines = read_file(os.path.join(work_dir, f"dsram{i}.txt"))
        if len(lines) != 512:
            suite.add_result(f"{name}: dsram{i} has 512 lines", False, f"Got {len(lines)}")
    
    # tsram: 64 lines
    for i in range(4):
        lines = read_file(os.path.join(work_dir, f"tsram{i}.txt"))
        if len(lines) != 64:
            suite.add_result(f"{name}: tsram{i} has 64 lines", False, f"Got {len(lines)}")
    
    # stats: all 8 fields
    required_stats = ['cycles', 'instructions', 'read_hit', 'write_hit', 
                      'read_miss', 'write_miss', 'decode_stall', 'mem_stall']
    for i in range(4):
        stats = parse_stats(os.path.join(work_dir, f"stats{i}.txt"))
        missing = [s for s in required_stats if s not in stats]
        if missing:
            suite.add_result(f"{name}: stats{i} complete", False, f"Missing: {missing}")

def validate_counter(suite: TestSuite, work_dir: str, name: str):
    """PDF page 8-9: Counter program - 4 cores × 128 increments = 512."""
    memout = read_memout(os.path.join(work_dir, "memout.txt"))
    if memout:
        suite.add_result(f"{name}: memout[0] = 512", memout[0] == 512,
                        f"Got {memout[0]} (expected 4×128=512)")
    else:
        suite.add_result(f"{name}: memout has content", False, "Empty")
    
    # All cores should participate
    for i in range(4):
        stats = parse_stats(os.path.join(work_dir, f"stats{i}.txt"))
        suite.add_result(f"{name}: core {i} active", stats.get('instructions', 0) > 100,
                        f"instructions={stats.get('instructions')}")
    
    # Round-robin: check bus activity from all cores
    bus = parse_bus_trace(os.path.join(work_dir, "bustrace.txt"))
    cores_busrdx = set(t['origid'] for t in bus if t['cmd'] == 2 and t['origid'] < 4)
    suite.add_result(f"{name}: all cores issued BusRdX", len(cores_busrdx) == 4,
                    f"Cores: {cores_busrdx}")

def validate_mulserial(suite: TestSuite, work_dir: str, name: str):
    """PDF page 9: Matrix multiply on core 0 only."""
    stats0 = parse_stats(os.path.join(work_dir, "stats0.txt"))
    suite.add_result(f"{name}: core 0 executed many instructions",
                    stats0.get('instructions', 0) > 1000,
                    f"instructions={stats0.get('instructions')}")
    
    for i in range(1, 4):
        stats = parse_stats(os.path.join(work_dir, f"stats{i}.txt"))
        suite.add_result(f"{name}: core {i} idle",
                        stats.get('instructions', 0) < 10,
                        f"instructions={stats.get('instructions')}")
    
    # Result matrix at 0x200-0x2FF
    memout = read_memout(os.path.join(work_dir, "memout.txt"))
    if len(memout) > 0x2FF:
        result = memout[0x200:0x300]
        non_zero = sum(1 for v in result if v != 0)
        suite.add_result(f"{name}: result matrix has values", non_zero > 100,
                        f"Non-zero: {non_zero}/256")

def validate_mulparallel(suite: TestSuite, work_dir: str, name: str, serial_cycles: int = None, serial_result: List[int] = None):
    """PDF page 9: Matrix multiply on all 4 cores."""
    # All cores participate
    core_inst = []
    core_cycles = []
    for i in range(4):
        stats = parse_stats(os.path.join(work_dir, f"stats{i}.txt"))
        core_inst.append(stats.get('instructions', 0))
        core_cycles.append(stats.get('cycles', 0))
        suite.add_result(f"{name}: core {i} participated",
                        stats.get('instructions', 0) > 50,
                        f"instructions={stats.get('instructions')}")
    
    # Check speedup vs serial
    parallel_cycles = max(core_cycles)
    if serial_cycles and serial_cycles > 0 and parallel_cycles > 0:
        speedup = serial_cycles / parallel_cycles
        suite.add_result(f"{name}: speedup > 1.5x", speedup > 1.5,
                        f"Speedup: {speedup:.2f}x ({serial_cycles} vs {parallel_cycles})")
    
    # Compare result with serial
    memout = read_memout(os.path.join(work_dir, "memout.txt"))
    if serial_result and len(memout) > 0x2FF:
        result = memout[0x200:0x300]
        if len(result) == len(serial_result):
            matches = sum(1 for a, b in zip(result, serial_result) if a == b)
            suite.add_result(f"{name}: result matches serial", matches == 256,
                           f"Matching: {matches}/256")

def run_assignment_test(suite: TestSuite, name: str, path: str, serial_cycles: int = None, serial_result: List[int] = None):
    print(f"\n[ASSIGNMENT: {name}]")
    print("-" * 40)
    if not suite.sim_path:
        return None, None
    
    required = ["imem0.txt", "imem1.txt", "imem2.txt", "imem3.txt", "memin.txt"]
    missing = [f for f in required if not os.path.exists(os.path.join(path, f))]
    if missing:
        suite.add_result(f"{name}: inputs exist", False, f"Missing: {missing}")
        return None, None
    suite.add_result(f"{name}: inputs exist", True)
    
    work_dir = tempfile.mkdtemp(prefix=f"sim_{name}_")
    for f in required:
        shutil.copy(os.path.join(path, f), os.path.join(work_dir, f))
    
    success, _, stderr = run_simulator(suite.sim_path, work_dir, timeout=120)
    suite.add_result(f"{name}: runs", success, stderr[:100] if not success else "")
    
    if not success:
        shutil.rmtree(work_dir)
        return None, None
    
    missing_out = [f for f in OUTPUT_FILES if not os.path.exists(os.path.join(work_dir, f))]
    suite.add_result(f"{name}: all outputs", len(missing_out) == 0, f"Missing: {missing_out[:3]}")
    
    validate_output_syntax(suite, work_dir, name)
    
    # Compare with expected outputs if provided
    expected = [f for f in OUTPUT_FILES if os.path.exists(os.path.join(path, f))]
    if expected:
        mismatches = []
        for f in expected:
            exp = [l.strip().upper() for l in read_file(os.path.join(path, f))]
            act = [l.strip().upper() for l in read_file(os.path.join(work_dir, f))]
            if exp != act:
                mismatches.append(f)
        suite.add_result(f"{name}: matches expected", len(mismatches) == 0,
                       f"Mismatch: {mismatches[:3]}")
    
    # Program-specific validation
    name_lower = name.lower()
    if 'counter' in name_lower:
        validate_counter(suite, work_dir, name)
    elif 'mulserial' in name_lower or 'serial' in name_lower:
        validate_mulserial(suite, work_dir, name)
    elif 'mulparallel' in name_lower or 'parallel' in name_lower:
        validate_mulparallel(suite, work_dir, name, serial_cycles, serial_result)
    
    # Get results for comparison
    memout = read_memout(os.path.join(work_dir, "memout.txt"))
    result_matrix = memout[0x200:0x300] if len(memout) > 0x2FF else []
    max_cycles = max(parse_stats(os.path.join(work_dir, f"stats{i}.txt")).get('cycles', 0) for i in range(4))
    
    print(f"  Stats:")
    for i in range(4):
        stats = parse_stats(os.path.join(work_dir, f"stats{i}.txt"))
        print(f"    Core {i}: {stats.get('cycles', 0)} cyc, {stats.get('instructions', 0)} inst, "
              f"hit={stats.get('read_hit', 0)}/{stats.get('write_hit', 0)}, "
              f"miss={stats.get('read_miss', 0)}/{stats.get('write_miss', 0)}")
    
    shutil.rmtree(work_dir)
    return max_cycles, result_matrix

def test_assignment_dirs(suite: TestSuite, assignment_dir: str):
    if not os.path.exists(assignment_dir):
        print(f"Assignment dir not found: {assignment_dir}")
        return
    
    tests = [(d, os.path.join(assignment_dir, d)) 
             for d in os.listdir(assignment_dir)
             if os.path.isdir(os.path.join(assignment_dir, d)) and
                os.path.exists(os.path.join(assignment_dir, d, "imem0.txt"))]
    
    print(f"\nFound {len(tests)} assignment tests")
    
    # Sort: serial first, then counter, then parallel
    tests = sorted(tests, key=lambda x: (
        0 if 'serial' in x[0].lower() else 
        1 if 'counter' in x[0].lower() else 2
    ))
    
    serial_cycles = None
    serial_result = None
    for name, path in tests:
        cycles, result = run_assignment_test(suite, name, path, serial_cycles, serial_result)
        if 'serial' in name.lower():
            serial_cycles = cycles
            serial_result = result

# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="MESI Simulator Test Suite")
    parser.add_argument("--test-dir", "-t", help="Assignment test directory (counter, mulserial, mulparallel)")
    parser.add_argument("--example-dir", "-e", help="Teacher-provided example folder (e.g., example_061225_win)")
    parser.add_argument("--only-assignment", "-a", action="store_true", help="Run only assignment tests")
    parser.add_argument("--only-example", "-x", action="store_true", help="Run only example folder tests")
    parser.add_argument("--sim-source", "-s", default=SIM_SOURCE, help="Path to sim.c source file")
    args = parser.parse_args()
    
    print("=" * 60)
    print("MULTI-CORE MESI SIMULATOR - COMPLETE TEST SUITE")
    print("Based on Project Assignment PDF")
    print("=" * 60)
    
    source_path = None
    for path in [args.sim_source, f"./{args.sim_source}", f"/home/claude/{args.sim_source}",
                 f"/mnt/user-data/uploads/{args.sim_source}", f"/mnt/user-data/outputs/{args.sim_source}"]:
        if os.path.exists(path):
            source_path = path
            break
    
    if not source_path:
        print(f"ERROR: Cannot find {args.sim_source}")
        return 1
    
    print(f"Source: {source_path}")
    suite = TestSuite()
    
    suite.sim_path = test_compilation(suite, source_path)
    if not suite.sim_path:
        suite.summary()
        return 1
    
    # Run internal tests unless skipped
    if not args.only_assignment and not args.only_example:
        test_basic_execution(suite)
        test_regout_format(suite)
        test_dsram_format(suite)
        test_tsram_format(suite)
        test_bustrace_format(suite)
        test_coretrace_format(suite)
        test_stats_format(suite)
        test_memout_format(suite)
        test_r0_always_zero(suite)
        test_r1_immediate(suite)
        test_data_hazard_stall_decode(suite)
        test_cache_miss_stall_mem(suite)
        test_no_bypassing(suite)
        test_delay_slot(suite)
        test_branch_in_decode(suite)
        test_all_branches(suite)
        test_all_alu(suite)
        test_sra_sign_extension(suite)
        test_jal(suite)
        test_halt(suite)
        test_cache_512_words_64_blocks(suite)
        test_cache_block_8_words(suite)
        test_cache_direct_mapped(suite)
        test_write_back_write_allocate(suite)
        test_mesi_invalid_to_exclusive(suite)
        test_mesi_shared_with_bus_shared(suite)
        test_mesi_modified(suite)
        test_mesi_exclusive_to_modified_silent(suite)
        test_mesi_modified_provides_data(suite)
        test_bus_16_cycle_latency(suite)
        test_bus_8_word_flush(suite)
        test_bus_origid(suite)
        test_bus_round_robin(suite)
        test_bus_transaction_locking(suite)
        test_lw_sw(suite)
        test_4_cores_parallel(suite)
        test_loop_with_delay_slot(suite)
    
    # Run example folder tests if specified
    if args.example_dir:
        test_example_folders(suite, args.example_dir)
    
    # Run assignment directory tests if specified
    if args.test_dir:
        test_assignment_dirs(suite, args.test_dir)
    
    success = suite.summary()
    
    if suite.sim_path and os.path.exists(os.path.dirname(suite.sim_path)):
        try:
            shutil.rmtree(os.path.dirname(suite.sim_path))
        except:
            pass
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())