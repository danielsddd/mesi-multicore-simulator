#!/usr/bin/env python3
"""
Multi-Core MESI Simulator - Exhaustive Test Suite
===================================================
Comprehensive testing for the sim.c implementation.

Tests Cover:
- Compilation
- Basic execution  
- Output file existence and format
- Pipeline behavior (5-stage, hazards, branches, delay slot)
- Cache behavior (MESI transitions, hits/misses)
- Bus behavior (arbitration, latency, writeback locking)
- Statistics correctness
- Exhaustive opcode coverage with edge cases
- Advanced MESI coherence and snooping
- Pipeline hazard stress tests
- Register integrity (R0 immutability, R1 immediate update)

NOTES ON HALT BEHAVIOR:
=======================
HALT stops fetching when it reaches WRITEBACK and sets halted=true.
Instructions already in the pipeline WILL complete (pipeline drain).
This is CORRECT behavior - example files pad with HALTs for this reason.

NOTES ON MEMOUT:
================
memout.txt reflects main memory state. With write-back policy, Modified
cache lines only go to memory on eviction or bus snoop. If sim ends with
Modified data still in cache, it may not appear in memout unless the
implementation flushes all caches at the end.
"""

import os, sys, subprocess, tempfile, shutil, re
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

# ============================================================================
# Configuration
# ============================================================================

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

MAX_INT_12 = 0x7FF
MIN_INT_12 = -0x800

# ============================================================================
# Test Result Tracking
# ============================================================================

@dataclass
class TestResult:
    name: str
    passed: bool
    message: str = ""
    details: str = ""

class TestSuite:
    def __init__(self):
        self.results: List[TestResult] = []
        self.sim_path: Optional[str] = None
        
    def add_result(self, name: str, passed: bool, message: str = "", details: str = ""):
        self.results.append(TestResult(name, passed, message, details))
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")
        if not passed and message:
            print(f"         {message}")
    
    def summary(self):
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        print("\n" + "=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        print(f"Total:  {total}")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")
        if failed > 0:
            print("\nFailed tests:")
            for r in self.results:
                if not r.passed:
                    print(f"  - {r.name}: {r.message}")
        print("=" * 60)
        return failed == 0

# ============================================================================
# Utility Functions
# ============================================================================

def encode_instruction(opcode: int, rd: int, rs: int, rt: int, imm: int) -> str:
    if imm < 0:
        imm = imm & 0xFFF
    inst = ((opcode & 0xFF) << 24) | ((rd & 0xF) << 20) | \
           ((rs & 0xF) << 16) | ((rt & 0xF) << 12) | (imm & 0xFFF)
    return f"{inst:08X}"

def sign_extend_12_to_32(val: int) -> int:
    """Sign-extend a 12-bit value to 32-bit signed."""
    val = val & 0xFFF  # Ensure 12-bit
    if val & 0x800:
        return (val | 0xFFFFF000) - 0x100000000  # Return as signed Python int
    return val

def to_signed_32(val: int) -> int:
    if val >= 0x80000000:
        return val - 0x100000000
    return val

# Opcode definitions
OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_MUL = 0, 1, 2, 3, 4, 5
OP_SLL, OP_SRA, OP_SRL = 6, 7, 8
OP_BEQ, OP_BNE, OP_BLT, OP_BGT, OP_BLE, OP_BGE = 9, 10, 11, 12, 13, 14
OP_JAL, OP_LW, OP_SW = 15, 16, 17
OP_HALT = 20

HALT_PAD = 10

def pad_with_halts(instructions: List[str]) -> List[str]:
    halts = [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD
    return instructions + halts

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
        return False, "", "Timeout expired"
    except Exception as e:
        return False, "", str(e)

def read_file(filepath: str) -> List[str]:
    try:
        with open(filepath, 'r') as f:
            return [line.strip() for line in f.readlines()]
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
    lines = read_file(filepath)
    return [to_signed_32(int(line, 16)) for line in lines if line]

def read_tsram(filepath: str) -> List[Tuple[int, int]]:
    result = []
    for line in read_file(filepath):
        if line:
            val = int(line, 16)
            result.append(((val >> 12) & 0x3, val & 0xFFF))
    return result

def read_memout(filepath: str) -> List[int]:
    return [int(line, 16) for line in read_file(filepath) if line]

# ============================================================================
# Test Functions - Basic
# ============================================================================

def test_compilation(suite: TestSuite, source_path: str) -> Optional[str]:
    print("\n[1] COMPILATION TESTS")
    print("-" * 40)
    if not os.path.exists(source_path):
        suite.add_result("Source file exists", False, f"Cannot find {source_path}")
        return None
    suite.add_result("Source file exists", True)
    temp_dir = tempfile.mkdtemp(prefix="sim_compile_")
    shutil.copy(source_path, temp_dir)
    output_path = os.path.join(temp_dir, SIM_BINARY)
    src_path = os.path.join(temp_dir, SIM_SOURCE)
    try:
        result = subprocess.run([CC] + CFLAGS + ["-o", output_path, src_path],
                               capture_output=True, text=True, timeout=30)
        if result.returncode == 0:
            suite.add_result("Compilation successful", True)
            warnings = [l for l in result.stderr.split('\n') if 'warning' in l.lower()]
            suite.add_result("No compiler warnings", len(warnings) == 0,
                           f"{len(warnings)} warnings found" if warnings else "")
            return output_path
        else:
            suite.add_result("Compilation successful", False, result.stderr[:500])
            shutil.rmtree(temp_dir)
            return None
    except Exception as e:
        suite.add_result("Compilation successful", False, str(e))
        shutil.rmtree(temp_dir)
        return None

def test_basic_execution(suite: TestSuite):
    print("\n[2] BASIC EXECUTION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Basic execution", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_basic_")
    imem = [[encode_instruction(OP_HALT, 0, 0, 0, 0)] * 10] * 4
    create_test_files(test_dir, imem)
    success, stdout, stderr = run_simulator(suite.sim_path, test_dir)
    suite.add_result("Simulator runs without crash", success, stderr[:200] if not success else "")
    if success:
        missing = [f for f in OUTPUT_FILES if not os.path.exists(os.path.join(test_dir, f))]
        suite.add_result("All output files created", len(missing) == 0, f"Missing: {', '.join(missing)}" if missing else "")
        stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
        suite.add_result("Quick termination on halt", stats.get('cycles', 0) <= 10, f"cycles={stats.get('cycles', 'N/A')}")
    shutil.rmtree(test_dir)

def test_output_formats(suite: TestSuite):
    print("\n[3] OUTPUT FORMAT TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Output format tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_format_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 5)]), make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("regout has 14 lines (R2-R15)", len(regout) == 14, f"Got {len(regout)} lines")
    dsram = read_file(os.path.join(test_dir, "dsram0.txt"))
    suite.add_result("dsram has 512 lines", len(dsram) == 512, f"Got {len(dsram)} lines")
    tsram = read_file(os.path.join(test_dir, "tsram0.txt"))
    suite.add_result("tsram has 64 lines", len(tsram) == 64, f"Got {len(tsram)} lines")
    shutil.rmtree(test_dir)

def test_data_hazards(suite: TestSuite):
    print("\n[4] DATA HAZARD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Hazard tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_hazard_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 3, 2, 1, 5)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("RAW hazard handled correctly", regout[0] == 10 and regout[1] == 15,
                    f"r2={regout[0]}, r3={regout[1]}, expected r2=10, r3=15")
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("Decode stalls reported", stats.get('decode_stall', 0) > 0, f"decode_stall={stats.get('decode_stall')}")
    shutil.rmtree(test_dir)

def test_branches_and_delay_slot(suite: TestSuite):
    print("\n[5] BRANCH AND DELAY SLOT TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Branch tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_branch_")
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 5), encode_instruction(OP_BEQ, 2, 1, 1, 5),
             encode_instruction(OP_ADD, 3, 0, 1, 100), encode_instruction(OP_ADD, 4, 0, 1, 200),
             encode_instruction(OP_ADD, 5, 0, 1, 300), encode_instruction(OP_ADD, 6, 0, 1, 50)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Branch jumps to correct target", regout[4] == 50, f"r6={regout[4]}, expected 50")
    suite.add_result("Delay slot instruction executes", regout[1] == 100, f"r3={regout[1]}, expected 100")
    suite.add_result("Instructions after delay slot skipped", regout[2] == 0 and regout[3] == 0,
                    f"r4={regout[2]}, r5={regout[3]}, expected 0")
    shutil.rmtree(test_dir)

def test_r1_update(suite: TestSuite):
    print("\n[6] R1 IMMEDIATE UPDATE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("R1 tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_r1_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 42), encode_instruction(OP_ADD, 3, 0, 1, 100),
                           encode_instruction(OP_ADD, 4, 0, 1, -5)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("R1 holds first immediate correctly", regout[0] == 42, f"r2={regout[0]}, expected 42")
    suite.add_result("R1 updates with second immediate", regout[1] == 100, f"r3={regout[1]}, expected 100")
    suite.add_result("R1 handles negative immediate", regout[2] == -5, f"r4={regout[2]}, expected -5")
    shutil.rmtree(test_dir)

def test_load_store(suite: TestSuite):
    print("\n[7] LOAD/STORE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Load/Store tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_ldst_")
    # Use a positive value that stays positive after sign extension
    # 0x7FF = 2047 (max positive 12-bit), 0x100 = 256 (safely positive)
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 0x100), encode_instruction(OP_SW, 2, 0, 1, 16),
                           encode_instruction(OP_LW, 3, 0, 1, 16)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    # 0x100 = 256, positive, no sign extension issue
    suite.add_result("Store source register correct", regout[0] == 256, f"r2={regout[0]}, expected 256")
    suite.add_result("Load returns stored value", regout[1] == 256, f"r3={regout[1]}, expected 256")
    shutil.rmtree(test_dir)

def test_bus_and_latency(suite: TestSuite):
    print("\n[8] BUS TRACE AND LATENCY TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Bus tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_bus_")
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 8)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    memin = ["00000000"] * 8 + ["DEADBEEF"]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    bus_trace = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    if bus_trace:
        suite.add_result("BusRd issued for cache miss", bus_trace[0]['cmd'] == 1, f"First cmd={bus_trace[0]['cmd']}")
        flush_count = sum(1 for t in bus_trace if t['cmd'] == 3)
        suite.add_result("8 Flush commands for block transfer", flush_count == 8, f"Got {flush_count} Flush commands")
        first_flush = [t for t in bus_trace if t['cmd'] == 3]
        if first_flush:
            latency = first_flush[0]['cycle'] - bus_trace[0]['cycle']
            suite.add_result("16-cycle memory latency", latency == 16, f"Latency was {latency} cycles")
    else:
        suite.add_result("BusRd issued for cache miss", False, "No bus trace")
    shutil.rmtree(test_dir)

def test_mesi_transitions(suite: TestSuite):
    print("\n[9] MESI STATE TRANSITION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("MESI tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mesi_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 42), encode_instruction(OP_SW, 2, 0, 1, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    tsram = read_tsram(os.path.join(test_dir, "tsram0.txt"))
    if tsram:
        mesi_state = tsram[0][0]
        suite.add_result("Write creates Modified state", mesi_state == 3, f"MESI state={mesi_state}, expected 3")
    else:
        suite.add_result("Write creates Modified state", False, "No tsram data")
    shutil.rmtree(test_dir)

def test_multicore_basic(suite: TestSuite):
    print("\n[10] MULTI-CORE BASIC TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Multi-core tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_multi_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, (i+1)*10)]) for i in range(4)]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    results = [read_regout(os.path.join(test_dir, f"regout{i}.txt"))[0] for i in range(4)]
    expected = [10, 20, 30, 40]
    suite.add_result("All 4 cores execute independently", results == expected,
                    f"Got {results}, expected {expected}")
    shutil.rmtree(test_dir)

# ============================================================================
# Exhaustive Opcode Tests
# ============================================================================

def test_all_alu_opcodes(suite: TestSuite):
    print("\n[11] EXHAUSTIVE ALU OPCODE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("ALU opcode tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_alu_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 3, 0, 1, 3),
        encode_instruction(OP_ADD, 4, 2, 3, 0), encode_instruction(OP_SUB, 5, 2, 3, 0),
        encode_instruction(OP_AND, 6, 2, 3, 0), encode_instruction(OP_OR, 7, 2, 3, 0),
        encode_instruction(OP_XOR, 8, 2, 3, 0), encode_instruction(OP_MUL, 9, 2, 3, 0),
        encode_instruction(OP_ADD, 10, 0, 1, 1), encode_instruction(OP_SLL, 11, 2, 10, 0),
        encode_instruction(OP_SRA, 12, 2, 10, 0), encode_instruction(OP_SRL, 13, 2, 10, 0),
    ]), make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    expected = {'ADD': (regout[2], 13), 'SUB': (regout[3], 7), 'AND': (regout[4], 2),
                'OR': (regout[5], 11), 'XOR': (regout[6], 9), 'MUL': (regout[7], 30),
                'SLL': (regout[9], 20), 'SRA': (regout[10], 5), 'SRL': (regout[11], 5)}
    for name, (actual, exp) in expected.items():
        suite.add_result(f"Opcode {name}", actual == exp, f"Expected {exp}, got {actual}")
    shutil.rmtree(test_dir)

def test_alu_edge_cases(suite: TestSuite):
    print("\n[12] ALU EDGE CASE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("ALU edge tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_edge_")
    imem = [pad_with_halts([
        encode_instruction(OP_ADD, 2, 0, 1, 0), encode_instruction(OP_ADD, 3, 2, 2, 0),
        encode_instruction(OP_ADD, 4, 0, 1, MAX_INT_12), encode_instruction(OP_ADD, 5, 0, 1, MIN_INT_12),
        encode_instruction(OP_MUL, 6, 4, 2, 0), encode_instruction(OP_ADD, 7, 4, 5, 0),
        encode_instruction(OP_SUB, 8, 4, 4, 0), encode_instruction(OP_OR, 9, 4, 2, 0),
        encode_instruction(OP_AND, 10, 4, 2, 0), encode_instruction(OP_XOR, 11, 4, 4, 0),
    ]), make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    test_cases = [("ADD with zero", regout[1], 0), ("ADD max positive imm", regout[2], MAX_INT_12),
                  ("MUL by zero", regout[4], 0), ("ADD positive + negative", regout[5], -1),
                  ("SUB to zero", regout[6], 0), ("OR with zero", regout[7], MAX_INT_12),
                  ("AND with zero", regout[8], 0), ("XOR with self", regout[9], 0)]
    for name, actual, expected in test_cases:
        suite.add_result(name, actual == expected, f"Expected {expected}, got {actual}")
    shutil.rmtree(test_dir)

def test_all_branch_opcodes(suite: TestSuite):
    print("\n[13] ALL BRANCH OPCODE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Branch opcode tests", False, "No simulator binary")
        return
    branch_tests = [
        (OP_BEQ, 5, 5, True, "BEQ equal"), (OP_BEQ, 5, 3, False, "BEQ not equal"),
        (OP_BNE, 5, 3, True, "BNE not equal"), (OP_BNE, 5, 5, False, "BNE equal"),
        (OP_BLT, 3, 5, True, "BLT less"), (OP_BLT, 5, 3, False, "BLT not less"),
        (OP_BGT, 5, 3, True, "BGT greater"), (OP_BGT, 3, 5, False, "BGT not greater"),
        (OP_BLE, 3, 5, True, "BLE less"), (OP_BLE, 5, 5, True, "BLE equal"),
        (OP_BGE, 5, 3, True, "BGE greater"), (OP_BGE, 5, 5, True, "BGE equal"),
    ]
    for opcode, rs_val, rt_val, should_branch, name in branch_tests:
        test_dir = tempfile.mkdtemp(prefix="sim_br_")
        imem = [[encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 3, 0, 1, rs_val),
                 encode_instruction(OP_ADD, 4, 0, 1, rt_val), encode_instruction(opcode, 2, 3, 4, 0),
                 encode_instruction(OP_ADD, 5, 0, 1, 111), encode_instruction(OP_ADD, 6, 0, 1, 222),
                 encode_instruction(OP_HALT, 0, 0, 0, 0)] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 3 +
                [encode_instruction(OP_ADD, 7, 0, 1, 333)] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
                make_halt_only(), make_halt_only(), make_halt_only()]
        create_test_files(test_dir, imem)
        run_simulator(suite.sim_path, test_dir)
        regout = read_regout(os.path.join(test_dir, "regout0.txt"))
        r5, r6, r7 = regout[3], regout[4], regout[5]
        if should_branch:
            branch_ok = (r7 == 333 and r6 == 0)
        else:
            branch_ok = (r6 == 222 and r7 == 0)
        suite.add_result(name, r5 == 111 and branch_ok, f"r5={r5}, r6={r6}, r7={r7}")
        shutil.rmtree(test_dir)

def test_branch_signed_comparison(suite: TestSuite):
    print("\n[14] BRANCH SIGNED COMPARISON TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Signed comparison tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_signed_")
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 3, 0, 1, -1),
             encode_instruction(OP_ADD, 4, 0, 1, 1), encode_instruction(OP_BLT, 2, 3, 4, 0),
             encode_instruction(OP_ADD, 5, 0, 1, 111), encode_instruction(OP_ADD, 6, 0, 1, 222),
             encode_instruction(OP_HALT, 0, 0, 0, 0)] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 3 +
            [encode_instruction(OP_ADD, 7, 0, 1, 333)] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("BLT uses signed comparison (-1 < 1)", regout[5] == 333 and regout[4] == 0,
                    f"r6={regout[4]}, r7={regout[5]}")
    shutil.rmtree(test_dir)

def test_jal_instruction(suite: TestSuite):
    print("\n[15] JAL INSTRUCTION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("JAL tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_jal_")
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 5), encode_instruction(OP_JAL, 2, 0, 0, 0),
             encode_instruction(OP_ADD, 3, 0, 1, 111), encode_instruction(OP_ADD, 4, 0, 1, 222),
             encode_instruction(OP_ADD, 5, 0, 1, 333), encode_instruction(OP_ADD, 6, 0, 1, 50)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("JAL saves return address to R15", regout[13] == 2, f"R15={regout[13]}, expected 2")
    suite.add_result("JAL jumps to target", regout[4] == 50, f"R6={regout[4]}, expected 50")
    suite.add_result("JAL delay slot executes", regout[1] == 111, f"R3={regout[1]}, expected 111")
    shutil.rmtree(test_dir)

def test_load_word_variations(suite: TestSuite):
    print("\n[16] LOAD WORD VARIATIONS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("LW tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_lw_")
    memin = ["00000000"] * 32
    memin[0], memin[8], memin[16] = "DEADBEEF", "12345678", "FFFFFFFF"
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0), encode_instruction(OP_LW, 3, 0, 1, 8),
                           encode_instruction(OP_LW, 4, 0, 1, 16)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("LW from address 0", regout[0] == to_signed_32(0xDEADBEEF), f"got {regout[0]}")
    suite.add_result("LW from address 8", regout[1] == 0x12345678, f"got {regout[1]}")
    suite.add_result("LW reads -1 correctly", regout[2] == -1, f"got {regout[2]}")
    shutil.rmtree(test_dir)

def test_store_word_variations(suite: TestSuite):
    """Test SW instruction with various scenarios.
    NOTE: With write-back cache, Modified data stays in cache until eviction.
    We verify the data is in the cache (dsram), not necessarily in memout.
    """
    print("\n[17] STORE WORD VARIATIONS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("SW tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_sw_")
    # Use positive values to avoid signed/unsigned confusion
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 0x123), encode_instruction(OP_SW, 2, 0, 1, 0),
                           encode_instruction(OP_LW, 3, 0, 1, 0),  # Read back to verify
                           encode_instruction(OP_ADD, 4, 0, 1, 0x456), encode_instruction(OP_SW, 4, 0, 1, 8),
                           encode_instruction(OP_LW, 5, 0, 1, 8)]),  # Read back to verify
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    # Verify via load-back (cache should have the data)
    suite.add_result("SW to address 0 (verified via LW)", regout[1] == 0x123, f"r3={regout[1]}, expected 0x123")
    suite.add_result("SW to address 8 (verified via LW)", regout[3] == 0x456, f"r5={regout[3]}, expected 0x456")
    shutil.rmtree(test_dir)

# ============================================================================
# Register Integrity Tests
# ============================================================================

def test_r0_immutable(suite: TestSuite):
    print("\n[18] R0 IMMUTABILITY TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("R0 tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_r0_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 0, 0, 1, 42), encode_instruction(OP_ADD, 2, 0, 1, 0),
                           encode_instruction(OP_SUB, 0, 0, 1, -10), encode_instruction(OP_ADD, 3, 0, 1, 0),
                           encode_instruction(OP_ADD, 4, 0, 1, 5), encode_instruction(OP_MUL, 0, 4, 4, 0),
                           encode_instruction(OP_ADD, 5, 0, 1, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("R0 unchanged after ADD write attempt", regout[0] == 0, f"r2={regout[0]}")
    suite.add_result("R0 unchanged after SUB write attempt", regout[1] == 0, f"r3={regout[1]}")
    suite.add_result("R0 unchanged after MUL write attempt", regout[3] == 0, f"r5={regout[3]}")
    shutil.rmtree(test_dir)

def test_r1_immediate_update_complex(suite: TestSuite):
    print("\n[19] R1 COMPLEX IMMEDIATE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("R1 complex tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_r1_complex_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, MAX_INT_12),
                           encode_instruction(OP_ADD, 3, 0, 1, MIN_INT_12),
                           encode_instruction(OP_ADD, 4, 0, 1, -1),
                           encode_instruction(OP_ADD, 5, 0, 1, -128),
                           encode_instruction(OP_ADD, 6, 0, 1, 0),
                           encode_instruction(OP_ADD, 7, 0, 1, 100),
                           encode_instruction(OP_ADD, 8, 7, 1, 50)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("R1 max positive (2047)", regout[0] == MAX_INT_12, f"got {regout[0]}")
    suite.add_result("R1 min negative (-2048)", regout[1] == -2048, f"got {regout[1]}")
    suite.add_result("R1 -1", regout[2] == -1, f"got {regout[2]}")
    suite.add_result("R1 updates between instructions", regout[6] == 150, f"got {regout[6]}")
    shutil.rmtree(test_dir)

# ============================================================================
# Pipeline Hazard Stress Tests
# ============================================================================

def test_hazard_distance_1(suite: TestSuite):
    print("\n[20] DISTANCE-1 HAZARD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Distance-1 hazard tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_haz1_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 3, 2, 1, 5),
                           encode_instruction(OP_ADD, 4, 3, 1, 3), encode_instruction(OP_ADD, 5, 4, 1, 2)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    expected = [10, 15, 18, 20]
    for i, exp in enumerate(expected):
        suite.add_result(f"Distance-1 chain step {i+1}", regout[i] == exp, f"Expected {exp}, got {regout[i]}")
    shutil.rmtree(test_dir)

def test_hazard_distance_2(suite: TestSuite):
    print("\n[21] DISTANCE-2 HAZARD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Distance-2 hazard tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_haz2_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 6, 0, 1, 1),
                           encode_instruction(OP_ADD, 3, 2, 1, 5), encode_instruction(OP_ADD, 7, 0, 1, 2),
                           encode_instruction(OP_ADD, 4, 3, 1, 3)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Distance-2: r2 = 10", regout[0] == 10, f"got {regout[0]}")
    suite.add_result("Distance-2: r3 = 15", regout[1] == 15, f"got {regout[1]}")
    suite.add_result("Distance-2: r4 = 18", regout[2] == 18, f"got {regout[2]}")
    shutil.rmtree(test_dir)

def test_load_use_hazard(suite: TestSuite):
    print("\n[22] LOAD-USE HAZARD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Load-use hazard tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_loaduse_")
    memin = ["00000000"] * 8 + ["00000064"]
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 8), encode_instruction(OP_ADD, 3, 2, 1, 5)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Load returns correct value", regout[0] == 100, f"r2={regout[0]}")
    suite.add_result("Load-use hazard handled correctly", regout[1] == 105, f"r3={regout[1]}")
    shutil.rmtree(test_dir)

def test_branch_hazard(suite: TestSuite):
    print("\n[23] BRANCH WITH HAZARD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Branch hazard tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_brhaz_")
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_BEQ, 2, 1, 1, 10),
             encode_instruction(OP_ADD, 3, 0, 1, 111), encode_instruction(OP_ADD, 4, 0, 1, 222)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 6 + [encode_instruction(OP_ADD, 5, 0, 1, 333)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Branch hazard on target register handled", regout[3] == 333 and regout[2] == 0,
                    f"r4={regout[2]}, r5={regout[3]}")
    shutil.rmtree(test_dir)

def test_store_hazard(suite: TestSuite):
    print("\n[24] STORE WITH HAZARD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Store hazard tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_swhaz_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 42), encode_instruction(OP_SW, 2, 0, 1, 0),
                           encode_instruction(OP_LW, 3, 0, 1, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Store hazard on data register handled", regout[1] == 42, f"r3={regout[1]}")
    shutil.rmtree(test_dir)

# ============================================================================
# Advanced MESI Coherence Tests
# ============================================================================

def test_mesi_ping_pong(suite: TestSuite):
    print("\n[25] MESI PING-PONG TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Ping-pong tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_pingpong_")
    core0_prog = [encode_instruction(OP_ADD, 2, 0, 1, 1), encode_instruction(OP_SW, 2, 0, 1, 0)] + \
                 [encode_instruction(OP_ADD, 6, 0, 1, 0)] * 5 + \
                 [encode_instruction(OP_ADD, 2, 0, 1, 3), encode_instruction(OP_SW, 2, 0, 1, 0)]
    core1_prog = [encode_instruction(OP_ADD, 6, 0, 1, 0)] * 3 + \
                 [encode_instruction(OP_ADD, 2, 0, 1, 2), encode_instruction(OP_SW, 2, 0, 1, 0)] + \
                 [encode_instruction(OP_ADD, 6, 0, 1, 0)] * 5 + \
                 [encode_instruction(OP_ADD, 2, 0, 1, 4), encode_instruction(OP_SW, 2, 0, 1, 0)]
    imem = [pad_with_halts(core0_prog), pad_with_halts(core1_prog), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir, timeout=120)
    bus_trace = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    busrdx_count = sum(1 for t in bus_trace if t['cmd'] == 2)
    suite.add_result("Multiple BusRdX for ping-pong writes", busrdx_count >= 2, f"BusRdX count: {busrdx_count}")
    shutil.rmtree(test_dir)

def test_mesi_shared_state(suite: TestSuite):
    print("\n[26] MESI SHARED STATE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Shared state tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_shared_")
    memin = ["00000000"] * 8 + ["DEADBEEF"]
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 8)]),
            pad_with_halts([encode_instruction(OP_ADD, 6, 0, 1, 0)] * 3 + [encode_instruction(OP_LW, 2, 0, 1, 8)]),
            make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout0 = read_regout(os.path.join(test_dir, "regout0.txt"))
    regout1 = read_regout(os.path.join(test_dir, "regout1.txt"))
    suite.add_result("Core 0 reads correct value", regout0[0] == to_signed_32(0xDEADBEEF), f"got {regout0[0]}")
    suite.add_result("Core 1 reads same value", regout1[0] == to_signed_32(0xDEADBEEF), f"got {regout1[0]}")
    bus_trace = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    busrd_with_shared = [t for t in bus_trace if t['cmd'] == 1 and t['shared'] == 1]
    suite.add_result("bus_shared set for second read", len(busrd_with_shared) >= 1, "No BusRd with shared=1 found")
    shutil.rmtree(test_dir)

def test_mesi_modified_flush(suite: TestSuite):
    print("\n[27] MESI MODIFIED FLUSH TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Modified flush tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mflush_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 0x42), encode_instruction(OP_SW, 2, 0, 1, 0)]),
            pad_with_halts([encode_instruction(OP_ADD, 6, 0, 1, 0)] * 5 + [encode_instruction(OP_LW, 3, 0, 1, 0)]),
            make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout1 = read_regout(os.path.join(test_dir, "regout1.txt"))
    suite.add_result("Core 1 reads Core 0's modified data", regout1[1] == 0x42, f"got {regout1[1]}")
    bus_trace = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    flush_from_core0 = [t for t in bus_trace if t['cmd'] == 3 and t['origid'] == 0]
    suite.add_result("Core 0 flushes on snoop", len(flush_from_core0) >= 1, f"Found {len(flush_from_core0)} flush(es)")
    shutil.rmtree(test_dir)

def test_cache_eviction_writeback(suite: TestSuite):
    print("\n[28] CACHE EVICTION WRITEBACK TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Eviction tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_evict_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 0xAB), encode_instruction(OP_SW, 2, 0, 1, 0),
                           encode_instruction(OP_ADD, 3, 0, 1, 0xCD), encode_instruction(OP_ADD, 4, 0, 1, 512),
                           encode_instruction(OP_SW, 3, 4, 0, 0), encode_instruction(OP_LW, 5, 0, 1, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Eviction writes back Modified data", regout[3] == 0xAB, f"r5={regout[3]}")
    memout = read_memout(os.path.join(test_dir, "memout.txt"))
    suite.add_result("Memory contains written back value", len(memout) > 0 and memout[0] == 0xAB,
                    f"mem[0]={memout[0] if memout else 'N/A'}")
    shutil.rmtree(test_dir)

def test_busrdx_invalidates_shared(suite: TestSuite):
    print("\n[29] BusRdX INVALIDATION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("BusRdX invalidation tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_busrdx_")
    memin = ["00000064"]
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0)] + [encode_instruction(OP_ADD, 6, 0, 1, 0)] * 8 +
                          [encode_instruction(OP_LW, 3, 0, 1, 0)]),
            pad_with_halts([encode_instruction(OP_ADD, 6, 0, 1, 0)] * 3 + [encode_instruction(OP_ADD, 2, 0, 1, 200),
                           encode_instruction(OP_SW, 2, 0, 1, 0)]),
            make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout0 = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("First read gets original value", regout0[0] == 100, f"got {regout0[0]}")
    suite.add_result("Second read gets updated value after invalidation", regout0[1] == 200, f"got {regout0[1]}")
    shutil.rmtree(test_dir)

# ============================================================================
# Edge Case Tests
# ============================================================================

def test_consecutive_loads_same_block(suite: TestSuite):
    print("\n[30] CONSECUTIVE LOAD TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Consecutive load tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_consload_")
    memin = [f"{i:08X}" for i in range(1, 9)]
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 0), encode_instruction(OP_LW, 3, 0, 1, 1),
                           encode_instruction(OP_LW, 4, 0, 1, 2), encode_instruction(OP_LW, 5, 0, 1, 7)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("All loads return correct values", regout[:4] == [1, 2, 3, 8], f"got {regout[:4]}")
    suite.add_result("Only one cache miss for block", stats.get('read_miss', 0) == 1, f"read_miss={stats.get('read_miss')}")
    suite.add_result("Cache hits for same block", stats.get('read_hit', 0) >= 3, f"read_hit={stats.get('read_hit')}")
    shutil.rmtree(test_dir)

def test_halt_behavior(suite: TestSuite):
    """Test that HALT properly stops core after pipeline drains.
    NOTE: When HALT is fetched, subsequent instructions may already be in pipeline.
    These instructions WILL complete (pipeline drain) - this is correct behavior.
    Example files pad with HALTs for exactly this reason.
    """
    print("\n[31] HALT BEHAVIOR TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("HALT tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_halt_")
    # Test that HALT eventually stops execution
    # Use multiple HALTs to ensure clean termination
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 42),
             encode_instruction(OP_HALT, 0, 0, 0, 0),
             encode_instruction(OP_HALT, 0, 0, 0, 0),
             encode_instruction(OP_HALT, 0, 0, 0, 0),
             encode_instruction(OP_HALT, 0, 0, 0, 0),
             encode_instruction(OP_HALT, 0, 0, 0, 0)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    suite.add_result("Instruction before HALT executes", regout[0] == 42, f"r2={regout[0]}")
    suite.add_result("Simulation terminates", stats.get('cycles', 0) > 0 and stats.get('cycles', 0) < 100,
                    f"cycles={stats.get('cycles')}")
    shutil.rmtree(test_dir)

def test_sign_extension_negative_immediate(suite: TestSuite):
    print("\n[32] SIGN EXTENSION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Sign extension tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_signext_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, -1), encode_instruction(OP_ADD, 3, 0, 1, -128),
                           encode_instruction(OP_ADD, 4, 0, 1, -2048), encode_instruction(OP_ADD, 5, 0, 1, -256)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Sign extend -1", regout[0] == -1, f"got {regout[0]}")
    suite.add_result("Sign extend -128", regout[1] == -128, f"got {regout[1]}")
    suite.add_result("Sign extend -2048", regout[2] == -2048, f"got {regout[2]}")
    suite.add_result("Sign extend -256", regout[3] == -256, f"got {regout[3]}")
    shutil.rmtree(test_dir)

def test_sra_sign_preservation(suite: TestSuite):
    print("\n[33] SRA SIGN PRESERVATION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("SRA tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_sra_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, -16), encode_instruction(OP_ADD, 3, 0, 1, 2),
                           encode_instruction(OP_SRA, 4, 2, 3, 0), encode_instruction(OP_ADD, 5, 0, 1, -1),
                           encode_instruction(OP_ADD, 6, 0, 1, 4), encode_instruction(OP_SRA, 7, 5, 6, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("SRA -16 >> 2 = -4", regout[2] == -4, f"got {regout[2]}")
    suite.add_result("SRA -1 >> 4 = -1 (sign fills)", regout[5] == -1, f"got {regout[5]}")
    shutil.rmtree(test_dir)

def test_mul_result(suite: TestSuite):
    print("\n[34] MUL RESULT TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("MUL tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_mul_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 0x100), encode_instruction(OP_MUL, 3, 2, 2, 0),
                           encode_instruction(OP_ADD, 4, 0, 1, -1), encode_instruction(OP_ADD, 5, 0, 1, 5),
                           encode_instruction(OP_MUL, 6, 4, 5, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("MUL 256 * 256 = 65536", regout[1] == 65536, f"got {regout[1]}")
    suite.add_result("MUL -1 * 5 = -5", regout[4] == -5, f"got {regout[4]}")
    shutil.rmtree(test_dir)

def test_statistics_accuracy(suite: TestSuite):
    print("\n[35] STATISTICS ACCURACY TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Statistics tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_stats_")
    memin = ["00000000"] * 16 + ["DEADBEEF"]
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 10), encode_instruction(OP_ADD, 3, 2, 1, 5),
                           encode_instruction(OP_LW, 4, 0, 1, 16), encode_instruction(OP_LW, 5, 0, 1, 17)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    # Note: HALT is NOT counted as an instruction per spec
    suite.add_result("Instruction count correct", stats.get('instructions', 0) == 4, f"got {stats.get('instructions')}")
    suite.add_result("Read miss count correct", stats.get('read_miss', 0) == 1, f"got {stats.get('read_miss')}")
    suite.add_result("Read hit count correct", stats.get('read_hit', 0) >= 1, f"got {stats.get('read_hit')}")
    suite.add_result("Decode stalls counted", stats.get('decode_stall', 0) > 0, f"got {stats.get('decode_stall')}")
    suite.add_result("Mem stalls counted", stats.get('mem_stall', 0) > 0, f"got {stats.get('mem_stall')}")
    shutil.rmtree(test_dir)

def test_four_core_contention(suite: TestSuite):
    print("\n[36] FOUR-CORE CONTENTION TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Four-core contention tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_4core_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, (i+1)*100), encode_instruction(OP_SW, 2, 0, 1, 0)])
            for i in range(4)]
    create_test_files(test_dir, imem, ["00000000"])
    run_simulator(suite.sim_path, test_dir)
    bus_trace = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    cores_with_busrdx = set(t['origid'] for t in bus_trace if t['cmd'] == 2)
    suite.add_result("Multiple cores issue BusRdX", len(cores_with_busrdx) >= 2, f"Cores: {cores_with_busrdx}")
    memout = read_memout(os.path.join(test_dir, "memout.txt"))
    suite.add_result("Final memory has valid value", memout and memout[0] in [100, 200, 300, 400],
                    f"mem[0]={memout[0] if memout else 'N/A'}")
    shutil.rmtree(test_dir)

def test_jal_return(suite: TestSuite):
    print("\n[37] JAL RETURN TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("JAL return tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_jalret_")
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 5), encode_instruction(OP_JAL, 2, 0, 0, 0),
             encode_instruction(OP_ADD, 3, 0, 1, 111), encode_instruction(OP_ADD, 4, 0, 1, 222),
             encode_instruction(OP_HALT, 0, 0, 0, 0), encode_instruction(OP_ADD, 5, 0, 1, 333),
             encode_instruction(OP_JAL, 15, 0, 0, 0), encode_instruction(OP_ADD, 6, 0, 1, 444)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 5,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("First delay slot executes", regout[1] == 111, f"r3={regout[1]}")
    suite.add_result("After return executes", regout[2] == 222, f"r4={regout[2]}")
    suite.add_result("Subroutine executes", regout[3] == 333, f"r5={regout[3]}")
    suite.add_result("Return delay slot executes", regout[4] == 444, f"r6={regout[4]}")
    shutil.rmtree(test_dir)

def test_shift_edge_cases(suite: TestSuite):
    print("\n[38] SHIFT EDGE CASE TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Shift edge tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_shift_")
    imem = [pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, -8), encode_instruction(OP_ADD, 3, 0, 1, 0),
                           encode_instruction(OP_ADD, 4, 0, 1, 2), encode_instruction(OP_SLL, 5, 2, 3, 0),
                           encode_instruction(OP_SRL, 6, 2, 4, 0), encode_instruction(OP_ADD, 7, 0, 1, 1),
                           encode_instruction(OP_SLL, 8, 7, 7, 0)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("SLL by 0 (no change)", regout[3] == -8, f"got {regout[3]}")
    suite.add_result("SRL negative by 2", regout[4] == 0x3FFFFFFE, f"got 0x{regout[4] & 0xFFFFFFFF:08X}")
    suite.add_result("SLL 1 by 1", regout[6] == 2, f"got {regout[6]}")
    shutil.rmtree(test_dir)

def test_branch_target_10_bits(suite: TestSuite):
    print("\n[39] BRANCH TARGET 10-BIT TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Branch 10-bit tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_br10_")
    imem = [[encode_instruction(OP_ADD, 2, 0, 1, 0x405), encode_instruction(OP_BEQ, 2, 1, 1, 0x405),
             encode_instruction(OP_ADD, 3, 0, 1, 111), encode_instruction(OP_ADD, 4, 0, 1, 222),
             encode_instruction(OP_HALT, 0, 0, 0, 0), encode_instruction(OP_ADD, 5, 0, 1, 333)] +
            [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Branch uses lower 10 bits of rd", regout[3] == 333 and regout[2] == 0,
                    f"r4={regout[2]}, r5={regout[3]}")
    shutil.rmtree(test_dir)

def test_memory_boundary_addresses(suite: TestSuite):
    print("\n[40] MEMORY BOUNDARY TESTS")
    print("-" * 40)
    if not suite.sim_path:
        suite.add_result("Memory boundary tests", False, "No simulator binary")
        return
    test_dir = tempfile.mkdtemp(prefix="sim_membound_")
    memin = ["00000000"] * 64
    memin[7], memin[8], memin[15] = "00000007", "00000008", "0000000F"
    imem = [pad_with_halts([encode_instruction(OP_LW, 2, 0, 1, 7), encode_instruction(OP_LW, 3, 0, 1, 8),
                           encode_instruction(OP_LW, 4, 0, 1, 15)]),
            make_halt_only(), make_halt_only(), make_halt_only()]
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    regout = read_regout(os.path.join(test_dir, "regout0.txt"))
    suite.add_result("Load from end of block 0", regout[0] == 7, f"got {regout[0]}")
    suite.add_result("Load from start of block 1", regout[1] == 8, f"got {regout[1]}")
    suite.add_result("Load from end of block 1", regout[2] == 15, f"got {regout[2]}")
    shutil.rmtree(test_dir)

# ============================================================================
# Main
# ============================================================================

def main():
    print("=" * 60)
    print("MULTI-CORE MESI SIMULATOR - EXHAUSTIVE TEST SUITE")
    print("=" * 60)
    
    source_path = None
    search_paths = [SIM_SOURCE, f"./{SIM_SOURCE}", f"/home/claude/{SIM_SOURCE}",
                   f"/mnt/user-data/uploads/{SIM_SOURCE}", f"/mnt/user-data/outputs/{SIM_SOURCE}"]
    for path in search_paths:
        if os.path.exists(path):
            source_path = path
            break
    
    if not source_path:
        print(f"ERROR: Cannot find {SIM_SOURCE}")
        print(f"Searched: {search_paths}")
        return 1
    
    print(f"Found source at: {source_path}")
    suite = TestSuite()
    
    sim_path = test_compilation(suite, source_path)
    suite.sim_path = sim_path
    
    if not sim_path:
        print("\nCompilation failed. Cannot run remaining tests.")
        suite.summary()
        return 1
    
    # Run all tests
    test_basic_execution(suite)
    test_output_formats(suite)
    test_data_hazards(suite)
    test_branches_and_delay_slot(suite)
    test_r1_update(suite)
    test_load_store(suite)
    test_bus_and_latency(suite)
    test_mesi_transitions(suite)
    test_multicore_basic(suite)
    test_all_alu_opcodes(suite)
    test_alu_edge_cases(suite)
    test_all_branch_opcodes(suite)
    test_branch_signed_comparison(suite)
    test_jal_instruction(suite)
    test_load_word_variations(suite)
    test_store_word_variations(suite)
    test_r0_immutable(suite)
    test_r1_immediate_update_complex(suite)
    test_hazard_distance_1(suite)
    test_hazard_distance_2(suite)
    test_load_use_hazard(suite)
    test_branch_hazard(suite)
    test_store_hazard(suite)
    test_mesi_ping_pong(suite)
    test_mesi_shared_state(suite)
    test_mesi_modified_flush(suite)
    test_cache_eviction_writeback(suite)
    test_busrdx_invalidates_shared(suite)
    test_consecutive_loads_same_block(suite)
    test_halt_behavior(suite)
    test_sign_extension_negative_immediate(suite)
    test_sra_sign_preservation(suite)
    test_mul_result(suite)
    test_statistics_accuracy(suite)
    test_four_core_contention(suite)
    test_jal_return(suite)
    test_shift_edge_cases(suite)
    test_branch_target_10_bits(suite)
    test_memory_boundary_addresses(suite)
    
    success = suite.summary()
    
    if sim_path and os.path.exists(os.path.dirname(sim_path)):
        try:
            shutil.rmtree(os.path.dirname(sim_path))
        except:
            pass
    
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())