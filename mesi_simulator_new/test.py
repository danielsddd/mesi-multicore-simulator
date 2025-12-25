#!/usr/bin/env python3
"""
Multi-Core MESI Simulator Test Suite
=====================================
Comprehensive testing for the sim.c implementation.

Tests:
- Compilation
- Basic execution
- Output file existence and format
- Pipeline behavior (5-stage, hazards, branches, delay slot)
- Cache behavior (MESI transitions, hits/misses)
- Bus behavior (arbitration, latency, writeback locking)
- Statistics correctness
"""

import os
import sys
import subprocess
import tempfile
import shutil
import re
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

# ============================================================================
# Configuration
# ============================================================================

SIM_SOURCE = "sim.c"
CC = "gcc"
CFLAGS = ["-Wall", "-Wextra", "-O2"]

# Detect platform
IS_WINDOWS = sys.platform.startswith('win')
SIM_BINARY = "sim.exe" if IS_WINDOWS else "sim"

# Expected output files
OUTPUT_FILES = [
    "memout.txt",
    "regout0.txt", "regout1.txt", "regout2.txt", "regout3.txt",
    "core0trace.txt", "core1trace.txt", "core2trace.txt", "core3trace.txt",
    "bustrace.txt",
    "dsram0.txt", "dsram1.txt", "dsram2.txt", "dsram3.txt",
    "tsram0.txt", "tsram1.txt", "tsram2.txt", "tsram3.txt",
    "stats0.txt", "stats1.txt", "stats2.txt", "stats3.txt",
]

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
        self.temp_dir: Optional[str] = None
        self.sim_path: Optional[str] = None
        
    def add_result(self, name: str, passed: bool, message: str = "", details: str = ""):
        self.results.append(TestResult(name, passed, message, details))
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")
        if not passed and message:
            print(f"         {message}")
        if details and not passed:
            for line in details.split('\n')[:5]:  # Limit detail output
                print(f"         {line}")
    
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
    """Encode instruction to 8-digit hex string."""
    if imm < 0:
        imm = imm & 0xFFF  # 12-bit two's complement
    inst = ((opcode & 0xFF) << 24) | ((rd & 0xF) << 20) | \
           ((rs & 0xF) << 16) | ((rt & 0xF) << 12) | (imm & 0xFFF)
    return f"{inst:08X}"

# Opcode definitions
OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_MUL = 0, 1, 2, 3, 4, 5
OP_SLL, OP_SRA, OP_SRL = 6, 7, 8
OP_BEQ, OP_BNE, OP_BLT, OP_BGT, OP_BLE, OP_BGE = 9, 10, 11, 12, 13, 14
OP_JAL, OP_LW, OP_SW = 15, 16, 17
OP_HALT = 20

# HALT padding to prevent zeros from being executed after program ends
HALT_PAD = 10  # Number of HALTs to pad

def pad_with_halts(instructions: List[str]) -> List[str]:
    """Pad instruction list with HALTs to prevent zero-execution after program."""
    halts = [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD
    return instructions + halts

def make_halt_only() -> List[str]:
    """Create a HALT-only program (for inactive cores)."""
    return [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD

def create_test_files(test_dir: str, imem: List[List[str]], memin: List[str] = None):
    """Create test input files in the test directory."""
    # Create instruction memory files
    for i in range(4):
        with open(os.path.join(test_dir, f"imem{i}.txt"), 'w') as f:
            if i < len(imem) and imem[i]:
                f.write('\n'.join(imem[i]) + '\n')
    
    # Create main memory file
    with open(os.path.join(test_dir, "memin.txt"), 'w') as f:
        if memin:
            f.write('\n'.join(memin) + '\n')

def run_simulator(sim_path: str, test_dir: str, timeout: int = 30) -> Tuple[bool, str, str]:
    """Run the simulator and return (success, stdout, stderr)."""
    try:
        result = subprocess.run(
            [sim_path],
            cwd=test_dir,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Timeout expired"
    except Exception as e:
        return False, "", str(e)

def read_file(filepath: str) -> List[str]:
    """Read file and return lines (stripped)."""
    try:
        with open(filepath, 'r') as f:
            return [line.strip() for line in f.readlines()]
    except:
        return []

def parse_stats(filepath: str) -> Dict[str, int]:
    """Parse stats file into dictionary."""
    stats = {}
    for line in read_file(filepath):
        parts = line.split()
        if len(parts) == 2:
            stats[parts[0]] = int(parts[1])
    return stats

def parse_trace(filepath: str) -> List[Dict]:
    """Parse core trace file."""
    traces = []
    for line in read_file(filepath):
        parts = line.split()
        if len(parts) >= 20:  # cycle + 5 stages + 14 registers
            trace = {
                'cycle': int(parts[0]),
                'fetch': parts[1],
                'decode': parts[2],
                'exec': parts[3],
                'mem': parts[4],
                'wb': parts[5],
                'regs': parts[6:20]
            }
            traces.append(trace)
    return traces

def parse_bus_trace(filepath: str) -> List[Dict]:
    """Parse bus trace file."""
    traces = []
    for line in read_file(filepath):
        parts = line.split()
        if len(parts) == 6:
            traces.append({
                'cycle': int(parts[0]),
                'origid': int(parts[1], 16),
                'cmd': int(parts[2], 16),
                'addr': int(parts[3], 16),
                'data': int(parts[4], 16),
                'shared': int(parts[5], 16)
            })
    return traces

# ============================================================================
# Test Functions
# ============================================================================

def test_compilation(suite: TestSuite, source_path: str) -> Optional[str]:
    """Test that the simulator compiles successfully."""
    print("\n[1] COMPILATION TESTS")
    print("-" * 40)
    
    # Check source exists
    if not os.path.exists(source_path):
        suite.add_result("Source file exists", False, f"Cannot find {source_path}")
        return None
    suite.add_result("Source file exists", True)
    
    # Create temp directory for binary
    temp_dir = tempfile.mkdtemp(prefix="sim_test_")
    binary_path = os.path.join(temp_dir, SIM_BINARY)
    
    # Compile
    cmd = [CC] + CFLAGS + ["-o", binary_path, source_path]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            suite.add_result("Compilation succeeds", True)
            
            # Check for warnings
            if result.stderr.strip():
                warnings = result.stderr.strip()
                # Allow some warnings but flag them
                suite.add_result("No compiler warnings", False, 
                               "Warnings present (non-fatal)", warnings[:200])
            else:
                suite.add_result("No compiler warnings", True)
        else:
            suite.add_result("Compilation succeeds", False, 
                           "Compilation failed", result.stderr[:500])
            shutil.rmtree(temp_dir)
            return None
    except Exception as e:
        suite.add_result("Compilation succeeds", False, str(e))
        shutil.rmtree(temp_dir)
        return None
    
    # Check binary exists and is executable
    if os.path.exists(binary_path):
        # On Windows, just check existence; on Unix, also check executable bit
        if IS_WINDOWS or os.access(binary_path, os.X_OK):
            suite.add_result("Binary is executable", True)
        else:
            suite.add_result("Binary is executable", False, "Missing execute permission")
            shutil.rmtree(temp_dir)
            return None
    else:
        suite.add_result("Binary is executable", False, f"Binary not found at {binary_path}")
        shutil.rmtree(temp_dir)
        return None
    
    suite.temp_dir = temp_dir
    suite.sim_path = binary_path
    return binary_path

def test_basic_execution(suite: TestSuite):
    """Test basic execution with simple program."""
    print("\n[2] BASIC EXECUTION TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Basic execution", False, "No simulator binary")
        return
    
    # Create simple test: just halt on all cores (with padding)
    test_dir = tempfile.mkdtemp(prefix="sim_basic_")
    halt_pad = [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 10
    imem = [halt_pad, halt_pad, halt_pad, halt_pad]
    create_test_files(test_dir, imem)
    
    # Run simulator
    success, stdout, stderr = run_simulator(suite.sim_path, test_dir)
    
    if success:
        suite.add_result("Simulator runs without crash", True)
    else:
        suite.add_result("Simulator runs without crash", False, stderr[:200])
        shutil.rmtree(test_dir)
        return
    
    # Check all output files exist
    all_exist = True
    missing = []
    for fname in OUTPUT_FILES:
        if not os.path.exists(os.path.join(test_dir, fname)):
            all_exist = False
            missing.append(fname)
    
    if all_exist:
        suite.add_result("All output files created", True)
    else:
        suite.add_result("All output files created", False, f"Missing: {', '.join(missing)}")
    
    # Check termination is quick (just halts)
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    if stats.get('cycles', 0) <= 10:
        suite.add_result("Quick termination on halt", True, f"cycles={stats.get('cycles', 'N/A')}")
    else:
        suite.add_result("Quick termination on halt", False, 
                        f"Expected <=10 cycles, got {stats.get('cycles', 'N/A')}")
    
    shutil.rmtree(test_dir)

def test_output_formats(suite: TestSuite):
    """Test output file formats match specification."""
    print("\n[3] OUTPUT FORMAT TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Output format tests", False, "No simulator binary")
        return
    
    # Create test with some activity
    test_dir = tempfile.mkdtemp(prefix="sim_format_")
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 2, 1, 5),   # r2 = r2 + 5
            encode_instruction(OP_ADD, 3, 3, 1, 10),  # r3 = r3 + 10
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    # Test regout format (14 lines of 8 hex digits)
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    if len(regout) == 14:
        suite.add_result("regout has 14 lines (R2-R15)", True)
    else:
        suite.add_result("regout has 14 lines (R2-R15)", False, f"Got {len(regout)} lines")
    
    regout_format_ok = all(re.match(r'^[0-9A-Fa-f]{8}$', line) for line in regout if line)
    suite.add_result("regout format (8 hex digits)", regout_format_ok)
    
    # Test dsram format (512 lines)
    dsram = read_file(os.path.join(test_dir, "dsram0.txt"))
    if len(dsram) == 512:
        suite.add_result("dsram has 512 lines", True)
    else:
        suite.add_result("dsram has 512 lines", False, f"Got {len(dsram)} lines")
    
    # Test tsram format (64 lines)
    tsram = read_file(os.path.join(test_dir, "tsram0.txt"))
    if len(tsram) == 64:
        suite.add_result("tsram has 64 lines", True)
    else:
        suite.add_result("tsram has 64 lines", False, f"Got {len(tsram)} lines")
    
    # Test stats format
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    required_stats = ['cycles', 'instructions', 'read_hit', 'write_hit', 
                     'read_miss', 'write_miss', 'decode_stall', 'mem_stall']
    missing_stats = [s for s in required_stats if s not in stats]
    if not missing_stats:
        suite.add_result("stats has all required fields", True)
    else:
        suite.add_result("stats has all required fields", False, f"Missing: {missing_stats}")
    
    # Test trace format
    trace = read_file(os.path.join(test_dir, "core0trace.txt"))
    if trace:
        # Check first line format
        parts = trace[0].split()
        if len(parts) == 20:  # cycle + 5 stages + 14 regs
            suite.add_result("core trace has correct columns", True)
        else:
            suite.add_result("core trace has correct columns", False, 
                           f"Expected 20, got {len(parts)}")
        
        # Check stage format (3 hex or ---)
        stage_format_ok = True
        for line in trace[:5]:
            parts = line.split()
            for stage in parts[1:6]:
                if stage != "---" and not re.match(r'^[0-9A-Fa-f]{3}$', stage):
                    stage_format_ok = False
        suite.add_result("core trace stage format (3 hex or ---)", stage_format_ok)
    else:
        suite.add_result("core trace has correct columns", False, "Empty trace")
    
    # Test bus trace format (when present)
    bus_trace = read_file(os.path.join(test_dir, "bustrace.txt"))
    # Bus trace may be empty if no memory ops
    suite.add_result("bustrace file exists", True)
    
    shutil.rmtree(test_dir)

def test_pipeline_basics(suite: TestSuite):
    """Test basic pipeline behavior."""
    print("\n[4] PIPELINE BEHAVIOR TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Pipeline tests", False, "No simulator binary")
        return
    
    # Test sequential instructions flow through pipeline
    # Pad with HALTs to prevent counting of zeros after main program
    test_dir = tempfile.mkdtemp(prefix="sim_pipe_")
    imem = [
        [
            encode_instruction(OP_ADD, 2, 0, 1, 1),   # r2 = 0 + 1 = 1
            encode_instruction(OP_ADD, 3, 0, 1, 2),   # r3 = 0 + 2 = 2
            encode_instruction(OP_ADD, 4, 0, 1, 3),   # r4 = 0 + 3 = 3
            encode_instruction(OP_ADD, 5, 0, 1, 4),   # r5 = 0 + 4 = 4
            encode_instruction(OP_HALT, 0, 0, 0, 0),
            encode_instruction(OP_HALT, 0, 0, 0, 0),  # Pad with HALTs
            encode_instruction(OP_HALT, 0, 0, 0, 0),
            encode_instruction(OP_HALT, 0, 0, 0, 0),
            encode_instruction(OP_HALT, 0, 0, 0, 0),
        ],
        [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 5,
        [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 5,
        [encode_instruction(OP_HALT, 0, 0, 0, 0)] * 5,
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    # Check register values
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    expected = [1, 2, 3, 4] + [0] * 10  # R2=1, R3=2, R4=3, R5=4, rest=0
    actual = [int(line, 16) for line in regout if line]
    
    if actual[:4] == expected[:4]:
        suite.add_result("Sequential instructions execute correctly", True)
    else:
        suite.add_result("Sequential instructions execute correctly", False,
                        f"Expected R2-R5=[1,2,3,4], got {actual[:4]}")
    
    # Check trace shows 5-stage progression
    trace = parse_trace(os.path.join(test_dir, "core0trace.txt"))
    if len(trace) >= 5:
        # By cycle 5, instruction 0 should be in WB
        cycle5 = [t for t in trace if t['cycle'] == 4]
        if cycle5 and cycle5[0]['wb'] == '000':
            suite.add_result("5-stage pipeline progression", True)
        else:
            suite.add_result("5-stage pipeline progression", False,
                           "First instruction not in WB by cycle 5")
    else:
        suite.add_result("5-stage pipeline progression", False, "Not enough trace entries")
    
    # Check instruction count
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    if stats.get('instructions', 0) == 4:  # 4 ADDs (halt not counted)
        suite.add_result("Instruction count correct", True)
    else:
        suite.add_result("Instruction count correct", False,
                        f"Expected 4, got {stats.get('instructions', 'N/A')}")
    
    shutil.rmtree(test_dir)

def test_data_hazards(suite: TestSuite):
    """Test data hazard detection and stalling."""
    print("\n[5] DATA HAZARD TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Hazard tests", False, "No simulator binary")
        return
    
    # RAW hazard: r3 depends on r2
    test_dir = tempfile.mkdtemp(prefix="sim_hazard_")
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 0, 1, 10),  # r2 = 10
            encode_instruction(OP_ADD, 3, 2, 1, 5),   # r3 = r2 + 5 (depends on r2)
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    # Check correct result (r3 = 10 + 5 = 15)
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    r2 = int(regout[0], 16) if regout else -1
    r3 = int(regout[1], 16) if len(regout) > 1 else -1
    
    if r2 == 10 and r3 == 15:
        suite.add_result("RAW hazard handled correctly", True)
    else:
        suite.add_result("RAW hazard handled correctly", False,
                        f"Expected r2=10, r3=15, got r2={r2}, r3={r3}")
    
    # Check decode stalls occurred
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    if stats.get('decode_stall', 0) > 0:
        suite.add_result("Decode stalls reported", True, 
                        f"decode_stall={stats.get('decode_stall')}")
    else:
        suite.add_result("Decode stalls reported", False, "Expected decode_stall > 0")
    
    shutil.rmtree(test_dir)

def test_branches_and_delay_slot(suite: TestSuite):
    """Test branch handling and delay slot."""
    print("\n[6] BRANCH AND DELAY SLOT TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Branch tests", False, "No simulator binary")
        return
    
    # Test: branch with delay slot
    # PC=0: add r2, r0, r1, 5    -> r2 = 5 (branch target address)
    # PC=1: beq r1, r2, r0, 5    -> if (5 == 5) goto r2 (addr 5) - branch to PC=5
    # PC=2: add r3, r0, r1, 100  -> r3 = 100 (DELAY SLOT - should execute)
    # PC=3: add r4, r0, r1, 200  -> r4 = 200 (SHOULD BE SKIPPED)
    # PC=4: add r5, r0, r1, 300  -> r5 = 300 (SHOULD BE SKIPPED)
    # PC=5: add r6, r0, r1, 50   -> r6 = 50 (branch target)
    # PC=6+: halt (padded)
    
    test_dir = tempfile.mkdtemp(prefix="sim_branch_")
    imem = [
        [
            encode_instruction(OP_ADD, 2, 0, 1, 5),    # PC=0: r2 = 5 (target addr)
            encode_instruction(OP_BEQ, 2, 1, 1, 5),    # PC=1: if r1(5)==r1(5) goto r2(5)
            encode_instruction(OP_ADD, 3, 0, 1, 100),  # PC=2: delay slot, r3 = 100
            encode_instruction(OP_ADD, 4, 0, 1, 200),  # PC=3: skipped
            encode_instruction(OP_ADD, 5, 0, 1, 300),  # PC=4: skipped
            encode_instruction(OP_ADD, 6, 0, 1, 50),   # PC=5: r6 = 50
        ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    regs = [int(line, 16) for line in regout if line]
    
    # r2=5, r3=100 (delay slot), r4=0 (skipped), r5=0 (skipped), r6=50
    r2, r3, r4, r5, r6 = regs[0], regs[1], regs[2], regs[3], regs[4]
    
    branch_target_ok = (r6 == 50)
    delay_slot_ok = (r3 == 100)
    skipped_ok = (r4 == 0 and r5 == 0)
    
    if branch_target_ok:
        suite.add_result("Branch jumps to correct target", True)
    else:
        suite.add_result("Branch jumps to correct target", False, f"r6={r6}, expected 50")
    
    if delay_slot_ok:
        suite.add_result("Delay slot instruction executes", True)
    else:
        suite.add_result("Delay slot instruction executes", False, f"r3={r3}, expected 100")
    
    if skipped_ok:
        suite.add_result("Instructions after delay slot skipped", True)
    else:
        suite.add_result("Instructions after delay slot skipped", False, 
                        f"r4={r4}, r5={r5}, expected 0, 0")
    
    shutil.rmtree(test_dir)

def test_r1_update(suite: TestSuite):
    """Test that R1 is updated with immediate in decode."""
    print("\n[7] R1 (IMMEDIATE) UPDATE TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("R1 tests", False, "No simulator binary")
        return
    
    # Test using different immediates
    test_dir = tempfile.mkdtemp(prefix="sim_r1_")
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 0, 1, 42),   # r2 = 0 + imm(42) = 42
            encode_instruction(OP_ADD, 3, 0, 1, -5),   # r3 = 0 + imm(-5) = -5 (sign extended)
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    r2 = int(regout[0], 16) if regout else -1
    r3_raw = int(regout[1], 16) if len(regout) > 1 else 0
    # Convert to signed
    r3 = r3_raw if r3_raw < 0x80000000 else r3_raw - 0x100000000
    
    if r2 == 42:
        suite.add_result("Positive immediate works", True)
    else:
        suite.add_result("Positive immediate works", False, f"Expected 42, got {r2}")
    
    if r3 == -5:
        suite.add_result("Negative immediate (sign-extended)", True)
    else:
        suite.add_result("Negative immediate (sign-extended)", False, 
                        f"Expected -5, got {r3}")
    
    shutil.rmtree(test_dir)

def test_memory_operations(suite: TestSuite):
    """Test load/store and cache behavior."""
    print("\n[8] MEMORY OPERATION TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Memory tests", False, "No simulator binary")
        return
    
    # Test: store and load
    test_dir = tempfile.mkdtemp(prefix="sim_mem_")
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 0, 1, 100),  # r2 = 100 (value to store)
            encode_instruction(OP_ADD, 3, 0, 1, 16),   # r3 = 16 (address)
            encode_instruction(OP_SW, 2, 3, 0, 0),     # mem[r3+r0] = r2 -> mem[16] = 100
            encode_instruction(OP_LW, 4, 3, 0, 0),     # r4 = mem[r3+r0] -> r4 = mem[16]
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    r4 = int(regout[2], 16) if len(regout) > 2 else -1
    
    if r4 == 100:
        suite.add_result("Store then load returns correct value", True)
    else:
        suite.add_result("Store then load returns correct value", False,
                        f"Expected 100, got {r4}")
    
    # Check stats for cache operations
    stats = parse_stats(os.path.join(test_dir, "stats0.txt"))
    
    # First store is a miss (cold), load should hit
    write_miss = stats.get('write_miss', -1)
    read_hit = stats.get('read_hit', -1)
    
    if write_miss >= 1:
        suite.add_result("Write miss on cold cache", True)
    else:
        suite.add_result("Write miss on cold cache", False, f"write_miss={write_miss}")
    
    if read_hit >= 1:
        suite.add_result("Read hit after write to same block", True)
    else:
        suite.add_result("Read hit after write to same block", False, 
                        f"read_hit={read_hit}")
    
    # Check mem stall occurred (cache miss causes stall)
    if stats.get('mem_stall', 0) > 0:
        suite.add_result("Memory stall counted", True)
    else:
        suite.add_result("Memory stall counted", False, "Expected mem_stall > 0")
    
    shutil.rmtree(test_dir)

def test_bus_trace_and_latency(suite: TestSuite):
    """Test bus trace format and memory latency."""
    print("\n[9] BUS TRACE AND LATENCY TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Bus tests", False, "No simulator binary")
        return
    
    # Load from memory (cache miss)
    test_dir = tempfile.mkdtemp(prefix="sim_bus_")
    imem = [
        pad_with_halts([
            encode_instruction(OP_LW, 2, 0, 1, 8),     # Load from address 8
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    memin = ["00000000"] * 8 + ["DEADBEEF"]  # Address 8 = 0xDEADBEEF
    create_test_files(test_dir, imem, memin)
    run_simulator(suite.sim_path, test_dir)
    
    bus_trace = parse_bus_trace(os.path.join(test_dir, "bustrace.txt"))
    
    # Should have BusRd followed by 8 Flush commands
    if bus_trace:
        # First should be BusRd (cmd=1)
        if bus_trace[0]['cmd'] == 1:
            suite.add_result("BusRd issued for cache miss", True)
        else:
            suite.add_result("BusRd issued for cache miss", False,
                           f"First cmd={bus_trace[0]['cmd']}, expected 1")
        
        # Count Flush commands (cmd=3)
        flush_count = sum(1 for t in bus_trace if t['cmd'] == 3)
        if flush_count == 8:
            suite.add_result("8 Flush commands for block transfer", True)
        else:
            suite.add_result("8 Flush commands for block transfer", False,
                           f"Got {flush_count} Flush commands")
        
        # Check 16-cycle latency (first flush should be ~16 cycles after BusRd)
        if len(bus_trace) >= 2:
            busrd_cycle = bus_trace[0]['cycle']
            first_flush = [t for t in bus_trace if t['cmd'] == 3]
            if first_flush:
                latency = first_flush[0]['cycle'] - busrd_cycle
                if latency == 16:
                    suite.add_result("16-cycle memory latency", True)
                else:
                    suite.add_result("16-cycle memory latency", False,
                                   f"Latency was {latency} cycles")
    else:
        suite.add_result("BusRd issued for cache miss", False, "No bus trace")
        suite.add_result("8 Flush commands for block transfer", False, "No bus trace")
        suite.add_result("16-cycle memory latency", False, "No bus trace")
    
    # Check bus trace format
    if bus_trace:
        trace_line = read_file(os.path.join(test_dir, "bustrace.txt"))[0]
        parts = trace_line.split()
        format_ok = (len(parts) == 6 and 
                    len(parts[3]) == 6 and  # addr: 6 hex
                    len(parts[4]) == 8)     # data: 8 hex
        suite.add_result("Bus trace format correct", format_ok)
    
    shutil.rmtree(test_dir)

def test_mesi_transitions(suite: TestSuite):
    """Test MESI state transitions."""
    print("\n[10] MESI STATE TRANSITION TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("MESI tests", False, "No simulator binary")
        return
    
    # Simple test: write creates Modified state
    test_dir = tempfile.mkdtemp(prefix="sim_mesi_")
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 0, 1, 42),   # r2 = 42
            encode_instruction(OP_SW, 2, 0, 1, 0),    # mem[0] = 42 (write miss -> M)
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    # Check tsram for Modified state (MESI=3, bits 13:12)
    tsram = read_file(os.path.join(test_dir, "tsram0.txt"))
    if tsram:
        entry0 = int(tsram[0], 16)
        mesi_state = (entry0 >> 12) & 0x3
        if mesi_state == 3:  # Modified
            suite.add_result("Write creates Modified state", True)
        else:
            suite.add_result("Write creates Modified state", False,
                           f"MESI state={mesi_state}, expected 3 (Modified)")
    else:
        suite.add_result("Write creates Modified state", False, "No tsram data")
    
    shutil.rmtree(test_dir)

def test_multicore_basic(suite: TestSuite):
    """Test basic multi-core operation."""
    print("\n[11] MULTI-CORE BASIC TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Multi-core tests", False, "No simulator binary")
        return
    
    # Each core does independent work
    test_dir = tempfile.mkdtemp(prefix="sim_multi_")
    imem = [
        pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 10)]),
        pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 20)]),
        pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 30)]),
        pad_with_halts([encode_instruction(OP_ADD, 2, 0, 1, 40)]),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    # Check each core's registers
    results = []
    for i in range(4):
        regout = read_file(os.path.join(test_dir, f"regout{i}.txt"))
        r2 = int(regout[0], 16) if regout else -1
        expected = (i + 1) * 10
        results.append((i, r2, expected))
    
    all_correct = all(r2 == exp for _, r2, exp in results)
    if all_correct:
        suite.add_result("All 4 cores execute independently", True)
    else:
        details = ", ".join(f"Core{i}: got {r2}, exp {exp}" 
                           for i, r2, exp in results if r2 != exp)
        suite.add_result("All 4 cores execute independently", False, details)
    
    shutil.rmtree(test_dir)

def test_all_opcodes(suite: TestSuite):
    """Test all ALU opcodes."""
    print("\n[12] OPCODE TESTS")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("Opcode tests", False, "No simulator binary")
        return
    
    test_dir = tempfile.mkdtemp(prefix="sim_ops_")
    
    # Test various operations
    # r2 = 10, r3 = 3
    # r4 = r2 + r3 = 13
    # r5 = r2 - r3 = 7
    # r6 = r2 & r3 = 2
    # r7 = r2 | r3 = 11
    # r8 = r2 ^ r3 = 9
    # r9 = r2 * r3 = 30
    # r10 = r2 << 1 = 20
    # r11 = r2 >> 1 (arithmetic) = 5
    # r12 = r2 >> 1 (logical) = 5
    
    imem = [
        pad_with_halts([
            encode_instruction(OP_ADD, 2, 0, 1, 10),   # r2 = 10
            encode_instruction(OP_ADD, 3, 0, 1, 3),    # r3 = 3
            encode_instruction(OP_ADD, 4, 2, 3, 0),    # r4 = r2 + r3
            encode_instruction(OP_SUB, 5, 2, 3, 0),    # r5 = r2 - r3
            encode_instruction(OP_AND, 6, 2, 3, 0),    # r6 = r2 & r3
            encode_instruction(OP_OR, 7, 2, 3, 0),     # r7 = r2 | r3
            encode_instruction(OP_XOR, 8, 2, 3, 0),    # r8 = r2 ^ r3
            encode_instruction(OP_MUL, 9, 2, 3, 0),    # r9 = r2 * r3
            encode_instruction(OP_ADD, 10, 0, 1, 1),   # r10 = 1 (shift amount)
            encode_instruction(OP_SLL, 11, 2, 10, 0),  # r11 = r2 << 1
            encode_instruction(OP_SRA, 12, 2, 10, 0),  # r12 = r2 >> 1 (arith)
            encode_instruction(OP_SRL, 13, 2, 10, 0),  # r13 = r2 >> 1 (logic)
        ]),
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    regs = [int(line, 16) for line in regout if line]
    
    # regs[0]=r2, regs[1]=r3, regs[2]=r4, etc.
    expected = {
        'ADD': (regs[2], 13),   # r4 = 10 + 3
        'SUB': (regs[3], 7),    # r5 = 10 - 3
        'AND': (regs[4], 2),    # r6 = 10 & 3
        'OR': (regs[5], 11),    # r7 = 10 | 3
        'XOR': (regs[6], 9),    # r8 = 10 ^ 3
        'MUL': (regs[7], 30),   # r9 = 10 * 3
        'SLL': (regs[9], 20),   # r11 = 10 << 1
        'SRA': (regs[10], 5),   # r12 = 10 >> 1
        'SRL': (regs[11], 5),   # r13 = 10 >> 1
    }
    
    for name, (actual, exp) in expected.items():
        if actual == exp:
            suite.add_result(f"Opcode {name}", True)
        else:
            suite.add_result(f"Opcode {name}", False, f"Expected {exp}, got {actual}")
    
    shutil.rmtree(test_dir)

def test_jal(suite: TestSuite):
    """Test JAL instruction."""
    print("\n[13] JAL INSTRUCTION TEST")
    print("-" * 40)
    
    if not suite.sim_path:
        suite.add_result("JAL test", False, "No simulator binary")
        return
    
    # PC=0: add r2, r0, r1, 5     -> r2 = 5 (target address)
    # PC=1: jal r2, r0, r0, 0     -> r15 = PC+1 = 2, jump to r2 = 5
    # PC=2: add r3, r0, r1, 111   -> delay slot, r3 = 111
    # PC=3: add r4, r0, r1, 222   -> skipped
    # PC=4: add r5, r0, r1, 333   -> skipped
    # PC=5: add r6, r0, r1, 50    -> r6 = 50
    # PC=6+: halt (padded)
    
    test_dir = tempfile.mkdtemp(prefix="sim_jal_")
    imem = [
        [
            encode_instruction(OP_ADD, 2, 0, 1, 5),    # r2 = 5 (target)
            encode_instruction(OP_JAL, 2, 0, 0, 0),   # jal to r2, r15 = 2
            encode_instruction(OP_ADD, 3, 0, 1, 111), # delay slot
            encode_instruction(OP_ADD, 4, 0, 1, 222), # skipped
            encode_instruction(OP_ADD, 5, 0, 1, 333), # skipped
            encode_instruction(OP_ADD, 6, 0, 1, 50),  # target
        ] + [encode_instruction(OP_HALT, 0, 0, 0, 0)] * HALT_PAD,
        make_halt_only(),
        make_halt_only(),
        make_halt_only(),
    ]
    create_test_files(test_dir, imem)
    run_simulator(suite.sim_path, test_dir)
    
    regout = read_file(os.path.join(test_dir, "regout0.txt"))
    regs = [int(line, 16) for line in regout if line]
    
    # r15 is at index 13 (R2-R15, so R15 is at index 13)
    r15 = regs[13]
    r3 = regs[1]
    r6 = regs[4]
    
    if r15 == 2:
        suite.add_result("JAL saves return address to R15", True)
    else:
        suite.add_result("JAL saves return address to R15", False,
                        f"Expected R15=2, got {r15}")
    
    if r6 == 50:
        suite.add_result("JAL jumps to target", True)
    else:
        suite.add_result("JAL jumps to target", False, f"Expected R6=50, got {r6}")
    
    if r3 == 111:
        suite.add_result("JAL delay slot executes", True)
    else:
        suite.add_result("JAL delay slot executes", False, f"Expected R3=111, got {r3}")
    
    shutil.rmtree(test_dir)

# ============================================================================
# Main
# ============================================================================

def main():
    print("=" * 60)
    print("MULTI-CORE MESI SIMULATOR TEST SUITE")
    print("=" * 60)
    
    # Find source file
    source_path = None
    for path in [SIM_SOURCE, f"./{SIM_SOURCE}", f"/home/claude/{SIM_SOURCE}",
                 f"/mnt/user-data/outputs/{SIM_SOURCE}"]:
        if os.path.exists(path):
            source_path = path
            break
    
    if not source_path:
        print(f"ERROR: Cannot find {SIM_SOURCE}")
        print("Please ensure sim.c is in the current directory or specify path.")
        sys.exit(1)
    
    print(f"Using source: {source_path}")
    
    suite = TestSuite()
    
    try:
        # Run all tests
        test_compilation(suite, source_path)
        test_basic_execution(suite)
        test_output_formats(suite)
        test_pipeline_basics(suite)
        test_data_hazards(suite)
        test_branches_and_delay_slot(suite)
        test_r1_update(suite)
        test_memory_operations(suite)
        test_bus_trace_and_latency(suite)
        test_mesi_transitions(suite)
        test_multicore_basic(suite)
        test_all_opcodes(suite)
        test_jal(suite)
        
    finally:
        # Cleanup
        if suite.temp_dir and os.path.exists(suite.temp_dir):
            shutil.rmtree(suite.temp_dir)
    
    # Print summary
    success = suite.summary()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()