# MESI Multi-Core Cache Coherence Simulator

A cycle-accurate simulator for a 4-core in-order processor with private caches kept coherent by the MESI protocol, written in C99. Computer Architecture course project (0512.4461), Tel Aviv University, Semester A.

**Daniel Simanovsky**, with a bus-trace fix from Liad Levy

## What this is

Four identical cores, each with its own 5-stage pipeline and private data cache, share one 2M-word main memory over a single bus. Nothing enforces correctness for you — if a core reads stale data because another core modified the same cache line and the coherence logic doesn't invalidate it in time, the simulator will happily produce a wrong answer with no error. The assignment was to implement the MESI protocol and the pipeline hazard handling correctly enough that four cores incrementing a shared counter 128 times each land on exactly 512, and that a matrix multiplication split across four cores produces the same result as one core doing it alone — just faster.

Everything here — the ISA, the pipeline stages, the cache geometry, the bus fields — follows a fixed spec (`project_assignment.pdf`, in Hebrew) rather than being an open design; the interesting part was building a bug-free implementation of it and being able to prove that in writing.

## Architecture

Each of the 4 cores has:

- 16 general-purpose registers (`R0` and `R1` hardwired to 0), a private 1024-word instruction memory, and its own PC.
- A direct-mapped data cache: 512 words, 8 words/block → 64 cache lines, each tagged with a MESI state (`Invalid` / `Shared` / `Exclusive` / `Modified`) plus a 12-bit tag, stored in a separate tag SRAM (`TSRAM`) from the data SRAM (`DSRAM`) — mirroring how the spec wants the two split.
- A 5-stage in-order pipeline: **Fetch → Decode → Execute → Mem → Writeback**, with a branch delay slot and no forwarding/bypassing — a value written by an instruction is only visible to instructions behind it starting the *next* cycle, so RAW hazards are resolved purely by stalling the Decode stage rather than by a bypass network. Branches resolve in Decode.

All four caches connect to main memory over one shared bus, arbitrated round-robin (whichever core waited longest since its last grant goes first). A cache miss issues `BusRd` (shared read) or `BusRdX` (exclusive read, for writes); a modified line owned by another core answers with a `Flush` that updates both the requester and main memory in parallel. Main memory itself takes 16 cycles to respond to the first word of a block and then streams the remaining 7 words on the following cycles. Every core snoops every bus transaction each cycle and asserts `bus_shared` if it holds the requested block, which is how a requester decides whether to land in `Shared` or `Exclusive` state.

21-instruction ISA (arithmetic, logical, shifts, 6 branch conditions, `jal`, `lw`/`sw`, `halt`), fixed-width instructions: `[opcode:8][rd:4][rs:4][rt:4][imm:12]`, one word each. Full opcode table and file formats are documented in `mesi_simulator_new/README.txt` and the spec PDF.

## Test programs

Three workloads, each exercising a different part of the protocol:

- **`counter`** — all 4 cores take turns incrementing a shared memory location, 128 times each, coordinated through a memory-based turn flag. This forces the same cache line to bounce between all 4 caches in `Modified` state hundreds of times in a row; the final value in memory must land on exactly 512. It ends by reading a different address to force a conflict miss and flush the line back to memory.
- **`mulserial`** — a 16×16 × 16×16 matrix multiplication run entirely on core 0, as a correctness and performance baseline.
- **`mulparallel`** — the same multiplication with the work split row-wise across all 4 cores, sharing the same input matrices (read-heavy, so lines mostly settle into `Shared` state) and writing disjoint output regions.

Running all three against this implementation:

| Test | Result |
|---|---|
| `counter` | memory location 0 = **512** (exactly right — no lost or duplicated increments across 512 total RMW cycles) |
| `mulserial` | 170,061 cycles on core 0 |
| `mulparallel` | 48,545 cycles (max across cores) — **3.5× speedup** over serial, on 4 cores |

## Verification

Correctness on a cycle-accurate multi-core simulator is hard to eyeball, so alongside `sim.c` there's a ~1,650-line Python test suite (`test.py`) that checks the implementation against the spec directly rather than just diffing output files: exact output file formats and field widths, all 21 instructions including edge cases, every MESI state transition, snooping and the `bus_shared` line, round-robin bus arbitration and the 16-cycle/8-word memory timing, and all 8 statistics counters. Run against this repo:

```
python test.py -t .
```

```
SUMMARY: 199/199 passed, 0 failed
```

`sim.c` also compiles clean with `-std=c99 -Wall -Werror` — zero warnings.

## Building and running

```bash
gcc -o sim.exe sim.c -std=c99 -Wall -Werror
```

The simulator takes 27 positional arguments — 4 instruction memory files, a main memory input file, and 22 output files (final memory image, per-core register dumps, per-core pipeline traces, a bus trace, per-core `DSRAM`/`TSRAM` dumps, and per-core statistics):

```bash
./sim.exe imem0.txt imem1.txt imem2.txt imem3.txt memin.txt memout.txt \
    regout0.txt regout1.txt regout2.txt regout3.txt \
    core0trace.txt core1trace.txt core2trace.txt core3trace.txt bustrace.txt \
    dsram0.txt dsram1.txt dsram2.txt dsram3.txt \
    tsram0.txt tsram1.txt tsram2.txt tsram3.txt \
    stats0.txt stats1.txt stats2.txt stats3.txt
```

Each `mesi_simulator_new/<test>/` folder already contains its own `imem0-3.txt` and `memin.txt`, so the simplest way to run a test is to build once and copy the binary in:

```bash
cd mesi_simulator_new
gcc -o sim.exe sim.c -std=c99 -Wall -Werror
cp sim.exe counter/ && cd counter
./sim.exe imem0.txt imem1.txt imem2.txt imem3.txt memin.txt memout.txt regout0.txt regout1.txt regout2.txt regout3.txt core0trace.txt core1trace.txt core2trace.txt core3trace.txt bustrace.txt dsram0.txt dsram1.txt dsram2.txt dsram3.txt tsram0.txt tsram1.txt tsram2.txt tsram3.txt stats0.txt stats1.txt stats2.txt stats3.txt
head -c 8 memout.txt   # -> 00000200 (512 decimal): every increment landed
```

## Repository layout

```
mesi_simulator_new/
├── sim.c                  # the simulator: cores, pipeline, cache, MESI bus, main memory
├── test.py                # ~1,650-line verification suite (not part of the assignment)
├── counter/                # test A: coherence / conflict-miss stress test
├── mulserial/               # test B: single-core matrix multiply baseline
├── mulparallel/              # test B: 4-core parallel matrix multiply
├── example_061225_win/     # instructor-provided worked example
└── README.txt              # original quick-start notes
project_assignment.pdf     # course spec (architecture, ISA, bus protocol, grading criteria)
general.txt                # build/run/git command cheatsheet
```

Each test directory holds an annotated `.asm` source alongside the hex-encoded `.txt` memory images the simulator actually reads — the assembler is manual/by-hand per the assignment, not a separate tool.

## Course context

Built for the Computer Architecture course (0512.4461) at Tel Aviv University, Semester A. Grading was split between the automated conflict-miss/coherence check, a cache-performance check requiring correct output *and* a speedup floor on the parallel matrix multiply, and manual code review against the spec.
