/*
 * Multi-Core MESI Cache Coherency Simulator
 * 
 * Implements:
 * - 4 symmetric cores with 5-stage pipelines
 * - Private instruction memory and data cache per core
 * - MESI cache coherency protocol
 * - Round-robin bus arbitration
 */

 #define _CRT_SECURE_NO_WARNINGS // Disable secure warnings for compatibility
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * CONFIGURATION CONSTANTS
 * ============================================================================ */

#define NUM_CORES           4
#define NUM_REGISTERS       16
#define IMEM_SIZE           1024
#define MAIN_MEM_SIZE       (1 << 21)   /* 2^21 words */
#define CACHE_SIZE          512         /* words */
#define BLOCK_SIZE          8           /* words per block */
#define NUM_CACHE_BLOCKS    64          /* 512 / 8 */
#define MEM_LATENCY         16          /* cycles for first word */

/* Compile-time assertion macro for verifying configuration constants */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define STATIC_ASSERT(cond, msg) _Static_assert(cond, msg)
#else
#define STATIC_ASSERT(cond, msg) typedef char static_assertion_##__LINE__[(cond) ? 1 : -1]
#endif

/* Ensure cache parameters are correct for the direct-mapped cache design */
STATIC_ASSERT(BLOCK_SIZE == 8, "BLOCK_SIZE must be 8 words");
STATIC_ASSERT(NUM_CACHE_BLOCKS == 64, "NUM_CACHE_BLOCKS must be 64 lines");

/* ============================================================================
 * OPCODE DEFINITIONS
 * ============================================================================ */

/* 
 * Instruction opcodes for the simulated processor ISA.
 * Opcodes 0-8 are arithmetic/logical operations,
 * Opcodes 9-15 are branch/jump instructions,
 * Opcodes 16-17 are memory operations,
 * Opcode 20 is the halt instruction.
 */
typedef enum {
    OP_ADD  = 0,    /* Addition: rd = rs + rt */
    OP_SUB  = 1,    /* Subtraction: rd = rs - rt */
    OP_AND  = 2,    /* Bitwise AND: rd = rs & rt */
    OP_OR   = 3,    /* Bitwise OR: rd = rs | rt */
    OP_XOR  = 4,    /* Bitwise XOR: rd = rs ^ rt */
    OP_MUL  = 5,    /* Multiplication: rd = rs * rt */
    OP_SLL  = 6,    /* Shift left logical: rd = rs << rt */
    OP_SRA  = 7,    /* Shift right arithmetic: rd = rs >> rt (sign-extended) */
    OP_SRL  = 8,    /* Shift right logical: rd = rs >> rt (zero-filled) */
    OP_BEQ  = 9,    /* Branch if equal: if (rs == rt) PC = rd */
    OP_BNE  = 10,   /* Branch if not equal: if (rs != rt) PC = rd */
    OP_BLT  = 11,   /* Branch if less than: if (rs < rt) PC = rd */
    OP_BGT  = 12,   /* Branch if greater than: if (rs > rt) PC = rd */
    OP_BLE  = 13,   /* Branch if less or equal: if (rs <= rt) PC = rd */
    OP_BGE  = 14,   /* Branch if greater or equal: if (rs >= rt) PC = rd */
    OP_JAL  = 15,   /* Jump and link: R15 = PC+1, PC = rd */
    OP_LW   = 16,   /* Load word: rd = Memory[rs + rt] */
    OP_SW   = 17,   /* Store word: Memory[rs + rt] = rd */
    OP_HALT = 20    /* Halt execution of this core */
} Opcode;

/* 
 * MESI Protocol States
 * The MESI protocol is a cache coherency protocol that ensures
 * data consistency across multiple caches in a multiprocessor system.
 * Each cache line can be in one of four states.
 */
typedef enum {
    MESI_INVALID   = 0,  /* Block is not valid - must fetch from memory/other cache */
    MESI_SHARED    = 1,  /* Block is clean and may exist in other caches (read-only) */
    MESI_EXCLUSIVE = 2,  /* Block is clean and only exists in this cache */
    MESI_MODIFIED  = 3   /* Block is dirty (modified) and only exists in this cache */
} MesiState;

/* 
 * Bus Commands for Cache Coherency
 * These commands are broadcast on the shared bus to coordinate
 * cache operations between cores.
 */
typedef enum {
    BUS_NO_CMD = 0,  /* No operation on bus this cycle */
    BUS_RD     = 1,  /* Read request - core wants shared access to a block */
    BUS_RDX    = 2,  /* Read exclusive - core wants exclusive access (for writing) */
    BUS_FLUSH  = 3   /* Flush data to bus - sends modified data to requestor */
} BusCmd;

/* Special originator ID indicating main memory is the data source */
#define BUS_ORIGID_MEM 4

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/*
 * Decoded Instruction Structure
 * Holds all fields extracted from a 32-bit instruction word.
 * The instruction format is: [opcode:8][rd:4][rs:4][rt:4][imm:12]
 */
typedef struct {
    uint8_t  opcode;    /* Operation code (8 bits) */
    uint8_t  rd;        /* Destination register index (4 bits) */
    uint8_t  rs;        /* Source register 1 index (4 bits) */
    uint8_t  rt;        /* Source register 2 index (4 bits) */
    int32_t  imm;       /* Sign-extended immediate value (12 bits -> 32 bits) */
    uint32_t raw;       /* Original unmodified instruction word */
} Instruction;

/*
 * Pipeline Register Structure
 * Passed between pipeline stages to carry instruction state.
 * Acts as the "latch" between stages in the 5-stage pipeline.
 */
typedef struct {
    Instruction inst;           /* The decoded instruction */
    uint32_t    pc;             /* Program counter when instruction was fetched */
    int32_t     rs_val;         /* Value read from source register rs */
    int32_t     rt_val;         /* Value read from source register rt */
    int32_t     rd_val;         /* Value read from rd (used for branches/stores) */
    int32_t     alu_result;     /* Computed result from the ALU stage */
    uint32_t    mem_addr;       /* Calculated memory address for load/store */
    int32_t     mem_data;       /* Data to write for store instructions */
    bool        valid;          /* True if this stage contains a valid instruction */
    bool        writes_reg;     /* True if instruction will write to a register */
    uint8_t     dest_reg;       /* Index of destination register being written */
    bool        cache_op_done;  /* Flag to track if cache hit/miss already counted */
} PipelineReg;

/*
 * Cache Structure
 * Implements a direct-mapped cache with separate data and tag SRAMs.
 * DSRAM stores the actual data words.
 * TSRAM stores the tag and MESI state for each cache line.
 */
typedef struct {
    uint32_t dsram[CACHE_SIZE];         /* Data SRAM - stores actual data words */
    uint32_t tsram[NUM_CACHE_BLOCKS];   /* Tag SRAM - stores [MESI:2][tag:12] */
} Cache;

/*
 * Bus Transaction State
 * Tracks ongoing bus transactions and handles arbitration.
 * The bus is shared among all cores and supports multi-cycle transfers.
 */
typedef struct {
    /* Current bus signals visible to all cores */
    int      origid;                    /* ID of core/memory originating this transaction */
    BusCmd   cmd;                       /* Current bus command */
    uint32_t addr;                      /* Address on the bus */
    uint32_t request_word_addr;         /* Original requested word address for trace */
    uint32_t data;                      /* Data word being transferred */
    int      shared;                    /* Signal: 1 if block exists in multiple caches */
    
    /* Multi-cycle transaction state */
    bool     busy;                      /* True if a transaction is in progress */
    int      flush_origid;              /* Core providing flush data (or MEM if from memory) */
    int      requesting_core;           /* Core that requested the data */
    int      delay_counter;             /* Countdown for memory access latency */
    int      words_transferred;         /* Number of words sent in current transfer */
    uint32_t block_buffer[BLOCK_SIZE];  /* Buffer holding incoming block data */
    BusCmd   original_cmd;              /* Original command type (BusRd or BusRdX) */
    uint32_t original_addr;             /* Block-aligned address of the request */
    int      sampled_shared;            /* Sampled shared signal when request issued */
    
    /* Writeback (eviction) tracking for modified blocks */
    bool     writeback_in_progress;     /* True during writeback operation */
    int      writeback_core;            /* Core performing the writeback */
    uint32_t writeback_addr;            /* Block address being written back */
    int      writeback_word;            /* Current word index in writeback */
} Bus;

/*
 * Core Statistics Structure
 * Collects performance metrics for analysis.
 */
typedef struct {
    int cycles;         /* Total clock cycles this core was active */
    int instructions;   /* Total instructions retired (completed) */
    int read_hit;       /* Number of cache read hits */
    int write_hit;      /* Number of cache write hits */
    int read_miss;      /* Number of cache read misses */
    int write_miss;     /* Number of cache write misses */
    int decode_stall;   /* Cycles stalled in decode due to data hazards */
    int mem_stall;      /* Cycles stalled waiting for memory/cache */
} Stats;

/*
 * Core State Structure
 * Contains complete state for a single processor core including
 * registers, PC, instruction memory, cache, pipeline registers,
 * and control signals.
 */
typedef struct {
    int          id;                        /* Core identifier (0-3) */
    uint32_t     regs[NUM_REGISTERS];       /* Register file (R0-R15) */
    uint32_t     pc;                        /* Program counter */
    uint32_t     imem[IMEM_SIZE];           /* Private instruction memory */
    Cache        cache;                     /* Private L1 data cache */
    
    /* Pipeline registers between stages (5-stage pipeline) */
    PipelineReg  if_id;     /* Fetch -> Decode latch */
    PipelineReg  id_ex;     /* Decode -> Execute latch */
    PipelineReg  ex_mem;    /* Execute -> Memory latch */
    PipelineReg  mem_wb;    /* Memory -> Writeback latch */
    
    /* Pipeline control signals */
    bool         halted;            /* True when core has executed HALT */
    bool         stall_fetch;       /* Stall the fetch stage */
    bool         stall_decode;      /* Stall the decode stage */
    bool         stall_mem;         /* Stall the memory stage */
    
    /* Branch handling */
    bool         branch_taken;      /* Branch was taken this cycle */
    uint32_t     branch_target;     /* Target PC for taken branch */
    
    /* Cache transaction state */
    bool         cache_busy;         /* Cache waiting for bus transaction to complete */
    bool         waiting_for_bus;    /* Core needs bus access */
    bool         bus_request_ready;  /* Bus request is ready after 1 cycle delay */
    BusCmd       pending_bus_cmd;    /* Command to issue when bus is granted */
    uint32_t     pending_addr;       /* Address for pending bus request */
    bool         need_writeback;     /* Must writeback dirty block before fetch */
    uint32_t     writeback_addr;     /* Address of block to writeback */
    
    /* Pending store state (for stores after cache fill completes) */
    bool         pending_store;      /* Store waiting to complete after cache fill */
    uint32_t     pending_store_addr; /* Address for pending store */
    uint32_t     pending_store_data; /* Data for pending store */
    
    /* Load completion tracking */
    bool         load_data_ready;    /* Load data is ready to forward */
    int32_t      load_result;        /* The loaded data value */
    
    Stats        stats;              /* Performance statistics */
} Core;

/* ============================================================================
 * GLOBAL STATE
 * ============================================================================ */

static Core     g_cores[NUM_CORES];     /* Array of all processor cores */
static uint32_t g_main_mem[MAIN_MEM_SIZE]; /* Shared main memory */
static Bus      g_bus;                  /* Shared system bus */
static int      g_last_granted;         /* Last core granted bus (for round-robin) */
static int      g_cycle;                /* Current simulation cycle */

/* Output file pointers for trace generation */
static FILE    *g_trace_files[NUM_CORES];   /* Per-core pipeline traces */
static FILE    *g_bus_trace;                /* Bus transaction trace */

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/*
 * sign_extend_12
 * ---------------
 * Sign-extends a 12-bit immediate value to a 32-bit signed integer.
 * If bit 11 (the sign bit) is set, the upper 20 bits are filled with 1s.
 * This is necessary because immediate values in instructions are only 12 bits
 * but need to be used in 32-bit arithmetic operations.
 */
static int32_t sign_extend_12(uint32_t val) {
    /* Check if sign bit (bit 11) is set */
    if (val & 0x800) {
        /* Extend with 1s by ORing with upper bits mask */
        return (int32_t)(val | 0xFFFFF000);
    }
    /* Positive number - just return as is */
    return (int32_t)val;
}

/*
 * get_tag
 * --------
 * Extracts the cache tag from a memory address.
 * In this direct-mapped cache design:
 * - Bits [2:0] = offset within block (3 bits for 8 words)
 * - Bits [8:3] = cache index (6 bits for 64 lines)
 * - Bits [20:9] = tag (12 bits to identify unique blocks)
 */
static uint32_t get_tag(uint32_t addr) {
    return (addr >> 9) & 0xFFF;  /* Extract bits [20:9] */
}

/*
 * get_index
 * ----------
 * Extracts the cache index from a memory address.
 * The index determines which cache line this address maps to.
 * With 64 cache lines, we need 6 bits for the index.
 */
static uint32_t get_index(uint32_t addr) {
    return (addr >> 3) & 0x3F;   /* Extract bits [8:3] */
}

/*
 * get_offset
 * -----------
 * Extracts the block offset from a memory address.
 * The offset identifies which word within the 8-word block.
 * With 8 words per block, we need 3 bits.
 */
static uint32_t get_offset(uint32_t addr) {
    return addr & 0x7;           /* Extract bits [2:0] */
}

/*
 * get_block_addr
 * ---------------
 * Returns the block-aligned address by zeroing out the offset bits.
 * This gives the starting address of the cache block containing 'addr'.
 */
static uint32_t get_block_addr(uint32_t addr) {
    return addr & ~0x7;  /* Clear the lower 3 bits */
}

/*
 * get_mesi_state
 * ---------------
 * Extracts the MESI state from a TSRAM entry.
 * The TSRAM format is: [unused:18][MESI:2][tag:12]
 * MESI state is stored in bits [13:12].
 */
static MesiState get_mesi_state(uint32_t tsram_entry) {
    return (MesiState)((tsram_entry >> 12) & 0x3);
}

/*
 * get_tsram_tag
 * --------------
 * Extracts the tag from a TSRAM entry.
 * The tag is stored in the lower 12 bits of the TSRAM entry.
 */
static uint32_t get_tsram_tag(uint32_t tsram_entry) {
    return tsram_entry & 0xFFF;
}

/*
 * make_tsram_entry
 * -----------------
 * Creates a TSRAM entry by packing the MESI state and tag together.
 * Format: [MESI:2 at bits 13:12][tag:12 at bits 11:0]
 */
static uint32_t make_tsram_entry(MesiState state, uint32_t tag) {
    return ((uint32_t)state << 12) | (tag & 0xFFF);
}

/*
 * decode_instruction
 * -------------------
 * Decodes a 32-bit raw instruction word into its component fields.
 * Instruction format: [opcode:8][rd:4][rs:4][rt:4][imm:12]
 * The immediate value is sign-extended from 12 to 32 bits.
 */
static Instruction decode_instruction(uint32_t raw) {
    Instruction inst;
    inst.raw = raw;
    inst.opcode = (raw >> 24) & 0xFF;   /* Extract bits [31:24] */
    inst.rd = (raw >> 20) & 0xF;        /* Extract bits [23:20] */
    inst.rs = (raw >> 16) & 0xF;        /* Extract bits [19:16] */
    inst.rt = (raw >> 12) & 0xF;        /* Extract bits [15:12] */
    inst.imm = sign_extend_12(raw & 0xFFF);  /* Extract and sign-extend bits [11:0] */
    return inst;
}

/*
 * is_branch
 * ----------
 * Returns true if the given opcode is a branch or jump instruction.
 * Branch opcodes are 9-15 (BEQ, BNE, BLT, BGT, BLE, BGE, JAL).
 */
static bool is_branch(uint8_t opcode) {
    return opcode >= OP_BEQ && opcode <= OP_JAL;
}

/*
 * writes_register
 * ----------------
 * Returns true if the given opcode writes a result to a register.
 * This includes arithmetic ops, shifts, JAL (writes return address),
 * and LW (writes loaded data). Branches, stores, and HALT don't write.
 */
static bool writes_register(uint8_t opcode) {
    switch (opcode) {
        case OP_ADD: case OP_SUB: case OP_AND: case OP_OR:
        case OP_XOR: case OP_MUL: case OP_SLL: case OP_SRA:
        case OP_SRL: case OP_JAL: case OP_LW:
            return true;
        default:
            return false;
    }
}

/*
 * get_dest_reg
 * -------------
 * Returns the destination register for an instruction.
 * Most instructions write to rd, but JAL writes to R15 (link register).
 */
static uint8_t get_dest_reg(Instruction *inst) {
    if (inst->opcode == OP_JAL) return 15;  /* JAL uses R15 as link register */
    return inst->rd;
}

/* ============================================================================
 * FILE I/O
 * ============================================================================ */

/*
 * load_imem
 * ----------
 * Loads instruction memory from a hex file.
 * Each line contains one 32-bit instruction in hexadecimal format.
 * Instructions are loaded sequentially starting at address 0.
 */
static void load_imem(const char *filename, uint32_t *imem) {
    FILE *f = fopen(filename, "r");
    if (!f) return;  /* File not found - leave imem zeroed */
    
    char line[32];
    int addr = 0;
    /* Read hex values line by line until EOF or memory full */
    while (fgets(line, sizeof(line), f) && addr < IMEM_SIZE) {
        imem[addr++] = (uint32_t)strtoul(line, NULL, 16);
    }
    
    fclose(f);
}

/*
 * load_main_mem
 * --------------
 * Loads main memory contents from a hex file.
 * Similar to load_imem but for the shared main memory.
 */
static void load_main_mem(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) return;
    
    char line[32];
    int addr = 0;
    while (fgets(line, sizeof(line), f) && addr < MAIN_MEM_SIZE) {
        g_main_mem[addr++] = (uint32_t)strtoul(line, NULL, 16);
    }
    
    fclose(f);
}

/*
 * write_main_mem
 * ---------------
 * Writes main memory contents to a hex file.
 * Only writes up to the last non-zero word to minimize file size.
 */
static void write_main_mem(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    /* Find the last non-zero word to determine output length */
    int last = 0;
    for (int i = 0; i < MAIN_MEM_SIZE; i++) {
        if (g_main_mem[i] != 0) last = i;
    }
    
    /* Write memory contents up to and including last non-zero word */
    for (int i = 0; i <= last; i++) {
        fprintf(f, "%08X\n", g_main_mem[i]);
    }
    
    fclose(f);
}

/*
 * write_regout
 * -------------
 * Writes the final register file contents to a file.
 * Only writes R2-R15 since R0 is constant 0 and R1 holds immediates.
 */
static void write_regout(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    /* Write registers R2 through R15 in hex format */
    for (int i = 2; i < NUM_REGISTERS; i++) {
        fprintf(f, "%08X\n", core->regs[i]);
    }
    
    fclose(f);
}

/*
 * write_dsram
 * ------------
 * Writes the cache data SRAM contents to a file.
 * Outputs all 512 words of the data array.
 */
static void write_dsram(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    for (int i = 0; i < CACHE_SIZE; i++) {
        fprintf(f, "%08X\n", core->cache.dsram[i]);
    }
    
    fclose(f);
}

/*
 * write_tsram
 * ------------
 * Writes the cache tag SRAM contents to a file.
 * Outputs all 64 tag entries (one per cache line).
 */
static void write_tsram(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    for (int i = 0; i < NUM_CACHE_BLOCKS; i++) {
        fprintf(f, "%08X\n", core->cache.tsram[i]);
    }
    
    fclose(f);
}

/*
 * write_stats
 * ------------
 * Writes performance statistics to a file.
 * Includes cycle count, instruction count, cache statistics, and stall counts.
 */
static void write_stats(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    fprintf(f, "cycles %d\n", core->stats.cycles);
    fprintf(f, "instructions %d\n", core->stats.instructions);
    fprintf(f, "read_hit %d\n", core->stats.read_hit);
    fprintf(f, "write_hit %d\n", core->stats.write_hit);
    fprintf(f, "read_miss %d\n", core->stats.read_miss);
    fprintf(f, "write_miss %d\n", core->stats.write_miss);
    fprintf(f, "decode_stall %d\n", core->stats.decode_stall);
    fprintf(f, "mem_stall %d\n", core->stats.mem_stall);
    
    fclose(f);
}

/* ============================================================================
 * TRACE OUTPUT
 * ============================================================================ */

/*
 * write_core_trace
 * -----------------
 * Writes pipeline trace for current cycle showing PC in each stage.
 * Format: cycle fetch_pc decode_pc exec_pc mem_pc wb_pc reg2 reg3 ... reg15
 * Returns true if trace was written (core was active this cycle).
 */
static bool write_core_trace(Core *core) {
    FILE *f = g_trace_files[core->id];
    if (!f) return false;
    
    /* Determine which pipeline stages have valid instructions */
    bool fetch_active = !core->halted;
    bool decode_active = core->if_id.valid;
    bool exec_active = core->id_ex.valid;
    bool mem_active = core->ex_mem.valid;
    bool wb_active = core->mem_wb.valid;
    
    bool any_active = fetch_active || decode_active || exec_active || mem_active || wb_active;
    
    /* Don't write trace if no stages are active */
    if (!any_active) return false;
    
    /* Write cycle number */
    fprintf(f, "%d ", g_cycle);
    
    /* Fetch stage - show current PC if not halted */
    if (fetch_active) {
        fprintf(f, "%03X ", core->pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Decode stage - show PC of instruction being decoded */
    if (decode_active) {
        fprintf(f, "%03X ", core->if_id.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Execute stage - show PC of instruction in execute */
    if (exec_active) {
        fprintf(f, "%03X ", core->id_ex.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Memory stage - show PC of instruction accessing memory */
    if (mem_active) {
        fprintf(f, "%03X ", core->ex_mem.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Writeback stage - show PC of instruction writing back */
    if (wb_active) {
        fprintf(f, "%03X ", core->mem_wb.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Output register values R2-R15 (values at START of cycle) */
    for (int i = 2; i < NUM_REGISTERS; i++) {
        fprintf(f, "%08X", core->regs[i]);
        if (i < NUM_REGISTERS - 1) fprintf(f, " ");
    }
    fprintf(f, "\n");
    return true;
}

/*
 * write_bus_trace
 * ----------------
 * Writes bus transaction trace for current cycle.
 * Only writes if there's an active command on the bus.
 * Format: cycle origid cmd addr data shared
 */
static void write_bus_trace(void) {
    /* No trace if bus is idle */
    if (g_bus.cmd == BUS_NO_CMD) return;
    if (!g_bus_trace) return;
    
    fprintf(g_bus_trace, "%d %X %X %06X %08X %X\n",
            g_cycle, g_bus.origid, g_bus.cmd, 
            g_bus.addr & 0x1FFFFF, g_bus.data, g_bus.shared);
}

/* ============================================================================
 * CACHE OPERATIONS
 * ============================================================================ */

/*
 * cache_has_block
 * ----------------
 * Checks if the cache contains a valid copy of the block at 'addr'.
 * Returns true only if the tag matches AND the state is not INVALID.
 * This is the cache lookup operation used before reads/writes.
 */
static bool cache_has_block(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    uint32_t tag = get_tag(addr);
    uint32_t tsram_entry = cache->tsram[index];
    MesiState state = get_mesi_state(tsram_entry);
    uint32_t stored_tag = get_tsram_tag(tsram_entry);
    
    /* Block is present if tag matches and state is not invalid */
    return (state != MESI_INVALID) && (stored_tag == tag);
}

/*
 * cache_read
 * -----------
 * Reads a word from the cache at the given address.
 * IMPORTANT: Caller must verify cache hit first using cache_has_block().
 * Computes the DSRAM index as: (cache_line_index * 8) + word_offset
 */
static uint32_t cache_read(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    uint32_t offset = get_offset(addr);
    /* Calculate position in DSRAM: line_index * words_per_line + word_offset */
    return cache->dsram[index * BLOCK_SIZE + offset];
}

/*
 * cache_write
 * ------------
 * Writes a word to the cache at the given address.
 * The caller is responsible for updating MESI state after write.
 */
static void cache_write(Cache *cache, uint32_t addr, uint32_t data) {
    uint32_t index = get_index(addr);
    uint32_t offset = get_offset(addr);
    cache->dsram[index * BLOCK_SIZE + offset] = data;
}

/*
 * cache_get_state
 * ----------------
 * Returns the MESI state of the cache line containing 'addr'.
 */
static MesiState cache_get_state(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    return get_mesi_state(cache->tsram[index]);
}

/*
 * cache_set_state
 * ----------------
 * Sets the MESI state for the cache line containing 'addr'.
 * Also updates the tag to match the new address.
 */
static void cache_set_state(Cache *cache, uint32_t addr, MesiState state) {
    uint32_t index = get_index(addr);
    uint32_t tag = get_tag(addr);
    cache->tsram[index] = make_tsram_entry(state, tag);
}

/*
 * cache_needs_writeback
 * ----------------------
 * Determines if a writeback is needed before allocating space for 'addr'.
 * Writeback is needed when: different tag (conflict miss) AND state is MODIFIED.
 * This implements the "write-allocate with write-back" policy where dirty blocks must be
 * written to memory before being evicted.
 */
static bool cache_needs_writeback(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    uint32_t tsram_entry = cache->tsram[index];
    MesiState state = get_mesi_state(tsram_entry);
    uint32_t stored_tag = get_tsram_tag(tsram_entry);
    uint32_t new_tag = get_tag(addr);
    
    /* Need writeback if we're evicting a modified block */
    return (stored_tag != new_tag) && (state == MESI_MODIFIED);
}

/*
 * cache_get_current_block_addr
 * -----------------------------
 * Reconstructs the memory address of the block currently in the cache line.
 * Used during writeback to determine where to write the dirty data.
 * Address = (stored_tag << 9) | (index << 3) | 0
 */
static uint32_t cache_get_current_block_addr(Cache *cache, uint32_t new_addr) {
    uint32_t index = get_index(new_addr);
    uint32_t stored_tag = get_tsram_tag(cache->tsram[index]);
    /* Reconstruct address from tag and index (offset is 0 for block-aligned) */
    return (stored_tag << 9) | (index << 3);
}

/*
 * cache_fill_block
 * -----------------
 * Fills an entire cache block with data received from bus/memory.
 * Copies all 8 words of data into the DSRAM and sets the tag and MESI state.
 * This is called when a cache miss is resolved.
 */
static void cache_fill_block(Cache *cache, uint32_t addr, uint32_t *data, MesiState state) {
    uint32_t index = get_index(addr);
    uint32_t tag = get_tag(addr);
    
    /* Copy all words of the block to DSRAM */
    for (int i = 0; i < BLOCK_SIZE; i++) {
        cache->dsram[index * BLOCK_SIZE + i] = data[i];
    }
    
    /* Update TSRAM with new tag and state */
    cache->tsram[index] = make_tsram_entry(state, tag);
}

/* ============================================================================
 * BUS OPERATIONS
 * ============================================================================ */

/*
 * arbitrate_bus
 * --------------
 * Implements round-robin bus arbitration among requesting cores.
 * Starts checking from the core after the last granted one to ensure fairness.
 * Returns the core ID that wins arbitration, or -1 if no requests pending.
 */
static int arbitrate_bus(bool requests[NUM_CORES]) {
    /* Start checking from core after last granted (round-robin) */
    for (int i = 1; i <= NUM_CORES; i++) {
        int idx = (g_last_granted + i) % NUM_CORES;
        if (requests[idx]) {
            return idx;
        }
    }
    return -1;  /* No requests pending */
}

/* ============================================================================
 * SNOOPING LOGIC
 * ============================================================================ */

/*
 * snoop_and_get_shared
 * ---------------------
 * Checks if any other cache has a copy of the block at 'addr'.
 * This sets the "shared" signal on the bus which affects MESI transitions.
 * Returns 1 if any other cache has the block, 0 otherwise.
 */
static int snoop_and_get_shared(uint32_t addr, int requesting_core) {
    for (int i = 0; i < NUM_CORES; i++) {
        if (i == requesting_core) continue;  /* Skip the requestor */
        
        Cache *cache = &g_cores[i].cache;
        if (cache_has_block(cache, addr)) {
            return 1;  /* Another cache has this block */
        }
    }
    return 0;  /* No other cache has this block */
}

/*
 * find_modified_owner
 * --------------------
 * Finds which core (if any) has the block in MODIFIED state.
 * This determines if we need cache-to-cache transfer or memory fetch.
 * Returns core ID if found, -1 if no modified copy exists.
 */
static int find_modified_owner(uint32_t addr, int requesting_core) {
    for (int i = 0; i < NUM_CORES; i++) {
        if (i == requesting_core) continue;
        
        Cache *cache = &g_cores[i].cache;
        if (cache_has_block(cache, addr)) {
            if (cache_get_state(cache, addr) == MESI_MODIFIED) {
                return i;  /* This core has the modified copy */
            }
        }
    }
    return -1;  /* No modified copy found */
}

/*
 * snoop_update_states
 * --------------------
 * Updates MESI states in snooping caches when BusRd/BusRdX is observed.
 * Implements the MESI protocol state transitions:
 * - BusRd: Exclusive -> Shared (another core wants to read)
 * - BusRdX: Shared/Exclusive -> Invalid (another core wants exclusive access)
 * Modified state is handled separately by flush logic.
 */
static void snoop_update_states(uint32_t addr, BusCmd cmd, int requesting_core) {
    for (int i = 0; i < NUM_CORES; i++) {
        if (i == requesting_core) continue;
        
        Cache *cache = &g_cores[i].cache;
        if (!cache_has_block(cache, addr)) continue;
        
        MesiState state = cache_get_state(cache, addr);
        
        if (cmd == BUS_RD) {
            /* Another core wants to read - downgrade to shared if needed */
            switch (state) {
                case MESI_EXCLUSIVE:
                    /* E -> S: No longer exclusive, but still clean */
                    cache_set_state(cache, addr, MESI_SHARED);
                    break;
                case MESI_MODIFIED:
                    /* M state handled by flush - state updated after flush completes */
                    break;
                default:
                    break;
            }
        } else if (cmd == BUS_RDX) {
            /* Another core wants exclusive access - must invalidate */
            switch (state) {
                case MESI_SHARED:
                case MESI_EXCLUSIVE:
                    /* S/E -> I: Another core will modify, invalidate our copy */
                    cache_set_state(cache, addr, MESI_INVALID);
                    break;
                case MESI_MODIFIED:
                    /* M state handled by flush - state updated after flush completes */
                    break;
                default:
                    break;
            }
        }
    }
}

/* ============================================================================
 * PIPELINE REGISTER READ/WRITE
 * ============================================================================ */

/*
 * read_reg
 * ---------
 * Reads a value from the register file with special handling for R0 and R1.
 * R0 is hardwired to 0 (always reads as zero).
 * R1 is the immediate register (returns the instruction's immediate value).
 */
static int32_t read_reg(Core *core, uint8_t reg, int32_t imm) {
    if (reg == 0) return 0;      /* R0 is always 0 */
    if (reg == 1) return imm;    /* R1 returns the immediate value */
    return (int32_t)core->regs[reg];
}

/*
 * write_reg
 * ----------
 * Writes a value to the register file.
 * Ignores writes to R0 (constant zero) and R1 (immediate register).
 */
static void write_reg(Core *core, uint8_t reg, int32_t value) {
    if (reg == 0 || reg == 1) return;  /* R0 and R1 are read-only */
    core->regs[reg] = (uint32_t)value;
}

/* ============================================================================
 * BUS TRANSACTION PROCESSING
 * ============================================================================ */

/*
 * process_bus_transactions
 * -------------------------
 * Handles all bus transactions for the current cycle.
 * This includes:
 * 1. Ongoing writebacks (evicting modified blocks to memory)
 * 2. Ongoing flush operations (cache-to-cache or memory transfers)
 * 3. New bus requests from cores (BusRd/BusRdX)
 * 
 * The bus can only handle one transaction at a time.
 * Priority: ongoing transactions > new requests
 */
static void process_bus_transactions(void) {
    /* Default: no command on bus this cycle */
    g_bus.cmd = BUS_NO_CMD;
    
    /* Handle ongoing writeback (eviction of modified block) */
    if (g_bus.writeback_in_progress) {
        Core *core = &g_cores[g_bus.writeback_core];
        uint32_t wb_addr = g_bus.writeback_addr + g_bus.writeback_word;
        
        /* Set bus signals for trace output */
        g_bus.origid = core->id;
        g_bus.cmd = BUS_FLUSH;
        g_bus.addr = wb_addr;
        g_bus.data = cache_read(&core->cache, wb_addr);
        g_bus.shared = 0;
        
        /* Write the word to main memory */
        g_main_mem[wb_addr] = g_bus.data;
        
        /* Move to next word in the block */
        g_bus.writeback_word++;
        if (g_bus.writeback_word >= BLOCK_SIZE) {
            /* Writeback complete - invalidate the evicted block */
            cache_set_state(&core->cache, g_bus.writeback_addr, MESI_INVALID);
            core->need_writeback = false;
            g_bus.writeback_in_progress = false;
            /* Bus is now free for the pending BusRd/BusRdX */
        }
        return;
    }
    
    /* Handle ongoing flush transaction (response to BusRd/BusRdX) */
    if (g_bus.busy) {
        if (g_bus.delay_counter > 0) {
            /* Still waiting for memory latency to expire */
            g_bus.delay_counter--;
            return;
        }
        
        /* Transfer next word from the block buffer */
        int word_idx = g_bus.words_transferred;
        uint32_t word_addr = g_bus.original_addr + word_idx;
        g_bus.data = g_bus.block_buffer[word_idx];
        
        /* Set bus signals for trace */
        g_bus.cmd = BUS_FLUSH;
        g_bus.origid = g_bus.flush_origid;
        g_bus.addr = word_addr;
        g_bus.shared = g_bus.sampled_shared;
        
        g_bus.words_transferred++;
        
        if (g_bus.words_transferred >= BLOCK_SIZE) {
            /* All words transferred - complete the cache fill */
            Core *req_core = &g_cores[g_bus.requesting_core];
            
            /* Determine new MESI state based on request type and sharing */
            MesiState new_state;
            if (g_bus.original_cmd == BUS_RDX) {
                /* Write request - will be modified immediately */
                new_state = MESI_MODIFIED;
            } else {
                /* Read request - exclusive if not shared, shared otherwise */
                new_state = (g_bus.sampled_shared ? MESI_SHARED : MESI_EXCLUSIVE);
            }
            
            /* Fill the cache block with received data */
            cache_fill_block(&req_core->cache, g_bus.original_addr, g_bus.block_buffer, new_state);
            
            /* If there's a pending store, complete it now */
            if (req_core->pending_store) {
                cache_write(&req_core->cache, req_core->pending_store_addr, req_core->pending_store_data);
                cache_set_state(&req_core->cache, req_core->pending_store_addr, MESI_MODIFIED);
                req_core->pending_store = false;
            }
            
            /* If this was a load, read the data and mark it ready */
            if (g_bus.original_cmd == BUS_RD && req_core->ex_mem.valid && 
                req_core->ex_mem.inst.opcode == OP_LW) {
                req_core->load_data_ready = true;
                req_core->load_result = (int32_t)cache_read(&req_core->cache, req_core->ex_mem.mem_addr);
            }
            
            /* Update source cache state if data came from another cache */
            if (g_bus.flush_origid != BUS_ORIGID_MEM) {
                Core *src_core = &g_cores[g_bus.flush_origid];
                if (g_bus.original_cmd == BUS_RD) {
                    /* BusRd: source goes to Shared state */
                    cache_set_state(&src_core->cache, g_bus.original_addr, MESI_SHARED);
                } else {
                    /* BusRdX: source invalidated */
                    cache_set_state(&src_core->cache, g_bus.original_addr, MESI_INVALID);
                }
            }
            
            /* Clear stalls - cache is now ready */
            req_core->cache_busy = false;
            req_core->stall_mem = false;
            req_core->stall_decode = false;
            req_core->stall_fetch = false;
            
            /* Clear bus busy state */
            g_bus.busy = false;
        }
        return;
    }
    
    /* Check for new bus requests from cores */
    /* Only consider requests that are ready (after 1 cycle delay) */
    bool requests[NUM_CORES] = {false};
    
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        /* Request bus if need writeback or waiting for data AND ready */
        if (core->need_writeback && core->bus_request_ready) {
            requests[i] = true;
        } else if (core->waiting_for_bus && core->bus_request_ready) {
            requests[i] = true;
        }
    }
    
    /* Arbitrate - determine which core gets the bus this cycle */
    int granted = arbitrate_bus(requests);
    if (granted < 0) {
        return;  /* No pending requests */
    }
    
    Core *core = &g_cores[granted];
    g_last_granted = granted;
    
    if (core->need_writeback) {
        /* Start writeback - this locks the bus for multiple cycles */
        g_bus.writeback_in_progress = true;
        g_bus.writeback_core = core->id;
        g_bus.writeback_addr = core->writeback_addr;
        g_bus.writeback_word = 0;
        
        /* Send first word of writeback */
        uint32_t wb_addr = g_bus.writeback_addr;
        g_bus.origid = core->id;
        g_bus.cmd = BUS_FLUSH;
        g_bus.addr = wb_addr;
        g_bus.data = cache_read(&core->cache, wb_addr);
        g_bus.shared = 0;
        
        /* Write to main memory */
        g_main_mem[wb_addr] = g_bus.data;
        
        g_bus.writeback_word++;
        if (g_bus.writeback_word >= BLOCK_SIZE) {
            /* Single-word block case (shouldn't happen with BLOCK_SIZE=8) */
            cache_set_state(&core->cache, g_bus.writeback_addr, MESI_INVALID);
            core->need_writeback = false;
            g_bus.writeback_in_progress = false;
        }
    } else {
        /* Initiate BusRd or BusRdX for cache miss */
        uint32_t block_addr = get_block_addr(core->pending_addr);
        uint32_t request_addr = core->pending_addr & 0x1FFFFF;
        
        /* Issue the command on the bus */
        g_bus.origid = core->id;
        g_bus.cmd = core->pending_bus_cmd;
        g_bus.request_word_addr = request_addr;
        g_bus.addr = request_addr;
        g_bus.data = 0;
        g_bus.requesting_core = core->id;
        g_bus.original_cmd = core->pending_bus_cmd;
        g_bus.original_addr = block_addr;
        
        core->waiting_for_bus = false;
        core->bus_request_ready = false;
        
        /* Snoop: check if block is shared (affects MESI state on fill) */
        g_bus.shared = 0;
        g_bus.sampled_shared = snoop_and_get_shared(block_addr, core->id);
        
        /* Update snooping caches based on this request */
        snoop_update_states(block_addr, core->pending_bus_cmd, core->id);
        
        /* Check if another cache has a modified copy */
        int mod_owner = find_modified_owner(block_addr, core->id);
        
        if (mod_owner >= 0) {
            /* Cache-to-cache transfer: another cache will provide data */
            g_bus.flush_origid = mod_owner;
            g_bus.sampled_shared = 1;  /* Block is shared */
            g_bus.delay_counter = 0;   /* No memory latency for cache-to-cache */
        } else {
            /* Memory will provide the data */
            g_bus.flush_origid = BUS_ORIGID_MEM;
            g_bus.delay_counter = MEM_LATENCY - 1;  /* Wait for memory latency */
        }
        
        /* Snapshot the entire block at the start of the transaction */
        if (g_bus.flush_origid == BUS_ORIGID_MEM) {
            /* Read block from main memory */
            for (int i = 0; i < BLOCK_SIZE; i++) {
                g_bus.block_buffer[i] = g_main_mem[block_addr + i];
            }
        } else {
            /* Read block from source cache and also write through to memory */
            Core *src_core = &g_cores[g_bus.flush_origid];
            for (int i = 0; i < BLOCK_SIZE; i++) {
                uint32_t word_addr = block_addr + i;
                g_bus.block_buffer[i] = cache_read(&src_core->cache, word_addr);
                g_main_mem[word_addr] = g_bus.block_buffer[i];  /* Write-through on flush */
            }
        }
        
        g_bus.busy = true;
        g_bus.words_transferred = 0;
    }
}

/* ============================================================================
 * SIMULATION MAIN LOOP
 * ============================================================================ */

/*
 * init_core
 * ----------
 * Initializes a core to its default state.
 * All memory is zeroed, including registers, caches, and pipeline state.
 */
static void init_core(Core *core, int id) {
    memset(core, 0, sizeof(Core));
    core->id = id;
}

/*
 * all_cores_done
 * ---------------
 * Checks if simulation is complete.
 * Returns true when all cores have halted AND all pipelines are empty
 * AND no bus transactions are in progress.
 */
static bool all_cores_done(void) {
    for (int i = 0; i < NUM_CORES; i++) {
        /* Check if core is still running or has instructions in flight */
        if (!g_cores[i].halted) return false;
        if (g_cores[i].if_id.valid) return false;
        if (g_cores[i].id_ex.valid) return false;
        if (g_cores[i].ex_mem.valid) return false;
        if (g_cores[i].mem_wb.valid) return false;
    }
    /* Also check if bus is still busy with a transaction */
    if (g_bus.busy) return false;
    if (g_bus.writeback_in_progress) return false;
    return true;
}

/*
 * simulate_cycle
 * ---------------
 * Simulates one clock cycle for all cores.
 * Processing order:
 * 1. Write traces (captures state at start of cycle)
 * 2. Count mem_stall cycles
 * 3. Process bus transactions
 * 4. Advance bus request readiness
 * 5. Process pipeline stages in reverse order (WB -> MEM -> EX -> ID -> IF)
 * 
 * Processing stages in reverse order ensures proper pipeline behavior
 * where later stages read state from earlier stages before it's updated.
 */
static void simulate_cycle(void) {
    /* Write trace at start of cycle (before any state changes) */
    bool core_was_active[NUM_CORES];
    for (int i = 0; i < NUM_CORES; i++) {
        core_was_active[i] = write_core_trace(&g_cores[i]);
    }
    
    /* Count mem_stall BEFORE bus processing clears the stall */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        if (core->stall_mem && core->ex_mem.valid) {
            core->stats.mem_stall++;
        }
    }
    
    /* Process bus transactions (may resolve cache misses) */
    process_bus_transactions();
    
    /* Write bus trace for this cycle */
    write_bus_trace();
    
    /* Advance bus_request_ready after bus processing */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        if ((core->waiting_for_bus || core->need_writeback) && !core->bus_request_ready) {
            core->bus_request_ready = true;
        }
    }
    
    /* Save current pipeline state for proper stage advancement.
     * We update stages in reverse order (WB -> IF), but each stage
     * should see the state from the START of the cycle.
     */
    PipelineReg saved_if_id[NUM_CORES];
    PipelineReg saved_id_ex[NUM_CORES];
    PipelineReg saved_ex_mem[NUM_CORES];
    PipelineReg saved_mem_wb[NUM_CORES];
    
    for (int i = 0; i < NUM_CORES; i++) {
        saved_if_id[i] = g_cores[i].if_id;
        saved_id_ex[i] = g_cores[i].id_ex;
        saved_ex_mem[i] = g_cores[i].ex_mem;
        saved_mem_wb[i] = g_cores[i].mem_wb;
        g_cores[i].branch_taken = false;  /* Reset branch signal for new cycle */
    }
    
    /* ========== WRITEBACK STAGE ========== */
    /* Final stage - writes results to register file */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (!saved_mem_wb[i].valid) continue;
        
        Instruction *inst = &saved_mem_wb[i].inst;
        
        /* Count instruction as executed (including HALT) */
        core->stats.instructions++;
        
        /* Write result to register file if instruction produces a result */
        if (saved_mem_wb[i].writes_reg) {
            write_reg(core, saved_mem_wb[i].dest_reg, saved_mem_wb[i].alu_result);
        }
        
        /* Mark core as halted when HALT reaches WB */
        if (inst->opcode == OP_HALT) {
            core->halted = true;
        }
        
        /* Clear the WB stage for next cycle */
        core->mem_wb.valid = false;
    }
    
    /* ========== MEMORY STAGE ========== */
    /* Handles load/store operations and cache access */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (!saved_ex_mem[i].valid) {
            if (!core->stall_mem) {
                core->ex_mem.valid = false;
            }
            continue;
        }
        
        /* Check if stalled waiting for cache miss to resolve */
        if (core->stall_mem) {
            /* mem_stall was already counted at start of cycle */
            continue;  /* Keep instruction in MEM stage */
        }
        
        Instruction *inst = &saved_ex_mem[i].inst;
        PipelineReg result = saved_ex_mem[i];
        
        if (inst->opcode == OP_LW) {
            /* Load Word operation */
            uint32_t addr = saved_ex_mem[i].mem_addr;
            
            /* Check if data was just loaded by completed bus transaction */
            if (core->load_data_ready) {
                result.alu_result = core->load_result;
                core->load_data_ready = false;
                /* Hit/miss was already counted when miss started */
            } else if (cache_has_block(&core->cache, addr)) {
                /* Cache hit - read data from cache */
                if (!saved_ex_mem[i].cache_op_done) {
                    core->stats.read_hit++;
                }
                result.alu_result = (int32_t)cache_read(&core->cache, addr);
            } else {
                /* Cache miss - need to fetch block from memory/other cache */
                if (!core->cache_busy) {
                    core->stats.read_miss++;
                    core->cache_busy = true;
                    core->waiting_for_bus = true;
                    core->bus_request_ready = false;  
                    core->pending_bus_cmd = BUS_RD;  /* Read request */
                    core->pending_addr = addr;
                    
                    /* Mark that we already counted this miss */
                    core->ex_mem.cache_op_done = true;
                    
                    /* Check if we need to evict a modified block first */
                    if (cache_needs_writeback(&core->cache, addr)) {
                        core->need_writeback = true;
                        core->writeback_addr = cache_get_current_block_addr(&core->cache, addr);
                    }
                }
                /* Stall pipeline until data arrives */
                core->stall_mem = true;
                core->stall_decode = true;
                core->stall_fetch = true;
                continue;
            }
        } else if (inst->opcode == OP_SW) {
            /* Store Word operation */
            uint32_t addr = saved_ex_mem[i].mem_addr;
            uint32_t data = (uint32_t)saved_ex_mem[i].mem_data;
            
            if (cache_has_block(&core->cache, addr)) {
                MesiState state = cache_get_state(&core->cache, addr);
                
                if (state == MESI_MODIFIED || state == MESI_EXCLUSIVE) {
                    /* Can write directly - we have exclusive access */
                    /* E->M is a silent transition (no bus activity) */
                    if (!saved_ex_mem[i].cache_op_done) {
                        core->stats.write_hit++;
                    }
                    cache_write(&core->cache, addr, data);
                    cache_set_state(&core->cache, addr, MESI_MODIFIED);
                } else {
                    /* Shared state - need BusRdX for exclusive access */
                    if (!core->cache_busy) {
                        core->stats.write_miss++;
                        core->cache_busy = true;
                        core->waiting_for_bus = true;
                        core->bus_request_ready = false;  
                        core->pending_bus_cmd = BUS_RDX;  /* Read exclusive */
                        core->pending_addr = addr;
                        
                        core->ex_mem.cache_op_done = true;
                        
                        /* Save store data for after we get exclusive access */
                        core->pending_store = true;
                        core->pending_store_addr = addr;
                        core->pending_store_data = data;
                    }
                    core->stall_mem = true;
                    core->stall_decode = true;
                    core->stall_fetch = true;
                    continue;
                }
            } else {
                /* Cache miss on write - write allocate policy */
                /* Must fetch block first, then write to it */
                if (!core->cache_busy) {
                    core->stats.write_miss++;
                    core->cache_busy = true;
                    core->waiting_for_bus = true;
                    core->bus_request_ready = false;  
                    core->pending_bus_cmd = BUS_RDX;  /* Read exclusive for write */
                    core->pending_addr = addr;
                    
                    core->ex_mem.cache_op_done = true;
                    
                    /* Save store data for after block is fetched */
                    core->pending_store = true;
                    core->pending_store_addr = addr;
                    core->pending_store_data = data;
                    
                    /* Check if writeback needed for evicted block */
                    if (cache_needs_writeback(&core->cache, addr)) {
                        core->need_writeback = true;
                        core->writeback_addr = cache_get_current_block_addr(&core->cache, addr);
                    }
                }
                core->stall_mem = true;
                core->stall_decode = true;
                core->stall_fetch = true;
                continue;
            }
        }
        
        /* Advance instruction to WB stage */
        core->mem_wb = result;
        core->ex_mem.valid = false;
    }
    
    /* ========== EXECUTE STAGE ========== */
    /* Performs ALU operations and calculates memory addresses */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (core->stall_mem) {
            /* EX is also stalled when MEM is stalled */
            continue;
        }
        
        if (!saved_id_ex[i].valid) {
            core->id_ex.valid = false;
            continue;
        }
        
        Instruction *inst = &saved_id_ex[i].inst;
        int32_t rs_val = saved_id_ex[i].rs_val;
        int32_t rt_val = saved_id_ex[i].rt_val;
        int32_t result = 0;
        
        /* Cast to unsigned for logical operations */
        uint32_t urs = (uint32_t)rs_val;
        uint32_t urt = (uint32_t)rt_val;
        uint32_t ures = 0;
        
        /* Perform ALU operation based on opcode */
        switch (inst->opcode) {
            case OP_ADD:
                ures = urs + urt;
                result = (int32_t)ures;
                break;
            case OP_SUB:
                ures = urs - urt;
                result = (int32_t)ures;
                break;
            case OP_AND:
                ures = urs & urt;
                result = (int32_t)ures;
                break;
            case OP_OR:
                ures = urs | urt;
                result = (int32_t)ures;
                break;
            case OP_XOR:
                ures = urs ^ urt;
                result = (int32_t)ures;
                break;
            case OP_MUL:
                ures = urs * urt;
                result = (int32_t)ures;
                break;
            case OP_SLL:
                /* Shift left logical - shift amount masked to 5 bits */
                ures = urs << (urt & 0x1F);
                result = (int32_t)ures;
                break;
            case OP_SRA:
                /* Shift right arithmetic - preserves sign bit */
                result = rs_val >> (rt_val & 0x1F);
                break;
            case OP_SRL:
                /* Shift right logical - fills with zeros */
                ures = urs >> (urt & 0x1F);
                result = (int32_t)ures;
                break;
            case OP_JAL:
                /* Jump and link - result is return address (PC+1) */
                result = saved_id_ex[i].pc + 1;
                break;
            case OP_LW:
            case OP_SW:
                /* Memory operations - calculate effective address */
                ures = urs + urt;
                result = (int32_t)ures;
                break;
            default:
                break;
        }
        
        /* Advance instruction to MEM stage */
        core->ex_mem = saved_id_ex[i];
        core->ex_mem.alu_result = result;
        core->ex_mem.mem_addr = (uint32_t)result & 0x1FFFFF;  /* Mask to 21-bit address */
        core->ex_mem.cache_op_done = false;  /* Reset flag for new instruction */
        core->id_ex.valid = false;
    }
    
    /* ========== DECODE STAGE ========== */
    /* Decodes instruction, reads registers, checks hazards, resolves branches */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (core->stall_mem) {
            /* Propagate stall from MEM stage */
            core->stall_decode = true;
            core->stall_fetch = true;
            continue;
        }
        
        if (!saved_if_id[i].valid) {
            core->if_id.valid = false;
            core->stall_decode = false;
            core->stall_fetch = false;
            continue;
        }
        
        Instruction *inst = &saved_if_id[i].inst;
        
        /* Data hazard detection - check for RAW hazards with instructions in flight */
        /* Must use SAVED state (values at start of cycle) */
        bool has_hazard = false;
        
        /* Check if rs has a hazard (instruction in later stage will write to rs) */
        if (inst->rs != 0 && inst->rs != 1) {
            if (saved_id_ex[i].valid && saved_id_ex[i].writes_reg && saved_id_ex[i].dest_reg == inst->rs)
                has_hazard = true;
            if (saved_ex_mem[i].valid && saved_ex_mem[i].writes_reg && saved_ex_mem[i].dest_reg == inst->rs)
                has_hazard = true;
            if (saved_mem_wb[i].valid && saved_mem_wb[i].writes_reg && saved_mem_wb[i].dest_reg == inst->rs)
                has_hazard = true;
        }
        
        /* Check if rt has a hazard */
        if (inst->rt != 0 && inst->rt != 1) {
            if (saved_id_ex[i].valid && saved_id_ex[i].writes_reg && saved_id_ex[i].dest_reg == inst->rt)
                has_hazard = true;
            if (saved_ex_mem[i].valid && saved_ex_mem[i].writes_reg && saved_ex_mem[i].dest_reg == inst->rt)
                has_hazard = true;
            if (saved_mem_wb[i].valid && saved_mem_wb[i].writes_reg && saved_mem_wb[i].dest_reg == inst->rt)
                has_hazard = true;
        }
        
        /* Check rd for branches and stores (they read rd as source) */
        if ((is_branch(inst->opcode) || inst->opcode == OP_SW) && inst->rd != 0 && inst->rd != 1) {
            if (saved_id_ex[i].valid && saved_id_ex[i].writes_reg && saved_id_ex[i].dest_reg == inst->rd)
                has_hazard = true;
            if (saved_ex_mem[i].valid && saved_ex_mem[i].writes_reg && saved_ex_mem[i].dest_reg == inst->rd)
                has_hazard = true;
            if (saved_mem_wb[i].valid && saved_mem_wb[i].writes_reg && saved_mem_wb[i].dest_reg == inst->rd)
                has_hazard = true;
        }
        
        if (has_hazard) {
            /* Stall decode and fetch until hazard is resolved */
            core->stall_decode = true;
            core->stall_fetch = true;
            core->stats.decode_stall++;
            continue;
        }
        
        /* No hazard - clear stalls and proceed */
        core->stall_decode = false;
        core->stall_fetch = false;
        
        /* HALT handling: when HALT is in DECODE, stop fetching new instructions.
         * HALT discards the instruction in FETCH (delay slot not executed)
         * Then HALT flows through pipeline to WB before core fully stops.
         */
        if (inst->opcode == OP_HALT) {
            core->halted = true;  /* Prevents new fetches */
        }
        
        /* Update R1 with sign-extended immediate for this instruction */
        core->regs[1] = (uint32_t)inst->imm;
        
        /* Read register values */
        int32_t rs_val = read_reg(core, inst->rs, inst->imm);
        int32_t rt_val = read_reg(core, inst->rt, inst->imm);
        int32_t rd_val = read_reg(core, inst->rd, inst->imm);
        
        /* Early branch resolution - branches are resolved in decode stage */
        bool take_branch = false;
        
        switch (inst->opcode) {
            case OP_BEQ:
                take_branch = (rs_val == rt_val);
                break;
            case OP_BNE:
                take_branch = (rs_val != rt_val);
                break;
            case OP_BLT:
                take_branch = (rs_val < rt_val);
                break;
            case OP_BGT:
                take_branch = (rs_val > rt_val);
                break;
            case OP_BLE:
                take_branch = (rs_val <= rt_val);
                break;
            case OP_BGE:
                take_branch = (rs_val >= rt_val);
                break;
            case OP_JAL:
                take_branch = true;  /* JAL always jumps */
                break;
            default:
                break;
        }
        
        if (take_branch) {
            core->branch_taken = true;
            core->branch_target = rd_val & 0x3FF;  /* Target PC (10 bits) */
        }
        
        /* Prepare ID/EX pipeline register for next stage */
        core->id_ex.inst = *inst;
        core->id_ex.pc = saved_if_id[i].pc;
        core->id_ex.rs_val = rs_val;
        core->id_ex.rt_val = rt_val;
        core->id_ex.rd_val = rd_val;
        core->id_ex.mem_data = rd_val;  /* For stores: data to write */
        core->id_ex.valid = true;
        core->id_ex.writes_reg = writes_register(inst->opcode);
        core->id_ex.dest_reg = get_dest_reg(inst);
        core->id_ex.cache_op_done = false;
        
        core->if_id.valid = false;
    }
    
    /* ========== FETCH STAGE ========== */
    /* Fetches next instruction from instruction memory */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (core->halted) continue;  /* Don't fetch if halted */
        
        if (core->stall_fetch) {
            /* Keep IF/ID as is - don't fetch new instruction */
            continue;
        }
        
        /* Fetch instruction from IMEM at current PC */
        uint32_t raw = core->imem[core->pc & 0x3FF];
        
        /* Decode and store in IF/ID latch */
        core->if_id.inst = decode_instruction(raw);
        core->if_id.pc = core->pc;
        core->if_id.valid = true;
        core->if_id.cache_op_done = false;
        
        /* Increment PC for next fetch (delay slot instruction) */
        core->pc = (core->pc + 1) & 0x3FF;
    }
    
    /* Apply branch targets AFTER fetch to implement delay slot.
     * The instruction at PC+1 (delay slot) was just fetched above.
     * Now we redirect PC to branch target for the NEXT fetch.
     */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        if (core->branch_taken && !core->stall_fetch) {
            core->pc = core->branch_target;
        }
    }
    
    /* Update cycle counts for active cores */
    for (int i = 0; i < NUM_CORES; i++) {
        if (core_was_active[i]) {
            g_cores[i].stats.cycles = g_cycle + 1;
        }
    }
    
    g_cycle++;
}


/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

/*
 * main
 * -----
 * Entry point for the simulator.
 * Handles command-line arguments, initializes cores and memory,
 * runs the simulation loop, and writes output files.
 * 
 * Expected arguments (if provided): 28 filenames for all I/O files
 * If no arguments, uses default filenames.
 */
int main(int argc, char *argv[]) {
    /* Default filenames for input and output */
    const char *imem_files[NUM_CORES] = {"imem0.txt", "imem1.txt", "imem2.txt", "imem3.txt"};
    const char *memin_file = "memin.txt";
    const char *memout_file = "memout.txt";
    const char *regout_files[NUM_CORES] = {"regout0.txt", "regout1.txt", "regout2.txt", "regout3.txt"};
    const char *trace_files[NUM_CORES] = {"core0trace.txt", "core1trace.txt", "core2trace.txt", "core3trace.txt"};
    const char *bustrace_file = "bustrace.txt";
    const char *dsram_files[NUM_CORES] = {"dsram0.txt", "dsram1.txt", "dsram2.txt", "dsram3.txt"};
    const char *tsram_files[NUM_CORES] = {"tsram0.txt", "tsram1.txt", "tsram2.txt", "tsram3.txt"};
    const char *stats_files[NUM_CORES] = {"stats0.txt", "stats1.txt", "stats2.txt", "stats3.txt"};
    
    /* Parse command line arguments if all 28 are provided */
    if (argc == 28) {
        imem_files[0] = argv[1];
        imem_files[1] = argv[2];
        imem_files[2] = argv[3];
        imem_files[3] = argv[4];
        memin_file = argv[5];
        memout_file = argv[6];
        regout_files[0] = argv[7];
        regout_files[1] = argv[8];
        regout_files[2] = argv[9];
        regout_files[3] = argv[10];
        trace_files[0] = argv[11];
        trace_files[1] = argv[12];
        trace_files[2] = argv[13];
        trace_files[3] = argv[14];
        bustrace_file = argv[15];
        dsram_files[0] = argv[16];
        dsram_files[1] = argv[17];
        dsram_files[2] = argv[18];
        dsram_files[3] = argv[19];
        tsram_files[0] = argv[20];
        tsram_files[1] = argv[21];
        tsram_files[2] = argv[22];
        tsram_files[3] = argv[23];
        stats_files[0] = argv[24];
        stats_files[1] = argv[25];
        stats_files[2] = argv[26];
        stats_files[3] = argv[27];
    }
    
    /* Initialize all cores */
    for (int i = 0; i < NUM_CORES; i++) {
        init_core(&g_cores[i], i);
        load_imem(imem_files[i], g_cores[i].imem);
    }
    
    /* Initialize main memory from input file */
    load_main_mem(memin_file);
    
    /* Initialize bus state */
    memset(&g_bus, 0, sizeof(g_bus));
    g_last_granted = NUM_CORES - 1;  /* Start so core 0 has highest priority */
    
    /* Open trace files for writing */
    for (int i = 0; i < NUM_CORES; i++) {
        g_trace_files[i] = fopen(trace_files[i], "w");
    }
    g_bus_trace = fopen(bustrace_file, "w");
    
    /* Run simulation until all cores halt and pipelines drain */
    g_cycle = 0;
    while (!all_cores_done()) {
        simulate_cycle();
        
        /* Safety check to prevent infinite loops in case of bugs */
        if (g_cycle > 100000000) {
            fprintf(stderr, "Simulation exceeded maximum cycles\n");
            break;
        }
    }
    
    /* Close all trace files */
    for (int i = 0; i < NUM_CORES; i++) {
        if (g_trace_files[i]) fclose(g_trace_files[i]);
    }
    if (g_bus_trace) fclose(g_bus_trace);
    
    /* Write final output files */
    write_main_mem(memout_file);
    
    for (int i = 0; i < NUM_CORES; i++) {
        write_regout(regout_files[i], &g_cores[i]);
        write_dsram(dsram_files[i], &g_cores[i]);
        write_tsram(tsram_files[i], &g_cores[i]);
        write_stats(stats_files[i], &g_cores[i]);
    }
    
    return 0;
}