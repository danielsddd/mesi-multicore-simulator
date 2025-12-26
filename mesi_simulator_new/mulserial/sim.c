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

/* ============================================================================
 * OPCODE DEFINITIONS
 * ============================================================================ */

typedef enum {
    OP_ADD  = 0,
    OP_SUB  = 1,
    OP_AND  = 2,
    OP_OR   = 3,
    OP_XOR  = 4,
    OP_MUL  = 5,
    OP_SLL  = 6,
    OP_SRA  = 7,
    OP_SRL  = 8,
    OP_BEQ  = 9,
    OP_BNE  = 10,
    OP_BLT  = 11,
    OP_BGT  = 12,
    OP_BLE  = 13,
    OP_BGE  = 14,
    OP_JAL  = 15,
    OP_LW   = 16,
    OP_SW   = 17,
    OP_HALT = 20
} Opcode;

/* MESI protocol states for cache coherency */
typedef enum {
    MESI_INVALID   = 0,  // Block not valid in cache
    MESI_SHARED    = 1,  // Block is clean and may exist in other caches
    MESI_EXCLUSIVE = 2,  // Block is clean and only in this cache
    MESI_MODIFIED  = 3   // Block is dirty and only in this cache
} MesiState;

/* Bus commands for cache coherency protocol */
typedef enum {
    BUS_NO_CMD = 0,  // No operation on bus
    BUS_RD     = 1,  // Read request (shared access)
    BUS_RDX    = 2,  // Read exclusive request (for writing)
    BUS_FLUSH  = 3   // Flush data to bus (response to BusRd/BusRdX)
} BusCmd;

#define BUS_ORIGID_MEM 4  // Indicates main memory is the originator

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * Decoded instruction structure
 * Holds all the fields extracted from a 32-bit instruction word
 */
typedef struct {
    uint8_t  opcode;
    uint8_t  rd;        // Destination register
    uint8_t  rs;        // Source register 1
    uint8_t  rt;        // Source register 2
    int32_t  imm;       /* sign-extended immediate value */
    uint32_t raw;       /* original instruction word */
} Instruction;

/**
 * Pipeline register structure
 * Passed between pipeline stages to carry instruction state
 */
typedef struct {
    Instruction inst;
    uint32_t    pc;
    int32_t     rs_val;      // Value read from rs
    int32_t     rt_val;      // Value read from rt
    int32_t     rd_val;      // Value read from rd (for branches/stores)
    int32_t     alu_result;  // Result from ALU
    uint32_t    mem_addr;    // Memory address for load/store
    int32_t     mem_data;    // Data to write for stores
    bool        valid;       // Is this stage active?
    bool        writes_reg;  // Does instruction write to a register?
    uint8_t     dest_reg;    // Which register is being written?
} PipelineReg;

/**
 * Cache structure with data and tag SRAMs
 */
typedef struct {
    uint32_t dsram[CACHE_SIZE];                 /* Data SRAM - actual data */
    uint32_t tsram[NUM_CACHE_BLOCKS];           /* Tag SRAM - MESI state + tag bits */
} Cache;

/**
 * Bus transaction state
 * Tracks ongoing bus transactions and arbitration
 */
typedef struct {
    // Current bus state (visible to all cores)
    int      origid;
    BusCmd   cmd;
    uint32_t addr;
    uint32_t data;
    int      shared;  // Set to 1 if block exists in multiple caches
    
    /* State for multi-cycle transactions */
    bool     busy;                  /* Transaction in progress */
    int      flush_origid;          /* Who is providing the flush data */
    int      requesting_core;       /* Which core requested the data */
    int      delay_counter;         /* Countdown for memory latency */
    int      words_transferred;     /* How many words sent so far */
    uint32_t block_buffer[BLOCK_SIZE];  /* Buffer for incoming block data */
    BusCmd   original_cmd;          /* Original command (BusRd/BusRdX) */
    uint32_t original_addr;         /* Original block address */
    int      sampled_shared;        /* Sampled bus_shared value when request issued */
    
    /* Writeback (eviction) tracking */
    bool     writeback_in_progress; /* Currently writing back modified block */
    int      writeback_core;        /* Core doing the writeback */
    uint32_t writeback_addr;        /* Block address being written back */
    int      writeback_word;        /* Current word index being written */
} Bus;

/**
 * Core statistics for performance analysis
 */
typedef struct {
    int cycles;
    int instructions;
    int read_hit;
    int write_hit;
    int read_miss;
    int write_miss;
    int decode_stall;
    int mem_stall;
} Stats;

/**
 * Core state structure
 * Contains all state for a single processor core
 */
typedef struct {
    int          id;
    uint32_t     regs[NUM_REGISTERS];
    uint32_t     pc;
    uint32_t     imem[IMEM_SIZE];
    Cache        cache;
    
    /* Pipeline registers between stages */
    PipelineReg  if_id;     /* Fetch -> Decode */
    PipelineReg  id_ex;     /* Decode -> Execute */
    PipelineReg  ex_mem;    /* Execute -> Memory */
    PipelineReg  mem_wb;    /* Memory -> Writeback */
    
    /* Pipeline control signals */
    bool         halted;            /* Core has executed HALT */
    bool         stall_fetch;       /* Stall fetch stage */
    bool         stall_decode;      /* Stall decode stage */
    bool         stall_mem;         /* Stall mem stage */
    
    /* Branch handling */
    bool         branch_taken;      /* Branch was taken this cycle */
    uint32_t     branch_target;     /* Target PC for branch */
    
    /* Cache transaction state */
    bool         cache_busy;         // Cache is waiting for bus transaction
    bool         waiting_for_bus;    // Waiting for bus access
    BusCmd       pending_bus_cmd;    // Command to issue when bus granted
    uint32_t     pending_addr;       // Address for pending request
    bool         need_writeback;     // Need to writeback before fetch
    uint32_t     writeback_addr;     // Address of block to writeback
    
    /* For pending store after cache fill */
    bool         pending_store;      // Store waiting to complete after fill
    uint32_t     pending_store_addr; // Address for pending store
    uint32_t     pending_store_data; // Data for pending store
    
    Stats        stats;
} Core;

/* ============================================================================
 * GLOBAL STATE
 * ============================================================================ */

static Core     g_cores[NUM_CORES];
static uint32_t g_main_mem[MAIN_MEM_SIZE];
static Bus      g_bus;
static int      g_last_granted;     /* For round-robin arbitration */
static int      g_cycle;

/* Output file pointers */
static FILE    *g_trace_files[NUM_CORES];
static FILE    *g_bus_trace;

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * Sign-extend 12-bit immediate to 32-bit signed integer
 * Checks bit 11 and extends if negative
 */
static int32_t sign_extend_12(uint32_t val) {
    if (val & 0x800) {
        return (int32_t)(val | 0xFFFFF000);
    }
    return (int32_t)val;
}

/**
 * Extract cache tag from memory address
 * Tag is bits [20:9] of the address
 */
static uint32_t get_tag(uint32_t addr) {
    return (addr >> 9) & 0xFFF;  /* bits [20:9] */
}

/**
 * Extract cache index from memory address
 * Index is bits [8:3] - selects which cache line
 */
static uint32_t get_index(uint32_t addr) {
    return (addr >> 3) & 0x3F;   /* bits [8:3] */
}

/**
 * Extract block offset from memory address
 * Offset is bits [2:0] - word within the block
 */
static uint32_t get_offset(uint32_t addr) {
    return addr & 0x7;           /* bits [2:0] */
}

/**
 * Get block-aligned address (zero out offset bits)
 */
static uint32_t get_block_addr(uint32_t addr) {
    return addr & ~0x7;
}

/**
 * Extract MESI state from TSRAM entry
 * MESI state is stored in bits [13:12]
 */
static MesiState get_mesi_state(uint32_t tsram_entry) {
    return (MesiState)((tsram_entry >> 12) & 0x3);
}

/**
 * Extract tag from TSRAM entry
 * Tag is stored in bits [11:0]
 */
static uint32_t get_tsram_tag(uint32_t tsram_entry) {
    return tsram_entry & 0xFFF;
}

/**
 * Create TSRAM entry from MESI state and tag
 * Packs both into a single 32-bit word
 */
static uint32_t make_tsram_entry(MesiState mesi, uint32_t tag) {
    return ((uint32_t)mesi << 12) | (tag & 0xFFF);
}

/**
 * Decode a 32-bit instruction word into its components
 * Extracts opcode, registers, and immediate value
 */
static Instruction decode_instruction(uint32_t raw) {
    Instruction inst;
    inst.raw    = raw;
    inst.opcode = (raw >> 24) & 0xFF;
    inst.rd     = (raw >> 20) & 0xF;
    inst.rs     = (raw >> 16) & 0xF;
    inst.rt     = (raw >> 12) & 0xF;
    inst.imm    = sign_extend_12(raw & 0xFFF);
    return inst;
}

/**
 * Check if opcode is a branch instruction
 */
static bool is_branch(uint8_t opcode) {
    return opcode >= OP_BEQ && opcode <= OP_JAL;
}

/**
 * Check if instruction writes to a register
 */
static bool writes_register(uint8_t opcode) {
    return opcode <= OP_SRL || opcode == OP_JAL || opcode == OP_LW;
}

/**
 * Get destination register for an instruction
 * JAL always writes to R15, others use rd field
 */
static uint8_t get_dest_reg(Instruction *inst) {
    if (inst->opcode == OP_JAL) {
        return 15;  /* jal always writes to R15 */
    }
    if (writes_register(inst->opcode)) {
        return inst->rd;
    }
    return 0;  /* No destination */
}

/* ============================================================================
 * FILE I/O FUNCTIONS
 * ============================================================================ */

/**
 * Load instruction memory from file
 * Each line is a 32-bit hex instruction
 */
static void load_imem(const char *filename, uint32_t *imem) {
    FILE *f = fopen(filename, "r");
    memset(imem, 0, IMEM_SIZE * sizeof(uint32_t));
    
    if (!f) return;
    
    char line[64];
    int addr = 0;
    while (addr < IMEM_SIZE && fgets(line, sizeof(line), f)) {
        imem[addr++] = (uint32_t)strtoul(line, NULL, 16);
    }
    
    fclose(f);
}

/**
 * Load main memory from file
 * Each line is a 32-bit hex word
 */
static void load_main_mem(const char *filename) {
    FILE *f = fopen(filename, "r");
    memset(g_main_mem, 0, sizeof(g_main_mem));
    
    if (!f) return;
    
    char line[64];
    int addr = 0;
    while (addr < MAIN_MEM_SIZE && fgets(line, sizeof(line), f)) {
        g_main_mem[addr++] = (uint32_t)strtoul(line, NULL, 16);
    }
    
    fclose(f);
}

/**
 * Write main memory to file at end of simulation
 * Only writes up to last non-zero word to save space
 */
static void write_main_mem(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    /* Find last non-zero word */
    int last = 0;
    for (int i = 0; i < MAIN_MEM_SIZE; i++) {
        if (g_main_mem[i] != 0) {
            last = i;
        }
    }
    
    /* Write up to and including last non-zero word */
    for (int i = 0; i <= last; i++) {
        fprintf(f, "%08X\n", g_main_mem[i]);
    }
    
    fclose(f);
}

/**
 * Write register file to output file
 * Only writes R2-R15 (R0 is constant 0, R1 is immediate)
 */
static void write_regout(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    /* Write R2-R15 only */
    for (int i = 2; i < NUM_REGISTERS; i++) {
        fprintf(f, "%08X\n", core->regs[i]);
    }
    
    fclose(f);
}

/**
 * Write cache data SRAM to file
 */
static void write_dsram(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    for (int i = 0; i < CACHE_SIZE; i++) {
        fprintf(f, "%08X\n", core->cache.dsram[i]);
    }
    
    fclose(f);
}

/**
 * Write cache tag SRAM to file
 */
static void write_tsram(const char *filename, Core *core) {
    FILE *f = fopen(filename, "w");
    if (!f) return;
    
    for (int i = 0; i < NUM_CACHE_BLOCKS; i++) {
        fprintf(f, "%08X\n", core->cache.tsram[i]);
    }
    
    fclose(f);
}

/**
 * Write performance statistics to file
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

/**
 * Write core pipeline trace for current cycle
 * Shows which instruction is in each pipeline stage and register values
 */
static void write_core_trace(Core *core) {
    FILE *f = g_trace_files[core->id];
    if (!f) return;
    
    /* Determine activity of each stage */
    bool fetch_active = !core->halted;
    bool decode_active = core->if_id.valid;
    bool exec_active = core->id_ex.valid;
    bool mem_active = core->ex_mem.valid;
    bool wb_active = core->mem_wb.valid;
    
    bool any_active = fetch_active || decode_active || exec_active || mem_active || wb_active;
    
    // Don't write trace if no stages are active
    if (!any_active) return;
    
    fprintf(f, "%d ", g_cycle);
    
    /* Fetch stage - PC being fetched this cycle */
    if (fetch_active && !core->stall_fetch) {
        fprintf(f, "%03X ", core->pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Decode stage */
    if (decode_active) {
        fprintf(f, "%03X ", core->if_id.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Execute stage */
    if (exec_active) {
        fprintf(f, "%03X ", core->id_ex.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Memory stage */
    if (mem_active) {
        fprintf(f, "%03X ", core->ex_mem.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Writeback stage */
    if (wb_active) {
        fprintf(f, "%03X ", core->mem_wb.pc & 0x3FF);
    } else {
        fprintf(f, "--- ");
    }
    
    /* Registers R2-R15 (values at START of cycle) */
    for (int i = 2; i < NUM_REGISTERS; i++) {
        fprintf(f, "%08X", core->regs[i]);
        if (i < NUM_REGISTERS - 1) fprintf(f, " ");
    }
    fprintf(f, "\n");
}

/**
 * Write bus trace for current cycle
 * Only writes if there's a command on the bus
 */
static void write_bus_trace(void) {
    if (g_bus.cmd == BUS_NO_CMD) return;
    if (!g_bus_trace) return;
    
    fprintf(g_bus_trace, "%d %X %X %06X %08X %X\n",
            g_cycle, g_bus.origid, g_bus.cmd, 
            g_bus.addr & 0x1FFFFF, g_bus.data, g_bus.shared);
}

/* ============================================================================
 * CACHE OPERATIONS
 * ============================================================================ */

/**
 * Check if cache has a block (tag match and valid state)
 * Returns true if block is present and not invalid
 */
static bool cache_has_block(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    uint32_t tag = get_tag(addr);
    uint32_t tsram_entry = cache->tsram[index];
    MesiState state = get_mesi_state(tsram_entry);
    uint32_t stored_tag = get_tsram_tag(tsram_entry);
    
    return (state != MESI_INVALID) && (stored_tag == tag);
}

/**
 * Read word from cache
 * Assumes cache hit (caller should check first)
 */
static uint32_t cache_read(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    uint32_t offset = get_offset(addr);
    return cache->dsram[index * BLOCK_SIZE + offset];
}

/**
 * Write word to cache
 */
static void cache_write(Cache *cache, uint32_t addr, uint32_t data) {
    uint32_t index = get_index(addr);
    uint32_t offset = get_offset(addr);
    cache->dsram[index * BLOCK_SIZE + offset] = data;
}

/**
 * Get MESI state for a block
 */
static MesiState cache_get_state(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    return get_mesi_state(cache->tsram[index]);
}

/**
 * Set MESI state for a block
 */
static void cache_set_state(Cache *cache, uint32_t addr, MesiState state) {
    uint32_t index = get_index(addr);
    uint32_t tag = get_tag(addr);
    cache->tsram[index] = make_tsram_entry(state, tag);
}

/**
 * Check if current block needs writeback before eviction
 * Returns true if different tag and block is modified
 */
static bool cache_needs_writeback(Cache *cache, uint32_t addr) {
    uint32_t index = get_index(addr);
    uint32_t tsram_entry = cache->tsram[index];
    MesiState state = get_mesi_state(tsram_entry);
    uint32_t stored_tag = get_tsram_tag(tsram_entry);
    uint32_t new_tag = get_tag(addr);
    
    /* Need writeback if different tag and modified */
    return (stored_tag != new_tag) && (state == MESI_MODIFIED);
}

/**
 * Get address of block currently in cache line (for writeback)
 * Reconstructs address from stored tag and index
 */
static uint32_t cache_get_current_block_addr(Cache *cache, uint32_t new_addr) {
    uint32_t index = get_index(new_addr);
    uint32_t stored_tag = get_tsram_tag(cache->tsram[index]);
    return (stored_tag << 9) | (index << 3);
}

/**
 * Fill cache block from buffer
 * Copies entire block into cache and updates tag/state
 */
static void cache_fill_block(Cache *cache, uint32_t addr, uint32_t *data, MesiState state) {
    uint32_t index = get_index(addr);
    uint32_t tag = get_tag(addr);
    
    /* Write data to DSRAM */
    for (int i = 0; i < BLOCK_SIZE; i++) {
        cache->dsram[index * BLOCK_SIZE + i] = data[i];
    }
    
    /* Update TSRAM */
    cache->tsram[index] = make_tsram_entry(state, tag);
}

/* ============================================================================
 * BUS OPERATIONS
 * ============================================================================ */

/**
 * Round-robin bus arbitration
 * Grants bus to next requesting core after last granted
 */
static int arbitrate_bus(bool requests[NUM_CORES]) {
    /* Start from core after last granted */
    for (int i = 1; i <= NUM_CORES; i++) {
        int idx = (g_last_granted + i) % NUM_CORES;
        if (requests[idx]) {
            return idx;
        }
    }
    return -1;  // No requests
}

/* ============================================================================
 * SNOOPING LOGIC
 * ============================================================================ */

/**
 * Check if any cache has the block and set bus_shared accordingly
 * Used when issuing BusRd to determine if block is shared
 */
static void snoop_and_set_shared(uint32_t addr, int requesting_core) {
    g_bus.shared = 0;
    
    for (int i = 0; i < NUM_CORES; i++) {
        if (i == requesting_core) continue;
        
        Cache *cache = &g_cores[i].cache;
        if (cache_has_block(cache, addr)) {
            g_bus.shared = 1;
            break;
        }
    }
}

/**
 * Find which core has the block in Modified state
 * Returns core ID or -1 if no core has it modified
 */
static int find_modified_owner(uint32_t addr, int requesting_core) {
    for (int i = 0; i < NUM_CORES; i++) {
        if (i == requesting_core) continue;
        
        Cache *cache = &g_cores[i].cache;
        if (cache_has_block(cache, addr)) {
            if (cache_get_state(cache, addr) == MESI_MODIFIED) {
                return i;
            }
        }
    }
    return -1;
}

/**
 * Update snooping caches when BusRd/BusRdX is issued
 * Implements MESI state transitions on other caches
 */
static void snoop_update_states(uint32_t addr, BusCmd cmd, int requesting_core) {
    for (int i = 0; i < NUM_CORES; i++) {
        if (i == requesting_core) continue;
        
        Cache *cache = &g_cores[i].cache;
        if (!cache_has_block(cache, addr)) continue;
        
        MesiState state = cache_get_state(cache, addr);
        
        if (cmd == BUS_RD) {
            /* Another core wants to read */
            switch (state) {
                case MESI_EXCLUSIVE:
                    // Transition from Exclusive to Shared
                    cache_set_state(cache, addr, MESI_SHARED);
                    break;
                case MESI_MODIFIED:
                    /* Will be handled by flush, state updated after flush completes */
                    break;
                default:
                    break;
            }
        } else if (cmd == BUS_RDX) {
            /* Another core wants exclusive access */
            switch (state) {
                case MESI_SHARED:
                case MESI_EXCLUSIVE:
                    // Invalidate block since other core wants to write
                    cache_set_state(cache, addr, MESI_INVALID);
                    break;
                case MESI_MODIFIED:
                    /* Will be handled by flush, state updated after flush completes */
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

/**
 * Read register value (handles R0 and R1 specially)
 * R0 is always 0, R1 is the immediate value
 */
static int32_t read_reg(Core *core, uint8_t reg, int32_t imm) {
    if (reg == 0) return 0;
    if (reg == 1) return imm;
    return (int32_t)core->regs[reg];
}

/**
 * Write register value (ignores R0 and R1)
 * R0 and R1 are read-only
 */
static void write_reg(Core *core, uint8_t reg, int32_t value) {
    if (reg == 0 || reg == 1) return;
    core->regs[reg] = (uint32_t)value;
}

/* ============================================================================
 * BUS TRANSACTION PROCESSING
 * ============================================================================ */

/**
 * Process bus transactions for current cycle
 * Handles writebacks, flushes, and new requests
 */
static void process_bus_transactions(void) {
    /* Default: no command on bus */
    g_bus.cmd = BUS_NO_CMD;
    
    /* Handle ongoing writeback (eviction) - bus is locked during this */
    if (g_bus.writeback_in_progress) {
        Core *core = &g_cores[g_bus.writeback_core];
        uint32_t wb_addr = g_bus.writeback_addr + g_bus.writeback_word;
        
        // Set bus signals for this word
        g_bus.origid = core->id;
        g_bus.cmd = BUS_FLUSH;
        g_bus.addr = wb_addr;
        g_bus.data = cache_read(&core->cache, wb_addr);
        g_bus.shared = 0;
        
        /* Write to main memory */
        g_main_mem[wb_addr] = g_bus.data;
        
        g_bus.writeback_word++;
        if (g_bus.writeback_word >= BLOCK_SIZE) {
            /* Writeback complete, invalidate old block */
            cache_set_state(&core->cache, g_bus.writeback_addr, MESI_INVALID);
            core->need_writeback = false;
            g_bus.writeback_in_progress = false;
            /* Bus is now free for the BusRd/BusRdX - will happen next cycle */
        }
        return;
    }
    
    /* Handle ongoing flush transaction (response to BusRd/BusRdX) */
    if (g_bus.busy) {
        if (g_bus.delay_counter > 0) {
            /* Still waiting for memory latency */
            g_bus.delay_counter--;
            return;
        }
        
        /* Transfer next word */
        int word_idx = g_bus.words_transferred;
        uint32_t word_addr = g_bus.original_addr + word_idx;
        
        if (g_bus.flush_origid == BUS_ORIGID_MEM) {
            /* Reading from main memory */
            g_bus.block_buffer[word_idx] = g_main_mem[word_addr];
            g_bus.data = g_bus.block_buffer[word_idx];
        } else {
            /* Reading from another cache (Modified) */
            Core *src_core = &g_cores[g_bus.flush_origid];
            g_bus.block_buffer[word_idx] = cache_read(&src_core->cache, word_addr);
            g_bus.data = g_bus.block_buffer[word_idx];
            
            /* Also update main memory */
            g_main_mem[word_addr] = g_bus.data;
        }
        
        /* Set bus state for trace */
        g_bus.cmd = BUS_FLUSH;
        g_bus.origid = g_bus.flush_origid;
        g_bus.addr = word_addr;
        g_bus.shared = g_bus.sampled_shared;  /* Maintain sampled value */
        
        g_bus.words_transferred++;
        
        if (g_bus.words_transferred >= BLOCK_SIZE) {
            /* Transaction complete - fill cache and clean up */
            Core *req_core = &g_cores[g_bus.requesting_core];
            
            /* Determine new MESI state based on request type and sharing */
            MesiState new_state;
            if (g_bus.original_cmd == BUS_RDX) {
                new_state = MESI_MODIFIED;  /* Will write immediately */
            } else {
                new_state = (g_bus.sampled_shared ? MESI_SHARED : MESI_EXCLUSIVE);
            }
            
            /* Fill cache block */
            cache_fill_block(&req_core->cache, g_bus.original_addr, g_bus.block_buffer, new_state);
            
            /* If there's a pending store, do it now */
            if (req_core->pending_store) {
                cache_write(&req_core->cache, req_core->pending_store_addr, req_core->pending_store_data);
                cache_set_state(&req_core->cache, req_core->pending_store_addr, MESI_MODIFIED);
                req_core->pending_store = false;
            }
            
            /* If source was Modified cache, update its state */
            if (g_bus.flush_origid != BUS_ORIGID_MEM) {
                Core *src_core = &g_cores[g_bus.flush_origid];
                if (g_bus.original_cmd == BUS_RD) {
                    cache_set_state(&src_core->cache, g_bus.original_addr, MESI_SHARED);
                } else {
                    cache_set_state(&src_core->cache, g_bus.original_addr, MESI_INVALID);
                }
            }
            
            /* Clear stalls - cache is ready now */
            req_core->cache_busy = false;
            req_core->stall_mem = false;
            req_core->stall_decode = false;
            req_core->stall_fetch = false;
            
            /* Clear bus state */
            g_bus.busy = false;
        }
        return;
    }
    
    /* Check for new bus requests from cores */
    bool requests[NUM_CORES] = {false};
    
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        /* Request bus if need writeback or waiting for data */
        if (core->need_writeback || core->waiting_for_bus) {
            requests[i] = true;
        }
    }
    
    /* Arbitrate - who gets the bus this cycle? */
    int granted = arbitrate_bus(requests);
    if (granted < 0) {
        return;  // No requests
    }
    
    Core *core = &g_cores[granted];
    g_last_granted = granted;
    
    if (core->need_writeback) {
        /* Start writeback - this locks the bus for multiple cycles */
        g_bus.writeback_in_progress = true;
        g_bus.writeback_core = core->id;
        g_bus.writeback_addr = core->writeback_addr;
        g_bus.writeback_word = 0;
        
        /* Send first word */
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
            /* Single-word block? Unlikely but handle it */
            cache_set_state(&core->cache, g_bus.writeback_addr, MESI_INVALID);
            core->need_writeback = false;
            g_bus.writeback_in_progress = false;
        }
    } else {
        /* Initiate BusRd or BusRdX */
        uint32_t block_addr = get_block_addr(core->pending_addr);
        
        // Issue the command on the bus
        g_bus.origid = core->id;
        g_bus.cmd = core->pending_bus_cmd;
        g_bus.addr = block_addr;
        g_bus.data = 0;
        g_bus.requesting_core = core->id;
        g_bus.original_cmd = core->pending_bus_cmd;
        g_bus.original_addr = block_addr;
        
        core->waiting_for_bus = false;
        
        /* Snoop: set bus_shared and sample it NOW */
        snoop_and_set_shared(block_addr, core->id);
        g_bus.sampled_shared = g_bus.shared;
        
        /* Update states of snooping caches */
        snoop_update_states(block_addr, core->pending_bus_cmd, core->id);
        
        /* Check if any cache has Modified copy */
        int mod_owner = find_modified_owner(block_addr, core->id);
        
        if (mod_owner >= 0) {
            /* Another cache will provide the data (cache-to-cache transfer) */
            g_bus.flush_origid = mod_owner;
            g_bus.shared = 1;
            g_bus.sampled_shared = 1;
            g_bus.delay_counter = 0;  /* No delay for cache-to-cache */
        } else {
            /* Main memory will provide the data */
            g_bus.flush_origid = BUS_ORIGID_MEM;
            g_bus.delay_counter = MEM_LATENCY - 1;  // First word takes 16 cycles
        }
        
        g_bus.busy = true;
        g_bus.words_transferred = 0;
    }
}

/* ============================================================================
 * SIMULATION MAIN LOOP
 * ============================================================================ */

/**
 * Initialize a core to default state
 */
static void init_core(Core *core, int id) {
    memset(core, 0, sizeof(Core));
    core->id = id;
}

/**
 * Check if all cores are done and pipelines are empty
 */
static bool all_cores_done(void) {
    for (int i = 0; i < NUM_CORES; i++) {
        // Check if core is still running or has instructions in pipeline
        if (!g_cores[i].halted) return false;
        if (g_cores[i].if_id.valid) return false;
        if (g_cores[i].id_ex.valid) return false;
        if (g_cores[i].ex_mem.valid) return false;
        if (g_cores[i].mem_wb.valid) return false;
    }
    // Also check if bus is still busy
    if (g_bus.busy) return false;
    if (g_bus.writeback_in_progress) return false;
    return true;
}

/**
 * Simulate one clock cycle for all cores
 * Processes bus first, then pipeline stages in reverse order (WB -> Fetch)
 */
static void simulate_cycle(void) {
    /* Write trace at start of cycle (before any state changes) */
    for (int i = 0; i < NUM_CORES; i++) {
        write_core_trace(&g_cores[i]);
    }
    
    /* Process bus transactions first */
    process_bus_transactions();
    
    /* Write bus trace */
    write_bus_trace();
    
    /* Save current pipeline state for proper stage advancement
     * Need to save because we update in reverse order (WB -> Fetch)
     * but each stage should see the state from START of cycle
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
        g_cores[i].branch_taken = false;
    }
    
    /* ========== WRITEBACK STAGE ========== */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (!saved_mem_wb[i].valid) continue;
        
        Instruction *inst = &saved_mem_wb[i].inst;
        
        /* Count executed instructions (not halts) */
        if (inst->opcode != OP_HALT) {
            core->stats.instructions++;
        }
        
        /* Write result to register file */
        if (saved_mem_wb[i].writes_reg) {
            write_reg(core, saved_mem_wb[i].dest_reg, saved_mem_wb[i].alu_result);
        }
        
        /* Check for halt completion */
        if (inst->opcode == OP_HALT) {
            core->halted = true;
        }
        
        // Clear the WB stage
        core->mem_wb.valid = false;
    }
    
    /* ========== MEMORY STAGE ========== */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (!saved_ex_mem[i].valid) {
            if (!core->stall_mem) {
                core->ex_mem.valid = false;
            }
            continue;
        }
        
        /* Check if stalled on memory access */
        if (core->stall_mem) {
            core->stats.mem_stall++;
            continue;  /* Keep instruction in MEM stage */
        }
        
        Instruction *inst = &saved_ex_mem[i].inst;
        PipelineReg result = saved_ex_mem[i];
        
        if (inst->opcode == OP_LW) {
            uint32_t addr = saved_ex_mem[i].mem_addr;
            
            if (cache_has_block(&core->cache, addr)) {
                /* Cache hit - read the data */
                core->stats.read_hit++;
                result.alu_result = (int32_t)cache_read(&core->cache, addr);
            } else {
                /* Cache miss - need to fetch block from bus */
                if (!core->cache_busy) {
                    core->stats.read_miss++;
                    core->cache_busy = true;
                    core->waiting_for_bus = true;
                    core->pending_bus_cmd = BUS_RD;
                    core->pending_addr = addr;
                    
                    // Check if we need to writeback current block first
                    if (cache_needs_writeback(&core->cache, addr)) {
                        core->need_writeback = true;
                        core->writeback_addr = cache_get_current_block_addr(&core->cache, addr);
                    }
                }
                // Stall pipeline until data arrives
                core->stall_mem = true;
                core->stall_decode = true;
                core->stall_fetch = true;
                core->stats.mem_stall++;
                continue;
            }
        } else if (inst->opcode == OP_SW) {
            uint32_t addr = saved_ex_mem[i].mem_addr;
            uint32_t data = (uint32_t)saved_ex_mem[i].mem_data;
            
            if (cache_has_block(&core->cache, addr)) {
                MesiState state = cache_get_state(&core->cache, addr);
                
                if (state == MESI_MODIFIED || state == MESI_EXCLUSIVE) {
                    /* Cache hit with write permission - can write directly */
                    core->stats.write_hit++;
                    cache_write(&core->cache, addr, data);
                    cache_set_state(&core->cache, addr, MESI_MODIFIED);
                } else if (state == MESI_SHARED) {
                    /* Have the block but it's shared - need BusRdX for exclusive access */
                    if (!core->cache_busy) {
                        core->stats.write_miss++;
                        core->cache_busy = true;
                        core->waiting_for_bus = true;
                        core->pending_bus_cmd = BUS_RDX;
                        core->pending_addr = addr;
                        core->pending_store = true;
                        core->pending_store_addr = addr;
                        core->pending_store_data = data;
                    }
                    core->stall_mem = true;
                    core->stall_decode = true;
                    core->stall_fetch = true;
                    core->stats.mem_stall++;
                    continue;
                }
            } else {
                /* Cache miss - need BusRdX to get block with write permission */
                if (!core->cache_busy) {
                    core->stats.write_miss++;
                    core->cache_busy = true;
                    core->waiting_for_bus = true;
                    core->pending_bus_cmd = BUS_RDX;
                    core->pending_addr = addr;
                    core->pending_store = true;
                    core->pending_store_addr = addr;
                    core->pending_store_data = data;
                    
                    // Check if we need to writeback current block first
                    if (cache_needs_writeback(&core->cache, addr)) {
                        core->need_writeback = true;
                        core->writeback_addr = cache_get_current_block_addr(&core->cache, addr);
                    }
                }
                core->stall_mem = true;
                core->stall_decode = true;
                core->stall_fetch = true;
                core->stats.mem_stall++;
                continue;
            }
        }
        
        /* Advance to WB stage */
        core->mem_wb = result;
        core->ex_mem.valid = false;
    }
    
    /* ========== EXECUTE STAGE ========== */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (core->stall_mem) {
            /* EX is also stalled if MEM is stalled */
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
        
        // Perform ALU operation based on opcode
        switch (inst->opcode) {
            case OP_ADD:
                result = rs_val + rt_val;
                break;
            case OP_SUB:
                result = rs_val - rt_val;
                break;
            case OP_AND:
                result = rs_val & rt_val;
                break;
            case OP_OR:
                result = rs_val | rt_val;
                break;
            case OP_XOR:
                result = rs_val ^ rt_val;
                break;
            case OP_MUL:
                result = rs_val * rt_val;
                break;
            case OP_SLL:
                result = rs_val << (rt_val & 0x1F);
                break;
            case OP_SRA:
                result = rs_val >> (rt_val & 0x1F);  // Arithmetic shift
                break;
            case OP_SRL:
                result = (int32_t)((uint32_t)rs_val >> (rt_val & 0x1F));  // Logical shift
                break;
            case OP_JAL:
                result = saved_id_ex[i].pc + 1;  // Return address
                break;
            case OP_LW:
            case OP_SW:
                result = rs_val + rt_val;  // Calculate memory address
                break;
            default:
                break;
        }
        
        // Advance to MEM stage
        core->ex_mem = saved_id_ex[i];
        core->ex_mem.alu_result = result;
        core->ex_mem.mem_addr = (uint32_t)result & 0x1FFFFF;  // Mask to 21 bits
        core->id_ex.valid = false;
    }
    
    /* ========== DECODE STAGE ========== */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (core->stall_mem) {
            // Propagate stall from MEM
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
        
        /* Check for data hazards - must use SAVED registers (state at start of cycle) */
        bool has_hazard = false;
        
        /* Check rs for hazard */
        if (inst->rs != 0 && inst->rs != 1) {
            // Check if any later stage is writing to rs
            if (saved_id_ex[i].valid && saved_id_ex[i].writes_reg && saved_id_ex[i].dest_reg == inst->rs)
                has_hazard = true;
            if (saved_ex_mem[i].valid && saved_ex_mem[i].writes_reg && saved_ex_mem[i].dest_reg == inst->rs)
                has_hazard = true;
            if (saved_mem_wb[i].valid && saved_mem_wb[i].writes_reg && saved_mem_wb[i].dest_reg == inst->rs)
                has_hazard = true;
        }
        
        /* Check rt for hazard */
        if (inst->rt != 0 && inst->rt != 1) {
            if (saved_id_ex[i].valid && saved_id_ex[i].writes_reg && saved_id_ex[i].dest_reg == inst->rt)
                has_hazard = true;
            if (saved_ex_mem[i].valid && saved_ex_mem[i].writes_reg && saved_ex_mem[i].dest_reg == inst->rt)
                has_hazard = true;
            if (saved_mem_wb[i].valid && saved_mem_wb[i].writes_reg && saved_mem_wb[i].dest_reg == inst->rt)
                has_hazard = true;
        }
        
        /* Check rd for branches and stores (they read rd) */
        if ((is_branch(inst->opcode) || inst->opcode == OP_SW) && inst->rd != 0 && inst->rd != 1) {
            if (saved_id_ex[i].valid && saved_id_ex[i].writes_reg && saved_id_ex[i].dest_reg == inst->rd)
                has_hazard = true;
            if (saved_ex_mem[i].valid && saved_ex_mem[i].writes_reg && saved_ex_mem[i].dest_reg == inst->rd)
                has_hazard = true;
            if (saved_mem_wb[i].valid && saved_mem_wb[i].writes_reg && saved_mem_wb[i].dest_reg == inst->rd)
                has_hazard = true;
        }
        
        if (has_hazard) {
            // Stall decode and fetch until hazard is resolved
            core->stall_decode = true;
            core->stall_fetch = true;
            core->stats.decode_stall++;
            continue;
        }
        
        /* No hazard - clear stalls and proceed */
        core->stall_decode = false;
        core->stall_fetch = false;
        
        /* HALT handling: when HALT is in DECODE, stop fetching new instructions.
         * According to spec: HALT in DECODE discards instruction in FETCH,
         * then HALT flows through pipeline to WB before core fully stops.
         */
        if (inst->opcode == OP_HALT) {
            core->halted = true;  /* Prevents new fetches */
        }
        
        /* Update R1 with sign-extended immediate */
        core->regs[1] = (uint32_t)inst->imm;
        
        /* Read register values */
        int32_t rs_val = read_reg(core, inst->rs, inst->imm);
        int32_t rt_val = read_reg(core, inst->rt, inst->imm);
        int32_t rd_val = read_reg(core, inst->rd, inst->imm);
        
        /* Handle branches in decode stage (early resolution) */
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
                take_branch = true;
                break;
            default:
                break;
        }
        
        if (take_branch) {
            core->branch_taken = true;
            core->branch_target = rd_val & 0x3FF;  // Target address (10 bits)
        }
        
        /* Prepare ID/EX pipeline register */
        core->id_ex.inst = *inst;
        core->id_ex.pc = saved_if_id[i].pc;
        core->id_ex.rs_val = rs_val;
        core->id_ex.rt_val = rt_val;
        core->id_ex.rd_val = rd_val;
        core->id_ex.mem_data = rd_val;  // For stores
        core->id_ex.valid = true;
        core->id_ex.writes_reg = writes_register(inst->opcode);
        core->id_ex.dest_reg = get_dest_reg(inst);
        
        core->if_id.valid = false;
    }
    
    /* ========== FETCH STAGE ========== */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        
        if (core->halted) continue;
        
        if (core->stall_fetch) {
            /* Keep IF/ID as is - don't fetch new instruction */
            continue;
        }
        
        /* Fetch instruction from IMEM */
        uint32_t raw = core->imem[core->pc & 0x3FF];
        
        core->if_id.inst = decode_instruction(raw);
        core->if_id.pc = core->pc;
        core->if_id.valid = true;
        
        /* Increment PC - delay slot instruction will be fetched next */
        core->pc = (core->pc + 1) & 0x3FF;
    }
    
    /* Apply branch targets AFTER fetch (to implement delay slot)
     * The instruction at PC+1 (delay slot) was just fetched above
     * Now we change PC to branch target for the NEXT fetch
     */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        if (core->branch_taken && !core->stall_fetch) {
            core->pc = core->branch_target;
        }
    }
    
    /* Update cycle counts */
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &g_cores[i];
        // Check if core is still active (running or draining pipeline)
        bool active = !core->halted || core->if_id.valid || core->id_ex.valid ||
                      core->ex_mem.valid || core->mem_wb.valid;
        if (active) {
            core->stats.cycles = g_cycle + 1;
        }
    }
    
    g_cycle++;
}


/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(int argc, char *argv[]) {
    /* Default filenames */
    const char *imem_files[NUM_CORES] = {"imem0.txt", "imem1.txt", "imem2.txt", "imem3.txt"};
    const char *memin_file = "memin.txt";
    const char *memout_file = "memout.txt";
    const char *regout_files[NUM_CORES] = {"regout0.txt", "regout1.txt", "regout2.txt", "regout3.txt"};
    const char *trace_files[NUM_CORES] = {"core0trace.txt", "core1trace.txt", "core2trace.txt", "core3trace.txt"};
    const char *bustrace_file = "bustrace.txt";
    const char *dsram_files[NUM_CORES] = {"dsram0.txt", "dsram1.txt", "dsram2.txt", "dsram3.txt"};
    const char *tsram_files[NUM_CORES] = {"tsram0.txt", "tsram1.txt", "tsram2.txt", "tsram3.txt"};
    const char *stats_files[NUM_CORES] = {"stats0.txt", "stats1.txt", "stats2.txt", "stats3.txt"};
    
    /* Parse command line arguments if provided */
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
    
    /* Initialize cores */
    for (int i = 0; i < NUM_CORES; i++) {
        init_core(&g_cores[i], i);
        load_imem(imem_files[i], g_cores[i].imem);
    }
    
    /* Initialize main memory */
    load_main_mem(memin_file);
    
    /* Initialize bus state */
    memset(&g_bus, 0, sizeof(g_bus));
    g_last_granted = NUM_CORES - 1;  /* Start with core 0 having highest priority */
    
    /* Open trace files for writing */
    for (int i = 0; i < NUM_CORES; i++) {
        g_trace_files[i] = fopen(trace_files[i], "w");
    }
    g_bus_trace = fopen(bustrace_file, "w");
    
    /* Run simulation until all cores halt and pipelines drain */
    g_cycle = 0;
    while (!all_cores_done()) {
        simulate_cycle();
        
        /* Safety check to prevent infinite loop */
        if (g_cycle > 100000000) {
            fprintf(stderr, "Simulation exceeded maximum cycles\n");
            break;
        }
    }
    
    /* Close trace files */
    for (int i = 0; i < NUM_CORES; i++) {
        if (g_trace_files[i]) fclose(g_trace_files[i]);
    }
    if (g_bus_trace) fclose(g_bus_trace);
    
    /* Write output files */
    write_main_mem(memout_file);
    
    for (int i = 0; i < NUM_CORES; i++) {
        write_regout(regout_files[i], &g_cores[i]);
        write_dsram(dsram_files[i], &g_cores[i]);
        write_tsram(tsram_files[i], &g_cores[i]);
        write_stats(stats_files[i], &g_cores[i]);
    }
    
    return 0;
}