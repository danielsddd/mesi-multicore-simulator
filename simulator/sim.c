/*
 * Multi-Core MESI Cache Coherency Simulator
 * Tel-Aviv University - Computer Architecture Course
 * FIXED VERSION - Solves Pipeline Stalls & Bus Timing
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// ========================= CONSTANTS =========================
#define NUM_CORES 4
#define NUM_REGISTERS 16
#define IMEM_SIZE 1024
#define MAIN_MEM_SIZE (1 << 21)
#define CACHE_SIZE 512
#define CACHE_BLOCK_SIZE 8
#define TSRAM_SIZE 64
#define MEM_READ_LATENCY 16
#define MAX_CYCLES 1000000

// MESI states
#define MESI_INVALID 0
#define MESI_SHARED 1
#define MESI_EXCLUSIVE 2
#define MESI_MODIFIED 3

// Bus commands
#define BUS_NO_CMD 0
#define BUS_RD 1
#define BUS_RDX 2
#define BUS_FLUSH 3

// Originator IDs
#define ORIG_MAIN_MEM 4

// Opcodes
#define OP_ADD 0
#define OP_SUB 1
#define OP_AND 2
#define OP_OR 3
#define OP_XOR 4
#define OP_MUL 5
#define OP_SLL 6
#define OP_SRA 7
#define OP_SRL 8
#define OP_BEQ 9
#define OP_BNE 10
#define OP_BLT 11
#define OP_BGT 12
#define OP_BLE 13
#define OP_BGE 14
#define OP_JAL 15
#define OP_LW 16
#define OP_SW 17
#define OP_HALT 20

// ========================= DATA STRUCTURES =========================

typedef struct {
    uint8_t opcode;
    uint8_t rd;
    uint8_t rs;
    uint8_t rt;
    int32_t immediate;
} Instruction;

typedef struct {
    bool valid;
    uint32_t pc;
    Instruction inst;
    int32_t alu_result;
    int32_t mem_data;
    int32_t rs_val;
    int32_t rt_val;
    int32_t rd_val;
    uint32_t return_addr;
} PipelineStage;

typedef struct {
    uint8_t mesi_state;
    uint32_t tag;
    bool dirty;
} CacheLine;

typedef struct {
    uint8_t origid;
    uint8_t cmd;
    uint32_t addr;
    uint32_t data;
    uint8_t shared;
    bool busy;
} Bus;

typedef struct {
    // Core state
    int32_t registers[NUM_REGISTERS];
    uint32_t pc;
    uint32_t next_pc;
    uint32_t imem[IMEM_SIZE];
    bool halted;
    bool branch_taken;
    
    // Pipeline stages
    PipelineStage fetch_stage;
    PipelineStage decode_stage;
    PipelineStage execute_stage;
    PipelineStage mem_stage;
    PipelineStage wb_stage;
    
    // Cache
    uint32_t dsram[CACHE_SIZE];
    CacheLine tsram[TSRAM_SIZE];
    
    // Stall handling
    bool stall_fetch;
    bool stall_decode;
    bool stall_mem;
    
    // Cache miss handling
    bool waiting_for_bus;
    uint32_t miss_addr;
    bool miss_is_write;
    uint32_t miss_write_data;
    bool receiving_block;
    int block_words_received;
    uint32_t block_base_addr;
    bool miss_shared_status;
    
    // Flush handling
    bool flushing_block;
    int flush_words_left;
    uint32_t flush_addr;
    uint32_t flush_block_base;
    
    // Statistics
    int cycles;
    int instructions_executed;
    int read_hits;
    int write_hits;
    int read_misses;
    int write_misses;
    int decode_stalls;
    int mem_stalls;
} Core;

typedef struct {
    Core cores[NUM_CORES];
    uint32_t main_memory[MAIN_MEM_SIZE];
    Bus bus;
    int global_cycle;
    int last_bus_user;
    
    // Memory response handling
    bool mem_responding;
    int mem_response_cycles_left;
    uint32_t mem_response_addr;
    int mem_response_words_left;
    int mem_response_dest;
    bool bus_locked;
} SimulatorState;

// ========================= GLOBALS =========================
SimulatorState sim;
FILE *trace_files[NUM_CORES];
FILE *bus_trace_file;

// ========================= HELPER FUNCTIONS =========================

int32_t sign_extend_12(uint32_t imm) {
    return (imm & 0x800) ? (int32_t)(imm | 0xFFFFF000) : (int32_t)imm;
}

Instruction decode_instruction(uint32_t word) {
    Instruction inst;
    inst.opcode = (word >> 24) & 0xFF;
    inst.rd = (word >> 20) & 0xF;
    inst.rs = (word >> 16) & 0xF;
    inst.rt = (word >> 12) & 0xF;
    inst.immediate = sign_extend_12(word & 0xFFF);
    return inst;
}

int get_block_index(uint32_t addr) {
    return (addr >> 3) & 0x3F;
}

uint32_t get_tag(uint32_t addr) {
    return (addr >> 9) & 0xFFF;
}

int get_offset(uint32_t addr) {
    return addr & 0x7;
}

uint32_t get_block_base(uint32_t addr) {
    return addr & 0xFFFFFFF8;
}

void reset_bus() {
    sim.bus.origid = 0;
    sim.bus.cmd = BUS_NO_CMD;
    sim.bus.addr = 0;
    sim.bus.data = 0;
    sim.bus.shared = 0;
    sim.bus.busy = false;
}

void write_bus_trace() {
    if (sim.bus.cmd != BUS_NO_CMD) {
        fprintf(bus_trace_file, "%d %X %X %06X %08X %X\n",
                sim.global_cycle,
                sim.bus.origid,
                sim.bus.cmd,
                sim.bus.addr,
                sim.bus.data,
                sim.bus.shared);
    }
}

// ========================= BUS ARBITRATION =========================

bool request_bus_access(int core_id) {
    if (sim.bus.busy) return false;
    
    Core *requesting_core = &sim.cores[core_id];
    
    // Respect Bus Lock (only allow FLUSH from current transaction owner/responder)
    if (sim.bus_locked && !requesting_core->flushing_block) {
        return false;
    }
    
    if (requesting_core->flushing_block || requesting_core->waiting_for_bus) {
        // Round Robin
        for (int i = 1; i <= NUM_CORES; i++) {
            int check_core = (sim.last_bus_user + i) % NUM_CORES;
            Core *c = &sim.cores[check_core];
            
            if (c->flushing_block || c->waiting_for_bus) {
                if (check_core == core_id) {
                    sim.last_bus_user = check_core;
                    return true;
                }
                return false;
            }
        }
    }
    return false;
}

// ========================= CACHE OPERATIONS =========================

bool cache_lookup(Core *core, uint32_t addr, uint32_t *data) {
    int block_idx = get_block_index(addr);
    uint32_t tag = get_tag(addr);
    CacheLine *line = &core->tsram[block_idx];
    
    if (line->mesi_state != MESI_INVALID && line->tag == tag) {
        int offset = get_offset(addr);
        int dsram_idx = (block_idx * CACHE_BLOCK_SIZE) + offset;
        *data = core->dsram[dsram_idx];
        return true;
    }
    return false;
}

void cache_write_word(Core *core, uint32_t addr, uint32_t data) {
    int block_idx = get_block_index(addr);
    int offset = get_offset(addr);
    int dsram_idx = (block_idx * CACHE_BLOCK_SIZE) + offset;
    
    core->dsram[dsram_idx] = data;
    
    uint32_t tag = get_tag(addr);
    CacheLine *line = &core->tsram[block_idx];
    if (line->tag == tag) {
        line->mesi_state = MESI_MODIFIED;
        line->dirty = true;
    }
}

void cache_install_block(Core *core, uint32_t base_addr, bool shared) {
    int block_idx = get_block_index(base_addr);
    uint32_t tag = get_tag(base_addr);
    CacheLine *line = &core->tsram[block_idx];
    
    line->tag = tag;
    line->mesi_state = shared ? MESI_SHARED : MESI_EXCLUSIVE;
    line->dirty = false;
}

// ========================= SNOOPING =========================

void snoop_bus(int core_id) {
    Core *core = &sim.cores[core_id];
    
    if (sim.bus.cmd == BUS_NO_CMD) return;
    
    // 1. Snarfing (Data Receive)
    if (sim.bus.cmd == BUS_FLUSH) {
        if (core->receiving_block) {
            if (get_block_base(sim.bus.addr) == core->block_base_addr) {
                int block_idx = get_block_index(sim.bus.addr);
                int offset = get_offset(sim.bus.addr);
                int dsram_idx = (block_idx * CACHE_BLOCK_SIZE) + offset;
                
                core->dsram[dsram_idx] = sim.bus.data;
                core->block_words_received++;
                
                if (core->block_words_received == CACHE_BLOCK_SIZE) {
                    cache_install_block(core, core->block_base_addr, core->miss_shared_status);
                    
                    core->receiving_block = false;
                    core->stall_mem = false;
                    core->waiting_for_bus = false;
                    
                    if (core->miss_is_write) {
                        cache_write_word(core, core->miss_addr, core->miss_write_data);
                    }
                    sim.bus_locked = false;
                }
            }
        }
        return;
    }
    
    // 2. Ignore self
    if (sim.bus.origid == core_id) return;
    
    // 3. MESI Logic
    uint32_t bus_addr = sim.bus.addr;
    int block_idx = get_block_index(bus_addr);
    uint32_t tag = get_tag(bus_addr);
    CacheLine *line = &core->tsram[block_idx];
    
    if (line->mesi_state != MESI_INVALID && line->tag == tag) {
        sim.bus.shared = 1;
        
        if (sim.bus.cmd == BUS_RD) {
            if (line->mesi_state == MESI_MODIFIED || line->mesi_state == MESI_EXCLUSIVE) {
                line->mesi_state = MESI_SHARED;
                
                if (line->dirty) {
                    line->dirty = false;
                    core->flushing_block = true;
                    core->flush_words_left = CACHE_BLOCK_SIZE;
                    core->flush_addr = (tag << 9) | (block_idx << 3);
                    core->flush_block_base = core->flush_addr;
                    
                    sim.mem_responding = false; // Cancel memory, cache will provide
                }
            }
        } 
        else if (sim.bus.cmd == BUS_RDX) {
            line->mesi_state = MESI_INVALID;
            
            if (line->dirty) {
                core->flushing_block = true;
                core->flush_words_left = CACHE_BLOCK_SIZE;
                core->flush_addr = (tag << 9) | (block_idx << 3);
                core->flush_block_base = core->flush_addr;
                
                sim.mem_responding = false;
            }
        }
    }
}

// ========================= CACHE MISS & MEM RESPONSE =========================

void handle_cache_miss(Core *core, int core_id) {
    if (core->flushing_block) {
        if (request_bus_access(core_id)) {
            sim.bus.busy = true;
            sim.bus.origid = core_id;
            sim.bus.cmd = BUS_FLUSH;
            sim.bus.addr = core->flush_addr;
            
            int block_idx = get_block_index(core->flush_addr);
            int offset = CACHE_BLOCK_SIZE - core->flush_words_left;
            int dsram_idx = (block_idx * CACHE_BLOCK_SIZE) + offset;
            sim.bus.data = core->dsram[dsram_idx];
            
            sim.main_memory[core->flush_addr] = sim.bus.data;
            
            core->flush_words_left--;
            core->flush_addr++;
            
            if (core->flush_words_left == 0) {
                core->flushing_block = false;
            }
        }
        return;
    }
    
    if (core->waiting_for_bus) {
        if (request_bus_access(core_id)) {
            sim.bus.busy = true;
            sim.bus_locked = true;
            sim.bus.origid = core_id;
            sim.bus.cmd = core->miss_is_write ? BUS_RDX : BUS_RD;
            sim.bus.addr = core->miss_addr;
            sim.bus.shared = 0;
            sim.bus.data = 0;
            
            sim.mem_responding = true;
            sim.mem_response_cycles_left = MEM_READ_LATENCY;
            sim.mem_response_addr = get_block_base(core->miss_addr);
            sim.mem_response_words_left = CACHE_BLOCK_SIZE;
            sim.mem_response_dest = core_id;
            
            core->waiting_for_bus = false;
            core->receiving_block = true;
            core->block_base_addr = get_block_base(core->miss_addr);
            core->block_words_received = 0;
            core->miss_shared_status = false;
        }
    }
}

void handle_memory_response() {
    if (!sim.mem_responding) return;
    
    if (sim.mem_response_cycles_left > 0) {
        sim.mem_response_cycles_left--;
        return;
    }
    
    if (!sim.bus.busy) {
        sim.bus.busy = true;
        sim.bus.origid = ORIG_MAIN_MEM;
        sim.bus.cmd = BUS_FLUSH;
        sim.bus.addr = sim.mem_response_addr;
        sim.bus.data = sim.main_memory[sim.mem_response_addr];
        
        sim.mem_response_addr++;
        sim.mem_response_words_left--;
        
        if (sim.mem_response_words_left == 0) {
            sim.mem_responding = false;
        }
    }
}

// ========================= HAZARD DETECTION =========================

void detect_hazards(Core *core) {
    core->stall_decode = false;
    core->stall_fetch = false;
    
    if (!core->decode_stage.valid) return;
    
    Instruction *dec = &core->decode_stage.inst;
    
    // Check RAW hazards with Exec, Mem, WB
    // Note: Checks even WB because write happens at END of cycle
    for (int i = 0; i < 3; i++) {
        PipelineStage *older_stage;
        switch (i) {
            case 0: older_stage = &core->execute_stage; break;
            case 1: older_stage = &core->mem_stage; break;
            case 2: older_stage = &core->wb_stage; break;
            default: continue;
        }
        
        if (older_stage->valid) {
            Instruction *older = &older_stage->inst;
            bool writes_reg = false;
            int dest_reg = -1;
            
            if (older->opcode <= OP_SRL || older->opcode == OP_LW) {
                writes_reg = (older->rd != 0 && older->rd != 1);
                dest_reg = older->rd;
            } else if (older->opcode == OP_JAL) {
                writes_reg = true;
                dest_reg = 15;
            }
            
            if (writes_reg) {
                if ((dec->rs == dest_reg && dec->rs != 1) || 
                    (dec->rt == dest_reg && dec->rt != 1) ||
                    (dec->opcode == OP_SW && dec->rd == dest_reg && dec->rd != 1)) {
                    
                    core->stall_decode = true;
                    core->stall_fetch = true;
                    core->decode_stalls++;
                    return;
                }
            }
        }
    }
}

// ========================= PIPELINE STAGES =========================

void stage_fetch(Core *core) {
    // 1. Backpressure Check: Is the stage already full?
    if (core->fetch_stage.valid) return;
    
    // 2. Hazard Stall Check
    if (core->halted || core->stall_fetch) return;
    
    if (core->branch_taken) {
        core->pc = core->next_pc;
        core->branch_taken = false;
    }
    
    if (core->pc >= IMEM_SIZE) {
        core->fetch_stage.valid = false;
        return;
    }
    
    uint32_t instr_word = core->imem[core->pc];
    if (instr_word == 0xFFFFFFFF) { // Skip empty/NOP
        core->pc++;
        core->fetch_stage.valid = false;
        return;
    }
    
    core->fetch_stage.valid = true;
    core->fetch_stage.pc = core->pc;
    core->pc++;
}

void stage_decode(Core *core) {
    // 1. Backpressure Check
    if (core->decode_stage.valid) return;
    
    // 2. Hazard Stall: If stalled, don't pull new instruction
    // (Wait, hazard means we can't send TO Execute, so Decode stays valid. 
    // The Backpressure check above handles this!)
    
    if (!core->fetch_stage.valid) return;
    
    // Move Fetch -> Decode
    core->decode_stage = core->fetch_stage;
    core->fetch_stage.valid = false;
    
    // Decoding Logic
    uint32_t word = core->imem[core->decode_stage.pc];
    core->decode_stage.inst = decode_instruction(word);
    Instruction *inst = &core->decode_stage.inst;
    
    core->registers[1] = inst->immediate;
    core->decode_stage.rs_val = core->registers[inst->rs];
    core->decode_stage.rt_val = core->registers[inst->rt];
    if (inst->opcode == OP_SW) core->decode_stage.rd_val = core->registers[inst->rd];
    
    // Branch Resolution
    bool take = false;
    int32_t rs = core->decode_stage.rs_val;
    int32_t rt = core->decode_stage.rt_val;
    switch (inst->opcode) {
        case OP_BEQ: take = (rs == rt); break;
        case OP_BNE: take = (rs != rt); break;
        case OP_BLT: take = (rs < rt); break;
        case OP_BGT: take = (rs > rt); break;
        case OP_BLE: take = (rs <= rt); break;
        case OP_BGE: take = (rs >= rt); break;
        case OP_JAL: take = true; break;
    }
    
    if (take) {
        if (inst->opcode == OP_JAL) core->decode_stage.return_addr = core->decode_stage.pc + 1;
        core->next_pc = core->registers[inst->rd] & 0x3FF;
        core->branch_taken = true;
    }
}

void stage_execute(Core *core) {
    // 1. Backpressure Check
    if (core->execute_stage.valid) return;
    
    // 2. Hazard Stall Check: Inject Bubble if Decode is stalled
    if (core->stall_decode) {
        core->execute_stage.valid = false; // Bubble
        return;
    }
    
    if (!core->decode_stage.valid) {
        core->execute_stage.valid = false;
        return;
    }
    
    // Move Decode -> Execute
    core->execute_stage = core->decode_stage;
    core->decode_stage.valid = false;
    
    // ALU Logic
    Instruction *inst = &core->execute_stage.inst;
    int32_t rs = core->execute_stage.rs_val;
    int32_t rt = core->execute_stage.rt_val;
    int32_t res = 0;
    
    switch (inst->opcode) {
        case OP_ADD: res = rs + rt; break;
        case OP_SUB: res = rs - rt; break;
        case OP_AND: res = rs & rt; break;
        case OP_OR:  res = rs | rt; break;
        case OP_XOR: res = rs ^ rt; break;
        case OP_MUL: res = rs * rt; break;
        case OP_SLL: res = rs << (rt & 0x1F); break;
        case OP_SRA: res = rs >> (rt & 0x1F); break;
        case OP_SRL: res = ((uint32_t)rs) >> (rt & 0x1F); break;
        case OP_LW: case OP_SW: res = rs + rt; break;
    }
    core->execute_stage.alu_result = res;
}

void stage_mem(Core *core, int core_id) {
    // 1. Stall Check (Cache Miss)
    if (core->stall_mem) {
        core->mem_stalls++;
        return; // Return without clearing execute_stage (Backpressure)
    }
    
    if (!core->execute_stage.valid) {
        core->mem_stage.valid = false;
        return;
    }
    
    // Move Execute -> Mem
    core->mem_stage = core->execute_stage;
    core->execute_stage.valid = false;
    
    Instruction *inst = &core->mem_stage.inst;
    if (inst->opcode == OP_LW) {
        uint32_t addr = core->mem_stage.alu_result & 0x1FFFFF;
        uint32_t data;
        if (cache_lookup(core, addr, &data)) {
            core->mem_stage.mem_data = data;
            core->read_hits++;
        } else {
            core->read_misses++;
            core->stall_mem = true;
            core->waiting_for_bus = true;
            core->miss_addr = addr;
            core->miss_is_write = false;
        }
    } else if (inst->opcode == OP_SW) {
        uint32_t addr = core->mem_stage.alu_result & 0x1FFFFF;
        uint32_t data = core->registers[inst->rd];
        int b_idx = get_block_index(addr);
        uint32_t tag = get_tag(addr);
        CacheLine *line = &core->tsram[b_idx];
        
        if (line->mesi_state != MESI_INVALID && line->tag == tag && line->mesi_state != MESI_SHARED) {
            cache_write_word(core, addr, data);
            core->write_hits++;
        } else {
            core->write_misses++;
            core->stall_mem = true;
            core->waiting_for_bus = true;
            core->miss_addr = addr;
            core->miss_is_write = true;
            core->miss_write_data = data;
        }
    } else if (inst->opcode == OP_HALT) {
        core->halted = true;
    }
}

void stage_writeback(Core *core) {
    // Writeback always runs (no downstream stall)
    if (!core->mem_stage.valid) {
        core->wb_stage.valid = false;
        return;
    }
    
    core->wb_stage = core->mem_stage;
    core->mem_stage.valid = false;
    
    Instruction *inst = &core->wb_stage.inst;
    core->instructions_executed++;
    
    if (inst->rd != 0 && inst->rd != 1) {
        if (inst->opcode >= OP_ADD && inst->opcode <= OP_SRL) 
            core->registers[inst->rd] = core->wb_stage.alu_result;
        else if (inst->opcode == OP_LW) 
            core->registers[inst->rd] = core->wb_stage.mem_data;
    }
    if (inst->opcode == OP_JAL) core->registers[15] = core->wb_stage.return_addr;
}

// ========================= SIMULATION LOOP =========================

void write_core_trace(int core_id) {
    Core *core = &sim.cores[core_id];
    bool active = core->fetch_stage.valid || core->decode_stage.valid || 
                  core->execute_stage.valid || core->mem_stage.valid || core->wb_stage.valid;
    if (!active && core->halted) return;

    fprintf(trace_files[core_id], "%d ", sim.global_cycle);
    
    if (core->fetch_stage.valid) fprintf(trace_files[core_id], "%03X ", core->fetch_stage.pc); else fprintf(trace_files[core_id], "--- ");
    if (core->decode_stage.valid) fprintf(trace_files[core_id], "%03X ", core->decode_stage.pc); else fprintf(trace_files[core_id], "--- ");
    if (core->execute_stage.valid) fprintf(trace_files[core_id], "%03X ", core->execute_stage.pc); else fprintf(trace_files[core_id], "--- ");
    if (core->mem_stage.valid) fprintf(trace_files[core_id], "%03X ", core->mem_stage.pc); else fprintf(trace_files[core_id], "--- ");
    if (core->wb_stage.valid) fprintf(trace_files[core_id], "%03X ", core->wb_stage.pc); else fprintf(trace_files[core_id], "--- ");

    for (int i = 2; i < NUM_REGISTERS; i++) {
        fprintf(trace_files[core_id], "%08X", core->registers[i]);
        if (i < NUM_REGISTERS - 1) fprintf(trace_files[core_id], " ");
    }
    fprintf(trace_files[core_id], "\n");
}

void simulate_cycle() {
    sim.global_cycle++;
    
    // 1. Reset Bus (Start of new cycle)
    reset_bus();
    
    // 2. Trace State (Start of cycle)
    for (int i = 0; i < NUM_CORES; i++) write_core_trace(i);
    
    // 3. Drive Bus (Arbitration)
    handle_memory_response();
    if (sim.bus.cmd == BUS_NO_CMD) {
        for (int i = 0; i < NUM_CORES; i++) handle_cache_miss(&sim.cores[i], i);
    }
    
    // 4. Trace Bus (Driven state)
    write_bus_trace();
    
    // 5. Snoop (Cores see driven bus)
    for (int i = 0; i < NUM_CORES; i++) snoop_bus(i);
    
    // 6. Shared Signal Latch
    if (sim.bus.cmd == BUS_RD && sim.bus.origid < NUM_CORES) {
        sim.cores[sim.bus.origid].miss_shared_status = (sim.bus.shared == 1);
    }
    
    // 7. Pipeline (Reverse Order)
    for (int i = 0; i < NUM_CORES; i++) {
        Core *core = &sim.cores[i];
        if (core->halted && !core->fetch_stage.valid && !core->decode_stage.valid && 
            !core->execute_stage.valid && !core->mem_stage.valid && !core->wb_stage.valid) continue;
            
        core->cycles++;
        detect_hazards(core);
        stage_writeback(core);
        stage_mem(core, i);
        stage_execute(core);
        stage_decode(core);
        stage_fetch(core);
    }
}

bool all_halted() {
    for (int i = 0; i < NUM_CORES; i++) {
        Core *c = &sim.cores[i];
        if (!c->halted) return false;
        if (c->fetch_stage.valid || c->decode_stage.valid || 
            c->execute_stage.valid || c->mem_stage.valid || c->wb_stage.valid) return false;
        if (c->waiting_for_bus || c->receiving_block || c->flushing_block) return false;
    }
    return true;
}

// ========================= FILE I/O =========================

void load_mem(const char *file, uint32_t *mem, int size) {
    FILE *f = fopen(file, "r");
    if (!f) return;
    char line[16];
    int addr = 0;
    while (fgets(line, sizeof(line), f) && addr < size) {
        sscanf(line, "%x", &mem[addr]);
        addr++;
    }
    fclose(f);
}

void save_mem(const char *file, uint32_t *mem, int size) {
    FILE *f = fopen(file, "w");
    if (!f) return;
    for (int i = 0; i < size; i++) fprintf(f, "%08X\n", mem[i]);
    fclose(f);
}

void save_regs(const char *file, Core *core) {
    FILE *f = fopen(file, "w");
    if (!f) return;
    for (int i = 2; i < NUM_REGISTERS; i++) fprintf(f, "%08X\n", core->registers[i]);
    fclose(f);
}

void save_stats(const char *file, Core *core) {
    FILE *f = fopen(file, "w");
    if (!f) return;
    fprintf(f, "cycles %d\n", core->cycles);
    fprintf(f, "instructions %d\n", core->instructions_executed);
    fprintf(f, "read_hit %d\n", core->read_hits);
    fprintf(f, "write_hit %d\n", core->write_hits);
    fprintf(f, "read_miss %d\n", core->read_misses);
    fprintf(f, "write_miss %d\n", core->write_misses);
    fprintf(f, "decode_stall %d\n", core->decode_stalls);
    fprintf(f, "mem_stall %d\n", core->mem_stalls);
    fclose(f);
}

void save_tsram(const char *file, Core *core) {
    FILE *f = fopen(file, "w");
    if (!f) return;
    for (int i = 0; i < TSRAM_SIZE; i++) {
        uint32_t val = (core->tsram[i].mesi_state << 12) | core->tsram[i].tag;
        fprintf(f, "%08X\n", val);
    }
    fclose(f);
}

void initialize_simulator() {
    memset(&sim, 0, sizeof(sim));
    sim.last_bus_user = NUM_CORES - 1;
}

int main(int argc, char *argv[]) {
    char *files[27];
    char *defaults[] = {
        "imem0.txt", "imem1.txt", "imem2.txt", "imem3.txt",
        "memin.txt", "memout.txt",
        "regout0.txt", "regout1.txt", "regout2.txt", "regout3.txt",
        "core0trace.txt", "core1trace.txt", "core2trace.txt", "core3trace.txt",
        "bustrace.txt",
        "dsram0.txt", "dsram1.txt", "dsram2.txt", "dsram3.txt",
        "tsram0.txt", "tsram1.txt", "tsram2.txt", "tsram3.txt",
        "stats0.txt", "stats1.txt", "stats2.txt", "stats3.txt"
    };
    
    if (argc == 28) {
        for (int i = 0; i < 27; i++) files[i] = argv[i + 1];
    } else {
        for (int i = 0; i < 27; i++) files[i] = defaults[i];
    }
    
    initialize_simulator();
    for (int i = 0; i < NUM_CORES; i++) load_mem(files[i], sim.cores[i].imem, IMEM_SIZE);
    load_mem(files[4], sim.main_memory, MAIN_MEM_SIZE);
    for (int i = 0; i < NUM_CORES; i++) trace_files[i] = fopen(files[10 + i], "w");
    bus_trace_file = fopen(files[14], "w");
    
    printf("Starting simulation...\n");
    while (!all_halted() && sim.global_cycle < MAX_CYCLES) simulate_cycle();
    
    for (int i = 0; i < NUM_CORES; i++) fclose(trace_files[i]);
    fclose(bus_trace_file);
    save_mem(files[5], sim.main_memory, MAIN_MEM_SIZE);
    for (int i = 0; i < NUM_CORES; i++) {
        save_regs(files[6 + i], &sim.cores[i]);
        save_mem(files[15 + i], sim.cores[i].dsram, CACHE_SIZE);
        save_tsram(files[19 + i], &sim.cores[i]);
        save_stats(files[23 + i], &sim.cores[i]);
    }
    return 0;
}