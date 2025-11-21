#ifndef PROCSIM_HPP
#define PROCSIM_HPP

#include <cstdint>
#include <cstdio>
#include <vector>

#define DEFAULT_K0 1
#define DEFAULT_K1 2
#define DEFAULT_K2 3
#define DEFAULT_R 8
#define DEFAULT_F 4

typedef struct _proc_inst_t
{
    uint32_t instruction_address;
    int32_t op_code;
    int32_t src_reg[2];
    int32_t dest_reg;
    uint64_t tag;
} proc_inst_t;

typedef struct _proc_stats_t
{
    float avg_inst_retired;
    float avg_inst_fired;
    float avg_disp_size;
    unsigned long max_disp_size;
    unsigned long retired_instruction;
    unsigned long cycle_count;
} proc_stats_t;

typedef struct _rs_entry_t {
    bool valid;
    proc_inst_t instruction;
    bool src1_ready;
    bool src2_ready;
    uint64_t src1_parent;
    uint64_t src2_parent;
    bool fired;
    bool completed;
    bool state_updated;
    uint64_t execute_cycles_left;
    uint64_t completed_cycle;
    bool tag_dispatched;
    
    // Cycle tracking for output
    uint64_t fetch_cycle;
    uint64_t dispatch_cycle;
    uint64_t schedule_cycle;
    uint64_t execute_cycle;
    uint64_t state_update_cycle;
    uint64_t fired_cycle;
} rs_entry_t;

bool read_instruction(proc_inst_t* p_inst);

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f);
void run_proc(proc_stats_t* p_stats);
void complete_proc(proc_stats_t *p_stats);

// Stage functions
void fetch_stage();
void dispatch_stage_first_half();
void dispatch_stage_second_half();
void schedule_stage_first_half();
void schedule_stage_second_half();
void execute_stage_first_half();
void execute_stage_second_half();
void state_update_stage_first_half();
void state_update_stage_second_half();

// Utility functions
bool all_rs_empty();

#endif