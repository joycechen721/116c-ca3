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
    int32_t dest_reg;
    int32_t src_reg[2];
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

typedef struct dispatch_entry_t {
    proc_inst_t instruction;
    bool dispatched;
} dispatch_entry_t;

typedef struct rs_entry_t {
    bool valid;
    proc_inst_t instruction;
    bool ready_to_fire;
    bool fired;
    bool completed;
    uint64_t execute_cycles_left;
} rs_entry_t;

bool read_instruction(proc_inst_t* p_inst);

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f);
void run_proc(proc_stats_t* p_stats);
void complete_proc(proc_stats_t* p_stats);

void fetch_stage();
void dispatch_stage();
void schedule_stage();
void execute_stage();
void state_update_stage();

bool all_rs_empty();

#endif /* PROCSIM_HPP */
