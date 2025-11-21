#include "procsim.hpp"
#include <deque>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <map>
#include <set>

// global processor states
uint64_t RESULT_BUSES;
uint64_t K0_FU_COUNT;
uint64_t K1_FU_COUNT;
uint64_t K2_FU_COUNT;
uint64_t FETCH_RATE;

// dispatch queue
std::deque<proc_inst_t> dispatch_queue;
// reserved slots (for dispatching to RS)
uint64_t reserved_slots = 0;

// reservation station
std::vector<rs_entry_t> reservation_station;

// result bus structure (matching reference)
struct ResultBus {
    bool busy;
    uint64_t tag;
    int32_t reg;
};
std::vector<ResultBus> result_buses;

// instructions completed and waiting for state update
std::vector<std::pair<uint32_t, rs_entry_t*>> waiting_instructions; // (op_code, entry)

const int32_t NUM_REGISTERS = 128;
// the latest dispatched writer to this register, and whether the register is ready
struct RegisterStatus {
    uint64_t tag;
    bool ready;
};
// register ready table
RegisterStatus register_status[NUM_REGISTERS];

// FU counters: tracks how many FUs are currently in use
uint64_t k0_counter = 0;
uint64_t k1_counter = 0;
uint64_t k2_counter = 0;

// global counters
uint64_t global_tag_counter = 0;
uint64_t current_cycle = 0;

// statistics tracking
uint64_t max_disp_size = 0;
uint64_t total_disp_size = 0;
uint64_t instructions_retired = 0;
bool done_fetching = false;

FILE* logging = fopen("log.txt", "w");
FILE* output = fopen("output.txt", "w");

// Track instruction cycle info for output
struct InstructionCycles {
    uint64_t fetch;
    uint64_t dispatch;
    uint64_t schedule;
    uint64_t execute;
    uint64_t state_update;
};
std::map<uint64_t, InstructionCycles> instruction_cycles;

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f) 
{
    RESULT_BUSES = r;
    K0_FU_COUNT = k0;
    K1_FU_COUNT = k1;
    K2_FU_COUNT = k2;
    FETCH_RATE = f;

    dispatch_queue.clear();
    reservation_station.clear();
    waiting_instructions.clear();

    fprintf(logging, "CYCLE\tOPERATION\tINSTRUCTION\n");

    uint64_t rs_size = 2 * (K0_FU_COUNT + K1_FU_COUNT + K2_FU_COUNT);
    reservation_station.resize(rs_size);
    for (auto& entry : reservation_station) {
        entry.valid = false;
    }

    // Initialize result buses
    result_buses.resize(RESULT_BUSES);
    for (auto& bus : result_buses) {
        bus.busy = false;
        bus.tag = 0;
        bus.reg = -1;
    }

    // Initialize counters to 0 (no FUs in use)
    k0_counter = 0;
    k1_counter = 0;
    k2_counter = 0;

    for (int32_t i = 0; i < NUM_REGISTERS; i++) {
        register_status[i].tag = 0;
        register_status[i].ready = true;
    }

    global_tag_counter = 0;
    current_cycle = 0;
    max_disp_size = 0;
    total_disp_size = 0;
    instructions_retired = 0;
    done_fetching = false;
    instruction_cycles.clear();
}

bool all_rs_empty();

void run_proc(proc_stats_t* p_stats)
{
    while (!done_fetching || !dispatch_queue.empty() || !all_rs_empty()) {
        current_cycle++;

        bool firstHalf = false;

        do {
            firstHalf = !firstHalf;

            state_update_stage(firstHalf);
            execute_stage(firstHalf);
            schedule_stage(firstHalf);
            dispatch_stage(firstHalf);
            fetch_stage(firstHalf);

        } while (firstHalf);
    }

    complete_proc(p_stats);
}

void complete_proc(proc_stats_t *p_stats) 
{
    p_stats->retired_instruction = instructions_retired;
    p_stats->cycle_count = current_cycle;

    if (current_cycle > 0) {
        p_stats->avg_inst_retired = (double)instructions_retired / (double)current_cycle;
    } else {
        p_stats->avg_inst_retired = 0.0;
    }

    p_stats->max_disp_size = max_disp_size;

    if (current_cycle > 0) {
        p_stats->avg_disp_size = (double)total_disp_size / (double)current_cycle;
    } else {
        p_stats->avg_disp_size = 0.0;
    }

    // print the cycle to the output file
    fprintf(output, "INST\tFETCH\tDISP\tSCHED\tEXEC\tSTATE\n");
    for (const auto& pair : instruction_cycles) {
        const auto& cycles = pair.second;
        fprintf(output, "%llu\t%llu\t%llu\t%llu\t%llu\t%llu\n", 
               (unsigned long long)(pair.first + 1),  // 1-indexed like reference
               (unsigned long long)cycles.fetch, 
               (unsigned long long)cycles.dispatch, 
               (unsigned long long)cycles.schedule, 
               (unsigned long long)cycles.execute, 
               (unsigned long long)cycles.state_update);
    }
}

void fetch_stage(bool firstHalf) {
    if (!firstHalf) {
        for (uint64_t i = 0; i < FETCH_RATE; i++) {
            proc_inst_t inst;
            if (read_instruction(&inst)) {
                inst.tag = global_tag_counter++;
                dispatch_queue.push_back(inst);

                fprintf(logging, "%llu\tFETCHED\t%llu\n", current_cycle, inst.tag + 1);

                // record the fetch cycle
                instruction_cycles[inst.tag].fetch = current_cycle;
                instruction_cycles[inst.tag].dispatch = current_cycle + 1;
            } else {
                done_fetching = true;
                break;
            }
        }

        // Track dispatch queue size AFTER fetching (like reference)
        total_disp_size += dispatch_queue.size();
        if (dispatch_queue.size() > max_disp_size) {
            max_disp_size = dispatch_queue.size();
        }
    }
}

void dispatch_stage(bool firstHalf) {
    if (firstHalf) {
        // Reserve slots in RS - minimum of available slots and dispatch queue size
        uint64_t available_slots = 0;
        for (const auto& entry : reservation_station) {
            if (!entry.valid) {
                available_slots++;
            }
        }
        reserved_slots = std::min(available_slots, (uint64_t)dispatch_queue.size());
    } else {
        // Add reserved_slots instructions to RS
        uint64_t dispatched = 0;
        for (size_t i = 0; i < reservation_station.size() && dispatched < reserved_slots && !dispatch_queue.empty(); i++) {
            if (!reservation_station[i].valid) {
                proc_inst_t inst = dispatch_queue.front();
                dispatch_queue.pop_front();

                // create the new RS entry
                rs_entry_t& entry = reservation_station[i];
                entry.valid = true;
                entry.instruction = inst;

                // check whether srcs are ready
                entry.src1_ready = (inst.src_reg[0] == -1) || register_status[inst.src_reg[0]].ready;
                entry.src2_ready = (inst.src_reg[1] == -1) || register_status[inst.src_reg[1]].ready;

                // store parent tags for wakeup
                entry.src1_parent = (inst.src_reg[0] != -1 && !entry.src1_ready) ? register_status[inst.src_reg[0]].tag : 0;
                entry.src2_parent = (inst.src_reg[1] != -1 && !entry.src2_ready) ? register_status[inst.src_reg[1]].tag : 0;

                entry.fired = false;
                entry.completed = false;
                entry.state_updated = false;
                entry.completed_cycle = 0;

                // log the dispatch
                fprintf(logging, "%llu\tDISPATCHED\t%llu\n", current_cycle, inst.tag + 1);

                // record dispatch and set schedule to next cycle (like reference)
                // Note: reference sets schedule = cycle_count + 1 at dispatch time
                instruction_cycles[inst.tag].schedule = current_cycle + 1;

                // mark destination register as not ready
                if (inst.dest_reg != -1) {
                    register_status[inst.dest_reg].ready = false;
                    register_status[inst.dest_reg].tag = inst.tag;
                }

                dispatched++;
            }
        }
        reserved_slots = 0;
    }
}

uint64_t* get_counter(int32_t op_code) {
    int32_t fu_type = (op_code == -1) ? 1 : op_code;
    if (fu_type == 0) return &k0_counter;
    if (fu_type == 1) return &k1_counter;
    if (fu_type == 2) return &k2_counter;
    return nullptr;
}

uint64_t get_fu_count(int32_t op_code) {
    int32_t fu_type = (op_code == -1) ? 1 : op_code;
    if (fu_type == 0) return K0_FU_COUNT;
    if (fu_type == 1) return K1_FU_COUNT;
    if (fu_type == 2) return K2_FU_COUNT;
    return 0;
}

void schedule_stage(bool firstHalf) {
    if (firstHalf) {
        // Try to fire instructions in RS (in tag order - map maintains order)
        std::map<uint64_t, rs_entry_t*> ready_instructions;
        for (auto& entry : reservation_station) {
            if (!entry.valid || entry.fired) {
                continue;
            }
            if (entry.src1_ready && entry.src2_ready) {
                ready_instructions[entry.instruction.tag] = &entry;
            }
        }

        for (auto& pair : ready_instructions) {
            rs_entry_t* entry = pair.second;
            uint64_t* counter = get_counter(entry->instruction.op_code);
            uint64_t fu_count = get_fu_count(entry->instruction.op_code);

            // Check if there's a free FU slot
            if (*counter < fu_count) {
                // Reserve the FU
                (*counter)++;
                entry->fired = true;

                fprintf(logging, "%llu\tSCHEDULED\t%llu\n", current_cycle, entry->instruction.tag + 1);
                // Reference sets execute cycle to current + 1 when scheduled
                instruction_cycles[entry->instruction.tag].execute = current_cycle + 1;
            }
        }
    } else {
        // Wakeup instructions from CDB broadcast
        for (auto& entry : reservation_station) {
            if (!entry.valid || entry.fired) {
                continue;
            }
            for (const auto& bus : result_buses) {
                if (bus.busy) {
                    if (!entry.src1_ready && entry.src1_parent == bus.tag) {
                        entry.src1_ready = true;
                    }
                    if (!entry.src2_ready && entry.src2_parent == bus.tag) {
                        entry.src2_ready = true;
                    }
                }
            }
        }
    }
}

void execute_stage(bool firstHalf) {
    if (firstHalf) {
        // Collect executed instructions (in tag order)
        std::map<uint64_t, std::pair<uint32_t, rs_entry_t*>> executed_this_cycle;

        for (auto& entry : reservation_station) {
            if (entry.valid && entry.fired && !entry.completed) {
                entry.completed = true;
                entry.completed_cycle = current_cycle;

                fprintf(logging, "%llu\tEXECUTED\t%llu\n", current_cycle, entry.instruction.tag + 1);

                int32_t actual_op = (entry.instruction.op_code == -1) ? 1 : entry.instruction.op_code;
                executed_this_cycle[entry.instruction.tag] = std::make_pair(actual_op, &entry);
            }
        }

        // Add to waiting instructions queue
        for (auto& pair : executed_this_cycle) {
            waiting_instructions.push_back(pair.second);
        }

        // Broadcast on result buses (oldest first)
        auto w = waiting_instructions.begin();
        for (auto& bus : result_buses) {
            if (w == waiting_instructions.end()) break;

            uint32_t op_code = w->first;
            rs_entry_t* entry = w->second;

            bus.busy = true;
            bus.tag = entry->instruction.tag;
            bus.reg = entry->instruction.dest_reg;

            // Update register file if this is still the latest writer
            if (bus.reg >= 0 && register_status[bus.reg].tag == bus.tag) {
                register_status[bus.reg].ready = true;
            }

            // Free the FU
            uint64_t* counter;
            if (op_code == 0) counter = &k0_counter;
            else if (op_code == 1) counter = &k1_counter;
            else counter = &k2_counter;

            if (*counter > 0) {
                (*counter)--;
            }

            entry->state_updated = true;
            entry->state_update_cycle = current_cycle;

            w = waiting_instructions.erase(w);
        }

        // Clear buses that weren't used this cycle
        // (Actually, reference doesn't clear - buses stay busy until overwritten)
    }
    // No second half actions
}

void state_update_stage(bool firstHalf) {
    if (!firstHalf) {
        // Remove completed instructions from RS (completed in previous cycle)
        for (auto& entry : reservation_station) {
            if (entry.valid && entry.state_updated && entry.state_update_cycle < current_cycle) {
                fprintf(logging, "%llu\tSTATE UPDATE\t%llu\n", current_cycle, entry.instruction.tag + 1);
                instruction_cycles[entry.instruction.tag].state_update = current_cycle;

                entry.valid = false;
                entry.state_updated = false;
                instructions_retired++;
            }
        }
    }
    // No first half actions
}

// Wrapper functions for compatibility
void state_update_stage_first_half() { state_update_stage(true); }
void state_update_stage_second_half() { state_update_stage(false); }
void execute_stage_first_half() { execute_stage(true); }
void execute_stage_second_half() { execute_stage(false); }
void schedule_stage_first_half() { schedule_stage(true); }
void schedule_stage_second_half() { schedule_stage(false); }
void dispatch_stage_first_half() { dispatch_stage(true); }
void dispatch_stage_second_half() { dispatch_stage(false); }

void fetch_stage() {
    fetch_stage(false);  // Original only had second half
}

bool all_rs_empty() 
{
    for (const auto& entry : reservation_station) {
        if (entry.valid) return false;
    }
    return true;
}