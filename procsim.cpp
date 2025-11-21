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

// result bus: tags of instructions that broadcast this cycle
std::vector<uint64_t> result_bus_tags;
// instructions completed and waiting for state update
std::vector<rs_entry_t*> completed_instructions;

const int32_t NUM_REGISTERS = 128;
// the latest dispatched writer to this register, and whether the register is ready
struct RegisterStatus {
    uint64_t tag;
    bool ready;
};
// register ready table
RegisterStatus register_status[NUM_REGISTERS];

// FU ready counts
uint64_t k0_fu_available;
uint64_t k1_fu_available;
uint64_t k2_fu_available;

// global counters
uint64_t global_tag_counter = 1;
uint64_t current_cycle = 0;

// statistics tracking
uint64_t max_disp_size = 0;
uint64_t total_disp_size = 0;
uint64_t disp_size_count = 0;
uint64_t instructions_retired = 0;
uint64_t first_instruction_cycle = 0;

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

  fprintf(logging, "CYCLE\tOPERATION\tINSTRUCTION\n");

  uint64_t rs_size = 2 * (K0_FU_COUNT + K1_FU_COUNT + K2_FU_COUNT);
  reservation_station.resize(rs_size);
  for (auto& entry : reservation_station) {
      entry.valid = false;
  }

  for (int32_t i = 0; i < NUM_REGISTERS; i++) {
        register_status[i].tag = 0;
        register_status[i].ready = true;
  }

  k0_fu_available = K0_FU_COUNT;
  k1_fu_available = K1_FU_COUNT;
  k2_fu_available = K2_FU_COUNT;

  global_tag_counter = 1;
  current_cycle = 0;
  max_disp_size = 0;
  total_disp_size = 0;
  disp_size_count = 0;
  instructions_retired = 0;
  first_instruction_cycle = 0;
  instruction_cycles.clear();
}

void run_proc(proc_stats_t* p_stats)
{
  bool done = false;
  
  while (!done) {
    current_cycle++;
    
    // Track dispatch queue size for statistics
    if (disp_size_count > 0 || !dispatch_queue.empty()) {
        total_disp_size += dispatch_queue.size();
        disp_size_count++;
        if (dispatch_queue.size() > max_disp_size) {
            max_disp_size = dispatch_queue.size();
        }
    }
    
    // FIRST HALF OF CYCLE
    state_update_stage_first_half();
    execute_stage_first_half();
    schedule_stage_first_half();
    dispatch_stage_first_half();

    // SECOND HALF OF CYCLE
    state_update_stage_second_half();
    execute_stage_second_half();
    schedule_stage_second_half();
    dispatch_stage_second_half();

    fetch_stage();

    done = (read_instruction(NULL) == false) && dispatch_queue.empty() && all_rs_empty();
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
    
    if (disp_size_count > 0) {
        p_stats->avg_disp_size = (double)total_disp_size / (double)disp_size_count;
    } else {
        p_stats->avg_disp_size = 0.0;
    }
    
    // print the cycle to the output file
    fprintf(output, "INST\tFETCH\tDISP\tSCHED\tEXEC\tSTATE\n");
    for (const auto& pair : instruction_cycles) {
        const auto& cycles = pair.second;
        fprintf(output, "%llu\t%llu\t%llu\t%llu\t%llu\t%llu\n", 
               (unsigned long long)pair.first, 
               (unsigned long long)cycles.fetch, 
               (unsigned long long)cycles.dispatch, 
               (unsigned long long)cycles.schedule, 
               (unsigned long long)cycles.execute, 
               (unsigned long long)cycles.state_update);
    }
}

void fetch_stage() {
  for (uint64_t i = 0; i < FETCH_RATE; i++) {
    proc_inst_t inst;
    if (read_instruction(&inst)) {
      inst.tag = global_tag_counter++;
      dispatch_queue.push_back(inst);

      fprintf(logging, "%llu\tFETCHED\t%llu\n", current_cycle, inst.tag);
      
      // record the fetch cycle
      instruction_cycles[inst.tag].fetch = current_cycle;
      instruction_cycles[inst.tag].dispatch = current_cycle + 1;
      
      if (first_instruction_cycle == 0) {
          first_instruction_cycle = current_cycle;
      }
    }
  }
}

void dispatch_stage_first_half() {
    // reserve slots in RS
    for (size_t i = 0; i < reservation_station.size(); i++) {
        // any slot that isn't valid can be reserved; bounded by RS size
        if (!reservation_station[i].valid && reserved_slots < reservation_station.size() && reserved_slots < dispatch_queue.size()) {
            reserved_slots++;
        }
    }
}

void dispatch_stage_second_half() {
    // add reserved_slots instructions to RS
    for (size_t i = 0; i < reservation_station.size(); i++) {
        if (reserved_slots == 0 || dispatch_queue.empty()) {
            break;
        }
        if (!reservation_station[i].valid) {
            proc_inst_t inst = dispatch_queue.front();
            dispatch_queue.pop_front();
            
            // create the new RS entry
            rs_entry_t& entry = reservation_station[i];
            entry.valid = true;
            entry.instruction = inst;
            // check whether srcs are ready (latest writer has state-updated)
            entry.src1_ready = (inst.src_reg[0] == -1) || register_status[inst.src_reg[0]].ready;
            entry.src2_ready = (inst.src_reg[1] == -1) || register_status[inst.src_reg[1]].ready;
            // if not ready, record the latest instruction (not state-updated yet) that writes to that src register (call this the direct parent. this is for tracking multiple sequential writes to same register, and RAW dependencies). later, once the tag of the parent is state-updated, we mark this src as ready. if any other writer is state-updated, the src is still not ready.
            entry.src1_parent = (inst.src_reg[0] != -1) ? register_status[inst.src_reg[0]].tag : 0;
            entry.src2_parent = (inst.src_reg[1] != -1) ? register_status[inst.src_reg[1]].tag : 0;
            entry.fired = false;
            entry.completed = false;
            entry.state_updated = false;
            entry.execute_cycles_left = 0;
            entry.completed_cycle = 0;
            entry.tag_dispatched = false;
            entry.fetch_cycle = current_cycle - 1; // fetched in previous cycle
            entry.dispatch_cycle = current_cycle;   // dispatched this cycle
            
            // log the dispatch
            fprintf(logging, "%llu\tDISPATCHED\t%llu\n", current_cycle, inst.tag);
            
            // record dispatch cycle
            instruction_cycles[inst.tag].dispatch = current_cycle;
            
            // mark destination register (register we write to) as not ready, so future instructions know the dependency. this instruction is the new parent for the register for future instructions (instructions are always dispatched in tag order, so this is always the latest dispatched writer).
            if (inst.dest_reg != -1) {
                register_status[inst.dest_reg].ready = false;
                register_status[inst.dest_reg].tag = inst.tag;
            }
            
            reserved_slots--;
        }
    }
}

// FIRE READY INSTRUCTIONS
void schedule_stage_first_half() {
    // try to fire instructions in RS
    for (auto& entry : reservation_station) {
        if (!entry.valid || entry.fired) {
            continue; 
        }
        // print src readiness
        // fprintf(logging, "%llu\tSRC_READY\t%llu\t%d\t%d\n", current_cycle, entry.instruction.tag, entry.src1_ready, entry.src2_ready);
        // check if both sources are ready
        if (entry.src1_ready && entry.src2_ready) {
            // check if FU is available, and mark it as used
            int fu_type = (entry.instruction.op_code == -1) ? 1 : entry.instruction.op_code;
            bool fu_available = false;
            if (fu_type == 0 && k0_fu_available > 0) {
                fu_available = true;
                k0_fu_available--;
            } else if (fu_type == 1 && k1_fu_available > 0) {
                fu_available = true;
                k1_fu_available--;
            } else if (fu_type == 2 && k2_fu_available > 0) {
                fu_available = true;
                k2_fu_available--;
            }
            // print fu availability
            // fprintf(logging, "%llu\tFU_AVAILABLE\t%llu\t%d\t%d\n", current_cycle, entry.instruction.tag, fu_available, fu_type ? 1 : 0);
            if (fu_available) {
                // fire the instruction
                entry.fired = true;
                fprintf(logging, "%llu\tSCHEDULED\t%llu\n", current_cycle, entry.instruction.tag);
                instruction_cycles[entry.instruction.tag].schedule = current_cycle;
            }
        }
    }
}

// WAKEUP INSTRUCTIONS IN RS AFTER BROADCAST
void schedule_stage_second_half() {
    // wakeup instructions from the broadcast in the first half; update the src register readiness in the RS
    // if the src register's direct parent is state-updated (i.e., its tag is in result_bus_tags), mark src as ready
    for (auto& entry : reservation_station) {
        if (!entry.valid || entry.fired) {
            continue; 
        }
        for (auto tag : result_bus_tags) {
            if (!entry.src1_ready && entry.src1_parent == tag) {
                entry.src1_ready = true;
            }
            if (!entry.src2_ready && entry.src2_parent == tag) {
                entry.src2_ready = true;
            }
        }
    }
}

// EXECUTE FIRED INSTRUCTIONS
// BROADCAST AVAILABILITIES TO RESULT BUS
void execute_stage_first_half() {
    // mark all fired instructions as completed and add to waiting queue
    for (auto& entry : reservation_station) {
        if (!entry.valid || !entry.fired || entry.completed) {
            continue; 
        }
        // execute the instruction
        entry.completed = true;
        completed_instructions.push_back(&entry);
        entry.completed_cycle = current_cycle;
        
        // log execution
        fprintf(logging, "%llu\tEXECUTED\t%llu\n", current_cycle, entry.instruction.tag);
        instruction_cycles[entry.instruction.tag].execute = current_cycle;
    }
    
    // broadcast on result bus - limited to RESULT_BUSES instructions
    // take from the front of completed_instructions (oldest first)
    uint64_t broadcasts = 0;
    result_bus_tags.clear();
    while (broadcasts < RESULT_BUSES && !completed_instructions.empty()) {
        // get the earliest completed instruction (front of queue)
        rs_entry_t* entry_ptr = completed_instructions.front();
        completed_instructions.erase(completed_instructions.begin());
        
        result_bus_tags.push_back(entry_ptr->instruction.tag);
        entry_ptr->state_updated = true;
        entry_ptr->state_update_cycle = current_cycle;

        // dest register of the instruction is now ready if this instruction is the latest writer
        if (entry_ptr->instruction.dest_reg != -1 && 
            register_status[entry_ptr->instruction.dest_reg].tag == entry_ptr->instruction.tag) {
            register_status[entry_ptr->instruction.dest_reg].ready = true;
        }

        // free the FU used by this instruction
        int fu_type = (entry_ptr->instruction.op_code == -1) ? 1 : entry_ptr->instruction.op_code;
        if (fu_type == 0) {
            k0_fu_available++;
        } else if (fu_type == 1) {
            k1_fu_available++;
        } else if (fu_type == 2) {
            k2_fu_available++;
        }
        
        broadcasts++;
    }
}

// NOTHING TO DO IN SECOND HALF OF EXECUTE STAGE
void execute_stage_second_half() {
}

// NOTHING TO DO IN FIRST HALF OF STATE UPDATE STAGE
void state_update_stage_first_half() {
}

// DELETE STATE-UPDATED INSTRUCTIONS FROM RS
void state_update_stage_second_half() 
{
  // Remove state-updated instructions from RS
  for (auto& entry : reservation_station) {
    if (entry.valid && entry.state_updated && entry.state_update_cycle < current_cycle) {
      // log state update
      fprintf(logging, "%llu\tSTATE UPDATE\t%llu\n", current_cycle, entry.instruction.tag);
      entry.valid = false;
      entry.state_updated = false;
      instructions_retired++;
    }
  }
}

bool all_rs_empty() 
{
  for (const auto& entry : reservation_station) {
    if (entry.valid) return false;
  }
  return true;
}