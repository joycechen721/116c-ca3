#include "procsim.hpp"
#include <deque>
#include <vector>
#include <algorithm>
#include <cstdio>

// global processor states
uint64_t RESULT_BUSES;
uint64_t K0_FU_COUNT;
uint64_t K1_FU_COUNT;
uint64_t K2_FU_COUNT;
uint64_t FETCH_RATE;

// dispatch queue
std::deque<proc_inst_t> dispatch_queue;

// reservation station
std::vector<rs_entry_t> reservation_station;

// result bus: tags of instructions that broadcast this cycle
std::vector<uint64_t> result_bus_tags;

// register ready table
const int32_t NUM_REGISTERS = 128;
bool register_ready[NUM_REGISTERS];

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

void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f) 
{
  RESULT_BUSES = r;
  K0_FU_COUNT = k0;
  K1_FU_COUNT = k1;
  K2_FU_COUNT = k2;
  FETCH_RATE = f;

  dispatch_queue.clear();
  reservation_station.clear();

  uint64_t rs_size = 2 * (K0_FU_COUNT + K1_FU_COUNT + K2_FU_COUNT);
  reservation_station.resize(rs_size);
  for (auto& entry : reservation_station) {
      entry.valid = false;
  }

  for (int32_t i = 0; i < NUM_REGISTERS; i++) {
      register_ready[i] = true;
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
    
    // FIRST HALF OF CYCLE (in order as specified)
    // 1. Register file is written (ready bits updated) via result bus
    // 2. FU of completed instruction is marked free
    // 3. Any independent instruction in scheduling queue is marked to fire
    // 4. Dispatch unit reserves slots in scheduling queues
    // state_update_stage_first_half();  // Writes register file & frees FUs
    // lol();
    // Execute stage (decrement cycles)
    state_update_stage_first_half();  // Writes register file & frees FUs
    execute_stage_first_half();                  // Decrement execute cycles
    schedule_stage_first_half();                 // Marks instructions to fire
    dispatch_stage_first_half();                 // Reserves slots in RS
    fetch_stage();                             

    // SECOND HALF OF CYCLE
    // 1. Register file is read by Dispatch
    // 2. Scheduling queues are updated via result bus
    // 3. State update unit deletes completed instructions from scheduling queue
    dispatch_stage_second_half();        
    schedule_stage_second_half();                 // Fires instructions  
    execute_stage_second_half();                  // (nothing)       

    state_update_stage_second_half();  // Delete from RS


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
}

void fetch_stage() {
  for (uint64_t i = 0; i < FETCH_RATE; i++) {
    proc_inst_t inst;
    if (read_instruction(&inst)) {
      inst.tag = global_tag_counter++;
      dispatch_queue.push_back(inst);
      printf("%llu\tFETCHED\t\t%llu\n", current_cycle, inst.tag);
      if (first_instruction_cycle == 0) {
          first_instruction_cycle = current_cycle;
      }
    }
  }
}

void dispatch_stage_first_half() {
    // Reserve slots in RS (scan dispatch queue from head to tail)
    for (auto& entry : reservation_station) {
        if (dispatch_queue.empty()) break;

        if (!entry.valid) {
            entry.instruction = dispatch_queue.front();
            entry.valid = true;
            entry.src1_ready = false;
            entry.src2_ready = false;
            entry.fired = false;
            entry.completed = false;
            entry.execute_cycles_left = 0;
            entry.tag_dispatched = true;
            dispatch_queue.pop_front();
            printf("%llu\tDISPATCHED\t%llu\n", current_cycle, entry.instruction.tag);
        }
    }
}

void dispatch_stage_second_half() {
    // Read register file (update source readiness for instructions in RS)
    // for (auto& entry : reservation_station) {
    //     if (!entry.valid) {
    //         continue;
    //     }
    //     // Update source readiness
    //     entry.src1_ready = (entry.instruction.src_reg[0] == -1) || register_ready[entry.instruction.src_reg[0]];
    //     entry.src2_ready = (entry.instruction.src_reg[1] == -1) || register_ready[entry.instruction.src_reg[1]];
    // }
}

void schedule_stage_first_half() {
    // Collect all ready instructions
    std::vector<std::pair<uint64_t, size_t>> ready_instructions; // <tag, index>
    
    for (size_t i = 0; i < reservation_station.size(); i++) {
        auto& entry = reservation_station[i];
        if (!entry.valid || entry.fired || entry.completed) {
            continue;
        }

        if (entry.src1_ready && entry.src2_ready) {
            ready_instructions.push_back({entry.instruction.tag, i});
        }
    }
    
    // Sort by tag order (lowest tag first)
    std::sort(ready_instructions.begin(), ready_instructions.end());
    
    // Process instructions in tag order
    for (const auto& [tag, idx] : ready_instructions) {
        auto& entry = reservation_station[idx];
        
        int fu_type = (entry.instruction.op_code == -1) ? 1 : entry.instruction.op_code;

        // Check if the required FU is available
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
        
        // If FU is available, fire the instruction
        if (fu_available) {
            entry.fired = true;
            entry.execute_cycles_left = 1; // All FUs have latency 1
            printf("%llu\tSCHEDULED\t%llu\n", current_cycle, entry.instruction.tag);
            
            // Mark destination register as not ready (will be ready when instruction completes)
            if (entry.instruction.dest_reg != -1) {
                register_ready[entry.instruction.dest_reg] = false;
            }
        }
    }
}

void schedule_stage_second_half() {
    // update source readiness for instructions in RS
    for (auto& entry : reservation_station) {
        if (!entry.valid || entry.fired || entry.completed) {
            continue; 
        }
        // Update source readiness
        entry.src1_ready = (entry.instruction.src_reg[0] == -1) || register_ready[entry.instruction.src_reg[0]];
        entry.src2_ready = (entry.instruction.src_reg[1] == -1) || register_ready[entry.instruction.src_reg[1]];
    }
}

void execute_stage_first_half() {
  // Decrement execution cycles for fired instructions
  for (auto& entry : reservation_station) {
    if (entry.valid && entry.fired && !entry.completed) {
      if (entry.execute_cycles_left > 0) {
        entry.execute_cycles_left--;
        if (entry.execute_cycles_left == 0) {
          entry.completed = true;
          entry.completed_cycle = current_cycle; // Track when it completed
          printf("%llu\tEXECUTED\t%llu\n", current_cycle, entry.instruction.tag);
        }
      }
    }
  }

    // Find completed instructions waiting for result bus
    std::vector<int> completed_indices;
    for (size_t i = 0; i < reservation_station.size(); i++) {
        if (reservation_station[i].valid && reservation_station[i].completed && 
            !reservation_station[i].state_updated) {
            completed_indices.push_back(i);
        }
    }

    // Sort by completion cycle first (earlier completions first), then by tag
    std::sort(completed_indices.begin(), completed_indices.end(),
    [](int a, int b) {
        const auto& entry_a = reservation_station[a];
        const auto& entry_b = reservation_station[b];
        
        // Prioritize earlier completion cycles
        if (entry_a.completed_cycle != entry_b.completed_cycle) {
            return entry_a.completed_cycle < entry_b.completed_cycle;
        }
        // If completed in same cycle, use tag order
        return entry_a.instruction.tag < entry_b.instruction.tag;
    });

    // Add to result bus, if there is space (up to RESULT_BUSES)
    for (size_t i = 0; i < completed_indices.size() && result_bus_tags.size() < RESULT_BUSES; i++) {
        int idx = completed_indices[i];
        result_bus_tags.push_back(reservation_station[idx].instruction.tag);
        // printf("%llu\tBROADCAST\t%llu\n", current_cycle, reservation_station[idx].instruction.tag);

        // Free up the FU
        rs_entry_t& rs_entry = reservation_station[idx];
        int fu_type = (rs_entry.instruction.op_code == -1) ? 1 : rs_entry.instruction.op_code;
        if (fu_type == 0) {
            k0_fu_available++;
        } else if (fu_type == 1) {
            k1_fu_available++;
        } else if (fu_type == 2) {
            k2_fu_available++;
        }

        // Mark destination register as ready
        if (rs_entry.instruction.dest_reg != -1) {
            register_ready[rs_entry.instruction.dest_reg] = true;
        }
    }
}

void execute_stage_second_half() {
  // Nothing to do in second half for execute stage
}

void state_update_stage_first_half() {
    // clear the result bus
    for (auto& entry : result_bus_tags) {
        printf("%llu\tSTATE UPDATE\t%llu\n", current_cycle, entry);
        // Find the RS entry with this tag
        rs_entry_t* rs_entry_ptr = nullptr;
        for (auto& rs_entry : reservation_station) {
            if (rs_entry.valid && rs_entry.instruction.tag == entry) {
                rs_entry_ptr = &rs_entry;
                break;
            }
        }
        if (rs_entry_ptr == nullptr) {
            continue; // Should not happen
        }
        rs_entry_t& rs_entry = *rs_entry_ptr;
        rs_entry.state_updated = true;
    }
    result_bus_tags.clear();
}

void state_update_stage_second_half() 
{
  // Remove state-updated instructions from RS
  for (auto& entry : reservation_station) {
    if (entry.valid && entry.state_updated) {
      entry.valid = false;
      entry.state_updated = false;
      instructions_retired++;
    //   printf("%llu\tRETIRED\t\t%llu\n", current_cycle, entry.instruction.tag);
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