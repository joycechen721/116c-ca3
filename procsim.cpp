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

// reservation station
std::vector<rs_entry_t> reservation_station;

// result bus: tags of instructions that broadcast this cycle
std::vector<uint64_t> result_bus_tags;

// register ready table
const int32_t NUM_REGISTERS = 128;
bool register_ready[NUM_REGISTERS];

// Track the last instruction that state-updated for each register
// Any writer before this is irrelevant
uint64_t register_last_state_updated[NUM_REGISTERS];

// Track ALL in-flight writers (dispatched but not state-updated) for each register
std::set<uint64_t> register_in_flight_writers[NUM_REGISTERS];

// Track the last scheduled writer (for making register not ready)
uint64_t register_last_scheduled_writer[NUM_REGISTERS];

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

FILE* hi = fopen("output.txt", "w");
FILE* out = fopen("meow.txt", "w");

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

  fprintf(hi, "CYCLE\tOPERATION\tINSTRUCTION\n");

  uint64_t rs_size = 2 * (K0_FU_COUNT + K1_FU_COUNT + K2_FU_COUNT);
  reservation_station.resize(rs_size);
  for (auto& entry : reservation_station) {
      entry.valid = false;
  }

  for (int32_t i = 0; i < NUM_REGISTERS; i++) {
      register_ready[i] = true;
      register_last_state_updated[i] = 0;
      register_in_flight_writers[i].clear();
      register_last_scheduled_writer[i] = 0;
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
    fetch_stage();

    // SECOND HALF OF CYCLE
    dispatch_stage_second_half();
    schedule_stage_second_half();
    execute_stage_second_half();
    state_update_stage_second_half();

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
    
    // Print the instruction cycle table to meow.txt
    fprintf(out, "INST\tFETCH\tDISP\tSCHED\tEXEC\tSTATE\n");
    for (const auto& pair : instruction_cycles) {
        const auto& cycles = pair.second;
        fprintf(out, "%llu\t%llu\t%llu\t%llu\t%llu\t%llu\n", 
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

      fprintf(hi, "%llu\tFETCHED\t%llu\n", current_cycle, inst.tag);
      
      // Record fetch cycle
      instruction_cycles[inst.tag].fetch = current_cycle;
      instruction_cycles[inst.tag].dispatch = current_cycle + 1;
      
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
            entry.state_updated = false;
            entry.fired_cycle = 0;
            
            // Record dispatch cycle
            instruction_cycles[entry.instruction.tag].schedule = current_cycle + 1;

            // Add this instruction to the in-flight writers list if it has a destination
            if (entry.instruction.dest_reg != -1) {
                register_in_flight_writers[entry.instruction.dest_reg].insert(entry.instruction.tag);
            }
            
            dispatch_queue.pop_front();
            fprintf(hi, "%llu\tDISPATCHED\t%llu\n", current_cycle, entry.instruction.tag);
        }
    }
}

void dispatch_stage_second_half() {
    // Nothing to do here
}

void schedule_stage_first_half() {
    // Collect all ready instructions
    std::vector<std::pair<uint64_t, size_t>> ready_instructions; // <tag, index>
    
    for (size_t i = 0; i < reservation_station.size(); i++) {
        auto& entry = reservation_station[i];
        if (!entry.valid || entry.fired || entry.completed) {
            continue;
        }

        // An instruction can read a register when:
        // 1. No source register (src == -1), OR
        // 2. last_state_updated >= my_tag (the value I need is already in the register), OR
        // 3. last_state_updated < my_tag AND no in-flight writers exist between them
        
        bool src1_ready = true;
        if (entry.instruction.src_reg[0] != -1) {
            int32_t src_reg = entry.instruction.src_reg[0];
            uint64_t last_state_updated = register_last_state_updated[src_reg];
            const auto& writers = register_in_flight_writers[src_reg];

            //print out last state updated and destination register
            fprintf(hi, "%llu\tLAST_STATE_UPDATED\t%llu\t%llu\n", current_cycle, last_state_updated, entry.instruction.dest_reg);
            // print all elements in register inflight
            for (uint64_t writer_tag : writers) {
                fprintf(hi, "%llu\tIN_FLIGHT_WRITER\t%llu\t%llu\n", current_cycle, writer_tag, entry.instruction.dest_reg);
            }
            
            // If the last state-updated writer has tag >= my_tag, the value is ready
            if (last_state_updated >= entry.instruction.tag) {
                src1_ready = true;
                for (uint64_t writer_tag : writers) {
                    if (writer_tag < entry.instruction.tag && writer_tag < last_state_updated) {
                        src1_ready = false;  // Found a writer I depend on that hasn't state-updated
                        break;
                    }
                }
            } else {
                if (register_in_flight_writers[src_reg].empty() ||
                last_state_updated >= *std::max_element(
                    register_in_flight_writers[src_reg].begin(),
                    register_in_flight_writers[src_reg].end()
                )) {
                    src1_ready = true;
                } else {
                    // last_state_updated < my_tag
                    // Check if there are any in-flight writers between last_state_updated and my_tag
                    src1_ready = true;  // Assume ready
                    for (uint64_t writer_tag : writers) {
                        if (writer_tag > last_state_updated && writer_tag < entry.instruction.tag) {
                            src1_ready = false;  // Found a writer I depend on that hasn't state-updated
                            break;
                        }
                    }
                }
            }
        }
        
        bool src2_ready = true;
        if (entry.instruction.src_reg[1] != -1) {
            int32_t src_reg = entry.instruction.src_reg[1];
            uint64_t last_state_updated = register_last_state_updated[src_reg];
            const auto& writers = register_in_flight_writers[src_reg];
            
            // If the last state-updated writer has tag >= my_tag, the value is ready
            if (last_state_updated >= entry.instruction.tag) {
                src2_ready = true;
                for (uint64_t writer_tag : writers) {
                    if (writer_tag < entry.instruction.tag && writer_tag < last_state_updated) {
                        src2_ready = false;  // Found a writer I depend on that hasn't state-updated
                        break;
                    }
                }
            } else {
                src2_ready = false;
                if (register_in_flight_writers[src_reg].empty() ||
                last_state_updated >= *std::max_element(
                    register_in_flight_writers[src_reg].begin(),
                    register_in_flight_writers[src_reg].end()
                )) {
                src2_ready = true;
                } else {
                    // last_state_updated < my_tag
                    // Check if there are any in-flight writers between last_state_updated and my_tag
                    src2_ready = true;  // Assume ready
                    for (uint64_t writer_tag : writers) {
                        if (writer_tag > last_state_updated && writer_tag < entry.instruction.tag) {
                            src2_ready = false;  // Found a writer I depend on that hasn't state-updated
                            break;
                        }
                    }
                }
            }
        }
        
        if (src1_ready && src2_ready) {
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
            entry.fired_cycle = current_cycle;
            entry.execute_cycles_left = 1; // All FUs have latency 1
            fprintf(hi, "%llu\tSCHEDULED\t%llu\n", current_cycle, entry.instruction.tag);
            
            // Mark destination register as not ready and track as last scheduled writer
            if (entry.instruction.dest_reg != -1) {
                register_ready[entry.instruction.dest_reg] = false;
                register_last_scheduled_writer[entry.instruction.dest_reg] = entry.instruction.tag;
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
  // Collect all instructions completing this cycle
  std::vector<size_t> completing_indices;
  
  for (size_t i = 0; i < reservation_station.size(); i++) {
    auto& entry = reservation_station[i];
    if (entry.valid && entry.fired && !entry.completed) {
      if (entry.execute_cycles_left > 0) {
        entry.execute_cycles_left--;
        if (entry.execute_cycles_left == 0) {
          entry.completed = true;
          entry.completed_cycle = current_cycle;
          completing_indices.push_back(i);
        }
      }
    }
  }
  
  // Sort by FU type first, then by tag order
  std::sort(completing_indices.begin(), completing_indices.end(),
    [](size_t a, size_t b) {
        const auto& entry_a = reservation_station[a];
        const auto& entry_b = reservation_station[b];
        
        int fu_type_a = (entry_a.instruction.op_code == -1) ? 1 : entry_a.instruction.op_code;
        int fu_type_b = (entry_b.instruction.op_code == -1) ? 1 : entry_b.instruction.op_code;
        
        // Sort by FU type first
        if (fu_type_a != fu_type_b) {
            return fu_type_a < fu_type_b;
        }
        // Then by tag order
        return entry_a.instruction.tag < entry_b.instruction.tag;
    });
  
  // Print EXECUTED events in sorted order
  for (size_t idx : completing_indices) {
      fprintf(hi, "%llu\tEXECUTED\t%llu\n", current_cycle, reservation_station[idx].instruction.tag);
      instruction_cycles[reservation_station[idx].instruction.tag].execute = current_cycle;
  }

    // Find completed instructions waiting for result bus
    std::vector<int> completed_indices;
    for (size_t i = 0; i < reservation_station.size(); i++) {
        if (reservation_station[i].valid && reservation_station[i].completed && 
            !reservation_station[i].state_updated) {
            completed_indices.push_back(i);
        }
    }

    // Sort by COMPLETION CYCLE first (earlier completions prioritized), then by TAG ORDER
    std::sort(completed_indices.begin(), completed_indices.end(),
    [](int a, int b) {
        const auto& entry_a = reservation_station[a];
        const auto& entry_b = reservation_station[b];
        
        // Prioritize instructions that completed earlier
        if (entry_a.completed_cycle != entry_b.completed_cycle) {
            return entry_a.completed_cycle < entry_b.completed_cycle;
        }
        // If completed in same cycle, use tag order
        return entry_a.instruction.tag < entry_b.instruction.tag;
    });

    // Add to result bus, respecting the R limit
    size_t buses_used = 0;
    for (int idx : completed_indices) {
        if (buses_used >= RESULT_BUSES) break;
        
        result_bus_tags.push_back(reservation_station[idx].instruction.tag);
        buses_used++;

        // Free up the FU when instruction gets on result bus
        rs_entry_t& rs_entry = reservation_station[idx];
        int fu_type = (rs_entry.instruction.op_code == -1) ? 1 : rs_entry.instruction.op_code;
        if (fu_type == 0) {
            k0_fu_available++;
        } else if (fu_type == 1) {
            k1_fu_available++;
        } else if (fu_type == 2) {
            k2_fu_available++;
        }
    }
}

void execute_stage_second_half() {
  // Nothing to do in second half for execute stage
}

void state_update_stage_first_half() {
    // State updates happen in the order instructions got on the result bus
    // which is: completion cycle first, then tag order
    
    // Get completion cycles for sorting
    std::vector<std::pair<uint64_t, uint64_t>> tag_with_completion;
    for (auto tag : result_bus_tags) {
        for (const auto& rs_entry : reservation_station) {
            if (rs_entry.valid && rs_entry.instruction.tag == tag) {
                tag_with_completion.push_back({tag, rs_entry.completed_cycle});
                break;
            }
        }
    }
    
    // Sort by completion cycle first, then tag order
    std::sort(tag_with_completion.begin(), tag_with_completion.end(),
        [](const std::pair<uint64_t, uint64_t>& a, const std::pair<uint64_t, uint64_t>& b) {
            if (a.second != b.second) {
                return a.second < b.second;
            }
            return a.first < b.first;
        });
    
    // Process in sorted order
    for (const auto& [tag, completion_cycle] : tag_with_completion) {
        fprintf(hi, "%llu\tSTATE UPDATE\t%llu\n", current_cycle, tag);
        // Record state update cycle
        instruction_cycles[tag].state_update = current_cycle;
        
        // Find the RS entry with this tag
        for (auto& rs_entry : reservation_station) {
            if (rs_entry.valid && rs_entry.instruction.tag == tag) {
                rs_entry.state_updated = true;
                
                // Update last state-updated writer and remove from in-flight list
                if (rs_entry.instruction.dest_reg != -1) {
                    int32_t dest_reg = rs_entry.instruction.dest_reg;
                    
                    // Update the last state-updated tag for this register
                    // (keep track of the maximum tag that has state-updated)
                    if (tag > register_last_state_updated[dest_reg]) {
                        register_last_state_updated[dest_reg] = tag;
                    }
                    
                    // Remove ONLY this instruction from in-flight list
                    // Keep earlier writers because other instructions may still depend on them
                    register_in_flight_writers[dest_reg].erase(tag);
                    
                    // Mark register as ready only if this was the last scheduled writer
                    if (register_last_scheduled_writer[dest_reg] == tag) {
                        register_ready[dest_reg] = true;
                        register_last_scheduled_writer[dest_reg] = 0;
                    }
                }
                break;
            }
        }
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