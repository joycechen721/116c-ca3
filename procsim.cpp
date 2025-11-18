#include "procsim.hpp"
#include <deque>
#include <vector>

// global processor states
uint64_t RESULT_BUSES;
uint64_t K0_FU_COUNT;
uint64_t K1_FU_COUNT;
uint64_t K2_FU_COUNT;
uint64_t FETCH_RATE;

// dispatch queue
std::deque<proc_inst_t> dispatch_queue;

// reservation station
std::deque<rs_entry_t> reservation_station;

// register ready table
const int32_t NUM_REGISTERS = 128;
bool register_ready[NUM_REGISTERS];

// FU ready counts
uint64_t k0_fu_available;
uint64_t k1_fu_available;
uint64_t k2_fu_available;

// result bus ready count
uint64_t result_buses_available;

// global counters
uint64_t global_tag_counter = 1;
uint64_t current_cycle = 0;

// statistics tracking
uint64_t max_disp_size = 0;
uint64_t total_disp_size = 0;
uint64_t disp_size_count = 0;
uint64_t instructions_fired = 0;
uint64_t instructions_retired = 0;

/**
 * Subroutine for initializing the processor. You many add and initialize any global or heap
 * variables as needed.
 * XXX: You're responsible for completing this routine
 *
 * @r ROB size
 * @k0 Number of k0 FUs
 * @k1 Number of k1 FUs
 * @k2 Number of k2 FUs
 * @f Number of instructions to fetch
 */
void setup_proc(uint64_t r, uint64_t k0, uint64_t k1, uint64_t k2, uint64_t f) 
{
  RESULT_BUSES = r; // number of result buses
  K0_FU_COUNT = k0; // number of k0 functional units
  K1_FU_COUNT = k1; // number of k1 functional units
  K2_FU_COUNT = k2; // number of k2 functional units
  FETCH_RATE = f;   // number of instructions to fetch

  dispatch_queue.clear();
  reservation_station.clear();

  // initialize reservation station
  uint64_t rs_size = 2 * (K0_FU_COUNT + K1_FU_COUNT + K2_FU_COUNT);
  reservation_station.resize(rs_size);
  for (auto& entry : reservation_station) {
      entry.valid = false;
  }
  // initialize register ready table
  for (int32_t i = 0; i < NUM_REGISTERS; i++) {
      register_ready[i] = true; // all registers are initially ready
  }

  // initialize FU ready counts
  k0_fu_available = K0_FU_COUNT;
  k1_fu_available = K1_FU_COUNT;
  k2_fu_available = K2_FU_COUNT;

  // initialize result bus ready count
  result_buses_available = RESULT_BUSES;
  printf("CYCLE	OPERATION	INSTRUCTION\n");
}

/**
 * Subroutine that simulates the processor.
 *   The processor should fetch instructions as appropriate, until all instructions have executed
 * XXX: You're responsible for completing this routine
 *
 * @p_stats Pointer to the statistics structure
 */
void run_proc(proc_stats_t* p_stats)
{
  bool done = false;
  
  while (!done) {
    current_cycle++;    
    // state_update_stage();
    // execute_stage();
    // schedule_stage();
    dispatch_stage();
    fetch_stage();
  }
}

/**
 * Subroutine for cleaning up any outstanding instructions and calculating overall statistics
 * such as average IPC, average fire rate etc.
 * XXX: You're responsible for completing this routine
 *
 * @p_stats Pointer to the statistics structure
 */
void complete_proc(proc_stats_t *p_stats) 
{

}

// FETCH STAGE
void fetch_stage() {
  for (uint64_t i = 0; i < FETCH_RATE; i++) {
    proc_inst_t inst;
    if (read_instruction(&inst)) {
      inst.tag = global_tag_counter++;
      printf("%llu\tFETCHED\t\t%llu\n", current_cycle, inst.tag);
      dispatch_queue.push_back(inst);
    } else {
      break;
    }
  }
}

// DISPATCH STAGE
void dispatch_stage() {
  uint64_t rs_size = 2 * (K0_FU_COUNT + K1_FU_COUNT + K2_FU_COUNT);
  while (!dispatch_queue.empty()) {
    // find empty slot in RS
    int empty_slot = -1;
    for (uint64_t i = 0; i < rs_size; i++) {
      if (!reservation_station[i].valid) {
        empty_slot = i;
        break;
      }
    }
    // move instruction from dispatch queue to RS
    if (empty_slot != -1) {
      proc_inst_t inst = dispatch_queue.front();
      dispatch_queue.pop_front();

      rs_entry_t rs_entry;
      rs_entry.instruction = inst;
      rs_entry.ready_to_fire = false;
      rs_entry.fired = false;
      rs_entry.completed = false;
      rs_entry.execute_cycles_left = 0;
      reservation_station[empty_slot] = rs_entry;
      printf("%llu\tDISPATCHED\t%llu\n", current_cycle, inst.tag);
    }
  }
}