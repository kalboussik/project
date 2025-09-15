// Enhanced wubble.h with Algorithm support
#ifndef _WUBBLE_ENHANCED_H 
#define _WUBBLE_ENHANCED_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../config/config.h"

#ifdef __cplusplus 
extern "C" { 
#endif

/**
 * @brief Host context enumeration
 * Represents different environmental/mobility contexts
 */
typedef enum {
    H1 = 1,    // Low density environment
    H2 = 2,    // Medium density environment  
    H3 = 3,    // Medium-high density environment
    H4 = 4,    // High density environment
    H5 = 5,    // Mobile/isolated context
    H_UNKNOWN = 0  // Unknown context
} host_context_t;

/**
 * @brief Probability table entry for P(H | H')
 */
typedef struct {
    host_context_t h_prime;  // Previous host context H'
    float probability;       // P(H | H') - probability of current context given previous
} probability_entry_t;

/**
 * @brief Algorithm state structure
 * Contains all state variables used in both algorithms
 */
typedef struct {
    host_context_t current_host_context;    // H - current host context
    host_context_t previous_host_context;   // H' - previous host context
    float confidence_degree;                 // conf - confidence in current context
    uint32_t time_in_host_context;          // ΔHost - time in current context (ms)
    uint32_t time_in_sleep;                  // ΔSleep - time in sleep mode (ms)
    uint32_t num_advertisements;             // NAdv - number of self-advertisements
    uint32_t num_wur_on_received;           // NWUR-on - WuR ON signals received
    bool eligibility;                        // eligibility flag
    uint32_t wur_authority_state;           // WuA - wake-up authority state
    uint32_t wur_signal_state;              // WuS - wake-up signal state  
    bool wur_on_received;                   // WuR On flag
    bool wur_off_received;                  // WuR Off flag
} algorithm_state_t;

/**
 * @brief Neighbor information structure
 * Extended from existing neighbour_t to include context info
 */
typedef struct {
    uint8_t neighbour[4];           // Neighbor ID
    uint8_t states_neighbour;       // Neighbor state
    host_context_t context;         // Neighbor's host context
    float confidence;               // Confidence in neighbor's context
    uint32_t last_seen;            // Timestamp of last contact
} enhanced_neighbour_t;

/**
 * @brief Enhanced neighbor table
 */
typedef struct {
    size_t size;
    enhanced_neighbour_t neighbours[MAX_NEIGHBOURS];
} enhanced_neighbour_table_t;

// Algorithm thresholds and parameters
#define TH_ELIGIBILITY 0.3f     // Threshold for eligibility calculation
#define TH_RELEVANCE 0.4f       // Threshold for relevance check
#define REF_HOST 5000           // Reference time in host context (ms)
#define MAX_BACKOFF 10000       // Maximum backoff time (ms)
#define MAX1_SCAN 5000          // Maximum scan time 1 (ms)
#define MAX2_BROADCAST 3000     // Maximum broadcast time (ms)
#define K_NEIGHBORS 5           // Target number of neighbors to discover

/**
 * @brief Get probability P(H | H') from hardcoded table
 * @param h_prime Previous host context
 * @return Probability value (0.0 to 1.0)
 */
float get_probability(host_context_t h_prime);

/**
 * @brief Calculate backoff time using Equation 3
 * @return Backoff time in milliseconds
 */
uint32_t calculate_backoff_time(void);

/**
 * @brief Update Wake-up Authority (WuA) based on eligibility
 * @param wus Wake-up signal state
 * @param eligibility Current eligibility status
 */
void update_wua(uint32_t wus, bool eligibility);

/**
 * @brief Identify entity and determine host context
 * Implementation of equations 4, 5, 6 from Algorithm 2
 * @param host_context Output parameter for determined context
 * @param confidence Output parameter for confidence degree
 */
void identify_entity(host_context_t* host_context, float* confidence);

/**
 * @brief Broadcast self advertisement
 * Uses existing BLE infrastructure to advertise presence
 */
void broadcast_self(void);

/**
 * @brief Send WuR ON signal to wake up neighbors
 * Uses SX1261 wake-up radio to send activation signal
 */
void send_wur_on(void);

/**
 * @brief Start scanning for neighbors
 * Initiates BLE scanning to discover nearby devices
 */
void start_scanning(void);

/**
 * @brief Add discovered neighbor to the list
 * Updates the neighbor table with new discoveries
 */
void add_neighbor(void);

/**
 * @brief Algorithm 1: Relevance-Based Wake-Up and Self-Advertisement
 * 
 * Implements the complete Algorithm 1 from the paper:
 * - Listens for WuR ON signals
 * - Checks relevance based on P(H | H') probability
 * - Performs backoff timing and self-advertisement
 * - Updates eligibility and WuA state
 */
void algorithm1_relevance_based_wakeup(void);

/**
 * @brief Algorithm 2: Neighborhood Discovery and Self-Advertisement
 * 
 * Implements the complete Algorithm 2 from the paper:
 * - Performs neighbor discovery with k-neighbor target
 * - Determines host context using entity identification
 * - Conducts self-advertisement while continuing discovery
 * - Returns discovered neighbor table
 * 
 * @return Enhanced neighbor table with discovered devices
 */
enhanced_neighbour_table_t algorithm2_neighborhood_discovery(void);

/**
 * @brief Enhanced main Wubble function
 * 
 * Orchestrates both algorithms in the proper sequence:
 * 1. Runs Algorithm 2 for neighborhood discovery
 * 2. Processes any incoming WuR signals
 * 3. Runs Algorithm 1 for relevance-based responses
 * 4. Manages state transitions and deep sleep modes
 */
void IRAM_ATTR wubble(void);

/**
 * @brief Initialize algorithm state and timers
 * Sets up all necessary components for algorithm execution
 */
void initialize_algorithms(void);

/**
 * @brief Get current algorithm state
 * @return Pointer to current algorithm state structure
 */
algorithm_state_t* get_algorithm_state(void);

/**
 * @brief Print current algorithm statistics
 * Debug function to display current state and metrics
 */
void print_algorithm_stats(void);

/**
 * @brief Reset algorithm state
 * Resets all counters and state variables to initial values
 */
void reset_algorithm_state(void);

/**
 * @brief Simulate WuR signal reception
 * Testing function to simulate receiving WuR signals
 * @param h_prime Previous host context of sender
 */
void simulate_wur_reception(host_context_t h_prime);

/**
 * @brief Convert host context to string
 * @param context Host context enumeration value
 * @return String representation of the context
 */
const char* host_context_to_string(host_context_t context);

#ifdef __cplusplus
} 
#endif 

#endif /* _WUBBLE_ENHANCED_H */