// Enhanced wubble.c with Algorithm 1 & 2 implementation
#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>
#include "esp_attr.h"
#include "sx1261_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "Nimble_adv.h"
#include <string.h>

static const char* TAG = "TDV";

// Host context definitions
typedef enum {
    H1 = 1,
    H2 = 2,
    H3 = 3,
    H4 = 4,
    H5 = 5,
    H_UNKNOWN = 0
} host_context_t;

// Hardcoded probability table P(H | H')
typedef struct {
    host_context_t h_prime;  // Previous host context
    float probability;       // P(H | H')
} probability_entry_t;

// Probability table - P(H | H') for current context
static const probability_entry_t probability_table[] = {
    {H1, 0.5f},
    {H2, 0.2f},
    {H3, 0.8f},
    {H4, 1.0f},
    {H5, 0.6f}
};

// Algorithm parameters and state variables
typedef struct {
    host_context_t current_host_context;    // H
    host_context_t previous_host_context;   // H'
    float confidence_degree;                 // conf
    uint32_t time_in_host_context;          // ΔHost (in ms)
    uint32_t time_in_sleep;                  // ΔSleep (in ms)
    uint32_t num_advertisements;             // NAdv
    uint32_t num_wur_on_received;           // NWUR-on
    bool eligibility;                        // eligibility flag
    uint32_t wur_authority_state;           // WuA
    uint32_t wur_signal_state;              // WuS
    bool wur_on_received;                   // WuR On flag
    bool wur_off_received;                  // WuR Off flag
} algorithm_state_t;

// Thresholds
#define TH_ELIGIBILITY 0.3f
#define TH_RELEVANCE 0.4f
#define REF_HOST 5000      // 5 seconds in ms
#define MAX_BACKOFF 10000  // 10 seconds
#define MAX1_SCAN 5000     // 5 seconds
#define MAX2_BROADCAST 3000 // 3 seconds
#define K_NEIGHBORS 5

static algorithm_state_t alg_state = {
    .current_host_context = H_UNKNOWN,
    .previous_host_context = H_UNKNOWN,
    .confidence_degree = 0.0f,
    .time_in_host_context = 0,
    .time_in_sleep = 0,
    .num_advertisements = 0,
    .num_wur_on_received = 0,
    .eligibility = true,
    .wur_authority_state = 1,
    .wur_signal_state = 1,
    .wur_on_received = false,
    .wur_off_received = false
};

static esp_timer_handle_t backoff_timer = NULL;
static esp_timer_handle_t scan_timer = NULL;
static esp_timer_handle_t broadcast_timer = NULL;
static esp_timer_handle_t host_context_timer = NULL;

// Function to get probability P(H | H') from the hardcoded table
float get_probability(host_context_t h_prime) {
    for (int i = 0; i < sizeof(probability_table)/sizeof(probability_entry_t); i++) {
        if (probability_table[i].h_prime == h_prime) {
            return probability_table[i].probability;
        }
    }
    return 0.0f; // Default if not found
}

// Calculate backoff timer value using Equation 3 (simplified)
uint32_t calculate_backoff_time(void) {
    // Simple implementation: random value scaled by probability
    float prob = get_probability(alg_state.previous_host_context);
    uint32_t base_time = (rand() % MAX_BACKOFF);
    
    // Higher probability = shorter backoff time
    return (uint32_t)(base_time * (1.0f - prob + 0.1f));
}

// Update Wake-up Authority (WuA) based on eligibility
void update_wua(uint32_t wus, bool eligibility) {
    if (eligibility) {
        alg_state.wur_authority_state = wus;
    } else {
        alg_state.wur_authority_state = 0; // Not eligible to wake up
    }
    ESP_LOGI(TAG, "WuA updated: %lu, Eligibility: %s", 
             alg_state.wur_authority_state, eligibility ? "true" : "false");
}

// Identify entity and determine host context (stub implementation)
void identify_entity(host_context_t* host_context, float* confidence) {
    neighbour_table_t table = get_neighbour_table();
    
    // Simple heuristic based on number of neighbors
    if (table.size >= 4) {
        *host_context = H4; // High density area
        *confidence = 0.9f;
    } else if (table.size >= 3) {
        *host_context = H3; // Medium-high density
        *confidence = 0.7f;
    } else if (table.size >= 2) {
        *host_context = H2; // Medium density
        *confidence = 0.5f;
    } else if (table.size >= 1) {
        *host_context = H1; // Low density
        *confidence = 0.3f;
    } else {
        *host_context = H5; // Isolated/mobile
        *confidence = 0.8f;
    }
    
    ESP_LOGI(TAG, "Entity identified: H%d, Confidence: %.2f, Neighbors: %zu", 
             *host_context, *confidence, table.size);
}

// Broadcast self advertisement
void broadcast_self(void) {
    extern int type_of_beacon;
    type_of_beacon = IOTBEACON;
    
    // Start advertising for a short duration
    ble_server_start();
    vTaskDelay(pdMS_TO_TICKS(500)); // Advertise for 500ms
    ble_server_stop();
    
    ESP_LOGI(TAG, "Self-advertisement broadcast complete");
}

// Send WuR ON signal
void send_wur_on(void) {
    ESP_LOGI(TAG, "Sending WuR ON signal");
    send_WUR(); // Use existing WuR function
}

// Start scanning for neighbors
void start_scanning(void) {
    ESP_LOGI(TAG, "Starting BLE scan for neighbors");
    BLE_scan();
}

// Add neighbor to the list
void add_neighbor(void) {
    // This is handled by the existing BLE scanning mechanism
    // The neighbor table is automatically populated
    ESP_LOGD(TAG, "Neighbor discovery in progress");
}

// Timer callbacks
void IRAM_ATTR backoff_timer_callback(void* arg) {
    alg_state.wur_off_received = false; // Reset for new cycle
    ESP_LOGI(TAG, "Backoff timer expired");
}

void IRAM_ATTR scan_timer_callback(void* arg) {
    scanner_stop();
    ESP_LOGI(TAG, "Scan timer expired");
}

void IRAM_ATTR broadcast_timer_callback(void* arg) {
    ESP_LOGI(TAG, "Broadcast timer expired");
}

void IRAM_ATTR host_context_timer_callback(void* arg) {
    alg_state.time_in_host_context += 1000; // Increment by 1 second
}

// Algorithm 1: Relevance-Based Wake-Up and Self-Advertisement
void algorithm1_relevance_based_wakeup(void) {
    ESP_LOGI(TAG, "=== Algorithm 1: Relevance-Based Wake-Up ===");
    
    // Step 1: Listen for WuR (runs in parallel - simulated here)
    Listen_WUR();
    
    // Simulate receiving WuR ON signal
    if (alg_state.wur_on_received && alg_state.wur_authority_state == alg_state.wur_signal_state) {
        alg_state.num_wur_on_received++;
        ESP_LOGI(TAG, "WuR ON received. Count: %lu", alg_state.num_wur_on_received);
        
        // Check Relevance
        if (alg_state.previous_host_context != H_UNKNOWN) {
            float probability = get_probability(alg_state.previous_host_context);
            ESP_LOGI(TAG, "Relevance check: P(H|H') = %.2f, Threshold = %.2f", 
                     probability, TH_RELEVANCE);
            
            if (probability > TH_RELEVANCE) {
                // Calculate backoff timer
                uint32_t backoff_time = calculate_backoff_time();
                ESP_LOGI(TAG, "Relevance satisfied. Backoff time: %lu ms", backoff_time);
                
                // Create and start backoff timer
                esp_timer_create_args_t backoff_args = {
                    .callback = &backoff_timer_callback,
                    .name = "backoff_timer"
                };
                esp_timer_create(&backoff_args, &backoff_timer);
                esp_timer_start_once(backoff_timer, backoff_time * 1000);
                
                // Wait for backoff timer or WuR OFF
                TickType_t start_time = xTaskGetTickCount();
                while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(backoff_time) && 
                       !alg_state.wur_off_received) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                
                if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(backoff_time)) {
                    // Backoff timer expired - wake up and advertise
                    alg_state.num_advertisements++;
                    ESP_LOGI(TAG, "Waking up for self-advertisement #%lu", 
                             alg_state.num_advertisements);
                    
                    while (!alg_state.wur_off_received) {
                        broadcast_self();
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                }
                
                esp_timer_delete(backoff_timer);
            }
        } else {
            // Relevance independent of requesting node
            if (alg_state.time_in_host_context >= REF_HOST) {
                ESP_LOGI(TAG, "Time-based relevance satisfied (ΔHost: %lu >= %d)", 
                         alg_state.time_in_host_context, REF_HOST);
                // Same logic as above (lines 8-16 in algorithm)
            }
        }
        
        // Update Eligibility
        float eligibility_ratio = (float)alg_state.num_advertisements / alg_state.num_wur_on_received;
        alg_state.eligibility = (eligibility_ratio < TH_ELIGIBILITY);
        ESP_LOGI(TAG, "Eligibility updated: %.3f < %.2f = %s", 
                 eligibility_ratio, TH_ELIGIBILITY, alg_state.eligibility ? "true" : "false");
        
        update_wua(alg_state.wur_signal_state, alg_state.eligibility);
    }
    
    // Reset sleep time and enter deep sleep
    alg_state.time_in_sleep = 0;
    ESP_LOGI(TAG, "Entering deep sleep mode");
    // Note: In real implementation, this would trigger deep sleep
}

// Algorithm 2: Neighborhood Discovery and Self-Advertisement
neighbour_table_t algorithm2_neighborhood_discovery(void) {
    ESP_LOGI(TAG, "=== Algorithm 2: Neighborhood Discovery ===");
    
    neighbour_table_t A = {0}; // Initialize empty neighbor array
    host_context_t R = H_UNKNOWN;
    float conf = 0.0f;
    
    // Step 1: Listen for WuR (parallel)
    Listen_WUR();
    
    // Step 2: Backoff timer
    uint32_t backoff_time = rand() % MAX_BACKOFF;
    ESP_LOGI(TAG, "Backoff timer: %lu ms", backoff_time);
    
    TickType_t start_time = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(backoff_time) && 
           !alg_state.wur_on_received) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(backoff_time)) {
        ESP_LOGI(TAG, "Backoff expired - initiating discovery");
        
        // Step 3: Wake-up neighbors
        send_wur_on();
        
        // Step 4: Neighborhood Discovery
        int i = 0;
        uint32_t scan_time = rand() % MAX1_SCAN;
        ESP_LOGI(TAG, "Starting neighbor discovery, scan time: %lu ms", scan_time);
        
        start_scanning();
        
        start_time = xTaskGetTickCount();
        while (i < K_NEIGHBORS && 
               (xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(scan_time)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            
            // Check if new neighbors found
            neighbour_table_t current_table = get_neighbour_table();
            if (current_table.size > i) {
                i = current_table.size;
                ESP_LOGI(TAG, "Discovered %d neighbors so far", i);
            }
        }
        
        scanner_stop();
        A = get_neighbour_table();
        ESP_LOGI(TAG, "Initial discovery complete. Found %zu neighbors", A.size);
        
        // Step 5: Determine host context H
        identify_entity(&R, &conf);
        alg_state.current_host_context = R;
        alg_state.confidence_degree = conf;
        
        // Step 6: Self-advertise and continue discovery if needed
        uint32_t broadcast_time = rand() % MAX2_BROADCAST;
        ESP_LOGI(TAG, "Starting self-advertisement phase: %lu ms", broadcast_time);
        
        start_time = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(broadcast_time)) {
            broadcast_self();
            
            // Continue neighbor discovery if not enough neighbors
            if (i < K_NEIGHBORS) {
                uint32_t additional_scan = rand() % MAX1_SCAN;
                start_scanning();
                
                TickType_t scan_start = xTaskGetTickCount();
                while (i < K_NEIGHBORS && 
                       (xTaskGetTickCount() - scan_start) < pdMS_TO_TICKS(additional_scan)) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    
                    neighbour_table_t updated_table = get_neighbour_table();
                    if (updated_table.size > i) {
                        i = updated_table.size;
                        ESP_LOGI(TAG, "Total neighbors discovered: %d", i);
                    }
                }
                scanner_stop();
            }
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // Final neighbor table
        A = get_neighbour_table();
        
        // Final host context determination if not done
        if (R == H_UNKNOWN) {
            identify_entity(&R, &conf);
            alg_state.current_host_context = R;
            alg_state.confidence_degree = conf;
        }
        
        // Update WuA
        update_wua(alg_state.wur_signal_state, conf > 0.5f);
        
        ESP_LOGI(TAG, "Discovery complete: %zu neighbors, Host: H%d, Confidence: %.2f", 
                 A.size, R, conf);
    }
    
    return A;
}

// Enhanced main wubble function
void IRAM_ATTR wubble(void) {
    ESP_LOGI(TAG, "=== Enhanced Wubble with Algorithms 1 & 2 ===");
    
    // Initialize SPI bus
    spi_bus_config_t spi_bus_config = {
        .sclk_io_num = PIN_SPI_SCLK,
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    
    esp_err_t ret = spi_bus_initialize(USED_SPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    
    // Initialize BLE
    ble_init();
    
    // Start host context timer (1 second intervals)
    esp_timer_create_args_t host_timer_args = {
        .callback = &host_context_timer_callback,
        .name = "host_context_timer"
    };
    esp_timer_create(&host_timer_args, &host_context_timer);
    esp_timer_start_periodic(host_context_timer, 1000000); // 1 second
    
    while (1) {
        ESP_LOGI(TAG, "=== Wubble Cycle Start ===");
        ESP_LOGI(TAG, "Current state - Host: H%d, Confidence: %.2f, Ads: %lu, WuR: %lu", 
                 alg_state.current_host_context, alg_state.confidence_degree,
                 alg_state.num_advertisements, alg_state.num_wur_on_received);
        
        // Run Algorithm 2: Neighborhood Discovery
        neighbour_table_t neighbors = algorithm2_neighborhood_discovery();
        
        // Simulate some time passing and potential WuR signal
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Simulate receiving a WuR signal (for demonstration)
        if (rand() % 3 == 0) { // 33% chance
            alg_state.wur_on_received = true;
            alg_state.previous_host_context = (host_context_t)((rand() % 5) + 1);
            ESP_LOGI(TAG, "Simulated WuR ON from H%d", alg_state.previous_host_context);
            
            // Run Algorithm 1: Relevance-Based Wake-Up
            algorithm1_relevance_based_wakeup();
            
            alg_state.wur_on_received = false;
        }
        
        // Wait before next cycle
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}