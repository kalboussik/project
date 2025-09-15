#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "sx1261_driver.h"
#include "Nimble_adv.h"
#include "wubble.h"

#define MAX_BACKOFF_MS 1000
#define MAX_SCAN_MS 500
#define MAX_BROADCAST_MS 300
#define MAX_DELTA 10

// ----------------------
// Probability table (hardcoded)
// For Algorithm 1 relevance calculation
// ----------------------
typedef struct {
    char host[4];
    float probability;
} HostProbability;

HostProbability probTable[] = {
    {"H1", 0.5},
    {"H2", 0.2},
    {"H3", 0.8},
    {"H4", 1.0},
    {"H5", 0.6},
};

// ----------------------
// Globals
// ----------------------
Neighbor neighbors[MAX_NEIGHBORS];
int neighbor_count = 0;

int NAdv = 0;     // Number of self-advertisements
int NWuR_on = 0;  // Number of wake-up events
int WuA = 0;      // Wake-up eligibility
int WuS = 1;      // Wake-up status
const float Threlevance = 0.5;
const float Theligibility = 0.3;

// ----------------------
// Random delta timer (used for backoff/delta sleep)
// ----------------------
int RandomDelta() { return rand() % MAX_DELTA + 1; }

// ----------------------
// Check if node woke up from external GPIO (EXT0/EXT1)
// ----------------------
bool WUR_IsOn() {
    esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
    return (reason == ESP_SLEEP_WAKEUP_EXT0 || reason == ESP_SLEEP_WAKEUP_EXT1);
}

// ----------------------
// Configure GPIO 6 for external wake-up (EXT0)
// ----------------------
void ConfigureExternalWakeup(void) {
    esp_rom_gpio_pad_select_gpio(WUR_GPIO);
    gpio_set_direction(WUR_GPIO, GPIO_MODE_INPUT);
    esp_sleep_enable_ext0_wakeup(WUR_GPIO, 1);  // wake on HIGH
}

// ----------------------
// SX1261 radio control
// ----------------------
void RadioSleep(void) { Listen_WUR(); }

// ----------------------
// Enter deep sleep until WUR triggers wakeup
// ----------------------
void DeepSleep(void) {
    RadioSleep();            // Algorithm 1, line 14: Radio module sleep
    ConfigureExternalWakeup();
    printf("Entering deep sleep...\n");
    fflush(stdout);
    esp_deep_sleep_start();  // Deep sleep until external wakeup (Algorithm 1, line 21)
}

// ----------------------
// Return probability for host H (used for relevance in Algorithm 1)
// ----------------------
float getProbability(const char* H) {
    for (int i = 0; i < sizeof(probTable)/sizeof(probTable[0]); i++) {
        if (strcmp(probTable[i].host, H) == 0) return probTable[i].probability;
    }
    return 0.0;
}

// ----------------------
// Identify host H with max probability (Algorithm 2, Step 2)
// ----------------------
void IdentifyEntity(char* H_out, float* conf_out) {
    float maxProb = 0.0;
    const char* bestHost = NULL;
    for (int i = 0; i < sizeof(probTable)/sizeof(probTable[0]); i++) {
        if (probTable[i].probability > maxProb) {
            maxProb = probTable[i].probability;
            bestHost = probTable[i].host;
        }
    }
    if (bestHost) {
        strcpy(H_out, bestHost);
        *conf_out = maxProb;
    } else {
        strcpy(H_out, "NULL");
        *conf_out = 0.0;
    }
}

// ----------------------
// Algorithm 1: Relevance-Based Wake-Up
// ----------------------
void RelevanceBasedWakeup(const char* Hprime, const char* Hcurrent) {
    // Algorithm 1, line 1: Listen WuR
    if (!WUR_IsOn()) return;

    // Algorithm 1, line 2: Check WuA == WuS
    if (WuA != WuS) return;

    NWuR_on++;  // Algorithm 1, line 3: Increment NWuR_on

    // Algorithm 1, line 5: Calculate relevance
    float relevance = 0.0;
    if (Hprime != NULL) {
        relevance = getProbability(Hprime); // P(H | H′)
    }

    // Algorithm 1, lines 6-16: Check relevance or independent ΔHost
    if ((Hprime && relevance > Threlevance) || (!Hprime && RandomDelta() > 5)) {
        int backoff = rand() % MAX_BACKOFF_MS + 1; // Algorithm 1, line 7: Backoff timer
        vTaskDelay(pdMS_TO_TICKS(backoff));

        // Algorithm 1, lines 8-14: Broadcast while WuR active
        while (WUR_IsOn()) {
            send_BLE_ID();  // BroadcastSelf
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        NAdv++;  // Algorithm 1, line 11
    }

    // Algorithm 1, line 18-19: Update eligibility and WuA
    bool eligibility = (NWuR_on > 0) ? ((float)NAdv / NWuR_on < Theligibility) : true;
    WuA = eligibility ? 1 : 0;

    // Algorithm 1, line 20: Reset delta sleep
    vTaskDelay(pdMS_TO_TICKS(RandomDelta() * 50));

    // Algorithm 1, line 21: Go to deep sleep
    DeepSleep();
}

// ----------------------
// Algorithm 2: Neighborhood Discovery & Self-Advertisement
// ----------------------
void NeighborhoodDiscovery(int k) {
    neighbor_count = 0;

    // Algorithm 2, line 5-7: Random backoff before discovery
    int backoff = rand() % MAX_BACKOFF_MS + 1;
    vTaskDelay(pdMS_TO_TICKS(backoff));

    // Algorithm 2, Step 1: Neighborhood Discovery
    BLE_scan();
    int timer1 = rand() % MAX_SCAN_MS + 1;
    int elapsed = 0;

    while (elapsed < timer1 && neighbor_count < k) {
        neighbour_table_t table = get_neighbour_table();
        for (size_t i = 0; i < table.size && neighbor_count < k; i++) {
            memcpy(neighbors[neighbor_count].id, table.neighbours[i].neighbour, 4);
            neighbors[neighbor_count].rssi = table.neighbours[i].states_neighbour;
            neighbors[neighbor_count].last_seen = xTaskGetTickCount();
            neighbor_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        elapsed += 50;
    }
    scanner_stop();

    // Algorithm 2, Step 2: Identify host H and confidence
    char H[4];
    float conf = 0;
    IdentifyEntity(H, &conf);

    // Algorithm 2, Step 3: Self-advertise
    int timer3 = rand() % MAX_BROADCAST_MS + 1;
    elapsed = 0;
    while (elapsed < timer3) {
        send_BLE_ID();
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }

    // Algorithm 2, line 30: Update WuA based on confidence
    WuA = conf > Threlevance ? 1 : 0;
}

// ----------------------
// Entry point Wubble
// ----------------------
void Wubble(void) {

    // BLE initialization
    ble_init();
    ble_server_start();

    srand(xTaskGetTickCount());

    while (1) {
        // Algorithm 2: Discover neighbors
        NeighborhoodDiscovery(5);

        // Algorithm 1: Handle relevance-based wakeup and self-advertisement
        RelevanceBasedWakeup("H3", "H1");

        // Print discovered neighbors
        printf("Neighbors (%d):\n", neighbor_count);
        for (int i = 0; i < neighbor_count; i++) {
            printf("  ID=%02X%02X%02X%02X RSSI=%d\n",
                neighbors[i].id[0], neighbors[i].id[1],
                neighbors[i].id[2], neighbors[i].id[3],
                neighbors[i].rssi);
        }

        // Enter deep sleep until external WUR triggers wakeup
        DeepSleep();
    }
}
