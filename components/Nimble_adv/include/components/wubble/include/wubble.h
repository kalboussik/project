#ifndef WUBBLE_H
#define WUBBLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_sleep.h"

#define MAX_NEIGHBORS 20
#define WUR_GPIO 6  // External wake-up GPIO

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------
// Neighbor structure
// ----------------------
typedef struct {
    char id[16];     // BLE neighbor ID
    int rssi;        // RSSI value
    int last_seen;   // Last seen tick
} Neighbor;

// ----------------------
// Globals
// ----------------------
extern Neighbor neighbors[MAX_NEIGHBORS];
extern int neighbor_count;

extern int NAdv;     // Number of self-advertisements
extern int NWuR_on;  // Number of external wake-up events
extern int WuA;      // Wake-up eligibility flag
extern int WuS;      // Wake-up status flag

extern const float Threlevance;    // Relevance threshold
extern const float Theligibility;  // Eligibility threshold

// ----------------------
// Entry point
// ----------------------
void Wubble(void);

// ----------------------
// Algorithm 1: Relevance-Based Wake-Up
// Listens for external WUR signal (GPIO 6)
// Increments NWuR_on when WuR received
// Checks relevance (P(H | H′)) vs Threlevance
// Backoff timer, broadcast BLE if relevant
// Updates eligibility: WuA = (NAdv / NWuR_on < Theligibility)
// Resets delta timers and enters deep sleep
// ----------------------
void RelevanceBasedWakeup(const char* Hprime, const char* Hcurrent);

// ----------------------
// Algorithm 2: Neighborhood Discovery & Self-Advertisement
// Performs BLE-based neighbor discovery (up to k neighbors)
// Random backoff before discovery
// Step 1: Scan BLE neighbors
// Step 2: Identify host H and confidence conf
// Step 3: Self-advertise with send_BLE_ID() for random period
// Updates WuA based on confidence
// ----------------------
void NeighborhoodDiscovery(int k);

// ----------------------
// Probability helpers
// ----------------------
float getProbability(const char* H);  // Returns P(H | H′) from table
void IdentifyEntity(char* H_out, float* conf_out); // Choose H with max probability

// ----------------------
// Timer helpers
// ----------------------
int RandomDelta(void);  // Random delta used for backoff or delta sleep

// ----------------------
// Wake-up helpers
// ----------------------
bool WUR_IsOn(void);  // Returns true if node woke from external GPIO

// ----------------------
// ESP32-S3 Sleep / Radio Functions
// ----------------------
void ConfigureExternalWakeup(void); // Configure GPIO 6 as EXT0 wakeup
void DeepSleep(void);               // Enter deep sleep until external wakeup
void RadioSleep(void);              // Put SX1261 radio to sleep
void RadioWakeup(void);             // Wake up SX1261 radio

#ifdef __cplusplus
}
#endif

#endif /* WUBBLE_H */
