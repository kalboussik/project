#ifndef _WUBBLE_H 
#define _WUBBLE_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include "../../config/config.h"

/**
 * @brief Initializes the BLE stack and starts advertising.
 */
void make_tdv(void);

/**
 * @brief Starts the BLE server and begins advertising.
 */
void send_tdv(void);

/**
 * @brief Main function for the Wubble application.
 */
void IRAM_ATTR wubble(void);




bool WurOn(void); // déjà défini
bool WurOff(void); // déjà défini

float compute_probability(int H, int H_prime);
int compute_backoff_timer(void);
void wake_up_radio_module(void);
void put_radio_to_sleep(void);
void broadcast_self(void);
void reset_Delta_sleep(void);
void DeepSleep(void);
void Sleep(void);

#endif //_WUBBLE_H 