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


#endif //_WUBBLE_H 