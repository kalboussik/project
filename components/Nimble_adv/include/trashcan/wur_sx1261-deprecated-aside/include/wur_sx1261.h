#ifndef WUR_SX1261_H_
#define WUR_SX1261_H_

#include <sys/_stdint.h>

#include "sx126x.h"
#include "ra01s.h"
#include "../../../config/config.h"


#define delay(ms) esp_rom_delay_us(ms*1000)

typedef struct context_s
{
	uint32_t frequency; // Center frequency
	uint8_t power; // Power of transmission
	uint8_t sync_word[8];

	sx126x_mod_params_gfsk_t mode_parameters;
	sx126x_pkt_params_gfsk_t pkt_parameters;

	sx126x_mod_params_lora_t mode_lora_parameters;
	sx126x_pkt_params_lora_t pkt_lora_parameters;

} context_t;

/** @brief initialize the radio parameters by storing them in a context
   @param context : pointer to the context that will contain radio parameters 
*/
void Sx1261_InitParam_WUR(context_t *context);


/** @brief calls the different commnands that initialize the radio parameters in the sx1261 board
   @param context : pointer to the context containing the radio parameters 
*/
void Sx1261_InitFonction_WUR(context_t *context);


/**
 * @brief Configure the sync word used in GFSK packet
 * 
 * @remark The synchronization word must be the same for transmitter and receiver, 
 * otherwise the alarm will not wake up 
 *
 * @param  context Chip implementation context
 * @param  sync_word Buffer holding the sync word to be configured
 * @param  sync_word_len Sync word length in byte
 *
 */
void set_sync_word(context_t *context, const uint8_t* sync_word, const uint8_t sync_word_length);

/**
 * @brief Set the chip in stand-by mode
 *
 * @param   context Chip implementation context
 * @param   cfg Stand-by mode configuration
 *
 * @returns Operation status
 */
void Sx1261_Standby(context_t *context);

/**
 * @brief Set the chip in Tx continuous wave (RF tone).
 * 
 * @remark just for testing that the card is working
 *
 * @param  context Chip implementation context
 *
 */
void Sx1261_ContinousWave(context_t *context);

/**
 * @brief Set the chip in Tx infinite preamble (modulated signal).
 * 
 * @remark just for testing that the card is working
 *
 * @param  context Chip implementation context
 *
 */
void Sx1261_InfinitePreamble(context_t *context);

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @param  context Chip implementation context
 * @param  rx_time_in_ms The timeout of Rx period - in millisecond
 * @param  sleep_time_in_ms The length of sleep period - in millisecond
 *
 */
void Sx1261_WurMode(context_t *context, uint32_t sleepTime, uint32_t rxTime);

/**
 * @brief Clear wur flag. to be called after receiving a WuB
 *
 * @param  context Chip implementation context
 *
 */
void Sx1261_RstWurFlag(context_t *context);

/**
 * @brief sends a frame that wakes up those who receive it
 *
 * @param  context Chip implementation context
 * @param  data data to be send with the frame
 * @param  length length of the data
 *
 */
void Sx1261_Send_WuR_Signal(context_t *context, uint8_t* data, uint8_t length );

/**
 * @brief simple task that send a WuB each 3 secondes
 * 
 */
void run_TX_WUR(void *pvParameters);

/**
 * @brief simple task that print the number of time that the iot woke up when he wakes up
 * 
 */
void run_RX_WUR(void *pvParameters);

/**
* @brief Send the Wake up radio signal
**/
void send_WUR();

/**
 * @brief Listen for the Wake up radio signal
 */
void Listen_WUR();


#endif /* WUR_SX1261_H_ */