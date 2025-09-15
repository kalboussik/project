#include <stdio.h>
#include "wur_sx1261.h"
#include "sx126x.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "../../config/config.h"


#define TAG "WUR"



void Sx1261_Standby(context_t *context)
{
	sx126x_set_standby(&context, SX126X_STANDBY_CFG_XOSC);
}

void Sx1261_ContinousWave(context_t *context)
{
	sx126x_set_tx_cw(&context);
}

void Sx1261_InfinitePreamble(context_t *context)
{
	sx126x_set_tx_infinite_preamble(&context);
}

void Sx1261_WurMode(context_t *context, uint32_t sleepTime, uint32_t rxTime)
{
	sx126x_set_rx_duty_cycle(&context, rxTime, sleepTime);
}

void Sx1261_RstWurFlag(context_t *context)
{
	sx126x_clear_irq_status(&context, SX126X_IRQ_SYNC_WORD_VALID);
}

void Sx1261_InitParam_WUR(context_t *context)
{
	context->sync_word[0] = 0x7e;
	context->sync_word[1] = 0x7e;
	context->sync_word[2] = 0x7e;
	context->sync_word[3] = 0x7e;
	context->sync_word[4] = 0x7e;
	context->sync_word[5] = 0x7e;
	context->sync_word[6] = 0x7e;
	context->sync_word[7] = 0x7e;

	context->frequency = 868000000;
	context->power = POWER;

	context->mode_parameters.br_in_bps = BR_IN_BPS;
	context->mode_parameters.fdev_in_hz = FDEV_IN_HZ;
	context->mode_parameters.pulse_shape = SX126X_GFSK_PULSE_SHAPE_OFF;
	context->mode_parameters.bw_dsb_param = BW_DSB_PARAM; // must be > br_in_bps + 2*fdev_in_hz

	context->pkt_parameters.preamble_len_in_bits = PREAMBLE_LENGTH_IN_BIT; //!< Preamble length in bits
	context->pkt_parameters.preamble_detector = PREAMBLE_DETECTION_LENGTH; //!< Preamble detection length /!\ 24bits max for 1ms RX time
	context->pkt_parameters.sync_word_len_in_bits = 8*SYNCWORD_LENGTH;	//!< Sync word length in bits (multiply by 8 because SYNCWORD_LENGTH is in bytes )
	context->pkt_parameters.address_filtering = SX126X_GFSK_ADDRESS_FILTERING_DISABLE;	//!< Address filtering configuration
	context->pkt_parameters.header_type = SX126X_GFSK_PKT_FIX_LEN;	//!< Header type
	context->pkt_parameters.pld_len_in_bytes = 10;	//!< Payload length in bytes
	context->pkt_parameters.crc_type = SX126X_GFSK_CRC_OFF;	//!< CRC type configuration
	context->pkt_parameters.dc_free = SX126X_GFSK_DC_FREE_OFF;	//!< Whitening configuration
}

void set_sync_word(context_t *context, const uint8_t* sync_word, const uint8_t sync_word_length)
{
	sx126x_set_gfsk_sync_word(context, sync_word, sync_word_length); 
}

void Sx1261_InitFonction_WUR(context_t *context)
{
	//all this steps are described in the datasheet p98
	(void) sx126x_set_pkt_type(context, SX126X_PKT_TYPE_GFSK);
	(void) sx126x_set_tx_params(context, context->power, RAMP_TIME);
	(void) sx126x_set_gfsk_mod_params(context, &(context->mode_parameters));
	(void) sx126x_set_gfsk_pkt_params(context, &(context->pkt_parameters));
	(void) sx126x_set_rf_freq(context, context->frequency);
	(void) sx126x_set_gfsk_sync_word(context, context->sync_word, context->pkt_parameters.sync_word_len_in_bits/8); //
	(void) sx126x_set_reg_mode(context, SX126X_REG_MODE_DCDC); // SX126X_REG_MODE_DCDC
	(void) sx126x_set_rx_tx_fallback_mode(context, SX126X_FALLBACK_STDBY_XOSC); // Return to standby xosc after RX or TX mode.
	(void) sx126x_set_dio_irq_params(context, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_SYNC_WORD_VALID, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
	(void) sx126x_set_buffer_base_address(context, 0x00, 0x00);
	(void) sx126x_set_standby(context, SX126X_STANDBY_CFG_XOSC);
}

void Sx1261_Send_WuR_Signal( context_t *context, uint8_t* data, uint8_t length )
{
	uint8_t n_length = length;
	if (n_length > 255) {
		n_length = 255;
	}
	n_length = 0;
	(void) sx126x_set_dio_irq_params(context, SX126X_IRQ_ALL, SX126X_IRQ_ALL, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    sx126x_clear_irq_status(context, SX126X_IRQ_ALL);
	sx126x_write_buffer( context, 0,  data, length );

	delay(10);
	sx126x_set_tx(context, TX_TIME_MS);
}

void run_TX_WUR(void *pvParameters)
{
    uint64_t exec_time = 0;
    uint8_t compteur = 0;
    uint8_t data[256] = {0};
    while (1)
    {
        //Sx1261_InitFonction(sx_parameters);
        ESP_LOGI(TAG, "Sx1261_SendSignal_WUR");
        Sx1261_Send_WuR_Signal(pvParameters,data, 0);
        exec_time = (esp_timer_get_time()%3000000);

        vTaskDelay(pdMS_TO_TICKS(3000-exec_time/1000));
        compteur = (compteur + 1) %3;
	}
}

void run_RX_WUR(void *pvParameters)
{
    int nb_reveils = 0;
    uint64_t init_time = esp_timer_get_time();; //heure au début à l'initialisation
    uint64_t current_time = 0; //heure courante
    sx126x_irq_mask_t sx126x_irq_mask;
    while (1)
    {
        //we check irq status (note : it has to be coded with interrupt esp)
        sx126x_get_irq_status(pvParameters,&sx126x_irq_mask);
        if (sx126x_irq_mask & SX126X_IRQ_SYNC_WORD_VALID)
        {
			ESP_LOGI(TAG, "inside if: Level Busy: %d, level nss: %d", gpio_get_level(CONFIG_BUSY_GPIO), gpio_get_level(CONFIG_NSS_GPIO));
            nb_reveils ++;
			current_time = esp_timer_get_time();
            ESP_LOGW(TAG, "irq : %d", sx126x_irq_mask);
			ESP_LOGI(TAG, "reveil, nb_reveils : %d ;  temps écoulé : %lld s ",
			nb_reveils, (current_time - init_time)/1000000);
			
        	vTaskDelay(pdMS_TO_TICKS(1000));
			//on repasse en sommeil
			ESP_LOGI(TAG, "Sx1261_WurMode");
            Sx1261_WurMode(pvParameters, 999, 1);
			SetDioIrqParams(SX126X_IRQ_ALL,   //all interrupts enabled
					SX126X_IRQ_ALL,  //interrupts on DIO1
					SX126X_IRQ_NONE,  //interrupts on DIO2
					SX126X_IRQ_NONE); //interrupts on DIO3
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void send_WUR(){
    context_t sx_parameters;
    uint8_t data[256] = {0};
    sx1261_init();
    sx126x_reset(&sx_parameters);
    ESP_LOGI(TAG, "initparam");
    Sx1261_InitParam_WUR(&sx_parameters);
    ESP_LOGI(TAG, "initfonction");
    Sx1261_InitFonction_WUR(&sx_parameters);
    Sx1261_Send_WuR_Signal(&sx_parameters, data, 0);
    return;
}


void Listen_WUR(){
    context_t sx_parameters;
    uint8_t data[256] = {0};
    sx1261_init();
    sx126x_reset(&sx_parameters);
    ESP_LOGI(TAG, "initparam");
    Sx1261_InitParam_WUR(&sx_parameters);
    ESP_LOGI(TAG, "initfonction");
    Sx1261_InitFonction_WUR(&sx_parameters);
    run_RX_WUR(&sx_parameters);
    return;
}
//----------------------------EOF----------------------------