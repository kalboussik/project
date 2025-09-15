#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include <driver/rtc_io.h>
#include "esp_timer.h"
#include "../../config/config.h"
#include <string.h>  // memcpy

//#include "sx126x.h"
//#include "sx126x_hal.h"
//#include "sx126x_regs.h"

#include "sx1261_driver.h"

#define TAG "driver"

//#include "wur_sx1261.h"
//#include "sx126x.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @file      driver.c
 *
 * @brief     SX126x radio driver implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


static const int SPI_Frequency = 2000000;
static spi_device_handle_t spiHandle_radio;

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

typedef enum gpio_state_e
{
    HAL_GPIO_RESET = 0,
    HAL_GPIO_SET   = 1,
} hal_gpio_state_t;

// Global Stuff
static uint8_t PacketParams[6];
static bool    txActive;
static bool    debugPrint;
static int     SX126x_SPI_SELECT;
static int     SX126x_RESET;
static int     SX126x_BUSY;
static int     SX126x_DIO1;
static int     SX126x_TXEN;
static int     SX126x_RXEN;
static volatile radio_mode_t radio_mode = RADIO_AWAKE;



// Arduino compatible macros
#define delayMicroseconds(us) esp_rom_delay_us(us)
#define delay(ms) esp_rom_delay_us(ms*1000)


void LoRaErrorDefault(int error)
{
	if (debugPrint) {
		ESP_LOGE(TAG, "LoRaErrorDefault=%d", error);
	}
	while (true) {
		vTaskDelay(1);
	}
}

__attribute__ ((weak, alias ("LoRaErrorDefault"))) void LoRaError(int error);


/**
 * @brief Internal frequency of the radio
 */
#define SX126X_XTAL_FREQ 32000000UL

/**
 * @brief Internal frequency of the radio
 */
#define SX126X_RTC_FREQ_IN_HZ 64000UL

/**
 * @brief Scaling factor used to perform fixed-point operations
 */
#define SX126X_PLL_STEP_SHIFT_AMOUNT ( 14 )

/**
 * @brief PLL step - scaled with SX126X_PLL_STEP_SHIFT_AMOUNT
 */
#define SX126X_PLL_STEP_SCALED ( SX126X_XTAL_FREQ >> ( 25 - SX126X_PLL_STEP_SHIFT_AMOUNT ) )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/**
 * Commands Interface
 */
typedef enum sx126x_commands_e
{
    // Operational Modes Functions
    SX126X_SET_SLEEP                  = 0x84,
    SX126X_SET_STANDBY                = 0x80,
    SX126X_SET_FS                     = 0xC1,
    SX126X_SET_TX                     = 0x83,
    SX126X_SET_RX                     = 0x82,
    SX126X_SET_STOP_TIMER_ON_PREAMBLE = 0x9F,
    SX126X_SET_RX_DUTY_CYCLE          = 0x94,
    SX126X_SET_CAD                    = 0xC5,
    SX126X_SET_TX_CONTINUOUS_WAVE     = 0xD1,
    SX126X_SET_TX_INFINITE_PREAMBLE   = 0xD2,
    SX126X_SET_REGULATOR_MODE         = 0x96,
    SX126X_CALIBRATE                  = 0x89,
    SX126X_CALIBRATE_IMAGE            = 0x98,
    SX126X_SET_PA_CFG                 = 0x95,
    SX126X_SET_RX_TX_FALLBACK_MODE    = 0x93,
    // Registers and buffer Access
    SX126X_WRITE_REGISTER = 0x0D,
    SX126X_READ_REGISTER  = 0x1D,
    SX126X_WRITE_BUFFER   = 0x0E,
    SX126X_READ_BUFFER    = 0x1E,
    // DIO and IRQ Control Functions
    SX126X_SET_DIO_IRQ_PARAMS         = 0x08,
    SX126X_GET_IRQ_STATUS             = 0x12,
    SX126X_CLR_IRQ_STATUS             = 0x02,
    SX126X_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D,
    SX126X_SET_DIO3_AS_TCXO_CTRL      = 0x97,
    // RF Modulation and Packet-Related Functions
    SX126X_SET_RF_FREQUENCY          = 0x86,
    SX126X_SET_PKT_TYPE              = 0x8A,
    SX126X_GET_PKT_TYPE              = 0x11,
    SX126X_SET_TX_PARAMS             = 0x8E,
    SX126X_SET_MODULATION_PARAMS     = 0x8B,
    SX126X_SET_PKT_PARAMS            = 0x8C,
    SX126X_SET_CAD_PARAMS            = 0x88,
    SX126X_SET_BUFFER_BASE_ADDRESS   = 0x8F,
    SX126X_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0,
    // Communication Status Information
    SX126X_GET_STATUS           = 0xC0,
    SX126X_GET_RX_BUFFER_STATUS = 0x13,
    SX126X_GET_PKT_STATUS       = 0x14,
    SX126X_GET_RSSI_INST        = 0x15,
    SX126X_GET_STATS            = 0x10,
    SX126X_RESET_STATS          = 0x00,
    // Miscellaneous
    SX126X_GET_DEVICE_ERRORS = 0x17,
    SX126X_CLR_DEVICE_ERRORS = 0x07,
} sx126x_commands_t;

/**
 * Commands Interface buffer sizes
 */
typedef enum sx126x_commands_size_e
{
    // Operational Modes Functions
    SX126X_SIZE_SET_SLEEP                  = 2,
    SX126X_SIZE_SET_STANDBY                = 2,
    SX126X_SIZE_SET_FS                     = 1,
    SX126X_SIZE_SET_TX                     = 4,
    SX126X_SIZE_SET_RX                     = 4,
    SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE = 2,
    SX126X_SIZE_SET_RX_DUTY_CYCLE          = 7,
    SX126X_SIZE_SET_CAD                    = 1,
    SX126X_SIZE_SET_TX_CONTINUOUS_WAVE     = 1,
    SX126X_SIZE_SET_TX_INFINITE_PREAMBLE   = 1,
    SX126X_SIZE_SET_REGULATOR_MODE         = 2,
    SX126X_SIZE_CALIBRATE                  = 2,
    SX126X_SIZE_CALIBRATE_IMAGE            = 3,
    SX126X_SIZE_SET_PA_CFG                 = 5,
    SX126X_SIZE_SET_RX_TX_FALLBACK_MODE    = 2,
    // Registers and buffer Access
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_REGISTER = 3,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_REGISTER = 4,
    // Full size: this value plus buffer size
    SX126X_SIZE_WRITE_BUFFER = 2,
    // Full size: this value plus buffer size
    SX126X_SIZE_READ_BUFFER = 3,
    // DIO and IRQ Control Functions
    SX126X_SIZE_SET_DIO_IRQ_PARAMS         = 9,
    SX126X_SIZE_GET_IRQ_STATUS             = 2,
    SX126X_SIZE_CLR_IRQ_STATUS             = 3,
    SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL = 2,
    SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL      = 5,
    // RF Modulation and Packet-Related Functions
    SX126X_SIZE_SET_RF_FREQUENCY           = 5,
    SX126X_SIZE_SET_PKT_TYPE               = 2,
    SX126X_SIZE_GET_PKT_TYPE               = 2,
    SX126X_SIZE_SET_TX_PARAMS              = 3,
    SX126X_SIZE_SET_MODULATION_PARAMS_GFSK = 9,
    SX126X_SIZE_SET_MODULATION_PARAMS_LORA = 5,
    SX126X_SIZE_SET_PKT_PARAMS_GFSK        = 10,
    SX126X_SIZE_SET_PKT_PARAMS_LORA        = 7,
    SX126X_SIZE_SET_CAD_PARAMS             = 8,
    SX126X_SIZE_SET_BUFFER_BASE_ADDRESS    = 3,
    SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT  = 2,
    // Communication Status Information
    SX126X_SIZE_GET_STATUS           = 1,
    SX126X_SIZE_GET_RX_BUFFER_STATUS = 2,
    SX126X_SIZE_GET_PKT_STATUS       = 2,
    SX126X_SIZE_GET_RSSI_INST        = 2,
    SX126X_SIZE_GET_STATS            = 2,
    SX126X_SIZE_RESET_STATS          = 7,
    // Miscellaneous
    SX126X_SIZE_GET_DEVICE_ERRORS = 2,
    SX126X_SIZE_CLR_DEVICE_ERRORS = 3,
    SX126X_SIZE_MAX_BUFFER        = 255,
    SX126X_SIZE_DUMMY_BYTE        = 1,
} sx126x_commands_size_t;

typedef struct
{
    uint32_t bw;
    uint8_t  param;
} gfsk_bw_t;

gfsk_bw_t gfsk_bw[] = {
    { 4800, SX126X_GFSK_BW_4800 },     { 5800, SX126X_GFSK_BW_5800 },     { 7300, SX126X_GFSK_BW_7300 },
    { 9700, SX126X_GFSK_BW_9700 },     { 11700, SX126X_GFSK_BW_11700 },   { 14600, SX126X_GFSK_BW_14600 },
    { 19500, SX126X_GFSK_BW_19500 },   { 23400, SX126X_GFSK_BW_23400 },   { 29300, SX126X_GFSK_BW_29300 },
    { 39000, SX126X_GFSK_BW_39000 },   { 46900, SX126X_GFSK_BW_46900 },   { 58600, SX126X_GFSK_BW_58600 },
    { 78200, SX126X_GFSK_BW_78200 },   { 93800, SX126X_GFSK_BW_93800 },   { 117300, SX126X_GFSK_BW_117300 },
    { 156200, SX126X_GFSK_BW_156200 }, { 187200, SX126X_GFSK_BW_187200 }, { 234300, SX126X_GFSK_BW_234300 },
    { 312000, SX126X_GFSK_BW_312000 }, { 373600, SX126X_GFSK_BW_373600 }, { 467000, SX126X_GFSK_BW_467000 },
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief 15.1.2 Workaround
 *
 * @remark Before any packet transmission, bit #2 of SX126X_REG_TX_MODULATION shall be set to:
 * 0 if the LoRa BW = 500 kHz
 * 1 for any other LoRa BW
 * 1 for any (G)FSK configuration
 *
 * @param [in] context Chip implementation context.
 * @param [in] pkt_type The modulation type (G)FSK/LoRa
 * @param [in] bw In case of LoRa modulation the bandwith must be specified
 *
 * @returns Operation status
 */
static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw );

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

sx126x_status_t sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg )
{
    const uint8_t buf[SX126X_SIZE_SET_SLEEP] = {
        SX126X_SET_SLEEP,
        ( uint8_t ) cfg,
    };
    ESP_LOGI("sx126x.c", "command : SX126X_SET_SLEEP");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_SLEEP, 0, 0 );
}

sx126x_status_t sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg )
{
    const uint8_t buf[SX126X_SIZE_SET_STANDBY] = {
        SX126X_SET_STANDBY,
        ( uint8_t ) cfg,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_STANDBY");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STANDBY, 0, 0 );
}

sx126x_status_t sx126x_set_fs( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_FS] = {
        SX126X_SET_FS,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_FS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_FS, 0, 0 );
}

sx126x_status_t sx126x_set_tx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return sx126x_set_tx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_TX] = {
        SX126X_SET_TX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_TX");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX, 0, 0 );
}

sx126x_status_t sx126x_set_rx( const void* context, const uint32_t timeout_in_ms )
{
    if( timeout_in_ms > SX126X_MAX_TIMEOUT_IN_MS )
    {
        return SX126X_STATUS_UNKNOWN_VALUE;
    }

    const uint32_t timeout_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( timeout_in_ms );

    return sx126x_set_rx_with_timeout_in_rtc_step( context, timeout_in_rtc_step );
}

sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_RX] = {
        SX126X_SET_RX,
        ( uint8_t )( timeout_in_rtc_step >> 16 ),
        ( uint8_t )( timeout_in_rtc_step >> 8 ),
        ( uint8_t )( timeout_in_rtc_step >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_RX");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX, 0, 0 );
}

sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable )
{
    const uint8_t buf[SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE] = {
        SX126X_SET_STOP_TIMER_ON_PREAMBLE,
        ( enable == true ) ? 1 : 0,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_STOP_TIMER_ON_PREAMBLE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_STOP_TIMER_ON_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms )
{
    const uint32_t rx_time_in_rtc_step    = sx126x_convert_timeout_in_ms_to_rtc_step( rx_time_in_ms );
    const uint32_t sleep_time_in_rtc_step = sx126x_convert_timeout_in_ms_to_rtc_step( sleep_time_in_ms );

    return sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( context, rx_time_in_rtc_step, sleep_time_in_rtc_step );
}

sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step )
{
    const uint8_t buf[SX126X_SIZE_SET_RX_DUTY_CYCLE] = {
        SX126X_SET_RX_DUTY_CYCLE,
        ( uint8_t )( rx_time_in_rtc_step >> 16 ),
        ( uint8_t )( rx_time_in_rtc_step >> 8 ),
        ( uint8_t )( rx_time_in_rtc_step >> 0 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 16 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 8 ),
        ( uint8_t )( sleep_time_in_rtc_step >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_RX_DUTY_CYCLE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_DUTY_CYCLE, 0, 0 );
}

sx126x_status_t sx126x_set_cad( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_CAD] = {
        SX126X_SET_CAD,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_CAD");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD, 0, 0 );
}

sx126x_status_t sx126x_set_tx_cw( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_CONTINUOUS_WAVE] = {
        SX126X_SET_TX_CONTINUOUS_WAVE,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_TX_CONTINUOUS_WAVE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_CONTINUOUS_WAVE, 0, 0 );
}

sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_INFINITE_PREAMBLE] = {
        SX126X_SET_TX_INFINITE_PREAMBLE,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_TX_INFINITE_PREAMBLE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_INFINITE_PREAMBLE, 0, 0 );
}

sx126x_status_t sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode )
{
    const uint8_t buf[SX126X_SIZE_SET_REGULATOR_MODE] = {
        SX126X_SET_REGULATOR_MODE,
        ( uint8_t ) mode,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_REGULATOR_MODE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_REGULATOR_MODE, 0, 0 );
}

sx126x_status_t sx126x_cal( const void* context, const sx126x_cal_mask_t param )
{
    const uint8_t buf[SX126X_SIZE_CALIBRATE] = {
        SX126X_CALIBRATE,
        ( uint8_t ) param,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_CALIBRATE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE, 0, 0 );
}

sx126x_status_t sx126x_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 )
{
    const uint8_t buf[SX126X_SIZE_CALIBRATE_IMAGE] = {
        SX126X_CALIBRATE_IMAGE,
        freq1,
        freq2,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_CALIBRATE_IMAGE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CALIBRATE_IMAGE, 0, 0 );
}

sx126x_status_t sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz )
{
    // Perform a floor() to get a value for freq1 corresponding to a frequency lower than or equal to freq1_in_mhz
    const uint8_t freq1 = freq1_in_mhz / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

    // Perform a ceil() to get a value for freq2 corresponding to a frequency higher than or equal to freq2_in_mhz
    const uint8_t freq2 =
        ( freq2_in_mhz + SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ - 1 ) / SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ;

    return sx126x_cal_img( context, freq1, freq2 );
}

sx126x_status_t sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PA_CFG] = {
        SX126X_SET_PA_CFG, params->pa_duty_cycle, params->hp_max, params->device_sel, params->pa_lut,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_PA_CFG");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PA_CFG, 0, 0 );
}

sx126x_status_t sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode )
{
    const uint8_t buf[SX126X_SIZE_SET_RX_TX_FALLBACK_MODE] = {
        SX126X_SET_RX_TX_FALLBACK_MODE,
        ( uint8_t ) fallback_mode,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_RX_TX_FALLBACK_MODE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RX_TX_FALLBACK_MODE, 0, 0 );
}

//
// Registers and buffer Access
//

sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_WRITE_REGISTER] = {
        SX126X_WRITE_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_WRITE_REGISTER");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_REGISTER, buffer, size );
}

sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_READ_REGISTER] = {
        SX126X_READ_REGISTER,
        ( uint8_t )( address >> 8 ),
        ( uint8_t )( address >> 0 ),
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_REGISTER, buffer, size );
}

sx126x_status_t sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_WRITE_BUFFER] = {
        SX126X_WRITE_BUFFER,
        offset,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_WRITE_BUFFER");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_WRITE_BUFFER, buffer, size );
}

sx126x_status_t sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size )
{
    const uint8_t buf[SX126X_SIZE_READ_BUFFER] = {
        SX126X_READ_BUFFER,
        offset,
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_READ_BUFFER, buffer, size );
}

//
// DIO and IRQ Control Functions
//
sx126x_status_t sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO_IRQ_PARAMS] = {
        SX126X_SET_DIO_IRQ_PARAMS,     ( uint8_t )( irq_mask >> 8 ),  ( uint8_t )( irq_mask >> 0 ),
        ( uint8_t )( dio1_mask >> 8 ), ( uint8_t )( dio1_mask >> 0 ), ( uint8_t )( dio2_mask >> 8 ),
        ( uint8_t )( dio2_mask >> 0 ), ( uint8_t )( dio3_mask >> 8 ), ( uint8_t )( dio3_mask >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_DIO_IRQ_PARAMS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO_IRQ_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq )
{    
    const uint8_t buf[SX126X_SIZE_GET_IRQ_STATUS] = {
        SX126X_GET_IRQ_STATUS,
        SX126X_NOP,
    };
    uint8_t         irq_local[sizeof( sx126x_irq_mask_t )] = { 0x00 };
    sx126x_status_t status                                 = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_IRQ_STATUS, irq_local,
                                                  sizeof( sx126x_irq_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *irq = ( ( sx126x_irq_mask_t ) irq_local[0] << 8 ) + ( ( sx126x_irq_mask_t ) irq_local[1] << 0 );
    }
    //ESP_LOGI("sx126x.c", "SX126X_GET_IRQ_STATUS : %d", *irq);
    return status;
}


sx126x_status_t sx126x_get_irq_status_bis( const void* context, sx126x_irq_mask_t* irq )
{
    const uint8_t buf[3] = {
        SX126X_GET_IRQ_STATUS,
        SX126X_NOP,
        SX126X_NOP,
    };
    uint8_t         irq_local[sizeof( sx126x_irq_mask_t )] = { 0x00 };
    sx126x_status_t status                                 = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, 3, irq_local,
                                                  sizeof( sx126x_irq_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *irq = (irq_local[1] << 8) | irq_local[2];
    }

    return status;
}

sx126x_status_t sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask )
{
    const uint8_t buf[SX126X_SIZE_CLR_IRQ_STATUS] = {
        SX126X_CLR_IRQ_STATUS,
        ( uint8_t )( irq_mask >> 8 ),
        ( uint8_t )( irq_mask >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_CLR_IRQ_STATUS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_IRQ_STATUS, 0, 0 );
}

sx126x_status_t sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq )
{
    sx126x_irq_mask_t sx126x_irq_mask = SX126X_IRQ_NONE;

    sx126x_status_t status = sx126x_get_irq_status( context, &sx126x_irq_mask );

    if( ( status == SX126X_STATUS_OK ) && ( sx126x_irq_mask != 0 ) )
    {
        status = sx126x_clear_irq_status( context, sx126x_irq_mask );
    }
    if( ( status == SX126X_STATUS_OK ) && ( irq != NULL ) )
    {
        *irq = sx126x_irq_mask;
    }
    return status;
}

sx126x_status_t sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL] = {
        SX126X_SET_DIO2_AS_RF_SWITCH_CTRL,
        ( enable == true ) ? 1 : 0,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_DIO2_AS_RF_SWITCH_CTRL");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO2_AS_RF_SWITCH_CTRL, 0, 0 );
}

sx126x_status_t sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout )
{
    const uint8_t buf[SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL] = {
        SX126X_SET_DIO3_AS_TCXO_CTRL, ( uint8_t ) tcxo_voltage,    ( uint8_t )( timeout >> 16 ),
        ( uint8_t )( timeout >> 8 ),  ( uint8_t )( timeout >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_DIO3_AS_TCXO_CTRL");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_DIO3_AS_TCXO_CTRL, 0, 0 );
}

//
// RF Modulation and Packet-Related Functions
//

sx126x_status_t sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz )
{
    const uint32_t freq = sx126x_convert_freq_in_hz_to_pll_step( freq_in_hz );

    return sx126x_set_rf_freq_in_pll_steps( context, freq );
}

sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq )
{
    const uint8_t buf[SX126X_SIZE_SET_RF_FREQUENCY] = {
        SX126X_SET_RF_FREQUENCY,  ( uint8_t )( freq >> 24 ), ( uint8_t )( freq >> 16 ),
        ( uint8_t )( freq >> 8 ), ( uint8_t )( freq >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_RF_FREQUENCY");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_RF_FREQUENCY, 0, 0 );
}

sx126x_status_t sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_TYPE] = {
        SX126X_SET_PKT_TYPE,
        ( uint8_t ) pkt_type,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_PKT_TYPE");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_TYPE, 0, 0 );
}

sx126x_status_t sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_TYPE] = {
        SX126X_GET_PKT_TYPE,
        SX126X_NOP,
    };

    return ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_TYPE, ( uint8_t* ) pkt_type, 1 );
}

sx126x_status_t sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm, const sx126x_ramp_time_t ramp_time )
{
    const uint8_t buf[SX126X_SIZE_SET_TX_PARAMS] = {
        SX126X_SET_TX_PARAMS,
        ( uint8_t ) pwr_in_dbm,
        ( uint8_t ) ramp_time,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_TX_PARAMS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_TX_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params )
{
    sx126x_status_t status  = SX126X_STATUS_ERROR;
    const uint32_t  bitrate = ( uint32_t )( 32 * SX126X_XTAL_FREQ / params->br_in_bps );
    const uint32_t  fdev    = sx126x_convert_freq_in_hz_to_pll_step( params->fdev_in_hz );
    const uint8_t   buf[SX126X_SIZE_SET_MODULATION_PARAMS_GFSK] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( bitrate >> 16 ),       ( uint8_t )( bitrate >> 8 ),
        ( uint8_t )( bitrate >> 0 ),  ( uint8_t )( params->pulse_shape ), params->bw_dsb_param,
        ( uint8_t )( fdev >> 16 ),    ( uint8_t )( fdev >> 8 ),           ( uint8_t )( fdev >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_MODULATION_PARAMS");
    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_GFSK, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_GFSK, ( sx126x_lora_bw_t ) 0 );
        // WORKAROUND END
    }
    return status;
}

sx126x_status_t sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params )
{
    sx126x_status_t status                                      = SX126X_STATUS_ERROR;
    const uint8_t   buf[SX126X_SIZE_SET_MODULATION_PARAMS_LORA] = {
        SX126X_SET_MODULATION_PARAMS, ( uint8_t )( params->sf ), ( uint8_t )( params->bw ),
        ( uint8_t )( params->cr ),    params->ldro & 0x01,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_MODULATION_PARAMS");
    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_MODULATION_PARAMS_LORA, 0, 0 );

    if( status == SX126X_STATUS_OK )
    {
        // WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 §15.1
        status = sx126x_tx_modulation_workaround( context, SX126X_PKT_TYPE_LORA, params->bw );
        // WORKAROUND END
    }

    return status;
}

sx126x_status_t sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_GFSK] = {
        SX126X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_bits >> 8 ),
        ( uint8_t )( params->preamble_len_in_bits >> 0 ),
        ( uint8_t )( params->preamble_detector ),
        params->sync_word_len_in_bits,
        ( uint8_t )( params->address_filtering ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_type ),
        ( uint8_t )( params->dc_free ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_PKT_PARAMS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_GFSK, 0, 0 );
}

sx126x_status_t sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    const uint8_t buf[SX126X_SIZE_SET_PKT_PARAMS_LORA] = {
        SX126X_SET_PKT_PARAMS,
        ( uint8_t )( params->preamble_len_in_symb >> 8 ),
        ( uint8_t )( params->preamble_len_in_symb >> 0 ),
        ( uint8_t )( params->header_type ),
        params->pld_len_in_bytes,
        ( uint8_t )( params->crc_is_on ? 1 : 0 ),
        ( uint8_t )( params->invert_iq_is_on ? 1 : 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_PKT_PARAMS");
    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_PKT_PARAMS_LORA, 0, 0 );

    // WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_SX1261-2_V1.2 §15.4
    if( status == SX126X_STATUS_OK )
    {
        uint8_t reg_value = 0;

        status = sx126x_read_register( context, SX126X_REG_IQ_POLARITY, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            if( params->invert_iq_is_on == true )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 when using inverted IQ polarity
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 when using standard IQ polarity
            }
            status = sx126x_write_register( context, SX126X_REG_IQ_POLARITY, &reg_value, 1 );
        }
    }
    // WORKAROUND END

    return status;
}

sx126x_status_t sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params )
{
    const uint8_t buf[SX126X_SIZE_SET_CAD_PARAMS] = {
        SX126X_SET_CAD_PARAMS,
        ( uint8_t ) params->cad_symb_nb,
        params->cad_detect_peak,
        params->cad_detect_min,
        ( uint8_t ) params->cad_exit_mode,
        ( uint8_t )( params->cad_timeout >> 16 ),
        ( uint8_t )( params->cad_timeout >> 8 ),
        ( uint8_t )( params->cad_timeout >> 0 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_CAD_PARAMS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_CAD_PARAMS, 0, 0 );
}

sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address )
{
    const uint8_t buf[SX126X_SIZE_SET_BUFFER_BASE_ADDRESS] = {
        SX126X_SET_BUFFER_BASE_ADDRESS,
        tx_base_address,
        rx_base_address,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_BUFFER_BASE_ADDRESS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_BUFFER_BASE_ADDRESS, 0, 0 );
}

sx126x_status_t sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         exp    = 0;
    uint8_t         mant =
        ( ( ( nb_of_symbs > SX126X_MAX_LORA_SYMB_NUM_TIMEOUT ) ? SX126X_MAX_LORA_SYMB_NUM_TIMEOUT : nb_of_symbs ) +
          1 ) >>
        1;

    while( mant > 31 )
    {
        mant = ( mant + 3 ) >> 2;
        exp++;
    }

    const uint8_t buf[SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT] = {
        SX126X_SET_LORA_SYMB_NUM_TIMEOUT,
        mant << ( 2 * exp + 1 ),
    };

    ESP_LOGI("sx126x.c", "command : SX126X_SET_LORA_SYMB_NUM_TIMEOUT");
    status = ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_SET_LORA_SYMB_NUM_TIMEOUT, 0, 0 );

    if( ( status == SX126X_STATUS_OK ) && ( nb_of_symbs > 0 ) )
    {
        uint8_t reg = exp + ( mant << 3 );
        status      = sx126x_write_register( context, SX126X_REG_LR_SYNCH_TIMEOUT, &reg, 1 );
    }

    return status;
}

//
// Communication Status Information
//

sx126x_status_t sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status )
{
    const uint8_t buf[SX126X_SIZE_GET_STATUS] = {
        SX126X_GET_STATUS,
    };
    uint8_t         status_local = 0;
    sx126x_status_t status       = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATUS, &status_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        radio_status->cmd_status =
            ( sx126x_cmd_status_t )( ( status_local & SX126X_CMD_STATUS_MASK ) >> SX126X_CMD_STATUS_POS );
        radio_status->chip_mode =
            ( sx126x_chip_modes_t )( ( status_local & SX126X_CHIP_MODES_MASK ) >> SX126X_CHIP_MODES_POS );
    }

    return status;
}

sx126x_status_t sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status )
{
    const uint8_t buf[SX126X_SIZE_GET_RX_BUFFER_STATUS] = {
        SX126X_GET_RX_BUFFER_STATUS,
        SX126X_NOP,
    };
    uint8_t         status_local[sizeof( sx126x_rx_buffer_status_t )] = { 0x00 };
    sx126x_status_t status                                            = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RX_BUFFER_STATUS, status_local,
                                                  sizeof( sx126x_rx_buffer_status_t ) );

    if( status == SX126X_STATUS_OK )
    {
        rx_buffer_status->pld_len_in_bytes     = status_local[0];
        rx_buffer_status->buffer_start_pointer = status_local[1];
    }

    return status;
}

sx126x_status_t sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_STATUS] = {
        SX126X_GET_PKT_STATUS,
        SX126X_NOP,
    };
    uint8_t         pkt_status_local[3] = { 0x00 };
    sx126x_status_t status              = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local, 3 );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rx_status.pkt_sent =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.pkt_received =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.abort_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.length_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.crc_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ) != 0 ) ? true : false;
        pkt_status->rx_status.adrs_error =
            ( ( pkt_status_local[0] & SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ) != 0 ) ? true : false;

        pkt_status->rssi_sync = ( int8_t )( -pkt_status_local[1] >> 1 );
        pkt_status->rssi_avg  = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status )
{
    const uint8_t buf[SX126X_SIZE_GET_PKT_STATUS] = {
        SX126X_GET_PKT_STATUS,
        SX126X_NOP,
    };
    uint8_t         pkt_status_local[sizeof( sx126x_pkt_status_lora_t )] = { 0x00 };
    sx126x_status_t status                                               = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_PKT_STATUS, pkt_status_local,
                                                  sizeof( sx126x_pkt_status_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        pkt_status->rssi_pkt_in_dbm        = ( int8_t )( -pkt_status_local[0] >> 1 );
        pkt_status->snr_pkt_in_db          = ( ( ( int8_t ) pkt_status_local[1] ) + 2 ) >> 2;
        pkt_status->signal_rssi_pkt_in_dbm = ( int8_t )( -pkt_status_local[2] >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm )
{
    const uint8_t buf[SX126X_SIZE_GET_RSSI_INST] = {
        SX126X_GET_RSSI_INST,
        SX126X_NOP,
    };
    uint8_t         rssi_local = 0x00;
    sx126x_status_t status     = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_RSSI_INST, &rssi_local, 1 );

    if( status == SX126X_STATUS_OK )
    {
        *rssi_in_dbm = ( int8_t )( -rssi_local >> 1 );
    }

    return status;
}

sx126x_status_t sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats )
{
    const uint8_t buf[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t         stats_local[sizeof( sx126x_stats_gfsk_t )] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS, stats_local,
                                                  sizeof( sx126x_stats_gfsk_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received  = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_len_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }

    return status;
}

sx126x_status_t sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats )
{
    const uint8_t buf[SX126X_SIZE_GET_STATS] = {
        SX126X_GET_STATS,
        SX126X_NOP,
    };
    uint8_t         stats_local[sizeof( sx126x_stats_lora_t )] = { 0 };
    sx126x_status_t status                                     = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_STATS, stats_local,
                                                  sizeof( sx126x_stats_lora_t ) );

    if( status == SX126X_STATUS_OK )
    {
        stats->nb_pkt_received     = ( ( uint16_t ) stats_local[0] << 8 ) + ( uint16_t ) stats_local[1];
        stats->nb_pkt_crc_error    = ( ( uint16_t ) stats_local[2] << 8 ) + ( uint16_t ) stats_local[3];
        stats->nb_pkt_header_error = ( ( uint16_t ) stats_local[4] << 8 ) + ( uint16_t ) stats_local[5];
    }
    return status;
}

sx126x_status_t sx126x_reset_stats( const void* context )
{
    const uint8_t buf[SX126X_SIZE_RESET_STATS] = {
        SX126X_RESET_STATS, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP, SX126X_NOP,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_RESET_STATS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_RESET_STATS, 0, 0 );
}

//
// Miscellaneous
//

sx126x_status_t sx126x_reset( const void* context )
{
    //return ( sx126x_status_t ) sx126x_hal_reset( context );
    Reset();
    return SX126X_STATUS_OK;
}

sx126x_status_t sx126x_wakeup( const void* context )
{
    return ( sx126x_status_t ) sx126x_hal_wakeup( context );
}

sx126x_status_t sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors )
{
    const uint8_t buf[SX126X_SIZE_GET_DEVICE_ERRORS] = {
        SX126X_GET_DEVICE_ERRORS,
        SX126X_NOP,
    };
    uint8_t         errors_local[sizeof( sx126x_errors_mask_t )] = { 0x00 };
    sx126x_status_t status                                       = SX126X_STATUS_ERROR;

    status = ( sx126x_status_t ) sx126x_hal_read( context, buf, SX126X_SIZE_GET_DEVICE_ERRORS, errors_local,
                                                  sizeof( sx126x_errors_mask_t ) );

    if( status == SX126X_STATUS_OK )
    {
        *errors = ( ( sx126x_errors_mask_t ) errors_local[0] << 8 ) + ( ( sx126x_errors_mask_t ) errors_local[1] << 0 );
    }

    return status;
}

sx126x_status_t sx126x_clear_device_errors( const void* context )
{
    const uint8_t buf[SX126X_SIZE_CLR_DEVICE_ERRORS] = {
        SX126X_CLR_DEVICE_ERRORS,
        SX126X_NOP,
        SX126X_NOP,
    };

    ESP_LOGI("sx126x.c", "command : SX126X_CLR_DEVICE_ERRORS");
    return ( sx126x_status_t ) sx126x_hal_write( context, buf, SX126X_SIZE_CLR_DEVICE_ERRORS, 0, 0 );
}

sx126x_status_t sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;

    if( bw != 0 )
    {
        status = SX126X_STATUS_UNKNOWN_VALUE;
        for( uint8_t i = 0; i < ( sizeof( gfsk_bw ) / sizeof( gfsk_bw_t ) ); i++ )
        {
            if( bw <= gfsk_bw[i].bw )
            {
                *param = gfsk_bw[i].param;
                status = SX126X_STATUS_OK;
                break;
            }
        }
    }

    return status;
}

uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw )
{
    uint32_t bw_in_hz = 0;

    switch( bw )
    {
    case SX126X_LORA_BW_007:
        bw_in_hz = 7812UL;
        break;
    case SX126X_LORA_BW_010:
        bw_in_hz = 10417UL;
        break;
    case SX126X_LORA_BW_015:
        bw_in_hz = 15625UL;
        break;
    case SX126X_LORA_BW_020:
        bw_in_hz = 20833UL;
        break;
    case SX126X_LORA_BW_031:
        bw_in_hz = 31250UL;
        break;
    case SX126X_LORA_BW_041:
        bw_in_hz = 41667UL;
        break;
    case SX126X_LORA_BW_062:
        bw_in_hz = 62500UL;
        break;
    case SX126X_LORA_BW_125:
        bw_in_hz = 125000UL;
        break;
    case SX126X_LORA_BW_250:
        bw_in_hz = 250000UL;
        break;
    case SX126X_LORA_BW_500:
        bw_in_hz = 500000UL;
        break;
    }

    return bw_in_hz;
}

uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p )
{
    const int32_t pld_len_in_bytes = pkt_p->pld_len_in_bytes;
    const int32_t sf               = mod_p->sf;
    const bool    pld_is_fix       = pkt_p->header_type == SX126X_LORA_PKT_IMPLICIT;
    const int32_t cr_denom         = mod_p->cr + 4;

    int32_t ceil_denominator;
    int32_t ceil_numerator =
        ( pld_len_in_bytes << 3 ) + ( pkt_p->crc_is_on ? 16 : 0 ) - ( 4 * sf ) + ( pld_is_fix ? 0 : 20 );

    if( sf <= 6 )
    {
        ceil_denominator = 4 * sf;
    }
    else
    {
        ceil_numerator += 8;

        if( mod_p->ldro )
        {
            ceil_denominator = 4 * ( sf - 2 );
        }
        else
        {
            ceil_denominator = 4 * sf;
        }
    }

    if( ceil_numerator < 0 )
    {
        ceil_numerator = 0;
    }

    // Perform integral ceil()
    int32_t intermed =
        ( ( ceil_numerator + ceil_denominator - 1 ) / ceil_denominator ) * cr_denom + pkt_p->preamble_len_in_symb + 12;

    if( sf <= 6 )
    {
        intermed += 2;
    }

    return ( uint32_t )( ( 4 * intermed + 1 ) * ( 1 << ( sf - 2 ) ) );
}

uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_lora_time_on_air_numerator( pkt_p, mod_p );
    uint32_t denominator = sx126x_get_lora_bw_in_hz( mod_p->bw );
    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p )
{
    return pkt_p->preamble_len_in_bits + ( pkt_p->header_type == SX126X_GFSK_PKT_VAR_LEN ? 8 : 0 ) +
           pkt_p->sync_word_len_in_bits +
           ( ( pkt_p->pld_len_in_bytes + ( pkt_p->address_filtering == SX126X_GFSK_ADDRESS_FILTERING_DISABLE ? 0 : 1 ) +
               sx126x_get_gfsk_crc_len_in_bytes( pkt_p->crc_type ) )
             << 3 );
}

uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p )
{
    uint32_t numerator   = 1000U * sx126x_get_gfsk_time_on_air_numerator( pkt_p );
    uint32_t denominator = mod_p->br_in_bps;

    // Perform integral ceil()
    return ( numerator + denominator - 1 ) / denominator;
}

sx126x_status_t sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n )
{
    sx126x_status_t status;

    uint8_t tmp_ana_lna   = 0x00;
    uint8_t tmp_ana_mixer = 0x00;
    uint8_t tmp           = 0x00;

    // Configure for random number generation
    status = sx126x_read_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_lna & ~( 1 << 0 );
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    status = sx126x_read_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    tmp    = tmp_ana_mixer & ~( 1 << 7 );
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Start RX continuous
    status = sx126x_set_rx_with_timeout_in_rtc_step( context, SX126X_RX_CONTINUOUS );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Store values
    for( unsigned int i = 0; i < n; i++ )
    {
        status = sx126x_read_register( context, SX126X_REG_RNGBASEADDRESS, ( uint8_t* ) &numbers[i], 4 );
        if( status != SX126X_STATUS_OK )
        {
            return status;
        }
    }

    status = sx126x_set_standby( context, SX126X_STANDBY_CFG_RC );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }

    // Restore registers
    status = sx126x_write_register( context, SX126X_REG_ANA_LNA, &tmp_ana_lna, 1 );
    if( status != SX126X_STATUS_OK )
    {
        return status;
    }
    status = sx126x_write_register( context, SX126X_REG_ANA_MIXER, &tmp_ana_mixer, 1 );

    return status;
}

uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz )
{
    uint32_t steps_int;
    uint32_t steps_frac;

    // Get integer and fractional parts of the frequency computed with a PLL step scaled value
    steps_int  = freq_in_hz / SX126X_PLL_STEP_SCALED;
    steps_frac = freq_in_hz - ( steps_int * SX126X_PLL_STEP_SCALED );

    // Apply the scaling factor to retrieve a frequency in Hz (+ ceiling)
    return ( steps_int << SX126X_PLL_STEP_SHIFT_AMOUNT ) +
           ( ( ( steps_frac << SX126X_PLL_STEP_SHIFT_AMOUNT ) + ( SX126X_PLL_STEP_SCALED >> 1 ) ) /
             SX126X_PLL_STEP_SCALED );
}

uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms )
{
    return ( uint32_t )( timeout_in_ms * ( SX126X_RTC_FREQ_IN_HZ / 1000 ) );
}

//
// Registers access
//

sx126x_status_t sx126x_cfg_rx_boosted( const void* context, const bool state )
{
    if( state == true )
    {
        return sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x96 }, 1 );
    }
    else
    {
        return sx126x_write_register( context, SX126X_REG_RXGAIN, ( const uint8_t[] ){ 0x94 }, 1 );
    }
}

sx126x_status_t sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buf[8] = { 0 };

    if( sync_word_len <= 8 )
    {
        memcpy( buf, sync_word, sync_word_len );
        status = sx126x_write_register( context, SX126X_REG_SYNCWORDBASEADDRESS, buf, 8 );
    }

    return status;
}

sx126x_status_t sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         buffer[2] = { 0x00 };

    status = sx126x_read_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );

    if( status == SX126X_STATUS_OK )
    {
        buffer[0] = ( buffer[0] & ~0xF0 ) + ( sync_word & 0xF0 );
        buffer[1] = ( buffer[1] & ~0xF0 ) + ( ( sync_word & 0x0F ) << 4 );

        status = sx126x_write_register( context, SX126X_REG_LR_SYNCWORD, buffer, 2 );
    }

    return status;
}

sx126x_status_t sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed )
{
    uint8_t s[] = { ( uint8_t )( seed >> 8 ), ( uint8_t ) seed };

    return sx126x_write_register( context, SX126X_REG_CRCSEEDBASEADDRESS, s, sizeof( s ) );
}

sx126x_status_t sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial )
{
    uint8_t poly[] = { ( uint8_t )( polynomial >> 8 ), ( uint8_t ) polynomial };

    return sx126x_write_register( context, SX126X_REG_CRCPOLYBASEADDRESS, poly, sizeof( poly ) );
}

sx126x_status_t sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    // The SX126X_REG_WHITSEEDBASEADDRESS @ref LSBit is used for the seed value. The 7 MSBits must not be modified.
    // Thus, we first need to read the current value and then change the LSB according to the provided seed @ref value.
    status = sx126x_read_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
    if( status == SX126X_STATUS_OK )
    {
        reg_value = ( reg_value & 0xFE ) | ( ( uint8_t )( seed >> 8 ) & 0x01 );
        status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS, &reg_value, 1 );
        if( status == SX126X_STATUS_OK )
        {
            reg_value = ( uint8_t ) seed;
            status    = sx126x_write_register( context, SX126X_REG_WHITSEEDBASEADDRESS + 1, &reg_value, 1 );
        }
    }

    return status;
}

sx126x_status_t sx126x_cfg_tx_clamp( const void* context )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0x00;

    status = sx126x_read_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        reg_value |= SX126X_REG_TX_CLAMP_CFG_MASK;
        status = sx126x_write_register( context, SX126X_REG_TX_CLAMP_CFG, &reg_value, 1 );
    }

    return status;
}

sx126x_status_t sx126x_stop_rtc( const void* context )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    reg_value = 0;
    status    = sx126x_write_register( context, SX126X_REG_RTC_CTRL, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );

        if( status == SX126X_STATUS_OK )
        {
            reg_value |= SX126X_REG_EVT_CLR_TIMEOUT_MASK;
            status = sx126x_write_register( context, SX126X_REG_EVT_CLR, &reg_value, 1 );
        }
    }

    return status;
}

sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma )
{
    return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_OCP, &ocp_in_step_of_2_5_ma, 1 );
}

sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb )
{
    uint8_t trimming_capacitor_values[2] = { trimming_cap_xta, trimming_cap_xtb };

    return ( sx126x_status_t ) sx126x_write_register( context, SX126X_REG_XTATRIM, trimming_capacitor_values, 2 );
}

sx126x_status_t sx126x_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
                                                        uint8_t register_nb )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buffer[9];

    status = sx126x_read_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );

    if( status == SX126X_STATUS_OK )
    {
        const uint8_t initial_nb_of_registers = buffer[0];
        uint8_t*      register_list           = &buffer[1];

        for( uint8_t index = 0; index < register_nb; index++ )
        {
            bool register_has_to_be_added = true;

            // Check if the current register is already added to the list
            for( uint8_t i = 0; i < buffer[0]; i++ )
            {
                if( register_addr[index] == ( ( uint16_t ) register_list[2 * i] << 8 ) + register_list[2 * i + 1] )
                {
                    register_has_to_be_added = false;
                    break;
                }
            }

            if( register_has_to_be_added == true )
            {
                if( buffer[0] < SX126X_MAX_NB_REG_IN_RETENTION )
                {
                    register_list[2 * buffer[0]]     = ( uint8_t )( register_addr[index] >> 8 );
                    register_list[2 * buffer[0] + 1] = ( uint8_t )( register_addr[index] >> 0 );
                    buffer[0] += 1;
                }
                else
                {
                    return SX126X_STATUS_ERROR;
                }
            }
        }

        if( buffer[0] != initial_nb_of_registers )
        {
            status = sx126x_write_register( context, SX126X_REG_RETENTION_LIST_BASE_ADDRESS, buffer, 9 );
        }
    }

    return status;
}

sx126x_status_t sx126x_init_retention_list( const void* context )
{
    const uint16_t list_of_registers[3] = { SX126X_REG_RXGAIN, SX126X_REG_TX_MODULATION, SX126X_REG_IQ_POLARITY };

    return sx126x_add_registers_to_retention_list( context, list_of_registers,
                                                   sizeof( list_of_registers ) / sizeof( list_of_registers[0] ) );
}

sx126x_status_t sx126x_get_lora_params_from_header( const void* context, sx126x_lora_cr_t* cr, bool* crc_is_on )
{
    sx126x_status_t status = SX126X_STATUS_ERROR;
    uint8_t         buffer_cr;
    uint8_t         buffer_crc;

    status = sx126x_read_register( context, SX126X_REG_LR_HEADER_CR, &buffer_cr, 1 );

    if( status == SX126X_STATUS_OK )
    {
        status = sx126x_read_register( context, SX126X_REG_LR_HEADER_CRC, &buffer_crc, 1 );

        if( status == SX126X_STATUS_OK )
        {
            *cr = ( sx126x_lora_cr_t )( ( buffer_cr & SX126X_REG_LR_HEADER_CR_MASK ) >> SX126X_REG_LR_HEADER_CR_POS );
            *crc_is_on = ( ( buffer_crc & SX126X_REG_LR_HEADER_CRC_MASK ) != 0 ) ? true : false;
        }
    }

    return status;
}


void sx126x_init(void)
{
    sx1261_init();
}


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static sx126x_status_t sx126x_tx_modulation_workaround( const void* context, sx126x_pkt_type_t pkt_type,
                                                        sx126x_lora_bw_t bw )
{
    sx126x_status_t status    = SX126X_STATUS_ERROR;
    uint8_t         reg_value = 0;

    status = sx126x_read_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );

    if( status == SX126X_STATUS_OK )
    {
        if( pkt_type == SX126X_PKT_TYPE_LORA )
        {
            if( bw == SX126X_LORA_BW_500 )
            {
                reg_value &= ~( 1 << 2 );  // Bit 2 set to 0 if the LoRa BW = 500 kHz
            }
            else
            {
                reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any other LoRa BW
            }
        }
        else
        {
            reg_value |= ( 1 << 2 );  // Bit 2 set to 1 for any (G)FSK configuration
        }

        status = sx126x_write_register( context, SX126X_REG_TX_MODULATION, &reg_value, 1 );
    }
    return status;
}

static inline uint32_t sx126x_get_gfsk_crc_len_in_bytes( sx126x_gfsk_crc_types_t crc_type )
{
    switch( crc_type )
    {
    case SX126X_GFSK_CRC_OFF:
        return 0;
    case SX126X_GFSK_CRC_1_BYTE:
        return 1;
    case SX126X_GFSK_CRC_2_BYTE:
        return 2;
    case SX126X_GFSK_CRC_1_BYTE_INV:
        return 1;
    case SX126X_GFSK_CRC_2_BYTE_INV:
        return 2;
    }

    return 0;
}



void sx1261_init(void)
{
	ESP_LOGI(TAG, "CONFIG_MISO_GPIO=%d", PIN_SPI_MISO);
	ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d", PIN_SPI_MOSI);
	ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d", PIN_SPI_SCLK);
	ESP_LOGI(TAG, "CONFIG_NSS_GPIO=%d", CONFIG_NSS_GPIO);
	ESP_LOGI(TAG, "CONFIG_RST_GPIO=%d", CONFIG_RST_GPIO);
	ESP_LOGI(TAG, "CONFIG_BUSY_GPIO=%d", CONFIG_BUSY_GPIO);
	ESP_LOGI(TAG, "CONFIG_DIO1_GPIO=%d", CONFIG_DIO1_GPIO);
	ESP_LOGI(TAG, "CONFIG_TXEN_GPIO=%d", CONFIG_TXEN_GPIO);
	ESP_LOGI(TAG, "CONFIG_RXEN_GPIO=%d", CONFIG_RXEN_GPIO);

	SX126x_SPI_SELECT = CONFIG_NSS_GPIO;
	SX126x_RESET = CONFIG_RST_GPIO;
	SX126x_BUSY	= CONFIG_BUSY_GPIO;
	SX126x_DIO1	= CONFIG_DIO1_GPIO;
	SX126x_TXEN	= CONFIG_TXEN_GPIO;
	SX126x_RXEN	= CONFIG_RXEN_GPIO;
	
	txActive = false;
	debugPrint = false;

	rtc_gpio_hold_dis(SX126x_SPI_SELECT);

	gpio_reset_pin(SX126x_SPI_SELECT);
	gpio_set_direction(SX126x_SPI_SELECT, GPIO_MODE_OUTPUT);
	gpio_set_level(SX126x_SPI_SELECT, 1);

	gpio_reset_pin(SX126x_RESET);
	gpio_set_direction(SX126x_RESET, GPIO_MODE_OUTPUT);
	
	gpio_reset_pin(SX126x_BUSY);
	gpio_set_direction(SX126x_BUSY, GPIO_MODE_INPUT);

	gpio_reset_pin(SX126x_DIO1);
	gpio_set_direction(SX126x_DIO1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(SX126x_DIO1, GPIO_PULLDOWN_ONLY);

	if (SX126x_TXEN != -1) {
		gpio_reset_pin(SX126x_TXEN);
		gpio_set_direction(SX126x_TXEN, GPIO_MODE_OUTPUT);
	}

	if (SX126x_RXEN != -1) {
		gpio_reset_pin(SX126x_RXEN);
		gpio_set_direction(SX126x_RXEN, GPIO_MODE_OUTPUT);
	}

/* 
	spi_bus_config_t spi_bus_config = {
		.sclk_io_num = PIN_SPI_SCLK,
		.mosi_io_num = PIN_SPI_MOSI,
		.miso_io_num = PIN_SPI_MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	esp_err_t ret = spi_bus_initialize(USED_SPI_HOST, &spi_bus_config, SPI_DMA_CH_AUTO);
	if (ret != ESP_OK) {
		printf("Failed to initialize SPI bus: %s\n", esp_err_to_name(ret));
	}

*/

	spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof( spi_device_interface_config_t ) );
	devcfg.clock_speed_hz = SPI_Frequency;
	// It does not work with hardware CS control.
	//devcfg.spics_io_num = SX126x_SPI_SELECT;
	// It does work with software CS control.
	devcfg.spics_io_num = -1;
	devcfg.queue_size = 7;
	devcfg.mode = 0;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

	//spi_device_handle_t handle;
	esp_err_t ret = spi_bus_add_device( USED_SPI_HOST, &devcfg, &spiHandle_radio);
	if (debugPrint) {
		ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
	}
	assert(ret==ESP_OK);
	Reset();
#if 0
	pinMode(SX126x_SPI_SELECT, OUTPUT);
	pinMode(SX126x_RESET, OUTPUT);
	pinMode(SX126x_BUSY, INPUT);
	if (SX126x_TXEN != -1) pinMode(SX126x_TXEN, OUTPUT);
	if (SX126x_RXEN != -1) pinMode(SX126x_RXEN, OUTPUT);

	SPI.begin();
#endif
}

bool spi_write_byte(uint8_t* Dataout, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = NULL;
		spi_device_transmit( spiHandle_radio, &SPITransaction );
	}

	return true;
}

bool spi_read_byte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength )
{
	spi_transaction_t SPITransaction;

	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Dataout;
		SPITransaction.rx_buffer = Datain;
		spi_device_transmit( spiHandle_radio, &SPITransaction );
	}

	return true;
}

uint8_t spi_transfer(uint8_t address)
{
	uint8_t datain[1];
	uint8_t dataout[1];
	dataout[0] = address;
	//spi_write_byte(dataout, 1 );
	spi_read_byte(datain, dataout, 1 );
	return datain[0];
}

void Reset(void)
{
	delay(10);
	gpio_set_level(SX126x_RESET,0);
	delay(20);
	gpio_set_level(SX126x_RESET,1);
	delay(10);
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);
}

void LoRaDebugPrint(bool enable) 
{
	debugPrint = enable;
}


int16_t LoRaBegin(uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage, bool useRegulatorLDO) 
{
	if ( txPowerInDbm > 14 )
		txPowerInDbm = 14;
	if ( txPowerInDbm < -17 )
		txPowerInDbm = -17;
	
	
	
	uint8_t wk[2];
	ReadRegister(SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2); // 0x0740
	uint16_t syncWord = (wk[0] << 8) + wk[1];
	ESP_LOGI(TAG, "syncWord=0x%x", syncWord);
	if (syncWord != SX126X_SYNC_WORD_PUBLIC && syncWord != SX126X_SYNC_WORD_PRIVATE) {
		ESP_LOGE(TAG, "SX126x error, maybe no SPI connection");
		return ERR_INVALID_MODE;
	}

	ESP_LOGI(TAG, "SX126x installed");
	SetStandby(SX126X_STANDBY_RC);

	SetDio2AsRfSwitchCtrl(false);
	ESP_LOGI(TAG, "tcxoVoltage=%f", tcxoVoltage);
	// set TCXO control, if requested
	if(tcxoVoltage > 0.0) {
		SetDio3AsTcxoCtrl(tcxoVoltage, RADIO_TCXO_SETUP_TIME); // Configure the radio to use a TCXO controlled by DIO3
	}

	Calibrate(	SX126X_CALIBRATE_IMAGE_ON
				| SX126X_CALIBRATE_ADC_BULK_P_ON
				| SX126X_CALIBRATE_ADC_BULK_N_ON
				| SX126X_CALIBRATE_ADC_PULSE_ON
				| SX126X_CALIBRATE_PLL_ON
				| SX126X_CALIBRATE_RC13M_ON
				| SX126X_CALIBRATE_RC64K_ON
				);

	ESP_LOGI(TAG, "useRegulatorLDO=%d", useRegulatorLDO);
	if (useRegulatorLDO) {
		SetRegulatorMode(SX126X_REGULATOR_LDO); // set regulator mode: LDO
	} else {
		SetRegulatorMode(SX126X_REGULATOR_DC_DC); // set regulator mode: DC-DC
	}

	SetBufferBaseAddress(0, 0);
#if 0
	// SX1261_TRANCEIVER
	SetPaConfig(0x06, 0x00, 0x01, 0x01); // PA Optimal Settings +15 dBm
	// SX1262_TRANCEIVER
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
	// SX1268_TRANCEIVER
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
#endif
	SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
	SetOvercurrentProtection(60.0);  // current max 60mA for the whole device
	SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U); //0 fuer Empfaenger
	SetRfFrequency(frequencyInHz);
	return ERR_NONE;
}

void FixInvertedIQ(uint8_t iqConfig)
{
	// fixes IQ configuration for inverted IQ
	// see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
	// When exchanging LoRa packets with inverted IQ polarity, some packet losses may be observed for longer packets.
	// Workaround: Bit 2 at address 0x0736 must be set to:
	// “0” when using inverted IQ polarity (see the SetPacketParam(...) command)
	// “1” when using standard IQ polarity

	

	// read current IQ configuration
	uint8_t iqConfigCurrent = 0;
	ReadRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736

	// set correct IQ configuration
	//if(iqConfig == SX126X_LORA_IQ_STANDARD) {
	if(iqConfig == SX126X_LORA_IQ_INVERTED) {
		iqConfigCurrent &= 0xFB; // using inverted IQ polarity
	} else {
		iqConfigCurrent |= 0x04; // using standard IQ polarity
	}
	
	
	// update with the new value
	WriteRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736
}


void LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) 
{
	Reset();
	ESP_LOGI(TAG, "Reset");
	SetStopRxTimerOnPreambleDetect(false);
	SetLoRaSymbNumTimeout(0); 
	SetPacketType(SX126X_PACKET_TYPE_LORA); // SX126x.ModulationParams.PacketType : MODEM_LORA
	uint8_t ldro = 0; // LowDataRateOptimize OFF
	SetModulationParams(spreadingFactor, bandwidth, codingRate, ldro);
	
	PacketParams[0] = (preambleLength >> 8) & 0xFF;
	PacketParams[1] = preambleLength;
	if ( payloadLen )
	{
		PacketParams[2] = 0x01; // Fixed length packet (implicit header)
		PacketParams[3] = payloadLen;
	}
	else
	{
		PacketParams[2] = 0x00; // Variable length packet (explicit header)
		PacketParams[3] = 0xFF;
	}

	if ( crcOn )
		PacketParams[4] = SX126X_LORA_IQ_INVERTED;
	else
		PacketParams[4] = SX126X_LORA_IQ_STANDARD;

	if ( invertIrq )
		PacketParams[5] = 0x01; // Inverted LoRa I and Q signals setup
	else
		PacketParams[5] = 0x00; // Standard LoRa I and Q signals setup

	// fixes IQ configuration for inverted IQ
	FixInvertedIQ(PacketParams[5]);

	//SetPacketParams
	WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C

	// Do not use DIO interruptst
	SetDioIrqParams(SX126X_IRQ_ALL,   //all interrupts enabled
					SX126X_IRQ_NONE,  //interrupts on DIO1
					SX126X_IRQ_NONE,  //interrupts on DIO2
					SX126X_IRQ_NONE); //interrupts on DIO3

	// Receive state no receive timeoout
	//SetRx(0xFFFFFF);
}



uint8_t LoRaReceive(uint8_t *pData, uint16_t len) 
{
	uint8_t rxLen = 0;
	uint16_t irqRegs = GetIrqStatus();
	//uint8_t status = GetStatus();
	
	if( irqRegs & SX126X_IRQ_RX_DONE )
	{
		//ClearIrqStatus(SX126X_IRQ_RX_DONE);
		ClearIrqStatus(SX126X_IRQ_ALL);
		rxLen = ReadBuffer(pData, len);
	}
	
	return rxLen;
}


bool LoRaSend(uint8_t *pData, uint8_t len, uint8_t mode)
{
	uint16_t irqStatus;
	bool rv = false;
	
	if ( txActive == false )
	{
		txActive = true;
		PacketParams[2] = 0x00; //Variable length packet (explicit header)
		PacketParams[3] = len;
		WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C
		
		//ClearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
		ClearIrqStatus(SX126X_IRQ_ALL);
		
		WriteBuffer(pData, len);
		//SetTxContinuousWave();
		SetTx(0);

		if ( mode & SX126x_TXMODE_SYNC )
		{
			irqStatus = GetIrqStatus();
			while ( (!(irqStatus & SX126X_IRQ_TX_DONE)) && (!(irqStatus & SX126X_IRQ_TIMEOUT)) )
			{
				delay(1);
				irqStatus = GetIrqStatus();
				ESP_LOGI(TAG, "on attends la fin, irqStatus: 0x%x" , irqStatus );
			}
			if (debugPrint) {
				ESP_LOGI(TAG, "irqStatus=0x%x", irqStatus);
				if (irqStatus & SX126X_IRQ_TX_DONE) {
					ESP_LOGI(TAG, "SX126X_IRQ_TX_DONE");
				}
				if (irqStatus & SX126X_IRQ_TIMEOUT) {
					ESP_LOGI(TAG, "SX126X_IRQ_TIMEOUT");
				}
			}
			txActive = false;
	
			SetRx(0xFFFFFF);
	
			if ( irqStatus & SX126X_IRQ_TX_DONE) {
				rv = true;
			}
		}
		else
		{
			rv = true;
		}
	}
	if (debugPrint) {
		ESP_LOGI(TAG, "Send rv=0x%x", rv);
	}
	return rv;
}


bool ReceiveMode(void)
{
	uint16_t irq;
	bool rv = false;

	if ( txActive == false )
	{
		rv = true;
	}
	else
	{
		irq = GetIrqStatus();
		if ( irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT) )
		{ 
			SetRx(0xFFFFFF);
			txActive = false;
			rv = true;
		}
	}

	return rv;
}


void GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket)
{
	uint8_t buf[4];
	ReadCommand( SX126X_CMD_GET_PACKET_STATUS, buf, 4 ); // 0x14
	*rssiPacket = (buf[3] >> 1) * -1;
	( buf[2] < 128 ) ? ( *snrPacket = buf[2] >> 2 ) : ( *snrPacket = ( ( buf[2] - 256 ) >> 2 ) );
}


void SetTxPower(int8_t txPowerInDbm)
{
	SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
}




void Wakeup(void)
{
	GetStatus();
}


void SetStandby(uint8_t mode)
{
	uint8_t data = mode;
	WriteCommand(SX126X_CMD_SET_STANDBY, &data, 1); // 0x80
}


uint8_t GetStatus(void)
{
	uint8_t rv;
	ReadCommand(SX126X_CMD_GET_STATUS, &rv, 1); // 0xC0
	return rv;
}


void SetDio3AsTcxoCtrl(float voltage, uint32_t delay)
{
	uint8_t buf[4];

	//buf[0] = tcxoVoltage & 0x07;
	if(fabs(voltage - 1.6) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_1_6;
	} else if(fabs(voltage - 1.7) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_1_7;
	} else if(fabs(voltage - 1.8) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_1_8;
	} else if(fabs(voltage - 2.2) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_2_2;
	} else if(fabs(voltage - 2.4) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_2_4;
	} else if(fabs(voltage - 2.7) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_2_7;
	} else if(fabs(voltage - 3.0) <= 0.001) {
		buf[0] = SX126X_DIO3_OUTPUT_3_0;
	} else {
		buf[0] = SX126X_DIO3_OUTPUT_3_3;
	}

	uint32_t delayValue = (float)delay / 15.625;
	buf[1] = ( uint8_t )( ( delayValue >> 16 ) & 0xFF );
	buf[2] = ( uint8_t )( ( delayValue >> 8 ) & 0xFF );
	buf[3] = ( uint8_t )( delayValue & 0xFF );

	WriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4); // 0x97
}


void Calibrate(uint8_t calibParam)
{
	uint8_t data = calibParam;
	WriteCommand(SX126X_CMD_CALIBRATE, &data, 1); // 0x89
}


void SetDio2AsRfSwitchCtrl(uint8_t enable)
{
	uint8_t data = enable;
	WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1); // 0x9D
}


void SetRfFrequency(uint32_t frequency)
{
	uint8_t buf[4];
	uint32_t freq = 0;

	CalibrateImage(frequency);

	freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
	buf[0] = (uint8_t)((freq >> 24) & 0xFF);
	buf[1] = (uint8_t)((freq >> 16) & 0xFF);
	buf[2] = (uint8_t)((freq >> 8) & 0xFF);
	buf[3] = (uint8_t)(freq & 0xFF);
	WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4); // 0x86
}


void CalibrateImage(uint32_t frequency)
{
	uint8_t calFreq[2];

	if( frequency> 900000000 )
	{
			calFreq[0] = 0xE1;
			calFreq[1] = 0xE9;
	}
	else if( frequency > 850000000 )
	{
			calFreq[0] = 0xD7;
			calFreq[1] = 0xD8;
	}
	else if( frequency > 770000000 )
	{
			calFreq[0] = 0xC1;
			calFreq[1] = 0xC5;
	}
	else if( frequency > 460000000 )
	{
			calFreq[0] = 0x75;
			calFreq[1] = 0x81;
	}
	else if( frequency > 425000000 )
	{
			calFreq[0] = 0x6B;
			calFreq[1] = 0x6F;
	}
	WriteCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2); // 0x98
}


void SetRegulatorMode(uint8_t mode)
{
	uint8_t data = mode;
	WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1); // 0x96
}


void SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
	uint8_t buf[2];

	buf[0] = txBaseAddress;
	buf[1] = rxBaseAddress;
	WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2); // 0x8F
}


void SetPowerConfig(int8_t power, uint8_t rampTime)
{
	uint8_t buf[2];

	if( power > 14 )
	{
			power = 14;
	}
	else if( power < -17 )
	{
			power = -17;
	}
		
	buf[0] = power;
	buf[1] = ( uint8_t )rampTime;
	WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2); // 0x8E
}


void SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
	uint8_t buf[4];

	buf[0] = paDutyCycle;
	buf[1] = hpMax;
	buf[2] = deviceSel;
	buf[3] = paLut;
	WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4); // 0x95
}


void SetOvercurrentProtection(float currentLimit)
{
	if((currentLimit >= 0.0) && (currentLimit <= 140.0)) {
		uint8_t buf[1];
		buf[0] = (uint8_t)(currentLimit / 2.5);
		WriteRegister(SX126X_REG_OCP_CONFIGURATION, buf, 1); // 0x08E7
	}
}

void SetSyncWord(int16_t sync) 
{
	uint8_t buf[2];

	buf[0] = (uint8_t)((sync >> 8) & 0x00FF);
	buf[1] = (uint8_t)(sync & 0x00FF);
	WriteRegister(SX126X_REG_LORA_SYNC_WORD_MSB, buf, 2); // 0x0740
}

void SetDioIrqParams( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
	uint8_t buf[8];

	buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
	buf[1] = (uint8_t)(irqMask & 0x00FF);
	buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
	buf[3] = (uint8_t)(dio1Mask & 0x00FF);
	buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
	buf[5] = (uint8_t)(dio2Mask & 0x00FF);
	buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
	buf[7] = (uint8_t)(dio3Mask & 0x00FF);
	WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8); // 0x08
}


void SetStopRxTimerOnPreambleDetect(bool enable)
{
	ESP_LOGI(TAG, "SetStopRxTimerOnPreambleDetect enable=%d", enable);
	//uint8_t data = (uint8_t)enable;
	uint8_t data = 0;
	if (enable) data = 1;
	WriteCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1); // 0x9F
}


void SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
	uint8_t data = SymbNum;
	WriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1); // 0xA0
}


void SetPacketType(uint8_t packetType)
{
	uint8_t data = packetType;
	WriteCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1); // 0x01
}


void SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize)
{
	uint8_t data[4];
	//currently only LoRa supported
	data[0] = spreadingFactor;
	data[1] = bandwidth;
	data[2] = codingRate;
	data[3] = lowDataRateOptimize;
	WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4); // 0x8B
}


void SetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout)
{
	uint8_t data[7];
	data[0] = cadSymbolNum;
	data[1] = cadDetPeak;
	data[2] = cadDetMin;
	data[3] = cadExitMode;
	data[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
	data[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
	data[6] = (uint8_t)(cadTimeout & 0xFF);
	WriteCommand(SX126X_CMD_SET_CAD_PARAMS, data, 7); // 0x88
}


void SetCad()
{
	uint8_t data = 0;
	WriteCommand(SX126X_CMD_SET_CAD, &data, 0); // 0xC5
}


uint16_t GetIrqStatus( void )
{
	uint8_t data[3];
	ReadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 3); // 0x12
	return (data[1] << 8) | data[2];
}


void ClearIrqStatus(uint16_t irq)
{
	uint8_t buf[2];

	buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
	buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
	WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2); // 0x02
}


void SetRx(uint32_t timeout)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "----- SetRx timeout=%"PRIu32, timeout);
	}
	SetStandby(SX126X_STANDBY_RC);
	SetRxEnable();
	uint8_t buf[3];
	buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t)(timeout & 0xFF);
	WriteCommand(SX126X_CMD_SET_RX, buf, 3); // 0x82

	for(int retry=0;retry<10;retry++) {
		if ((GetStatus() & 0x70) == 0x50) break;
		delay(1);
	}
	if ((GetStatus() & 0x70) != 0x50) {
		ESP_LOGE(TAG, "SetRx Illegal Status");
		LoRaError(ERR_INVALID_SETRX_STATE);
	}
}


void SetRxEnable(void)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "SetRxEnable:SX126x_TXEN=%d SX126x_RXEN=%d", SX126x_TXEN, SX126x_RXEN);
	}
	if ((SX126x_TXEN != -1) && (SX126x_RXEN != -1)) {
		gpio_set_level(SX126x_RXEN, HIGH);
		gpio_set_level(SX126x_TXEN, LOW);
	}
}


void SetTx(uint32_t timeoutInMs)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "----- SetTx timeoutInMs=%"PRIu32, timeoutInMs);
	}
	SetStandby(SX126X_STANDBY_RC);
	SetTxEnable();
	uint8_t buf[3];
	uint32_t tout = timeoutInMs;
	if (timeoutInMs != 0) {
		uint32_t timeoutInUs = timeoutInMs * 1000;
		tout = (uint32_t)(timeoutInUs / 0.015625);
	}
	if (debugPrint) {
		ESP_LOGI(TAG, "SetTx timeoutInMs=%"PRIu32" tout=%"PRIu32, timeoutInMs, tout);
	}
	buf[0] = (uint8_t)((tout >> 16) & 0xFF);
	buf[1] = (uint8_t)((tout >> 8) & 0xFF);
	buf[2] = (uint8_t )(tout & 0xFF);
	WriteCommand(SX126X_CMD_SET_TX, buf, 3); // 0x83
	
	for(int retry=0;retry<10;retry++) {
		if ((GetStatus() & 0x70) == 0x60) break;
		vTaskDelay(1);
	}
	if ((GetStatus() & 0x70) != 0x60) {
		ESP_LOGE(TAG, "SetTx Illegal Status");
		LoRaError(ERR_INVALID_SETTX_STATE);
	}
}


void SetTxEnable(void)
{
	if (debugPrint) {
		ESP_LOGI(TAG, "SetTxEnable:SX126x_TXEN=%d SX126x_RXEN=%d", SX126x_TXEN, SX126x_RXEN);
	}
	if ((SX126x_TXEN != -1) && (SX126x_RXEN != -1)){
		gpio_set_level(SX126x_RXEN, LOW);
		gpio_set_level(SX126x_TXEN, HIGH);
	}
}


uint8_t GetRssiInst()
{
	uint8_t buf[2];
	ReadCommand( SX126X_CMD_GET_RSSI_INST, buf, 2 ); // 0x15
	return buf[1];
}


void GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer)
{
	uint8_t buf[3];
	ReadCommand( SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 3 ); // 0x13
	*payloadLength = buf[1];
	*rxStartBufferPointer = buf[2];
}


void WaitForIdle(unsigned long timeout)
{
    //unsigned long start = millis();
    TickType_t start = xTaskGetTickCount();
    delayMicroseconds(1);
    while(xTaskGetTickCount() - start < (timeout/portTICK_PERIOD_MS)) {
        if (gpio_get_level(SX126x_BUSY) == 0) break;
        delayMicroseconds(1);
    }
    if (gpio_get_level(SX126x_BUSY)) {
		if (debugPrint) {
        	ESP_LOGE(TAG, "WaitForIdle Timeout timeout=%lu", timeout);
		}
        LoRaError(ERR_IDLE_TIMEOUT);
    }
}


uint8_t ReadBuffer(uint8_t *rxData, uint8_t maxLen)
{
	uint8_t offset = 0;
	uint8_t payloadLength = 0;
	GetRxBufferStatus(&payloadLength, &offset);
	if( payloadLength > maxLen )
	{
		if (debugPrint) {
			ESP_LOGW(TAG, "ReadBuffer maxLen too small");
		}
		return 0;
	}

	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(SX126x_SPI_SELECT, LOW);

	spi_transfer(SX126X_CMD_READ_BUFFER); // 0x1E
	spi_transfer(offset);
	spi_transfer(SX126X_CMD_NOP);
	for( uint16_t i = 0; i < payloadLength; i++ )
	{
		rxData[i] = spi_transfer(SX126X_CMD_NOP);  
	}

	// stop transfer
	gpio_set_level(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);

	return payloadLength;
}


void WriteBuffer(uint8_t *txData, uint8_t txDataLen)
{
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(SX126x_SPI_SELECT, LOW);

	spi_transfer(SX126X_CMD_WRITE_BUFFER); // 0x0E
	spi_transfer(0); //offset in tx fifo
	for( uint16_t i = 0; i < txDataLen; i++ )
	{ 
		 spi_transfer( txData[i]);	
	}

	// stop transfer
	gpio_set_level(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
}


void WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes) 
{
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	if(debugPrint) {
		if (debugPrint) {
			ESP_LOGI(TAG, "WriteRegister: REG=0x%02x", reg);
		}
	}
	// start transfer
	gpio_set_level(SX126x_SPI_SELECT, LOW);

	// send command byte
	spi_transfer(SX126X_CMD_WRITE_REGISTER); // 0x0D
	spi_transfer((reg & 0xFF00) >> 8);
	spi_transfer(reg & 0xff);
	
	for(uint8_t n = 0; n < numBytes; n++) {
		uint8_t in = spi_transfer(data[n]);
		(void)in;
		if(debugPrint) {
			ESP_LOGI(TAG, "%02x --> %02x", data[n], in);
			//ESP_LOGI(TAG, "DataOut:%02x ", data[n]);
		}
	}

	// stop transfer
	gpio_set_level(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
#if 0
	if(waitForBusy) {
		WaitForIdle(BUSY_WAIT);
	}
#endif
}


void SetTxContinuousWave() 
{
	WriteCommand(0xD1, NULL, 0);
}

void ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes) 
{
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	if(debugPrint) {
		ESP_LOGI(TAG, "ReadRegister: REG=0x%02x", reg);
	}

	// start transfer
	gpio_set_level(SX126x_SPI_SELECT, LOW);

	// send command byte
	spi_transfer(SX126X_CMD_READ_REGISTER); // 0x1D
	spi_transfer((reg & 0xFF00) >> 8);
	spi_transfer(reg & 0xff);
	spi_transfer(SX126X_CMD_NOP);

	for(uint8_t n = 0; n < numBytes; n++) {
		data[n] = spi_transfer(SX126X_CMD_NOP);
		if(debugPrint) {
			ESP_LOGI(TAG, "DataIn:%02x ", data[n]);
		}
	}

	// stop transfer
	gpio_set_level(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
#if 0
	if(waitForBusy) {
		WaitForIdle(BUSY_WAIT);
	}
#endif
}

// WriteCommand with retry
void WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes) 
{
	uint8_t status;
	for (int retry=1; retry<10; retry++) {
		status = WriteCommand2(cmd, data, numBytes);
		if (debugPrint) {
			ESP_LOGD(TAG, "status=%02x", status);
		}
		if (status == 0) break;
		if (debugPrint) {
			ESP_LOGW(TAG, "WriteCommand2 status=%02x retry=%d", status, retry);
		}
	}
	if (status != 0) {
		if (debugPrint) {
			ESP_LOGE(TAG, "SPI Transaction error:0x%02x", status);
		}
		LoRaError(ERR_SPI_TRANSACTION);
	}
}

uint8_t WriteCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes) 
{
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(SX126x_SPI_SELECT, LOW);

	// send command byte
	if(debugPrint) {
		ESP_LOGI(TAG, "WriteCommand: CMD=0x%02x", cmd);
	}
	spi_transfer(cmd);

	// variable to save error during SPI transfer
	uint8_t status = 0;

	// send/receive all bytes
	for(uint8_t n = 0; n < numBytes; n++) {
		uint8_t in = spi_transfer(data[n]);
		if(debugPrint) {
			ESP_LOGI(TAG, "%02x --> %02x", data[n], in);
		}

		// check status
		if(((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
		 ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
		 ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
			status = in & 0b00001110;
			break;
		} else if(in == 0x00 || in == 0xFF) {
			status = SX126X_STATUS_SPI_FAILED;
			break;
		}
	} 

	// stop transfer
	gpio_set_level(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
#if 0
	if(waitForBusy) {
		WaitForIdle(BUSY_WAIT);
	}
#endif

#if 0
	if (status != 0) {
		ESP_LOGE(TAG, "SPI Transaction error:0x%02x", status);
		LoRaError(ERR_SPI_TRANSACTION);
	}
#endif
	return status;
}


void ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes) {
	// ensure BUSY is low (state meachine ready)
	WaitForIdle(BUSY_WAIT);

	// start transfer
	gpio_set_level(SX126x_SPI_SELECT, LOW);

	// send command byte
	if(debugPrint) {
		ESP_LOGI(TAG, "ReadCommand: CMD=0x%02x", cmd);
	}
	spi_transfer(cmd);

	// send/receive all bytes
	for(uint8_t n = 0; n < numBytes; n++) {
		data[n] = spi_transfer(SX126X_CMD_NOP);
		if(debugPrint) {
			ESP_LOGI(TAG, "DataIn:%02x", data[n]);
		}
	}

	// stop transfer
	gpio_set_level(SX126x_SPI_SELECT, HIGH);

	// wait for BUSY to go low
	WaitForIdle(BUSY_WAIT);
#if 0
	if(waitForBusy) {
		WaitForIdle(BUSY_WAIT);
	}
#endif
}


void sx126x_hal_wait_on_busy( const int busy_pin )
{
    //while( hal_gpio_get_value( busy_pin ) == HAL_GPIO_SET )
    while (gpio_get_level(busy_pin) == 1)
    {
		if (debugPrint) {
        	ESP_LOGI(TAG, "WaitForIdle ");
		}
    };
}

void sx126x_hal_check_device_ready( const sx126x_hal_context_t* sx126x_context )
{

	// Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
	gpio_set_level(CONFIG_NSS_GPIO, 0);
	sx126x_hal_wait_on_busy( CONFIG_BUSY_GPIO );
	gpio_set_level(CONFIG_NSS_GPIO, 1);
	radio_mode = RADIO_AWAKE;

}


sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ) {
	if (debugPrint) {
    	ESP_LOGI(TAG, "write");
	}
    // Get context
    const sx126x_hal_context_t* sx126x_context = ( const sx126x_hal_context_t* ) context;

    // Wait for readiness
    sx126x_hal_check_device_ready( sx126x_context );

    // Put NSS low to start spi transaction
	gpio_set_level(CONFIG_NSS_GPIO, HAL_GPIO_RESET);

    // send command byte
    for( uint16_t i = 0; i < command_length; i++) {
        spi_transfer(command[i]);
		if (debugPrint) {
        	ESP_LOGI(TAG, "writecommand %02x ", command[i]);
		}
    }
	for(uint8_t n = 0; n < data_length; n++) {
		uint8_t in = spi_transfer(data[n]);
		(void)in;
		if (debugPrint) {
        	ESP_LOGI(TAG, "%02x --> %02x", data[n], in);
		}
	}

	// stop transfer
	gpio_set_level(CONFIG_NSS_GPIO, 1);
    // Check whether the command is a sleep command to keep the state up to date
    /*
    if( ( command_length == 2 ) && ( command[0] == 0x84 ) )
    {
        radio_mode = RADIO_SLEEP;
    }*/
	sx126x_hal_wait_on_busy( CONFIG_BUSY_GPIO );

    return SX126X_HAL_STATUS_OK;
}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
	if (debugPrint) {
    	ESP_LOGI(TAG, "read");
	}
    const sx126x_hal_context_t* sx126x_context = ( const sx126x_hal_context_t* ) context;

    // Wait for readiness
    sx126x_hal_check_device_ready( sx126x_context );

    // Put NSS low to start spi transaction
    //hal_gpio_set_value( sx126x_context->nss, HAL_GPIO_RESET );
	gpio_set_level(CONFIG_NSS_GPIO, 0);

    //send command
    for( uint16_t i = 0; i < command_length; i++) {
        spi_transfer(command[i]);
	if (debugPrint) {
    	ESP_LOGI(TAG, "command:%02x ", command[i]);
	}
    }
    // Get read response bytes
    for(uint8_t n = 0; n < data_length; n++) {
		data[n] = spi_transfer(0x00);
		if (debugPrint) {
        	ESP_LOGI(TAG, "DataIn:%02x ", data[n]);
		}
	}
    // Finish SPI transaction
    //hal_gpio_set_value( sx126x_context->nss, HAL_GPIO_SET );
	gpio_set_level(CONFIG_NSS_GPIO, 1);


    return SX126X_HAL_STATUS_OK;
}


sx126x_hal_status_t sx126x_hal_wakeup( const void* context )
{
    const sx126x_hal_context_t* sx126x_context = ( const sx126x_hal_context_t* ) context;

    sx126x_hal_check_device_ready( sx126x_context );
    return SX126X_HAL_STATUS_OK;
}

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

void send_WUR()
{
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


void Listen_WUR()
{
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

#ifdef __cplusplus
}
#endif
