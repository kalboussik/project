/**
 * @file      sx1261_driver.h
 *
 * @brief     SX126x radio driver definition
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



#ifndef _DRIVER_H
#define _DRIVER_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "../../../config/config.h"

#define delay(ms) esp_rom_delay_us(ms*1000)


/**
 * @brief SX126X APIs return status enumeration definition
 */
typedef enum sx126x_status_e
{
    SX126X_STATUS_OK = 0,
    SX126X_STATUS_UNSUPPORTED_FEATURE,
    SX126X_STATUS_UNKNOWN_VALUE,
    SX126X_STATUS_ERROR,
} sx126x_status_t;

/**
 * @brief SX126X sleep mode configurations definition
 */
typedef enum sx126x_sleep_cfgs_e
{
    SX126X_SLEEP_CFG_COLD_START = ( 0 << 2 ),
    SX126X_SLEEP_CFG_WARM_START = ( 1 << 2 ),
} sx126x_sleep_cfgs_t;

/**
 * @brief SX126X standby modes enumeration definition
 */
typedef enum sx126x_standby_cfgs_e
{
    SX126X_STANDBY_CFG_RC   = 0x00,
    SX126X_STANDBY_CFG_XOSC = 0x01,
} sx126x_standby_cfgs_t;

typedef uint8_t sx126x_standby_cfg_t;

/**
 * @brief SX126X power regulator modes enumeration definition
 */
typedef enum sx126x_reg_mods_e
{
    SX126X_REG_MODE_LDO  = 0x00,  // default
    SX126X_REG_MODE_DCDC = 0x01,
} sx126x_reg_mod_t;

/**
 * @brief SX126X power amplifier configuration parameters structure definition
 */
typedef struct sx126x_pa_cfg_params_s
{
    uint8_t pa_duty_cycle;
    uint8_t hp_max;
    uint8_t device_sel;
    uint8_t pa_lut;
} sx126x_pa_cfg_params_t;

/**
 * @brief SX126X fallback modes enumeration definition
 */
typedef enum sx126x_fallback_modes_e
{
    SX126X_FALLBACK_STDBY_RC   = 0x20,
    SX126X_FALLBACK_STDBY_XOSC = 0x30,
    SX126X_FALLBACK_FS         = 0x40,
} sx126x_fallback_modes_t;

/**
 * @brief SX126X interrupt masks enumeration definition
 */
enum sx126x_irq_masks_e
{
    SX126X_IRQ_NONE              = ( 0 << 0 ),
    SX126X_IRQ_TX_DONE           = ( 1 << 0 ),
    SX126X_IRQ_RX_DONE           = ( 1 << 1 ),
    SX126X_IRQ_PREAMBLE_DETECTED = ( 1 << 2 ),
    SX126X_IRQ_SYNC_WORD_VALID   = ( 1 << 3 ),
    SX126X_IRQ_HEADER_VALID      = ( 1 << 4 ),
    SX126X_IRQ_HEADER_ERROR      = ( 1 << 5 ),
    SX126X_IRQ_CRC_ERROR         = ( 1 << 6 ),
    SX126X_IRQ_CAD_DONE          = ( 1 << 7 ),
    SX126X_IRQ_CAD_DETECTED      = ( 1 << 8 ),
    SX126X_IRQ_TIMEOUT           = ( 1 << 9 ),
    SX126X_IRQ_LR_FHSS_HOP       = ( 1 << 14 ),
    SX126X_IRQ_ALL               = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                     SX126X_IRQ_SYNC_WORD_VALID | SX126X_IRQ_HEADER_VALID | SX126X_IRQ_HEADER_ERROR |
                     SX126X_IRQ_CRC_ERROR | SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED | SX126X_IRQ_TIMEOUT |
                     SX126X_IRQ_LR_FHSS_HOP,
};

typedef uint16_t sx126x_irq_mask_t;

/**
 * @brief Calibration settings
 */
enum sx126x_cal_mask_e{
    SX126X_CAL_RC64K      = ( 1 << 0 ),
    SX126X_CAL_RC13M      = ( 1 << 1 ),
    SX126X_CAL_PLL        = ( 1 << 2 ),
    SX126X_CAL_ADC_PULSE  = ( 1 << 3 ),
    SX126X_CAL_ADC_BULK_N = ( 1 << 4 ),
    SX126X_CAL_ADC_BULK_P = ( 1 << 5 ),
    SX126X_CAL_IMAGE      = ( 1 << 6 ),
    SX126X_CAL_ALL        = SX126X_CAL_RC64K | SX126X_CAL_RC13M | SX126X_CAL_PLL | SX126X_CAL_ADC_PULSE |
                     SX126X_CAL_ADC_BULK_N | SX126X_CAL_ADC_BULK_P | SX126X_CAL_IMAGE,
};

typedef uint8_t sx126x_cal_mask_t;

/**
 * @brief SX126X TCXO control voltages enumeration definition
 */
typedef enum sx126x_tcxo_ctrl_voltages_e
{
    SX126X_TCXO_CTRL_1_6V = 0x00,
    SX126X_TCXO_CTRL_1_7V = 0x01,
    SX126X_TCXO_CTRL_1_8V = 0x02,
    SX126X_TCXO_CTRL_2_2V = 0x03,
    SX126X_TCXO_CTRL_2_4V = 0x04,
    SX126X_TCXO_CTRL_2_7V = 0x05,
    SX126X_TCXO_CTRL_3_0V = 0x06,
    SX126X_TCXO_CTRL_3_3V = 0x07,
} sx126x_tcxo_ctrl_voltages_t;

/**
 * @brief SX126X packet types enumeration definition
 */
typedef enum sx126x_pkt_types_e
{
    SX126X_PKT_TYPE_GFSK    = 0x00,
    SX126X_PKT_TYPE_LORA    = 0x01,
    SX126X_PKT_TYPE_LR_FHSS = 0x03,
} sx126x_pkt_type_t;

/**
 * @brief SX126X power amplifier ramp-up timings enumeration definition
 */
typedef enum sx126x_ramp_time_e
{
    SX126X_RAMP_10_US   = 0x00,
    SX126X_RAMP_20_US   = 0x01,
    SX126X_RAMP_40_US   = 0x02,
    SX126X_RAMP_80_US   = 0x03,
    SX126X_RAMP_200_US  = 0x04,
    SX126X_RAMP_800_US  = 0x05,
    SX126X_RAMP_1700_US = 0x06,
    SX126X_RAMP_3400_US = 0x07,
} sx126x_ramp_time_t;

/**
 * @brief SX126X GFSK modulation shaping enumeration definition
 */
typedef enum sx126x_gfsk_pulse_shape_e
{
    SX126X_GFSK_PULSE_SHAPE_OFF   = 0x00,
    SX126X_GFSK_PULSE_SHAPE_BT_03 = 0x08,
    SX126X_GFSK_PULSE_SHAPE_BT_05 = 0x09,
    SX126X_GFSK_PULSE_SHAPE_BT_07 = 0x0A,
    SX126X_GFSK_PULSE_SHAPE_BT_1  = 0x0B,
} sx126x_gfsk_pulse_shape_t;

/**
 * @brief SX126X GFSK Rx bandwidth enumeration definition
 */
typedef enum sx126x_gfsk_bw_e
{
    SX126X_GFSK_BW_4800   = 0x1F,
    SX126X_GFSK_BW_5800   = 0x17,
    SX126X_GFSK_BW_7300   = 0x0F,
    SX126X_GFSK_BW_9700   = 0x1E,
    SX126X_GFSK_BW_11700  = 0x16,
    SX126X_GFSK_BW_14600  = 0x0E,
    SX126X_GFSK_BW_19500  = 0x1D,
    SX126X_GFSK_BW_23400  = 0x15,
    SX126X_GFSK_BW_29300  = 0x0D,
    SX126X_GFSK_BW_39000  = 0x1C,
    SX126X_GFSK_BW_46900  = 0x14,
    SX126X_GFSK_BW_58600  = 0x0C,
    SX126X_GFSK_BW_78200  = 0x1B,
    SX126X_GFSK_BW_93800  = 0x13,
    SX126X_GFSK_BW_117300 = 0x0B,
    SX126X_GFSK_BW_156200 = 0x1A,
    SX126X_GFSK_BW_187200 = 0x12,
    SX126X_GFSK_BW_234300 = 0x0A,
    SX126X_GFSK_BW_312000 = 0x19,
    SX126X_GFSK_BW_373600 = 0x11,
    SX126X_GFSK_BW_467000 = 0x09,
} sx126x_gfsk_bw_t;

/**
 * @brief SX126X GFSK modulation parameters structure definition
 */
typedef struct sx126x_mod_params_gfsk_s
{
    uint32_t                  br_in_bps;
    uint32_t                  fdev_in_hz;
    sx126x_gfsk_pulse_shape_t pulse_shape;
    sx126x_gfsk_bw_t          bw_dsb_param;
} sx126x_mod_params_gfsk_t;

/**
 * @brief SX126X LoRa spreading factor enumeration definition
 */
typedef enum sx126x_lora_sf_e
{
    SX126X_LORA_SF5  = 0x05,
    SX126X_LORA_SF6  = 0x06,
    SX126X_LORA_SF7  = 0x07,
    SX126X_LORA_SF8  = 0x08,
    SX126X_LORA_SF9  = 0x09,
    SX126X_LORA_SF10 = 0x0A,
    SX126X_LORA_SF11 = 0x0B,
    SX126X_LORA_SF12 = 0x0C,
} sx126x_lora_sf_t;

/**
 * @brief SX126X LoRa bandwidth enumeration definition
 */
typedef enum sx126x_lora_bw_e
{
    SX126X_LORA_BW_500 = 6,
    SX126X_LORA_BW_250 = 5,
    SX126X_LORA_BW_125 = 4,
    SX126X_LORA_BW_062 = 3,
    SX126X_LORA_BW_041 = 10,
    SX126X_LORA_BW_031 = 2,
    SX126X_LORA_BW_020 = 9,
    SX126X_LORA_BW_015 = 1,
    SX126X_LORA_BW_010 = 8,
    SX126X_LORA_BW_007 = 0,
} sx126x_lora_bw_t;

/**
 * @brief SX126X LoRa coding rate enumeration definition
 */
typedef enum sx126x_lora_cr_e
{
    SX126X_LORA_CR_4_5 = 0x01,
    SX126X_LORA_CR_4_6 = 0x02,
    SX126X_LORA_CR_4_7 = 0x03,
    SX126X_LORA_CR_4_8 = 0x04,
} sx126x_lora_cr_t;

/**
 * @brief SX126X LoRa modulation parameters structure definition
 */
typedef struct sx126x_mod_params_lora_s
{
    sx126x_lora_sf_t sf;    //!< LoRa Spreading Factor
    sx126x_lora_bw_t bw;    //!< LoRa Bandwidth
    sx126x_lora_cr_t cr;    //!< LoRa Coding Rate
    uint8_t          ldro;  //!< Low DataRate Optimization configuration
} sx126x_mod_params_lora_t;

/**
 * @brief SX126X GFSK preamble length Rx detection size enumeration definition
 */
typedef enum sx126x_gfsk_preamble_detector_e
{
    SX126X_GFSK_PREAMBLE_DETECTOR_OFF        = 0x00,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS  = 0x04,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS = 0x05,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS = 0x06,
    SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS = 0x07,
} sx126x_gfsk_preamble_detector_t;

/**
 * @brief SX126X GFSK address filtering configuration enumeration definition
 */
typedef enum sx126x_gfsk_address_filtering_e
{
    SX126X_GFSK_ADDRESS_FILTERING_DISABLE                      = 0x00,
    SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS                 = 0x01,
    SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES = 0x02,
} sx126x_gfsk_address_filtering_t;

/**
 * @brief SX126X GFSK packet length enumeration definition
 */
typedef enum sx126x_gfsk_pkt_len_modes_e
{
    SX126X_GFSK_PKT_FIX_LEN = 0x00,  //!< The packet length is known on both sides, no header included
    SX126X_GFSK_PKT_VAR_LEN = 0x01,  //!< The packet length is variable, header included
} sx126x_gfsk_pkt_len_modes_t;

/**
 * @brief SX126X GFSK CRC type enumeration definition
 */
typedef enum sx126x_gfsk_crc_types_e
{
    SX126X_GFSK_CRC_OFF         = 0x01,
    SX126X_GFSK_CRC_1_BYTE      = 0x00,
    SX126X_GFSK_CRC_2_BYTES     = 0x02,
    SX126X_GFSK_CRC_1_BYTE_INV  = 0x04,
    SX126X_GFSK_CRC_2_BYTES_INV = 0x06,
} sx126x_gfsk_crc_types_t;

/**
 * @brief SX126X GFSK whitening control enumeration definition
 */
typedef enum sx126x_gfsk_dc_free_e
{
    SX126X_GFSK_DC_FREE_OFF       = 0x00,
    SX126X_GFSK_DC_FREE_WHITENING = 0x01,
} sx126x_gfsk_dc_free_t;

/**
 * @brief SX126X LoRa packet length enumeration definition
 */
typedef enum sx126x_lora_pkt_len_modes_e
{
    SX126X_LORA_PKT_EXPLICIT = 0x00,  //!< Header included in the packet
    SX126X_LORA_PKT_IMPLICIT = 0x01,  //!< Header not included in the packet
} sx126x_lora_pkt_len_modes_t;

/**
 * @brief SX126X LoRa packet parameters structure definition
 */
typedef struct sx126x_pkt_params_lora_s
{
    uint16_t                    preamble_len_in_symb;  //!< Preamble length in symbols
    sx126x_lora_pkt_len_modes_t header_type;           //!< Header type
    uint8_t                     pld_len_in_bytes;      //!< Payload length in bytes
    bool                        crc_is_on;             //!< CRC activation
    bool                        invert_iq_is_on;       //!< IQ polarity setup
} sx126x_pkt_params_lora_t;

/**
 * @brief SX126X GFSK packet parameters structure definition
 */
typedef struct sx126x_pkt_params_gfsk_s
{
    uint16_t                        preamble_len_in_bits;   //!< Preamble length in bits
    sx126x_gfsk_preamble_detector_t preamble_detector;      //!< Preamble detection length
    uint8_t                         sync_word_len_in_bits;  //!< Sync word length in bits
    sx126x_gfsk_address_filtering_t address_filtering;      //!< Address filtering configuration
    sx126x_gfsk_pkt_len_modes_t     header_type;            //!< Header type
    uint8_t                         pld_len_in_bytes;       //!< Payload length in bytes
    sx126x_gfsk_crc_types_t         crc_type;               //!< CRC type configuration
    sx126x_gfsk_dc_free_t           dc_free;                //!< Whitening configuration
} sx126x_pkt_params_gfsk_t;

/**
 * @brief SX126X LoRa CAD number of symbols enumeration definition
 *
 * @note Represents the number of symbols to be used for a CAD operation
 */
typedef enum sx126x_cad_symbs_e
{
    SX126X_CAD_01_SYMB = 0x00,
    SX126X_CAD_02_SYMB = 0x01,
    SX126X_CAD_04_SYMB = 0x02,
    SX126X_CAD_08_SYMB = 0x03,
    SX126X_CAD_16_SYMB = 0x04,
} sx126x_cad_symbs_t;

/**
 * @brief SX126X LoRa CAD exit modes enumeration definition
 *
 * @note Represents the action to be performed after a CAD is done
 */
typedef enum sx126x_cad_exit_modes_e
{
    SX126X_CAD_ONLY = 0x00,
    SX126X_CAD_RX   = 0x01,
    SX126X_CAD_LBT  = 0x10,
} sx126x_cad_exit_modes_t;

/**
 * @brief SX126X CAD parameters structure definition
 */
typedef struct sx126x_cad_param_s
{
    sx126x_cad_symbs_t      cad_symb_nb;      //!< CAD number of symbols
    uint8_t                 cad_detect_peak;  //!< CAD peak detection
    uint8_t                 cad_detect_min;   //!< CAD minimum detection
    sx126x_cad_exit_modes_t cad_exit_mode;    //!< CAD exit mode
    uint32_t                cad_timeout;      //!< CAD timeout value
} sx126x_cad_params_t;

/**
 * @brief SX126X chip mode enumeration definition
 */
typedef enum sx126x_chip_modes_e
{
    SX126X_CHIP_MODE_UNUSED    = 0,
    SX126X_CHIP_MODE_RFU       = 1,
    SX126X_CHIP_MODE_STBY_RC   = 2,
    SX126X_CHIP_MODE_STBY_XOSC = 3,
    SX126X_CHIP_MODE_FS        = 4,
    SX126X_CHIP_MODE_RX        = 5,
    SX126X_CHIP_MODE_TX        = 6,
} sx126x_chip_modes_t;

/**
 * @brief SX126X command status enumeration definition
 */
typedef enum sx126x_cmd_status_e
{
    SX126X_CMD_STATUS_RESERVED          = 0,
    SX126X_CMD_STATUS_RFU               = 1,
    SX126X_CMD_STATUS_DATA_AVAILABLE    = 2,
    SX126X_CMD_STATUS_CMD_TIMEOUT       = 3,
    SX126X_CMD_STATUS_CMD_PROCESS_ERROR = 4,
    SX126X_CMD_STATUS_CMD_EXEC_FAILURE  = 5,
    SX126X_CMD_STATUS_CMD_TX_DONE       = 6,
} sx126x_cmd_status_t;

/**
 * @brief SX126X chip status structure definition
 */
typedef struct sx126x_chip_status_s
{
    sx126x_cmd_status_t cmd_status;  //!< Previous command status
    sx126x_chip_modes_t chip_mode;   //!< Current chip mode
} sx126x_chip_status_t;

/**
 * @brief SX126X RX buffer status structure definition
 */
typedef struct sx126x_rx_buffer_status_s
{
    uint8_t pld_len_in_bytes;      //!< Number of bytes available in the buffer
    uint8_t buffer_start_pointer;  //!< Position of the first byte in the buffer
} sx126x_rx_buffer_status_t;

typedef struct sx126x_rx_status_gfsk_s
{
    bool pkt_sent;
    bool pkt_received;
    bool abort_error;
    bool length_error;
    bool crc_error;
    bool adrs_error;
} sx126x_rx_status_gfsk_t;

/**
 * @brief SX126X GFSK packet status structure definition
 */
typedef struct sx126x_pkt_status_gfsk_s
{
    sx126x_rx_status_gfsk_t rx_status;
    int8_t                  rssi_sync;  //!< The RSSI measured on last packet
    int8_t                  rssi_avg;   //!< The averaged RSSI
} sx126x_pkt_status_gfsk_t;

/**
 * @brief SX126X LoRa packet status structure definition
 */
typedef struct sx126x_pkt_status_lora_s
{
    int8_t rssi_pkt_in_dbm;         //!< RSSI of the last packet
    int8_t snr_pkt_in_db;           //!< SNR of the last packet
    int8_t signal_rssi_pkt_in_dbm;  //!< Estimation of RSSI (after despreading)
} sx126x_pkt_status_lora_t;

/**
 * @brief SX126X GFSK reception statistics structure definition
 */
typedef struct sx126x_stats_gfsk_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_len_error;
} sx126x_stats_gfsk_t;

/**
 * @brief SX126X LoRa reception statistics structure definition
 */
typedef struct sx126x_stats_lora_s
{
    uint16_t nb_pkt_received;
    uint16_t nb_pkt_crc_error;
    uint16_t nb_pkt_header_error;
} sx126x_stats_lora_t;

/**
 * @brief SX126X errors enumeration definition
 */
enum sx126x_errors_e{
    SX126X_ERRORS_RC64K_CALIBRATION = ( 1 << 0 ),
    SX126X_ERRORS_RC13M_CALIBRATION = ( 1 << 1 ),
    SX126X_ERRORS_PLL_CALIBRATION   = ( 1 << 2 ),
    SX126X_ERRORS_ADC_CALIBRATION   = ( 1 << 3 ),
    SX126X_ERRORS_IMG_CALIBRATION   = ( 1 << 4 ),
    SX126X_ERRORS_XOSC_START        = ( 1 << 5 ),
    SX126X_ERRORS_PLL_LOCK          = ( 1 << 6 ),
    SX126X_ERRORS_PA_RAMP           = ( 1 << 8 ),
};

typedef uint16_t sx126x_errors_mask_t;


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


typedef enum sx126x_hal_status_e
{
    SX126X_HAL_STATUS_OK    = 0,
    SX126X_HAL_STATUS_ERROR = 3,
} sx126x_hal_status_t;

typedef struct
{
    uint8_t nss;
    uint8_t busy;
    uint8_t reset;
    uint32_t spi_id;
} sx126x_hal_context_t;











/**
 * @brief The address of the register holding the first byte defining the CRC seed
 */
#define SX126X_REG_CRCSEEDBASEADDRESS 0x06BC

/**
 * @brief The address of the register holding the first byte defining the CRC polynomial
 */
#define SX126X_REG_CRCPOLYBASEADDRESS 0x06BE

/**
 * @brief The address of the register holding the first byte defining the whitening seed
 */
#define SX126X_REG_WHITSEEDBASEADDRESS 0x06B8

/**
 * @brief The addresses of the registers holding SyncWords values
 */
#define SX126X_REG_SYNCWORDBASEADDRESS 0x06C0

/**
 * @brief The addresses of the register holding LoRa Modem SyncWord value
 *        0x1424: LoRaWAN private network,
 *        0x3444: LoRaWAN public network
 */
#define SX126X_REG_LR_SYNCWORD 0x0740

/**
 * @brief The address of the register holding the coding rate configuration extracted from a received LoRa header
 */
#define SX126X_REG_LR_HEADER_CR 0x0749
#define SX126X_REG_LR_HEADER_CR_POS ( 4U )
#define SX126X_REG_LR_HEADER_CR_MASK ( 0x07UL << SX126X_REG_LR_HEADER_CR_POS )

/**
 * @brief The address of the register holding the CRC configuration extracted from a received LoRa header
 */
#define SX126X_REG_LR_HEADER_CRC 0x076B
#define SX126X_REG_LR_HEADER_CRC_POS ( 4U )
#define SX126X_REG_LR_HEADER_CRC_MASK ( 0x01UL << SX126X_REG_LR_HEADER_CRC_POS )

/*!
 * The address of the register giving a 32-bit random number
 */
#define SX126X_REG_RNGBASEADDRESS 0x0819

/*!
 * The address of the register used to disable the LNA
 */
#define SX126X_REG_ANA_LNA 0x08E2

/*!
 * The address of the register used to disable the mixer
 */
#define SX126X_REG_ANA_MIXER 0x08E5

/*!
 * The address of the register holding RX Gain value
 *     0x94: power saving,
 *     0x96: rx boosted
 */
#define SX126X_REG_RXGAIN 0x08AC

/**
 * @brief Change the value on the device internal trimming capacitor
 */
#define SX126X_REG_XTATRIM 0x0911

/**
 * @brief Set the current max value in the over current protection
 */
#define SX126X_REG_OCP 0x08E7

/**
 * @brief WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
 */
#define SX126X_REG_IQ_POLARITY 0x0736

/**
 * @brief WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
 */
#define SX126X_REG_TX_MODULATION 0x0889

/**
 * @brief WORKAROUND - Better resistance to antenna mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
 */
#define SX126X_REG_TX_CLAMP_CFG 0x08D8
#define SX126X_REG_TX_CLAMP_CFG_POS ( 1U )
#define SX126X_REG_TX_CLAMP_CFG_MASK ( 0x0FUL << SX126X_REG_TX_CLAMP_CFG_POS )

/**
 * @brief RTC control
 */
#define SX126X_REG_RTC_CTRL 0x0902

/**
 * @brief Event clear
 */
#define SX126X_REG_EVT_CLR 0x0944
#define SX126X_REG_EVT_CLR_TIMEOUT_POS ( 1U )
#define SX126X_REG_EVT_CLR_TIMEOUT_MASK ( 0x01UL << SX126X_REG_EVT_CLR_TIMEOUT_POS )

/**
 * @brief RX address pointer
 */
#define SX126X_REG_RX_ADDRESS_POINTER 0x0803

/**
 * @brief RX/TX payload length
 */
#define SX126X_REG_RXTX_PAYLOAD_LEN 0x06BB

/**
 * @brief Output disable
 */
#define SX126X_REG_OUT_DIS_REG 0x0580
#define SX126X_REG_OUT_DIS_REG_DIO3_POS ( 3U )
#define SX126X_REG_OUT_DIS_REG_DIO3_MASK ( 0x01UL << SX126X_REG_OUT_DIS_REG_DIO3_POS )

/**
 * @brief Input enable
 */
#define SX126X_REG_IN_EN_REG 0x0583
#define SX126X_REG_IN_EN_REG_DIO3_POS ( 3U )
#define SX126X_REG_IN_EN_REG_DIO3_MASK ( 0x01UL << SX126X_REG_IN_EN_REG_DIO3_POS )

/**
 * @brief TX bitbang A
 */
#define SX126X_REG_BITBANG_A_REG 0x0680
#define SX126X_REG_BITBANG_A_REG_ENABLE_POS ( 4U )
#define SX126X_REG_BITBANG_A_REG_ENABLE_MASK ( 0x07UL << SX126X_REG_BITBANG_A_REG_ENABLE_POS )
#define SX126X_REG_BITBANG_A_REG_ENABLE_VAL ( 0x01UL << SX126X_REG_BITBANG_A_REG_ENABLE_POS )

/**
 * @brief TX bitbang B
 */
#define SX126X_REG_BITBANG_B_REG 0x0587
#define SX126X_REG_BITBANG_B_REG_ENABLE_POS ( 0U )
#define SX126X_REG_BITBANG_B_REG_ENABLE_MASK ( 0x0FUL << SX126X_REG_BITBANG_B_REG_ENABLE_POS )
#define SX126X_REG_BITBANG_B_REG_ENABLE_VAL ( 0x0CUL << SX126X_REG_BITBANG_B_REG_ENABLE_POS )

/**
 * @brief Number of symbols given as SX126X_REG_LR_SYNCH_TIMEOUT[7:3] * 2 ^ (2*SX126X_REG_LR_SYNCH_TIMEOUT[2:0] + 1)
 */
#define SX126X_REG_LR_SYNCH_TIMEOUT 0x0706

/**
 * @brief Base address of the register retention list
 */
#define SX126X_REG_RETENTION_LIST_BASE_ADDRESS 0x029F

//return values
#define ERR_NONE                        0
#define ERR_PACKET_TOO_LONG             1
#define ERR_UNKNOWN                     2
#define ERR_TX_TIMEOUT                  3
#define ERR_RX_TIMEOUT                  4
#define ERR_CRC_MISMATCH                5
#define ERR_WRONG_MODEM                 6
#define ERR_INVALID_BANDWIDTH           7
#define ERR_INVALID_SPREADING_FACTOR    8
#define ERR_INVALID_CODING_RATE         9
#define ERR_INVALID_FREQUENCY_DEVIATION 10
#define ERR_INVALID_BIT_RATE            11
#define ERR_INVALID_RX_BANDWIDTH        12
#define ERR_INVALID_DATA_SHAPING        13
#define ERR_INVALID_SYNC_WORD           14
#define ERR_INVALID_OUTPUT_POWER        15
#define ERR_INVALID_MODE                16
#define ERR_INVALID_TRANCEIVER          17
#define ERR_INVALID_SETRX_STATE         18
#define ERR_INVALID_SETTX_STATE         19
#define ERR_IDLE_TIMEOUT                20
#define ERR_SPI_TRANSACTION             21

// SX126X physical layer properties
#define XTAL_FREQ                       ( double )32000000
#define FREQ_DIV                        ( double )pow( 2.0, 25.0 )
#define FREQ_STEP                       ( double )( XTAL_FREQ / FREQ_DIV )

#define LOW                             0
#define HIGH                            1
#define BUSY_WAIT                       5000

// SX126X Model
#define SX1261_TRANCEIVER                             0x01
#define SX1262_TRANCEIVER                             0x02
#define SX1268_TRANCEIVER                             0x08

// SX126X SPI commands
// operational modes commands
#define SX126X_CMD_NOP                                0x00
#define SX126X_CMD_SET_SLEEP                          0x84
#define SX126X_CMD_SET_STANDBY                        0x80
#define SX126X_CMD_SET_FS                             0xC1
#define SX126X_CMD_SET_TX                             0x83
#define SX126X_CMD_SET_RX                             0x82
#define SX126X_CMD_STOP_TIMER_ON_PREAMBLE             0x9F
#define SX126X_CMD_SET_RX_DUTY_CYCLE                  0x94
#define SX126X_CMD_SET_CAD                            0xC5
#define SX126X_CMD_SET_TX_CONTINUOUS_WAVE             0xD1
#define SX126X_CMD_SET_TX_INFINITE_PREAMBLE           0xD2
#define SX126X_CMD_SET_REGULATOR_MODE                 0x96
#define SX126X_CMD_CALIBRATE                          0x89
#define SX126X_CMD_CALIBRATE_IMAGE                    0x98
#define SX126X_CMD_SET_PA_CONFIG                      0x95
#define SX126X_CMD_SET_RX_TX_FALLBACK_MODE            0x93

// register and buffer access commands
#define SX126X_CMD_WRITE_REGISTER                     0x0D
#define SX126X_CMD_READ_REGISTER                      0x1D
#define SX126X_CMD_WRITE_BUFFER                       0x0E
#define SX126X_CMD_READ_BUFFER                        0x1E

// DIO and IRQ control
#define SX126X_CMD_SET_DIO_IRQ_PARAMS                 0x08
#define SX126X_CMD_GET_IRQ_STATUS                     0x12
#define SX126X_CMD_CLEAR_IRQ_STATUS                   0x02
#define SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL         0x9D
#define SX126X_CMD_SET_DIO3_AS_TCXO_CTRL              0x97

// RF, modulation and packet commands
#define SX126X_CMD_SET_RF_FREQUENCY                   0x86
#define SX126X_CMD_SET_PACKET_TYPE                    0x8A
#define SX126X_CMD_GET_PACKET_TYPE                    0x11
#define SX126X_CMD_SET_TX_PARAMS                      0x8E
#define SX126X_CMD_SET_MODULATION_PARAMS              0x8B
#define SX126X_CMD_SET_PACKET_PARAMS                  0x8C
#define SX126X_CMD_SET_CAD_PARAMS                     0x88
#define SX126X_CMD_SET_BUFFER_BASE_ADDRESS            0x8F
#define SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT          0xA0

#define SX126X_PA_CONFIG_SX1261                       0x01
#define SX126X_PA_CONFIG_SX1262                       0x00

// status commands
#define SX126X_CMD_GET_STATUS                         0xC0
#define SX126X_CMD_GET_RSSI_INST                      0x15
#define SX126X_CMD_GET_RX_BUFFER_STATUS               0x13
#define SX126X_CMD_GET_PACKET_STATUS                  0x14
#define SX126X_CMD_GET_DEVICE_ERRORS                  0x17
#define SX126X_CMD_CLEAR_DEVICE_ERRORS                0x07
#define SX126X_CMD_GET_STATS                          0x10
#define SX126X_CMD_RESET_STATS                        0x00


// SX126X register map
#define SX126X_REG_HOPPING_ENABLE                     0x0385
#define SX126X_REG_PACKECT_LENGTH                     0x0386
#define SX126X_REG_NB_HOPPING_BLOCKS                  0x0387
#define SX126X_REG_NB_SYMBOLS0                        0x0388
#define SX126X_REG_FREQ0                              0x038A
#define SX126X_REG_NB_SYMBOLS15                       0x03E2
#define SX126X_REG_FREQ15                             0x03E4
#define SX126X_REG_DIOX_OUTPUT_ENABLE                 0x0580
#define SX126X_REG_DIOX_INPUT_ENABLE                  0x0583
#define SX126X_REG_DIOX_PILL_UP_CONTROL               0x0584
#define SX126X_REG_DIOX_PULL_DOWN_CONTROL             0x0585
#define SX126X_REG_WHITENING_INITIAL_MSB              0x06B8
#define SX126X_REG_WHITENING_INITIAL_LSB              0x06B9
#define SX126X_REG_CRC_INITIAL_MSB                    0x06BC
#define SX126X_REG_CRC_INITIAL_LSB                    0x06BD
#define SX126X_REG_CRC_POLYNOMIAL_MSB                 0x06BE
#define SX126X_REG_CRC_POLYNOMIAL_LSB                 0x06BF
#define SX126X_REG_SYNC_WORD_0                        0x06C0
#define SX126X_REG_SYNC_WORD_1                        0x06C1
#define SX126X_REG_SYNC_WORD_2                        0x06C2
#define SX126X_REG_SYNC_WORD_3                        0x06C3
#define SX126X_REG_SYNC_WORD_4                        0x06C4
#define SX126X_REG_SYNC_WORD_5                        0x06C5
#define SX126X_REG_SYNC_WORD_6                        0x06C6
#define SX126X_REG_SYNC_WORD_7                        0x06C7
#define SX126X_REG_NODE_ADDRESS                       0x06CD
#define SX126X_REG_BROADCAST_ADDRESS                  0x06CE
#define SX126X_REG_IQ_POLARITY_SETUP                  0x0736
#define SX126X_REG_LORA_SYNC_WORD_MSB                 0x0740
#define SX126X_REG_LORA_SYNC_WORD_LSB                 0x0741
#define SX126X_REG_RANDOM_NUMBER_0                    0x0819
#define SX126X_REG_RANDOM_NUMBER_1                    0x081A
#define SX126X_REG_RANDOM_NUMBER_2                    0x081B
#define SX126X_REG_RANDOM_NUMBER_3                    0x081C
#define SX126X_REG_TX_MODULETION                      0x0889
#define SX126X_REG_RX_GAIN                            0x08AC
#define SX126X_REG_TX_CLAMP_CONFIG                    0x08D8
#define SX126X_REG_OCP_CONFIGURATION                  0x08E7
#define SX126X_REG_RTC_CONTROL                        0x0902
#define SX126X_REG_XTA_TRIM                           0x0911
#define SX126X_REG_XTB_TRIM                           0x0912
#define SX126X_REG_DIO3_OUTPUT_VOLTAGE_CONTROL        0x0920
#define SX126X_REG_EVENT_MASK                         0x0944


// SX126X SPI command variables
//SX126X_CMD_SET_SLEEP
#define SX126X_SLEEP_START_COLD                       0b00000000  //  2     2     sleep mode: cold start, configuration is lost (default)
#define SX126X_SLEEP_START_WARM                       0b00000100  //  2     2                 warm start, configuration is retained
#define SX126X_SLEEP_RTC_OFF                          0b00000000  //  0     0     wake on RTC timeout: disabled
#define SX126X_SLEEP_RTC_ON                           0b00000001  //  0     0                          enabled

//SX126X_CMD_SET_STANDBY
#define SX126X_STANDBY_RC                             0x00        //  7     0     standby mode: 13 MHz RC oscillator
#define SX126X_STANDBY_XOSC                           0x01        //  7     0                   32 MHz crystal oscillator

//SX126X_CMD_SET_RX
#define SX126X_RX_TIMEOUT_NONE                        0x000000    //  23    0     Rx timeout duration: no timeout (Rx single mode)
#define SX126X_RX_TIMEOUT_INF                         0xFFFFFF    //  23    0                          infinite (Rx continuous mode)

//SX126X_CMD_STOP_TIMER_ON_PREAMBLE
#define SX126X_STOP_ON_PREAMBLE_OFF                   0x00        //  7     0     stop timer on: sync word or header (default)
#define SX126X_STOP_ON_PREAMBLE_ON                    0x01        //  7     0                    preamble detection

//SX126X_CMD_SET_REGULATOR_MODE
#define SX126X_REGULATOR_LDO                          0x00        //  7     0     set regulator mode: LDO (default)
#define SX126X_REGULATOR_DC_DC                        0x01        //  7     0                         DC-DC

//SX126X_CMD_CALIBRATE
#define SX126X_CALIBRATE_IMAGE_OFF                    0b00000000  //  6     6     image calibration: disabled
#define SX126X_CALIBRATE_IMAGE_ON                     0b01000000  //  6     6                        enabled
#define SX126X_CALIBRATE_ADC_BULK_P_OFF               0b00000000  //  5     5     ADC bulk P calibration: disabled
#define SX126X_CALIBRATE_ADC_BULK_P_ON                0b00100000  //  5     5                             enabled
#define SX126X_CALIBRATE_ADC_BULK_N_OFF               0b00000000  //  4     4     ADC bulk N calibration: disabled
#define SX126X_CALIBRATE_ADC_BULK_N_ON                0b00010000  //  4     4                             enabled
#define SX126X_CALIBRATE_ADC_PULSE_OFF                0b00000000  //  3     3     ADC pulse calibration: disabled
#define SX126X_CALIBRATE_ADC_PULSE_ON                 0b00001000  //  3     3                            enabled
#define SX126X_CALIBRATE_PLL_OFF                      0b00000000  //  2     2     PLL calibration: disabled
#define SX126X_CALIBRATE_PLL_ON                       0b00000100  //  2     2                      enabled
#define SX126X_CALIBRATE_RC13M_OFF                    0b00000000  //  1     1     13 MHz RC osc. calibration: disabled
#define SX126X_CALIBRATE_RC13M_ON                     0b00000010  //  1     1                                 enabled
#define SX126X_CALIBRATE_RC64K_OFF                    0b00000000  //  0     0     64 kHz RC osc. calibration: disabled
#define SX126X_CALIBRATE_RC64K_ON                     0b00000001  //  0     0                                 enabled

//SX126X_CMD_CALIBRATE_IMAGE
#define SX126X_CAL_IMG_430_MHZ_1                      0x6B
#define SX126X_CAL_IMG_430_MHZ_2                      0x6F
#define SX126X_CAL_IMG_470_MHZ_1                      0x75
#define SX126X_CAL_IMG_470_MHZ_2                      0x81
#define SX126X_CAL_IMG_779_MHZ_1                      0xC1
#define SX126X_CAL_IMG_779_MHZ_2                      0xC5
#define SX126X_CAL_IMG_863_MHZ_1                      0xD7
#define SX126X_CAL_IMG_863_MHZ_2                      0xDB
#define SX126X_CAL_IMG_902_MHZ_1                      0xE1
#define SX126X_CAL_IMG_902_MHZ_2                      0xE9

//SX126X_CMD_SET_PA_CONFIG
#define SX126X_PA_CONFIG_HP_MAX                       0x07
#define SX126X_PA_CONFIG_SX1268                       0x01
#define SX126X_PA_CONFIG_PA_LUT                       0x01

//SX126X_CMD_SET_RX_TX_FALLBACK_MODE
#define SX126X_RX_TX_FALLBACK_MODE_FS                 0x40        //  7     0     after Rx/Tx go to: FS mode
#define SX126X_RX_TX_FALLBACK_MODE_STDBY_XOSC         0x30        //  7     0                        standby with crystal oscillator
#define SX126X_RX_TX_FALLBACK_MODE_STDBY_RC           0x20        //  7     0                        standby with RC oscillator (default)

//SX126X_CMD_SET_DIO_IRQ_PARAMS
#define SX126X_IRQ_TIMEOUT                            0b1000000000  //  9     9     Rx or Tx timeout
#define SX126X_IRQ_CAD_DETECTED                       0b0100000000  //  8     8     channel activity detected
#define SX126X_IRQ_CAD_DONE                           0b0010000000  //  7     7     channel activity detection finished
#define SX126X_IRQ_CRC_ERR                            0b0001000000  //  6     6     wrong CRC received
#define SX126X_IRQ_HEADER_ERR                         0b0000100000  //  5     5     LoRa header CRC error
#define SX126X_IRQ_HEADER_VALID                       0b0000010000  //  4     4     valid LoRa header received
#define SX126X_IRQ_SYNC_WORD_VALID                    0b0000001000  //  3     3     valid sync word detected
#define SX126X_IRQ_PREAMBLE_DETECTED                  0b0000000100  //  2     2     preamble detected
#define SX126X_IRQ_RX_DONE                            0b0000000010  //  1     1     packet received
#define SX126X_IRQ_TX_DONE                            0b0000000001  //  0     0     packet transmission completed
#define SX126X_IRQ_ALL                                0b1111111111  //  9     0     all interrupts
#define SX126X_IRQ_NONE                               0b0000000000  //  9     0     no interrupts

//SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL
#define SX126X_DIO2_AS_IRQ                            0x00        //  7     0     DIO2 configuration: IRQ
#define SX126X_DIO2_AS_RF_SWITCH                      0x01        //  7     0                         RF switch control

//SX126X_CMD_SET_DIO3_AS_TCXO_CTRL
#define SX126X_DIO3_OUTPUT_1_6                        0x00        //  7     0     DIO3 voltage output for TCXO: 1.6 V
#define SX126X_DIO3_OUTPUT_1_7                        0x01        //  7     0                                   1.7 V
#define SX126X_DIO3_OUTPUT_1_8                        0x02        //  7     0                                   1.8 V
#define SX126X_DIO3_OUTPUT_2_2                        0x03        //  7     0                                   2.2 V
#define SX126X_DIO3_OUTPUT_2_4                        0x04        //  7     0                                   2.4 V
#define SX126X_DIO3_OUTPUT_2_7                        0x05        //  7     0                                   2.7 V
#define SX126X_DIO3_OUTPUT_3_0                        0x06        //  7     0                                   3.0 V
#define SX126X_DIO3_OUTPUT_3_3                        0x07        //  7     0                                   3.3 V

//Radio complete Wake-up Time with TCXO stabilisation time
#define RADIO_TCXO_SETUP_TIME                         5000        // [us]

//SX126X_CMD_SET_PACKET_TYPE
#define SX126X_PACKET_TYPE_GFSK                       0x00        //  7     0     packet type: GFSK
#define SX126X_PACKET_TYPE_LORA                       0x01        //  7     0                  LoRa

//SX126X_CMD_SET_TX_PARAMS
#define SX126X_PA_RAMP_10U                            0x00        //  7     0     ramp time: 10 us
#define SX126X_PA_RAMP_20U                            0x01        //  7     0                20 us
#define SX126X_PA_RAMP_40U                            0x02        //  7     0                40 us
#define SX126X_PA_RAMP_80U                            0x03        //  7     0                80 us
#define SX126X_PA_RAMP_200U                           0x04        //  7     0                200 us
#define SX126X_PA_RAMP_800U                           0x05        //  7     0                800 us
#define SX126X_PA_RAMP_1700U                          0x06        //  7     0                1700 us
#define SX126X_PA_RAMP_3400U                          0x07        //  7     0                3400 us

//SX126X_CMD_SET_MODULATION_PARAMS
#define SX126X_GFSK_FILTER_NONE                       0x00        //  7     0     GFSK filter: none
#define SX126X_GFSK_FILTER_GAUSS_0_3                  0x08        //  7     0                  Gaussian, BT = 0.3
#define SX126X_GFSK_FILTER_GAUSS_0_5                  0x09        //  7     0                  Gaussian, BT = 0.5
#define SX126X_GFSK_FILTER_GAUSS_0_7                  0x0A        //  7     0                  Gaussian, BT = 0.7
#define SX126X_GFSK_FILTER_GAUSS_1                    0x0B        //  7     0                  Gaussian, BT = 1
#define SX126X_GFSK_RX_BW_4_8                         0x1F        //  7     0     GFSK Rx bandwidth: 4.8 kHz
#define SX126X_GFSK_RX_BW_5_8                         0x17        //  7     0                        5.8 kHz
#define SX126X_GFSK_RX_BW_7_3                         0x0F        //  7     0                        7.3 kHz
#define SX126X_GFSK_RX_BW_9_7                         0x1E        //  7     0                        9.7 kHz
#define SX126X_GFSK_RX_BW_11_7                        0x16        //  7     0                        11.7 kHz
#define SX126X_GFSK_RX_BW_14_6                        0x0E        //  7     0                        14.6 kHz
#define SX126X_GFSK_RX_BW_19_5                        0x1D        //  7     0                        19.5 kHz
#define SX126X_GFSK_RX_BW_23_4                        0x15        //  7     0                        23.4 kHz
#define SX126X_GFSK_RX_BW_29_3                        0x0D        //  7     0                        29.3 kHz
#define SX126X_GFSK_RX_BW_39_0                        0x1C        //  7     0                        39.0 kHz
#define SX126X_GFSK_RX_BW_46_9                        0x14        //  7     0                        46.9 kHz
#define SX126X_GFSK_RX_BW_58_6                        0x0C        //  7     0                        58.6 kHz
#define SX126X_GFSK_RX_BW_78_2                        0x1B        //  7     0                        78.2 kHz
#define SX126X_GFSK_RX_BW_93_8                        0x13        //  7     0                        93.8 kHz
#define SX126X_GFSK_RX_BW_117_3                       0x0B        //  7     0                        117.3 kHz
#define SX126X_GFSK_RX_BW_156_2                       0x1A        //  7     0                        156.2 kHz
#define SX126X_GFSK_RX_BW_187_2                       0x12        //  7     0                        187.2 kHz
#define SX126X_GFSK_RX_BW_234_3                       0x0A        //  7     0                        234.3 kHz
#define SX126X_GFSK_RX_BW_312_0                       0x19        //  7     0                        312.0 kHz
#define SX126X_GFSK_RX_BW_373_6                       0x11        //  7     0                        373.6 kHz
#define SX126X_GFSK_RX_BW_467_0                       0x09        //  7     0                        467.0 kHz
#define SX126X_LORA_BW_7_8                            0x00        //  7     0     LoRa bandwidth: 7.8 kHz
#define SX126X_LORA_BW_10_4                           0x08        //  7     0                     10.4 kHz
#define SX126X_LORA_BW_15_6                           0x01        //  7     0                     15.6 kHz
#define SX126X_LORA_BW_20_8                           0x09        //  7     0                     20.8 kHz
#define SX126X_LORA_BW_31_25                          0x02        //  7     0                     31.25 kHz
#define SX126X_LORA_BW_41_7                           0x0A        //  7     0                     41.7 kHz
#define SX126X_LORA_BW_62_5                           0x03        //  7     0                     62.5 kHz
#define SX126X_LORA_BW_125_0                          0x04        //  7     0                     125.0 kHz
#define SX126X_LORA_BW_250_0                          0x05        //  7     0                     250.0 kHz
#define SX126X_LORA_BW_500_0                          0x06        //  7     0                     500.0 kHz
#define SX126X_LORA_CR_4_5                            0x01        //  7     0     LoRa coding rate: 4/5
#define SX126X_LORA_CR_4_6                            0x02        //  7     0                       4/6
#define SX126X_LORA_CR_4_7                            0x03        //  7     0                       4/7
#define SX126X_LORA_CR_4_8                            0x04        //  7     0                       4/8
#define SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_OFF        0x00        //  7     0     LoRa low data rate optimization: disabled
#define SX126X_LORA_LOW_DATA_RATE_OPTIMIZE_ON         0x01        //  7     0                                      enabled

//SX126X_CMD_SET_PACKET_PARAMS
#define SX126X_GFSK_PREAMBLE_DETECT_OFF               0x00        //  7     0     GFSK minimum preamble length before reception starts: detector disabled
#define SX126X_GFSK_PREAMBLE_DETECT_8                 0x04        //  7     0                                                           8 bits
#define SX126X_GFSK_PREAMBLE_DETECT_16                0x05        //  7     0                                                           16 bits
#define SX126X_GFSK_PREAMBLE_DETECT_24                0x06        //  7     0                                                           24 bits
#define SX126X_GFSK_PREAMBLE_DETECT_32                0x07        //  7     0                                                           32 bits
#define SX126X_GFSK_ADDRESS_FILT_OFF                  0x00        //  7     0     GFSK address filtering: disabled
#define SX126X_GFSK_ADDRESS_FILT_NODE                 0x01        //  7     0                             node only
#define SX126X_GFSK_ADDRESS_FILT_NODE_BROADCAST       0x02        //  7     0                             node and broadcast
#define SX126X_GFSK_PACKET_FIXED                      0x00        //  7     0     GFSK packet type: fixed (payload length known in advance to both sides)
#define SX126X_GFSK_PACKET_VARIABLE                   0x01        //  7     0                       variable (payload length added to packet)
#define SX126X_GFSK_CRC_OFF                           0x01        //  7     0     GFSK packet CRC: disabled
#define SX126X_GFSK_CRC_1_BYTE                        0x00        //  7     0                      1 byte
#define SX126X_GFSK_CRC_2_BYTE                        0x02        //  7     0                      2 byte
#define SX126X_GFSK_CRC_1_BYTE_INV                    0x04        //  7     0                      1 byte, inverted
#define SX126X_GFSK_CRC_2_BYTE_INV                    0x06        //  7     0                      2 byte, inverted
#define SX126X_GFSK_WHITENING_OFF                     0x00        //  7     0     GFSK data whitening: disabled
#define SX126X_GFSK_WHITENING_ON                      0x01        //  7     0                          enabled
#define SX126X_LORA_HEADER_EXPLICIT                   0x00        //  7     0     LoRa header mode: explicit
#define SX126X_LORA_HEADER_IMPLICIT                   0x01        //  7     0                       implicit
#define SX126X_LORA_CRC_OFF                           0x00        //  7     0     LoRa CRC mode: disabled
#define SX126X_LORA_CRC_ON                            0x01        //  7     0                    enabled
#define SX126X_LORA_IQ_STANDARD                       0x00        //  7     0     LoRa IQ setup: standard
#define SX126X_LORA_IQ_INVERTED                       0x01        //  7     0                    inverted

//SX126X_CMD_SET_CAD_PARAMS
#define SX126X_CAD_ON_1_SYMB                          0x00        //  7     0     number of symbols used for CAD: 1
#define SX126X_CAD_ON_2_SYMB                          0x01        //  7     0                                     2
#define SX126X_CAD_ON_4_SYMB                          0x02        //  7     0                                     4
#define SX126X_CAD_ON_8_SYMB                          0x03        //  7     0                                     8
#define SX126X_CAD_ON_16_SYMB                         0x04        //  7     0                                     16
#define SX126X_CAD_GOTO_STDBY                         0x00        //  7     0     after CAD is done, always go to STDBY_RC mode
#define SX126X_CAD_GOTO_RX                            0x01        //  7     0     after CAD is done, go to Rx mode if activity is detected

//SX126X_CMD_GET_STATUS
#define SX126X_STATUS_MODE_STDBY_RC                   0b00100000  //  6     4     current chip mode: STDBY_RC
#define SX126X_STATUS_MODE_STDBY_XOSC                 0b00110000  //  6     4                        STDBY_XOSC
#define SX126X_STATUS_MODE_FS                         0b01000000  //  6     4                        FS
#define SX126X_STATUS_MODE_RX                         0b01010000  //  6     4                        RX
#define SX126X_STATUS_MODE_TX                         0b01100000  //  6     4                        TX
#define SX126X_STATUS_DATA_AVAILABLE                  0b00000100  //  3     1     command status: packet received and data can be retrieved
#define SX126X_STATUS_CMD_TIMEOUT                     0b00000110  //  3     1                     SPI command timed out
#define SX126X_STATUS_CMD_INVALID                     0b00001000  //  3     1                     invalid SPI command
#define SX126X_STATUS_CMD_FAILED                      0b00001010  //  3     1                     SPI command failed to execute
#define SX126X_STATUS_TX_DONE                         0b00001100  //  3     1                     packet transmission done
#define SX126X_STATUS_SPI_FAILED                      0b11111111  //  7     0     SPI transaction failed

//SX126X_CMD_GET_PACKET_STATUS
#define SX126X_GFSK_RX_STATUS_PREAMBLE_ERR            0b10000000  //  7     7     GFSK Rx status: preamble error
#define SX126X_GFSK_RX_STATUS_SYNC_ERR                0b01000000  //  6     6                     sync word error
#define SX126X_GFSK_RX_STATUS_ADRS_ERR                0b00100000  //  5     5                     address error
#define SX126X_GFSK_RX_STATUS_CRC_ERR                 0b00010000  //  4     4                     CRC error
#define SX126X_GFSK_RX_STATUS_LENGTH_ERR              0b00001000  //  3     3                     length error
#define SX126X_GFSK_RX_STATUS_ABORT_ERR               0b00000100  //  2     2                     abort error
#define SX126X_GFSK_RX_STATUS_PACKET_RECEIVED         0b00000010  //  2     2                     packet received
#define SX126X_GFSK_RX_STATUS_PACKET_SENT             0b00000001  //  2     2                     packet sent

//SX126X_CMD_GET_DEVICE_ERRORS
#define SX126X_PA_RAMP_ERR                            0b100000000  //  8     8     device errors: PA ramping failed
#define SX126X_PLL_LOCK_ERR                           0b001000000  //  6     6                    PLL failed to lock
#define SX126X_XOSC_START_ERR                         0b000100000  //  5     5                    crystal oscillator failed to start
#define SX126X_IMG_CALIB_ERR                          0b000010000  //  4     4                    image calibration failed
#define SX126X_ADC_CALIB_ERR                          0b000001000  //  3     3                    ADC calibration failed
#define SX126X_PLL_CALIB_ERR                          0b000000100  //  2     2                    PLL calibration failed
#define SX126X_RC13M_CALIB_ERR                        0b000000010  //  1     1                    RC13M calibration failed
#define SX126X_RC64K_CALIB_ERR                        0b000000001  //  0     0                    RC64K calibration failed


// SX126X SPI register variables
//SX126X_REG_LORA_SYNC_WORD_MSB + LSB
#define SX126X_SYNC_WORD_PUBLIC                       0x3444
#define SX126X_SYNC_WORD_PRIVATE                      0x1424

#define SX126x_TXMODE_ASYNC                           0x01
#define SX126x_TXMODE_SYNC                            0x02
#define SX126x_TXMODE_BACK2RX                         0x04



#define SX126X_NOP ( 0x00 )



/**
 * @brief Maximum value for parameter timeout_in_rtc_step in both functions @ref sx126x_set_rx_with_timeout_in_rtc_step
 * and @ref sx126x_set_tx_with_timeout_in_rtc_step
 */
#define SX126X_MAX_TIMEOUT_IN_RTC_STEP 0x00FFFFFE

/**
 * @brief  Maximum value for parameter timeout_in_ms in both functions @ref sx126x_set_rx and @ref sx126x_set_tx
 */
#define SX126X_MAX_TIMEOUT_IN_MS ( SX126X_MAX_TIMEOUT_IN_RTC_STEP / 64 )

/**
 * @brief Timeout parameter in \ref sx126x_set_rx_with_timeout_in_rtc_step to set the chip in reception until a
 * reception occurs
 */
#define SX126X_RX_SINGLE_MODE 0x00000000

/**
 * @brief Timeout parameter in @ref sx126x_set_rx_with_timeout_in_rtc_step to launch a continuous reception
 */
#define SX126X_RX_CONTINUOUS 0x00FFFFFF

/**
 * @brief Over-current protection default value after @ref sx126x_set_pa_cfg is called with @ref device_sel set to 1
 */
#define SX126X_OCP_PARAM_VALUE_60_MA 0x18

/**
 * @brief Over-current protection default value after @ref sx126x_set_pa_cfg is called with @ref device_sel set to 0
 */
#define SX126X_OCP_PARAM_VALUE_140_MA 0x38

/**
 * @brief  Maximum value for parameter nb_of_symbs in @ref sx126x_set_lora_symb_nb_timeout
 */
#define SX126X_MAX_LORA_SYMB_NUM_TIMEOUT 248

/**
 * @brief Maximum number of register that can be added to the retention list
 */
#define SX126X_MAX_NB_REG_IN_RETENTION 4

/*!
 * @brief Frequency step in MHz used to compute the image calibration parameter
 *
 * @see sx126x_cal_img_in_mhz
 */
#define SX126X_IMAGE_CALIBRATION_STEP_IN_MHZ 4

#define SX126X_CHIP_MODES_POS ( 4U )
#define SX126X_CHIP_MODES_MASK ( 0x07UL << SX126X_CHIP_MODES_POS )

#define SX126X_CMD_STATUS_POS ( 1U )
#define SX126X_CMD_STATUS_MASK ( 0x07UL << SX126X_CMD_STATUS_POS )

#define SX126X_GFSK_RX_STATUS_PKT_SENT_POS ( 0U )
#define SX126X_GFSK_RX_STATUS_PKT_SENT_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_SENT_POS )

#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS ( 1U )
#define SX126X_GFSK_RX_STATUS_PKT_RECEIVED_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_PKT_RECEIVED_POS )

#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS ( 2U )
#define SX126X_GFSK_RX_STATUS_ABORT_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ABORT_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS ( 3U )
#define SX126X_GFSK_RX_STATUS_LENGTH_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_LENGTH_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_CRC_ERROR_POS ( 4U )
#define SX126X_GFSK_RX_STATUS_CRC_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_CRC_ERROR_POS )

#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS ( 5U )
#define SX126X_GFSK_RX_STATUS_ADRS_ERROR_MASK ( 0x01UL << SX126X_GFSK_RX_STATUS_ADRS_ERROR_POS )



/**
 * @brief Initializes the SX1261 transceiver hardware and prepares it for operation.
 */
void     sx1261_init(void);


/*
int16_t  LoRaBegin(uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage, bool useRegulatorLDO);
void     LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq);
uint8_t  LoRaReceive(uint8_t *pData, uint16_t len);
bool     LoRaSend(uint8_t *pData, uint8_t len, uint8_t mode);

void     LoRaDebugPrint(bool enable);
*/

/**
 * @brief Writes a sequence of bytes to the SPI bus.
 *
 * This function transmits the specified number of bytes from the provided buffer
 * over the SPI interface. It returns true if the operation is successful, or false otherwise.
 *
 * @param Dataout Pointer to the buffer containing the data to be sent.
 * @param DataLength Number of bytes to write from the buffer.
 * @return true if the write operation was successful, false otherwise.
 */
bool     spi_write_byte(uint8_t* Dataout, size_t DataLength );


/**
 * @brief Reads a sequence of bytes from the SPI bus.
 *
 * This function reads data from the SPI interface into the provided buffer.
 * It returns true if the operation is successful, or false otherwise.
 *
 * @param Datain Pointer to the buffer where the read data will be stored.
 * @param Dataout Pointer to the buffer containing the data to be sent while reading.
 * @param DataLength Number of bytes to read and write.
 * @return true if the read operation was successful, false otherwise.
 */
bool     spi_read_byte(uint8_t* Datain, uint8_t* Dataout, size_t DataLength );


/**
 * @brief Transfers a single byte over SPI to the specified address.
 *
 * This function sends the provided address byte over the SPI bus and returns
 * the byte received from the SPI slave device during the transfer.
 *
 * @param address The address or data byte to send over SPI.
 * @return uint8_t The byte received from the SPI slave device.
 */
uint8_t  spi_transfer(uint8_t address);

/**
 * @brief Sets the device into receive mode to listen for incoming data.
 *
 * @return true if the device successfully enters receive mode, false otherwise.
 */
bool     ReceiveMode(void);


/**
 * @brief Retrieves the current packet status, including RSSI and SNR values.
 *
 * This function obtains the received signal strength indicator (RSSI) and
 * signal-to-noise ratio (SNR) for the most recently received packet.
 *
 * @param[out] rssiPacket Pointer to an int8_t variable where the RSSI value will be stored (in dBm).
 * @param[out] snrPacket  Pointer to an int8_t variable where the SNR value will be stored (in dB).
 */
void     GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket);


/**
 * @brief Sets the transmission power of the radio module.
 *
 * Configures the output power for transmissions, specified in dBm.
 * The valid range of txPowerInDbm depends on the hardware capabilities.
 *
 * @param txPowerInDbm Transmission power in dBm.
 */
void     SetTxPower(int8_t txPowerInDbm);

/**
 * @brief Configures the IQ inversion setting for the radio transceiver.
 *
 * This function sets the IQ (In-phase and Quadrature) inversion mode based on the provided configuration value.
 * IQ inversion is used to distinguish between uplink and downlink signals or to comply with specific communication protocols.
 *
 * @param iqConfig The IQ configuration value to set. Typically, 0 disables inversion and 1 enables inversion,
 *                 but refer to the hardware documentation for valid values.
 */
void     FixInvertedIQ(uint8_t iqConfig);

/**
 * @brief Configures DIO3 pin as a TCXO (Temperature Compensated Crystal Oscillator) control.
 *
 * This function sets the DIO3 pin to control the TCXO voltage and applies a delay to allow the TCXO to stabilize.
 *
 * @param voltage The voltage (in volts) to be supplied to the TCXO via DIO3.
 * @param delay The delay (in milliseconds) to wait after enabling the TCXO before proceeding.
 */
void     SetDio3AsTcxoCtrl(float voltage, uint32_t delay);

/**
 * @brief Configures DIO2 pin as RF switch control.
 *
 * This function enables or disables the use of the DIO2 pin to control the RF switch.
 *
 * @param enable Set to 1 to enable DIO2 as RF switch control, or 0 to disable.
 */
void     SetDio2AsRfSwitchCtrl(uint8_t enable);

/**
 * @brief Resets the device or module to its default state.
 *
 * This function typically performs a hardware or software reset,
 * reinitializing the device and clearing any previous configurations or states.
 * Use this function to ensure the device starts from a known state.
 */
void     Reset(void);

/**
 * @brief Sets the device to standby mode.
 *
 * This function configures the device to enter standby mode, which reduces power consumption
 * while maintaining the ability to quickly resume normal operation. The specific standby mode
 * is determined by the provided mode parameter.
 *
 * @param mode The standby mode to set. The value of this parameter determines the type of standby
 *             (e.g., RC oscillator or crystal oscillator) as defined by the device datasheet.
 */
void     SetStandby(uint8_t mode);

/**
 * @brief Sets the RF (Radio Frequency) frequency for the device.
 *
 * This function configures the radio transceiver to operate at the specified frequency.
 *
 * @param frequency The desired RF frequency in Hertz (Hz).
 */
void     SetRfFrequency(uint32_t frequency);

/**
 * @brief Performs calibration of the radio module with the specified parameters.
 *
 * This function initiates the calibration process for the radio hardware.
 * The calibration parameters determine which subsystems are calibrated.
 *
 * @param calibParam A bitmask specifying which calibration routines to execute.
 *                   Refer to the hardware documentation for valid values.
 */
void     Calibrate(uint8_t calibParam);

/**
 * @brief Calibrates the image rejection for the specified frequency.
 *
 * This function performs image calibration for the radio transceiver,
 * optimizing receiver performance at the given frequency.
 *
 * @param frequency The frequency (in Hz) at which to perform image calibration.
 */
void     CalibrateImage(uint32_t frequency);

/**
 * @brief Sets the regulator mode of the device.
 *
 * This function configures the power regulator mode for the device,
 * allowing selection between different power supply options (e.g., LDO or DC-DC).
 *
 * @param mode The regulator mode to set. The value should correspond to the
 *             supported modes defined by the device's datasheet or API.
 */
void     SetRegulatorMode(uint8_t mode);

/**
 * @brief Sets the buffer base addresses for transmission and reception.
 *
 * This function configures the base addresses for the transmit and receive buffers,
 * which are used to store data packets during communication.
 *
 * @param txBaseAddress The base address for the transmit buffer.
 * @param rxBaseAddress The base address for the receive buffer.
 */
void     SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress);

/**
 * @brief Sets the transmission parameters for the device.
 *
 * This function configures the transmission parameters, including the power level and ramp time.
 *
 * @param power The transmission power level in dBm.
 * @param rampTime The time taken for the power amplifier to ramp up to full power, in milliseconds.
 */
void     SetPowerConfig(int8_t power, uint8_t rampTime);

/**
 * @brief Sets the overcurrent protection limit for the device.
 *
 * This function configures the overcurrent protection threshold, which helps prevent damage
 * to the device by limiting the maximum current during transmission.
 *
 * @param currentLimit The current limit in Amperes (A).
 */
void     SetOvercurrentProtection(float currentLimit);

/**
 * @brief Sets the sync word for the device.
 *
 * This function configures the synchronization word used for packet detection and filtering.
 * The sync word is typically used to identify valid packets in a communication channel.
 *
 * @param sync The sync word value, typically a 16-bit integer.
 */
void     SetSyncWord(int16_t sync);

/**
 * @brief Configures the Power Amplifier (PA) settings for the device.
 *
 * This function sets the PA configuration parameters, which control the output power,
 * efficiency, and performance of the radio transceiver.
 *
 * @param paDutyCycle Duty cycle for the PA (range and meaning device-specific).
 * @param hpMax       Maximum high power value (device-specific).
 * @param deviceSel   Device selection parameter (e.g., selects between different PA circuits).
 * @param paLut       Lookup table selection for PA settings.
 */
void     SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);

/**
 * @brief Configures the DIO IRQ parameters for the RA01S module.
 *
 * This function sets the interrupt request (IRQ) mask and the masks for DIO1, DIO2, and DIO3 pins.
 * It allows enabling or disabling specific interrupt sources and mapping them to the desired DIO pins.
 *
 * @param irqMask   Bitmask specifying which IRQ sources to enable.
 * @param dio1Mask  Bitmask specifying which IRQ sources are mapped to DIO1.
 * @param dio2Mask  Bitmask specifying which IRQ sources are mapped to DIO2.
 * @param dio3Mask  Bitmask specifying which IRQ sources are mapped to DIO3.
 */
void     SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);

/**
 * @brief Enables or disables stopping the RX timer upon preamble detection.
 *
 * When enabled, the RX timer will stop automatically when a preamble is detected during reception.
 * This can be useful for optimizing power consumption or timing in certain radio operations.
 *
 * @param enable Set to true to stop the RX timer on preamble detection, false to disable this behavior.
 */
void     SetStopRxTimerOnPreambleDetect(bool enable);

/**
 * @brief Sets the number of LoRa symbols to use for the timeout period.
 *
 * This function configures the transceiver to use the specified number of LoRa symbols
 * as the timeout duration for packet reception or transmission. The timeout is measured
 * in units of LoRa symbols, allowing for precise control over radio operations.
 *
 * @param SymbNum The number of LoRa symbols to use for the timeout. Valid range depends on the hardware specification.
 */
void     SetLoRaSymbNumTimeout(uint8_t SymbNum);


void     SetPacketType(uint8_t packetType);


void     SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize);


void     SetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout);


void     SetCad();


uint8_t  GetStatus(void);


uint16_t GetIrqStatus(void);


void     ClearIrqStatus(uint16_t irq);


void     SetTxEnable(void);


void     SetRxEnable(void);


void     SetRx(uint32_t timeout);


void     SetTx(uint32_t timeoutInMs);


uint8_t  GetRssiInst();


void     GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer);


void     Wakeup(void);


void     WaitForIdle(unsigned long timeout);


uint8_t  ReadBuffer(uint8_t *rxData,  uint8_t maxLen);


void     WriteBuffer(uint8_t *txData, uint8_t txDataLen);


void     WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes);


void     ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes);


void     WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes);


uint8_t  WriteCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes);


void     ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes);


void     SetTxContinuousWave();


void     SPItransfer(uint8_t cmd, bool write, uint8_t* dataOut, uint8_t* dataIn, uint8_t numBytes, bool waitForBusy);


void     LoRaError(int error);


static void sx126x_hal_wait_on_busy( const int busy_pin );


static void sx126x_hal_check_device_ready( const sx126x_hal_context_t* sx126x_context );


sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length );

sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length );

sx126x_hal_status_t sx126x_hal_wakeup( const void* context );


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

//
// Operational Modes Functions
//

/**
 * @brief Set the chip in sleep mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Sleep mode configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_sleep( const void* context, const sx126x_sleep_cfgs_t cfg );

/**
 * @brief Set the chip in stand-by mode
 *
 * @param [in]  context Chip implementation context
 * @param [in]  cfg Stand-by mode configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_standby( const void* context, const sx126x_standby_cfg_t cfg );

/**
 * @brief Set the chip in frequency synthesis mode
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_fs( const void* context );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms The timeout configuration in millisecond for Tx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx( const void* context, const uint32_t timeout_in_ms );

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration can be computed with the formula:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times * \frac{1}{64} \f$
 *
 * @remark Maximal value is SX126X_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms)
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step The timeout configuration for Tx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values        | Meaning                                                                               |
 * | ----------------------| --------------------------------------------------------------------------------------|
 * | SX126X_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_ms The timeout configuration in millisecond for Rx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx( const void* context, const uint32_t timeout_in_ms );

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * sx126x_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration is obtained by:
 * \f$ timeout\_duration\_ms = timeout_in_rtc_step \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is SX126X_MAX_TIMEOUT_IN_RTC_STEP (i.e. 262 143 ms).
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values        | Meaning                                                                               |
 * | ----------------------| --------------------------------------------------------------------------------------|
 * | SX126X_RX_SINGLE_MODE | Single: the chip stays in RX mode until a reception occurs, then switch to standby RC |
 * | SX126X_RX_CONTINUOUS  | Continuous: the chip stays in RX mode even after reception of a packet                |
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_in_rtc_step The timeout configuration for Rx operation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_with_timeout_in_rtc_step( const void* context, const uint32_t timeout_in_rtc_step );

/**
 * @brief Configure the event on which the Rx timeout is stopped
 *
 * @remark The two options are:
 *   - Syncword / Header detection (default)
 *   - Preamble detection
 *
 * @param [in] context Chip implementation context
 * @param [in] enable If true, the timer stops on Syncword / Header detection
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_stop_timer_on_preamble( const void* context, const bool enable );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_time_in_ms The timeout of Rx period - in millisecond
 * @param [in] sleep_time_in_ms The length of sleep period - in millisecond
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_duty_cycle( const void* context, const uint32_t rx_time_in_ms,
                                          const uint32_t sleep_time_in_ms );

/**
 * @brief Set the chip in reception mode with duty cycling
 *
 * @remark The Rx mode duration is defined by:
 * \f$ rx\_duration\_ms = rx_time \times \frac{1}{64} \f$
 *
 * @remark The sleep mode duration is defined by:
 * \f$ sleep\_duration\_ms = sleep_time \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is 0xFFFFFF (i.e. 511 seconds).
 *
 * @param [in] context Chip implementation context
 * @param [in] rx_time The timeout of Rx period
 * @param [in] sleep_time The length of sleep period
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_duty_cycle_with_timings_in_rtc_step( const void*    context,
                                                                   const uint32_t rx_time_in_rtc_step,
                                                                   const uint32_t sleep_time_in_rtc_step );

/**
 * @brief Set the chip in CAD (Channel Activity Detection) mode
 *
 * @remark The LoRa packet type shall be selected with @ref sx126x_set_pkt_type before this function is called.
 *
 * @remark The fallback mode is configured with @ref sx126x_set_cad_params.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_cad( const void* context );

/**
 * @brief Set the chip in Tx continuous wave (RF tone).
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_cw( const void* context );

/**
 * @brief Set the chip in Tx infinite preamble (modulated signal).
 *
 * @remark The packet type shall be configured with @ref sx126x_set_pkt_type before using this command.
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_infinite_preamble( const void* context );

/**
 * @brief Configure the regulator mode to be used
 *
 * @remark This function shall be called to set the correct regulator mode, depending on the usage of LDO or DC/DC on
 * the PCB implementation.
 *
 * @param [in] context Chip implementation context
 * @param [in] mode Regulator mode configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_reg_mode( const void* context, const sx126x_reg_mod_t mode );

/**
 * @brief Perform the calibration of the requested blocks
 *
 * @remark This function shall only be called in stand-by RC mode
 *
 * @remark The chip will return to stand-by RC mode on exit. Potential calibration issues can be read out with @ref
 * sx126x_get_device_errors command.
 *
 * @param [in] context Chip implementation context
 * @param [in] param Mask holding the blocks to be calibrated
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cal( const void* context, const sx126x_cal_mask_t param );

/**
 * @brief Launch an image calibration valid for all frequencies inside an interval, in steps
 *
 * @param [in] context Chip implementation context
 * @param [in] freq1 Image calibration interval lower bound, in steps
 * @param [in] freq2 Image calibration interval upper bound, in steps
 *
 * @remark freq1 must be less than or equal to freq2
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cal_img( const void* context, const uint8_t freq1, const uint8_t freq2 );

/**
 * @brief Launch an image calibration valid for all frequencies inside an interval, in MHz
 *
 * @param [in] context Chip implementation context
 * @param [in] freq1_in_mhz Image calibration interval lower bound, in MHz
 * @param [in] freq2_in_mhz Image calibration interval upper bound, in MHz
 *
 * @remark freq1_in_mhz must be less than or equal to freq2_in_mhz
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cal_img_in_mhz( const void* context, const uint16_t freq1_in_mhz, const uint16_t freq2_in_mhz );

/**
 * @brief Configure the PA (Power Amplifier)
 *
 * @remark The parameters depend on the chip being used
 *
 * @param [in] context Chip implementation context
 * @param [in] params Power amplifier configuration parameters
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_pa_cfg( const void* context, const sx126x_pa_cfg_params_t* params );

/**
 * @brief Set chip mode to be used after successful transmission or reception.
 *
 * @remark This setting is not taken into account during Rx Duty Cycle mode or Auto TxRx.
 *
 * @param [in] context Chip implementation context
 * @param [in] fallback_mode Selected fallback mode
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rx_tx_fallback_mode( const void* context, const sx126x_fallback_modes_t fallback_mode );

//
// Registers and Buffer Access
//

/**
 * @brief Write data into register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start writing operation
 * @param [in] buffer The buffer of bytes to write into memory
 * @param [in] size Number of bytes to write into memory, starting from address
 *
 * @returns Operation status
 *
 * @see sx126x_read_register
 */
sx126x_status_t sx126x_write_register( const void* context, const uint16_t address, const uint8_t* buffer,
                                       const uint8_t size );

/**
 * @brief Read data from register memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] address The register memory address to start reading operation
 * @param [in] buffer The buffer of bytes to be filled with data from registers
 * @param [in] size Number of bytes to read from memory, starting from address
 *
 * @returns Operation status
 *
 * @see sx126x_write_register
 */
sx126x_status_t sx126x_read_register( const void* context, const uint16_t address, uint8_t* buffer,
                                      const uint8_t size );

/**
 * @brief Write data into radio Tx buffer memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] offset Start address in the Tx buffer of the chip
 * @param [in] buffer The buffer of bytes to write into radio buffer
 * @param [in] size The number of bytes to write into Tx radio buffer
 *
 * @returns Operation status
 *
 * @see sx126x_read_buffer
 */
sx126x_status_t sx126x_write_buffer( const void* context, const uint8_t offset, const uint8_t* buffer,
                                     const uint8_t size );

/**
 * @brief Read data from radio Rx buffer memory space.
 *
 * @param [in] context Chip implementation context
 * @param [in] offset Start address in the Rx buffer of the chip
 * @param [in] buffer The buffer of bytes to be filled with content from Rx radio buffer
 * @param [in] size The number of bytes to read from the Rx radio buffer
 *
 * @returns Operation status
 *
 * @see sx126x_write_buffer
 */
sx126x_status_t sx126x_read_buffer( const void* context, const uint8_t offset, uint8_t* buffer, const uint8_t size );

//
// DIO and IRQ Control Functions
//

/**
 * @brief Set which interrupt signals are redirected to the dedicated DIO pin
 *
 * @remark By default, no interrupt signal is redirected.
 *
 * @remark An interrupt will not occur until it is enabled system-wide, even if it is redirected to a specific DIO.
 *
 * @remark The DIO pin will remain asserted until all redirected interrupt signals are cleared with a call to @ref
 * sx126x_clear_irq_status.
 *
 * @remark DIO2 and DIO3 are shared with other features. See @ref sx126x_set_dio2_as_rf_sw_ctrl and @ref
 * sx126x_set_dio3_as_tcxo_ctrl
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt mask
 * @param [in] dio1_mask Variable that holds the interrupt mask for dio1
 * @param [in] dio2_mask Variable that holds the interrupt mask for dio2
 * @param [in] dio3_mask Variable that holds the interrupt mask for dio3
 *
 * @returns Operation status
 *
 * @see sx126x_clear_irq_status, sx126x_get_irq_status, sx126x_set_dio2_as_rf_sw_ctrl, sx126x_set_dio3_as_tcxo_ctrl
 */
sx126x_status_t sx126x_set_dio_irq_params( const void* context, const uint16_t irq_mask, const uint16_t dio1_mask,
                                           const uint16_t dio2_mask, const uint16_t dio3_mask );

/**
 * @brief Get system interrupt status
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status
 *
 * @returns Operation status
 *
 * @see sx126x_clear_irq_status
 */
sx126x_status_t sx126x_get_irq_status( const void* context, sx126x_irq_mask_t* irq );

/**
 * @brief Clear selected system interrupts
 *
 * @param [in] context Chip implementation context
 * @param [in] irq_mask Variable that holds the system interrupt to be cleared
 *
 * @returns Operation status
 *
 * @see sx126x_get_irq_status
 */
sx126x_status_t sx126x_clear_irq_status( const void* context, const sx126x_irq_mask_t irq_mask );

/**
 * @brief Clears any radio irq status flags that are set and returns the flags that
 * were cleared.
 *
 * @param [in] context Chip implementation context
 * @param [out] irq Pointer to a variable for holding the system interrupt status; can be NULL
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_and_clear_irq_status( const void* context, sx126x_irq_mask_t* irq );

/**
 * @brief Configure the embedded RF switch control
 *
 * @param [in] context Chip implementation context
 * @param [in] enable Enable this feature if set to true
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_dio2_as_rf_sw_ctrl( const void* context, const bool enable );

/**
 * @brief Configure the embedded TCXO switch control
 *
 * @remark This function shall only be called in standby RC mode.
 *
 * @remark The chip will wait for the timeout to happen before starting any operation that requires the TCXO.
 *
 * @param [in] context Chip implementation context
 * @param [in] tcxo_voltage Voltage used to power the TCXO
 * @param [in] timeout Time needed for the TCXO to be stable
 *
 * @returns Operation status
 *
 */
sx126x_status_t sx126x_set_dio3_as_tcxo_ctrl( const void* context, const sx126x_tcxo_ctrl_voltages_t tcxo_voltage,
                                              const uint32_t timeout );

//
// RF Modulation and Packet-Related Functions
//

/**
 * @brief Set the RF frequency for future radio operations.
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq_in_hz The frequency in Hz to set for radio operations
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rf_freq( const void* context, const uint32_t freq_in_hz );

/**
 * @brief Set the RF frequency for future radio operations - parameter in PLL steps
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] context Chip implementation context
 * @param [in] freq The frequency in PLL steps to set for radio operations
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_rf_freq_in_pll_steps( const void* context, const uint32_t freq );

/**
 * @brief Set the packet type
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] pkt_type Packet type to set
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_pkt_type( const void* context, const sx126x_pkt_type_t pkt_type );

/**
 * @brief Get the current packet type
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_type Pointer to a variable holding the packet type
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_pkt_type( const void* context, sx126x_pkt_type_t* pkt_type );

/**
 * @brief Set the parameters for TX power and power amplifier ramp time
 *
 * @param [in] context Chip implementation context
 * @param [in] pwr_in_dbm The Tx output power in dBm
 * @param [in] ramp_time The ramping time configuration for the PA
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_tx_params( const void* context, const int8_t pwr_in_dbm,
                                      const sx126x_ramp_time_t ramp_time );

/**
 * @brief Set the modulation parameters for GFSK packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of GFSK modulation configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_mod_params( const void* context, const sx126x_mod_params_gfsk_t* params );

/**
 * @brief Set the modulation parameters for LoRa packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of LoRa modulation configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_mod_params( const void* context, const sx126x_mod_params_lora_t* params );

/**
 * @brief Set the packet parameters for GFSK packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this
 * one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of GFSK packet configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_pkt_params( const void* context, const sx126x_pkt_params_gfsk_t* params );

/**
 * @brief Set the packet parameters for LoRa packets
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of LoRa packet configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_pkt_params( const void* context, const sx126x_pkt_params_lora_t* params );

/**
 * @brief Set the parameters for CAD operation
 *
 * @remark The command @ref sx126x_set_pkt_type must be called prior to this one.
 *
 * @param [in] context Chip implementation context
 * @param [in] params The structure of CAD configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_cad_params( const void* context, const sx126x_cad_params_t* params );

/**
 * @brief Set buffer start addresses for both Tx and Rx operations
 *
 * @param [in] context Chip implementation context
 * @param [in] tx_base_address The start address used for Tx operations
 * @param [in] rx_base_address The start address used for Rx operations
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_buffer_base_address( const void* context, const uint8_t tx_base_address,
                                                const uint8_t rx_base_address );

/**
 * @brief Set the timeout to be used when the chip is configured in Rx mode (only in LoRa)
 *
 * @remark The maximum timeout is \ref SX126X_MAX_LORA_SYMB_NUM_TIMEOUT
 * @remark The function is disabled if the timeout is set to 0
 *
 * @param [in] context Chip implementation context
 * @param [in] nb_of_symbs Timeout in number of symbol
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_symb_nb_timeout( const void* context, const uint8_t nb_of_symbs );

//
// Communication Status Information
//

/**
 * @brief Get the chip status
 *
 * @param [in] context Chip implementation context
 * @param [out] radio_status Pointer to a structure holding the radio status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_status( const void* context, sx126x_chip_status_t* radio_status );

/**
 * @brief Get the current Rx buffer status for both LoRa and GFSK Rx operations
 *
 * @details This function is used to get the length of the received payload and the start address to be used when
 * reading data from the Rx buffer.
 *
 * @param [in] context Chip implementation context
 * @param [out] rx_buffer_status Pointer to a structure to store the current status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_rx_buffer_status( const void* context, sx126x_rx_buffer_status_t* rx_buffer_status );

/**
 * @brief Get the status of the last GFSK packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_gfsk_pkt_status( const void* context, sx126x_pkt_status_gfsk_t* pkt_status );

/**
 * @brief Get the status of the last LoRa packet received
 *
 * @param [in] context Chip implementation context
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_lora_pkt_status( const void* context, sx126x_pkt_status_lora_t* pkt_status );

/**
 * @brief Get the instantaneous RSSI value.
 *
 * @remark This function shall be called when in Rx mode.
 *
 * @param [in] context Chip implementation context
 * @param [out] rssi_in_dbm Pointer to a variable to store the RSSI value in dBm
 *
 * @returns Operation status
 *
 * @see sx126x_set_rx
 */
sx126x_status_t sx126x_get_rssi_inst( const void* context, int16_t* rssi_in_dbm );

/**
 * @brief Get the statistics about GFSK communication
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Pointer to a structure to store GFSK-related statistics
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_gfsk_stats( const void* context, sx126x_stats_gfsk_t* stats );

/**
 * @brief Get the statistics about LoRa communication
 *
 * @param [in] context Chip implementation context
 * @param [out] stats Pointer to a structure to store LoRa-related statistics
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_lora_stats( const void* context, sx126x_stats_lora_t* stats );

/**
 * @brief Reset all the statistics for both Lora and GFSK communications
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_reset_stats( const void* context );

//
// Miscellaneous
//

/**
 * @brief Perform a hard reset of the chip
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_reset( const void* context );

/**
 * @brief Wake the radio up from sleep mode.
 *
 * @param [in]  context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_wakeup( const void* context );

/**
 * @brief Get the list of all active errors
 *
 * @param [in] context Chip implementation context
 * @param [out] errors Pointer to a variable to store the error list
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_device_errors( const void* context, sx126x_errors_mask_t* errors );

/**
 * @brief Clear all active errors
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_clear_device_errors( const void* context );

/**
 * @brief Get the parameter corresponding to a GFSK Rx bandwith immediately above the minimum requested one.
 *
 * @param [in] bw Minimum required bandwith in Hz
 * @param [out] param Pointer to a value to store the parameter
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_gfsk_bw_param( const uint32_t bw, uint8_t* param );

/**
 * @brief Get the actual value in Hertz of a given LoRa bandwidth
 *
 * @param [in] bw LoRa bandwidth parameter
 *
 * @returns Actual LoRa bandwidth in Hertz
 */
uint32_t sx126x_get_lora_bw_in_hz( sx126x_lora_bw_t bw );

/**
 * @brief Compute the numerator for LoRa time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the LoRa bandwidth in Hertz.
 *
 * @param [in] pkt_p Pointer to the structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to the structure holding the LoRa modulation parameters
 *
 * @returns LoRa time-on-air numerator
 */
uint32_t sx126x_get_lora_time_on_air_numerator( const sx126x_pkt_params_lora_t* pkt_p,
                                                const sx126x_mod_params_lora_t* mod_p );

/**
 * @brief Get the time on air in ms for LoRa transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the LoRa packet parameters
 * @param [in] mod_p Pointer to a structure holding the LoRa modulation parameters
 *
 * @returns Time-on-air value in ms for LoRa transmission
 */
uint32_t sx126x_get_lora_time_on_air_in_ms( const sx126x_pkt_params_lora_t* pkt_p,
                                            const sx126x_mod_params_lora_t* mod_p );

/**
 * @brief Compute the numerator for GFSK time-on-air computation.
 *
 * @remark To get the actual time-on-air in second, this value has to be divided by the GFSK bitrate in bits per
 * second.
 *
 * @param [in] pkt_p Pointer to the structure holding the GFSK packet parameters
 *
 * @returns GFSK time-on-air numerator
 */
uint32_t sx126x_get_gfsk_time_on_air_numerator( const sx126x_pkt_params_gfsk_t* pkt_p );

/**
 * @brief Get the time on air in ms for GFSK transmission
 *
 * @param [in] pkt_p Pointer to a structure holding the GFSK packet parameters
 * @param [in] mod_p Pointer to a structure holding the GFSK modulation parameters
 *
 * @returns Time-on-air value in ms for GFSK transmission
 */
uint32_t sx126x_get_gfsk_time_on_air_in_ms( const sx126x_pkt_params_gfsk_t* pkt_p,
                                            const sx126x_mod_params_gfsk_t* mod_p );

/**
 * @brief Generate one or more 32-bit random numbers.
 *
 * @remark A valid packet type must have been configured with @ref sx126x_set_pkt_type
 *         before using this command.
 *
 * @param [in]  context Chip implementation context
 * @param [out] numbers Array where numbers will be stored
 * @param [in]  n Number of desired random numbers
 *
 * @returns Operation status
 *
 * This code can potentially result in interrupt generation. It is the responsibility of
 * the calling code to disable radio interrupts before calling this function,
 * and re-enable them afterwards if necessary, or be certain that any interrupts
 * generated during this process will not cause undesired side-effects in the software.
 *
 * Please note that the random numbers produced by the generator do not have a uniform or Gaussian distribution. If
 * uniformity is needed, perform appropriate software post-processing.
 */
sx126x_status_t sx126x_get_random_numbers( const void* context, uint32_t* numbers, unsigned int n );

/**
 * @brief Get the number of PLL steps for a given frequency in Hertz
 *
 * @param [in] freq_in_hz Frequency in Hertz
 *
 * @returns Number of PLL steps
 */
uint32_t sx126x_convert_freq_in_hz_to_pll_step( uint32_t freq_in_hz );

/**
 * @brief Get the number of RTC steps for a given timeout in millisecond
 *
 * @param [in] timeout_in_ms Timeout in millisecond
 *
 * @returns Number of RTC steps
 */
uint32_t sx126x_convert_timeout_in_ms_to_rtc_step( uint32_t timeout_in_ms );

//
// Registers access
//

/**
 * @brief Configure the boost mode in reception
 *
 * @remark This configuration is not kept in the retention memory. Rx boosted mode shall be enabled each time the chip
 * leaves sleep mode.
 *
 * @param [in] context Chip implementation context
 * @param [in] state Boost mode activation
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cfg_rx_boosted( const void* context, const bool state );

/**
 * @brief Configure the sync word used in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Buffer holding the sync word to be configured
 * @param [in] sync_word_len Sync word length in byte
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_sync_word( const void* context, const uint8_t* sync_word, const uint8_t sync_word_len );

/**
 * @brief Configure the sync word used in LoRa packet
 *
 * @remark In the case of a LoRaWAN use case, the two following values are specified:
 *   - 0x12 for a private LoRaWAN network (default)
 *   - 0x34 for a public LoRaWAN network
 *
 * @param [in] context Chip implementation context
 * @param [in] sync_word Sync word to be configured
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_lora_sync_word( const void* context, const uint8_t sync_word );

/**
 * @brief Configure the seed used to compute CRC in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used to compute the CRC value
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_crc_seed( const void* context, uint16_t seed );

/**
 * @brief Configure the polynomial used to compute CRC in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] polynomial Polynomial value used to compute the CRC value
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_crc_polynomial( const void* context, const uint16_t polynomial );

/**
 * @brief Configure the whitening seed used in GFSK packet
 *
 * @param [in] context Chip implementation context
 * @param [in] seed Seed value used in data whitening
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_gfsk_whitening_seed( const void* context, const uint16_t seed );

/**
 * @brief Configure the Tx PA clamp
 *
 * @remark Workaround - With a SX1262, during the chip initialization, calling this function optimizes the PA clamping
 * threshold. The call must be done after a Power On Reset or a wake-up from cold start (see DS_SX1261-2_V1.2 datasheet
 * chapter 15.2)
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_cfg_tx_clamp( const void* context );

/**
 * @brief Stop the RTC and clear the related event
 *
 * @remark Workaround - It is advised to call this function after ANY reception with timeout active sequence, which
 * stop the RTC and clear the timeout event, if any (see DS_SX1261-2_V1.2 datasheet chapter 15.4)
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_stop_rtc( const void* context );

/**
 * @brief Configure the Over Current Protection (OCP) value
 *
 * @remark The maximum value that can be configured is 63 (i.e. 157.5 mA)
 *
 * @param [in] context Chip implementation context
 * @param [in] ocp_in_step_of_2_5_ma OCP value given in steps of 2.5 mA
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_ocp_value( const void* context, const uint8_t ocp_in_step_of_2_5_ma );

/**
 * @brief Configure the internal trimming capacitor values
 *
 * @remark The device is fitted with internal programmable capacitors connected independently to the pins XTA and XTB of
 * the device. Each capacitor can be controlled independently in steps of 0.47 pF added to the minimal value 11.3pF.
 *
 * @param [in] context Chip implementation context
 * @param [in] trimming_cap_xta Value for the trimming capacitor connected to XTA pin
 * @param [in] trimming_cap_xtb Value for the trimming capacitor connected to XTB pin
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_set_trimming_capacitor_values( const void* context, const uint8_t trimming_cap_xta,
                                                      const uint8_t trimming_cap_xtb );

/**
 * @brief Add registers to the retention list
 *
 * @remark Up to 4 registers can be added to the retention list
 * @remark This function actually appends registers to the list until it is full
 * @remark Registers already added to the list cannot be removed unless the chip goes in sleep mode without retention or
 * a reset is issued
 *
 * @param [in] context Chip implementation context
 * @param [in] register_address The array with addresses of the register to be kept in retention
 * @param [in] register_nb The number of register to be kept in retention
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_add_registers_to_retention_list( const void* context, const uint16_t* register_addr,
                                                        uint8_t register_nb );

/**
 * @brief Add SX126X_REG_RXGAIN, SX126X_REG_TX_MODULATION and SX126X_REG_IQ_POLARITY registers to the retention list
 *
 * @remark These registers are used in workarounds implemented in this driver
 * @remark This function adds 3 registers out of the 4 available slots to the retention list
 * @remark It is recommended to call this function once during initialization phase if the application requires the chip
 * to enter sleep mode without retention
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 *
 * @see sx126x_add_registers_to_retention_list
 */
sx126x_status_t sx126x_init_retention_list( const void* context );

/**
 * @brief Get LoRa coding rate and CRC configurations from received header
 *
 * @remark The output of this function is only valid if the field header_type of pkt_params is equal to @ref
 * SX126X_LORA_PKT_EXPLICIT when calling @ref sx126x_set_lora_pkt_params()
 * @remark The values for cr and crc_is_on are extracted from the header of the received LoRa packet
 *
 * @param [in]  context    Chip implementation context
 * @param [out] cr         LoRa coding rate
 * @param [out] crc_is_on  LoRa CRC configuration
 *
 * @returns Operation status
 */
sx126x_status_t sx126x_get_lora_params_from_header( const void* context, sx126x_lora_cr_t* cr, bool* crc_is_on );

/**
 * @brief init spi connection
 */
void sx126x_init(void);

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


void run_RX_WUR_copy(void *pvParameters);


/**
* @brief Send the Wake up radio signal
**/
void send_WUR();

/**
 * @brief Listen for the Wake up radio signal
 */
void Listen_WUR();

#ifdef __cplusplus
}
#endif

#endif  // DRIVER_H

/* --- EOF ------------------------------------------------------------------ */

