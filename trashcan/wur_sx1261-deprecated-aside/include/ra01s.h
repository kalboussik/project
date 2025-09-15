#ifndef _RA01S_H
#define _RA01S_H

#include "driver/spi_master.h"

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


#endif

