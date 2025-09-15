#ifndef CONFIG_H
#define CONFIG_H





//ANCHOR - wubble
#define K 10
#define MAX 10000000 //10s in µs
#define MAX1 5000000 //5s in µs
#define MAX2 5000000 //5s in µs






//ANCHOR - SPI Config for accelerometer and wake up radio
#define SPI_DMA_CHAN SPI_DMA_CH_AUTO
#define PIN_SPI_SCLK 9
#define PIN_SPI_MOSI 10
#define PIN_SPI_MISO 11
#define USED_SPI_HOST SPI2_HOST

//ANCHOR - Accelerometer Chip Select
#define PIN_NSS 4

//ANCHOR - WUR SX1261
#define SYNCWORD     { 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f}
#define SYNCWORD_LENGTH 8 //in bytes
#define PREAMBLE_LENGTH_IN_BIT 50000 //preamble bit length
#define PREAMBLE_DETECTION_LENGTH SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS //!< Preamble detection length /!\ 24bits max for 1ms RX time
#define BR_IN_BPS 50000 //bit rate in bit/second
#define FDEV_IN_HZ 23848 //frequency deivation
#define BW_DSB_PARAM SX126X_GFSK_BW_312000 //bandwidth 
#define RAMP_TIME 0x07 //SX126X_RAMP_?_US
#define POWER 14 //14 max

#define CONFIG_NSS_GPIO 7
#define CONFIG_RST_GPIO 1
#define CONFIG_BUSY_GPIO 5
#define CONFIG_DIO1_GPIO 6
#define CONFIG_TXEN_GPIO -1
#define CONFIG_RXEN_GPIO -1

#define DEEP_SLEEP_SLEEP_PERIOD_MS 999
#define DEEP_SLEEP_RX_PERIOD_MS 1
#define LIGHT_SLEEP_SLEEP_PERIOD_MS 200
#define LIGHT_SLEEP_RX_PERIOD_MS 800
#define TX_TIME_MS 2000

//ANCHOR - BLE
#define ID_BLE_SIZE 4
#define MIN_BLE_RECEIVED 1
#define MAX_IOT_IDS_IN_NEIGHBOUR_TABLE 16
#define ID_IOT { 0x66, 0x66, 0x66, 0x66 }  


//NOTE - Time sending Ibeacon.
#define SEC_IBEACON                 1
#define BLE_SCAN_ADV_MAX_DURATION_S 2 //FIXME - to define 

/* Configuration parameters of the Neighbours Discovery Process*/
#define NB_REPEAT_WUR                   3       // Number of consecutive WuR signals sent to wake up other devices
#define ADDITIONAL_3_BURST_WAKEUP_DSCV  1       // Additional time due to rafale mode (in seconde) for extending the scanning time
#define ADV_DURATION_DSCV_S             4      // Duration of the advertising in second //FIXME - to define
#define LOWER_BOUND_DSCV                200    // Minimal time to wait before send advertising in ms
#define UPPER_BOUNDE_DSCV               500    // Maximal time to wait before send advertising in ms  
#define TH1_SLEEP_DSCV                  1       // Minimum sleep time (in minutes) to consider before sending data during advertising

// NOTE stored and sent in big endian format
#define GF_BEACON_UUID                                                    \
{                                                                     \
    0x85, 0x0c, 0x7d, 0x3c, 0xd4, 0x35, 0x46, 0x62, 0xa6, 0x1f, 0x25, \
        0x62, 0x76, 0x93, 0xdd, 0xac                                  \
}
#define ESP_MAJOR 0x0000
#define ESP_MINOR 0x0000

//ANCHOR - Sniffing WIFI
#define MAC_ADDR_SIZE 6
#define MAX_MACS_IN_WIFI_TABLE         5
#define MIN_MACS_IN_WIFI_TABLE         1

//ANCHOR - Timer
#define TIMER_WUR_DISABLE_M                 2   //NOTE - Timer of Wur disable when received a wur signal in rest state
#define TIMER_WAIT_INACTIVITY_M             5   //NOTE - timer of the IoT waiting in fenwick state to have no movement
#define TIMER_WAIT_DELAYED_WUR_M            10 //FIXME - verify this timer 
#define TIMER_WUR_LISTEN_S                  10
#define TIMER_BETWEEN_LORA_S                10

#define INITIAL_VALUE_RANDOM_MS                400000 //NOTE - approximate time when the timer generates and the ESP goes to sleep
#define TIME_TO_WAIT_OTHER_ACCEL_MS            5000000 //NOTE - JUST TEST Some accels trigger their activity or innactivity timer a little bit late, this is made to wait those accels to pass to their transition state


#define SEC_TO_MIN  60
#define MS_TO_SEC   1000
#define SEC_TO_MS   1000
#define SEC_TO_US   1000000
#define SEC_TO_100S 100

//ANCHOR - INTERRUPTION PINS
#define PIN_TILT_INTERRUPT          21
#define PIN_REED_SWITCH_INTERRUPT   2
#define PIN_ACTIVITY_INTERRUPT      16
#define PIN_INACTIVITY_INTERRUPT    15
#define PIN_WAKE_UP_RADIO_INTERRUPT 6

//ANCHOR -  DEEP SLEEP
#define TILT_MASK           1ULL << 5
#define REED_SWITCH_MASK    1ULL << 4
#define ACTIVITY_MASK       1ULL << 3
#define INACTIVITY_MASK     1ULL << 2
#define WUR_MASK            1ULL << 1
#define NONE_MASK           0x00

#define ON  1 //for configure wake up mode adxl
#define OFF 0

//ANCHOR - Errors macro
#define ERRORS_STORAGE "ERRORS_STORAGE"
#define ERRORS_MASK    "ERRORS"

#define ERRORS_RESET 0x00

#define BLE_ERROR      1ULL << 6
#define LORA_ERROR     1ULL << 5
#define WIFI_ERROR     1ULL << 4
#define AI_ERROR       1ULL << 3
#define ADXL363_ERROR  1ULL << 2
#define WUR_ERROR      1ULL << 1

//ANCHOR - ADXL363 THRESHOLD AND TIME PARAMETERS
#define TIME_FENWICK 0.5                  // Continuous inactivity detection time in minutes (Fenwick State)
#define TIME_MOVEMENT_CAMION 1          // Continuous inactivity detection time in minutes (Movement Camion  State)
#define TIME_BETWEEN_SHOCKS 1           // Time to wait a new shock in seconds (shock detection in parallel)
#define NB_SHOCK_SAMPLES 2               // Number of consecutive values over threshold to detect a shock  (Freq ~ 4.3 Hz)    
#define NB_ACTIVITY_SAMPLES  9          // Number of consecutive values over threshold to detect a continuous activity movement (Freq ~ 4.3 Hz)

#define SHOCK_THRESHOLD     0.5            // Threshold for a shock detection (g)
#define SHOCK_THRESHOLD_ACCEL 0.5

#define ACTIVITY_THRESHOLD 0.3          // Threshold for a continuous activity detection (g)
#define ACTIVITY_THRESHOLD_ACCEL 0.3

#define INACTIVITY_THRESHOLD_ACCEL 0.2  // Threshold for a continuous inactivity detection (g)

#define MAX_STACK_SIZE 5                // Max size for timers stack

//ANCHOR - NB-IoT
#define LENGHT_QUEUE_NB_IOT 3
#define PSM_BUTTON GPIO_NUM_8
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_18
#define BAUDRATE_NB_IOT 115200
#define TICKS_TO_WAIT 50            //Ticks to wait an answer from the module
#define TIMEOUT_ANSWER 30         // Time in seconds to wait an answer from NB IOT module before trying to reconnect to the network
#define MAX_TRY_TO_CONNECT 3 // Number of tries to connect to the network before going to sleep

//ANCHOR - Battery Level

#define ADC_CHANNEL ADC_CHANNEL_2 // GPIO 3 is ADC1 Channel 2
#define ADC_ATTEN ADC_ATTEN_DB_12  // 11dB attenuation (0-3.6V range)
#define ADC_MAX_READING_VALUE 4095.0 //4095.0 Represents the maximum ADC reading in 12-bit resolution ADC
#define ADC_MAX_RANGE 3.6 // 3.6V is ADC max range
#define DIVIDER_RATIO 3.7 // 1 (R1/R2) with R1 = 27000 Ohm and R2 = 10000 Ohm
#define MIN_VOLTAGE 3.0
#define MAX_VOLTAGE 4.2

//ANCHOR - SD Card
#define ACTIVATE_SD_CARD 0
#define PIN_SD_MISO  14 //D0
#define PIN_SD_MOSI  12 //D1
#define PIN_SD_CLK   13
#define PIN_SD_CS    GPIO_NUM_48
#define SD_SPI_HOST SPI3_HOST

#define MOUNT_POINT "/sdcard"
#define FILENAME "/sdcard/logs.txt"
#define FILENAME_ACCEL "/sdcard/accel.csv"


#endif