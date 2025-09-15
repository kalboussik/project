#ifndef NIMBLE_ADV_H 
#define NIMBLE_ADV_H

//#include "esp_beacon_api.h"

#ifdef __cplusplus 
extern "C" { 
#endif

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#include "../../config/config.h"

/**
 * @brief Enum for Types of BLE frames.
 **/
enum { IBEACON, IOTBEACON, APPBEACON };

/* Major and Minor part are stored in big endian mode in iBeacon packet,
 * need to use this macro to transfer while creating or processing
 * iBeacon data */
#define ENDIAN_CHANGE_U16(x) ((((x)&0xFF00) >> 8) + (((x)&0x00FF) << 8))

#ifndef ESP_UUID_LEN_128
#define ESP_UUID_LEN_128 16
#endif

typedef uint8_t esp_ble_uuid_t[ESP_UUID_LEN_128];

#define SIZE_OF_BLE_TYPE 1

typedef struct {
    uint8_t flags[3];
    uint8_t length;
    uint8_t type;
} __attribute__((packed)) adv_head_t;

typedef struct {
    adv_head_t adv_head;
    uint16_t   company_id;
    uint16_t   beacon_type;
} __attribute__((packed)) esp_ble_ibeacon_head_t;

typedef struct {
    esp_ble_uuid_t proximity_uuid;
    uint16_t   major;
    uint16_t   minor;
    int8_t     measured_power;
} __attribute__((packed)) esp_ble_ibeacon_vendor_t;

typedef struct {
    esp_ble_ibeacon_head_t   ibeacon_head;
    esp_ble_ibeacon_vendor_t ibeacon_vendor;
} __attribute__((packed)) esp_ble_ibeacon_t;

typedef struct {
    adv_head_t adv_head;
    esp_ble_uuid_t uuid;
    uint8_t    iot_id[ID_BLE_SIZE];
    uint8_t    both_states;
    uint8_t    relevance; //TODO - missing param to send in BLE
    uint8_t    x; //TODO - missing param to send in BLE   
} __attribute__((packed)) goodfloow_iot_beacon_t;

#define GF_IOTBEACON_LENGTH                             \
    SIZE_OF_BLE_TYPE + sizeof(goodfloow_iot_beacon_t) - \
        sizeof(adv_head_t)  // type + UUID + iot_id

typedef struct {
    uint8_t length;
    uint8_t type;
} __attribute__((packed)) goodfloow_app_beacon_head_t;

typedef uint8_t goodfloow_app_beacon_data_t[ID_BLE_SIZE];

typedef struct {
    adv_head_t                  adv_head;  // Filter
    esp_ble_uuid_t                  uuid;      // Filter
    goodfloow_app_beacon_head_t manufacturer_head_ble;
    goodfloow_app_beacon_data_t manufacturer_data_ble;
} __attribute__((packed)) goodfloow_app_beacon_t;

/**
 * @brief Create iBeacon packet
 *
 * @param vendor_config Static data to put into packet
 * @param ibeacon_adv_data Data of packet where vendor_config is put
 **/
esp_err_t esp_ble_config_ibeacon_data(const esp_ble_ibeacon_vendor_t* vendor_config,
                                      esp_ble_ibeacon_t* ibeacon_adv_data);

/**
 * @brief Create GoodFloow IOT Beacon packet
 *
 * @param iot_id The iot id to put inside the beacon
 * @param gf_beacon_adv_data pointer to the struct where to write the data
 **/
esp_err_t esp_ble_config_gf_iot_beacon_data(
    const uint8_t* iot_id,
    const uint8_t* both_states,
    const uint8_t* relevance, //TODO - missing param to send in BLE
    const uint8_t* x, //TODO - missing param to send in BLE 
    goodfloow_iot_beacon_t* beacon);

/**
 * @brief Create GoodFloow App Beacon packet
 *
 * @param iot_id The iot id to put inside the beacon
 * @param gf_beacon_adv_data pointer to the struct where to write the data
 **/
esp_err_t esp_ble_config_gf_app_beacon_data(
    const goodfloow_app_beacon_data_t* app_data,
    goodfloow_app_beacon_t* beacon);

/**
 * @brief Check if packet received is GoodFloow Beacon using GF_BEACON_UUID
 *
 * @param adv_data the ble packet received received
 * @param adv_data_len the len of the ble packet received received
 **/
bool esp_ble_is_gf_iot_beacon_packet(const uint8_t* adv_data, uint8_t adv_data_len);

#define MAX_NEIGHBOURS 128//MAX_IOT_IDS_IN_NEIGHBOUR_TABLE

typedef struct { 
    uint8_t neighbour[/*ID_BLE_SIZE*/4]; 
    uint8_t states_neighbour; 
} neighbour_t;

typedef struct { 
    size_t size; 
    neighbour_t neighbours[MAX_NEIGHBOURS];
} neighbour_table_t;

typedef struct my_ble_struct{
    int time1;           
    int time2;            
    int time3;            
    uint16_t service_UUID[16];
    uint16_t reading_UUID[16];
    uint16_t writing_UUID[16];
    char char_value[128];  // Default characteristic value
    uint8_t data[16]; 
} ble_conf;

void ble_init(void); 

void host_task(void *param);

void ble_server_start();

void ble_server_stop();

void send_BLE_ID(void); 

void advertise_BLE_to_app(void); 

void BLE_scan(void); 

void scanner_stop(void);

neighbour_table_t get_neighbour_table(void);

void receive_gf_iot_beacon(uint8_t* gf_beacon);
bool are_arrays_equal(uint8_t* a, uint8_t* b, size_t size);
bool is_iot_id_in_ble_table(size_t table_size, uint8_t* value);
void print_ble_table();



#ifdef __cplusplus
} 
#endif 

#endif /* NIMBLE_ADV_H */
