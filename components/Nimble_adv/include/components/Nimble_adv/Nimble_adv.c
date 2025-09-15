#include "Nimble_adv.h"
#include <stdio.h>
//#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "ble_hci_trans.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"

//#include "state_machine.h"

#ifdef __cplusplus 
extern "C" { 
#endif

int type_of_beacon = IBEACON; // Default type of beacon

static const esp_ble_ibeacon_head_t ibeacon_common_head = {
    .adv_head = {{0x02, 0x01, 0x06, 0x1A, 0xFF}},  // Add extra braces for nested structure
    .company_id  = 0x004C,  // Apple's Bluetooth SIG company ID
    .beacon_type = 0x1502   // Apple's iBeacon type
};

static const esp_ble_uuid_t goodfloow_ble_uuid = GF_BEACON_UUID;

/* Vendor-specific part of iBeacon payload */
esp_ble_ibeacon_vendor_t vendor_config = {
    .proximity_uuid = GF_BEACON_UUID,
    .major          = ENDIAN_CHANGE_U16(ESP_MAJOR),
    .minor          = ENDIAN_CHANGE_U16(ESP_MINOR),
    .measured_power = (int8_t)0xC5
};

static const adv_head_t gf_iot_beacon_common_head = {
    .flags  = {0x02, 0x01, 0x06},
    .length = GF_IOTBEACON_LENGTH,
    .type   = 0x21  // Service Data - 128-bit UUID
};

static const adv_head_t gf_app_beacon_common_head = {
    .flags  = {0x02, 0x01, 0x06},
    .length = SIZE_OF_BLE_TYPE + sizeof(esp_ble_uuid_t),
    .type   = 0x07  // Complete list of 128-bit Service UUIDs
};

static const goodfloow_app_beacon_head_t gf_app_beacon_manufacturer_head = {
    .length = SIZE_OF_BLE_TYPE + sizeof(goodfloow_app_beacon_data_t),
    .type   = 0xFF  // Manufacturer Specific Data
};

esp_err_t esp_ble_config_ibeacon_data(const esp_ble_ibeacon_vendor_t* vendor_cfg,
                                      esp_ble_ibeacon_t* ibeacon_adv_data) {
    if (!vendor_cfg || !ibeacon_adv_data) return ESP_ERR_INVALID_ARG;

    memcpy(&ibeacon_adv_data->ibeacon_head, &ibeacon_common_head, sizeof(esp_ble_ibeacon_head_t));
    memcpy(&ibeacon_adv_data->ibeacon_vendor, vendor_cfg, sizeof(esp_ble_ibeacon_vendor_t));
    return ESP_OK;
}

esp_err_t esp_ble_config_gf_iot_beacon_data(const uint8_t* iot_id,
                                            const uint8_t* both_states,
                                            const uint8_t* relevance,
                                            const uint8_t* x, //TODO - missing param to send in BLE
                                            goodfloow_iot_beacon_t* beacon) {
    if (!beacon) return ESP_ERR_INVALID_ARG;

    memcpy(&beacon->adv_head, &gf_iot_beacon_common_head, sizeof(adv_head_t));
    memcpy(&beacon->uuid, &goodfloow_ble_uuid, sizeof(esp_ble_uuid_t));
    memcpy(&beacon->iot_id, iot_id, ID_BLE_SIZE);
    memcpy(&beacon->both_states, both_states, sizeof(uint8_t));

    memcpy(&beacon->relevance, relevance, sizeof(uint8_t));
    memcpy(&beacon->x, x, sizeof(uint8_t)); //TODO - Finish this when adding the param
    
    return ESP_OK;
}

esp_err_t esp_ble_config_gf_app_beacon_data(const goodfloow_app_beacon_data_t* app_data,
                                            goodfloow_app_beacon_t* beacon) {
    if (!beacon || !app_data) return ESP_ERR_INVALID_ARG;

    memcpy(&beacon->adv_head, &gf_app_beacon_common_head, sizeof(adv_head_t));
    memcpy(&beacon->uuid, &goodfloow_ble_uuid, sizeof(esp_ble_uuid_t));
    memcpy(&beacon->manufacturer_head_ble, &gf_app_beacon_manufacturer_head, sizeof(goodfloow_app_beacon_head_t));
    memcpy(&beacon->manufacturer_data_ble, app_data, sizeof(goodfloow_app_beacon_data_t));
    return ESP_OK;
}

bool esp_ble_is_gf_iot_beacon_packet(const uint8_t* adv_data, uint8_t adv_data_len) {
    if (!adv_data || adv_data_len != sizeof(goodfloow_iot_beacon_t)) return false;

    if (!memcmp(adv_data, &gf_iot_beacon_common_head, sizeof(adv_head_t)) &&
        !memcmp(adv_data + sizeof(adv_head_t), &goodfloow_ble_uuid, sizeof(esp_ble_uuid_t))) {
        return true;
    }
    return false;
}

static const char* TAG = "BLE_NIMBLE"; 
static QueueHandle_t neighbour_table_full_queue; 
static neighbour_t neighbour_table_ids[MAX_NEIGHBOURS]; 
static size_t neighbour_table_size; 
static uint8_t own_addr_type;
static bool initialized = false; 

static int device_count = 0;
uint8_t ble_addr_type;

static const ble_uuid128_t expected_uuid = BLE_UUID128_INIT(
    0x85, 0x0c, 0x7d, 0x3c, 
    0xd4, 0x35, 0x46, 0x62, 
    0xa6, 0x1f, 0x25, 0x62, 
    0x76, 0x93, 0xdd, 0xac,
);

bool uuid_matches(const ble_uuid128_t *a, const ble_uuid128_t *b) {
    return memcmp(a->value, b->value, 16) == 0;
}

// Callback for handling BLE scan results
static int ble_gap_event(struct ble_gap_event *event, void *arg) {


    struct ble_hs_adv_fields fields;

    switch (event->type)
    {
    // NimBLE event discovery
    case BLE_GAP_EVENT_DISC:
        // Check if we've reached the device storage limit
        if (device_count >= 128) {
            ESP_LOGI(TAG, "Device storage limit reached.");
            return 0;
        }

        int rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) {
            ESP_LOGW(TAG, "Error parsing advertisement fields.");
            return 0;
        }

        

        // Look for our UUID
        for (int i = 0; i < fields.num_uuids128; i++) {
            if (uuid_matches(&fields.uuids128[i], &expected_uuid)) {
                receive_gf_iot_beacon(fields.mfg_data);
            }

        }
        return 0;
    break; 
    
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            //ble_app_advertise();
        }
        break;
    
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT %d", event->disconnect.reason);
        //ble_app_advertise();
        break;
    // Advertise again after completion of the event

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE GAP ADV_COMPLETE");
        //ble_app_advertise();
        break;
        
    default:
        break;
    }
    return 0;
}

// Semaphore for protecting access to the device list
static SemaphoreHandle_t device_semaphore;

// Scan parameters (can be modified for your application)
static struct ble_gap_disc_params scan_params = {
    .itvl = 0x0010,  // Scan interval
    .window = 0x0010,  // Scan window
    .filter_policy = 0,
    .limited = 0,
    .passive = 0,
    .filter_duplicates = 1
};

void ble_init(void) { 
    if (initialized) return;
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
    // Initialize semaphore
    device_semaphore = xSemaphoreCreateMutex();
    if (device_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore.");
        return;
    }

    // Initialize NimBLE
    esp_nimble_hci_init();
    nimble_port_init();
    ble_hs_cfg.sync_cb = BLE_scan;
    nimble_port_freertos_init(nimble_port_run);
    initialized = true;
}

void host_task(void *param)
{   
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void ble_server_start()
{

    // Array of pointers to other service definitions
    // UUID - Universal Unique Identifier
  
  
    nvs_flash_init();                          // 1 - Initialize NVS flash using
    esp_nimble_hci_init();                     // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_hs_cfg.sync_cb = send_BLE_ID;       // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
    ESP_LOGI("SERVER", "This ble_server is running on %d!\n", xPortGetCoreID() );

}

void ble_server_stop()
{
  ESP_ERROR_CHECK_WITHOUT_ABORT(nimble_port_deinit());
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_nimble_hci_deinit());
  //Delete this task
  nimble_port_freertos_deinit();
  ESP_LOGI("SERVER", "Stopping server");

}

void send_BLE_ID(void) { 
    if (type_of_beacon == IBEACON) {
        ESP_LOGI(TAG, "Configuring adverting of IBEACON");

        esp_ble_ibeacon_t ibeacon_adv_data;
        esp_err_t         err =
            esp_ble_config_ibeacon_data(&vendor_config, &ibeacon_adv_data);
        if (err == ESP_OK) {
            ble_gap_adv_set_data((uint8_t*)&ibeacon_adv_data,
                                            sizeof(ibeacon_adv_data));
        }
    } else if (type_of_beacon == IOTBEACON) {
        ESP_LOGI(TAG, "Configuring adverting of IOTBEACON");

        ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
        goodfloow_iot_beacon_t gf_beacon; 
        uint8_t id[ID_BLE_SIZE] = ID_IOT; 
        uint8_t both_states = (uint8_t)(0 << 4) | (uint8_t)( 7); 
        esp_ble_config_gf_iot_beacon_data(id, &both_states, id,
                                            id, //TODO - missing param to send in BLE
                                            &gf_beacon);

        struct ble_hs_adv_fields fields;
        memset(&fields, 0, sizeof(fields));
        static ble_uuid128_t uuid128_list[1] = {{
            .u = {
                .type = BLE_UUID_TYPE_128,
            },
            .value = GF_BEACON_UUID
        }};

        fields.uuids128 = uuid128_list;
        fields.num_uuids128 = 1;
        fields.uuids128_is_complete = 1;

        uint8_t mfg_data[ID_BLE_SIZE + 1];
        memcpy(mfg_data, (uint8_t[])ID_IOT, ID_BLE_SIZE);
        mfg_data[ID_BLE_SIZE] = both_states; // Append both_states to the end of ID_IOT
        fields.mfg_data = mfg_data; // Data sent via BLE
        fields.mfg_data_len = 5;
        int rc = ble_gap_adv_set_fields(&fields);
        if (rc) { ESP_LOGE(TAG, "adv set fields failed %d", rc); return; }
    } else if (type_of_beacon == APPBEACON) {
        ESP_LOGI(TAG, "Configuring adverting of APPBEACON");

        goodfloow_app_beacon_t gf_app_beacon_adv_data;
        uint8_t                id[ID_BLE_SIZE] = ID_IOT;

        esp_err_t err =
            esp_ble_config_gf_app_beacon_data(id, &gf_app_beacon_adv_data);
        if (err == ESP_OK) {
            ble_gap_adv_set_data((uint8_t*)&gf_app_beacon_adv_data,
                                            sizeof(gf_app_beacon_adv_data));
        }
    }
    
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

neighbour_table_t get_neighbour_table(void) { 
    neighbour_table_t table = { 
        .size = neighbour_table_size 
    }; 
    memcpy(table.neighbours, neighbour_table_ids, neighbour_table_size * sizeof(neighbour_t));
    return table; 
}

void BLE_scan(void) { 
    neighbour_table_size = 0; 
    struct ble_gap_disc_params scan_params = {0}; 
    if (ble_gap_disc_active()) {
            
        //ESP_LOGI(TAG, "BLE scanning is already in progress.\n");

    } else {

        // Reset device count
        xSemaphoreTake(device_semaphore, portMAX_DELAY);
        device_count = 0;
        xSemaphoreGive(device_semaphore);

        // Start BLE scan
        int rc = ble_gap_disc(0, BLE_HS_FOREVER, &scan_params, ble_gap_event, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to start scanning: %d", rc);
        } else {
            ESP_LOGI(TAG, "BLE scanning started.");
        }

    }

    return;
}

void scanner_stop(void){
    ble_gap_disc_cancel();
    ESP_LOGI(TAG, "BLE scanning stopped.");
    //print_ble_table();
}

bool are_arrays_equal(uint8_t* a, uint8_t* b, size_t size) {
    for (int i = 0; i < size; i++) {
        if (a[i] != b[i]) {
            return false;
        }
    }

    return true;
}

bool is_iot_id_in_ble_table(size_t table_size, uint8_t* value) {
    for (int i = 0; i < table_size; i++) {
        if (are_arrays_equal(neighbour_table_ids[i].neighbour, value, ID_BLE_SIZE)) {
            return true;
        }
    }

    return false;
}

void print_ble_table() {
    printf("----------------\n");
    printf("TABLE CREATE\n");
    for (size_t i = 0; i < neighbour_table_size; i++) {
        for (size_t j = 0; j < ID_BLE_SIZE; j++) { 
            printf(" %x", neighbour_table_ids[i].neighbour[j]);
        }
        printf(" %x", neighbour_table_ids[i].states_neighbour);
        printf("\n");
    }
    printf("----------------\n");
}

void receive_gf_iot_beacon(uint8_t* gf_beacon) {
    if (neighbour_table_size < MAX_IOT_IDS_IN_NEIGHBOUR_TABLE) {
        uint8_t id[ID_BLE_SIZE];
        for (size_t i = 0; i < ID_BLE_SIZE; i++)
        {
            id[i] = gf_beacon[i];
        }
        uint8_t both_states = gf_beacon[ID_BLE_SIZE]; //NOTE - added the state at the end of the table
        if (!is_iot_id_in_ble_table(neighbour_table_size, id)) {
            ESP_LOGI("SCAN", "Target UUID matched!");

                
            ESP_LOGI("SCAN", "MFG data: %02x %02x %02x %02x %02x",
                id[0], id[1], id[2], id[3], both_states);

            for (size_t i = 0; i < ID_BLE_SIZE; i++)
            {
                neighbour_table_ids[neighbour_table_size].neighbour[i] = id[i];
            }

            neighbour_table_ids[neighbour_table_size].states_neighbour = both_states; //NOTE - added the state at the end of the table
            
            neighbour_table_size++;
           
            if (neighbour_table_size == MAX_IOT_IDS_IN_NEIGHBOUR_TABLE) {
                ESP_LOGI(TAG,
                         "MAX_IOT_IDS_IN_NEIGHBOUR_TABLE amount of neighbours "
                         "reached");
                bool ok = true;
                if (xQueueSend(neighbour_table_full_queue, &ok, 0) != pdPASS) {
                    // NOTE this is never logged, if the queue is full we get a
                    // panic here as: assert failed: xQueueGenericSend
                    // queue.c:820 (pxQueue)
                    ESP_LOGE(
                        TAG,
                        "Problem: MAX_IOT_IDS_IN_NEIGHBOUR_TABLE amount of "
                        "neighbours reached but neighbour_table_full_queue is "
                        "already full");
                }
            }
        }
    }
}

#ifdef __cplusplus
} 
#endif 