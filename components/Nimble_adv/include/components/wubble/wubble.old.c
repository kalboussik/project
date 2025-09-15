#include <stdio.h>
#include <esp_log.h>
#include "esp_attr.h"
#include "sx1261_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "Nimble_adv.h"

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG ="TDV";

int role = 1; //TODO - replace with the actual role (0 for listener, 1 for initiator)



/* *******************************************************************
 * This file is part of the Wubble project.
 * 
 * Wubble is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Wubble is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Wubble.  If not, see <http://www.gnu.org/licenses/>.
/ ******************************************************************** */




/************************************************************** */

// GPIO Wake-up
#define WAKEUP_GPIO GPIO_NUM_0

// Seuils (ajustables)
#define Th_eligibility 0.5
#define Th_conf 0.6
#define Th_relevance 0.7
#define Ref_Host 100.0
#define Ref_WuROn 5

// Variables globales
float conf = 0.8;
float conf_prime = 0.75;
float Delta_Host = 120.0;
float Delta_sleep = 0;
int N_adv = 8;
int N_WUR_on = 5;


// Structures Host
#define FEATURE_SIZE 3

typedef struct {
    int id;                          // Unique ID of the node
    float feature_vector[FEATURE_SIZE]; // Feature vector representing the host
} Host;



Host* H = NULL;
Host* H_prime = NULL;

// ------------ PERSISTANCE NVS --------------

void load_state_from_nvs() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        nvs_get_i32(nvs, "N_adv", &N_adv);
        nvs_get_i32(nvs, "N_WUR_on", &N_WUR_on);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Loaded NVS: N_adv=%d, N_WUR_on=%d", N_adv, N_WUR_on);
    } else {
        ESP_LOGW(TAG, "No NVS data, starting fresh.");
    }
}

void save_state_to_nvs() {
    nvs_handle_t nvs;
    nvs_open("storage", NVS_READWRITE, &nvs);
    nvs_set_i32(nvs, "N_adv", N_adv);
    nvs_set_i32(nvs, "N_WUR_on", N_WUR_on);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "Saved to NVS: N_adv=%d, N_WUR_on=%d", N_adv, N_WUR_on);
}

void save_hosts_nvs(Host hosts[], int n) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("hosts_ns", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    for (int i = 0; i < n; i++) {
        char key[16];
        // store features
        for (int j = 0; j < FEATURE_SIZE; j++) {
            snprintf(key, sizeof(key), "h%d_f%d", i, j);
            nvs_set_blob(my_handle, key, &hosts[i].feature_vector[j], sizeof(float));
        }

    }
    nvs_commit(my_handle);
    nvs_close(my_handle);
}

void load_hosts_nvs(Host hosts[], int n) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("hosts_ns", NVS_READONLY, &my_handle);
    if (err != ESP_OK) return;

    for (int i = 0; i < n; i++) {
        char key[16];
        // load features
        for (int j = 0; j < FEATURE_SIZE; j++) {
            snprintf(key, sizeof(key), "h%d_f%d", i, j);
            size_t required_size = sizeof(float);
            nvs_get_blob(my_handle, key, &hosts[i].feature_vector[j], &required_size);
        }

    }

    nvs_close(my_handle);
}



//******************************************* Probability Algo ************************************************** */

// Compute similarity-based likelihood P(H′ | H)
float likelihood(Host* current, Host* prev) {
    float dot = 0.0f;
    float normC = 0.0f, normP = 0.0f;

    for (int i = 0; i < FEATURE_SIZE; i++) {
        dot   += current->feature_vector[i] * prev->feature_vector[i];
        normC += current->feature_vector[i] * current->feature_vector[i];
        normP += prev->feature_vector[i] * prev->feature_vector[i];
    }

    // Compute norms dynamically
    normC = sqrtf(normC);
    normP = sqrtf(normP);

    if (normC == 0.0f || normP == 0.0f) return 0.0f;

    float cosine = dot / (normC * normP);
    return (1.0f + cosine) / 2.0f; // map [-1,1] -> [0,1]
}



// Compute P(H | H′) using Bayes' theorem
float P_H_given_Hprev(Host hosts[], int h_index, Host* Hprev, float priors[], int num_hosts) {
    float numerator = likelihood(&hosts[h_index], Hprev) * priors[h_index];
    float marginal = 0.0f;

    for (int i = 0; i < num_hosts; i++)
        marginal += likelihood(&hosts[i], Hprev) * priors[i];

    if (marginal == 0.0f) return 0.0f;
    return numerator / marginal;
}



/*

// Compute similarity-based likelihood P(H′ | H)
float likelihood(Host* current, Host* prev) {
    float dot = 0.0f, normC = 0.0f, normP = 0.0f;
    for (int i = 0; i < FEATURE_SIZE; i++) {
        dot   += current->feature_vector[i] * prev->feature_vector[i];
        normC += current->feature_vector[i] * current->feature_vector[i];
        normP += prev->feature_vector[i] * prev->feature_vector[i];
    }
    if (normC == 0 || normP == 0) return 0.0f;
    float cosine = dot / (sqrtf(normC) * sqrtf(normP));
    return (1.0f + cosine) / 2.0f; // map [-1,1] -> [0,1]
}

// Compute P(H | H′) using Bayes' theorem
float P_H_given_Hprev(Host hosts[], int h_index, Host* Hprev) {
    // Step 1: compute numerator
    float numerator = likelihood(&hosts[h_index], Hprev) * hosts[h_index].prior;

    // Step 2: compute marginal P(H′)
    float marginal = 0.0f;
    for (int i = 0; i < NUM_HOSTS; i++) {
        marginal += likelihood(&hosts[i], Hprev) * hosts[i].prior;
    }

    if (marginal == 0.0f) return 0.0f;
    return numerator / marginal;
}
*/
//**************************************************************************************************** */


int compute_backoff_timer() {
    return (rand() % 300) + 100;
}


bool WurOn() {
    return true;
}

bool WurOff() {
    static int count = 0;
    count++;
    return (count > 5);
}


/*

void wake_up_radio_module() {
    ESP_LOGI(TAG, "Radio ON");
}

void put_radio_to_sleep() {
    ESP_LOGI(TAG, "Radio OFF");
}

void broadcast_self() {
    ESP_LOGI(TAG, "Broadcasting...");
}

*/

void reset_Delta_sleep() {
    Delta_sleep = 0;
}

void DeepSleep() {
    ESP_LOGI(TAG, "Going to deep sleep. Waiting for GPIO wakeup...");
    esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 0); // Wake up on LOW
    save_state_to_nvs();
    esp_deep_sleep_start();
}

void Sleep() {
    ESP_LOGI(TAG, "Radio module short sleep...");
    vTaskDelay(200 / portTICK_PERIOD_MS);
}

// ----------- Main Algorithm --------------
/*
void Listen(void* pvParameters) {
    if (WurOn()) {
        if (H != NULL && ((float)N_adv / N_WUR_on) < Th_eligibility) {
            N_WUR_on++;

            if (H_prime != NULL) {
                if (conf_prime > Th_conf) {
                    float P = compute_probability(H, H_prime);

                    if (P > Th_relevance) {
                        int backoff = compute_backoff_timer();
                        TickType_t start = xTaskGetTickCount();

                        while ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS < backoff && !WurOff()) {
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                        }

                        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS >= backoff) {
                            //wake_up_radio_module();
                            N_adv++;

                            while (!WurOff()) {
                                broadcast_self();
                                vTaskDelay(10 / portTICK_PERIOD_MS);
                            }

                            //put_radio_to_sleep();
                            Sleep();
                        }
                    }
                }
            } else {
                if (conf > Th_conf && Delta_Host >= Ref_Host) {
                    int backoff = compute_backoff_timer();
                    TickType_t start = xTaskGetTickCount();

                    while ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS < backoff && !WurOff()) {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                    }

                    if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS >= backoff) {
                        wake_up_radio_module();
                        N_adv++;

                        while (!WurOff()) {
                            broadcast_self();
                            vTaskDelay(10 / portTICK_PERIOD_MS);
                        }

                        put_radio_to_sleep();
                        Sleep();
                    }
                }
            }
        }
    }

    reset_Delta_sleep();
    DeepSleep();
}

// ------------ app_main ------------

void app_main() {
    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Init NVS vars
    load_state_from_nvs();

    // Init GPIO Wakeup Pin
    //gpio_pad_select_gpio(WAKEUP_GPIO);
    gpio_set_direction(WAKEUP_GPIO, GPIO_MODE_INPUT);
    gpio_pulldown_en(WAKEUP_GPIO);

    // Simuler deux hôtes
    H = malloc(sizeof(Host));
    H_prime = malloc(sizeof(Host));

    strcpy(H->id, "NodeA");
    H->feature_vector[0] = 1.0;
    H->feature_vector[1] = 0.5;
    H->feature_vector[2] = 0.2;

    strcpy(H_prime->id, "NodeB");
    H_prime->feature_vector[0] = 0.9;
    H_prime->feature_vector[1] = 0.6;
    H_prime->feature_vector[2] = 0.3;

    // Exécute l'algo une seule fois après réveil
    Listen_WuR(NULL);
}
*/

/************************************************************** */

void make_tdv(void){
    ble_init();
    BLE_scan();
    vTaskDelay(pdMS_TO_TICKS(10000));
    scanner_stop();
    print_ble_table();
    //TODO - complete this part;

}

void send_tdv(void){
    extern int type_of_beacon;
    type_of_beacon = IOTBEACON; // Default type of beacon
    ble_server_start();
    vTaskDelay(pdMS_TO_TICKS(3000));
    ble_server_stop();
}


// Shared callback for all timers
void IRAM_ATTR timer_callback(int arg) {
    ESP_LOGI(TAG, "Timer ");
    int timer_id = (int)arg;

    ESP_LOGI(TAG, "Timer '%i' expired!", arg);
    if (timer_id == 0) {
        // Handle backoff timer expiration
        role = 0; //TODO - replace with the actual role (0 for listener, 1 for initiator)
    } else if (timer_id==1) {
        // Handle Timer-L1 expiration
        ESP_LOGI(TAG, "Timer-L1 expired!");
    } else if (timer_id == 2) {
        // Handle Timer-L2 expiration
        ESP_LOGI(TAG, "Timer-L2 expired!");
    }
}



void IRAM_ATTR wubble(void){



/*
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    Host hosts[NUM_HOSTS] = {
        {{1.0f, 0.0f, 0.0f}, 0.0f, 0.3f},
        {{0.0f, 1.0f, 0.0f}, 0.0f, 0.5f},
        {{0.0f, 0.0f, 1.0f}, 0.0f, 0.2f}
    };

    save_hosts_nvs(hosts, NUM_HOSTS);

    // Later: load hosts dynamically
    Host hosts_loaded[NUM_HOSTS];
    load_hosts_nvs(hosts_loaded, NUM_HOSTS);
    compute_norms(hosts_loaded, NUM_HOSTS);

    Host Hprev = {{0.8f, 0.1f, 0.0f}, 0.0f, 0.0f}; // received dynamically

    for (int i = 0; i < NUM_HOSTS; i++) {
        float p = P_H_given_Hprev(hosts_loaded, i, &Hprev);
        printf("P(H=%d | H′) = %.3f\n", i, p);
    }
}
*/






    neighbour_table_t table = { 
        .size = 128 
    }; 

    // Timer 1: fires after 1 second
    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args = {
        .callback = &timer_callback,
        .arg = 0,
        .name = "Backoff-timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer1_args, &timer1));
    ESP_ERROR_CHECK(esp_timer_start_once(timer1, 3000000));  // 3s

    // Timer 2: fires after 2.5 seconds
    esp_timer_handle_t timer2;
    const esp_timer_create_args_t timer2_args = {
        .callback = &timer_callback,
        .arg = 1,
        .name = "Timer-L1"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer2_args, &timer2));
    ESP_ERROR_CHECK(esp_timer_start_once(timer2, 3000000));  // 3s


    // Timer 3: fires after 500 ms
    esp_timer_handle_t timer3;
    const esp_timer_create_args_t timer3_args = {
        .callback = &timer_callback,
        .arg = 2,
        .name = "Timer-L2"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer3_args, &timer3));
    ESP_ERROR_CHECK(esp_timer_start_once(timer3, 4000000));   // 4s

    while (1) {

        ESP_LOGI(TAG, "Wubble running...");
        vTaskDelay(pdMS_TO_TICKS(100));


        //check for voyageur timer and listen for wur signal

        //if timer is expired role = initiateur

        //if wur signal is received role = ecouteur


       
        switch (role)
            {
            case 0: //NOTE - initiateur
                ESP_LOGI(TAG, "Initiateur : Sending WUR and going ecouteur...");
                send_WUR();
                role = 1;
                //TODO - complete this part

                break;
            
            case 1: //NOTE - ecouteur
                ESP_LOGI(TAG, "ecouteur : making TDV...");
                make_tdv();
                table = get_neighbour_table();
                
                if (table.size==0){
                    ESP_LOGI(TAG, "No neighbours found");
                    role = 2;
                    break;
                }

                //TODO - complete this part
                break;
            
            case 2: //NOTE - annonceur
                
                send_tdv();
                
                //TODO - complete this part
                break;
            case 3: //NOTE - voyageur
                
                ESP_LOGI(TAG, "Voyageur Waiting for backoff timer or WuR...");
                //listening for wur
                break;

            default:
                break;

        }

    }

}