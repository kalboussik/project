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
#include <string.h>


static const char* TAG ="TDV";

int role = 3; //TODO - replace with the actual role (0 for listener, 1 for initiator)


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




    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .name = "Backoff-timer"
    };

    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_once(timer, 1000000)); // 1 second in microseconds

    while (1) {

        //ESP_LOGI(TAG, "Wubble running...");
        //vTaskDelay(pdMS_TO_TICKS(100));


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