#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>
#include "esp_attr.h"
//#include "wur_sx1261.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "Nimble_adv.h"
#include <string.h>


static const char* TAG ="TDV";
static int Backoff_timer = 0; // Backoff timer in µs
neighbour_table_t A = { 
    .size = K 
}; // Neighbour table with K neighbours 

static neighbour_t R; 
int conf = 0;
int role = 4;





// Shared callback for all timers
void IRAM_ATTR timer_callback(int arg) {
    ESP_LOGI(TAG, "Timer ");
    int timer_id = (int)arg;

    ESP_LOGI(TAG, "Timer '%i' expired!", arg);
    if (timer_id == 0) {
        // Handle backoff timer expiration
        ESP_LOGW(TAG, "Backoff_timer expired!");
        role = 0; //TODO - replace with the actual role (0 for initiator, 1 for listener)
    } else if (timer_id==1) {
        // Handle Timer-L1 expiration
        ESP_LOGI(TAG, "Timer1 expired!");
        scanner_stop();
        print_ble_table();
        if (A.size == 0) {
            ESP_LOGI(TAG, "No neighbours found");
            role = 3; // Set role to voyager
        } else {
            ESP_LOGI(TAG, "Neighbours found, not enough, proceeding to determine R...");
            role = 2; // Set role to announcer

            
        }
    } else if (timer_id == 2) {
        // Handle Timer-L2 expiration
        ESP_LOGI(TAG, "Timer-L2 expired!");
    }
}



void IRAM_ATTR wubble(void){

    Backoff_timer = rand() % (MAX + 1);


    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args = {
        .callback = &timer_callback,
        .arg = 0,
        .name = "Backoff-timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer1_args, &timer1));
    ESP_ERROR_CHECK(esp_timer_start_once(timer1, Backoff_timer));  




    esp_timer_handle_t timer3;
    const esp_timer_create_args_t timer3_args = {
        .callback = &timer_callback,
        .arg = 2,
        .name = "MAX2"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer3_args, &timer3));
    ESP_ERROR_CHECK(esp_timer_start_once(timer3, MAX2));   // 4s




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
                break;
            
            case 1: //NOTE - ecouteur
                ESP_LOGI(TAG, "ecouteur : making TDV...");
                make_tdv();
                A = get_neighbour_table();

                if (A.size==0){
                    ESP_LOGI(TAG, "No neighbours found");
                    role = 2;
                    break;
                }

                //TODO - complete this part
                break;
            
            case 2: //NOTE - annonceur
                Identify_Entity(&R, &conf);
                ESP_LOGI(TAG, "Annonceur : Sending TDV...");
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
    


void make_tdv(void){
    
    esp_timer_handle_t timer2;
    const esp_timer_create_args_t timer2_args = {
        .callback = &timer_callback,
        .arg = 1,
        .name = "MAX1"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer2_args, &timer2));
    ESP_ERROR_CHECK(esp_timer_start_once(timer2, MAX2));   


    ESP_LOGI(TAG, "Making TDV...");
    // Initialize BLE and start scanning
    ble_init();
    BLE_scan();

}

void send_tdv(void){
    extern int type_of_beacon;
    type_of_beacon = IOTBEACON; // Default type of beacon
    ble_server_start();
    vTaskDelay(pdMS_TO_TICKS(3000));
    ble_server_stop();
}


void Identify_Entity(int* R, int* Conf){
    //TODO - Implement the logic to identify the entity
    // This function should set the values of R and Conf based on the identification process
    *R = 1; // Example value, replace with actual identification logic
    *Conf = 1; // Example value, replace with actual confidence level
    ESP_LOGI(TAG, "Identified Entity: R = %d, Conf = %d", *R, *Conf);

    

}