#include <stdio.h>
#include <esp_log.h>
#include "sx1261_driver.h"
#include "wubble.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <time.h>

#include "driver/rtc_io.h"
#include "esp32/rtc.h"

//#include "Nimble_adv.h"
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "../../config/config.h"


/*!SECTION
void app_main(void){

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




    while(1) {

        wubble();
        vTaskDelay(pdMS_TO_TICKS(1000));
    };


}

*/



    static const char *TAG = "Main";

    // RTC variable to store last RTC counter value
    RTC_DATA_ATTR uint64_t last_rtc_us = 0;
    
    int64_t current_time_us = 0;


    void app_main(void)
    {    

        // Read current RTC time in microseconds (using RTC slow clock)
        uint64_t rtc_now_us = rtc_time_get();

        if (last_rtc_us == 0) {
            // First boot
            ESP_LOGI(TAG, "First boot, initializing RTC timestamp");
        } else {
            // Calculate deep sleep duration
            uint64_t elapsed_us = rtc_now_us - last_rtc_us;
            double elapsed_sec = elapsed_us / 1000000.0;
            ESP_LOGI(TAG, "ESP32 was in deep sleep for %.2f seconds", elapsed_sec);
        }

        // Save current RTC timestamp for next deep sleep cycle
        last_rtc_us = rtc_now_us;



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
    
        esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

        switch (wakeup_reason) {
            case ESP_SLEEP_WAKEUP_EXT0:
                ESP_LOGI("WAKEUP", "Wakeup caused by EXT0 (WuR_On)");
                //wubble();
                break;
            default:
                ESP_LOGI("WAKEUP", "Wakeup not caused by WuR_On");
                break;
        }

        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << CONFIG_DIO1_GPIO), // Select the GPIO pin
            .mode = GPIO_MODE_INPUT,              // Set as input mode
            .pull_up_en = GPIO_PULLUP_DISABLE,     // Enable pull-up resistor if required
            .pull_down_en = GPIO_PULLDOWN_ENABLE // Disable pull-down resistor
        };
        gpio_config(&io_conf);


        ESP_LOGI(TAG, "gpio_config done for DIO1 GPIO");

        esp_sleep_enable_ext0_wakeup(CONFIG_DIO1_GPIO, HIGH); // Trigger on high level

        ESP_LOGI(TAG, "Going to deep sleep...");

        Listen_WUR();

        gpio_hold_en(CONFIG_RST_GPIO);
        gpio_hold_en(CONFIG_NSS_GPIO);
        gpio_hold_en(CONFIG_DIO1_GPIO);

        esp_deep_sleep_start();



        
/*
        while(1) {
            send_WUR();
            vTaskDelay(pdMS_TO_TICKS(1000));

        }
*/         
}




    



