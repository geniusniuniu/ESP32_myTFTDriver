#include <stdio.h>  
#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"  
#include "driver/gpio.h"  
#include "SPI.h"
#include "esp_log.h"

static const char *TAG = "app_main";

// void tft_show(void *arg)
// {
//     while(1)
//     {
//         // lcd_display_off(spi_tft_handle);
//         // vTaskDelay(1000 / portTICK_PERIOD_MS);

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }

// }

void app_main(void)
{ 
    gpio_init();
    spi_init(&spi_tft_handle);
    lcd_init(spi_tft_handle);
    ESP_LOGI(TAG, "ALL initialized");

    for (int i = 10; i < 100; i++)
    {
        //画点
        lcd_draw_point(i, i,BLACK);
    }
//    xTaskCreatePinnedToCore(tft_show,"tft_lcd",2048,NULL,5,NULL,0);
}  


