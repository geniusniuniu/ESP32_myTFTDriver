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

    //画直线
    lcd_draw_line(10,10,0,310,BLACK);
    lcd_draw_line(0,220,300,300,BLACK);
    lcd_draw_line(10,150,200,10,BLACK);
    //画圆
    lcd_draw_circle(100,100,80,WHITE,TFT_NOT_FILLED);
    
    lcd_draw_circle(150,150,80,WHITE,TFT_IS_FILLED);

    //画矩形
    lcd_draw_rectangle(50,50,200,200,WHITE,TFT_IS_FILLED);
    //显示一个字符
    ESP_LOGI(TAG,"START");
    lcd_show_char(50,50,'A',16,0,BLACK);
    ESP_LOGI(TAG,"END");
//    xTaskCreatePinnedToCore(tft_show,"tft_lcd",2048,NULL,5,NULL,0);
}  


