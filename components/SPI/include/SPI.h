#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

// Using SPI2 in the example
#define LCD_HOST                             SPI2_HOST
//#define TOUCH_HOST                       SPI3_HOST

#define LCD_BK_LIGHT_ON_LEVEL       1
#define LCD_BK_LIGHT_OFF_LEVEL      0

#define PIN_NUM_SCLK                         3
#define PIN_NUM_MOSI                        45
#define PIN_NUM_MISO                        46
//通过DC引脚的高低电平来直接指示当前传输的内容是命令还是数据，简化通信协议
#define PIN_NUM_LCD_DC                    47   

#define PIN_NUM_LCD_RST                   21
#define PIN_NUM_LCD_CS                     14
#define PIN_NUM_BK_LIGHT                 0


// #define EXAMPLE_PIN_NUM_TOUCH_CS          1
// #define EXAMPLE_PIN_NUM_TOUCH_SCLK      42
// #define EXAMPLE_PIN_NUM_TOUCH_MOSI     2
// #define EXAMPLE_PIN_NUM_TOUCH_MISO     41

#define LCD_H_RES              240
#define LCD_V_RES               320

/* 扫描方向定义 */
#define L2R_U2D         0           /* 从左到右,从上到下 */
#define L2R_D2U         1           /* 从左到右,从下到上 */
#define R2L_U2D         2           /* 从右到左,从上到下 */
#define R2L_D2U         3           /* 从右到左,从下到上 */

#define U2D_L2R         4           /* 从上到下,从左到右 */
#define U2D_R2L         5           /* 从上到下,从右到左 */
#define D2U_L2R         6           /* 从下到上,从左到右 */
#define D2U_R2L         7           /* 从下到上,从右到左 */

#define DFT_SCAN_DIR    L2R_U2D     /* 默认的扫描方向 */

/* 常用画笔颜色 */
#define WHITE           0xFFFF      /* 白色 */
#define BLACK           0x0000      /* 黑色 */
#define RED             0xF800      /* 红色 */
#define GREEN           0x07E0      /* 绿色 */
#define BLUE            0x001F      /* 蓝色 */ 
#define MAGENTA         0XF81F      /* 品红色/紫红色 = BLUE + RED */
#define YELLOW          0XFFE0      /* 黄色 = GREEN + RED */
#define CYAN            0X07FF      /* 青色 = GREEN + BLUE */  

/* 非常用颜色 */
#define BROWN           0XBC40      /* 棕色 */
#define BRRED           0XFC07      /* 棕红色 */
#define GRAY            0X8430      /* 灰色 */ 
#define DARKBLUE        0X01CF      /* 深蓝色 */
#define LIGHTBLUE       0X7D7C      /* 浅蓝色 */ 
#define GRAYBLUE        0X5458      /* 灰蓝色 */ 
#define LIGHTGREEN      0X841F      /* 浅绿色 */  
#define LGRAY           0XC618      /* 浅灰色(PANNEL),窗体背景色 */ 
#define LGRAYBLUE       0XA651      /* 浅灰蓝色(中间层颜色) */ 
#define LBBLUE          0X2B12      /* 浅棕蓝色(选择条目的反色) */ 

extern spi_device_handle_t spi_tft_handle;

/* LCD重要参数集 */
typedef struct
{
    uint16_t width;     /* LCD 宽度 */
    uint16_t height;    /* LCD 高度 */
    uint16_t id;       		/* LCD ID */
    uint8_t dir;        	/* 横屏还是竖屏控制：0，竖屏；1，横屏。 */
    uint16_t wramcmd;   /* 开始写gram指令 */
    uint16_t setxcmd;   /* 设置x坐标指令 */
    uint16_t setycmd;   /* 设置y坐标指令 */
} lcd_dev;

void spi_init(spi_device_handle_t * spi_handle);
void gpio_init(void);
void lcd_init(spi_device_handle_t spi_handle);
uint32_t lcd_get_id(spi_device_handle_t spi_handle);
void lcd_spi_pre_transfer_callback(spi_transaction_t *t);
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd, bool keep_cs_active);
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len);

void lcd_display_dir(uint8_t dir);
void lcd_scan_dir(uint8_t dir);
void lcd_clear(uint16_t color);
void lcd_set_cursor(uint16_t x, uint16_t y);
void lcd_write_ram_prepare(void);
void lcd_display_on(spi_device_handle_t spi);
void lcd_display_off(spi_device_handle_t spi);

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color);