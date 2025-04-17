#include <stdio.h>
#include "SPI.h"

static const char *TAG = "SPI";
spi_device_handle_t spi_tft_handle = NULL;
lcd_dev lcd_tft;

void spi_init(spi_device_handle_t * spi_handle)  
{  
    spi_bus_config_t spiBusCfg = {};  
    spiBusCfg.miso_io_num = PIN_NUM_MISO;  
    spiBusCfg.mosi_io_num = PIN_NUM_MOSI;  
    spiBusCfg.sclk_io_num   = PIN_NUM_SCLK;  
    spiBusCfg.quadwp_io_num = -1;  
    spiBusCfg.quadhd_io_num = -1;  
    spiBusCfg.max_transfer_sz = PARALLEL_LINES * 320 * 2 + 8;  

    spi_device_interface_config_t spiDevCfg = {};  
    spiDevCfg.clock_speed_hz = 10 * 1000 * 1000;     // Clock out at 10 MHz  
    spiDevCfg.mode = 0;                            // SPI mode 0  
    spiDevCfg.spics_io_num = PIN_NUM_LCD_CS;        // CS pin  
    spiDevCfg.queue_size = 10;  
    spiDevCfg.pre_cb = lcd_spi_pre_transfer_callback;  

    // Initialize the SPI bus  
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &spiBusCfg, SPI_DMA_CH_AUTO));  

    // Attach the LCD to the SPI bus and传递地址给句柄  
    ESP_ERROR_CHECK(spi_bus_add_device((spi_host_device_t)LCD_HOST, &spiDevCfg, spi_handle));  
    ESP_LOGI(TAG, "SPI INIT OVER !");  
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_LCD_DC, dc);
}

//初始化其他非spi总线上的引脚
void gpio_init(void)
{	
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = ((1ULL << PIN_NUM_LCD_DC) | (1ULL << PIN_NUM_LCD_RST) | (1ULL << PIN_NUM_BK_LIGHT));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = true;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void lcd_init(spi_device_handle_t spi_handle)
{
	//Reset the display
    gpio_set_level(PIN_NUM_LCD_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_LCD_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

	//  //detect LCD type
	uint32_t lcd_id = lcd_get_id(spi_handle);
	printf("LCD ID: %08"PRIx32"\n", lcd_id);

	//Send all the commands to initialize the LCD
	lcd_cmd(spi_handle,0xCF,false);
	lcd_data(spi_handle, (uint8_t[]){0x00, 0xC1, 0X30}, 3);
	lcd_cmd(spi_handle,0xED,false);
	lcd_data(spi_handle, (uint8_t[]){0x64, 0x03, 0x12, 0x81}, 4);
	lcd_cmd(spi_handle,0xE8,false);
	lcd_data(spi_handle, (uint8_t[]){0x85, 0x10, 0x7A}, 3);
	lcd_cmd(spi_handle,0xCB,false);
	lcd_data(spi_handle, (uint8_t[]){0x39, 0x2C, 0x00, 0x34, 0x02}, 5);
	lcd_cmd(spi_handle,0xF7,false);
	lcd_data(spi_handle, (uint8_t[]){0x20}, 1);
	lcd_cmd(spi_handle,0xEA,false);
	lcd_data(spi_handle, (uint8_t[]){0x00, 0x00}, 2);
	lcd_cmd(spi_handle,0xC0,false);
	lcd_data(spi_handle, (uint8_t[]){0x1B}, 1);
	lcd_cmd(spi_handle,0xC1,false);
	lcd_data(spi_handle, (uint8_t[]){0x01}, 1);
	lcd_cmd(spi_handle,0xC5,false);
	lcd_data(spi_handle, (uint8_t[]){0x30, 0x30}, 2);
	lcd_cmd(spi_handle,0xC7,false);
	lcd_data(spi_handle, (uint8_t[]){0xB7}, 1);
	lcd_cmd(spi_handle,0x36,false);
	lcd_data(spi_handle, (uint8_t[]){0x48}, 1);
	lcd_cmd(spi_handle,0x3A,false);
	lcd_data(spi_handle, (uint8_t[]){0x55}, 1);
	lcd_cmd(spi_handle,0xB1,false);
	lcd_data(spi_handle, (uint8_t[]){0x00, 0x1A}, 2);
	lcd_cmd(spi_handle,0xB6,false);
	lcd_data(spi_handle, (uint8_t[]){0x0A, 0xA2}, 2);
	lcd_cmd(spi_handle,0xF2,false);
	lcd_data(spi_handle, (uint8_t[]){0x00}, 1);
	lcd_cmd(spi_handle,0x26,false);
	lcd_data(spi_handle, (uint8_t[]){0x01}, 1);
	lcd_cmd(spi_handle,0xE0,false);
	lcd_data(spi_handle, (uint8_t[]){0x0F, 0x2A, 0x28, 0x08, 0x0E, 0x08, 0x54, 0xA9,0x43, 0x0A, 0x0F, 0x00, 0x00, 0x00,0x00}, 15);
	lcd_cmd(spi_handle,0xE1,false);
	lcd_data(spi_handle, (uint8_t[]){0x00, 0x15, 0x17, 0x07, 0x11, 0x06, 0x2B, 0x56, 0x3C, 0x05, 0x10, 0x0F, 0x3F, 0x3F, 0x0F}, 15);
	lcd_cmd(spi_handle,0x2A,false);
	lcd_data(spi_handle, (uint8_t[]){0x00, 0x00, 0x00, 0xEF}, 4);
	lcd_cmd(spi_handle,0x2B,false);
	lcd_data(spi_handle, (uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4);
	lcd_cmd(spi_handle,0x2C,false);
	lcd_data(spi_handle, (uint8_t[]){0}, 0);
	lcd_cmd(spi_handle,0xB7,false);
	lcd_data(spi_handle, (uint8_t[]){0x07}, 1);
	lcd_cmd(spi_handle,0xB6,false);
	lcd_data(spi_handle, (uint8_t[]){0x0A, 0x82,0x27,0x00}, 4);
	lcd_cmd(spi_handle,0x11,false);
	vTaskDelay(100/portTICK_PERIOD_MS);
	lcd_cmd(spi_handle,0x29,false);
	vTaskDelay(100/portTICK_PERIOD_MS);

    //竖屏模式
    lcd_display_dir(0);

	//Enable backlight
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    lcd_clear(RED); // Clear the screen with white color
	ESP_LOGI(TAG, "LCD initialized");
}

//获取LCD ID
uint32_t lcd_get_id(spi_device_handle_t spi_handle)
{
    // When using SPI_TRANS_CS_KEEP_ACTIVE, bus must be locked/acquired
    spi_device_acquire_bus(spi_handle, portMAX_DELAY);

    //get_id cmd
    lcd_cmd(spi_handle, 0x04, true);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;

    esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
    assert(ret == ESP_OK);

    // Release bus
    spi_device_release_bus(spi_handle);

    return *(uint32_t*)t.rx_data;
}

//轮询发送命令
void lcd_cmd(spi_device_handle_t spi_handle, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t = {};
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                  			 //Command is 8 bits
    t.tx_buffer = &cmd;               //The data is the cmd itself
    t.user = (void*)0;                   //D/C needs to be set to 0
    if (keep_cs_active) {
        t.flags = SPI_TRANS_CS_KEEP_ACTIVE;   //Keep CS active after data transfer
    }
    ret = spi_device_polling_transmit(spi_handle, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}

//发送数据
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t = {};
    if (len == 0) {
        return;    //no need to send anything
    }
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len * 8;             //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;             //Data
    t.user = (void*)1;              //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); //Transmit!
    assert(ret == ESP_OK);          //Should have had no issues.
}

void lcd_write_reg(const uint8_t cmd, const uint8_t *data)
{
    lcd_cmd(spi_tft_handle, cmd, false);
    lcd_data(spi_tft_handle, data, 1);
}

/**
 * @brief       准备写GRAM
 * @param       无
 * @retval      无
 */
__attribute__((always_inline)) void lcd_write_ram_prepare(void)
{
    lcd_cmd(spi_tft_handle,lcd_tft.wramcmd,false);
}


void lcd_display_on(spi_device_handle_t spi)
{
	lcd_cmd(spi, 0x29, false); // Display on
	gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
}

void lcd_display_off(spi_device_handle_t spi)
{
	lcd_cmd(spi, 0x28, false); // Display off
	gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
}

/**
 * @brief       设置LCD显示方向
 * @param       dir:0,竖屏; 1,横屏
 * @retval      无
 */
void lcd_display_dir(uint8_t dir)
{
    lcd_tft.dir = dir;
    if (dir == 0)   //竖屏
    {
        lcd_tft.width = LCD_H_RES;
        lcd_tft.height = LCD_V_RES;
        lcd_tft.wramcmd = 0x2C;
        lcd_tft.setxcmd = 0x2A;
        lcd_tft.setycmd = 0x2B;
        lcd_cmd(spi_tft_handle, 0x36, false);
        lcd_data(spi_tft_handle, (uint8_t[]){0x00}, 1);
    } 
    else if (dir == 1)   //横屏
    {
        lcd_tft.width = LCD_V_RES;
        lcd_tft.height = LCD_H_RES;
        lcd_tft.wramcmd = 0x2C;
        lcd_tft.setxcmd = 0x2A;
        lcd_tft.setycmd = 0x2B;
    }
    lcd_scan_dir(DFT_SCAN_DIR);
}

void lcd_scan_dir(uint8_t dir)
{
    uint16_t regval = 0;
    uint16_t dirreg = 0x36;
    if(dir == L2R_U2D)
    {
         regval |= (0 << 7) | (0 << 6) | (0 << 5);
    }
    regval |= 0X00;
    lcd_write_reg(dirreg, (uint8_t *)&regval);
    if (regval & 0X20)
        {
            if (lcd_tft.width < lcd_tft.height)   /* 交换X,Y */
            {
                uint16_t temp = lcd_tft.width;
                lcd_tft.width = lcd_tft.height;
                lcd_tft.height = temp;
            }
        }
        else
        {
            if (lcd_tft.width > lcd_tft.height)   /* 交换X,Y */
            {
                uint16_t temp = lcd_tft.width;
                lcd_tft.width = lcd_tft.height;
                lcd_tft.height = temp;
            }
        }
        //设置显示开窗大小
        lcd_cmd(spi_tft_handle, lcd_tft.setxcmd, false);
        lcd_data(spi_tft_handle, (uint8_t[]){0x00, 0x00, (lcd_tft.width - 1) >> 8,  ((lcd_tft.width - 1) & 0XFF)}, 4);
        lcd_cmd(spi_tft_handle, lcd_tft.setycmd, false);
        lcd_data(spi_tft_handle, (uint8_t[]){0x00, 0x00, (lcd_tft.height - 1) >> 8, ((lcd_tft.height - 1) & 0XFF)}, 4);
}

/**
 * @brief       清屏函数
 * @param       color: 要清屏的颜色
 * @retval      无
 */
void lcd_clear(uint16_t color)
{
    uint8_t *color_copy = ( uint8_t*)&color;
    uint8_t color_data[8] = {color_copy[0], color_copy[1], color_copy[0], color_copy[1],color_copy[0], color_copy[1],color_copy[0], color_copy[1]};
    uint32_t totalpoint = lcd_tft.width * lcd_tft.height / 4;    //像素点数
    lcd_set_cursor(0x00, 0x0000);   /* 设置光标位置 */
    lcd_write_ram_prepare();
    for (int index = 0; index < totalpoint; index++)
    {
        lcd_data(spi_tft_handle, color_data, 8);
    }
}

/**
 * @brief       设置光标位置(对RGB屏无效)
 * @param       x,y: 坐标
 * @retval      无
 */
void lcd_set_cursor(uint16_t x, uint16_t y)
{
    lcd_cmd(spi_tft_handle,lcd_tft.setxcmd, false);
    lcd_data(spi_tft_handle, (uint8_t[]){x >> 8, x & 0XFF}, 2);
    lcd_cmd(spi_tft_handle,lcd_tft.setycmd, false);
    lcd_data(spi_tft_handle, (uint8_t[]){y >> 8, y & 0XFF}, 2);
}


void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
    lcd_set_cursor(x, y);       /* 设置光标位置 */
    lcd_write_ram_prepare();    /* 开始写入GRAM */
    lcd_data(spi_tft_handle, (uint8_t *)&color, 2); /* 写入点的颜色 */
}