#include <stdio.h>
#include "SPI.h"
#include "lcdFont.h"

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

void lcd_draw_line(uint16_t x_start, uint16_t x_end,uint16_t y_start ,uint16_t y_end,uint16_t color)
{
    uint8_t yflag = 0, xyflag = 0;
    int delta_x ;
    int delta_y ;
    int incrE ;
    int incrNE ;
    int d;  //初始误差
    int x;
    int y;
   
    // 确保坐标在显示范围内  
    assert((x_start < LCD_H_RES && x_end < LCD_H_RES) && (y_start < LCD_V_RES && y_end < LCD_V_RES) && " position must in 240*320 ");

    //确定画线方向
    if (x_end - x_start == 0)   //垂直线
    {
        if(y_end < y_start){xy_SWAP(&y_start,&y_end);}  //交换首尾坐标
        for(; y_start < y_end; y_start++)
        {
            lcd_draw_point(x_start, y_start, color);
        }
    } else if (y_end - y_start == 0)    //水平线
    {
        if(x_end < x_start){xy_SWAP(&x_start,&x_end);}  //交换首尾坐标
        for(;x_start < x_end; x_start++)
        {
            lcd_draw_point(x_start, y_start, color);
        }
    }
    else    // 画斜线 
    {
        if(x_start > x_end)
        {
            /*交换两点坐标*/
			/*交换后不影响画线，但是画线方向由第一、二、三、四象限变为第一、四象限*/
            xy_SWAP(&x_start,&x_end);
            xy_SWAP(&y_start,&y_end);
        }  
        if(y_start > y_end)
        {
            /*将Y坐标取负*/
			/*取负后影响画线，但是画线方向由第一、四象限变为第一象限*/
            y_start = -y_start;
            y_end = -y_end;
            /*置标志位yflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
            yflag = 1;
        }
        if(y_end -y_start > x_end - x_start)    //直线斜率大于1
        {
            /*将X坐标与Y坐标互换*/
			/*互换后影响画线，但是画线方向由第一象限0~90度范围变为第一象限0~45度范围*/
            xy_SWAP(&x_start,&y_start);
            xy_SWAP(&x_end,&y_end);
            /*置标志位xyflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
            xyflag = 1;
        }

        /*使用Bresenham算法画直线*/
		/*算法要求，画线方向必须为第一象限0~45度范围*/
        delta_x = x_end - x_start;
        delta_y = y_end - y_start;
        incrE = 2 * delta_y;
		incrNE = 2 * (delta_y - delta_x);
        d = 2*delta_y - delta_x;  //初始误差
        x = x_start;
        y = y_start;

        /*画起始点，同时判断标志位，将坐标换回来*/
        if(yflag && xyflag)            lcd_draw_point(y, -x, color);
        else if(yflag)                      lcd_draw_point(x, -y, color);
        else if(!yflag && xyflag)   lcd_draw_point(y, x, color);
        else                                    lcd_draw_point(x, y, color);

        while(x < x_end)
        {    
            x++;
            if(d < 0)   //选择E
            {
                d += incrE;
            }
            else    //选择NE
            {
                y++;
                d += incrNE;
            }
            /*画点，同时判断标志位，将坐标换回来*/
            if(yflag && xyflag)            lcd_draw_point(y, -x, color);
            else if(yflag)                      lcd_draw_point(x, -y, color);
            else if(!yflag && xyflag)   lcd_draw_point(y, x, color);
            else                                    lcd_draw_point(x, y, color);
        }
    }
    ESP_LOGI(TAG, "Draw Line OK!");     
}  

//在指定区域填充颜色
// (sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex - sx + 1) * (ey - sy + 1)
void lcd_fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
    uint16_t i, j;
    uint16_t xlen = ex - sx + 1;
    uint8_t *color_copy = ( uint8_t*)&color;
    uint8_t color_data[8] = {color_copy[0], color_copy[1], color_copy[0], color_copy[1],color_copy[0], color_copy[1], color_copy[0], color_copy[1]};
    for (i = sy; i <= ey; i++)
    {
        lcd_set_cursor(sx, i);      /* 设置光标位置 */
        lcd_write_ram_prepare();    /* 开始写入GRAM */

        for (j = 0; j < xlen/4+0.5; j++)
        {
            lcd_data(spi_tft_handle,color_data,8);     /* 写入数据 */
        }
    }
}

void lcd_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color,uint8_t filled)
{
    if(filled == TFT_NOT_FILLED)
    {
        //只画出边框
        lcd_draw_line(x1, x2, y1, y1, color); //上边框
        lcd_draw_line(x1, x2, y2, y2, color); //下边框
        lcd_draw_line(x1, x1, y1, y2, color); //左边框
        lcd_draw_line(x2, x2, y1, y2, color); //右边框 
    }
    else
    {
        //填充矩形
        lcd_fill(x1, y1, x2, y2, color);
    }
}

//画水平线
void lcd_draw_hline(uint16_t x, uint16_t y, uint16_t len, uint16_t color)
{
    if ((len == 0) || (x > lcd_tft.width) || (y > lcd_tft.height))return;
    uint8_t *color_copy = ( uint8_t*)&color;
    uint8_t color_data[4] = {color_copy[0], color_copy[1], color_copy[0], color_copy[1]};
  
    lcd_set_cursor(x, y);      /* 设置光标位置 */
    lcd_write_ram_prepare();    /* 开始写入GRAM */
    for(int i = 0;i<len/2+0.5;i++)
    {
        lcd_data(spi_tft_handle,color_data,4);     /* 写入数据 */
    }
}

//画圆
void lcd_draw_circle(uint16_t A, uint16_t B, uint16_t r, uint16_t color,uint8_t filled)
{
    // 确保坐标在显示范围内
    assert((A+r < LCD_H_RES) && (B+r < LCD_V_RES) && " position must in 240*320 ");
    assert((A-r > 0) && (B-r > 0) && " position must in 240*320 ");
    if(filled == TFT_NOT_FILLED)
    {
        int d = 1-r;
        int X=0;
        int Y=r;

        //先画出x，y轴上的点
        //x轴正方向
        lcd_draw_point(A+0, B+r, color);
        //x轴负方向
        lcd_draw_point(A-0, B-r, color);
        //y轴正方向
        lcd_draw_point(A+Y, B+0, color);
        //y轴负方向
        lcd_draw_point(A-Y, B-0, color);

        //在第一象限遍历每个点
        while (X < Y)
        {
            X++;
            if(d<0) //东方衍生新像素点
            {
                d += 2*X+1;
            }
            else    //东南方衍生新像素点
            {
                Y--;
                d += 2*X-2*Y+1;
            }
            //画出这个点对应的8个点
            lcd_draw_point(A+X, B+Y, color); 
            lcd_draw_point(A+Y, B+X, color); 
            lcd_draw_point(A-X, B-Y,  color); 
            lcd_draw_point(A-Y, B-X,  color);
            lcd_draw_point(A+X, B-Y, color);
            lcd_draw_point(A+Y, B-X, color);
            lcd_draw_point(A-X, B+Y, color);
            lcd_draw_point(A-Y, B+X, color);
        }
    }
    else if(filled == TFT_IS_FILLED)
    {
        uint32_t i;
        uint32_t imax = ((uint32_t)r * 707) / 1000 + 1;
        uint32_t sqmax = (uint32_t)r * (uint32_t)r + (uint32_t)r / 2;
        uint32_t xr = r;

        lcd_draw_hline(A - r, B, 2 * r, color);

        for (i = 1; i <= imax; i++)
        {
            if ((i * i + xr * xr) > sqmax)
            {
                /* draw lines from outside */
                if (xr > imax)
                {
                    lcd_draw_hline (A - i + 1, B + xr, 2 * (i - 1), color);
                    lcd_draw_hline (A - i + 1, B - xr, 2 * (i - 1), color);
                }

                xr--;
            }
            /* draw lines from inside (center) */
            lcd_draw_hline(A - xr, B+ i, 2 * xr, color);
            lcd_draw_hline(A - xr, B - i, 2 * xr, color);
        }
    }
    // ESP_LOGI(TAG, "Draw Circle OK!");
}


/* LCD的画笔颜色和背景色 */
uint32_t g_point_color = 0XF800;    /* 画笔颜色 */
uint32_t g_back_color  = 0XFFFF;    /* 背景色 */
/**
 * @brief       在指定位置显示一个字符
 * @param       x,y  : 坐标
 * @param       chr  : 要显示的字符:" "--->"~"
 * @param       size : 字体大小 12/16/24/32
 * @param       mode : 叠加方式(1); 非叠加方式(0);
 * @param       color : 字符的颜色;
 * @retval      无
 */
void lcd_show_char(uint16_t x, uint16_t y, uint32_t chr, uint8_t size, uint8_t mode, uint16_t color)
{
    uint8_t temp, t1, t;
    uint16_t y0 = y;
    uint8_t csize = 0;
    uint8_t *pfont = 0;

    csize = (size / 8 + ((size % 8) ? 1 : 0)) * (size / 2); /* 得到字体一个字符对应点阵集所占的字节数 */
    chr = chr - ' ';    /* 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库） */

    switch (size)
    {
        case 12:
            pfont = (uint8_t *)asc2_1206[chr];  /* 调用1206字体 */
            break;

        case 16:
            pfont = (uint8_t *)asc2_1608[chr];  /* 调用1608字体 */
            break;

        case 24:
        //     pfont = (uint8_t *)asc2_2412[chr];  /* 调用2412字体 */
            break;

        case 32:
        //     pfont = (uint8_t *)asc2_3216[chr];  /* 调用3216字体 */
            break;

        default:
            return ;
    }

    for (t = 0; t < csize; t++)
    {
        temp = pfont[t];    /* 获取字符的点阵数据 */

        for (t1 = 0; t1 < 8; t1++)   /* 一个字节8个点 */
        {
            if (temp & 0x80)        /* 有效点,需要显示 */
            {
                lcd_draw_point(x, y, color);        /* 画点出来,要显示这个点 */
            }
            else if (mode == 0)     /* 无效点,不显示 */
            {
                lcd_draw_point(x, y, g_back_color); /* 画背景色,相当于这个点不显示(注意背景色由全局变量控制) */
            }

            temp <<= 1; /* 移位, 以便获取下一个位的状态 */
            y++;

            if (y >= lcd_tft.height)return;  /* 超区域了 */

            if ((y - y0) == size)   /* 显示完一列了? */
            {
                y = y0; /* y坐标复位 */
                x++;    /* x坐标递增 */

                if (x >= lcd_tft.width)return;   /* x坐标超区域了 */

                break;
            }
        }
    }
}
