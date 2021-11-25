#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "string.h"
#include <esp_http_server.h>
//#include <sys/param.h>
#include "esp_netif.h"
#include "sdkconfig.h"
#include "u8g2.h"
#include "spi_handler.h"
#include "spi_interface.h"



LCD_ST7920::LCD_ST7920(transmission_mode tr_mode, spi_handler* handler)
    :spi(handler)
{
    if(tr_mode == serial)
    {
        //Set psb pin to low
        gpio_set_level(LCD_PIN_NUM_PSB, 0);
    }

    //framebuffer = new uint16_t[LCD_X_SIZE/LCD_DATA_WIDTH][LCD_Y_SIZE/LCD_SEGMENTS];//16 bits in single line/ 16 segments / 32 lines  -> 1kB
    framebuffer = new uint16_t [LCD_X_SIZE * LCD_Y_SIZE];
    memset(framebuffer, 0, LCD_X_SIZE * LCD_Y_SIZE);
}

LCD_ST7920::~LCD_ST7920()
{
    delete (framebuffer);
}

void LCD_ST7920::put_text(const char *text, uint8_t length)
{
    int counter = length;
    char ch;
    while (counter--)
    {
        ch = *text++;
        spi->send_byte_spi(mode::data, (const uint8_t)ch);
        ESP_LOGD(TAG, "sending character  %c. \n", ch);
    }
}

void LCD_ST7920::lcd_init(lcd_mode mode)
{
    if(mode == lcd_mode::text)
    {
        spi->send_byte_spi(mode::command, 0x30);  //Basic functions
        spi->send_byte_spi(mode::command, 0x01);  //Clear screen
        spi->send_byte_spi(mode::command, 0x06);
        spi->send_byte_spi(mode::command, 0x0C);
    }else{
        //Set the functions to extended
        spi->send_byte_spi(mode::command, LCD_EXTEND_FUNCTION); //set 8 bit interface
        spi->send_byte_spi(mode::command, LCD_EXTEND_FUNCTION | 0x04 ); //RE=1 - extende function
        spi->send_byte_spi(mode::command, LCD_EXTEND_FUNCTION | 0x04 | 0x02); //RE=1 and G=1 -> graphics mode on
    }
}


void LCD_ST7920::set_start_address(uint8_t addr)
{
    spi->send_byte_spi(mode::command, addr);
}

void LCD_ST7920::clear(lcd_mode mode)
{
    if(mode == lcd_mode::text)
    {
        spi->send_byte_spi(mode::command, 1);
        //set the AC controll - auto shift 
        spi->send_byte_spi(mode::command, 20);
    }else{

        for (uint8_t x = 0; x < 16; x++)
        {
            for (uint8_t y = 0; y < 32; y++) 
            {
                spi->send_byte_spi(mode::command, 0x80 | y);
                spi->send_byte_spi(mode::command, 0x80 | x);
                spi->send_byte_spi(mode::data, 0x00);
                spi->send_byte_spi(mode::data, 0x00);
            }
        }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

//ST7920 always accept 24 bits
//First byte has:
// 5 bits set to 1 as sync string
//2 bits for RW and RS
//last bit 0 as control
//Second byte consist 4 bits of data D7-D4
//and 4 bits 0 as control
//Third byte consist data 
//4 bits D3-D0
//and 4 control bits 0

void LCD_ST7920::set_point(point a, point b, uint8_t valuea, uint16_t valueb)
{
    //LCD is split into 4 section vertically, 16 px each in vertical
    //and 2 section horizontally

    //Set the address of point for horizontal section
    // 1. x=(0,0) => 0x80  2. x=(0,0) => 0x88
    // 1. y=(0,0) => 0x80 + y pos

    /*
        set point 0,0-> set address to 0x80 and 0x80 for vertical and horizontal
        and set 0x01 as data


    */

    /*for (uint8_t x = a.x; x < a.y; x++)
        {
            for (uint8_t y = b.x; y < b.y; y++) 
            {*/
    uint8_t data1, data2;
    data1 = (valueb >> 8);
    data2 = ((valueb << 8) >> 8);
    if(data2)
        data1 = 0;
    spi->send_byte_spi(mode::command, 0x80 | a.y);
    spi->send_byte_spi(mode::command, 0x80 | a.x);
    ESP_LOGI(TAG, "Pixel high %d  ->  ", data1);
    spi->send_byte_spi(mode::data, data1);
    ESP_LOGI(TAG, "Pixel low %d  \n", data2);
    spi->send_byte_spi(mode::data, data2);
    //spi->send_byte_spi(mode::data, valueb);
    //spi->send_byte_spi(mode::data, valueb);
    //spi->send_byte_spi(mode::data, valueb);
    //spi->send_byte_spi(mode::data, valueb);

    /* }
        }*/

    /*
    uint8_t vert, horiz;

    if(a.y >= 32)
    {
        vert  = 0x80 | (a.y - 32);
        horiz = 0x88 | a.x;
    }else{
        vert  = 0x80 | a.y;
        horiz = 0x80 | a.x;
    }
    //send the address
    spi->send_byte_spi(mode::command, vert);
    spi->send_byte_spi(mode::command, horiz);
    //set the point
    spi->send_byte_spi(mode::data, 0xFF);
    spi->send_byte_spi(mode::data, 0xFF);
    */
}





void LCD_ST7920::draw_line(point a, point b)
{
    //set_point(a);
    //set_point(b);
}

// Graphic functions
void LCD_ST7920::fill_GDRAM() //unsigned char *bitmap) {
{

    unsigned char i, j, k;
 
    for ( i = 0 ; i < 2 ; i++ ) {
        for ( j = 0 ; j < 32 ; j++ ) {
            spi->send_byte_spi(mode::command, GRAPHIC_RAM_ADDRESS | j) ;  //#define SET_GRAPHIC_RAM_ADDRESS 0x80
            if ( i == 0 ) {
                spi->send_byte_spi(mode::command, GRAPHIC_RAM_ADDRESS) ;
            } else {
                spi->send_byte_spi(mode::command, GRAPHIC_RAM_ADDRESS | 0x08) ;
            }
            for ( k = 0 ; k < 16 ; k++ ) {
                spi->send_byte_spi(mode::data, 1); //*bitmap++ ) ;
            }
        }
    }
}

void LCD_ST7920::highlight_menu_item(uint8_t idx, bool fill)
{
    idx &= 0x03; // 4 rows only

    uint8_t y = idx * 16;
    uint8_t x_addr = 0x80;

    // adjust cooridinates and address
    if (y >= 32) {
        y -= 32;
        x_addr = 0x88;
    }

    for (uint8_t x = 0; x < 8; x++) {
        spi->send_byte_spi(mode::command, 0x80 | y);
        spi->send_byte_spi(mode::command, x_addr | x);
        fill ? spi->send_byte_spi(mode::data, 0xFF) : spi->send_byte_spi(mode::data, 0x00);
        fill ? spi->send_byte_spi(mode::data, 0xFF) : spi->send_byte_spi(mode::data, 0x00);

        spi->send_byte_spi(mode::command, 0x80 | (y + 15));
        spi->send_byte_spi(mode::command, x_addr | x);
        fill ? spi->send_byte_spi(mode::data, 0xFF) : spi->send_byte_spi(mode::data, 0x00);
        fill ? spi->send_byte_spi(mode::data, 0xFF) : spi->send_byte_spi(mode::data, 0x00);
    }

    for (uint8_t y1 = y + 1; y1 < y + 15; y1++) {
        spi->send_byte_spi(mode::command, 0x80 | y1);
        spi->send_byte_spi(mode::command, x_addr);
        fill ? spi->send_byte_spi(mode::data, 0x80) : spi->send_byte_spi(mode::data, 0x00);
        spi->send_byte_spi(mode::data, 0x00);

        spi->send_byte_spi(mode::command, 0x80 | y1);
        spi->send_byte_spi(mode::command, x_addr + 7);
        spi->send_byte_spi(mode::data, 0x00);
        fill ? spi->send_byte_spi(mode::data, 0x01) : spi->send_byte_spi(mode::data, 0x00);
    }
}

void LCD_ST7920::set_point_f(point a, uint8_t mode)
{
    //Each line in segment has 16 bits
    //LCD has 2 horizontal segments
    //Each bit in framework is related to smae memory in ST
    //1. Find a vertical segment - here are a.x /8 segments - it shall be =< LCD_X_SIZE /8
    //uint8_t x_segment = a.x / LCD_DATA_WIDTH;
    //2. Copy actual value of segment for horizontal line to variable 16 bits
    //uint16_t line_state = framebuffer[x_segment][a.y];
    //set the point with value of mode
    //the the x_segment*8 - it is a first point in segment (position)
    //get diff a.x and first point of segment
    //shift mode by diff bits
    //uint8_t bit_position = a.x - (x_segment * LCD_DATA_WIDTH);
    //line_state |= (mode <<  bit_position);
    //Set new value to framebuffer
    //framebuffer[x_segment][a.y] = line_state;
    //Now the LCD shall be refreshed 
    //By triggering interrupt (30 times/sec)


    uint8_t  indexa = a.x / LCD_DATA_WIDTH;
    uint16_t index = (a.y * (LCD_X_SIZE / LCD_DATA_WIDTH)) + indexa;
    
    uint16_t line_value = framebuffer[index];

    //set the value, take a bit
    uint8_t bit_position = a.x % LCD_DATA_WIDTH;
    if(mode)
        line_value |= (mode << bit_position); //set point
    else
        line_value &= ~(1UL << bit_position); //clear point
    framebuffer[index] = line_value;
}