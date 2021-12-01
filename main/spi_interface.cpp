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
#include "basic_figures.h"
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
    framebuffer = new uint16_t [LCD_X_SIZE * LCD_Y_SIZE/LCD_DATA_WIDTH];
    memset(framebuffer, 0, (LCD_X_SIZE * LCD_Y_SIZE/LCD_DATA_WIDTH) * sizeof(uint16_t));
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
        //for (int idx = 0; idx < 512;  idx++)
            //framebuffer[idx] = 0x0000;
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
    /*if(data2)
        data1 = 0;*/
    if(valuea == 0)
    {
        spi->send_byte_spi(mode::command, 0x80 | a.y);
        spi->send_byte_spi(mode::command, 0x80 | a.x);
    }
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
/*The memory counter is automatically increased by 1 
* whe we set the position to 0,0 - segment and horizontal
* and send the uint16 value, the lcd will fill the line
* and jump to next segment
* after 16 segment we need to move memory address to 
* segmen 0 and horizontal +1
* it is usefull to copy framework to LCD
*/
void LCD_ST7920::set_segment_line(uint8_t segment, uint8_t hor_pos, uint16_t value)
{
    if(segment == 0)
    {
        spi->send_byte_spi(mode::command, 0x80 | hor_pos);
        spi->send_byte_spi(mode::command, 0x80 );
    }
    uint8_t data1, data2;
    data1 = (value >> 8);
    data2 = ((value << 8) >> 8);

    ESP_LOGI(TAG, "Pixel high %d  ->  ", data1);
    spi->send_byte_spi(mode::data, data1);
    ESP_LOGI(TAG, "Pixel low %d  \n", data2);
    spi->send_byte_spi(mode::data, data2);

}



void LCD_ST7920::draw_line(point a, point b)
{
    //set_point(a);
    //set_point(b);
}

// Graphic functions
void LCD_ST7920::fill_GDRAM() //unsigned char *bitmap) {
{
    /*1. Set the memory address to 0.0
    * 2. Get 16 two-bytes from framebuffer
    * 3. Copy data to display
    * The memory counter will be automatically increased
    * Increase the horizontal counter
    * 1. Set the memory address to 0,horizontal
    * Repeat 2. and 3.
    */
    uint8_t data1, data2;
    uint8_t index = 0;
    uint16_t  idx; //index is used to went by horizontal
    uint8_t counter;
    uint16_t value;
    for (; index < (LCD_Y_SIZE / LCD_SEGMENTS); index++)
    {   
        spi->send_byte_spi(mode::command, 0x80 | index);// - vertical address - 0->64
        spi->send_byte_spi(mode::command, 0x80 );       // - horizontal address 0-16 (segment)
        counter = 0;
        idx = index * LCD_SECTIONS;
        //value = &framebuffer[idx];
        //ESP_LOGI(TAG, "index %d  value=%d   \n", idx, *value);
        for (; counter < LCD_SECTIONS; counter++)
        {
            value = framebuffer[idx + counter];
            //ESP_LOGI(TAG, "index %d  value=%d   \n", idx+counter, value);
            data1 = (value >> 8);
            data2 = ((value << 8) >> 8);
            //if(value)
               // ESP_LOGI(TAG, "value %d  data1=%d  data2=%d \n", value, data1, data2);
            //ESP_LOGI(TAG, "Pixel high %d  ->  ", data1);
            spi->send_byte_spi(mode::data, data1);
            //ESP_LOGI(TAG, "Pixel low %d  \n", data2);
            //vTaskDelay(1 / portTICK_PERIOD_MS);
            spi->send_byte_spi(mode::data, data2);
            //vTaskDelay(1 / portTICK_PERIOD_MS);
            //value++;

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

    /*
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
    */

    /* The point position is always calculated 
    * first 16 two-bytes are equivalent y=0 and 0 <= x < 128
    * second two-bytes is equivalent y=1 and 0 <= x < 128
    * */
    //horizontal size has 64
    //but only 32 can be used for sgment/section
    //find the element in memory

    //memory is organised in following way:
    /*
    			y	x	y	x
index	0	15	0	127	32	127
        16	31	1	127	33	127
        32	47	2	127	34	127
        48	63	3	127	35	127
        64	79	4	127	36	127
        80	95	5	127	37	127
        96	111	6	127	38	127
        112	127	7	127	39	127
        128	143	8	127	40	127
        144	159	9	127	41	127
        160	175	10	127	42	127
        176	191	11	127	43	127
        192	207	12	127	44	127
        208	223	13	127	45	127
        224	239	14	127	46	127
        240	255	15	127	47	127
        256	271	16	127	48	127
        272	287	17	127	49	127
        288	303	18	127	50	127
        304	319	19	127	51	127
        320	335	20	127	52	127
        336	351	21	127	53	127
        352	367	22	127	54	127
        368	383	23	127	55	127
        384	399	24	127	56	127
        400	415	25	127	57	127
        416	431	26	127	58	127
        432	447	27	127	59	127
        448	463	28	127	60	127
        464	479	29	127	61	127
        480	495	30	127	62	127
        496	511	31	127	63	127
    */
    
        //if y>31 then y -= 32;
        //we have 16 two-bytes elements
        //first 8 elements are covering  0<=y<32 and 0<=x<128
        //second 8 elements are covering 32<=y<64 and 0<=x<128

    uint16_t hor_index = ((a.y > 31 ? (a.y - 32) : a.y)) * LCD_SECTIONS;
    //8 block for 0=<y<32 and next 8 blocks for 32=< y < 64
    //section is identified by x postion / 16 ; +8 if y>31
    uint8_t vert_index = (a.x / LCD_DATA_WIDTH) + (a.y > 31 ? (8) : 0);  //ok
    uint16_t mem_block = hor_index + vert_index; //ok
    //ESP_LOGI(TAG, "mem_block %d  x=%d  y=%d  \n", mem_block, a.x, a.y);

    uint16_t line_value = framebuffer[mem_block];
    //set the value, take a bit
    uint8_t bit_position = 15 - a.x % LCD_DATA_WIDTH;
    if(mode)
        line_value |= (mode << bit_position); //set point
    else
        line_value &= ~(1UL << bit_position); //clear point

    //ESP_LOGI(TAG, "bit_pos %d  line_value=%d  \n", bit_position, line_value);
    framebuffer[mem_block] = line_value;
}

void LCD_ST7920::show_frame_buff()
{
    for (uint16_t idx = 0; idx < (LCD_X_SIZE * LCD_Y_SIZE / LCD_DATA_WIDTH); idx++)
    {
        if(framebuffer[idx])
            ESP_LOGI(TAG, "mem_block %d  value=%d  \n", idx, framebuffer[idx]);
    }
}