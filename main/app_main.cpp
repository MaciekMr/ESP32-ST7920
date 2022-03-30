/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include "esp_netif.h"
#include "sdkconfig.h"
#include "u8g2.h"
#include "spi_handler.h"
#include "basic_figures.h"
#include "spi_interface.h"
#include "timer_control.h"
#include "task.h"


extern "C" {
	void app_main(void);
}

static xQueueHandle s_timer_queue;

void show_info()
{
     /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    for (int i = 2; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    show_info();
    printf("Initialise task \n");
    //vAddTask();
    start_thread();

        printf("Initialise queue \n");
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));
    set_queue(s_timer_queue);
    //Reset ST7920
    /*gpio_set_level(LCD_PIN_NUM_RST, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(LCD_PIN_NUM_RST, 1);*/
    ESP_LOGD(TAG,"Starting SPI now.\n");

    //Wait until LCD supply to be stable - 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);

    //Configure SPI
    ESP_LOGD(TAG,"handler is creating \n");
    spi_handler handler(LCD_PIN_NUM_MISO, LCD_PIN_NUM_MOSI, LCD_PIN_NUM_CLK, LCD_TRANSFER_MAX_SIZE);
    
    //Initiate SPI to communicate with LCD
    ESP_LOGD(TAG,"handler is initiating \n");
    handler.init_spi_master(LCD_SPI_HOST);
    
    //Add LCD device to SPI driver
    ESP_LOGD(TAG,"device is initiating \n");
    handler.spi_device_init(SPI_CLK_FREQ, LCD_INPUT_DELAY_NS);

    //Now ready to send data via SPI
    //Set the LCD interface
    ESP_LOGD(TAG,"ST7920 is creaitng \n");
    LCD_ST7920 lcd(transmission_mode::serial, &handler);
    
    //Initialise LCD
    lcd.lcd_init(lcd_mode::text); //initiate into text mode
    //Send text to LCD
    const char *text = "ST7920 Graphic LCD";
    ESP_LOGD(TAG,"Text is sending \n");
    //lcd.set_start_address(0x88);
    lcd.clear(lcd_mode::text);
    
    lcd.put_text(text, 18);
    lcd.clear(lcd_mode::text);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    //ST7920_setGraphicMode(true);
    
    lcd.lcd_init(lcd_mode::grap);

    lcd.clear(lcd_mode::grap);

    
    //Set two linex in teh framebuffer
    //draw two boxex
    //poont(3,3) -> point (50,3)
    //point(3,25) -> point (50,25)
    printf("Before \n");
    //lcd.show_frame_buff();
    uint8_t point_idx;

    /*lcd.set_point_f(point{3,3}, 1);

    lcd.set_point_f(point{15,3}, 1);

    lcd.set_point_f(point{18,3}, 1);
    lcd.set_point_f(point{127,3}, 1);
    */
    uint8_t delta = 0;
    for (uint8_t inside_box = 0; inside_box < 8; inside_box++)
    {
        
        for (point_idx = (2+delta); point_idx < (125 - delta); point_idx++)
        {
            //draw vertical line
            lcd.set_point_f(point{point_idx, (uint8_t)(3+delta)}, 1);
            lcd.set_point_f(point{point_idx, (uint8_t)(62-delta)}, 1);
        }
        //printf("After \n");
        //lcd.show_frame_buff();
        
        for (point_idx = 3+delta; point_idx <= 62-delta; point_idx ++)
        {
            //draw vertical line
            lcd.set_point_f(point{(uint8_t)(2+delta), point_idx}, 1);
            lcd.set_point_f(point{(uint8_t)(125-delta), point_idx}, 1);
        }
        delta += 4;
    }
    printf("Timer is starting \n");
    set_timer();
    /*
    for (point_idx = 2; point_idx < 125; point_idx++)
    {
        //draw vertical line
        lcd.set_point_f(point{point_idx, 3}, 1);
        lcd.set_point_f(point{point_idx, 62}, 1);
    }
    //printf("After \n");
    //lcd.show_frame_buff();
    
    for (point_idx = 3; point_idx < 26; point_idx ++)
    {
        //draw vertical line
        lcd.set_point_f(point{2, point_idx}, 1);
        lcd.set_point_f(point{50, point_idx}, 1);
    }
    */
    lcd.fill_GDRAM();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    //void show_counter();


    while (1) {
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        if (evt.info.auto_reload) {
            printf("Timer Group with auto reload\n");
        } else {
            printf("Timer Group without auto reload\n");
        }
        printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

        /* Print the timer values passed by event */
        printf("------- EVENT TIME --------\n");
        print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        //timer_get_counter_value(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
        print_timer_counter(evt.timer_counter_value); //task_counter_value);
    }

    /*int x_val = 0xFFFF;
    for (uint8_t hor_pos = 0; hor_pos < 32; hor_pos++)
    {
        for (uint8_t xbank = 0; xbank < 16; xbank++)
        {
            lcd.set_segment_line(xbank, hor_pos, x_val);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "counter hor_pos:%d xbank:%d\n", hor_pos, xbank);
        }
    }*/

    /*
        for (uint8_t xbank = 0; xbank < 16; xbank++)
        {
            x_val = 0xFFFF;
            for (uint8_t count = 0; count < 32; count++)
            {
                lcd.set_point(point{xbank, count}, point{0, count}, count, x_val);
                //x_val = (x_val >> 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                ESP_LOGI(TAG, "counter %d %d\n", count, x_val);
            }
        }*/
    //lcd.fill_GDRAM();
    //ST7920_ClearGraphicMem();
    //lcd.clear(lcd_mode::grap);
    // highlight title
    //lcd.set_point(point{10, 10});
    //lcd.draw_line(point{3, 3}, point{8, 8});
    /*
    for (byte y = 0; y < 15; y++)
    {
        for (byte x = 0; x < 8; x++) {
            ST7920_Write(LCD_COMMAND, 0x80 | y);
            ST7920_Write(LCD_COMMAND, 0x80 | x);
            ST7920_Write(LCD_DATA, 0xFF);
            ST7920_Write(LCD_DATA, 0xFF);
        }
    }
    */
    // initial menu selection
    /*lcd.highlight_menu_item(1, true);

    lcd.lcd_init(lcd_mode::text);
    lcd.set_start_address(0x80);
    lcd.put_text(" RGB Controller ", 16);
    lcd.set_start_address(0x90);
    lcd.put_text(" Red    99%", 11);
    lcd.set_start_address(0x88);
    lcd.put_text(" Green  99%", 11);
    lcd.set_start_address(0x98);
    lcd.put_text(" Blue   99%", 11);
    */
    //esp_restart();
}
