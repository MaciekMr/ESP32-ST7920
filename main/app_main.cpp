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
#include "spi_interface.h"


extern "C" {
	void app_main(void);
}

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

    //ST7920_setGraphicMode(true);
    //lcd.lcd_init(lcd_mode::grap);
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
