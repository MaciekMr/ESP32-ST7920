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
#include <esp_http_server.h>
#include "esp_netif.h"
#include "sdkconfig.h"
#include "u8g2.h"
#include "spi_handler.h"

/*

            HSPI	VSPI
CLK	        GPIO14	GPIO18
MISO	    GPIO12	GPIO19
DAWDLE/MOSI	GPIO13	GPIO23
SS	        GPIO15	GPIO5
*/

static void cs_high_callback(spi_transaction_t* t)
{
    
    ESP_LOGD(TAG,"cs high %d.", ((lcd_ctx *)t->user)->cs_io);
    gpio_set_level(((lcd_ctx*)t->user)->cs_io, 1);
    //vTaskDelay(1 / portTICK_PERIOD_MS);
}

static void cs_low_callback(spi_transaction_t* t)
{
    //vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(((lcd_ctx*)t->user)->cs_io, 0);
    ESP_LOGD(TAG,"cs low %d.", ((lcd_ctx *)t->user)->cs_io);
    
}


spi_handler::spi_handler(PIN_NUM miso, PIN_NUM mosi, PIN_NUM clk, uint8_t max_transfer_size)
    :spi_miso(miso), spi_mosi(mosi), spi_clk(clk)
{
    memset(&spi_device, 0, sizeof(spi_device_handle_t));
    memset(&spi_bus, 0, sizeof(spi_bus_config_t));
    memset(&spi_host, 0, sizeof(spi_host_device_t));
    memset(&spi_interface, 0, sizeof(spi_device_interface_config_t));
    set_spi_pin_mode();
    lcd_cont = new lcd_ctx;
    spi_bus.miso_io_num = miso;
    spi_bus.mosi_io_num = mosi;
    spi_bus.sclk_io_num = clk;
    spi_bus.quadwp_io_num = -1;
    spi_bus.quadhd_io_num = -1;
    spi_bus.max_transfer_sz = max_transfer_size;
    #ifdef ST7920
    buffer = new uint8_t[3];
    #endif
    ESP_LOGD(TAG, "Create spi_handler. \n");
}

void spi_handler::set_spi_pin_mode()
{
    gpio_set_pull_mode(LCD_PIN_NUM_MOSI, GPIO_FLOATING);
    gpio_set_pull_mode(LCD_PIN_NUM_CLK, GPIO_FLOATING);
    gpio_set_pull_mode(LCD_PIN_NUM_CS, GPIO_FLOATING);
    gpio_set_pull_mode(LCD_PIN_NUM_PSB, GPIO_FLOATING);
    gpio_set_pull_mode(LCD_PIN_NUM_RST, GPIO_FLOATING);
    gpio_set_direction(LCD_PIN_NUM_MOSI, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_NUM_CLK, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_NUM_PSB, GPIO_MODE_OUTPUT);
    gpio_set_direction(LCD_PIN_NUM_RST, GPIO_MODE_OUTPUT);
}

spi_handler::~spi_handler()
{
    delete (lcd_cont);
}

esp_err_t spi_handler::init_spi_master(SPI_HOST_NO host)
{
    spi_host = host;
    ESP_LOGD(TAG,"Init spi master %d", spi_host);
    esp_err_t ret = spi_bus_initialize(spi_host, &spi_bus, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    return (ret);
}

esp_err_t spi_handler::spi_device_init(uint_fast32_t spi_clock, uint_least16_t delay_ns)
{
    
    spi_interface.command_bits = 10;
    spi_interface.clock_speed_hz = spi_clock;
    spi_interface.mode = 0; //SPI mode 0 - https://en.m.wikipedia.org/wiki/Serial_Peripheral_Interface#Clock_polarity_and_phase
    
    /*
    * The timing requirements to read the busy signal from the LCD cannot be easily emulated
    * by SPI transactions. We need to control CS pin by SW to check the busy signal manually.
    */
    spi_interface.spics_io_num = -1;  //hardware CS pin is not used we are calling callbacks to set CS
    spi_interface.queue_size = 3;
    spi_interface.flags = SPI_DEVICE_HALFDUPLEX; // | SPI_DEVICE_POSITIVE_CS;
    spi_interface.pre_cb = cs_high_callback;
    spi_interface.post_cb = cs_low_callback;
    spi_interface.input_delay_ns = LCD_INPUT_DELAY_NS; //delay_ns; //LCD_INPUT_DELAY_NS;

    esp_err_t ret = spi_bus_add_device(spi_host, &spi_interface, &spi_device);
    ESP_ERROR_CHECK(ret);
    ESP_LOGD(TAG,"device added to bus %d \n", ret);
    return (ret);
}

/*
* We can send multiple bytest through the spi
* After each send, delay has to be applied ~1ms
*
*/
esp_err_t spi_handler::send_spi(uint8_t mode, const uint8_t *data, uint8_t size)
{
    esp_err_t ret = ESP_OK;
    int count = size;
    while(--count > 0)
    {
        ret = send_byte_spi(mode, *data++);
        assert(ret==ESP_OK);
        //delay 1ms
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    return (ret);
}

esp_err_t spi_handler::send_byte_spi(uint8_t mode, const uint8_t data)
{
    ESP_LOGD(TAG,"received data %d \n", data);
    lcd_format_data(mode, data);
    ESP_EARLY_LOGV(TAG, "sending data  %d %d %d. \n", buffer[2], buffer[1], buffer[0]);
    ESP_LOGD(TAG,"sending data  %d %d %d. in mode %d \n", buffer[2], buffer[1], buffer[0], mode);
    esp_err_t ret;
    spi_transaction_t t;
    
    lcd_cont->host = spi_host;
    lcd_cont->cs_io = LCD_PIN_NUM_CS;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 3*8;                   //Len is in bytes, transaction length is in bits.
    t.tx_buffer = (const void *) buffer;           //Data
    t.user = (void*)lcd_cont;                //pass the parameter to callback
    ret = spi_device_polling_transmit(spi_device, &t);  //Transmit!
    assert(ret==ESP_OK);
    ESP_LOGD(TAG,"data sent %d \n", ret);
    return (ret);
}

/*
* mode 0 -> command  1 -> data
*/
void spi_handler::lcd_format_data(uint8_t mode, const uint8_t data)
{
    //For ST7920 we hae 3 bytes of data to send
    //First byte has:
    // 5 bits set to 1 as sync string
    //2 bits for RW and RS
    //last bit 0 as control
    //Second byte consist 4 bits of data D7-D4
    //and 4 bits 0 as control
    //Third byte consist data 
    //4 bits D3-D0
    //and 4 control bits 0
#ifdef ST7920
    memset((void *)buffer, 0, 3*sizeof(uint8_t));

    buffer[0] = (1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (0 << 0);
    //mode instruction RS = 0 RW = 0 . data RS = 1 RW = 0 -> instruction mode = 0 , data mode = 1
    if (mode == mode::data)
        buffer[0] |= (0 << 2) | (1 << 1);
    else
        buffer[0] |= (0 << 2) | (0 << 1);

    buffer[1] = ((data >> 4) << 4);

    buffer[2] = ((data) << 4);
#endif
}

/*Callback method for u8g2 library
* passed to functions as callback
*/
uint8_t spi_handler::u8g2_esp32_spi_byte_cb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int,  void* arg_ptr)
{
    ESP_LOGD(TAG, "spi_byte_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);
    switch (msg) {
        case U8X8_MSG_BYTE_SET_DC:
        
        break;

        case U8X8_MSG_BYTE_INIT: 
        
        //SPI is set on the class level

        break;

        case U8X8_MSG_BYTE_SEND: 
        //SPI sent byte by class
        //send_byte_spi();

        // ESP_LOGI(TAG, "... Transmitting %d bytes.", arg_int);
        
        break;
        
    }
    return 0;
}  // u8g2_esp32_spi_byte_cb


uint8_t spi_handler::u8g2_esp32_gpio_and_delay_cb(u8x8_t* u8x8,  uint8_t msg, uint8_t arg_int, void* arg_ptr)
{
    ESP_LOGD(TAG, "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg, arg_int, arg_ptr);

    switch (msg) {
      // Initialize the GPIO and DELAY HAL functions.  If the pins for DC and
      // RESET have been specified then we define those pins as GPIO outputs.
        case U8X8_MSG_GPIO_AND_DELAY_INIT: {
        
        //Set the pin mode and direction
        break;
        }

        // Set the GPIO reset pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_RESET:
        
        break;
        // Set the GPIO client select pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_CS:
            //CS pin is controlled by SPI
        break;
        // Set the Software I²C pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_I2C_CLOCK:
        
            //				printf("%c",(arg_int==1?'C':'c'));
        
        break;
        // Set the Software I²C pin to the value passed in through arg_int.
        case U8X8_MSG_GPIO_I2C_DATA:
        
        break;

        // Delay for the number of milliseconds passed in through arg_int.
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
        break;
    }
    return 0;

}