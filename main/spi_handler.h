#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "st7920_commands_list.h"

#define ST7920  1

#define LCD_SPI_HOST VSPI_HOST
#define LCD_PIN_NUM_MISO GPIO_NUM_18
#define LCD_PIN_NUM_MOSI GPIO_NUM_23
#define LCD_PIN_NUM_CLK  GPIO_NUM_19
#define LCD_PIN_NUM_CS   GPIO_NUM_5
#define LCD_PIN_NUM_PSB  GPIO_NUM_17
#define LCD_PIN_NUM_RST  GPIO_NUM_16
#define LCD_TRANSFER_MAX_SIZE 32
#define SPI_CLK_FREQ (25*10*1000)   //When powered by 3.3V, LCD max freq is 1MHz, sometimes it is too fast 
//#define SPI_CLK_FREQ 500000
#define LCD_INPUT_DELAY_NS   ((150*1000*1000/SPI_CLK_FREQ)+20)

static const char TAG[] = "spi_if";

typedef uint8_t PIN_NUM;
typedef spi_host_device_t SPI_HOST_NO;
enum mode {command = 0, data = 1};

typedef struct {
    spi_host_device_t host; ///< The SPI host used, set before calling `spi_eeprom_init()`
    gpio_num_t cs_io;  
} lcd_ctx;

class spi_handler
{
private:
    PIN_NUM             spi_miso;
    PIN_NUM             spi_mosi;
    PIN_NUM             spi_clk;
    PIN_NUM             spi_cs;

    lcd_ctx             *lcd_cont;
    uint8_t             *buffer;
    spi_bus_config_t    spi_bus;
    spi_device_handle_t spi_device;
    //SPI specific settings
    spi_host_device_t   spi_host;
    //spi interface settings
    spi_device_interface_config_t spi_interface;
    void set_spi_pin_mode();
    void cs_high(spi_transaction_t* t);
    void cs_low(spi_transaction_t* t);
    void lcd_format_data(uint8_t mode, const uint8_t data);

public:
    ~spi_handler();
    spi_handler(PIN_NUM miso, PIN_NUM mosi, PIN_NUM clk, uint8_t max_transfer_size);
    esp_err_t init_spi_master(SPI_HOST_NO host);
    esp_err_t spi_device_init(uint_fast32_t spi_clock, uint_least16_t delay_ns);
    //send command or data via spi <size> of bytes from data
    esp_err_t send_spi(uint8_t mode, const uint8_t * data, uint8_t size = 3);
    //send command or data via spi <size> of bytes from data
    esp_err_t send_byte_spi(uint8_t mode, const uint8_t data);

    uint8_t u8g2_esp32_spi_byte_cb(u8x8_t* u8x8, uint8_t msg, uint8_t arg_int,  void* arg_ptr);

    uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t* u8x8,  uint8_t msg, uint8_t arg_int, void* arg_ptr);

};