#include <stdio.h>
#include <thread>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "task.h"



static uint8_t ucParameterToPass;

inline void vToggleLED()
{
    if(gpio_get_level(BLINK_GPIO) == 1)
        gpio_set_level(BLINK_GPIO, 0);
    else
        gpio_set_level(BLINK_GPIO, 1);

}

void vTaskCode( void * pvParameters )
{
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    for( ;; )
    {
        // Task code goes here.
        vToggleLED();
        vTaskDelay( xDelay );
    }
}

void vAddTask( void )
{
    
    TaskHandle_t xHandle = NULL;

    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
    // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
    // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
    // the new task attempts to access it.
    xTaskCreate( vTaskCode, "NAME", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
    configASSERT( xHandle );

    // Use the handle to delete the task.
    if( xHandle != NULL )
    {
        //vTaskDelete( xHandle );
    }
}

void thread_example()
{
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
    
    for(int i=0; i<20 ; i++ )
    {
        // Task code goes here.
        //vToggleLED();
        if(gpio_get_level(BLINK_GPIO) == 1)
            gpio_set_level(BLINK_GPIO, 0);
        else
            gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay( xDelay );
    }
}

void start_thread()
{
    gpio_set_pull_mode(BLINK_GPIO, GPIO_FLOATING);
    
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    std::thread one(thread_example);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    one.join();

    printf("Thread is over \n");
}