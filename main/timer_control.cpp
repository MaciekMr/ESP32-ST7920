
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
#include "freertos/queue.h"
#include "timer_control.h"

static const char TAG[] = "Timer";

static xQueueHandle s_timer_queue_local;

void set_queue(xQueueHandle queue)
{
    s_timer_queue_local = queue;
    printf("queue is set! \n");
}

static int32_t counter_timer;

static bool IRAM_ATTR timer_callback(void *args)
{
    //printf("callback is calling");
    //ESP_LOGI(TAG, "Callback is calling \n");
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);
    //ESP_LOGI(TAG, "Callback is calling timer=%llu \n", timer_counter_value);

    example_timer_event_t evt;
    evt.info.timer_group = info->timer_group;
    evt.info.timer_group = info->timer_group,
    evt.info.timer_idx = info->timer_idx,
    evt.info.auto_reload = info->auto_reload,
    evt.info.alarm_interval = info->alarm_interval,
    evt.timer_counter_value = timer_counter_value;
    
    counter_timer++;
    if(counter_timer > 23456)
        counter_timer = 0;

    //xQueueGenericSendFromISR
    xQueueSendFromISR(s_timer_queue_local, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE;
}

void show_counter()
{
    printf("test %d", counter_timer);
    ESP_LOGI(TAG, "Callback is calling timer=%d \n", counter_timer);
}

void set_timer()
{
    esp_err_t error = ESP_OK;
    counter_timer = 0;
    timer_config_t timer1 =
    {
        .alarm_en = timer_alarm_t::TIMER_ALARM_EN,
        .counter_en = timer_start_t::TIMER_PAUSE,
        .intr_type = timer_intr_mode_t::TIMER_INTR_LEVEL,
        .counter_dir = timer_count_dir_t::TIMER_COUNT_UP,
        .auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN,
        .clk_src = timer_src_clk_t::TIMER_SRC_CLK_APB,
        .divider = TIMER_COUNTER
    };

    error = timer_init(TIMER_GROUP_0, TIMER_0, &timer1);

    error = timer_set_counter_value(TIMER_GROUP_0, TIMER_0, /*uint64_t*/ 0); //we need 100kHz - so the value ia 10

    //error = timer_set_divider(TIMER_GROUP_0, TIMER_0, 1000);
    /* Configure the alarm value and the interrupt on alarm. */
    error = timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, /*timer_interval_sec*/ 3 * TIMER_SCALE);
    
    error = timer_enable_intr(TIMER_GROUP_0, TIMER_0);

    example_timer_info_t *timer_info = reinterpret_cast<example_timer_info_t*>( calloc(1, sizeof(example_timer_info_t)));
    timer_info->timer_group = TIMER_GROUP_0;
    timer_info->timer_idx = TIMER_0;
    timer_info->auto_reload = timer_autoreload_t::TIMER_AUTORELOAD_EN;
    timer_info->alarm_interval = 3*TIMER_SCALE;

    error = timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_callback, timer_info, 0);

    if(error != ESP_OK)
        printf("Error %d", error);
    else
        printf("Start timer");

    error = timer_start(TIMER_GROUP_0, TIMER_0);

}