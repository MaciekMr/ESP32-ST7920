#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"

#include <stddef.h>
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "driver/timer.h"

#define TIMER_COUNTER         16
#define TIMER_SCALE           (TIMER_BASE_CLK  / TIMER_COUNTER)

void set_queue(xQueueHandle queue);

typedef struct {
    timer_group_t timer_group;
    timer_idx_t timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}
/*static*/ void set_timer();
void show_counter();

// /*static*/ bool IRAM_ATTR timer_callback(void *args);

class timer
{

private:
    timer_config_t configuration;

public:
};



//static intr_handle_t s_timer_handle;


//*****************************************
//*****************************************
//********** TIMER TG0 INTERRUPT **********
//*****************************************
//*****************************************
/*
static void timer_tg0_isr(void* arg)
{
	static int io_state = 0;

	//Reset irq and set for next time
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;


    //----- HERE EVERY #uS -----

	//Toggle a pin so we can verify the timer is working using an oscilloscope
	io_state ^= 1;									//Toggle the pins state
	gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_5, io_state);



}
*/

//******************************************
//******************************************
//********** TIMER TG0 INITIALISE **********
//******************************************
//******************************************
/*
void timer_tg0_initialise (int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,				//Alarm Enable
            .counter_en = false,			//If the counter is enabled it will start incrementing / decrementing immediately after calling timer_init()
            .intr_type = TIMER_INTR_LEVEL,	//Is interrupt is triggered on timer’s alarm (timer_intr_mode_t)
            .counter_dir = TIMER_COUNT_UP,	//Does counter increment or decrement (timer_count_dir_t)
            .auto_reload = true,			//If counter should auto_reload a specific initial value on the timer’s alarm, or continue incrementing or decrementing.
            .divider = 80   				//Divisor of the incoming 80 MHz (12.5nS) APB_CLK clock. E.g. 80 = 1uS per timer tick
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_tg0_isr, NULL, 0, &s_timer_handle);

    timer_start(TIMER_GROUP_0, TIMER_0);
}*/
