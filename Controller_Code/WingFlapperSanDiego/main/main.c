/**
 * Author:    Pranav Kedia
 * Created:   15.01.2024
 * 
 * WingFlapperSanDiego © 2023 by Pranav Kedia is licensed under CC BY 4.0 
 **/
#include <stdio.h>
#include <stdlib.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "motor.h"
#include "gpio.h"
#include "math.h"
#include "commands.h"
#include "types.h"
#include "motor.h"
#include "esp_log.h"
#include "esp_int_wdt.h"
#include "soc/rtc_wdt.h"



#define TIMER_DIVIDER   80             // should result in 1 MHz ^= 1µs timer interval


//Switch is expected to be in NO(Normally opened configuration)

volatile int stopCounter =0;

volatile int    global_tick_count   = 0;

volatile int    direction            = 0;
 
volatile short  motor_flags         = 0; //bit mask, which motor timer to start / stop 

char out_buf[100];

static const char* TAG = "MyModule";

volatile int updateFlag = 0 ;

bool IRAM_ATTR global_tick_isr() // timer group 0, ISR
{   
    updateFlag = 1; 
    
    return 0; 
}



static void init_timer(timer_group_t timer_group, timer_idx_t timer_idx, uint64_t alarm_val, timer_isr_t isr_callback, void * param, bool start_timer)
{
    timer_config_t config;
    config.alarm_en         = 1; // timer alarm enable
    config.auto_reload      = 1; // auto-reload enable
    config.counter_dir      = TIMER_COUNT_UP; 
    config.divider          = TIMER_DIVIDER;
    config.intr_type        = TIMER_INTR_LEVEL; // timer config: interrupt mode = timer triggers isr
    config.counter_en       = TIMER_PAUSE; // do not start timer right away

    // configure timer
    timer_init(timer_group, timer_idx, &config);
    // stop timer counter (should not be necessary because we initialized it to be paused)
    timer_pause(timer_group, timer_idx);
    // load counter value
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    // set alarm value (in µs)
    timer_set_alarm_value(timer_group, timer_idx, alarm_val);
    // enable timer interrupt
    timer_enable_intr(timer_group, timer_idx);
    // set interrupt service routine
    // timer_isr_register(timer_group, timer_idx, timer_G0T0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    //if (isr_function) 
    //    timer_isr_register(timer_group, timer_idx, isr_function, NULL, ESP_INTR_FLAG_IRAM, NULL);
    //else        
    timer_isr_callback_add(timer_group, timer_idx, isr_callback, param, 0);
    
    if (start_timer)
        timer_start(timer_group, timer_idx);
}

void pin_toogle(int gpio_pin, int dir)
{
    // select which timer group and timer idx
    
}


void init_GPIO(gpio_num_t motor_gpio, gpio_int_type_t interrupt_type, gpio_mode_t mode)
{
    gpio_config_t io_conf;
    io_conf.intr_type       = interrupt_type;
    io_conf.pin_bit_mask    = ((uint64_t)(((uint64_t)1)<<motor_gpio));
    io_conf.mode            = mode;
    io_conf.pull_down_en    = 0;
    io_conf.pull_up_en      = 0;
    gpio_config(&io_conf);
}

void Init_GPIO_ISR()
{
    //////// PIN CONFIGURATION
    
    
    // // wing switches
    init_GPIO(GPIO_WING_IN1, GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_WING_IN2, GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);

    

    init_GPIO(GPIO_MOTOR_ENABLE,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    

    /////////// INSTALL ISRs
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    
    
}

void Init_AllTimers(uint64_t alarm_val)
{
    
    init_timer(TIMER_GROUP_0, TIMER_0, alarm_val, global_tick_isr, NULL, true);
    
}

void app_main()
{	  
    printf("Flapper - Minimal Prototype\n");
    ESP_LOGI(TAG, "try ");
    Init_GPIO_ISR();
    ESP_LOGI(TAG, "try2 ");
    printf("Flaper - Checking\n");
    
    printf("Flapper - All Motor Timers and Global Loop Timer Initialised\n");
    uint64_t Frequency_divider =  20000 ;    //  1000 value updates the update flag every 1ms, change this get required frequency
    // Currently set to 20000 which leads to 20ms update time, hence a frequency of 50 Hertz
    // https://docs.espressif.com/projects/esp-idf/en/v4.3/esp32/api-reference/peripherals/timer.html
    Init_AllTimers(Frequency_divider);
    stopCounter =0;
    u8 DanceInitFlag = 0;
    global_tick_count =0;
    gpio_set_level(GPIO_MOTOR_ENABLE,1);
    
    while (1)
    {   

        if(updateFlag)
        {   
            
            gpio_set_level(GPIO_WING_IN1, !direction);
            gpio_set_level(GPIO_WING_IN2, !direction);
            direction = !direction;
        
        }
    }

}