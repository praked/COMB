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
#include "dance_model_static_trajectory.h"
#include "motor.h"
#include "gpio.h"
#include "math.h"
#include "commands.h"
#include "dancegen.h"
#include "types.h"
#include "motor.h"
#include "esp_log.h"
#include "esp_int_wdt.h"
#include "soc/rtc_wdt.h"



#define TIMER_DIVIDER   80             // should result in 1 MHz ^= 1µs timer interval

/////////////////////////////////////////////////////////////////////////////////////
//Global Parameters for the GUI UART0
#define GUI_TXD   1               // 1 and 3 are the respective pins for UART0
#define GUI_RXD   3
#define GUI_RTS (UART_PIN_NO_CHANGE)
#define GUI_CTS (UART_PIN_NO_CHANGE)

#define GUI_UART_PORT_NUM      0       // set for UART0
#define GUI_UART_BAUD_RATE     115200
#define GUI_TASK_STACK_SIZE    2048

/////////////////////////////////////////////////////////////////////////////////////
//Global Parameters for the Raspberry UART2
#define Raspi_TXD   26               //  26 and 25 are the respective pins for UART2
#define Raspi_RXD   25
#define Raspi_RTS (UART_PIN_NO_CHANGE)
#define Raspi_CTS (UART_PIN_NO_CHANGE)

#define Raspi_UART_PORT_NUM      2       // set for UART2
#define Raspi_UART_BAUD_RATE     115200
#define Raspi_TASK_STACK_SIZE    2048


//////////////////////////////////////////////////////////////////////////////////////
//Global Parameters for the Keypad UART1
#define KEYPAD_TXD   17               // 17 and 16 are the respective pins for UART1
#define KEYPAD_RXD   16
#define KEYPAD_RTS (UART_PIN_NO_CHANGE)
#define KEYPAD_CTS (UART_PIN_NO_CHANGE)

#define KEYPAD_UART_PORT_NUM      1   // set for UART1
#define KEYPAD_UART_BAUD_RATE     57600
#define KEYPAD_TASK_STACK_SIZE    2048

#define BUF_SIZE 1024

////////////////////////Keypad UART recv data
u8 keypad = 0;
u8 keypadSpeed = 0;

s16 motorSpeedFactor = 20;
s16 motorX_Key_Offset = 0;
s16 motorZ_Key_Offset = 0;
s16 motorPHI_Key_Offset = 0;

int sysState =0;
u8 state = 0;
volatile u8 rasp_recv = 0;
volatile u8 rasp_send = 0;
volatile u8 rasp_underprocess =0;
volatile u8 rasp_motor_id = 0;
volatile int rasp_motor_speed = 0;
volatile int rasp_motor_time = 0;
u8 limitSwitches[2][4] = {{GPIO_HOME_SWITCH_X_MIN, GPIO_HOME_SWITCH_X_MAX, GPIO_HOME_SWITCH_Z_MIN, GPIO_HOME_SWITCH_Z_MAX},{0,0,0,0}}; 

float diff;
float fTemp;
float w_old;
float w_cur;
float vx;
float vy;
float vx_old = 0;
float vy_old = 0;
V2Df p_old;
V2Df p_cur;
u16 i;

u16 wTemp =0;
u16 wPeriod = 3;  // in ms
u16 wTotalPeriod = 77; // in ms
u8 firstWaggleStart;
u8 testWings;
u8 wings_timer=0;


enum Mode {RotateNewWaggle, RotateContinueWaggle, FixedWaggle, RotateMotion};
u8 CurrMode = 0;
u8 ModeSwitch = 1;
u8 ResetEnc = 0;
volatile int stopCounter =0;


#define RAMP_ACCEL_FACTOR 0.03
#define MOTION_ACCEL_FACTOR 0.003
float rampFactor;
float motionFactorX;
float motionFactorZ;

volatile short  encoder_val            = 0;
volatile short  temp_GPIO_A         = 0;
volatile short  temp_GPIO_B         = 0;


volatile int    global_tick_count   = 0;
volatile uint   cnt[3]              = {0,0,0};
volatile const uint pins[6]         = {GPIO_MOTOR_PHI_STEP, GPIO_MOTOR_PHI_DIR, GPIO_MOTOR_X_STEP, GPIO_MOTOR_X_DIR, GPIO_MOTOR_Z_STEP, GPIO_MOTOR_Z_DIR};
volatile int    cnt_phi             = 0;
volatile int    cnt_x               = 0;
volatile int    cnt_z               = 0;
volatile short  motor_flags         = 0; //bit mask, which motor timer to start / stop 



#define TopSpeed 1000
#define ReduSpeed 20

#define beelength 0.22
volatile int current_angle = 0;
volatile int Rotation_speed_Phi = 1540;
volatile int Rotation_speed_X = 0;
volatile int Rotation_speed_Y = 0;
volatile int mflag = 0;
float Rotphi = 0;
volatile short  Rot_dir_phi         = 0;

int finaltime=0,inittime=0;

#define Kp  0.01  
#define output  0.00     

//global variable to be updated upon keyboard events
volatile int    period  = 0;
volatile int    addspeedX = 0;
volatile int    addspeedZ = 0;

char out_buf[100];

static const char* TAG = "MyModule";

int a[] = {0,0,0,0};
void motor_isr_toggle(int motor_id, int dir);
void motor_on_off(int motor_id, bool on);
void test_motor_phi();

volatile int dirToggle = 0;
int GP = 26;

volatile int dirToggle2 = 0;
int GP2 = 32;


typedef struct {
    int motorx;
    int motorz;
    int motorphi;
} motor_timers_t;

volatile int oneMilliSecondFlag = 0 ;

bool IRAM_ATTR global_tick_isr() // timer group 0, ISR
{   
    oneMilliSecondFlag = 1; 
    
    return 0; 
}

void motorXZPHIDrive(int x,int z,int phi)
{   
        uint timer_val;
        timer_val = (phi == 0) ? 1000000 : rotary_speed_to_timer_value( abs(phi) ) ; // set timer value for a long duration (1 s) if speed is 0
        set_motor_direction_pin(GPIO_MOTOR_PHI_DIR, phi > 0 ? 0:1)    
        timer_set_alarm_value(0, 1, timer_val);
            
        timer_val = (x == 0) ? 1000000 : linear_speed_to_timer_value_XY( abs(x ) ) ; 
        set_motor_direction_pin(GPIO_MOTOR_X_DIR, x  > 0 ? 1:0)    
        timer_set_alarm_value(1, 0, timer_val );

            
        timer_val = (z == 0) ? 1000000 : linear_speed_to_timer_value_Z( abs(z ) ) ; 
        set_motor_direction_pin(GPIO_MOTOR_Z_DIR, z > 0 ? 1:0)    
        timer_set_alarm_value(1, 1, timer_val );
}

void motor_isr_toggle(int motor_id, int dir)
{
    // select which timer group and timer idx
    int group = (motor_id == MOTOR_PHI)? 0 : 1;
    int timer = (motor_id == MOTOR_X)? 0 : 1;
    
    // if motor is on, switch it off
    set_motor_direction_pin(Motor_Direction[motor_id],dir);
    if (motor_flags & (1<<motor_id) )
        timer_pause(group, timer);
    else
    {
        timer_set_alarm_value(group, timer, (motor_id == MOTOR_X)?linear_speed_to_timer_value_XY(2500):linear_speed_to_timer_value_Z(2500));
        timer_start(group, timer);

    }// update motor flag
    motor_flags ^= (1<<motor_id);
}

bool IRAM_ATTR pin_toggle(void * param)
{ 
    uint motor_idx = (uint) param;
    gpio_set_level(pins[2*motor_idx], cnt[motor_idx] % 2);
    cnt[motor_idx]++;
    return false;
}


static void homing_switch_handler(void * param )
{   
    int change =0;
    int i=0;
    while(1)
    {
        
        for(i=0;i<4;i++)
        {
            limitSwitches[1][i] = !gpio_get_level(limitSwitches[0][i]);
        }
        //ESP_LOGI(TAG, "%d",encoder_val);
        vTaskDelay( 50 / portTICK_PERIOD_MS );
        for(i=0;i<4;i++)
        {
            limitSwitches[1][i] &= !gpio_get_level(limitSwitches[0][i]);
        }
        vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
    
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

void motor_toggle(int motor_id, int dir)
{
    // select which timer group and timer idx
    int group = (motor_id == MOTOR_PHI)? 0 : 1;
    int timer = (motor_id == MOTOR_X)? 0 : 1;
    
    // if motor is on, switch it off
    set_motor_direction_pin(Motor_Direction[motor_id],dir);
    if (motor_flags & (1<<motor_id) )
        timer_pause(group, timer);
    else
    {
        timer_set_alarm_value(group, timer, (motor_id == MOTOR_X)?linear_speed_to_timer_value_XY(628):linear_speed_to_timer_value_Z(628));
        timer_start(group, timer);

    }// update motor flag
    motor_flags ^= (1<<motor_id);
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

    // // homing switches
    init_GPIO(GPIO_HOME_SWITCH_X_MIN, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    init_GPIO(GPIO_HOME_SWITCH_X_MAX, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    init_GPIO(GPIO_HOME_SWITCH_Z_MIN, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    init_GPIO(GPIO_HOME_SWITCH_Z_MAX, GPIO_INTR_DISABLE, GPIO_MODE_INPUT);
    
    // motor pins   
    init_GPIO(GPIO_MOTOR_PHI_DIR,   GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_PHI_STEP,  GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_X_DIR,     GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_X_STEP,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_Z_DIR,     GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    init_GPIO(GPIO_MOTOR_Z_STEP,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);

    init_GPIO(GPIO_MOTOR_ENABLE,    GPIO_INTR_DISABLE, GPIO_MODE_OUTPUT);
    

    /////////// INSTALL ISRs
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    
    
}

void Init_AllTimers()
{
    init_timer(TIMER_GROUP_0, TIMER_1, 1000000, pin_toggle, MOTOR_PHI,  true);
    init_timer(TIMER_GROUP_1, TIMER_0, 1000000, pin_toggle, MOTOR_X,  true);
    init_timer(TIMER_GROUP_1, TIMER_1, 1000000, pin_toggle, MOTOR_Z,  true);
    init_timer(TIMER_GROUP_0, TIMER_0, 1000, global_tick_isr, NULL, true);
    
}

static void read_from_Keypad_UART_task(void *arg)
{
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = KEYPAD_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(KEYPAD_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(KEYPAD_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(KEYPAD_UART_PORT_NUM, KEYPAD_TXD, KEYPAD_RXD, KEYPAD_RTS, KEYPAD_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len = 0;
    int temp_val = encoder_val;
    int temp_val2 = CurrMode;
    data[0]= 0;
    data[1]= 0;
    data[2]= 0;
    uart_write_bytes((uart_port_t)KEYPAD_UART_PORT_NUM, data, 3);
    while (1) {
        if (temp_val != encoder_val || temp_val2 != CurrMode)
        {
            temp_val = encoder_val;
            temp_val2 = CurrMode;
            
            data[0]= temp_val & 0b11111111;
            data[1]= temp_val >>8;
            data[2]= CurrMode;
            uart_write_bytes((uart_port_t)KEYPAD_UART_PORT_NUM, data, 3);
            //SP_LOGI(TAG, "K %d E %d",(data[1]<<8) + data[0],encoder_val);

        }
        // // Read data from the UART
        data[0] = 0;
        data[1] = 0;
        len = uart_read_bytes(KEYPAD_UART_PORT_NUM, data, 2, 10 / portTICK_RATE_MS);
        
        // if data was received
        if ((len))
        {   
            
            //looks for any change in the keypad buffer and check if keypad is enabled for control
            if ((sysState & SYS_STATE_KEYBOARD_CTRL_ENABLED)&& data[0] != keypad )  
            {
                keypad= data[0];
                keypadSpeed = data[1];
            }   
        }
        
        len = 0;    
        
    }
}

static void read_from_Raspberry_UART_task(void *arg)
{
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = Raspi_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(Raspi_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(Raspi_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(Raspi_UART_PORT_NUM, Raspi_TXD, Raspi_RXD, Raspi_RTS, Raspi_CTS));
    int ResetEnc =0;
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len =0;
    data[0] = RASP_SEND_INIT_OK;
    uart_write_bytes((uart_port_t)Raspi_UART_PORT_NUM, data, 1);
    while(!(uart_read_bytes(Raspi_UART_PORT_NUM, data, 1, 20 / portTICK_RATE_MS)))
    {   
        vTaskDelay( 1000 / portTICK_RATE_MS);
        uart_write_bytes((uart_port_t)Raspi_UART_PORT_NUM, data, 1);
        ESP_LOGI(TAG, "Send ");
    }
    rasp_underprocess = 0;
    rasp_send = 0;
    while (1) {
        // Read data from the UART
        len =0;
        
        vTaskDelay( 40 / portTICK_RATE_MS);
        if(rasp_send & RASP_SEND_HOME_OK )
        {
            data[0] = RASP_SEND_HOME_OK;
            
            uart_write_bytes((uart_port_t)Raspi_UART_PORT_NUM, data, 1);
            ESP_LOGI(TAG, "Send HOME %d",data[0]);
            
            rasp_underprocess = 0;
            rasp_send = 0;
        }
        else if(rasp_send & RASP_SEND_MOVE_OK )
        {
            data[0] = RASP_SEND_MOVE_OK;
              
            vTaskDelay( 400 / portTICK_RATE_MS);
            uart_write_bytes((uart_port_t)Raspi_UART_PORT_NUM, data, 1);
            ESP_LOGI(TAG, "Send MOVE  %d",data[0]);
            
            rasp_underprocess = 0;
            rasp_send = 0;
        }
        else if(!rasp_underprocess)
        {
            len = uart_read_bytes(Raspi_UART_PORT_NUM, data, 6, 20 / portTICK_RATE_MS);
            if(len)
            {
            //rasp_recv = RASP_RECV_HOME;    
            rasp_recv = data[0];
            rasp_motor_id = data[1]&0x0f;
            rasp_motor_speed = (data[1]>>4)?(-1*((data[3]<<8) + data[2])):((data[3]<<8) + data[2]);
            rasp_motor_time = (data[5]<<8) + data[4];
            if (rasp_recv == RASP_RECV_HOME || rasp_recv == RASP_RECV_MOVE)
                rasp_underprocess = 1;
            ESP_LOGI(TAG, "Recv %d %d %d %d ",rasp_recv, rasp_motor_id, rasp_motor_speed ,rasp_motor_time);
            }
        }
        //ESP_LOGI(TAG, "Recv %d %d ",rasp_recv,rasp_underprocess );
    }
}

static void read_from_GUI_UART_task(void *arg)
{
    
    /* Configure parameters of an UART driver,
    * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate  = GUI_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(GUI_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(GUI_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GUI_UART_PORT_NUM, GUI_TXD, GUI_RXD, GUI_RTS, GUI_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    int len =0;
    while (1) {
        // Read data from the UART
        vTaskDelay( 40 / portTICK_RATE_MS);
        len = uart_read_bytes(GUI_UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // if data was received
        //ESP_LOGI(TAG, "GUI RECV %d ",encoder_val);
        if (len)
        {
            switch (data[0])
            {
            case '1':
                ESP_LOGI(TAG, "UART1 ");
                rasp_recv |= RASP_RECV_HOME;
                break;
            case '3':
                ESP_LOGI(TAG, "UART3 ");
                rasp_recv |= RASP_RECV_MOVE;
                rasp_motor_id = MOTOR_X;
                rasp_motor_speed = 2000;
                rasp_motor_time = 1000;
                break;

            case '4':
                ESP_LOGI(TAG, "UART4 ");
                rasp_recv |= RASP_RECV_MOVE;
                rasp_motor_id = MOTOR_Y;
                rasp_motor_speed = 2000;
                rasp_motor_time = 1000;
                break;
            case '5':
                ESP_LOGI(TAG, "UART5 ");
                rasp_recv |= RASP_RECV_MOVE;
                rasp_motor_id = MOTOR_X;
                rasp_motor_speed = -2000;
                rasp_motor_time = 1000;
                break;

            case '6':
                ESP_LOGI(TAG, "UART6 ");
                rasp_recv |= RASP_RECV_MOVE;
                rasp_motor_id = MOTOR_Y;
                rasp_motor_speed = -2000;
                rasp_motor_time = 1000;
                break;        
            default: 
                break;
            
            }
            
        }
    }
}

void app_main()
{	  
    printf("Hiveopolis RoboBee - Minimal Prototype\n");
    ESP_LOGI(TAG, "try ");
    Init_GPIO_ISR();
    sysState |= SYS_STATE_KEYBOARD_CTRL_ENABLED;
    ESP_LOGI(TAG, "try2 ");
    printf("Hiveopolis RoboBee - Checking\n");
    
    printf("Hiveopolis RoboBee - All Motor Timers and Global Loop Timer Initialised\n");
    ESP_LOGI(TAG, "sysState %d",sysState);
    xTaskCreate(read_from_Keypad_UART_task, "read_from_Keypad_UART_task", KEYPAD_TASK_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(read_from_GUI_UART_task, "read_from_GUI_UART_task", GUI_TASK_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(homing_switch_handler, "homing_switch_handler", GUI_TASK_STACK_SIZE, NULL, 9, NULL);
    xTaskCreate(read_from_Raspberry_UART_task, "read_from_Raspi_UART_task", Raspi_TASK_STACK_SIZE, NULL, 9, NULL);
    ESP_LOGI(TAG, "sysState %d",sysState);
    rampFactor = 1;
    w_old = 0;
    firstWaggleStart = 1;
    V3Df temp;
    motor_timers_t mtt;
    mtt.motorx = 0;
    mtt.motorz = 0;
    mtt.motorphi = 0;
    
    Init_AllTimers();
    stopCounter =0;
    u8 DanceInitFlag = 0;
    global_tick_count =0;

    
    while (1)
    {   
        mtt.motorx = 0;
        mtt.motorz = 0;
        mtt.motorphi = 0;

        temp.x = 0.0;
        temp.z = 0.0;
        temp.phi = 0.0;
        
        if(sysState & SYS_STATE_KEYBOARD_CTRL_ENABLED)
        {
            //ESP_LOGI(TAG, "key press: %d  %d   %d  %d", limitSwitches[2][0],limitSwitches[2][1],limitSwitches[2][2],limitSwitches[2][3]);
            if((keypad & KEY_LEFT) && !(keypad & KEY_RIGHT))
            {    
                //ESP_LOGI(TAG, "KEY_LEFT ");
                if(limitSwitches[1][0])
                    motionFactorX = 0;
                else 
                {
                    if ( motionFactorX < 1 )
                    {
                        motionFactorX += MOTION_ACCEL_FACTOR;
                    }
                    else     
                        motionFactorX = 1;
                }
                motorX_Key_Offset = -keypadSpeed*motionFactorX;

            }
            else if(!(keypad & KEY_LEFT) && (keypad & KEY_RIGHT))
            {    
                //ESP_LOGI(TAG, "KEY_RIGHT");
                if(limitSwitches[1][1])
                    motionFactorX = 0;
                else 
                {
                    if ( motionFactorX < 1 )
                    {
                        motionFactorX += MOTION_ACCEL_FACTOR;
                        
                    }
                    else     
                        motionFactorX = 1;
                }
                motorX_Key_Offset = keypadSpeed*motionFactorX;
            }
            else
            {
                motorX_Key_Offset = 0;
                motionFactorX = 0;
            }
            if((keypad & KEY_UP) && !(keypad & KEY_DOWN))
            {    
                //ESP_LOGI(TAG, "KEY_UP");
                if(limitSwitches[1][3])
                    motionFactorZ = 0;
                else 
                {
                    if ( motionFactorZ < 1 )
                    {
                        motionFactorZ += MOTION_ACCEL_FACTOR;
                        
                    }
                    else     
                        motionFactorZ = 1;
                }
                motorZ_Key_Offset = keypadSpeed*motionFactorZ;

            }
            else if(!(keypad & KEY_UP) && (keypad & KEY_DOWN))
            {    
                //ESP_LOGI(TAG, "KEY_DOWN");
                if(limitSwitches[1][2])
                    motionFactorZ = 0;
                else 
                {
                    if ( motionFactorZ < 1 )
                    {
                        motionFactorZ += MOTION_ACCEL_FACTOR;
                        //ESP_LOGI(TAG, "motionFactorZ - %f",motionFactorZ );
                    }
                    else     
                        motionFactorZ = 1;
                    
                }
                motorZ_Key_Offset = -keypadSpeed*motionFactorZ;
            }  
            else 
            {   
                motionFactorZ = 0;
                motorZ_Key_Offset = 0;    
            }

            // Z motor CW/CCW (only one works)
            if ((keypad & KEY_ZCW) && !(keypad & KEY_ZCCW))
                motorPHI_Key_Offset = keypadSpeed;
            else if (!(keypad & KEY_ZCW) && (keypad& KEY_ZCCW))
                motorPHI_Key_Offset = -keypadSpeed;
            else
                motorPHI_Key_Offset = 0;

            state |= STATE_NEW_KEY_EVENT;    
        }
        else
        {
            keypad = 0xFF;
            state &= ~STATE_NEW_KEY_EVENT;
        }   
        
        //Check if robot has reached its side limits i.e. Limit switches
        if(!(limitSwitches[1][0]|limitSwitches[1][1]|limitSwitches[1][2]|limitSwitches[1][3]))
            sysState &= ~SYS_STATE_DOWN_ROBOT;
        else
            sysState |= SYS_STATE_DOWN_ROBOT;  

        if(oneMilliSecondFlag)
        {   
            
            
            oneMilliSecondFlag = 0;
            //ESP_LOGI(TAG, "rasp_recv %d ",rasp_recv);
            if(rasp_recv == RASP_RECV_HOME)
            {
                if(limitSwitches[1][1] && limitSwitches[1][3])
                {
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 1);
                    motorXZPHIDrive( 0, 0, 0); 
                    timer_pause(TIMER_GROUP_0, TIMER_1);
                    timer_pause(TIMER_GROUP_1, TIMER_0);
                    timer_pause(TIMER_GROUP_1, TIMER_1);
                    rasp_recv &= ~RASP_RECV_HOME;
                    ESP_LOGI(TAG, "HOMED");
                    rasp_send |= RASP_SEND_HOME_OK ;
                }
                else if(limitSwitches[1][1])
                {
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                    timer_start(TIMER_GROUP_0, TIMER_1);
                    timer_start(TIMER_GROUP_1, TIMER_0);
                    timer_start(TIMER_GROUP_1, TIMER_1);
                    motorXZPHIDrive( 0, 2000, 0); 
                }
                else if(limitSwitches[1][3])
                {
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                    timer_start(TIMER_GROUP_0, TIMER_1);
                    timer_start(TIMER_GROUP_1, TIMER_0);
                    timer_start(TIMER_GROUP_1, TIMER_1);
                    motorXZPHIDrive( 2000, 0, 0); 
                }
                else
                {
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                    timer_start(TIMER_GROUP_0, TIMER_1);
                    timer_start(TIMER_GROUP_1, TIMER_0);
                    timer_start(TIMER_GROUP_1, TIMER_1);
                    motorXZPHIDrive( 2000, 2000, 0); 
                    ESP_LOGI(TAG, "Home move both");
                }

            }
            else if(rasp_recv == RASP_RECV_MOVE)
            {
                if(rasp_motor_time == 0 )
                {
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 1);
                    motorXZPHIDrive( 0, 0, 0); 
                    timer_pause(TIMER_GROUP_0, TIMER_1);
                    timer_pause(TIMER_GROUP_1, TIMER_0);
                    timer_pause(TIMER_GROUP_1, TIMER_1);
                    rasp_recv &= ~RASP_RECV_MOVE;
                    ESP_LOGI(TAG, "MOVED");  
                    rasp_send |= RASP_SEND_MOVE_OK ; 
                }
                else
                { 
                    rasp_motor_time--;
                    set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                    timer_start(TIMER_GROUP_0, TIMER_1);
                    timer_start(TIMER_GROUP_1, TIMER_0);
                    timer_start(TIMER_GROUP_1, TIMER_1);
                    if(rasp_motor_id == MOTOR_X)
                    {
                        if(rasp_motor_speed > 0)
                        {
                            if(limitSwitches[1][1])
                                rasp_motor_time = 0;
                            else    
                                motorXZPHIDrive( rasp_motor_speed, 0, 0); 
                        }
                        else 
                        {
                            if(limitSwitches[1][0])
                                rasp_motor_time = 0;
                            else    
                                motorXZPHIDrive( rasp_motor_speed, 0, 0); 
                        }
                    }
                    else if(rasp_motor_id == MOTOR_Y)
                    {
                        if(rasp_motor_speed > 0)
                        {
                            if(limitSwitches[1][3])
                                rasp_motor_time = 0;
                            else    
                                motorXZPHIDrive(  0,rasp_motor_speed, 0); 
                        }
                        else 
                        {
                            if(limitSwitches[1][2])
                                rasp_motor_time = 0;
                            else    
                                motorXZPHIDrive(  0,rasp_motor_speed, 0); 
                        }
                    }
                }
            }
            else if ( (state & STATE_NEW_KEY_EVENT)) 
            {  
                state &= ~STATE_NEW_KEY_EVENT;
                
                if ( (keypad & KEY_MODE) )
                {
                    if (keypad==128)
                    {
                        // Motoren ausschalten
                       
                        
                        //timer_pause(TIMER_GROUP_0, TIMER_0);           // Dancetimer stop
                        motorXZPHIDrive(0,0,0);   // Motoren stop
                        timer_pause(TIMER_GROUP_0, TIMER_1);
                        timer_pause(TIMER_GROUP_1, TIMER_0);
                        timer_pause(TIMER_GROUP_1, TIMER_1);
                        set_motor_direction_pin(GPIO_MOTOR_ENABLE, 1);
                        set_motor_direction_pin(GPIO_MOTOR_PHI_STEP,  0);
                        set_motor_direction_pin(GPIO_MOTOR_X_STEP,    0);
                        set_motor_direction_pin(GPIO_MOTOR_Z_STEP,    0); 
                        //ESP_LOGI(TAG, "Robot HALT %d ",encoder_val);
                    }
                    else if(keypad!=128)
                    {
                        // Motoren start
                        stopCounter = 0;
                        state |= STATE_KEY_MOTORS_ON;
                        set_motor_direction_pin(GPIO_MOTOR_ENABLE, 0);
                        
                        

                        timer_start(TIMER_GROUP_0, TIMER_1);
                        timer_start(TIMER_GROUP_1, TIMER_0);
                        timer_start(TIMER_GROUP_1, TIMER_1);
                        

                        if(!(keypad & KEY_D_ONOFF))
                        {
                            
                            //ESP_LOGI(TAG, "f4 %f",diff );
                            motorXZPHIDrive( motorX_Key_Offset*motorSpeedFactor, motorZ_Key_Offset*motorSpeedFactor, 0);
                        
                        }       
                    }
                } 
                
            }
            else if((sysState&SYS_STATE_DOWN_ROBOT) )
            {   
                stopCounter =1;
                timer_pause(TIMER_GROUP_0, TIMER_1);
                timer_pause(TIMER_GROUP_1, TIMER_0);
                timer_pause(TIMER_GROUP_1, TIMER_1); 
                set_motor_direction_pin(GPIO_MOTOR_PHI_STEP,  0);
                set_motor_direction_pin(GPIO_MOTOR_X_STEP,    0);
                set_motor_direction_pin(GPIO_MOTOR_Z_STEP,   0);  
                ESP_LOGI(TAG, "Robot LIMIT SW  ");
                      
            }
        
        }
    }

}