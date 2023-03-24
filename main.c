#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include "esp_adc_cal.h"
#include "esp_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "math.h"
#include "dft.h"
#include "ets_sys.h"
#include "esp_idf_lib_helpers.h"
#include "adc.h"
#include "ssd1306_comp.h"
#include "font8x8_basic.h"

#define DEFAULT_VREF    1100  

//Define buffer status and size
#define BUF_FREE 0
#define BUF_FULL 1
#define BUF_SAMPLE 2
#define BFR_SIZE 64

#define CAL_STOP 0
#define CAL_GO 1

#define SOUND_OFF 0
#define SOUND_ON 1
#define SOUND_THRESHOLD 15

//Define metal types
#define FERROUS 0
#define NONFERROUS 1
#define NA 2

//Divide timer0 clock to obtain sample frequency of 8 kHz
#define TIM0_DIV  (10000)

//Define timer1 clock divider for freq of 2kHz
#define TIM1_DIV  (40000)

//Define PWM output pin
#define PWM_GPIO 26 //changed from 16 to 26 (PCB layout)

//Define Calibration Button input pin
#define CAL_GPIO 17

#define SOUND_BTN_GPIO 16 
#define SOUND_OUT_GPIO 27 //changed from 27 to 16 (PCB layout)

#define CAL_CYCLES 50

#define tag "SSD1306"

static SSD1306_t dev;
 
//ADC configuration statics
static esp_adc_cal_characteristics_t *adc_chars1;
static const adc_channel_t channel1 = ADC_CHANNEL_6;    
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_unit_t unit1 = ADC_UNIT_1;

//Attenuating the ADC input to measure from 0.1V to 3.1V
static const adc_atten_t atten1 = ADC_ATTEN_DB_11;

//ISR counters and buffer
volatile int idx = 0, int_count = 0, cycle_count = 0;
volatile char BUF_STAT = BUF_FREE;
volatile short x_sample[BFR_SIZE] = {0}; 

volatile char CAL_STAT = CAL_STOP;

volatile char SOUND_STAT = SOUND_OFF;
volatile char Sound_Btn_Reset = 0;

volatile char METAL_ID = NA;

static TaskHandle_t xNotifyDft = NULL;
static TaskHandle_t xNotifyCal = NULL;
static TaskHandle_t xNotifyDisp = NULL;
static TaskHandle_t xNotifySoundBtn = NULL;
static TaskHandle_t xNotifySound = NULL;
static TaskHandle_t xStartupDisplay = NULL;
static TaskHandle_t xStartupSound = NULL;
static TaskHandle_t xSoundDisplay = NULL;

static double ramp = 0, rphase = 0;
static double cal_amp = 0, cal_phase = 0;
static double filt_amp = 0, filt_phase = 0;
static double disp_amp = 0, disp_phase = 0;

static char sound_display = 0;
static char cal_display = 0;
static int sound_counter = 0;

static void IRAM_ATTR cal_interrupt_handler(void *args)
{
    CAL_STAT = CAL_GO; //removed unnecessary if-statement
}

static void IRAM_ATTR sound_btn_interrupt_handler(void *args)
{
    BaseType_t xSoundBtnTaskWoken = pdFALSE;
    ets_printf("\nsnd btn");
    vTaskNotifyGiveFromISR(xNotifySoundBtn, &xSoundBtnTaskWoken);
    portYIELD_FROM_ISR(xSoundBtnTaskWoken);
}

static bool IRAM_ATTR timer0_group_isr_callback(void *para){
    BaseType_t xDftTaskWoken = pdFALSE;

    //Full cycle?
    if (int_count == 3){
        //Reset counter
        int_count = 0;
    }
    else{
        if (int_count == 0){
            //Set PWM high
            gpio_set_level(PWM_GPIO, 1);

            if (BUF_STAT == BUF_FREE){
            BUF_STAT = BUF_SAMPLE;
            }
            if (cycle_count < 4){
                cycle_count++;
            }
            if (cycle_count == 3)
            {
                //CAL_STAT = CAL_GO;
            }
        }
        else if (int_count == 2){
            //Set PWM low
            gpio_set_level(PWM_GPIO, 0);
        }
        //Increment interrupt counter
        int_count++;
    }
    
    //4 cycle start-up time over?
    if (cycle_count == 4){
        //Buffer free?
        if (BUF_STAT == BUF_SAMPLE){
            x_sample[idx++] = adc1_get_raw((adc1_channel_t)channel1);
            //Buffer full?
            if (idx == BFR_SIZE){
                idx = 0;
                BUF_STAT = BUF_FULL;
                vTaskNotifyGiveFromISR(xNotifyDft, &xDftTaskWoken);
                portYIELD_FROM_ISR(xDftTaskWoken);
            }
        }
    }
    return (xDftTaskWoken == pdTRUE);
}

static bool IRAM_ATTR timer1_group_isr_callback(void *para){
    BaseType_t xSoundTaskWoken = pdFALSE;
    //ets_printf("\ntimer1 isr");
    vTaskNotifyGiveFromISR(xNotifySound, &xSoundTaskWoken);
    portYIELD_FROM_ISR(xSoundTaskWoken);
    return (xSoundTaskWoken == pdTRUE);
}

static void tg0_timer0_init()
{
    int timer_group = TIMER_GROUP_0;
    int timer_idx = TIMER_0;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIM0_DIV;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, 1);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    //timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer0_group_isr_callback, NULL, 0); 
}

static void tg1_timer1_init()
{
    int timer_group = TIMER_GROUP_1;
    int timer_idx = TIMER_1;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIM1_DIV;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, 200);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    //timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    timer_isr_callback_add(TIMER_GROUP_1, TIMER_1, timer1_group_isr_callback, NULL, 0); 
}

void dft_task(void *pvParameters)
{   
    static uint32_t dft_notification;
    int dft_counter = 0;
    // if (cycle_count == 0 && int_count == 0){
    //     timer_start(TIMER_GROUP_0, TIMER_0);
    // }
    for(;;){
        dft_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(dft_notification)
        {
            make_dft(&ramp, &rphase, x_sample);
            // if (rphase >= 90){
            //     rphase = 270 - rphase;
            // }
            // else {
            //     rphase = -90 - rphase;
            // }
            // if (rphase < 0)
            // {
            //     rphase += 360;
            // }
            if (CAL_STAT == CAL_GO){
                xTaskNotifyGive(xNotifyCal);
            }
            else {
                //Filter display values (IIR)
                filt_amp = 0.9*filt_amp + 0.1*ramp;
                filt_phase = 0.9*filt_phase + 0.1*rphase;

                //Calibrate display values
                disp_amp = filt_amp - cal_amp;
                disp_phase = filt_phase - cal_phase;
                
                // //Adjust phase boundaries
                // if (disp_phase > 180){
                //     disp_phase -= 360 ;
                // }
                // else if (disp_phase < -180){
                //     disp_phase += 360;
                // }
                // disp_phase = -disp_phase; //invert sign of phase
                

                //Identify metal
                if (disp_amp > 0.2)
                {
                    if (disp_phase <= 65){
                        METAL_ID = FERROUS;
                    }
                    else {
                        METAL_ID = NONFERROUS;
                    }
                }
                else {
                    METAL_ID = NA;
                }
                

                if (dft_counter == 9){
                    xTaskNotifyGive(xNotifyDisp);
                    dft_counter = 0;
                }
                else {
                    dft_counter++;
                }
            }
            BUF_STAT = BUF_FREE;
        }
    }
}

void cal_task(void *pvParameters)
{   
    int cal_counter = 0;
    static uint32_t cal_notification;

    for(;;){
        cal_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(cal_notification)
        {   
            if (cal_counter == 0)
            {
                cal_amp = 0;
                cal_phase = 0;
            }
        
            if (cal_counter++ < CAL_CYCLES){
                cal_amp += (ramp/CAL_CYCLES);
                cal_phase += (rphase/CAL_CYCLES);
            }
            else {
                cal_counter = 0;
                printf("\ncalibration successful");
                CAL_STAT = CAL_STOP;
            }
        }
    }
}

void display_task(void *pvParameters)
{
    // ssd1306_init(&dev, 128, 64);
    // ssd1306_clear_screen(&dev, false);
	// ssd1306_contrast(&dev, 0xff);
    char amp_string[20] = {0};
    char phase_string[20] = {0};

    static uint32_t disp_notification;
    
    for ( ;; )
    {
        if(sound_display == 0 && cal_display == 0){
            disp_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            if(disp_notification){
                sprintf(amp_string, "A:%03d", (int)(100*disp_amp));
                sprintf(phase_string, "P:%03d", (int)disp_phase);
                // sendStrXY(amp_string, 1, 0);
                // sendStrXY(phase_string, 2, 0);
                ssd1306_display_text_x3(&dev, 2, amp_string, 6, false);
                ssd1306_display_text_x3(&dev, 5, phase_string, 6, false);
                if (METAL_ID == FERROUS)
                {
                    ssd1306_display_text(&dev, 0, "ID:     FERROUS", 15, false);
                }
                else if (METAL_ID == NONFERROUS)
                {
                    ssd1306_display_text(&dev, 0, "ID: NON-FERROUS", 15, false);
                }
                else
                {
                    ssd1306_display_text(&dev, 0, "ID:         N/A", 15, false);
                }
                //printf("\nR: %03d F: %03d D: %03d", (int)rphase, (int)filt_phase, (int)disp_phase);
            }
        }
    }
}

void sound_btn_task(void *pvParameters)
{
    static uint32_t sound_btn_notification;

    for ( ;; )
    {
        sound_btn_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(sound_btn_notification){
            gpio_set_intr_type(SOUND_BTN_GPIO, GPIO_INTR_DISABLE);
            SOUND_STAT = !SOUND_STAT;
            ets_printf("\nsound stat: %d", (int)SOUND_STAT);
            sound_counter = 0;
            if (SOUND_STAT == SOUND_ON)
            {
                ets_printf("\ntimer1_start");
                timer_start(TIMER_GROUP_1, TIMER_1);
                xTaskNotifyGive(xStartupSound);
            }
            else{
                ets_printf("\ntimer1_pause");
                timer_pause(TIMER_GROUP_1, TIMER_1);
                gpio_set_level(SOUND_OUT_GPIO, 0);
            }
            sound_display = 1;
            xTaskNotifyGive(xSoundDisplay);
            usleep(1500000);
            gpio_set_intr_type(SOUND_BTN_GPIO, GPIO_INTR_NEGEDGE);
            //Sound_Btn_Reset = 1;
        }
    }
}

void sound_task(void *pvParameters)
{
    static uint32_t sound_notification;
    int sc_max = 0;
    int sound_amp = 0;

    for ( ;; )
    {
        sound_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(sound_notification){
            sound_amp = (int)100*disp_amp;
            if (sound_amp < 0)
            {
                sound_amp = 0;
            }
            sc_max = 30 - (4.5*sound_amp/10);
            if (sc_max < 2)
            {
                sc_max = 2;
            }
            if (sound_counter == 0){ 
                if (sound_amp > SOUND_THRESHOLD){
                gpio_set_level(SOUND_OUT_GPIO, 1);
                //ets_printf("\nsound on");
                }
            }
            else if (sound_counter == 1){
                gpio_set_level(SOUND_OUT_GPIO, 0);
                //ets_printf("\nsound off");
            }
            sound_counter++;
            if (sound_counter >= sc_max)
            {   
                ets_printf("\nsound cycle");
                sound_counter = 0;
                // if (Sound_Btn_Reset == 1){
                //     gpio_set_intr_type(SOUND_BTN_GPIO, GPIO_INTR_NEGEDGE);
                //     Sound_Btn_Reset = 0;
                // }
            } 
        }
    }
}

void sd_task(void *pvParameters)
{
    static uint32_t sd_notification;
    for ( ;; )
    {
        sd_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(sd_notification){
            ssd1306_clear_screen(&dev, false);
            ssd1306_display_text_x3(&dev, 2, "SOUND", 5, false);
            if (SOUND_STAT == SOUND_ON)
            {
                ssd1306_display_text_x3(&dev, 5, "ON", 2, false);
            }
            else
            {
                ssd1306_display_text_x3(&dev, 5, "OFF", 3, false);
            }
            vTaskDelay(1500 / portTICK_PERIOD_MS);
            ssd1306_clear_screen(&dev, false);
            sound_display = 0;
        }
    }
}

void startup_display(void *pvParameters)
{
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
    //int top = 2;
	int center = 3;
	//int bottom = 8;

    // Startup Display 
	ssd1306_display_text_x3(&dev, center, "GOLD", 4, false);
	ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_LEFT);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_STOP);
    //ssd1306_clear_screen(&dev, false);
    
    ssd1306_display_text_x3(&dev, center, " DIG", 4, false);
	vTaskDelay(300 / portTICK_PERIOD_MS);
    ssd1306_display_text_x3(&dev, center, " GER", 4, false);
    vTaskDelay(400 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_DOWN);
	vTaskDelay(400 / portTICK_PERIOD_MS);
    ssd1306_hardware_scroll(&dev, SCROLL_STOP);
    ssd1306_clear_screen(&dev, true);

    //xTaskNotifyGive(xStartupSound);

    for (int i = 0; i < 15; i++)
    {
        ssd1306_display_text_x3(&dev, center, " 2000", 5, false);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ssd1306_display_text_x3(&dev, center, " 2000", 5, true);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    
    ssd1306_fadeout(&dev);

    gpio_set_intr_type(CAL_GPIO, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(SOUND_BTN_GPIO, GPIO_INTR_NEGEDGE);
    timer_start(TIMER_GROUP_0, TIMER_0);

    vTaskDelete(xStartupDisplay);
}

void startup_sound(void *pvParameters)
{
    static uint32_t startup_notification;
    int snd_tggle = 1;

    for(;;){
        startup_notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(startup_notification){
            //Play Darude - Sandstorm
            for (int i = 0; i < 8; i++)
            {
                gpio_set_level(SOUND_OUT_GPIO, snd_tggle);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                snd_tggle = !snd_tggle;
            }

            gpio_set_level(SOUND_OUT_GPIO, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(SOUND_OUT_GPIO, 0);
        }
    }
}

void app_main(void)
{
    //ADC configuration
    check_efuse();
    if (unit1 == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel1, atten1);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel1, atten1);
    }
    adc_chars1 = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type1 = esp_adc_cal_characterize(unit1, atten1, width, DEFAULT_VREF, adc_chars1);
    print_char_val_type(val_type1);

    //Output pin configuration
    gpio_set_direction(PWM_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(SOUND_OUT_GPIO, GPIO_MODE_OUTPUT);

    //Calibration button configuration
    gpio_pad_select_gpio(CAL_GPIO);
    gpio_set_direction(CAL_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(CAL_GPIO);
    gpio_pulldown_dis(CAL_GPIO);
    // gpio_set_intr_type(CAL_GPIO, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CAL_GPIO, cal_interrupt_handler, (void *)CAL_GPIO);

    //Sound button configuration
    gpio_pad_select_gpio(SOUND_BTN_GPIO);
    gpio_set_direction(SOUND_BTN_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(SOUND_BTN_GPIO);
    gpio_pulldown_dis(SOUND_BTN_GPIO);
    gpio_set_intr_type(SOUND_BTN_GPIO, GPIO_INTR_NEGEDGE);
    //gpio_install_isr_service(0); //commented out 
    gpio_isr_handler_add(SOUND_BTN_GPIO, sound_btn_interrupt_handler, (void *)SOUND_BTN_GPIO);

    //i2C configuration for OLED
    #if CONFIG_I2C_INTERFACE
        ESP_LOGI(tag, "INTERFACE is i2c");
        ESP_LOGI(tag, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
        ESP_LOGI(tag, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
        ESP_LOGI(tag, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
        i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
    #endif // CONFIG_I2C_INTERFACE
    
    //Timer initialization
    tg0_timer0_init();
    tg1_timer1_init();

    //Create RTOS tasks
    xTaskCreate(dft_task, "dft_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xNotifyDft);
    xTaskCreate(cal_task, "cal_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xNotifyCal);
    xTaskCreate(display_task, "display_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xNotifyDisp);
    xTaskCreate(sound_btn_task, "sound_btn_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xNotifySoundBtn);
    xTaskCreate(sound_task, "sound_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xNotifySound);
    xTaskCreate(startup_display, "startup_display", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 2, &xStartupDisplay);
    xTaskCreate(startup_sound, "startup_sound", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 2, &xStartupSound);
    xTaskCreate(sd_task, "sd_task", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xSoundDisplay);
}

