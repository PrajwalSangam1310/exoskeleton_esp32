/* @brief the test project to work with gpio and analog pins */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_adc_cal.h"
#include <inttypes.h>

// digital pins
#define BLDC_PWM_U_GPIO 13
#define BLDC_PWM_V_GPIO 12
#define BLDC_PWM_W_GPIO 14

// ledc configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY               (512) // Set duty to 50%. ((2 ** 10) - 1) * 50% = 1023
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

#define PWM_MAX_DUTY 255
#define PWM_DUTY_STEP 1
#define PWM_MIN_DUTY 0
#define LEDC_U_CHANNEL LEDC_CHANNEL_0
#define LEDC_V_CHANNEL LEDC_CHANNEL_1
#define LEDC_W_CHANNEL LEDC_CHANNEL_2

//inhibit pins
#define BLDC_EN_U_GPIO 27
#define BLDC_EN_V_GPIO 26
#define BLDC_EN_W_GPIO 25

// analog pins
#define HALL_CAP_U_GPIO  33
#define HALL_CAP_V_GPIO  32
#define HALL_CAP_W_GPIO  35
#define ESP_INTR_FLAG_DEFAULT 0
#define TEST_BUTTON_GPIO 34

//hall pins as the inputs
#define GPIO_HALL_PIN_SEL ((1ULL<<HALL_CAP_U_GPIO) | (1ULL<<HALL_CAP_V_GPIO) | (1ULL<<HALL_CAP_W_GPIO))

//adc channel
#define POTENTIO_METER_IN ADC1_CHANNEL_3 //gpio 39, SVN
static const adc_bits_width_t width = ADC_WIDTH_BIT_10;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static uint32_t duty;
#define DEFAULT_VREF    1100 

//en pins as outputs
#define GPIO_EN_PIN_SEL ((1ULL<<BLDC_EN_U_GPIO) | (1ULL<<BLDC_EN_V_GPIO) | (1ULL<<BLDC_EN_W_GPIO))

uint32_t hall_sensor_value;

static inline uint32_t bldc_get_hall_sensor_value(bool ccw)
{
    uint32_t hall_val = gpio_get_level(HALL_CAP_U_GPIO) * 4 + gpio_get_level(HALL_CAP_V_GPIO) * 2 + gpio_get_level(HALL_CAP_W_GPIO) * 1;
    return ccw ? hall_val ^ (0x07) : hall_val;
}

void update_bldc_speed(){
  uint32_t adc_reading = adc1_get_raw((adc1_channel_t)POTENTIO_METER_IN);
  duty = adc_reading;
}

void update_hardware(){
      //run the motor 
  switch (hall_sensor_value) {
    case 1:
      gpio_set_level(BLDC_EN_U_GPIO,1);
      gpio_set_level(BLDC_EN_V_GPIO,1);
      gpio_set_level(BLDC_EN_W_GPIO,0);
      ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, duty);
      ledc_set_duty(LEDC_MODE,LEDC_V_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_W_CHANNEL, 0);
      ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_V_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_W_CHANNEL);
      break;
      
    case 2:
      gpio_set_level(BLDC_EN_U_GPIO,1);
      gpio_set_level(BLDC_EN_V_GPIO,0);
      gpio_set_level(BLDC_EN_W_GPIO,1);
      ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, duty);
      ledc_set_duty(LEDC_MODE,LEDC_V_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_W_CHANNEL, 0);
      ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_V_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_W_CHANNEL);
      break;

   case 3:
      gpio_set_level(BLDC_EN_U_GPIO,0);
      gpio_set_level(BLDC_EN_V_GPIO,1);
      gpio_set_level(BLDC_EN_W_GPIO,1);
      ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_V_CHANNEL, duty);
      ledc_set_duty(LEDC_MODE,LEDC_W_CHANNEL, 0);
      ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_V_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_W_CHANNEL);
      break;

   case 4:
      gpio_set_level(BLDC_EN_U_GPIO,1);
      gpio_set_level(BLDC_EN_V_GPIO,1);
      gpio_set_level(BLDC_EN_W_GPIO,0);
      ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_V_CHANNEL, duty);
      ledc_set_duty(LEDC_MODE,LEDC_W_CHANNEL, 0);
      ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_V_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_W_CHANNEL);
      break;

    case 5:
      gpio_set_level(BLDC_EN_U_GPIO,1);
      gpio_set_level(BLDC_EN_V_GPIO,0);
      gpio_set_level(BLDC_EN_W_GPIO,1);
      ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_V_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_W_CHANNEL, duty);
      ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_V_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_W_CHANNEL);
      break;

    case 6:
      gpio_set_level(BLDC_EN_U_GPIO,0);
      gpio_set_level(BLDC_EN_V_GPIO,1);
      gpio_set_level(BLDC_EN_W_GPIO,1);
      ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_V_CHANNEL, 0);
      ledc_set_duty(LEDC_MODE,LEDC_W_CHANNEL, duty);
      ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_V_CHANNEL);
      ledc_update_duty(LEDC_MODE,LEDC_W_CHANNEL);
      break;

   default:
   break;
  } 
}

static void pwm_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    //pwm is done through ledc in esp32
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
    };

    ledc_channel.gpio_num       = BLDC_PWM_U_GPIO;
    ledc_channel.channel = LEDC_U_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.gpio_num       = BLDC_PWM_V_GPIO;
    ledc_channel.channel = LEDC_V_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.gpio_num = BLDC_PWM_W_GPIO;
    ledc_channel.channel = LEDC_W_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    printf("PWM initialization done with duty cycle 127");
}

// static void IRAM_ATTR gpio_isr_handler(void* arg)
// {
//     hall_sensor_value = bldc_get_hall_sensor_value(false);
// }

static void IRAM_ATTR commutation_button_isr_handler(void* arg){
  hall_sensor_value += 1;
  if(hall_sensor_value > 6){
    hall_sensor_value = 1;
  }
  update_hardware();
}



void app_main(void)
{
    //start the pwm signals
    pwm_init();

    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_HALL_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // enable pins init
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_EN_PIN_SEL;

    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    // gpio_isr_handler_add(HALL_CAP_U_GPIO, gpio_isr_handler, (void*) HALL_CAP_U_GPIO);
    // gpio_isr_handler_add(HALL_CAP_V_GPIO, gpio_isr_handler, (void*) HALL_CAP_V_GPIO);
    // gpio_isr_handler_add(HALL_CAP_W_GPIO, gpio_isr_handler, (void*) HALL_CAP_W_GPIO);

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << TEST_BUTTON_GPIO);
    gpio_config(&io_conf);
    gpio_isr_handler_add(TEST_BUTTON_GPIO, commutation_button_isr_handler, (void*) TEST_BUTTON_GPIO);

    //setting up the ad channel as input
    adc1_config_width(width);
    adc1_config_channel_atten(POTENTIO_METER_IN, atten);
    esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    // print_char_val_type(val_type);

    // create the periodic call back function
    const esp_timer_create_args_t bldc_timer_args = {
        .callback = update_bldc_speed,
        .name = "bldc_speed"
    };

    esp_timer_handle_t bldc_speed_timer;
    ESP_ERROR_CHECK(esp_timer_create(&bldc_timer_args, &bldc_speed_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(bldc_speed_timer, 200000));

    //ledc config for pwm
    while (1) {
        // The rotation direction is controlled by inverting the hall sensor value
        printf("The hall state is %lu \n", (unsigned long)hall_sensor_value);
        printf(" The duty value is %lu \n", (unsigned long)duty);
        printf(" The raw value is %lu \n", (unsigned long)adc1_get_raw((adc1_channel_t)POTENTIO_METER_IN));
        vTaskDelay(100/portTICK_RATE_MS);
    }
}
