#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#define BLDC_PWM_U_GPIO 13
#define BLDC_PWM_V_GPIO 12
#define BLDC_PWM_W_GPIO 14

#define LEDC_U_CHANNEL LEDC_CHANNEL_0
#define LEDC_V_CHANNEL LEDC_CHANNEL_1
#define LEDC_W_CHANNEL LEDC_CHANNEL_2

// ledc configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_9_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY               (1) // Set duty to 50%. ((2 ** 10) - 1) * 50% = 1023
#define LEDC_FREQUENCY          (100000) // Frequency in Hertz. Set frequency at 5 kHz

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

void app_main(void){
    pwm_init();
    printf("PWM INITIALIZED!\n");
}