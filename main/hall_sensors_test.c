/* @brief the test project to work with gpio and analog pins */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include <inttypes.h>

// digital pins
// #define PWM_U 25
// #define PWM_V 26
// #define PWM_W 27
// #define EN_U 14
// #define EN_V 12
// #define EN_W 13

// analog pins
#define HALL_CAP_U_GPIO  33
#define HALL_CAP_V_GPIO  32
#define HALL_CAP_W_GPIO  35

#define GPIO_HALL_PIN_SEL ((1ULL<<HALL_CAP_U_GPIO) | (1ULL<<HALL_CAP_V_GPIO) | (1ULL<<HALL_CAP_W_GPIO))


static inline uint32_t bldc_get_hall_sensor_value(bool ccw)
{
    uint32_t hall_val = gpio_get_level(HALL_CAP_U_GPIO) * 4 + gpio_get_level(HALL_CAP_V_GPIO) * 2 + gpio_get_level(HALL_CAP_W_GPIO) * 1;
    return ccw ? hall_val ^ (0x07) : hall_val;
}

void app_main(void)
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
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

    uint32_t hall_sensor_value;
    while (1) {
        // The rotation direction is controlled by inverting the hall sensor value
        hall_sensor_value = bldc_get_hall_sensor_value(false);
        printf("The hall state is %lu \n", (unsigned long)hall_sensor_value);
        vTaskDelay(10/portTICK_RATE_MS);
    }
    
}
