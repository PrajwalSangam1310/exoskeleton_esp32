/* Pulse counter module - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "example";

/**
 * TEST CODE BRIEF
 *
 * Use PCNT module to count rising edges generated by LEDC module.
 *
 * Functionality of GPIOs used in this example:
 *   - GPIO18 - output pin of a sample 1 Hz pulse generator,
 *   - GPIO4 - pulse input pin,
 *   - GPIO5 - control input pin.
 *
 * Load example, open a serial port to view the message printed on your screen.
 *
 * To do this test, you should connect GPIO18 with GPIO4.
 * GPIO5 is the control signal, you can leave it floating with internal pull up,
 * or connect it to ground. If left floating, the count value will be increasing.
 * If you connect GPIO5 to GND, the count value will be decreasing.
 *
 * An interrupt will be triggered when the counter value:
 *   - reaches 'thresh1' or 'thresh0' value,
 *   - reaches 'l_lim' value or 'h_lim' value,
 *   - will be reset to zero.
 */
#define PCNT_H_LIM_VAL      12
#define PCNT_L_LIM_VAL     -12
#define PCNT_THRESH1_VAL    6
#define PCNT_THRESH0_VAL   -6
#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

// ledc configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_MAX_VALUE 100 
#define LEDC_DUTY               (127) // Set duty to 50%. ((2 ** 10) - 1) * 50% = 1023
#define LEDC_FREQUENCY          (136553) // Frequency in Hertz. Set frequency at 5 kHz
#define U_LEDC 21
#define LEDC_U_CHANNEL LEDC_CHANNEL_2
// #define LEDC_V_CHANNEL LEDC_CHANNEL_1
// #define LEDC_W_CHANNEL LEDC_CHANNEL_2

//sinusoidal variables

float sin_values[31] = {
0.500000000000000,
0.676597532934096,
0.826344966318691,
0.930873429586344,
0.985614527309043,
1,
0.992533073424763,
0.982672028056257,
0.982672028056257,
0.992533073424763,
1,
0.985614527309043,
0.930873429586344,
0.826344966318691,
0.676597532934096,
0.500000000000000,
0.323402467065905,
0.173655033681310,
0.0691265704136561,
0.0143854726909571,
0,
0.00746692657523730,
0.0173279719437430,
0.0173279719437430,
0.00746692657523730,
0,
0.0143854726909571,
0.0691265704136561,
0.173655033681310,
0.323402467065905,
0.500000000000000
};

#define SPEED_CONSTANT 0.5;
uint32_t duty_sine_values[31];


int u_index = 0;
int index_offset = 10; 
int n_samples = 31;
uint32_t duty;
xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */

void update_duty_sine_values(float speed_constant){
    for(int i =0;i < n_samples; i++){
        duty_sine_values[i] = sin_values[i]*speed_constant * LEDC_MAX_VALUE;
    }
}
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;

    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);

    u_index++;
    u_index %= 31;
    ledc_set_duty(LEDC_MODE,LEDC_U_CHANNEL, duty_sine_values[u_index]);
    ledc_update_duty(LEDC_MODE,LEDC_U_CHANNEL);
}



/* Configure LED PWM Controller
 * to output sample pulses at 1 Hz with duty of about 10%
 */
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_4_BIT;
    ledc_timer.freq_hz          = 11367;  // set output frequency at 1 Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = LEDC_OUTPUT_IO;
    ledc_channel.duty       = 7; // set duty at about 10%
    ledc_channel.hpoint     = 0;
    ledc_channel_config(&ledc_channel);
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

    ledc_channel.gpio_num       = U_LEDC;
    ledc_channel.channel =  LEDC_U_CHANNEL;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    printf("PWM initialization done with duty cycle 127");
}



/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(int unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_INC,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_DISABLE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

void app_main(void)
{
    int pcnt_unit = PCNT_UNIT_0;
    /* Initialize LEDC to generate sample pulse signal */
    pwm_init();
    ledc_init();
    update_duty_sine_values(0.5);
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init(pcnt_unit);
    int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    uint32_t test2_duty;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            pcnt_get_counter_value(pcnt_unit, &count);
            ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
            if (evt.status & PCNT_EVT_L_LIM) {
                ESP_LOGI(TAG, "L_LIM EVT");
            }
            if (evt.status & PCNT_EVT_H_LIM) {
                ESP_LOGI(TAG, "H_LIM EVT");
            }
        } else {
            pcnt_get_counter_value(pcnt_unit, &count);
            ESP_LOGI(TAG, "Current counter value :%d", count);
            test2_duty = (uint32_t)(sin_values[u_index]*LEDC_MAX_VALUE); 
            printf("the duty value is: %d", test2_duty);
        }
    }
}

