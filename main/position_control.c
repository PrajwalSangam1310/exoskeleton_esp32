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
#include "esp_timer.h"
#include "rotary_encoder.h"

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
#define PCNT_THRESH1_VAL    6
#define PCNT_THRESH0_VAL   -6
#define ENCODER_PIN_A   17  // Pulse Input GPIO
#define ENCODER_PIN_B  16  // Control GPIO HIGH=count up, LOW=count down
#define ENCODER_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator
#define ENCODER_OUTPUT_FREQUENCY 68267

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



#define FUNCTION_CALL_TIME 200000
#define PI 3.142857143
#define REDUCTION_RATIO 70
#define KP_POSITION 10
#define KI_POSITION 0.0001
#define KP_VELOCITY 10
#define KI_VELOCITY 0.0001
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
int ENCODER_PCNT_UNIT = PCNT_UNIT_0;
rotary_encoder_t *encoder = NULL;
int reference_position;
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

/* Configure LED PWM Controller
 * to output sample pulses at 1 Hz with duty of about 10%
 */
static void ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num        = LEDC_TIMER_1;
    ledc_timer.duty_resolution  = LEDC_TIMER_2_BIT;
    ledc_timer.freq_hz          = ENCODER_OUTPUT_FREQUENCY;  
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel    = LEDC_CHANNEL_1;
    ledc_channel.timer_sel  = LEDC_TIMER_1;
    ledc_channel.intr_type  = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num   = ENCODER_OUTPUT_IO;
    ledc_channel.duty       = 2; // set duty at about 10%
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
void encoder_init()
{
    uint32_t pcnt_unit = ENCODER_PCNT_UNIT;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, ENCODER_PIN_A, ENCODER_PIN_B);
    
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));
}

int get_counter_from_angle(float angle_radians){
    int reference_counter_value = angle_radians/2/PI*4096*REDUCTION_RATIO;
    return reference_counter_value;
}

void update_bldc_speed(){
    static int16_t past_counter_value;
    static int16_t current_counter_value;
    static int64_t past_time = 0; // time in microseconds
    

    static int position_error_i = 0;
    // update current velocity
    current_counter_value = encoder->get_counter_value(encoder);
    printf("current counter value is %d\n", current_counter_value);
    float current_velocity = (current_counter_value - past_counter_value)*1000000/(esp_timer_get_time() - past_time);
    past_time = esp_timer_get_time();
    printf("The current velocity is: %f\n", current_velocity);
    past_counter_value =  current_counter_value;
    
    //pi position equation
    int reference_counter_value = get_counter_from_angle(PI/3);
    float position_error = (reference_counter_value - current_counter_value);
    position_error_i += position_error;

    float reference_velocity = KP_POSITION*position_error * KI_POSITION*position_error_i; 
    // printf("Reference speed is calculated as: %f \n", reference_velocity);

    //pi velocity control
    float velocity_error = reference_velocity - current_velocity;
    float speed_constant = KP_VELOCITY*velocity_error + KI_VELOCITY*velocity_error;
    // printf("The calculated duty cycle is: %f\n", speed_constant);
}

void periodic_speed_update_func_init(){
        // create the periodic call back function
    const esp_timer_create_args_t bldc_timer_args = {
        .callback = update_bldc_speed,
        .name = "bldc_speed"
    };

    esp_timer_handle_t bldc_speed_timer;
    ESP_ERROR_CHECK(esp_timer_create(&bldc_timer_args, &bldc_speed_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(bldc_speed_timer, FUNCTION_CALL_TIME));
}

void app_main(void)
{

    /* Initialize LEDC to generate sample pulse signal */
    pwm_init();
    ledc_init();
    update_duty_sine_values(0.5);
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    encoder_init();
    periodic_speed_update_func_init();
    int16_t count = 0;
    pcnt_evt_t evt;
    portBASE_TYPE res;
    uint32_t test2_duty;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        // res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        // if (res == pdTRUE) {
        //     pcnt_get_counter_value(ENCODER_PCNT_UNIT, &count);
        //     ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
        //     if (evt.status & PCNT_EVT_L_LIM) {
        //         ESP_LOGI(TAG, "L_LIM EVT");
        //     }
        //     if (evt.status & PCNT_EVT_H_LIM) {
        //         ESP_LOGI(TAG, "H_LIM EVT");
        //     }
        // } else {
        //     pcnt_get_counter_value(ENCODER_PCNT_UNIT, &count);
        //     ESP_LOGI(TAG, "Current counter value :%d", count);
        //     test2_duty = (uint32_t)(sin_values[u_index]*LEDC_MAX_VALUE); 
        //     printf("the duty value is: %d", test2_duty);
        // }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

