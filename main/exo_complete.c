/* @brief the test project to work with gpio and analog pins */


//standard includes
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

//motor includes
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_adc_cal.h"
#include "rotary_encoder.h"

//esp32 includes
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

//mqtt includes
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"

//motor initialization
// digital pins
#define BLDC_PWM_U_GPIO 13
#define BLDC_PWM_V_GPIO 12
#define BLDC_PWM_W_GPIO 14

// ledc configuration
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_9_BIT // Set duty resolution to 8 bits
#define LEDC_DUTY               (127) // Set duty to 50%. ((2 ** 10) - 1) * 50% = 1023
#define LEDC_FREQUENCY          (3125) // Frequency in Hertz. Set frequency at 5 kHz

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
#define POTENTIO_METER_IN ADC1_CHANNEL_6 //gpio 39, SVN
static const adc_bits_width_t width = ADC_WIDTH_BIT_9;
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;
static uint32_t duty;
uint16_t hall_counts; // for speed calculation
bool rotation_direction = true; // true for counter clockwise
float motor_speed;
#define DEFAULT_VREF    1100 

//en pins as outputs
#define GPIO_EN_PIN_SEL ((1ULL<<BLDC_EN_U_GPIO) | (1ULL<<BLDC_EN_V_GPIO) | (1ULL<<BLDC_EN_W_GPIO))

uint32_t hall_sensor_value;
uint32_t old_hall_sensor_value;
uint32_t commutation_state = 1;
static const char *TAG = "example";

//encoder pins
// #define ENCODER_PIN_A 36 //svp
// #define ENCODER_PIN_B 39 //svn
#define ENCODER_PIN_A 2 //svp
#define ENCODER_PIN_B 15 //svn
rotary_encoder_t *encoder = NULL;



//mqtt initialization

#define EXO_ESP_ID "RIGHT_THIGH"
esp_mqtt_client_handle_t client;
const char send_topic[] = "/" EXO_ESP_ID "/feedback_data"; 
const char receive_topic[] = "/" EXO_ESP_ID "/command_data"; 

//enum for control mode selection 
enum CONTROL_MODE {
    POSITION = 0,
    VELCOCITY = 1,
    EFFORT = 2,
};

//enum for inturrpts
enum STOP_INTURRPT{
    NO_INTERRUPT = 0,
    MANUAL_STOP = 1,
    MASTER_STOP = 2,
    LOCAL_STOP = 3,
};

//command payload received by the broker
struct command_payload{
    float goal_position;
    float goal_velocity;
    float effort;
    enum CONTROL_MODE mode;
    enum STOP_INTURRPT emergency_command;
} ;

struct command_payload received_payload = {};

// feeedback paylaod to be sent back to the broker
struct feedback_payload{
    float joint_angle;
    float velocity;
    float battery_level;
    float motor_current;
    float imu[4];
};

struct feedback_payload test_feedback_payload = {
    .joint_angle = 0.5,
    .velocity = 0.5,
    .battery_level = 0.5,
    .motor_current = 0.5,
    .imu = {0.5,0.5,0.5,0.5}
};

//bldc definations
static inline uint32_t bldc_get_hall_sensor_value(bool ccw)
{
    uint32_t hall_val = gpio_get_level(HALL_CAP_U_GPIO) * 4 + gpio_get_level(HALL_CAP_V_GPIO) * 2 + gpio_get_level(HALL_CAP_W_GPIO) * 1;
    return ccw ? hall_val ^ (0x07) : hall_val;
}

void update_bldc_speed(){
  duty = 100;
  motor_speed = hall_counts/(4096*4*0.2);//the update speed in 0.2
  hall_counts = 0;
}

void update_hardware(){
      //run the motor 
  switch (commutation_state) {
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
        .freq_hz          = 31250,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 0, // Set duty to 0%
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

static void IRAM_ATTR gpio_isr_handler_func(void* arg){
  hall_sensor_value = bldc_get_hall_sensor_value(false);
//   printf("this isr is running on %d\n", xPortGetCoreID());

  if (old_hall_sensor_value != hall_sensor_value)
  {
    update_hardware();
    old_hall_sensor_value = hall_sensor_value;
    commutation_state == 6 ? commutation_state = 1 : (commutation_state++) ;
  }
  hall_counts++;
}

void hall_sensor_init(){
        //hall_sensor_config
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
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
    //install gpio isr service

}

void enable_pin_init(){
        //hall_sensor_config
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_EN_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void potentiometer_init(){
        //setting up the ad channel as input
    adc1_config_width(width);
    adc1_config_channel_atten(POTENTIO_METER_IN, atten);
    esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
}

void periodic_speed_update_func_init(){
        // create the periodic call back function
    const esp_timer_create_args_t bldc_timer_args = {
        .callback = update_bldc_speed,
        .name = "bldc_speed"
    };

    esp_timer_handle_t bldc_speed_timer;
    ESP_ERROR_CHECK(esp_timer_create(&bldc_timer_args, &bldc_speed_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(bldc_speed_timer, 200000));
}

void update_feedback(){
    test_feedback_payload.joint_angle = (float) encoder->get_counter_value(encoder);
    test_feedback_payload.battery_level = (float)hall_sensor_value;
    test_feedback_payload.velocity = (float)duty;
    esp_mqtt_client_publish(client, send_topic,(char*)&test_feedback_payload ,sizeof(struct feedback_payload), 1, 0);

}

void periodic_feedback_update_func_init(){
        // create the periodic call back function
    const esp_timer_create_args_t update_feedback_timer_args = {
        .callback = update_feedback,
        .name = "feedbacck_update"
    };

    esp_timer_handle_t update_feedback_timer;
    ESP_ERROR_CHECK(esp_timer_create(&update_feedback_timer_args, &update_feedback_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(update_feedback_timer, 200000));
}


void core1_tasks(void *params){
    //setting the isr rotuine in core 1 
    hall_sensor_init();

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(HALL_CAP_U_GPIO, gpio_isr_handler_func, (void*) HALL_CAP_U_GPIO);
    gpio_isr_handler_add(HALL_CAP_V_GPIO, gpio_isr_handler_func, (void*) HALL_CAP_V_GPIO);
    gpio_isr_handler_add(HALL_CAP_W_GPIO, gpio_isr_handler_func, (void*) HALL_CAP_W_GPIO);

    uint32_t old_hall_sensor_value = 0;
    while(1){
        printf("Doing nothing i am in core %d aaa\n",xPortGetCoreID());
        vTaskDelay(1000/portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

//mqtt function definations
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

void show_received_payload(const char* received_data){
    memcpy(&received_payload, received_data, sizeof(struct command_payload));
    printf("the received string is: %.20s\n", received_data);
    printf("goal position is: %f\n",received_payload.goal_position);
    printf("goal velocity is: %f\n",received_payload.goal_velocity);
    printf("goal effort is: %f\n",received_payload.effort);
    printf("control mode is: %d\n",received_payload.mode);
    printf("Inturrpt command is: %d\n",received_payload.emergency_command);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

        msg_id = esp_mqtt_client_publish(client, send_topic,(char*)&test_feedback_payload ,sizeof(struct feedback_payload), 1, 0);
        ESP_LOGI(TAG, "sent publish request successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, receive_topic, 0);
        ESP_LOGI(TAG, "sent subscribe resquest successful, msg_id=%d", msg_id);

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "Sucessfully subscribed to the topic, msg_id=%d", event->msg_id);
        ESP_LOGI(TAG, "Sending the test data=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, receive_topic, "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "Unsubscribed from the topic, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "Successfully published to the topic, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        printf("Got data from the master node\n");
        if (event->data_len == sizeof(struct command_payload)){
            show_received_payload(event->data);
        }
        else{
            printf("The topic length mismatch with payload length\n");
            printf("Topic data length = %d,   Receive payload length = %d\n", sizeof(struct command_payload), event->data_len );
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://192.168.13.251:1883",
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void encoder_init(){
    uint32_t pcnt_unit = 0;

    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, ENCODER_PIN_A, ENCODER_PIN_B);
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));
}

void motor_init(){
    pwm_init();
    enable_pin_init();
    periodic_speed_update_func_init();
    encoder_init();
}

void mqtt_ros_init(){
    ESP_LOGI(TAG, "[MQTT] Startup..");
    ESP_LOGI(TAG, "[MQTT] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[MQTT] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());


    periodic_feedback_update_func_init();
    mqtt_app_start();
}

void app_main(void)
{
    //start the pwm signals

    //start mqtt
    mqtt_ros_init();
    motor_init();
    xTaskCreatePinnedToCore(core1_tasks, "core1_tasks", 1024*4,NULL,1,NULL,1);
    //ledc config for pwm
    while (1) {
        // The rotation direction is controlled by inverting the hall sensor value
        printf("The hall state is %lu \n", (unsigned long)hall_sensor_value);
        printf("The duty value is %lu \n", (unsigned long)duty);
        printf("Encoder value : %d\n", encoder->get_counter_value(encoder));
        vTaskDelay(1000/portTICK_RATE_MS);
    }
}
