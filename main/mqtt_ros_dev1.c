#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

// #define CONFIG_BROKER_URL_FROM_STDIN 1

#define EXO_ESP_ID "RIGHT_THIGH"
static const char *TAG = "MQTT_EXAMPLE";

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
    esp_mqtt_client_handle_t client = event->client;
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
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

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

    mqtt_app_start();
}