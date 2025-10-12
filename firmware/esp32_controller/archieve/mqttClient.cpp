#include <WiFi.h>
#include <mqtt_client.h>
#include "mqttClient.h"
#include "esp_log.h"

static const char *TAG = "mqttClient";

static void mqtt_initialize(const JsonDocument &config_doc)
{
    // Define the MQTT configuration (without event_handle)
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .hostname = config_doc["mqtt_broker_ip"].as<const char *>(),
                .transport = MQTT_TRANSPORT_OVER_TCP,
                .path = NULL,
                .port = 1883,
            },
            .verification = {.certificate = NULL, .certificate_len = 0, .skip_cert_common_name_check = false, .common_name = NULL}},
        .credentials = {.username = "", .client_id = NULL, .set_null_client_id = true, .authentication = {.password = ""}}};

    if (WiFi.getMode() != WIFI_MODE_AP)
    {
        // Initialize MQTT client
        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

        // Register your event handler separately
        esp_mqtt_client_register_event(
            mqtt_client,
            (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, // Catch all events
            mqtt_event_handler,                    // Your existing handler function
            NULL                                   // Optional argument (can be NULL)
        );

        // Start the MQTT client
        esp_mqtt_client_start(mqtt_client);
    }
}

// MQTT event handler callback function
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    esp_mqtt_error_codes_t *error_handle_struct = event->error_handle;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "your/topic", 0);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // Example: print received data
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (error_handle_struct)
        {
            ESP_LOGI(TAG, "connect_return_code %d",
                     error_handle_struct->connect_return_code);
        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

MQTT_PublishResult mqtt_publish(JsonDocument &state, const char *topic, int qos = 1, int retain = 0)
{
    if (WiFi.getMode() == WIFI_MODE_AP)
    {
        ESP_LOGW(TAG, "WiFi in AP mode, skipping MQTT publish");
        return MQTT_PublishResult::WIFI_ERR;
    }

    size_t jsonSize = measureJson(state) + 1; // +1 for null terminator
    ESP_LOGI(TAG, "JSON size: %u", jsonSize);

    constexpr size_t STACK_BUFFER_SIZE = 512;
    char *payload = nullptr;
    bool useHeap = false;

    if (jsonSize <= STACK_BUFFER_SIZE)
    {
        // Use stack buffer
        char stackBuffer[STACK_BUFFER_SIZE];
        payload = stackBuffer;
    }
    else
    {
        // Use heap allocation for large JSON
        payload = (char *)malloc(jsonSize);
        if (!payload)
        {
            ESP_LOGE(TAG, "Memory allocation failed for payload");
            return MQTT_PublishResult::ALLOC_ERR;
        }
        useHeap = true;
    }

    size_t written = serializeJson(state, payload, jsonSize);
    if (written == 0)
    {
        ESP_LOGE(TAG, "Failed to serialize JSON");
        if (useHeap)
            free(payload);
        return MQTT_PublishResult::SERIALIZE_ERR;
    }

    ESP_LOGI(TAG, "Serialized JSON length: %u", written);
    int msgId = esp_mqtt_client_publish(mqtt_client, topic, payload, written, qos, retain);
    ESP_LOGI(TAG, "Publish Return: %i", msgId);

    if (useHeap)
        free(payload);

    return (msgId > 0) ? MQTT_PublishResult::MSG_ID : MQTT_PublishResult::OK;
}
