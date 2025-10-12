#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H
#include <ArduinoJson.h>

/* Usage Example
StaticJsonDocument<256> doc;
doc["pumpid"] = pumpid;
doc["control"] = peristaltik_control[pumpid].active;

MQTT_PublishResult result = mqtt_publish(doc, "mydevice/pump/state", 1, 0);

switch (result)
{
    case MQTT_PublishResult::OK:
        ESP_LOGI(TAG, "Publish successful (QoS 0)");
        break;
    case MQTT_PublishResult::MSG_ID:
        ESP_LOGI(TAG, "Publish successful, message ID returned");
        break;
    case MQTT_PublishResult::WIFI_ERR:
        ESP_LOGE(TAG, "Publish failed: Wi-Fi in AP mode");
        break;
    case MQTT_PublishResult::TOO_LARGE:
        ESP_LOGE(TAG, "Publish failed: payload too large");
        break;
    case MQTT_PublishResult::ALLOC_ERR:
        ESP_LOGE(TAG, "Publish failed: memory allocation error");
        break;
    case MQTT_PublishResult::SERIALIZE_ERR:
        ESP_LOGE(TAG, "Publish failed: JSON serialization error");
        break;
}
*/

// MQTT

enum class MQTT_PublishResult : int
{
    OK = 0,            // Message sent successfully (QoS 0)
    MSG_ID = 1,        // Message ID returned (QoS > 0)
    WIFI_ERR = -1,     // Wi-Fi in AP mode
    TOO_LARGE = -2,    // JSON payload too large
    ALLOC_ERR = -3,    // Memory allocation failed
    SERIALIZE_ERR = -4 // JSON serialization failed
};

static esp_mqtt_client_handle_t mqtt_client;

static void mqtt_initialize(const JsonDocument &config_doc);

// MQTT event handler callback function
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data);

MQTT_PublishResult mqtt_publish(JsonDocument &state, const char *topic, int qos = 1, int retain = 0);

#endif // MQTT_CLIENT_H