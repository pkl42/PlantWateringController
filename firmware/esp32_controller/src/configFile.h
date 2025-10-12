#ifndef MY_APP_CONFIG_H
#define MY_APP_CONFIG_H

#include <ReadWriteConfigFile.h>
#include "esp_log.h"

// Example: application-specific config
class MyAppConfig : public ReadWriteConfig
{
public:
    explicit MyAppConfig(const char *file) : ReadWriteConfig(file) {}

protected:
    const char *TAG = "MyAppConfig";
    void set_defaults(JsonDocument &doc) override
    {
        doc.clear();
        doc["ap_ssid"] = "Plant Controller";
        doc["ap_password"] = "";
        doc["ln_ssid"] = "to be defined"; // "to be defined"
        doc["ln_password"] = "basicraw";  // "basicraw"
        doc["mqtt_broker_ip"] = "192.168.0.xxx";
        doc["mqtt_broker_port"] = "1883";
        ESP_LOGI(TAG, "Default configuration set");
    }
};

#endif // MY_APP_CONFIG_H