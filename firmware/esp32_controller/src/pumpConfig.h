

#ifndef PUMP_CONFIG_H
#define PUMP_CONFIG_H

#include <ReadWriteConfigFile.h>

#include "PumpController.h"

// Example: application-specific config
class PumpConfig : public ReadWriteConfig
{
public:
    explicit PumpConfig(const char *file, size_t pumpCount) : ReadWriteConfig(file), pumpCount(pumpCount) {}

protected:
    void set_defaults(JsonDocument &doc) override
    {
        doc.clear();
        JsonArray pumpArray = doc["pumps"].to<JsonArray>();
        // ESP_LOGI("", "pumpCount: %i", pumpCount);
        for (int i = 0; i < pumpCount; i++)
        {
            JsonObject pump = pumpArray.add<JsonObject>();
            pump["pumpID"] = i + 1;
            pump["mlPerMin"] = 104.f;
        }

        ESP_LOGI("", "Default configuration set");
    }

private:
    size_t pumpCount;
};

#endif // PUMP_CONFIG_H