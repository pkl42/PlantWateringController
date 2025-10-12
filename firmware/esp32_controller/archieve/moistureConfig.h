#ifndef MOISTURE_CONFIG_H
#define MOISTURE_CONFIG_H

#include <ReadWriteConfigFile.h>

#include "MoistureController.h"

// Example: application-specific config
class MoistureConfig : public ReadWriteConfig
{
public:
    explicit MoistureConfig(const char *file, size_t sensorCount) : ReadWriteConfig(file), sensorCount(sensorCount) {}

protected:
    void set_defaults(JsonDocument &doc) override
    {
        doc.clear();
        JsonArray rawDry = doc["rawDry"].to<JsonArray>();
        JsonArray rawWet = doc["rawWet"].to<JsonArray>();
        // if (arr.isNull())
        // {
        //     arr = doc.createNestedArray("pumpMlPerSec");
        // }

        for (int i = 0; i < sensorCount; i++)
        {
            rawDry.add(3147);
            rawWet.add(985);
        }

        ESP_LOGI("", "Default configuration set");
    }

private:
    size_t sensorCount;
};

#endif // MOISTURE_CONFIG_H