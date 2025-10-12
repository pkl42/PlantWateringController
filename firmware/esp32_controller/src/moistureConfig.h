
#ifndef MOISTURE_CONFIG_H
#define MOISTURE_CONFIG_H

#include <ReadWriteConfigFile.h>
#include "MoistureController.h"
#include "mqttHelper.h"

#define NUM_SENSORS 20
// Example: application-specific config
class MoistureConfig : public ReadWriteConfig
{
public:
    explicit MoistureConfig(const char *file) : ReadWriteConfig(file) {}

protected:
    void set_defaults(JsonDocument &doc) override
    {
        doc.clear();
        doc["lifeTimeSensorDataSec"] = 120;
        doc["sleepTimeRemoteSensors"] = 0;

        JsonArray sensorArray = doc["sensors"].to<JsonArray>();
        // JsonArray rawDry = doc["rawDry"].to<JsonArray>();
        // JsonArray rawWet = doc["rawWet"].to<JsonArray>();
        //  Default calibration data for each sensor
        int defaultDry[NUM_SENSORS / 2] = {3123, 3060, 3113, 3133, 3163, 3151, 3187, 3182, 3167, 3195};
        int defaultWet[NUM_SENSORS / 2] = {1006, 935, 996, 960, 985, 1003, 1017, 978, 974, 1000};

        for (int i = 0; i < sensorCount / 2; i++)
        {
            // rawDry.add(defaultDry[i]);
            // rawWet.add(defaultWet[i]);

            // If you also want structured sensor configs, you can do:
            JsonObject thisSensor = sensorArray.add<JsonObject>();
            thisSensor["sensorID"] = i + 1;
            thisSensor["sensorName"] = String("local_") + String(i + 1);
            thisSensor["dry"] = defaultDry[i];
            thisSensor["wet"] = defaultWet[i];
            thisSensor["pin"] = -1;           // or assign real pins
            thisSensor["connType"] = "LOCAL"; // or whatever enum/string mapping
        }
        for (int i = sensorCount / 2; i < sensorCount; i++)
        {
            JsonObject thisSensor = sensorArray.add<JsonObject>();
            thisSensor["sensorID"] = i + 1;
            thisSensor["sensorName"] = String("MQTT_CLIENT_ID_") + String(i + 1);
            thisSensor["dry"] = INVALID_INT;
            thisSensor["wet"] = INVALID_INT;
            thisSensor["pin"] = -1;          // or assign real pins
            thisSensor["connType"] = "MQTT"; // or whatever enum/string mapping
        }

        ESP_LOGI("", "Default configuration set");
    }

private:
    size_t sensorCount = NUM_SENSORS;
};

#endif // MOISTURE_CONFIG_H