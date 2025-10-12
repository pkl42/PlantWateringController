

#ifndef PLANT_CONFIG_H
#define PLANT_CONFIG_H

#include <ReadWriteConfigFile.h>
#include "PlantTypes.h"

// Example: application-specific config
class PlantControllerConfig : public ReadWriteConfig
{
public:
    explicit PlantControllerConfig(const char *file) : ReadWriteConfig(file) {}

protected:
    Plant defaultPlants[NUM_PLANTS] = {
        {{1, 1, 5, 445., 30, 35, 1}, {{INVALID_FLOAT, 0}, {0, 0}, 0, 0, 0}},
        {{2, 2, 6, 445., 35, 30, 0}, {{INVALID_FLOAT, 0}, {0, 0}, 0, 0, 0}},
        {{VALUE_NOT_SET, 3, VALUE_NOT_SET, 445., 35, 30, 0}, {{INVALID_FLOAT, 0}, {0, 0}, 0, 0, 0}},
        {{VALUE_NOT_SET, 4, VALUE_NOT_SET, 800., 35, 30, 0}, {{INVALID_FLOAT, 0}, {0, 0}, 0, 0, 0}},
        {{VALUE_NOT_SET, 5, VALUE_NOT_SET, 800., 35, 30, 0}, {{INVALID_FLOAT, 0}, {0, 0}, 0, 0, 0}},
        {{VALUE_NOT_SET, 6, VALUE_NOT_SET, 800., 35, 30, 0}, {{INVALID_FLOAT, 0}, {0, 0}, 0, 0, 0}}};

    void set_defaults(JsonDocument &doc) override
    {
        doc.clear();
        doc["distEmptyCM"] = 18.5;
        doc["distFullCM"] = 2;
        doc["controlMode"] = 0; // 1=AUTO,0=MANUAL
        doc["enablePumpOnNightHours"] = 0;
        doc["sensorReadIntervalSeconds"] = 10UL;

        doc["sleepIntervalSeconds"] = 0UL;
        doc["sleepTimeRemoteSensors"] = 0UL;
        doc["publishIntervalSeconds"] = 60UL;

        doc["minWateringPCT"] = 10.f; //
        doc["maxWateringPCT"] = 30.f; //
        // not sure if needed, but i could be that the soil does not soak the spotted watering as desired
        // const float STAGE_FRACTION = 1.0f;                            // dose 30% of computed need per stage
        doc["absorptionWaitSec"] = 5 * 60; // 5 minutes const unsigned long ABSORPTION_WAIT_MS = 5UL * 60UL * 1000UL; // 5 minutes

        JsonArray plantsArray = doc["plants"].to<JsonArray>();

        for (int i = 0; i < NUM_PLANTS; i++)
        {
            JsonObject plantObj = plantsArray.add<JsonObject>();
            // JsonObject plantObj = plantsArray.createNestedObject();
            //  Default config
            JsonObject cfg = plantObj["config"].to<JsonObject>();

            // JsonObject cfg = plantObj.createNestedObject("config");
            cfg["pumpPort"] = defaultPlants[i].config.pumpPort;
            cfg["sensorPort"] = defaultPlants[i].config.sensorPort;
            cfg["sensorID"] = defaultPlants[i].config.sensorID;
            cfg["potVolume_cm3"] = defaultPlants[i].config.potVolume_cm3;
            cfg["fieldCapacity_pct"] = defaultPlants[i].config.fieldCapacity_pct;
            cfg["targetMoisture_pct"] = defaultPlants[i].config.targetMoisture_pct;

            cfg["autoMode"] = defaultPlants[i].config.autoMode;

            JsonObject state = plantObj["state"].to<JsonObject>();

            // JsonObject state = plantObj.createNestedObject("state");
            JsonObject lastMoisture = state["lastMoisture"].to<JsonObject>();
            lastMoisture["percent"] = defaultPlants[i].state.lastMoisture.percent;
            lastMoisture["timestamp"] = defaultPlants[i].state.lastMoisture.timestamp;

            JsonObject lastWatering = state["lastWatering"].to<JsonObject>();
            lastWatering["doseMl"] = defaultPlants[i].state.lastWatering.doseMl;
            lastWatering["timestamp"] = defaultPlants[i].state.lastWatering.timestamp;

            state["nextRecheckAtEpoch"] = defaultPlants[i].state.nextRecheckAtEpoch;
            state["countTNA"] = defaultPlants[i].state.countTNA;
            state["statusCode"] = 0;
        }

        ESP_LOGI("", "Default configuration set");
    }
};

#endif // PLANT_CONFIG_H