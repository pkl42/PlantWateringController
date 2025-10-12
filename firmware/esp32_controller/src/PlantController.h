/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PLANT_CONTROLLER_H
#define PLANT_CONTROLLER_H

// Number of plants (moisture sensors + pumps)
#define NUM_PLANTS 6

#include <WiFi.h>
#include "mqttHelper.h"
#include "PumpController.h"
#include "MoistureController.h"
#include "UltrasonicSensor.h"

#include "plantControllerConfig.h"
#include "PlantTypes.h"
#include "TimeZone.h"

constexpr int RESERVOIR_TRIG_PIN = 9;
constexpr int RESERVOIR_ECHO_PIN = 10;
constexpr int RESERVOIR_POWER_PIN = 15;

class PlantController
{
public:
    PlantController(TimeZone &timezone);

    void init(const char *mqtt_broker, uint16_t mqtt_port);
    boolean connectToMQTTBroker();
    void subscribeCmndTopics(bool forceFlag = false);

    void run();
    void readSensors();
    void controlPlants();
    void controlNow();

    void loadSetupFromConfig(JsonDocument &doc);
    void updateStateToConfig(JsonDocument &doc);

private:
    unsigned long lastPublishSec = 0;
    unsigned long lastSensorReadSec = 0;
    const char *TAG = "PlantController";

    // ---------- Plant Configuration ----------
    PlantControllerConfig plantControllerConfig; // declare without parameters
    JsonDocument plant_config_doc;
    TimeZone &tz;

    Plant plants[NUM_PLANTS]; // predefined in plantControllerConfig.h

    // ---------- Staged watering control ----------

    unsigned long absorptionWaitSec = 5UL * 60UL; // 5 minutes
    float minWateringPCT = 10.f;
    float maxWateringPCT = 30.f;
    uint8_t maxCountTNA = 3; // max. number where target moisture not reached in series

    // ---------- Moisture Sensor config and calibration ----------
    // ADC pins for moisture sensors (GPIOs must be ADC-capable)
    std::vector<int> moisturePins = {36, 39, 34, 35, 32, 33};
    MoistureController moistureController;

    // ---------- Ultrasonic Sensor ----------
    // HC-SR04 reservoir level
    // Power gating for unltrasonic sensors (5V switched via MOSFET)
    UltrasonicSensor reservoir;

    // ---------- Pump Configuration ----------
    std::vector<int> pumpPins = {13, 12, 14, 27, 26, 25};
    PumpController pumps;

    // Network config

    const char *MQTT_USER = "";
    const char *MQTT_PASS = "";
    const char *MQTT_CLIENT_ID_PREFIX = "PLANT_"; // as 6 digits of MAC are used in mqttClientID, take care <17 char here

    // Major Topics

    const char *TOPIC_CONTROLLER = "CONTROLLER"; // state +/<idx> + cmd
    const char *TOPIC_PLANT = "PLANT";           // state +/<idx> + cmd
    const char *TOPIC_MOISTURE = "MOISTURE";     // state +/<idx>
    const char *TOPIC_RESERVOIR = "RESERVOIR";   // state
    const char *TOPIC_PUMP = "PUMP";             // cmd +/<idx>

    const char *TOPIC_SLEEPING_STATE = "SLEEPING";     // state
    const char *TOPIC_CMD_CONTROL_NOW = "CONTROL_NOW"; // cmd
    const char *TOPIC_CMD_RESTART = "RESTART";         // cmd
    const char *TOPIC_CMD_DEL_CFG = "DELETE_CFG";      // cmd
    const char *TOPIC_CMD_ESTOP = "ESTOP";             // cmd

    MqttDispatcher mqttDispatcher;

    // Timings
    const unsigned long MQTT_WAIT_WINDOW_MS = 8000UL;
    RTC_DATA_ATTR static unsigned long sleepIntervalSeconds;

    unsigned long publishIntervalSeconds = 60UL; // default publish interval for tele topics when awake
    unsigned long sensorReadIntervalSeconds = 10UL;

    unsigned int sleeping_state = 0;

    unsigned int control_mode = 1; // 0=manual, 1=automatic (default)
    unsigned int enablePumpOnNightHours = 0;

    // Wi-Fi / MQTT
    WiFiClient espClient;
    PubSubClient mqttClient;
    bool mqttLostFlag = true;

    // Private methods

    bool connectWiFi();
    bool connectMQTT();

    void processMQTTControllerMessage(const std::vector<String> &topicParts, const String &payload);
    void processMQTTPlantMessage(const std::vector<String> &topicParts, const String &payload);

    void publishPlantConfiguration(const int plantIdx = INVALID_INT);
    void publishPlantTelemetry(const int plantIdx = INVALID_INT);
    void publishControllerState();
    //    void publishControllerTelemetry(void); // 20250922 currently no need to implement

    void publishUltrasonicState(void);
    void publishUltrasonicTelemetry(void);

    // Control logic
    void controlPlant(int plantIdx, bool forceFlag = false);
    float computeWaterNeedMl(const PlantConfig &pc, float currentMoisture);
};

#endif
