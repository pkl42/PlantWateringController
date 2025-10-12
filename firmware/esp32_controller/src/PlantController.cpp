/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "PlantController.h"

#include <DualWifiMode.h>
#include <esp32-hal-adc.h>

unsigned long PlantController::sleepIntervalSeconds = 0UL; // 1800UL;

PlantController::PlantController(TimeZone &timezone)
    : tz(timezone),
      mqttClient(espClient),
      pumps(pumpPins),
      moistureController(moisturePins, timezone),
      reservoir(RESERVOIR_TRIG_PIN, RESERVOIR_ECHO_PIN, RESERVOIR_POWER_PIN),
      plantControllerConfig("/plantConfig.cfg"),
      mqttDispatcher(mqttClient)
{
}

void PlantController::init(const char *mqtt_broker, uint16_t mqtt_port)
{
    ESP_LOGI(TAG, "init Broker: %s : %u", mqtt_broker, mqtt_port);
    mqttClient.setServer(mqtt_broker, mqtt_port);
    mqttClient.setCallback([this](char *topic, byte *payload, unsigned int length)
                           { this->mqttDispatcher.mqttCallback(topic, payload, length); });

    mqttClient.setBufferSize(512);

    String mqttClientID = buildClientID(String(MQTT_CLIENT_ID_PREFIX), getDeviceID());
    mqttDispatcher.init(mqttClientID);

    pumps.init(mqttClient, mqttClientID);

    mqttDispatcher.registerHandler(
        mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_PUMP, -1, "#"),
        [this](const std::vector<String> &topic, const String &payload)
        {
            this->pumps.processMQTTMessage(topic, payload);
        });

    //

    moistureController.init(mqttClient, mqttClientID);

    mqttDispatcher.registerHandler(
        mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_MOISTURE, -1, "#"),
        [this](const std::vector<String> &topic, const String &payload)
        {
            this->moistureController.processMQTTMessage(topic, payload);
            // this->pumps->processMQTTMessage(topic, payload); // if pumps is a pointer
        });
    // when working with mqtt moisture sensors also the moistureController will handle these topics
    mqttDispatcher.registerHandler(
        mqttTopicBuilder("+", CLASS_TELE, TOPIC_MOISTURE, -1, "#"),
        [this](const std::vector<String> &topic, const String &payload)
        {
            this->moistureController.processMQTTMessage(topic, payload);
        });

    // read plant configuration from file
    // plantControllerConfig.remove();
    plant_config_doc = plantControllerConfig.read_config();
    JsonArray plant_arr = plant_config_doc["plants"];
    if (plant_arr.isNull() || plant_arr.size() != NUM_PLANTS)
    {
        ESP_LOGW(TAG, "plant_config_doc: Config invalid or missing, resetting to defaults");
        plantControllerConfig.remove();                         // delete corrupt file
        plant_config_doc = plantControllerConfig.read_config(); // re-create defaults
        plant_arr = plant_config_doc["plants"];
    }

    loadSetupFromConfig(plant_config_doc);
    // initialize used sensors
    for (uint8_t plantIdx = 0; plantIdx < NUM_PLANTS; ++plantIdx)
    {
        // Measure moisture
        ESP_LOGI("initSensor", "plantIdx %i, sensorID %i, sensorPort %i", plantIdx, plants[plantIdx].config.sensorID, plants[plantIdx].config.sensorPort);
        moistureController.initSensor(plants[plantIdx].config.sensorID, plants[plantIdx].config.sensorPort);
    }

    mqttDispatcher.registerHandler(
        mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_PLANT, -1, "#"),
        [this](const std::vector<String> &topic, const String &payload)
        {
            processMQTTPlantMessage(topic, payload);
        });

    // read and assign controller configuration

    reservoir.init(plant_config_doc["distEmptyCM"], plant_config_doc["distFullCM"]);

    control_mode = plant_config_doc["controlMode"]; // 20250912 deactivated
    enablePumpOnNightHours = plant_config_doc["enablePumpOnNightHours"];

    sleepIntervalSeconds = plant_config_doc["sleepIntervalSeconds"];
    publishIntervalSeconds = plant_config_doc["publishIntervalSeconds"];

    minWateringPCT = plant_config_doc["minWateringPCT"];
    maxWateringPCT = plant_config_doc["maxWateringPCT"];
    absorptionWaitSec = plant_config_doc["absorptionWaitSec"];

    sleeping_state = 0;
    bool pubStatus = mqttClient.publish(mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_SLEEPING_STATE).c_str(), String(sleeping_state).c_str());

    mqttDispatcher.registerHandler(
        mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_CONTROLLER, -1, "#"),
        [this](const std::vector<String> &topic, const String &payload)
        {
            processMQTTControllerMessage(topic, payload);
        });

    mqttDispatcher.registerHandler(
        mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_RESERVOIR, -1, "#"),
        [this](const std::vector<String> &topic, const String &payload)
        {
            processMQTTControllerMessage(topic, payload);
        });

    tz.maybeResync(24); // only sync if >24h since last s
}

void PlantController::loadSetupFromConfig(JsonDocument &doc)
{
    ESP_LOGI(TAG, "loadSetupFromConfig");

    JsonArray plantsArray = doc["plants"];

    if (plantsArray.isNull() || plantsArray.size() != NUM_PLANTS)
    {
        ESP_LOGE(TAG, "plants array missing or wrong size");
        return; // or handle recovery
    }

    int i = 0;

    for (JsonObject plantObj : plantsArray)
    {
        if (i >= NUM_PLANTS)
            break;
        if (plantObj.isNull())
        {
            ESP_LOGW(TAG, "Plant object %d is null, skipping", i);
            i++;
            continue;
        }
        JsonObject cfg = plantObj["config"];
        if (cfg.isNull())
        {
            ESP_LOGW(TAG, "Config missing for plant %d, skipping", i);
            i++;
            continue;
        }
        plants[i].config.pumpPort = cfg["pumpPort"] | VALUE_NOT_SET; // use default if missing
        plants[i].config.sensorPort = cfg["sensorPort"] | VALUE_NOT_SET;
        plants[i].config.sensorID = cfg["sensorID"] | -1;
        plants[i].config.potVolume_cm3 = cfg["potVolume_cm3"] | 500.0;
        plants[i].config.fieldCapacity_pct = cfg["fieldCapacity_pct"] | 40.0;
        plants[i].config.targetMoisture_pct = cfg["targetMoisture_pct"] | 70.0;
        plants[i].config.autoMode = cfg["autoMode"] | 0;

        JsonObject state = plantObj["state"];
        JsonObject lm = state["lastMoisture"].as<JsonObject>();
        if (!lm.isNull())
        {
            plants[i].state.lastMoisture.percent = lm["percent"] | INVALID_FLOAT;
            plants[i].state.lastMoisture.timestamp = lm["timestamp"] | 0UL;
        }
        ESP_LOGI(TAG, "plants[i].state.lastMoisture.percent: %f", plants[i].state.lastMoisture.percent);

        // --- lastWatering (nested PumpData) ---
        JsonObject lw = state["lastWatering"].as<JsonObject>();
        if (!lw.isNull())
        {
            plants[i].state.lastWatering.doseMl = lw["doseMl"] | 0.0f;
            plants[i].state.lastWatering.timestamp = lw["timestamp"] | 0UL;
        }

        plants[i].state.nextRecheckAtEpoch = state["nextRecheckAtEpoch"] | 0UL;
        plants[i].state.countTNA = state["countTNA"] | 0UL;
        plants[i].state.statusCode = state["statusCode"] | 0;

        i++;
    }
    ESP_LOGI(TAG, "loadPlantsFromConfig done");
}

void PlantController::updateStateToConfig(JsonDocument &doc)
{
    for (uint8_t plantIdx = 0; plantIdx < NUM_PLANTS; ++plantIdx)
    {
        doc["plants"][plantIdx]["state"]["lastMoisture"]["percent"] = plants[plantIdx].state.lastMoisture.percent;
        doc["plants"][plantIdx]["state"]["lastMoisture"]["timestamp"] = plants[plantIdx].state.lastMoisture.timestamp;
        doc["plants"][plantIdx]["state"]["lastWatering"]["doseMl"] = plants[plantIdx].state.lastWatering.doseMl;
        doc["plants"][plantIdx]["state"]["lastWatering"]["timestamp"] = plants[plantIdx].state.lastWatering.timestamp;
        doc["plants"][plantIdx]["state"]["nextRecheckAtEpoch"] = plants[plantIdx].state.nextRecheckAtEpoch;
        doc["plants"][plantIdx]["state"]["countTNA"] = plants[plantIdx].state.countTNA;

        doc["plants"][plantIdx]["state"]["statusCode"] = plants[plantIdx].state.statusCode;
    }
    plantControllerConfig.write_config(&doc); // write plant status
}

boolean PlantController::connectToMQTTBroker()
{

    WiFiModeStatus wifi_status = startDualWiFi();

    switch (wifi_status)
    {
    case WiFiModeStatus::STA_CONNECTED:
        // ESP_LOGI(TAG, "connectToMQTTBroker: Safe to enter deep sleep after work is done");
        break;

    case WiFiModeStatus::AP:
        // ESP_LOGI(TAG, "connectToMQTTBroker: Running in AP mode, stay awake");
        sleepIntervalSeconds = 0;
        // check if web server is running
        return false; // in AP-mode no connection to MQTT Broker possible
        break;

    case WiFiModeStatus::STA_FAILED:
        if (sleepIntervalSeconds > 0)
        {
            ESP_LOGI(TAG, "connectToMQTTBroker: Station mode failed, Sleeping...");
            updateStateToConfig(plant_config_doc);
            esp_sleep_enable_timer_wakeup((uint64_t)sleepIntervalSeconds * 1000000ULL);
            Serial.flush();
            esp_deep_sleep_start();
        }
        else
        {
            ESP_LOGI(TAG, "connectToMQTTBroker: Retrying Wi-Fi in 5 seconds...");
            delay(5000);
            mqttLostFlag = true;
            return false;
        }
        break;
    }

    if (!connectMQTT())
    {
        ESP_LOGI(TAG, "connectToMQTTBroker: MQTT connect failed.");
        if (sleepIntervalSeconds > 0)
        {
            ESP_LOGI(TAG, "connectToMQTTBroker: Sleeping...");
            esp_sleep_enable_timer_wakeup((uint64_t)sleepIntervalSeconds * 1000000ULL);
            Serial.flush();
            esp_deep_sleep_start();
        }
        else
        {
            ESP_LOGI(TAG, "connectToMQTTBroker: Retrying MQTT in 5 seconds...");
            delay(5000);
            mqttLostFlag = true;
            return false;
        }
    }

    mqttDispatcher.mqttSelfTest();

    if (mqttDispatcher.subscriptionLost())
    {
        mqttLostFlag = true;
    }

    if (mqttLostFlag)
    {
        subscribeCmndTopics(true);
        mqttLostFlag = false;
    }

    return true;
}

void PlantController::subscribeCmndTopics(bool forceFlag)
{
    static bool subscribed = false;
    // Subscribe to pump control & settings

    moistureController.subscribeSensorTopics(forceFlag);

    if (!subscribed || forceFlag)
    {
        ESP_LOGI(TAG, "subscribe command Topics...");

        String mqttClientID = mqttDispatcher.getClientID();
        mqttClient.subscribe(mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_PUMP, -1, "#").c_str(), 0);
        mqttClient.subscribe(mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_MOISTURE, -1, "#").c_str(), 1);
        mqttClient.subscribe(mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_PLANT, -1, "#").c_str(), 1);
        mqttClient.subscribe(mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_RESERVOIR, -1, "#").c_str(), 1);
        mqttClient.subscribe(mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_CONTROLLER, -1, "#").c_str(), 1);

        mqttClient.subscribe(mqttTopicBuilder(mqttClientID, CLASS_CMD, TOPIC_PING, -1, "").c_str(), 1);

        subscribed = true;
    }
}

void PlantController::run()
{
    static bool first_call = true;

    // Auto-publish based on interval
    if (millis() / 1000UL - lastPublishSec >= publishIntervalSeconds)
    {
        publishPlantTelemetry();
        publishUltrasonicTelemetry();
        lastPublishSec = millis() / 1000UL;
    }

    // First-call MQTT wait
    if ((sleepIntervalSeconds > 0) && first_call)
    {
        unsigned long startWait = millis();
        while (millis() - startWait < MQTT_WAIT_WINDOW_MS)
        {
            mqttClient.loop();
            vTaskDelay(pdMS_TO_TICKS(10)); // yield to other tasks
        }
        first_call = false;
    }

    mqttClient.loop();

    if (millis() / 1000UL - lastSensorReadSec >= sensorReadIntervalSeconds)
    {
        readSensors();
        lastSensorReadSec = millis() / 1000UL;
    }

    // Control loop for each plant
    // if (control_mode == 1) 20250912 deactivated
    if (lastSensorReadSec != 0) // ensure that sensors are read at least once
    {
        controlPlants();
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // short idle delay

    // Wait until no pump jobs are queued or running before deep sleep
    if ((sleepIntervalSeconds > 0) && pumps.isQueueEmpty() && !pumps.jobRunning)
    {
        sleeping_state = 1;
        bool pubStatus = mqttClient.publish(mqttTopicBuilder(mqttDispatcher.getClientID(), CLASS_STATE, TOPIC_SLEEPING_STATE).c_str(), String(sleeping_state).c_str());

        vTaskDelay(pdMS_TO_TICKS(200));

        esp_sleep_enable_timer_wakeup((uint64_t)sleepIntervalSeconds * 1000000ULL);
        ESP_LOGI(TAG, "Going to deep sleep for %s seconds.", String(sleepIntervalSeconds));
        Serial.flush();
        esp_deep_sleep_start();
    }
}

void PlantController::controlNow()
{
    unsigned long now = millis();
    for (int i = 0; i < NUM_PLANTS; i++)
    {
        if (plants[i].config.pumpPort != VALUE_NOT_SET)
        {
            plants[i].state.nextRecheckAtEpoch = now / 1000UL;
        }
    }
    controlPlants();
}

void PlantController::controlPlants()
{
    if (enablePumpOnNightHours == 0)
    {
        tm t = tz.getLocalTimeStruct();
        if (t.tm_hour < 6 && t.tm_hour >= 20)
        {
            ESP_LOGI(TAG, "Nighttime, skip watering as enablePumpOnNightHours==0");
            return;
        }
    }
    for (uint8_t i = 0; i < NUM_PLANTS; ++i)
    {
        controlPlant(i);
    }
}

void PlantController::readSensors()
{
    for (uint8_t plantIdx = 0; plantIdx < NUM_PLANTS; ++plantIdx)
    {
        PlantState &st = plants[plantIdx].state;
        // Measure moisture
        st.lastMoisture = moistureController.readPercent(plants[plantIdx].config.sensorID);
    }
}

// ====== Private Methods ======

bool PlantController::connectMQTT()
{
    if (mqttClient.connected())
        return true;
    // mqttClientID = buildClientID(String(MQTT_CLIENT_ID_PREFIX), getDeviceID());
    mqttLostFlag = true;

    if (strlen(MQTT_USER) > 0)
    {
        return mqttClient.connect(mqttDispatcher.getClientID().c_str(), MQTT_USER, MQTT_PASS);
    }
    else
    {
        return mqttClient.connect(mqttDispatcher.getClientID().c_str());
    }
}

void PlantController::processMQTTControllerMessage(const std::vector<String> &topicParts, const String &payload)
{
    bool update_flag = false;
    int topicSize = topicParts.size();
    if (topicSize < 3)
    {
        ESP_LOGI(TAG, "topicSize need to be minimum 3: %i -> return", topicSize);
        return;
    }

    // when payload empty then state and tele information are published
    if (payload.isEmpty() && topicSize == 3)
    {
        if (topicParts[2] == TOPIC_RESERVOIR)
        {
            publishUltrasonicTelemetry();
        }
        else
        {
            publishControllerState();
            publishUltrasonicState();
            publishUltrasonicTelemetry();
        }
        return;
    }

    // Handle controller commands

    if (topicParts[2] == TOPIC_CONTROLLER)
    {
        // --- Handle commands ---

        if (topicSize == 4)
        {
            // ESP_LOGI("", "topicSize: %i, topicParts[2]: %s, topicParts[3]: %s", topicSize, topicParts[2].c_str(), topicParts[3].c_str());
            if (topicParts[3] == TOPIC_CMD_CONTROL_NOW)
                controlNow();
            if (topicParts[3] == TOPIC_CMD_RESTART)
                ESP.restart();
            if (topicParts[3] == TOPIC_CMD_DEL_CFG)
            {
                plantControllerConfig.remove();
                plant_config_doc = plantControllerConfig.read_config();
                loadSetupFromConfig(plant_config_doc);
            }
            if (topicParts[3] == TOPIC_CMD_ESTOP)
            {
                pumps.stopAndClearQueue();
            }

            return;
        }
    }

    //
    // Handle controller configuration

    JsonDocument doc; // adjust if needed
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
        ESP_LOGE("", "JSON parse error: %s\n", error.c_str());
        return;
    }
    // ESP_LOGI("", "payload: %s", payload.c_str());
    JsonObject obj = doc.as<JsonObject>();

    // --- Handle Ultrasonic Sensor ---
    if (topicParts[2] == TOPIC_RESERVOIR)
    {
        float newFloat = isValidFloat((JsonVariantConst)obj["DISTEMPTYCM"], 0., 100.);

        if (newFloat != INVALID_FLOAT && plant_config_doc["distEmptyCM"].as<float>() != newFloat)
        {
            ESP_LOGI("", "new distEmptyCM: %f", newFloat);
            plant_config_doc["distEmptyCM"] = newFloat;
            reservoir.setDistEmptyCM(newFloat);
            update_flag = true;
        }

        newFloat = isValidFloat((JsonVariantConst)obj["DISTFULLCM"], 0., 100.);

        if (newFloat != INVALID_FLOAT && plant_config_doc["distFullCM"].as<float>() != newFloat)
        {
            plant_config_doc["distFullCM"] = newFloat;

            reservoir.setDistFullCM(newFloat);
            update_flag = true;
        }

        if (update_flag)
        {
            plantControllerConfig.write_config(&plant_config_doc);
        }
        publishUltrasonicState();
        return;
    }

    //
    // Handle controller configuration
    update_flag = false;

    if (topicParts[2] == TOPIC_CONTROLLER)
    {
        // --- Handle sleep interval ---
        unsigned long newLong = isValidInt((JsonVariantConst)obj["SLEEPINTERVAL"], 0, 32768);

        if (newLong != INVALID_INT)
        {

            sleepIntervalSeconds = (newLong >= 10) ? newLong : 0;
            if (plant_config_doc["sleepIntervalSeconds"].as<long>() != sleepIntervalSeconds)
            {
                plant_config_doc["sleepIntervalSeconds"] = sleepIntervalSeconds;
                update_flag = true;
            }
        }

        // --- Handle publishIntervalSeconds ---

        newLong = isValidInt((JsonVariantConst)obj["PUBLISHINTERVALSECONDS"], 0, 32768);

        if (newLong != INVALID_INT)
        {
            publishIntervalSeconds = newLong;
            if (plant_config_doc["publishIntervalSeconds"].as<long>() != publishIntervalSeconds)
            {
                plant_config_doc["publishIntervalSeconds"] = publishIntervalSeconds;
                update_flag = true;
            }
        }

        // --- Handle sensorReadIntervalSeconds ---

        newLong = isValidInt((JsonVariantConst)obj["SENSORREADINTERVALSECONDS"], 0, 32768);

        if (newLong != INVALID_INT)
        {
            sensorReadIntervalSeconds = newLong;
            if (plant_config_doc["SENSORREADINTERVALSECONDS"].as<long>() != sensorReadIntervalSeconds)
            {
                plant_config_doc["SENSORREADINTERVALSECONDS"] = sensorReadIntervalSeconds;
                update_flag = true;
            }
        }

        // --- Handle enablePumpOnNightHours ---
        unsigned int newInt = isValidInt((JsonVariantConst)obj["ENABLEPUMPONNIGHTHOURS"], 0, 1);

        if (newInt != INVALID_INT)
        {
            enablePumpOnNightHours = newInt;
            if (plant_config_doc["enablePumpOnNightHours"].as<u_int>() != enablePumpOnNightHours)
            {
                plant_config_doc["enablePumpOnNightHours"] = enablePumpOnNightHours;
                update_flag = true;
            }
        }

        // --- Handle absorptionWaitSec ---

        newLong = isValidInt((JsonVariantConst)obj["ABSORPTIONWAITSEC"], 60, 32768);

        if (newLong != INVALID_INT)
        {
            absorptionWaitSec = newLong;
            if (plant_config_doc["absorptionWaitSec"].as<long>() != absorptionWaitSec)
            {
                plant_config_doc["absorptionWaitSec"] = absorptionWaitSec;
                update_flag = true;
            }
        }

        // --- Handle targetTolerancePCT ---

        float newFloat = isValidFloat((JsonVariantConst)obj["MINWATERINGPCT"], 0., 30.);

        if (newFloat != INVALID_FLOAT)
        {
            minWateringPCT = newFloat;
            if (plant_config_doc["minWateringPCT"].as<float>() != minWateringPCT)
            {
                plant_config_doc["minWateringPCT"] = minWateringPCT;
                update_flag = true;
            }
        }

        // --- Handle maxWateringPCT ---
        newFloat = isValidInt((JsonVariantConst)obj["MAXWATERINGPCT"], 0., 70.);

        if (maxWateringPCT != INVALID_FLOAT)
        {
            maxWateringPCT = newFloat;
            if (plant_config_doc["maxWateringPCT"].as<float>() != maxWateringPCT)
            {
                plant_config_doc["maxWateringPCT"] = maxWateringPCT;
                update_flag = true;
            }
        }

        // --- Handle control_mode ---
        if (obj["CONTROLMODE"].is<const char *>())
        {
            const char *modeStr = obj["CONTROLMODE"];
            if (strcmp(modeStr, "AUTO") == 0)
            {
                control_mode = 1;
            }
            else if (strcmp(modeStr, "MANUAL") == 0)
            {
                control_mode = 0;
            }
            if (plant_config_doc["controlMode"].as<int>() != control_mode)
            {
                plant_config_doc["controlMode"] = control_mode;
                update_flag = true;
            }
        }

        if (update_flag)
        {
            plantControllerConfig.write_config(&plant_config_doc);
            // JsonDocument test = plantControllerConfig.read_config();
        }
        publishControllerState();
    }
}

void PlantController::processMQTTPlantMessage(const std::vector<String> &topicParts, const String &payload)
{
    // --- Handle plant configuration ---
    bool update_flag = false;
    bool init_sensor_flag = false;
    int topicSize = topicParts.size();
    if (topicSize < 4)
    {
        if (payload.isEmpty())
        {
            publishPlantConfiguration();
            publishPlantTelemetry();
        }
        else
        {
            ESP_LOGI(TAG, "topicSize need to be minimum 3: %i -> return", topicSize);
        }
        return;
    }

    ESP_LOGI(TAG, "plantID: %s", topicParts[3].c_str());
    int plantID = isValidInt(topicParts[3], 1, NUM_PLANTS);
    if (plantID == INVALID_INT)
    {
        ESP_LOGI(TAG, "plantID is invalid -> return");
        return;
    }

    int plantIndex = plantID - 1;

    if (payload.isEmpty())
    {
        publishPlantConfiguration(plantIndex);
        publishPlantTelemetry(plantIndex);
        return;
    }

    JsonDocument doc; // adjust if needed
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
        ESP_LOGE("", "JSON parse error: %s", error.c_str());
        return;
    }
    // ESP_LOGI("", "payload: %s", payload.c_str());
    JsonObject obj = doc.as<JsonObject>();

    int newInt;

    newInt = isValidInt((JsonVariantConst)obj["PUMPPORT"], 1, pumpPins.size());
    if (newInt != INVALID_INT && plants[plantIndex].config.pumpPort != newInt)
    {
        ESP_LOGI("", "pumpPort %i", newInt);
        plants[plantIndex].config.pumpPort = newInt;
        plant_config_doc["plants"][plantIndex]["config"]["pumpPort"] = newInt;
        update_flag = true;
    }

    newInt = isValidInt((JsonVariantConst)obj["SENSORID"], VALUE_NOT_SET, NUM_SENSORS);

    if (newInt != INVALID_INT && plants[plantIndex].config.sensorID != newInt)
    {
        if (plants[plantIndex].config.sensorID != VALUE_NOT_SET)
        {
            ESP_LOGI("", "deleteSensorInstance %i", plants[plantIndex].config.sensorID);
            int oldSensorID = plants[plantIndex].config.sensorID;
            ESP_LOGI("", "deleteSensorInstance oldSensorID %i", oldSensorID);
            moistureController.deleteSensorInstance(oldSensorID);
        }
        ESP_LOGI("", "sensorID %i", newInt);
        plants[plantIndex].config.sensorID = newInt;
        ESP_LOGI(TAG, "new SensorID set: %i", newInt);
        plant_config_doc["plants"][plantIndex]["config"]["sensorID"] = newInt;
        init_sensor_flag = true;
        update_flag = true;
    }

    newInt = isValidInt((JsonVariantConst)obj["SENSORPORT"], VALUE_NOT_SET, pumpPins.size());
    if (newInt != INVALID_INT && plants[plantIndex].config.sensorPort != newInt)
    {
        // ESP_LOGI("", "sensorPort %i", newInt);
        plants[plantIndex].config.sensorPort = newInt;
        plant_config_doc["plants"][plantIndex]["config"]["sensorPort"] = newInt;
        init_sensor_flag = true;
        update_flag = true;
    }

    float newFloat = isValidFloat((JsonVariantConst)obj["POTVOLUME_CM3"], 1., 1000000.);

    if (newFloat != INVALID_FLOAT && plants[plantIndex].config.potVolume_cm3 != newFloat)
    {
        ESP_LOGI("", "potVolume_cm3 %f", newFloat);
        plants[plantIndex].config.potVolume_cm3 = newFloat;
        plant_config_doc["plants"][plantIndex]["config"]["potVolume_cm3"] = newFloat;

        update_flag = true;
    }

    newFloat = isValidFloat((JsonVariantConst)obj["FIELDCAPACITY_PCT"], 0., 100.);

    if (newFloat != INVALID_FLOAT && plants[plantIndex].config.fieldCapacity_pct != newFloat)
    {
        ESP_LOGI("", "fieldCapacity_pct %f", newFloat);
        plants[plantIndex].config.fieldCapacity_pct = newFloat;
        plant_config_doc["plants"][plantIndex]["config"]["fieldCapacity_pct"] = newFloat;

        update_flag = true;
    }

    newFloat = isValidFloat((JsonVariantConst)obj["TARGETMOISTURE_PCT"], 0., 100.);

    if (newFloat != INVALID_FLOAT && plants[plantIndex].config.targetMoisture_pct != newFloat)
    {
        ESP_LOGI("", "targetMoisture_pct %f", newFloat);
        plants[plantIndex].config.targetMoisture_pct = newFloat;
        plant_config_doc["plants"][plantIndex]["config"]["targetMoisture_pct"] = newFloat;

        update_flag = true;
    }

    if (obj["CONTROLMODE"].is<const char *>())
    {

        const char *modeStr = obj["CONTROLMODE"];
        int newInt = -1;
        if (strcmp(modeStr, "ON") == 0)
        {
            newInt = 1;
        }
        else if (strcmp(modeStr, "OFF") == 0)
        {
            newInt = 0;
        }

        newInt = isValidInt(String(newInt), 0, 1);
        if (newInt != INVALID_INT && plants[plantIndex].config.autoMode != newInt)
        {
            plants[plantIndex].config.autoMode = newInt;
            plant_config_doc["plants"][plantIndex]["config"]["autoMode"] = newInt;
            update_flag = true;
        }
    }

    if (update_flag)
    {
        plantControllerConfig.write_config(&plant_config_doc);
    }

    if (init_sensor_flag)
    {
        moistureController.initSensor(plants[plantIndex].config.sensorID, plants[plantIndex].config.sensorPort);
    }

    publishPlantConfiguration(plantIndex);
    return;
}

void PlantController::publishUltrasonicState()
{
    JsonDocument doc;
    doc["distEmptyCM"] = plant_config_doc["distEmptyCM"];
    doc["distFullCM"] = plant_config_doc["distFullCM"];

    String payload;
    serializeJson(doc, payload);

    bool pubStatus = mqttClient.publish(
        mqttTopicBuilder(mqttDispatcher.getClientID(), CLASS_STATE, TOPIC_RESERVOIR).c_str(),
        payload.c_str(),
        true);
    if (!pubStatus)
    {
        ESP_LOGE("", "mqttClient.publish failed %d %s", payload.length(), payload.c_str());
    }
}

// state of plant controller
void PlantController::publishControllerState()
{
    JsonDocument doc;
    doc["controlMode"] = (control_mode == 1 ? "AUTO" : "MANUAL"); // 20250912 deactivated, but keep for emergency stop usage
    doc["enablePumpOnNightHours"] = enablePumpOnNightHours;

    doc["sleepIntervalSeconds"] = sleepIntervalSeconds;
    doc["publishIntervalSeconds"] = publishIntervalSeconds;

    doc["minWateringPCT"] = minWateringPCT;
    doc["maxWateringPCT"] = maxWateringPCT;
    doc["absorptionWaitSec"] = absorptionWaitSec;

    String payload;
    serializeJson(doc, payload);

    bool pubStatus = mqttClient.publish(
        mqttTopicBuilder(mqttDispatcher.getClientID(), CLASS_STATE, TOPIC_CONTROLLER, -1, "").c_str(),
        payload.c_str(),
        true);
    if (!pubStatus)
    {
        ESP_LOGE("", "mqttClient.publish failed %d %s", payload.length(), payload.c_str());
    }
}

void PlantController::publishPlantConfiguration(const int plantIdx)
{
    for (int i = 0; i < NUM_PLANTS; ++i)
    {
        JsonObject cfg = plant_config_doc["plants"][i]["config"];

        if (plantIdx != INVALID_INT && i != plantIdx)
            continue;

        // Build payload using a temporary DynamicJsonDocument
        JsonDocument tempDoc; // adjust size
        JsonObject tempCfg = tempDoc.to<JsonObject>();

        // Copy all fields except autoMode
        for (JsonPair kv : cfg)
        {
            if (strcmp(kv.key().c_str(), "autoMode") != 0)
                tempCfg[kv.key()] = kv.value();
        }

        // Add autoMode as string
        int mode = cfg["autoMode"];
        tempCfg["autoMode"] = (mode == 1) ? "ON" : "OFF";

        // Serialize
        String payload;
        serializeJson(tempCfg, payload);

        bool pubStatus = mqttClient.publish(
            mqttTopicBuilder(mqttDispatcher.getClientID(), CLASS_STATE, TOPIC_PLANT, i + 1, "").c_str(),
            payload.c_str(),
            true);
        if (!pubStatus)
        {
            ESP_LOGE("", "mqttClient.publish failed %d %s", payload.length(), payload.c_str());
        }
    }
}

// ---------- Telemetry ----------

void PlantController::publishPlantTelemetry(const int plantIdx)
{

    moistureController.publishTelemetrie();
    // pumps do not have telemetry

    for (int i = 0; i < NUM_PLANTS; ++i)
    {
        PlantState &st = plants[i].state;
        int plantID = i + 1;
        if (plantIdx != INVALID_INT && i != plantIdx)
        {
            continue;
        }

        JsonDocument doc;
        doc["Time"] = tz.getLocalTimeString();
        doc["plantID"] = plantID;
        doc["lastMoisturePer"] = roundToDecimals(st.lastMoisture.percent, 0);
        doc["lastMoistureEpoch"] = st.lastMoisture.timestamp;

        doc["lastWatering"] = tz.epochToISO8601(st.lastWatering.timestamp);
        doc["lastWateringEpoch"] = st.lastWatering.timestamp;
        doc["lastDoseMl"] = roundToDecimals(st.lastWatering.doseMl, 0);
        doc["nextRecheckAt"] = tz.epochToISO8601(st.nextRecheckAtEpoch);
        doc["nextRecheckAtEpoch"] = st.nextRecheckAtEpoch;
        doc["countTNA"] = st.countTNA;
        doc["statusCode"] = "";

        String payload;
        serializeJson(doc, payload);
        bool pubStatus = mqttClient.publish(
            mqttTopicBuilder(mqttDispatcher.getClientID(), CLASS_TELE, TOPIC_PLANT, plantID).c_str(),
            payload.c_str(),
            true);
        if (!pubStatus)
        {
            ESP_LOGE("", "mqttClient.publish failed %d %s", payload.length(), payload.c_str());
        }
    }
}

void PlantController::publishUltrasonicTelemetry(void)
{
    // --- Reservoir payload ---
    int level = reservoir.readLevelPercent();
    if (level >= 0)
    {
        JsonDocument doc;
        doc["Time"] = tz.getLocalTimeString();
        doc["level"] = level;

        String payload;
        serializeJson(doc, payload);

        bool pubStatus = mqttClient.publish(
            mqttTopicBuilder(mqttDispatcher.getClientID(), CLASS_TELE, TOPIC_RESERVOIR).c_str(),
            payload.c_str(),
            true);
        if (!pubStatus)
        {
            ESP_LOGE("", "mqttClient.publish failed %d %s", payload.length(), payload.c_str());
        }
    }
}

// ---------- Control logic ----------
float PlantController::computeWaterNeedMl(const PlantConfig &pc, float currentMoisture)
{
    // Amount of water in ml needed to move from current to target moisture,
    // scaled by field capacity (how much of the pot volume can be water).
    // ml ~= cm3, moisture % assumed relative to "saturation/field capacity".
    float deltaPct = pc.targetMoisture_pct - currentMoisture;
    if (deltaPct <= 0)
        return 0.0f;

    ESP_LOGI(TAG, "PumpPort: %i, Moisture net deltaPct: %f=%f-%f", pc.pumpPort, deltaPct, pc.targetMoisture_pct, currentMoisture);

    // as from experience if delta is lower minWateringPCT it might have no durable effect on moisture,
    // therefor minWateringPCT is added
    deltaPct += minWateringPCT;

    // vice versa if delta (gap) is too high, the water will not stay in plant
    // so value is limited to maxWateringPCT

    if (deltaPct > maxWateringPCT)
    {
        deltaPct = maxWateringPCT;
    }

    float capacityFraction = constrain(pc.fieldCapacity_pct / 100.0f, 0.0f, 1.0f);
    ESP_LOGI(TAG, "PumpPort: %i,Soil capacityFraction: %f, deltaPct incl. min max watering caps %f", pc.pumpPort, capacityFraction, deltaPct);

    float ml = pc.potVolume_cm3 * capacityFraction * (deltaPct / 100.0f);

    ESP_LOGI(TAG, "PumpPort: %i,Calculated ml: %f, Pot-Volume: %f", pc.pumpPort, ml, pc.potVolume_cm3);
    return ml;
}

void PlantController::controlPlant(int plantIdx, bool forceFlag)
{
    bool update_flag = false;
    if (plantIdx >= NUM_PLANTS)
        return;

    PlantConfig &cfg = plants[plantIdx].config;
    PlantState &st = plants[plantIdx].state;

    // Skip if no pump assigned
    if (cfg.pumpPort == VALUE_NOT_SET)
        return;

    // Skip if control is done manually
    if (cfg.autoMode == 0 && !forceFlag)
        return;

    bool utcFlag;
    unsigned long nowInSec = tz.getNow(utcFlag);

    float neededMl = 0.f;

    // Respect recheck delay after a watering stage
    if (st.nextRecheckAtEpoch != 0 && nowInSec < st.nextRecheckAtEpoch)
        return;
    // if sensor data are not refreshed between lastWatering and now -> skip
    if (st.lastWatering.timestamp != 0 && st.lastMoisture.timestamp < st.lastWatering.timestamp)
    {
        // ESP_LOGI(TAG, "Plant ID %u skipped as no newer sensor data are available. Sensor: %u, Pump: %u", plantIdx + 1, st.lastMoisture.timestamp, st.lastWatering.timestamp);
        return;
    }
    // last measured moisture invalid
    if (st.lastMoisture.percent == INVALID_FLOAT)
        return;
    // Already at or above target → reset countTNA
    if (st.lastMoisture.percent >= cfg.targetMoisture_pct)
    {
        if (st.countTNA != 0)
        {
            update_flag = true;
            st.countTNA = 0;
        }
        if (st.nextRecheckAtEpoch != 0)
        {
            update_flag = true;
            st.nextRecheckAtEpoch = 0;
        }
        if (update_flag)
            updateStateToConfig(plant_config_doc);

        return;
    }

    // Water only if below min moisture
    if (st.lastMoisture.percent <= cfg.targetMoisture_pct)
    {
        neededMl = computeWaterNeedMl(cfg, st.lastMoisture.percent);
        if (neededMl <= 0.0f)
        {
            if (st.countTNA != 0)
            {
                update_flag = true;
                st.countTNA = 0;
            }
            if (st.nextRecheckAtEpoch != 0)
            {
                update_flag = true;
                st.nextRecheckAtEpoch = 0;
            }
            if (update_flag)
                updateStateToConfig(plant_config_doc);
            return;
        }

        ESP_LOGI(TAG, "Plant ID %u PumpIndex: %i", plantIdx + 1, (cfg.pumpPort - 1));
        if (pumps.pushJobByVolume(cfg.pumpPort - 1, neededMl))
        {
            st.lastWatering.timestamp = nowInSec;
            st.lastWatering.doseMl = neededMl;
            st.nextRecheckAtEpoch = nowInSec + absorptionWaitSec;
            st.countTNA += 1;
            ESP_LOGI(TAG, "Plant ID %u Moisture=%.1f%%, dose=%.0fml, now %u, recheck: %u, st.countTNA: %u",
                     plantIdx + 1, st.lastMoisture.percent, neededMl, st.lastWatering.timestamp, st.nextRecheckAtEpoch, st.countTNA);

            //
            if (st.countTNA > maxCountTNA)
            {
                // change status, TBD
            }
            updateStateToConfig(plant_config_doc);
        }
        return;
    }
}
