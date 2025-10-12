/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "MoistureController.h"
#include <ArduinoJson.h>

#include "esp_log.h"

IMoistureSensor::IMoistureSensor(TimeZone &timeZone, const String &sensorName)
    : tz(timeZone), sensorName(sensorName)
{
}

void IMoistureSensor::setCalibration(int dry, int wet)
{
    if (dry != VALUE_NOT_SET)
    {
        rawDry = dry;
    }
    if (wet != VALUE_NOT_SET)
    {
        rawWet = wet;
    }
}

void IMoistureSensor::setRaw(int rawValue, unsigned long utcTime)
{
    lastReadRaw = rawValue;

    if (utcTime > 0)
    {
        // Use provided UTC/Epoch timestamp
        lastUpdateRaw = utcTime;
        lastUpdateRawUsingEpoch = true;
        // ESP_LOGI("", "setRaw utcTime %lu, %lu, %i, %i", utcTime, lastUpdateRaw, lastReadRaw, lastUpdateRawUsingEpoch);
    }
    else
    {
        // Fallback to system time or millis
        lastUpdateRaw = tz.getNow(lastUpdateRawUsingEpoch);
        // ESP_LOGI("", "setRaw Fallback %lu, %i, %i", lastUpdateRaw, lastReadRaw, lastUpdateRawUsingEpoch);
    }
}

void IMoistureSensor::setPercent(float percentValue, unsigned long utcTime)
{
    lastReadPer = percentValue;

    if (utcTime > 0)
    {
        // Use provided UTC timestamp
        lastUpdatePer = (unsigned long)utcTime * 1000UL;
        lastUpdatePerUsingEpoch = true;
    }
    else
    {
        // Fallback to system time or millis
        lastUpdatePer = tz.getNow(lastUpdatePerUsingEpoch);
    }
    // ESP_LOGI("", "setPercent %u, %i", lastUpdateRaw, lastReadRaw);
}

bool IMoistureSensor::isDataValid(unsigned long lastUpdate, bool dateInUtc)
{
    unsigned long nowMs = tz.getNow(lastCheckUtc);
    return (nowMs - lastUpdate) <= maxLifetimeMs;
}

void IMoistureSensor::setLifetime(unsigned int lifetimeSec)
{
    maxLifetimeMs = lifetimeSec * 1000UL;
    ESP_LOGI(sensorName, "Lifetime updated to %u seconds", lifetimeSec);
}

unsigned int IMoistureSensor::getLifeTime()
{
    return (unsigned int)(maxLifetimeMs / 1000UL);
}

//
// private functions
//

//
// MoistureSensorLocal
//
MoistureSensorLocal::MoistureSensorLocal(TimeZone &timeZone, const String &sensorName, int pin, int dry, int wet)
    : IMoistureSensor(timeZone, sensorName)
{
    IMoistureSensor::connType = ConnectionType::LOCAL;
    sensorPin = pin;
    setCalibration(dry, wet);
}

void MoistureSensorLocal::init()
{
    if (sensorPin > 0)
    {
        analogSetPinAttenuation(sensorPin, ADC_11db);
        analogReadResolution(12);
        // Enable internal pull-up to bias floating pin
        pinMode(sensorPin, INPUT_PULLUP);
        analogRead(sensorPin); // warm-up read
    }
}

void MoistureSensorLocal::setPin(int pin)
{
    if (pin == sensorPin)
    {
        return;
    }

    sensorPin = pin;
    init();
}

MoistureData MoistureSensorLocal::readPercent()
{
    MoistureData result{};
    result.percent = INVALID_FLOAT;
    result.timestamp = 0;

    int raw = readRaw();
    if (raw == INVALID_INT)
    {
        return result;
    }
    result.timestamp = getRawTimestamp();

    if (rawWet == rawDry)
    {
        result.percent = 100.0f; // fail-safe: assume fully wet
        return result;
    }

    float pct = ((float)(rawDry - raw) * 100.0f) / (float)(rawDry - rawWet);
    pct = constrain(pct, 0.0f, 100.0f);

    result.percent = pct;

    return result;
}

int MoistureSensorLocal::readRaw()
{
    const int samples = 8;
    long acc = 0;
    for (int i = 0; i < samples; i++)
    {
        acc += analogRead(sensorPin);
        delay(2);
    }
    lastReadRaw = (acc / samples);
    // --- Detect invalid reading ---
    // Your "dry" baseline is around 3100–3200.
    // Floating w/ pull-up will be closer to 4095 (max at 12-bit).
    int helpInt = lastReadRaw;
    if ((lastReadRaw > 3500) || (lastReadRaw < 800))
    {
        // Mark as invalid (you can choose 0, -1, or a special flag)
        lastReadRaw = INVALID_INT;
    }
    // ESP_LOGI("", "%s lastReadRaw %i => %i", (this->getSensorName()).c_str(), helpInt, lastReadRaw);
    setRaw(lastReadRaw);
    return lastReadRaw;
}

//
// ----- MoistureSensorMQTT Class -----
//

const char *TOPIC_MOISTURE = "MOISTURE";
const char *TOPIC_SENSOR_TASMOTA = "SENSOR";

MoistureSensorMQTT::MoistureSensorMQTT(TimeZone &timeZone, const String &sensorName,
                                       int dry,
                                       int wet,
                                       unsigned long lifeTimeSec)
    : IMoistureSensor(timeZone, sensorName)
{
    IMoistureSensor::connType = ConnectionType::MQTT;
    maxLifetimeMs = (lifeTimeSec * 1000UL);
    setCalibration(dry, wet);
}

void MoistureSensorMQTT::init()
{
    // the mqttCallback is called from global mqttCallback function
    // mqttClient.setCallback([this](char *topic, byte *payload, unsigned int length)
    //                        { this->mqttCallback(topic, payload, length); });

    // mqttClient.subscribe(mqttTopicBuilder(sensorID, CLASS_STATE, TOPIC_MOISTURE, -1, "#").c_str(), 1);
}

int MoistureSensorMQTT::readRaw()
{
    if (!isDataValid(lastUpdateRaw, lastUpdateRawUsingEpoch))
    {
        // ESP_LOGW("MoistureMQTT", "Data expired for %s", sensorName.c_str());
        lastReadRaw = INVALID_INT;
    }
    return lastReadRaw;
}

MoistureData MoistureSensorMQTT::readPercent()
{
    MoistureData result{};
    result.percent = INVALID_FLOAT;
    result.timestamp = 0;

    if (lastUpdatePer < lastUpdateRaw)
    {
        int raw = readRaw();
        // ESP_LOGI("", "readPercentage(1) lastUpdatePer %u, lastUpdateRaw %u %i", lastUpdatePer, lastUpdateRaw, raw);
        if (raw == INVALID_INT)
        {
            if (readPercentState)
            {
                ESP_LOGW("MoistureMQTT", "invalid read (logged again once valid read): %u", millis() / 1000UL);
                readPercentState = false;
            }

            return result;
        }

        if (rawWet == rawDry)
        {
            result.percent = 100.0f; // fail-safe: assume fully wet
            return result;
        }

        float pct = ((float)(rawDry - raw) * 100.0f) / (float)(rawDry - rawWet);
        pct = constrain(pct, 0.0f, 100.0f);

        lastReadPer = pct;
        result.percent = pct;
        lastUpdatePer = lastUpdateRaw;

        if (!readPercentState)
        {
            ESP_LOGI("MoistureMQTT", "valid read for %s (logged again once invalid read happens)", sensorName.c_str());
            readPercentState = true;
        }
    }
    else
    {
        if (!isDataValid(lastUpdatePer, lastUpdatePerUsingEpoch))
        {
            if (readPercentState)
            {
                ESP_LOGW("MoistureMQTT", "Data expired for %s (logged again once valid read)", sensorName.c_str());
                readPercentState = false;
            }
        }
        else
        {
            result.percent = lastReadPer;
            result.timestamp = lastUpdatePer;
        }
    }

    // ESP_LOGI("", "readPercentage lastUpdatePer: %u, lastUpdateRaw: %u, Percentage: %f", lastUpdatePer, lastUpdateRaw, lastReadPer);
    return result;
}

//
// ----- MoistureController Class -----
//
#include "mqttHelper.h"
unsigned long MoistureController::sleepTimeRemoteSensors = 0UL; // 1800UL;

MoistureController::MoistureController(const std::vector<int> &pins, TimeZone &timezone)
    : moisturePins(pins), moistureConfig("/sensorConfig.cfg"), tz(timezone)
{
}

MoistureController::~MoistureController()
{
    // Delete all sensor instances to free memory
    for (auto &pair : sensorsByID)
    {
        delete pair.second;
    }
    sensorsByID.clear();
    sensorsByName.clear();
    portBySensorID.clear();
}

void MoistureController::init(PubSubClient &mqttClient, const String &mqttClientID)
{
    this->mqttClient = &mqttClient;
    this->mqttClientID = mqttClientID;

    // moistureConfig.remove();
    moisture_config_doc = moistureConfig.read_config();
    JsonArray arr = moisture_config_doc["sensors"];
    if (arr.isNull() || arr.size() != NUM_SENSORS)
    {
        ESP_LOGW("", "Config invalid or missing, resetting to defaults");
        moistureConfig.remove();                            // delete corrupt file
        moisture_config_doc = moistureConfig.read_config(); // re-create defaults
        arr = moisture_config_doc["sensors"];
    }

    if (arr.isNull() || arr.size() != NUM_SENSORS)
    {
        ESP_LOGE("", "Config format invalid, using defaults");
    }

    // loadSetupFromConfig(moisture_config_doc);
    lifeTimeSensorDataSec = moisture_config_doc["lifeTimeSensorDataSec"];
    sleepTimeRemoteSensors = moisture_config_doc["sleepTimeRemoteSensors"] | 0UL;
}

void MoistureController::loadSetupFromConfig(JsonDocument &doc)
{
    ESP_LOGI("", "loadSetupFromConfig");

    JsonArray sensorArray = doc["sensors"];

    if (sensorArray.isNull() || sensorArray.size() != NUM_SENSORS)
    {
        ESP_LOGE("", "sensor array missing or wrong size");
        return; // or handle recovery
    }

    lifeTimeSensorDataSec = doc["lifeTimeSensorDataSec"];
    sleepTimeRemoteSensors = doc["sleepTimeRemoteSensors"] | 0UL;

    // Loop through each element in the array
    for (JsonVariant sensor : sensorArray)
    {
        // If each sensor is an object, you can access its keys
        if (sensor.is<JsonObject>())
        {
            for (JsonPair kv : sensor.as<JsonObject>())
            {
                String val = kv.value().as<String>();
                ESP_LOGI("", "%s : %s", kv.key().c_str(), val.c_str());
            }
        }
        else
        {
            // If it's just a primitive value
            ESP_LOGI("", "sensor: %s", sensor.as<const char *>());
        }
    }

    int i = 0;
    // for (JsonObject sensorObj : sensorArray)
    // {
    //     if (i >= NUM_SENSORS)
    //         break;
    //     if (sensorObj.isNull())
    //     {
    //         ESP_LOGW(TAG, "Sensor object %d is null, skipping", i);
    //         i++;
    //         continue;
    //     }

    //     sensorObj["sensorID"];
    //     sensorObj["sensorName"];
    //     sensorObj["dry"];
    //     sensorObj["wet"];
    //     sensorObj["pin"];
    //     sensorObj["connType"];
    //     i++;
    // }
    ESP_LOGI("", "loadSetupFromConfig done");
}

IMoistureSensor *MoistureController::getSensorByName(const String &sensorName)
{
    auto it = sensorsByName.find(sensorName);
    if (it != sensorsByName.end())
        return it->second;
    return nullptr;
}

IMoistureSensor *MoistureController::getSensorByID(const int sensorID)
{
    auto it = sensorsByID.find(sensorID);
    if (it != sensorsByID.end())
        return it->second;
    return nullptr;
}

int MoistureController::getSensorPortID(const int sensorID)
{
    auto it = portBySensorID.find(sensorID);
    if (it != portBySensorID.end())
        return it->second;
    return VALUE_NOT_SET;
}

int MoistureController::updateSensor(const int sensorID,
                                     const String &sensorName,
                                     ConnectionType connectionType,
                                     int rawDry,
                                     int rawWet)
{
    int returnValue = 0;
    bool update_flag = false;
    bool reinstantiate_flag = false;
    IMoistureSensor *sensorInstance;
    JsonObject sensor;
    // try to get sensor data within configuration
    if (sensorID != VALUE_NOT_SET)
    {
        if (sensorID > NUM_SENSORS)
        {
            return (-1);
        }
        sensor = moisture_config_doc["sensors"][sensorID - 1];
        if (!sensor.isNull())
        {
            sensorInstance = getSensorByID(sensorID);
        }
    }

    if ((sensorName.length() > 0) && (sensorID == VALUE_NOT_SET))
    {
        sensorInstance = getSensorByName(sensorName);
        if (sensorInstance != nullptr)
        {
            sensor = moisture_config_doc["sensors"][(sensorInstance->getSensorID() - 1)];
        }
    }
    else
    {
        if (sensorName.length() > 0)
        {
            if (sensorName != sensor["sensorName"].as<String>())
            {
                ESP_LOGI("", "new sensorName: %s old sensorName: %s", sensorName.c_str(), sensor["sensorName"].as<String>().c_str());
                sensor["sensorName"] = sensorName;
                update_flag = true;
                if (sensorInstance != nullptr)
                    reinstantiate_flag = true;
            }
        }
    }

    if (connectionType != ConnectionType::UNDEFINED)
    {
        String currentConnType = sensor["connType"] | "";
        String newConnType = (connectionType == ConnectionType::LOCAL) ? "LOCAL" : "MQTT";
        if (currentConnType != newConnType)
        {
            sensor["connType"] = newConnType;
            if (sensorInstance != nullptr)
                reinstantiate_flag = true;
            update_flag = true;
        }
    }

    int currentDry = sensor["dry"] | VALUE_NOT_SET;
    if (rawDry != VALUE_NOT_SET && rawDry != currentDry)
    {
        sensor["dry"] = rawDry;
        if (sensorInstance != nullptr)
        {
            sensorInstance->setCalibration(rawDry, VALUE_NOT_SET);
        }
        update_flag = true;
    }

    int currentWet = sensor["wet"] | VALUE_NOT_SET;
    if (rawWet != VALUE_NOT_SET && rawWet != currentWet)
    {
        sensor["wet"] = rawWet;
        if (sensorInstance != nullptr)
        {
            sensorInstance->setCalibration(rawDry, VALUE_NOT_SET);
        }
        update_flag = true;
    }

    if (update_flag)
    {
        moistureConfig.write_config(&moisture_config_doc);
    }
    if (reinstantiate_flag)
    {
        int localPort = getSensorPortID(sensorID);
        initSensor(sensorID, localPort, true);
    }
    publishSensorState(sensorID);

    return returnValue;
}

void MoistureController::deleteSensorInstance(const int sensorID)
{
    ESP_LOGI("", "sensorID: %i", sensorID);
    auto it = sensorsByID.find(sensorID);
    if (it != sensorsByID.end())
    {
        ESP_LOGI("", "delete sensorID: %i", sensorID);

        IMoistureSensor *sensor = it->second;
        String sensorName = sensor->getSensorName();
        if (sensor->getConnectionType() == ConnectionType::MQTT)
        {
            mqttClient->unsubscribe(mqttTopicBuilder(sensorName, CLASS_TELE, TOPIC_MOISTURE, -1, "#").c_str());
        }

        // Remove from name map first
        auto nameIt = sensorsByName.find(sensorName);
        if (nameIt != sensorsByName.end())
        {
            sensorsByName.erase(nameIt);
        }

        // Remove from ID map
        sensorsByID.erase(it);

        // Free memory
        delete sensor;
    }
}

void MoistureController::initSensor(const int sensorID, const int localPort, const bool forceFlag)
{
    // check sensor is not yet instantiate
    ESP_LOGI("", "initSensor %i", sensorID);
    if (sensorID == VALUE_NOT_SET)
    {
        ESP_LOGE("", "not a valid SensorID");
        return;
    }
    IMoistureSensor *thisSensor = getSensorByID(sensorID);
    if (thisSensor != nullptr && forceFlag)
    {
        ESP_LOGI("", "reinstantiate Sensor %i", sensorID);

        deleteSensorInstance(sensorID);
        thisSensor = getSensorByID(sensorID);
    }

    if (thisSensor == nullptr)
    {
        JsonObject sensor = moisture_config_doc["sensors"][sensorID - 1];

        String sensorName = sensor["sensorName"].as<String>();
        int rawDry = sensor["dry"] | 0;
        int rawWet = sensor["wet"] | 0;

        String connTypeString = sensor["connType"].as<String>();
        ConnectionType connectionType = (connTypeString == "LOCAL") ? ConnectionType::LOCAL : ConnectionType::MQTT;

        thisSensor = createSensorInstance(sensorID, sensorName, connectionType, localPort, rawDry, rawWet);
    }
    else
    {
        portBySensorID[sensorID] = localPort;
    }

    if (thisSensor == nullptr)
    {
        return;
    }
    ESP_LOGI("", "thisSensor %i -> %p ", sensorID, thisSensor);
    if (thisSensor->getConnectionType() == ConnectionType::MQTT)
    {
        // subscribe topic
        ESP_LOGI("", "subscribe for remote Sensor: %s", mqttTopicBuilder(thisSensor->getSensorName(), CLASS_TELE, TOPIC_MOISTURE, -1, "#").c_str());
        mqttClient->subscribe(mqttTopicBuilder(thisSensor->getSensorName(), CLASS_TELE, TOPIC_MOISTURE, -1, "#").c_str(), 1);
        // set

        String payload = String(sleepTimeRemoteSensors);
        mqttClient->publish(
            mqttTopicBuilder(thisSensor->getSensorName(), CLASS_CMD, "Mem3").c_str(),
            payload.c_str(),
            true);
    }

    thisSensor->init();
}

void MoistureController::subscribeSensorTopics(bool forceFlag)
{
    static bool subscribed = false;
    // Subscribe to pump control & settings

    if (!subscribed || forceFlag)
    {
        ESP_LOGI("", "subscribe tele Topics...");
        for (auto &pair : sensorsByID)
        {
            if (pair.second == nullptr)
                continue;
            if (pair.second->getConnectionType() == ConnectionType::MQTT)
            {
                mqttClient->subscribe(mqttTopicBuilder(pair.second->getSensorName(), CLASS_TELE, TOPIC_MOISTURE, -1, "#").c_str(), 1);
            }
        }
        subscribed = true;
    }
}

MoistureData MoistureController::readPercent(const int sensorID)
{
    MoistureData result{};
    result.percent = INVALID_FLOAT;
    result.timestamp = 0;

    if (sensorID == VALUE_NOT_SET)
        return result;
    IMoistureSensor *thisSensor = getSensorByID(sensorID);
    if (thisSensor == nullptr)
    {
        // try to initialize
        int localPort = getSensorPortID(sensorID);
        initSensor(sensorID, localPort, true);
        thisSensor = getSensorByID(sensorID);
        if (thisSensor == nullptr)
        {
            return result;
        }
    }
    result = thisSensor->readPercent();
    // ESP_LOGI("", "value read %i: %f", sensorID, readPercValue);
    return (result);
}

void MoistureController::processMQTTMessage(const std::vector<String> &topicParts, const String &payload)
{
    // Note: payload is capitalized!!
    bool update_flag = false;
    int topicSize = topicParts.size();

    // ESP_LOGI("", "payload: %s", payload.c_str());

    JsonDocument doc; // adjust if needed
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
        ESP_LOGE("", "JSON parse error: %s\n", error.c_str());
        return;
    }
    JsonObject obj = doc.as<JsonObject>();
    // ESP_LOGI("", "topicSize: %i, payload: %s", topicSize, payload.c_str());
    //
    if (topicSize == 3 && topicParts[0] == CLASS_CMD)
    {
        // --- Handle lifeTimeSensorDataSec ---
        unsigned long newLong = isValidLong((JsonVariantConst)obj["LIFETIMESENSORDATASEC"], 0, 32768);

        if (newLong != INVALID_LONG)
        {
            lifeTimeSensorDataSec = newLong;
            if (moisture_config_doc["lifeTimeSensorDataSec"].as<long>() != lifeTimeSensorDataSec)
            {
                setLifetime(lifeTimeSensorDataSec);
                moisture_config_doc["lifeTimeSensorDataSec"] = lifeTimeSensorDataSec;
                update_flag = true;
            }
        }

        // --- Handle sleep interval for Remote Sensors ---
        newLong = isValidLong((JsonVariantConst)obj["SLEEPTIMEREMOTESENSORS"], 0, 32768);

        if (newLong != INVALID_LONG)
        {
            newLong = (newLong >= 10) ? newLong : 0;
            if (moisture_config_doc["sleepTimeRemoteSensors"].as<long>() != newLong)
            {
                setSleepTimeSensors(newLong);
                moisture_config_doc["sleepTimeRemoteSensors"] = newLong;
                update_flag = true;
            }
        }

        if (update_flag)
        {
            moistureConfig.write_config(&moisture_config_doc);
        }
        publishState();
        return;
    }

    if (topicSize < 3)
    {
        ESP_LOGI("", "topicSize need to be minimum 3: %i -> return", topicSize);
        return;
    }

    IMoistureSensor *currentSensor;

    //
    // handle STATE and TELE
    //
    if (topicParts[0] == CLASS_STATE || topicParts[0] == CLASS_TELE)
    {
        currentSensor = getSensorByName(topicParts[1]);
        if (currentSensor == nullptr)
        {
            ESP_LOGE("", "sensor does not exist: %s", topicParts[1]);
            return;
        }

        // --- Handle Moisture ---
        if (topicParts[2] == TOPIC_MOISTURE) // with custom payload format
        {

            unsigned long newULong = isValidULong((JsonVariantConst)obj["UTCTIME"], 0, INVALID_ULONG - 1);
            if (newULong != INVALID_ULONG)
            {
                if (time(nullptr) - newULong > lifeTimeSensorDataSec)
                {
                    ESP_LOGI("", "Incoming Sensordata from %s stale/outdated.", topicParts[1]);
                    return;
                }
            }

            int newInt = isValidInt((JsonVariantConst)obj["ADC"], 0, 4096);
            // ESP_LOGI("", "ADC value: %i, timestamp: %u", newInt,newULong);
            if (newInt != INVALID_INT)
            {
                if (newULong != INVALID_ULONG)
                {
                    currentSensor->setRaw(newInt, newULong);
                }
                else
                {
                    currentSensor->setRaw(newInt);
                }
            }

            float newFloat = isValidFloat((JsonVariantConst)obj["PCT"], 0., 100.);
            if (newFloat != INVALID_FLOAT)
            {

                if (newULong != INVALID_ULONG)
                {
                    currentSensor->setPercent(newInt, newULong);
                }
                else
                {
                    currentSensor->setPercent(newInt);
                }
            }
        }
        return;

        if (topicParts[2] == TOPIC_SENSOR_TASMOTA) // with tasmota payload format
        {
            // Navigate to ANALOG -> A0
            JsonObject analog = obj["ANALOG"];
            if (!analog.isNull())
            {
                int newInt = isValidInt((JsonVariantConst)analog["A0"], 0, 1024);

                if (newInt != INVALID_INT)
                    currentSensor->setRaw(newInt);
                else
                {
                    ESP_LOGI("", "No ANALOG object in payload");
                }
            }
            return;
        }
    }

    if (topicSize < 4)
    {
        ESP_LOGI("", "topicSize need to be minimum 4: %i -> return", topicSize);
        return;
    }
    //
    // handle CMND
    //
    if (topicParts[0] != CLASS_CMD)
    {
        ESP_LOGI("", "topicParts[0] is not CLASS_CMD->return");
        return;
    }

    int sensorID = isValidInt(topicParts[3], 1, NUM_SENSORS);
    if (sensorID == INVALID_INT)
    {
        ESP_LOGI("", "sensorID==INVALID_INT->return");
        return;
    }

    // Collect parameters safely
    String sensorName = obj["SENSORNAME"].is<const char *>() ? obj["SENSORNAME"].as<String>() : "";

    ConnectionType connType = ConnectionType::UNDEFINED;
    if (obj["CONNTYPE"].is<const char *>())
    {
        String ctype = obj["CONNTYPE"].as<String>();
        if (ctype == "LOCAL")
            connType = ConnectionType::LOCAL;
        else if (ctype == "MQTT")
            connType = ConnectionType::MQTT;
    }

    int rawDry = isValidInt((JsonVariantConst)obj["DRY"], 0, 4096);
    int rawWet = isValidInt((JsonVariantConst)obj["WET"], 0, 4096);

    // Call updateSensor once with collected parameters
    updateSensor(sensorID, sensorName, connType, rawDry, rawWet);
}

void MoistureController::publishState()
{
    JsonDocument doc;
    doc["lifeTimeSensorDataSec"] = lifeTimeSensorDataSec;
    doc["sleepTimeRemoteSensors"] = sleepTimeRemoteSensors;

    String payload;
    serializeJson(doc, payload);

    mqttClient->publish(
        mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_MOISTURE, -1, "").c_str(),
        payload.c_str(),
        true);
}
void MoistureController::publishSensorState(const int sensorID)
{

    for (int i = 0; i < NUM_SENSORS; ++i)
    {

        JsonObject sensor = moisture_config_doc["sensors"][i];

        // If a specific sensorID is requested, skip others
        if (sensorID != VALUE_NOT_SET && sensor["sensorID"].as<int>() != sensorID)
            continue;

        String payload;
        serializeJson(sensor, payload); // Serialize the whole object as-is

        mqttClient->publish(
            mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_MOISTURE, i + 1, "").c_str(),
            payload.c_str(),
            true);
        ESP_LOGI("", "topic: %s", mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_MOISTURE, i + 1, "").c_str());
        ESP_LOGI("", "payload: %s", payload.c_str());
    }
}

void MoistureController::publishTelemetrie(const int sensorID)
{
    int thisSensorID;
    for (int i = 0; i < NUM_SENSORS; ++i)
    {

        JsonObject sensor = moisture_config_doc["sensors"][i];
        if (sensor.isNull())
            continue;

        // If a specific sensorID is requested, skip others
        if (sensorID != VALUE_NOT_SET && sensor["sensorID"].as<int>() != sensorID)
        {
            // ESP_LOGI("", "skipped as %i != %i (sensorID)", sensor["sensorID"].as<int>() != sensorID);
            continue;
        }
        thisSensorID = sensor["sensorID"].as<int>();

        JsonDocument doc;

        IMoistureSensor *thisSensor = getSensorByID(thisSensorID);

        if (thisSensor == nullptr)
        {
            // ESP_LOGI("", "skipped as %i (thisSensorID) has no instance (thisSensor==nullptr)", thisSensorID);
            continue;
        }
        doc["sensorID"] = thisSensorID;
        doc["sensorName"] = sensor["sensorName"];
        doc["localPort"] = portBySensorID[thisSensorID];
        doc["adc"] = thisSensor->readRaw();
        doc["adcTimestamp"] = thisSensor->getRawTimestamp();

        MoistureData result = thisSensor->readPercent();
        doc["pct"] = result.percent;
        doc["pctTimestamp"] = result.timestamp;

        String payload;
        serializeJson(doc, payload);

        mqttClient->publish(
            mqttTopicBuilder(mqttClientID, CLASS_TELE, TOPIC_MOISTURE, thisSensorID, "").c_str(),
            payload.c_str(),
            true);
    }
}

void MoistureController::setLifetime(unsigned int lifeTimeSensorDataSec)
{
    this->lifeTimeSensorDataSec = lifeTimeSensorDataSec;
    // update all sensors
    for (auto &pair : sensorsByID)
    {
        if (pair.second != nullptr)
        {
            pair.second->setLifetime(lifeTimeSensorDataSec);
        };
    }
};

void MoistureController::setSleepTimeSensors(long sleepTimeRemoteSensors)
{
    this->sleepTimeRemoteSensors = sleepTimeRemoteSensors;
    String payload = String(sleepTimeRemoteSensors);
    // update all sensors
    for (auto &pair : sensorsByID)
    {
        if (pair.second != nullptr)
        {
            if (pair.second->getConnectionType() == ConnectionType::MQTT)
            {
                mqttClient->publish(
                    mqttTopicBuilder(pair.second->getSensorName(), CLASS_CMD, "Mem3").c_str(),
                    payload.c_str(),
                    true);
            };
        };
    }
};
//
// ----- private functions ----
//

IMoistureSensor *MoistureController::createSensorInstance(const int sensorID,
                                                          const String &sensorName,
                                                          ConnectionType connectionType,
                                                          int localPort,
                                                          int rawDry,
                                                          int rawWet)
{
    IMoistureSensor *s = nullptr;
    ESP_LOGI("", "sensorID %i, sensorName %s, localPort %i, rawDry %i, rawWet %i", sensorID, sensorName, localPort, rawDry, rawWet);
    if (connectionType == ConnectionType::LOCAL)
    {
        if (localPort <= 0 || localPort > (int)moisturePins.size())
        {
            ESP_LOGE("ERROR", "localPort %i moisturePins.size(): %i", localPort, (int)moisturePins.size());
            return nullptr;
        }
        s = new MoistureSensorLocal(tz, sensorName, moisturePins[localPort - 1], rawDry, rawWet);
    }
    else if (connectionType == ConnectionType::MQTT)
    {
        s = new MoistureSensorMQTT(tz, sensorName, rawDry, rawWet, lifeTimeSensorDataSec);
        // s = new MoistureSensorMQTT(sensorID, mqttClient, rawDry, rawWet);
        ESP_LOGI("SENSOR", "created MoistureSensorMQTT %p", s);
    }
    if (s)
    {
        // Insert into maps
        sensorsByID[sensorID] = s;
        sensorsByName[sensorName] = s;
        portBySensorID[sensorID] = localPort;

        ESP_LOGI("SENSOR", "Added sensor %s (ID=%d) at %p", sensorName.c_str(), sensorID, s);
    }
    return s;
}
