/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOISTURE_SENSOR_H
#define MOISTURE_SENSOR_H
#include "Arduino.h"
#include <vector>
#include "mqttHelper.h"     //for MoistureSensorMQTT
#include <map>              //for MoistureFactory
#include "moistureConfig.h" //for MoistureFactory
#include "TimeZone.h"

enum class ConnectionType : uint8_t
{
    UNDEFINED,
    LOCAL,
    MQTT
};

// struct SensorConfig
// {
//     String sensorID;
//     ConnectionType connType;
//     int localPin = -1;
//     int dry = 985;
//     int wet = 3147;
// };

struct MoistureData
{
    float percent;
    unsigned long timestamp; // in seconds
};

//
// ----- Base Class -----
//

class IMoistureSensor
{
public:
    IMoistureSensor(TimeZone &timeZone, const String &sensorName = "");

    virtual ~IMoistureSensor() {}

    virtual void init() = 0;                // prepare the sensor
    virtual MoistureData readPercent() = 0; // normalized 0–100%
    virtual int readRaw() = 0;              // raw sensor value (if applicable)
    virtual void setRaw(int rawValue, unsigned long utcTime = 0L);
    virtual void setPercent(float percentValue, unsigned long utcTime = 0L);
    void setCalibration(int dry = INVALID_INT, int wet = INVALID_INT);

    int getRawDry() const { return rawDry; }
    int getRawWet() const { return rawWet; }
    unsigned long getRawTimestamp() const { return lastUpdateRaw; }
    unsigned long getPerTimestamp() const { return lastUpdatePer; }

    const String &getSensorName() const { return sensorName; }
    const int getSensorID() const { return sensorID; }
    const ConnectionType getConnectionType() const { return connType; }

    virtual void setLifetime(unsigned int lifetimeSec);
    virtual unsigned int getLifeTime();
    bool isDataValid(unsigned long lastUpdate, bool dateInEpoch);

protected:
    int lastReadRaw = -1;
    unsigned long lastUpdateRaw = 0;
    bool lastUpdateRawUsingEpoch = false; // did we store UTC-derived time?

    bool lastCheckUtc = false; // did isDataValid() use UTC? - for logging purposes

    unsigned long lastReadPer = 0;
    unsigned long lastUpdatePer = 0;
    bool lastUpdatePerUsingEpoch = false; // did we store UTC-derived time?

    unsigned long maxLifetimeMs = 1000UL; // max. lifetime of sensor data in seconds before becoming stale
    int rawDry = 0;                       // default calibration values
    int rawWet = 100;
    String sensorName; // for remote sensors it is mqttClientID, for local it is sensor number (not port)
    int sensorID;

    ConnectionType connType;
    TimeZone &tz;
};

class MoistureSensorLocal : public IMoistureSensor
{
public:
    MoistureSensorLocal(TimeZone &timeZone,
                        const String &sensorName,
                        int pin,
                        int dry = 985,
                        int wet = 3147);
    void init() override;
    int readRaw() override;
    MoistureData readPercent() override;

    void setPin(int pin);

private:
    int sensorPin = -1;
};

extern const char *TOPIC_REMOTE_MOISTURE;

class MoistureSensorMQTT : public IMoistureSensor
{
public:
    MoistureSensorMQTT(TimeZone &timeZone,
                       const String &sensorName, // mqttClientID=sensorName
                       int dry = 985,
                       int wet = 3147,
                       unsigned long lifeTimeSec = 3600);
    void init() override;
    int readRaw() override;
    MoistureData readPercent() override;

private:
    bool readPercentState = false;
};

//
// ----- MoistureController Class -----
//

class MoistureController
{
public:
    MoistureController(const std::vector<int> &pins, TimeZone &timezone);
    ~MoistureController();

    void init(PubSubClient &mqttClient, const String &mqttClientID);

    IMoistureSensor *getSensorByName(const String &sensorName);
    IMoistureSensor *getSensorByID(const int sensorID);
    int getSensorPortID(const int sensorID);

    int updateSensor(const int sensorID = VALUE_NOT_SET,
                     const String &sensorName = "",
                     ConnectionType connectionType = ConnectionType::UNDEFINED,
                     int rawDry = VALUE_NOT_SET,
                     int rawWet = VALUE_NOT_SET);

    void initSensor(const int sensorID, const int localPort = -1, const bool forceFlag = false);
    MoistureData readPercent(const int sensorID);

    void setLifetime(unsigned int lifetimeSec);
    void setSleepTimeSensors(long sleepTimeRemoteSensors);

    void processMQTTMessage(const std::vector<String> &topicParts, const String &payload);
    void publishSensorState(const int sensorID = VALUE_NOT_SET);
    void publishState();

    void publishTelemetrie(const int sensorID = VALUE_NOT_SET);

    void deleteSensorInstance(const int sensorID);

    void loadSetupFromConfig(JsonDocument &doc);
    void subscribeSensorTopics(bool forceFlag = false);

private:
    std::vector<int>
        moisturePins;

    TimeZone &tz;

    std::map<int, int> portBySensorID;

    std::map<int, IMoistureSensor *> sensorsByID; // sensorID -> pointer
    std::map<String, IMoistureSensor *> sensorsByName;

    MoistureConfig moistureConfig;
    JsonDocument moisture_config_doc;

    IMoistureSensor *createSensorInstance(const int sensorID,
                                          const String &sensorName,
                                          ConnectionType connectionType,
                                          int localPort,
                                          int rawDry,
                                          int rawWet);

    PubSubClient *mqttClient = nullptr;
    String mqttClientID;

    unsigned int lifeTimeSensorDataSec = 120;
    RTC_DATA_ATTR static unsigned long sleepTimeRemoteSensors;
};

#endif // MOISTURE_SENSOR_H
