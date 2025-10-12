/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MQTT_HELPER_FUNC_H
#define MQTT_HELPER_FUNC_H

#include "Arduino.h"
#include <ArduinoJson.h>
#include <vector>
#include <limits.h>
#include <functional>
#include <map>

#include <PubSubClient.h>

// -------------------------
// Constants
// -------------------------
extern const char *CLASS_CMD;
extern const char *CLASS_STATE;
extern const char *CLASS_TELE;
extern const char *TOPIC_PING;

const int INVALID_INT = INT16_MIN;
const long INVALID_LONG = LONG_MIN;
const long INVALID_ULONG = 4294967295;
const int VALUE_NOT_SET = -1;
const float INVALID_FLOAT = -9999.0f;

// -------------------------
// Utility Functions
// -------------------------
String mqttTopicBuilder(const String &device,
                        const char *cmd_class,
                        const char *topic,
                        const int index = -1,
                        const char *subtopic = "");

std::vector<String> splitTopic(const char *topic, char delimiter = '/');

int isValidInt(const String &msg, const int minInt, const int maxInt);
int isValidInt(const JsonVariantConst &val, int minInt, int maxInt);

long isValidLong(const String &msg, long minLong, long maxLong);
long isValidLong(const JsonVariantConst &val, long minLong, long maxLong);

unsigned long isValidULong(const String &msg, unsigned long minLong, unsigned long maxLong);
unsigned long isValidULong(const JsonVariantConst &val, unsigned long minLong, unsigned long maxLong);

float isValidFloat(const String &msg, const float minVal, const float maxVal);
float isValidFloat(const JsonVariantConst &val, const float minVal, const float maxVal);

String buildClientID(String mqttClientIDPrefix, String deviceID);

float roundToDecimals(float value, int decimals);

// -------------------------
// Handler Dispatcher
// -------------------------

using HandlerFunc = std::function<void(const std::vector<String> &, const String &)>;

struct HandlerEntry
{
    std::vector<String> patternParts;
    HandlerFunc handler;
    String id; // Unique identifier for this handler
};

bool topicMatches(const std::vector<String> &topicParts, const std::vector<String> &patternParts);

// Dispatcher class for handling MQTT topics
class MqttDispatcher
{
public:
    MqttDispatcher(PubSubClient &client);

    void init(const String &clientID);

    void registerHandler(const String &topicPattern, HandlerFunc handler, const String &id = "");
    void dispatch(const std::vector<String> &topicParts, const String &payload);
    void mqttCallback(char *topic, byte *payload, unsigned int length);
    // --- Getter & Setter for clientID ---
    void setClientID(const String &id)
    {
        // MQTT v3.1.1 spec: Client ID max length = 23 chars
        if (id.length() > 23)
        {
            clientID = id.substring(0, 23);
        }
        else
        {
            clientID = id;
        }
    }

    String getClientID() const
    {
        return clientID;
    }

    void mqttSelfTest();
    void selfPing(void);
    bool subscriptionLost(void);

private:
    const char *TAG = "MqttDispatcher";
    std::vector<HandlerEntry> handlerTable;
    String clientID; // Note: max. 23 characters in v3.1.1 allowed

    PubSubClient &mqttClient;

    volatile bool loopbackReceived = false;
    unsigned long lastSelfTest = 0;
    const unsigned long selfTestInterval = 60000; // every 60s
};
#endif // MQTT_HELPER_H