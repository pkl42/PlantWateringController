/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mqttHelper.h"
#include "esp_log.h"

const char *CLASS_CMD = "cmnd";
const char *CLASS_STATE = "stat";
const char *CLASS_TELE = "tele";

const char *TOPIC_PING = "PING";

String mqttTopicBuilder(const String &device,
                        const char *cmd_class,
                        const char *topic,
                        const int index,
                        const char *subtopic)
{
    String result = String(cmd_class) + "/" + device + "/" + topic;

    if (index >= 0)
    {
        result += "/" + String(index);
    }

    if (subtopic != nullptr && strlen(subtopic) > 0)
    {
        result += "/" + String(subtopic);
    }

    return result;
}

std::vector<String> splitTopic(const char *topic, char delimiter)
{
    std::vector<String> parts;
    String token;
    for (int i = 0; topic[i] != '\0'; i++)
    {
        if (topic[i] == delimiter)
        {
            if (token.length() > 0)
            {
                parts.push_back(token);
                token = "";
            }
        }
        else
        {
            token += topic[i];
        }
    }
    if (token.length() > 0)
    {
        parts.push_back(token);
    }

    // Logging original topic
    // ESP_LOGI("", "Original topic: %s", topic);

    // Logging all parts in one line
    String partsStr = "[";
    for (size_t i = 0; i < parts.size(); i++)
    {
        partsStr += parts[i];
        if (i < parts.size() - 1)
            partsStr += ", ";
    }
    partsStr += "]";
    // ESP_LOGI("", "Split parts: %s", partsStr.c_str());

    return parts;
}

// ---------- Character validation helper ----------
template <typename T>
bool isValidNumberString(const String &msg, T minVal)
{
    if (msg.length() == 0)
        return false;

    for (size_t i = 0; i < msg.length(); i++)
    {
        if (i == 0 && msg[i] == '-' && minVal < 0)
        {
            continue; // allow leading minus if negatives allowed
        }
        if (!isDigit(msg[i]))
        {
            return false; // contains non-digit
        }
    }
    return true;
}

int isValidInt(const String &msg, int minInt, int maxInt)
{
    if (!isValidNumberString<int>(msg, minInt))
        return INVALID_INT;

    long val = msg.toInt();
    if (val < minInt || val > maxInt)
        return INVALID_INT;

    return static_cast<int>(val);
}

int isValidInt(const JsonVariantConst &val, int minInt, int maxInt)
{
    if (val.isNull())
    {
        return INVALID_INT; // missing key
    }

    // Reuse String-based validator
    return isValidInt(val.as<String>(), minInt, maxInt);
}

long isValidLong(const String &msg, long minLong, long maxLong)
{
    if (!isValidNumberString<long>(msg, minLong))
        return INVALID_LONG;

    long val = msg.toInt(); // Arduino/ESP32 toInt() returns long
    if (val < minLong || val > maxLong)
        return INVALID_LONG;

    return val;
}

long isValidLong(const JsonVariantConst &val, long minLong, long maxLong)
{
    if (val.isNull())
        return INVALID_LONG; // missing key
    return isValidLong(val.as<String>(), minLong, maxLong);
}

unsigned long isValidULong(const String &msg, unsigned long minLong, unsigned long maxLong)
{
    if (!isValidNumberString<long>(msg, minLong))
        return INVALID_ULONG;

    unsigned long val = msg.toInt(); // Arduino/ESP32 toInt() returns long
    if (val < minLong || val > maxLong)
        return INVALID_ULONG;

    return val;
}

unsigned long isValidULong(const JsonVariantConst &val, unsigned long minLong, unsigned long maxLong)
{
    if (val.isNull())
        return INVALID_ULONG; // missing key
    return isValidULong(val.as<String>(), minLong, maxLong);
}

float isValidFloat(const String &msg, const float minVal, const float maxVal)
{
    if (msg.length() == 0)
    {
        return INVALID_FLOAT; // empty string → invalid
    }

    bool hasDecimal = false;

    for (size_t i = 0; i < msg.length(); i++)
    {
        if (i == 0 && msg[i] == '-' && minVal < 0)
        {
            continue; // allow leading minus
        }
        if (msg[i] == '.')
        {
            if (hasDecimal)
            {
                return INVALID_FLOAT; // more than one dot
            }
            hasDecimal = true;
            continue;
        }
        if (!isDigit(msg[i]))
        {
            return INVALID_FLOAT; // non-digit (and not dot/minus)
        }
    }
    float val = msg.toFloat();

    if (isnan(val) || val < minVal || val > maxVal)
    {
        return INVALID_FLOAT; // not a number or out of range
    }

    return val;
}

float isValidFloat(const JsonVariantConst &val, const float minVal, const float maxVal)
{
    if (val.isNull())
    {
        return INVALID_FLOAT; // missing key
    }

    // Reuse String-based validator
    return isValidFloat(val.as<String>(), minVal, maxVal);
}

float roundToDecimals(float value, int decimals)
{
    float factor = powf(10.0f, decimals);
    return roundf(value * factor) / factor;
}

String buildClientID(String mqttClientIDPrefix, String deviceID)
{
    String id = mqttClientIDPrefix + deviceID;

    // enforce max 23 characters for MQTT client ID
    if (id.length() > 23)
    {
        id = id.substring(0, 23);
    }
    ESP_LOGI("", "buildClientID: %s", id);
    return id;
}

// Match topicParts against patternParts (supports + and #)
bool topicMatches(const std::vector<String> &topicParts, const std::vector<String> &patternParts)
{
    // ESP_LOGI("", "topicParts.size: %i patternParts.size: %i ", topicParts.size(), patternParts.size());

    for (size_t i = 0, j = 0; i < topicParts.size() && j < patternParts.size(); i++, j++)
    {
        // ESP_LOGI("", "topic %i %s pattern %i %s", i, topicParts[i].c_str(), j, patternParts[j].c_str());
        if (patternParts[j] == "#")
        {
            return true; // matches everything after this point
        }
        if (patternParts[j] == "+")
        {
            continue; // matches exactly one level
        }
        if (patternParts[j] != topicParts[i])
        {
            return false; // mismatch
        }
    }
    if (patternParts.size() > 0 && patternParts.back() == "#")
    {
        return true; // trailing # always matches
    }

    return topicParts.size() == patternParts.size();
}

//
// MqttDispatcher class
//

MqttDispatcher::MqttDispatcher(PubSubClient &client) : mqttClient(client) {}

void MqttDispatcher::registerHandler(const String &topicPattern, HandlerFunc handler, const String &id)
{
    auto tokens = splitTopic(topicPattern.c_str());
    String effectiveId = id.isEmpty() ? topicPattern : id;

    // Check if same topic+id already in the table
    for (auto &entry : handlerTable)
    {
        if (entry.patternParts == tokens && entry.id == effectiveId)
        {
            // Same topic+ID → skip duplicate registration
            ESP_LOGI(TAG, "pattern and id already registered: %s %s", topicPattern.c_str(), effectiveId.c_str());
            return;
        }
    }

    // Otherwise add new entry
    ESP_LOGI(TAG, "register handler for topic: %s/%s/%s/%s",
             tokens.size() > 0 ? tokens[0].c_str() : "<unknown>",
             tokens.size() > 1 ? tokens[1].c_str() : "<unknown>",
             tokens.size() > 2 ? tokens[2].c_str() : "<unknown>",
             tokens.size() > 3 ? tokens[3].c_str() : "<unknown>");
    handlerTable.push_back({tokens, handler, effectiveId});
}

void MqttDispatcher::dispatch(const std::vector<String> &topicParts, const String &payload)
{
    bool matched = false;
    for (auto &entry : handlerTable)
    {
        if (topicMatches(topicParts, entry.patternParts))
        {
            entry.handler(topicParts, payload);
            matched = true;
        }
    }
    if (!matched)
    {
        ESP_LOGW(TAG, "No handler matched topic: %s/%s/%s/%s",
                 topicParts.size() > 0 ? topicParts[0].c_str() : "<unknown>",
                 topicParts.size() > 1 ? topicParts[1].c_str() : "<unknown>",
                 topicParts.size() > 2 ? topicParts[2].c_str() : "<unknown>",
                 topicParts.size() > 3 ? topicParts[3].c_str() : "<unknown>");
    }
}

void MqttDispatcher::mqttCallback(char *topic, byte *payload, unsigned int length)
{
    // Convert payload to string
    String payloadStr;
    payloadStr.reserve(length);
    for (unsigned int i = 0; i < length; ++i)
    {
        payloadStr += (char)payload[i];
    }
    payloadStr.trim();
    payloadStr.toUpperCase();

    std::vector<String> topicParts = splitTopic(topic);

    ESP_LOGI(TAG, "t: %s p: %s", topic, payloadStr.c_str());

    // Minimal validation
    if (topicParts.size() < 3)
    {
        ESP_LOGW(TAG, "Topic too short, ignoring");
        return;
    }

    dispatch(topicParts, payloadStr);
}

void MqttDispatcher::mqttSelfTest()
{
    if (millis() - lastSelfTest > selfTestInterval)
    {
        loopbackReceived = false;
        mqttClient.publish(mqttTopicBuilder(clientID, CLASS_STATE, TOPIC_PING).c_str(), "");

        lastSelfTest = millis();
    }
}

void MqttDispatcher::selfPing(void)
{
    loopbackReceived = true;
}

bool MqttDispatcher::subscriptionLost(void)
{
    if (!loopbackReceived && millis() - lastSelfTest > selfTestInterval * 2)
    {
        ESP_LOGI(TAG, "MQTT subscription lost → re-subscribing...");
        return true;
        // also re-subscribe your other cmnd topics here
    }
    return false;
}

void MqttDispatcher::init(const String &clientID)
{
    setClientID(clientID);

    registerHandler(
        mqttTopicBuilder(clientID, CLASS_CMD, TOPIC_PING, -1, ""),
        [this](const std::vector<String> &topic, const String &payload)
        {
            selfPing();
        });
}
