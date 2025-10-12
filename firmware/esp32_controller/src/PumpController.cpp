/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Arduino.h>
#include "esp_log.h"
#include "PumpController.h"
#include "mqttHelper.h"

PumpController::PumpController(const std::vector<int> &pins, float defaultMlPerMin)
    : pumpPins(pins),
      pumpConfig("/pumpConfig.cfg", pins.size()),
      pumpMlPerMin(pins.size(), defaultMlPerMin),
      pumpState(pins.size())
{
    // Create mutex
    pumpQueueMutex = xSemaphoreCreateMutex();

    initPins();

    // Start background pump task
    xTaskCreatePinnedToCore(
        [](void *param)
        {
            static_cast<PumpController *>(param)->pumpTask();
        },
        "PumpTask", // task name
        2048,       // stack size
        this,       // parameter
        1,          // priority
        nullptr,    // task handle
        1           // core
    );
}

void PumpController::init(PubSubClient &mqttClient, const String &mqttClientID)
{
    this->mqttClient = &mqttClient;
    this->mqttClientID = mqttClientID;
    // read pump configuration from file
    // pumpConfig.remove();
    pump_config_doc = pumpConfig.read_config();
    JsonArray arr = pump_config_doc["pumps"];

    if (arr.isNull() || arr.size() != pumpPins.size())
    {
        ESP_LOGW(TAG, "Config invalid or missing, resetting to defaults");
        pumpConfig.remove();                        // delete corrupt file
        pump_config_doc = pumpConfig.read_config(); // re-create defaults
        arr = pump_config_doc["pumps"];
    }

    if (!arr.isNull() && arr.size() == pumpPins.size())
    {
        for (int i = 0; i < pumpPins.size(); i++)
        {
            setPumpMlPerMin(i, arr[i]["mlPerMin"] | 104.f); // fallback default
        }
    }
    else
    {
        ESP_LOGE(TAG, "Config format invalid, using defaults. arr.size %i, pumpPins.size %i", arr.size(), pumpPins.size());
    }
}

bool PumpController::setPumpState(unsigned int pumpIndex, bool state)
{
    if (pumpIndex >= pumpPins.size())
    {
        ESP_LOGI(TAG, "pumpIndex: %i higher than registered pumpPins.size: %i", pumpIndex, pumpPins.size());
        return false;
    }
    if (state)
    {
        if (jobRunning)
        {
            ESP_LOGI(TAG, "Job is active, cannot manually turn on pump.");
            return false;
        }
        pushJobByDuration(pumpIndex, DEFAULT_MAX_MANUAL_SECONDS);
    }
    else
    {
        manualStopRequested = true;
    }
    return true;
}

bool PumpController::setPumpMlPerMin(unsigned int pumpIndex, float mlPerMin)
{
    if (pumpIndex >= pumpPins.size() || mlPerMin <= 0)
        return false;

    pumpMlPerMin[pumpIndex] = mlPerMin;
    return true;
}

float PumpController::getPumpMlPerMin(unsigned int pumpIndex) const
{
    if (pumpIndex >= pumpPins.size())
        return -1.0f;

    return pumpMlPerMin[pumpIndex];
}

void PumpController::processMQTTMessage(const std::vector<String> &topicParts, const String &payload)
{

    if (topicParts[0] != CLASS_CMD)
    {
        ESP_LOGI(TAG, "only %s are handled -> return", CLASS_CMD);
        return;
    }

    bool update_flag = false;
    int topicSize = topicParts.size();
    if (topicSize < 4)
    {
        if (payload.isEmpty())
        {
            publishSetup();
            publishState();
        }
        return;
    }
    if (topicSize < 4)
    {
        ESP_LOGI(TAG, "topicSize need to minium 4: %i -> return", topicSize);
        return;
    }

    int pumpPort = isValidInt(topicParts[3], 1, pumpPins.size());
    if (pumpPort == INVALID_INT)
    {
        ESP_LOGI(TAG, "topicParts[3] either not a valid integer: %s or number outside pumpPins Array range: %i -> return", topicParts[3], pumpPins.size());
        return;
    }
    ESP_LOGI(TAG, "pumpPort: %s payload: %s", String(pumpPort), payload.c_str());

    if (payload.isEmpty())
    {
        publishSetup(pumpPort);
        publishState(pumpPort);
    }
    //
    if (topicParts.size() == 5 && topicParts[4] == "STATE")
    {
        if (payload == "ON")
        {
            setPumpState(pumpPort - 1, true);
        }
        else if (payload == "OFF")
        {
            setPumpState(pumpPort - 1, false);
        }
        else
        {
            uint32_t sec = isValidInt(payload.c_str(), 1, DEFAULT_MAX_MANUAL_SECONDS);
            if (sec > 0)
            {
                pushJobByDuration(pumpPort - 1, sec);
            }
            else
            {
                ESP_LOGI(TAG, "processMQTTMessage Invalid payload: %s", payload.c_str());
            }
        }
        return;
    }

    // Pump Configuration

    JsonDocument doc; // adjust if needed
    DeserializationError error = deserializeJson(doc, payload);
    if (error)
    {
        ESP_LOGE("", "JSON parse error: %s\n", error.c_str());
        return;
    }
    // ESP_LOGI("", "payload: %s", payload.c_str());
    JsonObject obj = doc.as<JsonObject>();
    float newFloat = isValidFloat((JsonVariantConst)obj["MLPERMIN"], 0., 100000.);

    if (newFloat != INVALID_FLOAT && pump_config_doc["pumps"][pumpPort - 1]["mlPerMin"].as<float>() != newFloat)
    {
        ESP_LOGI("", "PumpPort: %i, old mlPerMin: %f, new: %f", pumpPort, pump_config_doc["pumps"][pumpPort - 1]["mlPerMin"].as<float>(), newFloat);
        setPumpMlPerMin(pumpPort - 1, newFloat);
        pump_config_doc["pumps"][pumpPort - 1]["mlPerMin"] = newFloat;
        update_flag = true;
    }

    if (update_flag)
    {
        pumpConfig.write_config(&pump_config_doc);
    }
    publishSetup(pumpPort);
}

void PumpController::publishSetup(const int pumpPort)
{
    for (int i = 0; i < pumpPins.size(); ++i)
    {

        JsonObject pump = pump_config_doc["pumps"][i];

        // If a specific sensorID is requested, skip others
        if (pumpPort != VALUE_NOT_SET && pump["pumpID"].as<int>() != pumpPort)
            continue;

        String payload;
        serializeJson(pump, payload); // Serialize the whole object as-is

        mqttClient->publish(
            mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_PUMP, pump["pumpID"].as<int>(), "").c_str(),
            payload.c_str(),
            true);
        ESP_LOGI("", "topic: %s", mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_PUMP, pump["pumpID"].as<int>(), "").c_str());
        ESP_LOGI("", "payload: %s", payload.c_str());
    }
}

void PumpController::publishState(const int pumpPort)
{

    for (int i = 0; i < pumpPins.size(); ++i)
    {
        JsonObject pump = pump_config_doc["pumps"][i];
        // ESP_LOGI("", "pumpPort: %i vs. doc: %i", pumpPort, pump["pumpID"].as<int>());
        if (pumpPort != VALUE_NOT_SET && pump["pumpID"].as<int>() != pumpPort)
            continue;

        bool state = pumpState[i].load(); // atomic read

        String payload = stateToStr(state);
        // ESP_LOGI("", "state: %i, %d, %s", i, state, payload.c_str());

        mqttClient->publish(
            mqttTopicBuilder(mqttClientID, CLASS_STATE, TOPIC_PUMP, pumpPort, "STATE").c_str(),
            payload.c_str(),
            true);
    }
}

// ====== Private Methods ======

void PumpController::initPins()
{
    for (size_t i = 0; i < pumpPins.size(); ++i)
    {
        pinMode(pumpPins[i], OUTPUT);
        digitalWrite(pumpPins[i], LOW);
        pumpState[i].store(false);
    }
}

bool PumpController::pushJobByVolume(int idx, float ml)
{
    unsigned long seconds = mlToSeconds(idx, ml);
    return pushJobByDuration(idx, seconds);
}

bool PumpController::pushJobByDuration(int idx, uint32_t seconds)
{
    if (idx < 0 || idx >= (int)pumpPins.size() || seconds <= 0)
    {
        ESP_LOGI("", "values not valid: seconds: %u idx: %i", seconds, idx);

        return false;
    }

    if (xSemaphoreTake(pumpQueueMutex, pdMS_TO_TICKS(10)))
    {
        int nextTail = (pumpQueueTail + 1) % MAX_QUEUE_ITEMS;
        if (nextTail == pumpQueueHead)
        {
            xSemaphoreGive(pumpQueueMutex);
            return false; // queue full
        }

        pumpQueue[pumpQueueTail] = {idx, seconds};
        pumpQueueTail = nextTail;
        xSemaphoreGive(pumpQueueMutex);
        return true;
    }
    return false;
}

bool PumpController::popJob(PumpJob &job)
{
    if (isQueueEmpty())
        return false;
    job = pumpQueue[pumpQueueHead];
    pumpQueueHead = (pumpQueueHead + 1) % MAX_QUEUE_ITEMS;
    return true;
}

bool PumpController::isQueueEmpty()
{
    return pumpQueueHead == pumpQueueTail;
}

void PumpController::pumpTask()
{
    PumpJob job;

    while (true)
    {
        bool hasJob = false;

        // Thread-safe pop
        if (xSemaphoreTake(pumpQueueMutex, portMAX_DELAY))
        {
            if (!isQueueEmpty())
            {
                job = pumpQueue[pumpQueueHead];
                pumpQueueHead = (pumpQueueHead + 1) % MAX_QUEUE_ITEMS;
                hasJob = true;
            }
            xSemaphoreGive(pumpQueueMutex);
        }

        if (hasJob)
        {
            jobRunning = true;
            manualStopRequested = false;

            // Stop previous pump if running
            if (activePump != -1 && activePump != job.pumpIndex)
            {
                //                pumpState[activePump] = false;
                pumpState[activePump].store(false);
                digitalWrite(pumpPins[activePump], LOW);
                publishState(activePump + 1);
            }
            // Start current pump
            // pumpState[activePump] = true;
            activePump = job.pumpIndex;
            pumpState[activePump].store(true);
            publishState(activePump + 1);
            digitalWrite(pumpPins[activePump], HIGH);

            unsigned long runStart = millis();
            unsigned long runDuration = job.runSeconds * 1000UL;

            while ((millis() - runStart < runDuration) && !manualStopRequested)
            {
                vTaskDelay(pdMS_TO_TICKS(50)); // non-blocking delay
            }

            // Stop pump
            digitalWrite(pumpPins[job.pumpIndex], LOW);
            // pumpState[activePump] = false;
            pumpState[activePump].store(false);
            publishState(job.pumpIndex + 1);

            if (activePump == job.pumpIndex)
                activePump = -1;

            jobRunning = false;

            if (manualStopRequested)
                stopAndClearQueue();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // idle wait
        }
    }
}

void PumpController::stopAndClearQueue()
{

    // Clear remaining jobs safely
    if (xSemaphoreTake(pumpQueueMutex, portMAX_DELAY))
    {
        pumpQueueHead = pumpQueueTail;
        xSemaphoreGive(pumpQueueMutex);
    }
    if (activePump != -1)
    {
        //                pumpState[activePump] = false;
        pumpState[activePump].store(false);
        digitalWrite(pumpPins[activePump], LOW);
        publishState(activePump + 1);
    }
}

unsigned long PumpController::mlToSeconds(int pumpIndex, float ml) const
{
    if (pumpIndex < 0 || pumpIndex >= (int)pumpPins.size())
        return 0;

    float rate = pumpMlPerMin[pumpIndex];

    // Prevent division by very small rates
    if (rate < 0.1f)
        rate = 1.0f;

    float secs = (ml * 60.0f) / rate; // convert mL/min to seconds

    // Minimum runtime 1 second
    if (secs < 1.0f)
        secs = 1.0f;

    return (unsigned long)roundf(secs);
}

const char *PumpController::stateToStr(bool state)
{
    return state ? "ON" : "OFF";
}
