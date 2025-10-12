/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// Example usage
// #include "PumpController.h"

// // For example: 3 pumps
// std::vector<int> pumpPins = {13, 12, 14};
// PumpController pumpController(pumpPins);

// void setup() {
//     // Calibrate one pump
//     pumpController.setpumpMlPerMin(0, 104.f);

//     // Queue a job
//     pumpController.pushJobByVolume(0, 100.0f); // 100 ml on pump 0
// }

#ifndef PUMP_CONTROLLER_H
#define PUMP_CONTROLLER_H

#include "mqttHelper.h"

#include <vector>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pumpConfig.h"

#define DEFAULT_MAX_MANUAL_SECONDS 300 // 5 minutes safety cutoff

class PumpController
{
public:
    PumpController(const std::vector<int> &pins, float defaultMlPerMin = 104.f);
    void init(PubSubClient &mqttClient, const String &mqttClientID);

    bool setPumpState(unsigned int pumpIndex, bool state);
    bool isQueueEmpty();
    void stopAndClearQueue();
    bool pushJobByVolume(int idx, float ml);
    bool pushJobByDuration(int idx, uint32_t seconds);

    volatile bool jobRunning = false;          // true while a job is active
    volatile bool manualStopRequested = false; // set by manual OFF

    int getActivePump() const { return activePump; }

    bool setPumpMlPerMin(unsigned int pumpIndex, float mlPerMin);
    float getPumpMlPerMin(unsigned int pumpIndex) const;

    size_t pumpCount() const { return pumpPins.size(); }

    void processMQTTMessage(const std::vector<String> &topicParts, const String &payload);
    void publishSetup(const int pumpPort = VALUE_NOT_SET);
    void publishState(const int pumpPort = VALUE_NOT_SET);

private:
    const char *TAG = "PumpController";
    const char *TOPIC_PUMP = "PUMP";

    void initPins();
    PumpConfig pumpConfig; // declare without parameters
    JsonDocument pump_config_doc;

    // Dynamic pump configuration
    std::vector<int> pumpPins;
    std::vector<float> pumpMlPerMin;
    std::vector<std::atomic<bool>> pumpState;

    // Helper to convert state to text
    const char *stateToStr(bool state);

    // Pump queue
    struct PumpJob
    {
        int pumpIndex;
        unsigned long runSeconds;
    };
    static const int MAX_QUEUE_ITEMS = 16;
    PumpJob pumpQueue[MAX_QUEUE_ITEMS];
    int pumpQueueHead = 0;
    int pumpQueueTail = 0;
    int activePump = -1; // -1 means no pump running

    SemaphoreHandle_t pumpQueueMutex;

    void pumpTask();

    bool popJob(PumpJob &job);
    unsigned long mlToSeconds(int pumpIndex, float ml) const;

    PubSubClient *mqttClient = nullptr;
    String mqttClientID;
};

#endif // PUMP_CONTROLLER_H
