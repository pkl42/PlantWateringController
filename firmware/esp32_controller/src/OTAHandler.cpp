/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <Arduino.h>
#include <WiFi.h>
#include "OTAHandler.h"
#include <ArduinoOTA.h>

bool otaInProgress = false; // define global flag

void setupOTA(const char *hostname, const char *password)
{
    ArduinoOTA.setHostname(hostname);

    if (password != nullptr)
    {
        ArduinoOTA.setPassword(password);
    }

    ArduinoOTA
        .onStart([]()
                 {
            otaInProgress = true;   // set flag
            Serial.println("OTA Update Start..."); })
        .onEnd([]()
               {
            otaInProgress = false;  // clear flag
            Serial.println("\nOTA Update Finished"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
            otaInProgress = false;  // reset on error too
            Serial.printf("Error[%u]\n", error); });

    ArduinoOTA.setTimeout(20000); // longer timeout
    ArduinoOTA.begin();
    Serial.println("OTA Ready!");
}

void handleOTA()
{
    ArduinoOTA.handle();
}
