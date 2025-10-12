/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

// first upload to ESP32 with old setting
// after that change the OTA Settings in .ini file
//
// [env:esp32dev] platform = espressif32
//     board = esp32dev
//         framework = arduino

//     ;
// Serial monitor options
//     monitor_speed = 115200

//     ;
// OTA settings
//     upload_protocol = espota
//         upload_port = 192.168.1.123;
// replace with your ESP32’s IP

// in the main.cpp add
//
// void setup() {
//     ...
//     // Init OTA (you can also pass hostname and password if desired)
//     setupOTA("esp32-ota", nullptr);
// }

// void loop() {
//     handleOTA(); // must be called regularly
// }

extern bool otaInProgress; // declare global flag

void setupOTA(const char *hostname = "esp32-ota", const char *password = nullptr);
void handleOTA();

#endif
