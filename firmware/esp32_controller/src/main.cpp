/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "PlantController.h"
#include <configFile.h>
#include <DualWifiMode.h>
#include "WebServerBase.h"
#include "OTAHandler.h"

JsonDocument config_doc;
MyAppConfig config("/config.cfg");
WebConfigServer configServer(80);

TimeZone tz("pool.ntp.org", "CET-1CEST,M3.5.0,M10.5.0/3");
PlantController controller(tz);

// Manual IP Configuration for Soft AP
IPAddress AP_LOCAL_IP(192, 168, 5, 10);
IPAddress AP_GATEWAY_IP(192, 168, 5, 254);
// Url for access: http://192.168.5.10/
// Define Pin to indicate start option
const uint8_t AP_PIN = 23; // do not use GPIO11!!!-crashes

void setup()
{
  WiFiModeStatus wifi_status;
  Serial.begin(115200);
  delay(200);
  Serial.println("=== ESP32 Plant Watering Controller ===");

  // config.remove();
  config_doc = config.read_config();

  initDualWiFiConfig(AP_PIN,
                     config_doc["ap_ssid"],
                     config_doc["ap_password"],
                     config_doc["ln_ssid"],
                     config_doc["ln_password"],
                     AP_LOCAL_IP,
                     AP_GATEWAY_IP);

  wifi_status = startDualWiFi();

  if (wifi_status == WiFiModeStatus::AP)
  {
    configServer.setDefaults(config_doc);
    configServer.onConfigSaved([&](const JsonDocument &doc)
                               { config.write_config(&doc); });
    Serial.println("start");
    configServer.start();
  }

  // Init OTA (you can also pass hostname and password if desired)
  setupOTA("esp32-ota", nullptr);

  // Init Plant Controller

  controller.init(config_doc["mqtt_broker_ip"],
                  static_cast<uint16_t>(config_doc["mqtt_broker_port"].as<int>()));
}

void loop()
{
  if (!otaInProgress)
  { // only run normal tasks if OTA is not happening
    // In deep sleep mode, run() will sleep at the end
    // In continuous mode, run() will return immediately and be called again
    if (controller.connectToMQTTBroker())
    {
      controller.subscribeCmndTopics();
      controller.run();
    }
  }

  handleOTA(); // must be called regularly
}
