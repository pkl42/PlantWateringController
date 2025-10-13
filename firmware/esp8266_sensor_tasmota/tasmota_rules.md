# Tasmota Rules for Moisture Sensor

These rules control the ESP8266 moisture sensor behavior, MQTT reporting, and deep sleep cycle.

---

## Rule1

~~~text
Rule1
ON System#Boot DO Backlog Power1 1; TelePeriod 30; Var3 2; DeepSleepTime 0 ENDON

ON Mqtt#Connected DO RuleTimer1 3 ENDON
ON Mqtt#Connected DO RuleTimer2 20 ENDON

ON Mem1#state= DO Backlog IF ($Mem1 == "") Mem1 752 ENDON
ON Mem2#state= DO Backlog IF ($Mem2 == "") Mem2 283 ENDON

ON Analog#A0 DO Var1 %value% ENDON

ON Rules#Timer=1 DO 
 Backlog
 publish2 tele/%topic%/MOISTURE {"Time":"%timestamp%","utctime":%utctime%,"adc":%Var1%};
 Var3 = %Var3%-1;

 IF ((%Mem3%>0) && (%Var3%<1)) Backlog Publish stat/%topic%/debug Going_to_sleep; DeepSleepTime %Mem3% ENDIF;
 IF (%Mem3%==0 || %Var3%>0) Backlog Publish stat/%topic%/debug Restart_Timer1; RuleTimer1 10 ENDIF;
ENDON
~~~

**Explanation**

| Variable       | Description                                                  |
| -------------- | ------------------------------------------------------------ |
| `Var1`         | Stores latest ADC (moisture) value                           |
| `Var3`         | Countdown until deep sleep                                   |
| `Mem1`, `Mem2` | Optional calibration memory values                           |
| `Mem3`         | DeepSleepTime in seconds (set via MQTT)                      |
| `TelePeriod`   | Telemetry interval                                           |
| `Power1`       | Turns sensor supply pin ON                                   |
| `RuleTimer1`   | Timer controlling when moisture data is sent                 |
| `RuleTimer2`   | Secondary timer (optional for MQTT connection stabilization) |

**MQTT Command to Set Sleep Time**

cmnd/<device_topic>/Mem3 <seconds>

~~~text
cmnd/MS_594908/Mem3 600
~~~

→ Sleeps for 10 minutes after sending next telemetry packet.

Debug Output

~~~text
stat/MS_594908/debug Restart_Timer1
stat/MS_594908/debug Going_to_sleep
~~~

These messages are useful for monitoring device behavior in Node-RED or the MQTT broker.

---

### **firmware/esp8266_sensor_tasmota/tele_format_example.json**

~~~json
{
  "Time": "2025-10-06T15:29:58",
  "utctime": 1759760998,
  "adc": 268
}
~~~
