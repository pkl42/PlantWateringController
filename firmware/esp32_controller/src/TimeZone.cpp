/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "TimeZone.h"
#include <WiFi.h>

#include "esp_log.h"

TimeZone::TimeZone(const char *ntpServer, const char *tzInfo)
{
    _ntpServer = ntpServer;
    _tzInfo = tzInfo;

    // Apply timezone immediately
    setenv("TZ", _tzInfo, 1);
    tzset();
}

bool TimeZone::isTimeSet()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        return false;
    }
    return (timeinfo.tm_year > (2016 - 1900));
}

void TimeZone::syncTime()
{
    configTime(0, 0, _ntpServer); // sync UTC time
    delay(2000);                  // give SNTP time to sync
    setenv("TZ", _tzInfo, 1);
    tzset();
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
        ESP_LOGI("", "Time synchronized with NTP.");
    }
    else
    {
        ESP_LOGI("", "Failed to sync with NTP.");
    }

    time_t now;
    time(&now);
    ESP_LOGI("", "Epoch now: %lu", (unsigned long)now);
}

void TimeZone::maybeResync(uint32_t intervalHours)
{
    time_t now = time(nullptr);
    if (!isTimeSet() || (now - lastSyncEpoch) > (intervalHours * 3600))
    {
        ESP_LOGI("", "More than %u hours since last sync → resyncing...\n", intervalHours);
        syncTime();
    }
    else
    {
        ESP_LOGI("", "No resync needed, RTC is fine.");
    }
}

void TimeZone::printLocalTime()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        ESP_LOGI("", "Failed to obtain time");
        return;
    }
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.println(buf);
}

tm TimeZone::getLocalTimeStruct()
{
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    return timeinfo;
}

String TimeZone::getLocalTimeString()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        return "1970-01-01T00:00:00";
    }

    char buffer[30];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &timeinfo);
    return String(buffer);
}

String TimeZone::formatTimeISO8601(time_t epoch)
{
    if (epoch <= 0)
    {
        return "N/A"; // safer than empty string
    }

    struct tm timeinfo;
    localtime_r(&epoch, &timeinfo);

    // First format without timezone
    char buffer[40];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    // Now append timezone with colon
    char tzbuf[10];
    strftime(tzbuf, sizeof(tzbuf), "%z", &timeinfo); // e.g. +0200

    // Insert colon in timezone
    String tzStr = String(tzbuf);
    if (tzStr.length() == 5)
    {
        tzStr = tzStr.substring(0, 3) + ":" + tzStr.substring(3);
    }

    return String(buffer) + tzStr; // e.g. 2025-09-26T18:42:10+02:00
}

// Convert a millis() timestamp to ISO8601 string for telemetry
// Returns empty string if eventMillis == 0
String TimeZone::millisToISO8601(unsigned long eventMillis)
{
    if (eventMillis == 0)
    {
        return "N/A"; // never happened
    }

    time_t now = time(nullptr);
    long delta = (long)eventMillis - (long)millis();

    // If eventMillis is in the past → subtract
    // If eventMillis is in the future → add
    time_t eventEpoch = now + (delta / 1000);

    return formatTimeISO8601(eventEpoch);
}

String TimeZone::epochToISO8601(unsigned long epochSec)
{
    if (epochSec == 0)
    {
        return "N/A"; // never happened
    }

    return formatTimeISO8601((time_t)epochSec);
}

time_t TimeZone::millisToEpoch(unsigned long eventMillis)
{
    if (eventMillis == 0)
    {
        return 0; // never happened
    }

    time_t now = time(nullptr);
    long delta = (long)eventMillis - (long)millis();
    time_t eventEpoch = now + (delta / 1000);

    // Sanity check: reject if > ±30 days away
    const time_t MAX_OFFSET = 30L * 24L * 3600L; // 30 days
    if (eventEpoch < (now - MAX_OFFSET) || eventEpoch > (now + MAX_OFFSET))
    {
        ESP_LOGW("TimeZone", "Suspicious millis→epoch conversion: eventEpoch=%ld (now=%ld, delta=%ld sec)",
                 eventEpoch, now, delta / 1000);
        return 0;
    }

    return eventEpoch;
}

unsigned long TimeZone::getNow(bool &utcFlag)
{
    time_t now;
    if (time(&now)) // returns UTC epoch seconds
    {
        utcFlag = true;
        // ESP_LOGI("", "getNow in utc %lu", (unsigned long)now);
        return (unsigned long)now; // seconds only
    }
    else
    {
        utcFlag = false;
        return millis() / 1000UL; // fallback: approx seconds since boot
    }
}
