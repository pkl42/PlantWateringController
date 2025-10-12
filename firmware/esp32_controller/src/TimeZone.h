/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TIME_ZONE_H
#define TIME_ZONE_H
#include <Arduino.h>
#include "time.h"

// persists across deep sleep cycles
RTC_DATA_ATTR static time_t lastSyncEpoch = 0;

class TimeZone
{
public:
    TimeZone(const char *ntpServer, const char *tzInfo);

    bool isTimeSet();
    void syncTime();                               // call once in a while (Wi-Fi must already be connected)
    void maybeResync(uint32_t intervalHours = 24); // auto-sync if interval passed
    void printLocalTime();
    tm getLocalTimeStruct(); // returns struct with hour/minute/second
    String getLocalTimeString();

    String formatTimeISO8601(time_t epoch);
    String millisToISO8601(unsigned long eventMillis);
    String epochToISO8601(unsigned long epochSec);

    time_t millisToEpoch(unsigned long eventMillis);

    unsigned long getNow(bool &utcFlag);

private:
    const char *TAG = "TimeZone";
    const char *_ntpServer;
    const char *_tzInfo;
};

#endif // TIME_ZONE_H