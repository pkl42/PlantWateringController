/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef PLANT_TYPES_H
#define PLANT_TYPES_H

#include <Arduino.h>

struct PumpData
{
    float doseMl;
    unsigned long timestamp; // in seconds
};

struct PlantConfig
{
    int pumpPort;
    int sensorPort;
    int sensorID;

    float potVolume_cm3;
    float fieldCapacity_pct;
    float targetMoisture_pct;
    int autoMode; // 0= OFF, 1=ON
};

struct PlantState
{
    MoistureData lastMoisture;
    PumpData lastWatering;

    unsigned int countTNA; // count target moisture not reached
    unsigned long nextRecheckAtEpoch;
    int statusCode;
};

struct Plant
{
    PlantConfig config;
    PlantState state;
};

#endif // PLANT_TYPES_H
