/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "Arduino.h"
#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(int trigPin, int echoPin, int sensorPower5VPin) : trigPin(trigPin), echoPin(echoPin), sensorPower5VPin(sensorPower5VPin)
{
}

void UltrasonicSensor::init(float distEmptyCM, float distFullCM)
{
    setDistEmptyCM(distEmptyCM);
    setDistFullCM(distFullCM);
    if (trigPin >= 0 && echoPin >= 0)
    {
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
        pinMode(sensorPower5VPin, OUTPUT);
        digitalWrite(trigPin, LOW);
    }
}

long UltrasonicSensor::readDistanceCm()
{
    // Turn sensor power on
    digitalWrite(sensorPower5VPin, HIGH);

    // Give sensor time to stabilize
    delay(50); // 50 ms is safe, you can test if less works reliably

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

    // Turn sensor off again to save power
    digitalWrite(sensorPower5VPin, LOW);

    return (duration == 0) ? -1 : duration / 58;
}

int UltrasonicSensor::readLevelPercent()
{
    long d = readDistanceCm();
    if (d < 0)
        return -1;

    if (d <= distFullCM)
        return 100;
    if (d >= distEmptyCM)
        return 0;
    float pct = 100.0f * (distEmptyCM - d) / (distEmptyCM - distFullCM);
    if (pct < 0)
        pct = 0;
    if (pct > 100)
        pct = 100;
    return (int)roundf(pct);
}
