/*
 * SPDX-FileCopyrightText: 2025 Peter Ludwig
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

class UltrasonicSensor
{
public:
    UltrasonicSensor(int trigPin, int echoPin, int sensorPower5VPin = -1);
    void init(float distEmptyCM = -1., float distFullCM = -1.);

    int readRaw();
    float readPercent();
    int rawDry;
    int rawWet;

    void setDistEmptyCM(float distEmptyCM) { this->distEmptyCM = distEmptyCM; }
    void setDistFullCM(float distFullCM) { this->distFullCM = distFullCM; }

    long readDistanceCm();
    int readLevelPercent(); // 0-100 or -1 if disabled

private:
    int sensorPower5VPin = -1; // set to -1 to disable power gating
    int trigPin;               // set to -1 to disable ultrasonic
    int echoPin;               // set to -1 to disable ultrasonic

    float distFullCM;
    float distEmptyCM;
};

#endif // ULTRASONIC_SENSOR_H

// const int sensorPower5VPin = 7; // set to -1 to disable power gating
// const int trigPin = 9;          // set to -1 to disable ultrasonic
// const int echoPin = 10;
