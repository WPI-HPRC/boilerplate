//
// Created by Daniel Coburn on 11/15/24.
//

#pragma once

#include <vector>
#include "../Sensor/Sensor.h"

class SensorManager {
public:
    bool addSensor(Sensor* sensorPtr); // add sensor, true if added, false if not added
    bool removeSensor(Sensor* sensorPtr); // remove sensor, true if success
    void run(); // going to be called
    virtual ~SensorManager() = default; // un-allocation
    bool sensorInit(); // true if success false if something fail

private:
    void** readSensors(); // array of reading pointers
    Time* timer;
    std::vector<Sensor*> sensors;

protected:
};