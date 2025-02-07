//
// Created by Daniel Coburn on 11/15/24.
//

#pragma once

#include <vector>
#include "../Sensor/Sensor.h"
#include <Arduino.h>

template<typename MillisFn> class SensorManager {
public:

    /**
    @brief Run before use 
    */
    void initialize() {
        this->startTime = millis();
        initialize_impl();
    }

    // adds a new sensor to the SensorManager
    bool addSensor(Sensor* sensorPtr) {
        auto it = std::find(sensors.begin(), sensors.end(), sensorPtr);

        if (it == sensors.end()) {
            sensors.push_back(sensorPtr);
            return true;
        }

        return false;        
    }

    // read the sensors
    void** loop() {
        void** data = new void*[sensors.size()];

        for (size_t i = 0; i < sensors.size(); i++) {
            if (sensors[i]->getInitStatus()) {
                long currentTime = this->millis();
                if (sensors[i]->getLastTimeRead() + sensors[i]->getPollingPeriod() >= currentTime) {
                    data[i] = sensors[i]->update(currentTime);

                }
            }
        }        
    }
        
    bool sensorInit() {
        bool success = true;
        for(size_t i = 0; i< sensors.size(); i++) {
            sensors[i]->init();
            success = success && sensors[i]->getInitStatus();
        }
        return success;
    } // true if success false if something fail

private:
    void** readSensors(); // array of reading pointers
    std::vector<Sensor*> sensors;
    virtual void initialize_impl() = 0;
    MillisFn millis; // time

protected:
};
