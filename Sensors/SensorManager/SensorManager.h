//
// Created by Daniel Coburn on 11/15/24.
//

#pragma once

#include "../Sensor/Sensor.h"
#include <vector>

template <typename MillisFn> class SensorManager {
  public:
    SensorManager<MillisFn>(std::vector<Sensor *> sensors, MillisFn millis)
        : sensors(sensors), millis(millis) {}

    // read the sensors
    void loop() {
        for (size_t i = 0; i < sensors.size(); i++) {
            if (sensors[i]->getInitStatus()) {
                long currentTime = this->millis();
                if (currentTime - sensors[i]->getLastTimeRead() >=
                    sensors[i]->getPollingPeriod()) {
                    sensors[i]->update(currentTime);
                }
            }
        }
    }

    bool sensorInit() {
        bool success = true;
        for (size_t i = 0; i < sensors.size(); i++) {
            sensors[i]->initStatus = sensors[i]->init();
            success = success && sensors[i]->getInitStatus();
        }
        return success;
    } // true if success false if something fail

  private:
    void **readSensors(); // array of reading pointers
    const std::vector<Sensor *> sensors;
    MillisFn millis; // time
    long long currentTime = 0;

  protected:
};
