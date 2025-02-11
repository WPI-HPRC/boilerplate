//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "../../Services/Time.h"
#include <cstdlib>

class Sensor {
  protected:
    long lastTimeRead = 0;
    void *data = nullptr;

    virtual void *poll() = 0;

  public:
    bool initStatus = false;
    /**
     * @param data allocated pointer to store sensor data in.
     * Must be `sensorDataBytes()` bytes long.
     */
    Sensor(void *data) : data(data) {}

    /**
     * gets if the sensor was initialized
     * @return whether or not the sensor was initialized
     */
    bool getInitStatus();

    /**
     *
     * @param currentTime current time of the rocket
     */
    void update(long currentTime);

    /**
     * gets the last time the sensor was read in millis
     * @return long, the last time the sensor was read in millis
     */
    long getLastTimeRead();

    virtual bool init() = 0;

    /**
     * Must be overriden by subclass for specific type of data that
     * that sensor stores
    */
    template <typename T>
    T getData() { __builtin_unreachable(); }

    /**
     * gets polling period of the sensor in millis
     * @return long, the polling period of the sensor in millis
     */
    virtual long getPollingPeriod() = 0;

    /**
     * @returns size of this sensor's data object
     */
    virtual size_t sensorDataBytes() const = 0;

    virtual ~Sensor() = default;
};
