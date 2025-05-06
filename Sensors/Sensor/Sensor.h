//
// Created by Daniel Coburn on 9/27/24.
//

#pragma once

#include "Print.h"
#include <cstdlib>

class Sensor {
  protected:
    long lastTimeRead = 0;
    void *data = nullptr;
    long pollingPeriod;

    /**
     * @brief Polls the sensor.
     * @note _Must_ set fields in struct pointed to by `data`
     */
    virtual void poll() = 0;
    virtual bool init_impl() = 0;

    /**
     * @param dataSize Must be sizeof the struct pointed to by `data`
     */
    Sensor(size_t dataSize, long pollingPeriod)
        : data(malloc(dataSize)), pollingPeriod(pollingPeriod) {}

  public:
    bool initStatus = false;

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

    bool init();

    /**
     * gets polling period of the sensor in millis
     * @return long, the polling period of the sensor in millis
     */
    long getPollingPeriod();

    virtual void debugPrint(Print &) = 0;
    virtual void logCsvHeader(Print &) = 0;
    virtual void logCsvRow(Print &) = 0;

    virtual ~Sensor() = default;
};
