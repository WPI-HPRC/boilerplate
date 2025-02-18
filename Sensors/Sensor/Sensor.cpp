//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

bool Sensor::init() {
    initStatus = init_impl();
    return initStatus;
}

void Sensor::update(long currentTime) {
    if (initStatus) {
        lastTimeRead = currentTime;
        data = poll();
    }
}

long Sensor::getLastTimeRead() { return lastTimeRead; }

bool Sensor::getInitStatus() { return initStatus; }

long Sensor::getPollingPeriod() { return pollingPeriod; }
