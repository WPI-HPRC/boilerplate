//
// Created by Daniel Coburn on 9/27/24.
//

#include "Sensor.h"

void Sensor::update(long currentTime) {
    if (initStatus) {
        lastTimeRead = currentTime;
        data = poll();
    }
}

long Sensor::getLastTimeRead() {
    return lastTimeRead;
}

bool Sensor::getInitStatus() {
    return initStatus;
}
