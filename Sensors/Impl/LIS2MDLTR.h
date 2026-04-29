#pragma once

#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include <LIS2MDLSensor.h>
#include <cstdint>

struct LIS2MDLData {
    float mag0, mag1, mag2;
};

#define LISM2_ODR 100.0f // not sure on this, should be checked

class LIS2MDL : public Sensor<LIS2MDL, LIS2MDLData> {
  public:
    LIS2MDL(SPIClass *spi, uint32_t cs)
        : Sensor(1000.0f / LISM2_ODR), lis2mdl(spi, cs) {}

    bool begin_impl() {
        Log.infoln("Beginning LISM2");

        if (lis2mdl.begin() != LIS2MDL_OK) {
            return false;
        }
        return true;
    }

    bool init_impl() {
        Log.infoln("Initialising LISM2");

        lis2mdl.SetOutputDataRate(LISM2_ODR);

        lis2mdl.Enable();

        return true;
    }

    bool poll_impl(uint32_t now_ms, LIS2MDLData &out) {
        int32_t mag[3];
        lis2mdl.GetAxes(mag);

        // these are in gauss
        out.mag0 = (float)mag[0] / 1000.0f;
        out.mag1 = (float)mag[1] / 1000.0f;
        out.mag2 = (float)mag[2] / 1000.0f;

        return true;
    }

  private:
    LIS2MDLSensor lis2mdl;
};
