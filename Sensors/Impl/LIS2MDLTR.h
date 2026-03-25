#pragma once

#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include <LIS2MDLSensor.h>
#include <cstdint>

struct LISM2Data {
    float mag0, mag1, mag2;
};

#define LISM2_ODR 100.0f // not sure on this, should be checked

class LISM2 : public Sensor<LISM2, LISM2Data> {
  public:
    LISM2(SPIClass *spi, uint32_t cs)
        : Sensor(1000.0f / LISM2_ODR), lis2mdl(spi, cs) {}

    bool init_impl() {
        Serial.print("Initialising LISM2... ");

        if (!lis2mdl.begin()) {
            Serial.println("FAILED");
            return false;
        }

        lis2mdl.SetOutputDataRate(LISM2_ODR);

        lis2mdl.Enable();

        return true;
    }

    void poll_impl(uint32_t now_ms, LISM2Data &out) {
        int32_t mag[3];
        lis2mdl.GetAxes(mag);

        out.mag0 = (float)mag[0] / 1000.0f;
        out.mag1 = (float)mag[1] / 1000.0f;
        out.mag2 = (float)mag[2] / 1000.0f;
        // these are in gauss
    }

  private:
    LIS2MDLSensor lis2mdl;
};
