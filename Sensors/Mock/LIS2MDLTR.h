#pragma once

#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include "../Impl/LIS2MDLTR.h"
#include "boilerplate/Utilities/CSVParser.h"
#include <cstdint>

class MockLIS2MDL : public Sensor<MockLIS2MDL, LIS2MDLData> {
  public:
    MockLIS2MDL(Stream *dataFile, float rate)
        : Sensor(1000.0f / rate), dataFile(dataFile) {}

    bool begin_impl() {
        Serial.println("Beginning LISM2");

        return true;
    }

    bool init_impl() {
        Serial.println("Initialising LISM2");
        return skipCSVRow(dataFile);
    }

    bool poll_impl(uint32_t now_ms, LIS2MDLData &out) {
        float mag[3];
        loadCSVRow(dataFile, 3, mag);

        // these are in gauss
        out.mag0 = mag[0];
        out.mag1 = mag[1];
        out.mag2 = mag[2];

        return true;
    }

  private:
      Stream *dataFile;
};
