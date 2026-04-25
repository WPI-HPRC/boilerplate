#pragma once

#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include "../Impl/LIS2MDLTR.h"
#include "STM32SD.h"
#include "boilerplate/Utilities/CSVParser.h"
#include <cstdint>

class MockLIS2MDL : public Sensor<MockLIS2MDL, LIS2MDLData> {
  public:
    MockLIS2MDL(const char *filename, float rate)
        : Sensor(1000.0f / rate), filename(filename) {}

    bool begin_impl() {
        Log.infoln("Beginning LISM2");
        dataFile = SD.open(filename);
        return true;
    }

    bool init_impl() {
        Log.infoln("Initialising LISM2");
        return true;
    }

    bool poll_impl(uint32_t now_ms, LIS2MDLData &out) {
        float mag[3];
        loadCSVRow(&dataFile, 3, mag);

        // these are in gauss
        out.mag0 = mag[0];
        out.mag1 = mag[1];
        out.mag2 = mag[2];

        return true;
    }

  private:
      const char *filename;
      File dataFile;
};
