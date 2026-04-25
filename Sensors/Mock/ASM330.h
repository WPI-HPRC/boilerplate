#pragma once

#include "../SensorManager/SensorBase.h"
#include "../Impl/ASM330.h"
#include "STM32SD.h"
#include "boilerplate/Utilities/CSVParser.h"
#include <Arduino.h>


class MockASM330 : public Sensor<MockASM330, ASM330Data> {
  public:
    MockASM330(const char *filename, float rate)
        : Sensor(1000.0 / rate), filename(filename) {}

    bool begin_impl() {
        Log.infoln("Beginning ASM330");

        dataFile = SD.open(filename);
        return true;
    }

    bool init_impl() {
        Log.infoln("Initializing ASM330");
        return true;
    }

    bool poll_impl(uint32_t now_ms, ASM330Data &out) {
        float vals[6];
        loadCSVRow(&dataFile, 6, vals);
        float *accel = vals+3, *gyro = vals;

        out.accel0 = accel[0];
        out.accel1 = accel[1];
        out.accel2 = accel[2];
        out.gyr0 = gyro[0];
        out.gyr1 = gyro[1];
        out.gyr2 = gyro[2];

        return true;
    }

  private:
      const char *filename;
      File dataFile;
};
