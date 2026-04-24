#pragma once

#include "../SensorManager/SensorBase.h"
#include "../Impl/ASM330.h"
#include "boilerplate/Utilities/CSVParser.h"
#include <Arduino.h>


class MockASM330 : public Sensor<MockASM330, ASM330Data> {
  public:
    MockASM330(Stream *dataFile, float rate)
        : Sensor(1000.0 / rate), dataFile(dataFile) {}

    bool begin_impl() {
        Serial.println("Beginning ASM330");
        return true;
    }

    bool init_impl() {
        Serial.println("Initializing ASM330");
        return skipCSVRow(dataFile);
    }

    bool poll_impl(uint32_t now_ms, ASM330Data &out) {
        float vals[6];
        loadCSVRow(dataFile, 6, vals);
        float *accel = vals, *gyro = vals+3;

        out.accel0 = accel[0];
        out.accel1 = accel[1];
        out.accel2 = accel[2];
        out.gyr0 = gyro[0];
        out.gyr1 = gyro[1];
        out.gyr2 = gyro[2];

        return true;
    }

  private:
      Stream *dataFile;
};
