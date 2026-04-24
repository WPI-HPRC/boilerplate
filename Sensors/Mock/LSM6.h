#pragma once

#include "../SensorManager/SensorBase.h"
#include "../Impl/LSM6.h"
#include "Stream.h"
#include "boilerplate/Utilities/CSVParser.h"
#include <Arduino.h>

class MockLSM6 : public Sensor<MockLSM6, LSM6Data> {
    public:
        MockLSM6(Stream *dataFile, float rate) 
        : Sensor(1000.0 / rate), dataFile(dataFile)
          {};

        bool begin_impl() {
            Serial.println("Beginning for Mock LSM6");
            return true;
        }

        bool init_impl() {
            Serial.println("Initializing for Mock LSM6");

            return skipCSVRow(dataFile);
        }

        bool poll_impl(uint32_t now_ms, LSM6Data &out) {
            float vals[6];
            loadCSVRow(dataFile, 6, vals);
            float *acc = vals, *gyr = vals+3;

            out.accel0 = acc[0];
            out.accel1 = acc[1];
            out.accel2 = acc[2];

            out.gyr0 = gyr[0];
            out.gyr1 = gyr[1];
            out.gyr2 = gyr[2];

            return true;
        }

    private:
        Stream *dataFile;
};
