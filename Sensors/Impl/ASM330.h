#pragma once

#include "../Sensor/Sensor.h"
#include "ASM330LHHSensor.h"
#include "Print.h"
#include <Arduino.h>

struct ASM330Data {
    float accelX;
    float accelY;
    float accelZ;

    float gyrX;
    float gyrY;
    float gyrZ;
};

class ASM330 : public Sensor {
  public:
    ASM330()
        : Sensor(sizeof(ASM330Data), 0), AccGyr(&Wire, ASM330LHH_I2C_ADD_H) {}

    const TimedPointer<ASM330Data> getData() const {
        return static_cast<TimedPointer<ASM330Data>>(data);
    }

    // clang-format off
    void debugPrint(Print& p) override {
        p.print("accelX: "); p.print(getData()->accelX, 4); p.print(", ");
        p.print("accelY: "); p.print(getData()->accelY, 4); p.print(", ");
        p.print("accelZ: "); p.print(getData()->accelZ, 4); p.print(", ");

        p.print("gyrX: "); p.print(getData()->gyrX, 4); p.print(", ");
        p.print("gyrY: "); p.print(getData()->gyrY, 4); p.print(", ");
        p.print("gyrZ: "); p.print(getData()->gyrZ, 4); p.println();
    };

    void logCsvHeader(Print& p) override {
        p.print("ASMaccelX,ASMaccelY,ASMaccelZ,ASMgyrX,ASMgyrY,ASMgyrZ");
    }

    void logCsvRow(Print& p) override {
        p.print(getData()->accelX, 4); p.print(",");
        p.print(getData()->accelY, 4); p.print(",");
        p.print(getData()->accelZ, 4); p.print(",");

        p.print(getData()->gyrX, 4); p.print(",");
        p.print(getData()->gyrY, 4); p.print(",");
        p.print(getData()->gyrZ, 4);
    }
    // clang-format on

  private:
    TimedPointer<ASM330Data> setData() {
        return static_cast<TimedPointer<ASM330Data>>(data);
    }

    ASM330LHHSensor AccGyr;
    bool init_impl() override {
        if (AccGyr.begin() == 0) {
            AccGyr.Set_X_ODR(52.0);
            AccGyr.Set_G_ODR(52.0);
            AccGyr.Set_X_FS(16);
            AccGyr.Set_G_FS(2000);
            AccGyr.Enable_X();
            AccGyr.Enable_G();

            float odr;
            AccGyr.Get_X_ODR(&odr);
            pollingPeriod = 1000 / odr;

            return true;
        }
        return false;
        // will we want to change the logic for this?
        // feel like there should be some check at the end that it works as well
    }

    void poll() override {
        static int32_t accelerometer[3] = {};
        static int32_t gyroscope[3] = {};
        AccGyr.Get_X_Axes(accelerometer);
        AccGyr.Get_G_Axes(gyroscope);

        setData()->accelX = (float)accelerometer[0] / 1000.0;
        setData()->accelY = (float)accelerometer[1] / 1000.0;
        setData()->accelZ = (float)accelerometer[2] / 1000.0;

        setData()->gyrX = (float)gyroscope[0] / 1000.0;
        setData()->gyrY = (float)gyroscope[1] / 1000.0;
        setData()->gyrZ = (float)gyroscope[2] / 1000.0;
    }
};
