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

    ASM330Data getData() { return *(ASM330Data *)data; }

    void debugPrint(Print& p) override {
        p.print("accelX: "); p.print(((ASM330Data *)data)->accelX, 4); p.print(", ");
        p.print("accelY: "); p.print(((ASM330Data *)data)->accelY, 4); p.print(", ");
        p.print("accelZ: "); p.print(((ASM330Data *)data)->accelZ, 4); p.print(", ");

        p.print("gyrX: "); p.print(((ASM330Data *)data)->gyrX, 4); p.print(", ");
        p.print("gyrY: "); p.print(((ASM330Data *)data)->gyrY, 4); p.print(", ");
        p.print("gyrZ: "); p.print(((ASM330Data *)data)->gyrZ, 4); p.println();
    };

    void logCsvHeader(Print& p) override {
        p.print("ASMaccelX,ASMaccelY,ASMaccelZ,ASMgyrX,ASMgyrY,ASMgyrZ");
    }

    void logCsvRow(Print& p) override {
        p.print(((ASM330Data *)data)->accelX, 4); p.print(",");
        p.print(((ASM330Data *)data)->accelY, 4); p.print(",");
        p.print(((ASM330Data *)data)->accelZ, 4); p.print(",");

        p.print(((ASM330Data *)data)->gyrX, 4); p.print(",");
        p.print(((ASM330Data *)data)->gyrY, 4); p.print(",");
        p.print(((ASM330Data *)data)->gyrZ, 4);
    }

  private:
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

    void *poll() override {
        static int32_t accelerometer[3] = {};
        static int32_t gyroscope[3] = {};
        AccGyr.Get_X_Axes(accelerometer);
        AccGyr.Get_G_Axes(gyroscope);

        ((ASM330Data *)data)->accelX = (float)accelerometer[0] / 1000.0;
        ((ASM330Data *)data)->accelY = (float)accelerometer[1] / 1000.0;
        ((ASM330Data *)data)->accelZ = (float)accelerometer[2] / 1000.0;

        ((ASM330Data *)data)->gyrX = (float)gyroscope[0] / 1000.0;
        ((ASM330Data *)data)->gyrY = (float)gyroscope[1] / 1000.0;
        ((ASM330Data *)data)->gyrZ = (float)gyroscope[2] / 1000.0;

        return data;

    }
};
