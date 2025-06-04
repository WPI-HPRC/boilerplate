#pragma once

#include "../Sensor/Sensor.h"
#include "ASM330LHHSensor.h"
#include "Print.h"
#include "boilerplate/Logging/Loggable.h"
#include <Arduino.h>

struct ASM330Data {
    float accelX;
    float accelY;
    float accelZ;

    float gyrX;
    float gyrY;
    float gyrZ;
};

#define ASM330_LOG_DESC(X)                                                            \
    X(0, "ASMaccelX", p.print(getData()->accelX, 3))                           \
    X(1, "ASMaccelY", p.print(getData()->accelY, 3))                           \
    X(2, "ASMaccelZ", p.print(getData()->accelZ, 3))                           \
    X(3, "ASMgyrX", p.print(getData()->gyrX, 3))                               \
    X(4, "ASMgyrY", p.print(getData()->gyrY, 3))                               \
    X(5, "ASMgyrZ", p.print(getData()->gyrZ, 3))

class ASM330 : public Sensor, public Loggable {
  public:
    ASM330()
        : Sensor(sizeof(ASM330Data), 0), Loggable(NUM_FIELDS(ASM330_LOG_DESC)),
          AccGyr(&Wire, ASM330LHH_I2C_ADD_H) {}

    const TimedPointer<ASM330Data> getData() const {
        return static_cast<TimedPointer<ASM330Data>>(data);
    }

  private:
    TimedPointer<ASM330Data> setData() {
        return static_cast<TimedPointer<ASM330Data>>(data);
    }

    MAKE_LOGGABLE(ASM330_LOG_DESC)

    uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

    ASM330LHHSensor AccGyr;
    bool init_impl() override {
        if (AccGyr.begin() == 0) {
            AccGyr.Set_X_ODR(26.0);
            AccGyr.Set_G_ODR(26.0);
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

        // X and Y axes rotated to match ICM orientation
        setData()->accelX = -(float)accelerometer[1] / 1000.0;
        setData()->accelY = (float)accelerometer[0] / 1000.0;
        setData()->accelZ = (float)accelerometer[2] / 1000.0;

        // X and Y axes rotated to match ICM orientation
        setData()->gyrX = -(float)gyroscope[1] / 1000.0;
        setData()->gyrY = (float)gyroscope[0] / 1000.0;
        setData()->gyrZ = (float)gyroscope[2] / 1000.0;
    }
};
