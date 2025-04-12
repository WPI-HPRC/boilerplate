#pragma once

#include "../Sensor/Sensor.h"
#include "ASM330LHHSensor.h"
#include "Print.h"
#include <Arduino.h>

struct ASM330Data {
    double xAcc;
    double yAcc;
    double zAcc;

    double xGyr;
    double yGyr;
    double zGyr;
};

class ASM330 : public Sensor {
  public:
    ASM330()
        : Sensor(sizeof(ASM330Data), 40), AccGyr(&Wire, ASM330LHH_I2C_ADD_H) {}
    // this number WILL change probably, i made it up

    ASM330Data getData() { return *(ASM330Data *)data; }

    void debugPrint(Print& p) {
        p.print("xAcc: "); p.print(((ASM330Data *)data)->xAcc, 4); p.print(", ");
        p.print("yAcc: "); p.print(((ASM330Data *)data)->yAcc, 4); p.print(", ");
        p.print("zAcc: "); p.print(((ASM330Data *)data)->zAcc, 4); p.print(", ");

        p.print("xGyr: "); p.print(((ASM330Data *)data)->xGyr, 4); p.print(", ");
        p.print("yGyr: "); p.print(((ASM330Data *)data)->yGyr, 4); p.print(", ");
        p.print("zGyr: "); p.print(((ASM330Data *)data)->zGyr, 4); p.println();
    };

  private:
    ASM330LHHSensor AccGyr;
    bool init_impl() override {
        if (AccGyr.begin() == 0) {
            AccGyr.Enable_X();
            AccGyr.Enable_G();

            return true;
        }
        return false;
        // will we want to change the logic for this?
        // feel like there should be some check at the end that it works as well
    }

    void *poll() override {
        // this logic will probably need to change? seems wastefull to me
        int32_t accelerometer[3] = {};
        int32_t gyroscope[3] = {};
        AccGyr.Get_X_Axes(accelerometer);
        AccGyr.Get_G_Axes(gyroscope);

        // i am not sure if this is accurate will need to confirm
        // will type casting matter here??
        // if so in the actual function to get the data it is possible to remove
        // the cast to int32 if we want to
        // we can also change sensitivity there to help with noise??
        ((ASM330Data *)data)->xAcc = accelerometer[0];
        ((ASM330Data *)data)->yAcc = accelerometer[1];
        ((ASM330Data *)data)->zAcc = accelerometer[2];

        ((ASM330Data *)data)->xGyr = gyroscope[0];
        ((ASM330Data *)data)->yGyr = gyroscope[1];
        ((ASM330Data *)data)->zGyr = gyroscope[2];

        return data;

    }
};
