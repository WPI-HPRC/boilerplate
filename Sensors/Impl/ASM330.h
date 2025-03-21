#pragma once

#include "../Sensor/Sensor.h"
#include "ASM330LHHSensor.h"
#include <Arduino.h>

struct ASM330Data {
    float xAcc;
    float yAcc;
    float zAcc;

    float xGyr;
    float yGyr;
    float zGyr;
};

class ASM330 : public Sensor {
  public:
    ASM330()
        : Sensor(sizeof(ASM330Data), 40), AccGyr(&Wire, ASM330LHH_I2C_ADD_L) {}
    // this number WILL change probably, i made it up

    ASM330Data getData() { return *(ASM330Data *)data; }

  private:
    ASM330LHHSensor AccGyr;
    bool init_impl() override {
        if (AccGyr.begin()) {
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
