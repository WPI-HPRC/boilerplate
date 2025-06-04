#pragma once

#include "../Sensor/Sensor.h"
#include "Adafruit_Sensor.h"
#include "boilerplate/Logging/Loggable.h"
#include "boilerplate/StateEstimator/kfConsts.h"
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Arduino.h>

struct ICMData {
    float accelX, accelY, accelZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
    float temp;
};

#define ICM_LOG_DESC(X)                                                            \
    X(0, "ICMaccelX", p.print(getData()->accelX, 4))                           \
    X(1, "ICMaccelY", p.print(getData()->accelY, 4))                           \
    X(2, "ICMaccelZ", p.print(getData()->accelZ, 4))                           \
    X(3, "ICMgyrX", p.print(getData()->gyrX, 4))                               \
    X(4, "ICMgyrY", p.print(getData()->gyrY, 4))                               \
    X(5, "ICMgyrZ", p.print(getData()->gyrZ, 4))                               \
    X(6, "ICMmagX", p.print(getData()->magX, 4))                               \
    X(7, "ICMmagY", p.print(getData()->magY, 4))                               \
    X(8, "ICMmagZ", p.print(getData()->magZ, 4))                               \
    X(9, "ICMtemp", p.print(getData()->temp, 4))

#define ODR 40

class ICM20948 : public Sensor, public Loggable {
  public:
    ICM20948()
        : Sensor(sizeof(ICMData), 1000 / ODR), Loggable(NUM_FIELDS(ICM_LOG_DESC)),
          icm() {}

    const TimedPointer<ICMData> getData() const {
        return static_cast<TimedPointer<ICMData>>(data);
    }

  private:
    Adafruit_ICM20948 icm;

    MAKE_LOGGABLE(ICM_LOG_DESC)

    uint32_t dataUpdatedAt() override {
        return getLastTimePolled();
    }

    TimedPointer<ICMData> setData() {
        return static_cast<TimedPointer<ICMData>>(data);
    }

    bool init_impl() override {
        if (!icm.begin_I2C(0x68)) {
            icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
            icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
            uint16_t accelRateDiv =
                1125 / ODR - 1; // Per datasheet: ODR = 1125 / (1 + div)
            uint8_t gyrRateDiv =
                1100 / ODR - 1; // Per datasheet: ODR = 1100 / (1 + div)
            icm.setAccelRateDivisor(accelRateDiv);
            icm.setGyroRateDivisor(gyrRateDiv);
            return false;
        }
        return true;
    }

    void poll() override {
        sensors_event_t accel, gyr, mag, temp;

        icm.getEvent(&accel, &gyr, &temp, &mag);

        setData()->accelX = accel.acceleration.x / g;
        setData()->accelY = accel.acceleration.y / g;
        setData()->accelZ = accel.acceleration.z / g;

        setData()->gyrX = gyr.gyro.x;
        setData()->gyrY = gyr.gyro.y;
        setData()->gyrZ = gyr.gyro.z;

        setData()->magX = mag.magnetic.x;
        setData()->magY = mag.magnetic.y;
        setData()->magZ = mag.magnetic.z;

        setData()->temp = temp.temperature;
    }
};
