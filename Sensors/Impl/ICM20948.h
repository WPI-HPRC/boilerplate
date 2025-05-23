#pragma once

#include "../Sensor/Sensor.h"
#include "Adafruit_Sensor.h"
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Arduino.h>

struct ICMData {
    float accelX, accelY, accelZ;
    float gyrX, gyrY, gyrZ;
    float magX, magY, magZ;
    float temp;
};

#define ODR 40

class ICM20948 : public Sensor {
  public:
    ICM20948() : Sensor(sizeof(ICMData), 1000 / ODR), icm() {}

    const TimedPointer<ICMData> getData() const {
        return static_cast<TimedPointer<ICMData>>(data);
    }

    // clang-format off
    void debugPrint(Print& p) override {
      p.print("accelX: "); p.print(getData()->accelX, 4); p.print(", ");
      p.print("accelY: "); p.print(getData()->accelY, 4); p.print(", ");
      p.print("accelZ: "); p.print(getData()->accelZ, 4); p.print(", ");
      p.print("gyrX: "); p.print(getData()->gyrX, 4); p.print(", ");
      p.print("gyrY: "); p.print(getData()->gyrY, 4); p.print(", ");
      p.print("gyrZ: "); p.print(getData()->gyrZ, 4); p.print(", ");
      p.print("magX: "); p.print(getData()->magX, 4); p.print(", ");
      p.print("magY: "); p.print(getData()->magY, 4); p.print(", ");
      p.print("magZ: "); p.print(getData()->magZ, 4); p.print(", ");
      p.print("temp: "); p.print(getData()->temp, 4); p.println();
    }

    void logCsvHeader(Print& p) override {
      p.print("ICMaccelX,ICMaccelY,ICMaccelZ,ICMgyrX,ICMgyrY,ICMgyrZ,magX,magY,magZ,ICMtemp");
    }

    void logCsvRow(Print &p) override {
      p.print(getData()->accelX, 4); p.print(",");
      p.print(getData()->accelY, 4); p.print(",");
      p.print(getData()->accelZ, 4); p.print(",");
      p.print(getData()->gyrX, 4); p.print(",");
      p.print(getData()->gyrY, 4); p.print(",");
      p.print(getData()->gyrZ, 4); p.print(",");
      p.print(getData()->magX, 4); p.print(",");
      p.print(getData()->magY, 4); p.print(",");
      p.print(getData()->magZ, 4); p.print(",");
      p.print(getData()->temp, 4);
    }
    // clang-format on

  private:
    Adafruit_ICM20948 icm;

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

        setData()->accelX = accel.acceleration.x;
        setData()->accelY = accel.acceleration.y;
        setData()->accelZ = accel.acceleration.z;

        setData()->gyrX = gyr.gyro.x;
        setData()->gyrY = gyr.gyro.y;
        setData()->gyrZ = gyr.gyro.z;

        setData()->magX = mag.magnetic.x;
        setData()->magY = mag.magnetic.y;
        setData()->magZ = mag.magnetic.z;

        setData()->temp = temp.temperature;
    }
};
