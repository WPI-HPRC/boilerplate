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

    ICMData getData() { return *(ICMData *)data; }

    void debugPrint(Print& p) override {
      p.print("accelX: "); p.print(((ICMData *)data)->accelX, 4); p.print(", ");
      p.print("accelY: "); p.print(((ICMData *)data)->accelY, 4); p.print(", ");
      p.print("accelZ: "); p.print(((ICMData *)data)->accelZ, 4); p.print(", ");
      p.print("gyrX: "); p.print(((ICMData *)data)->gyrX, 4); p.print(", ");
      p.print("gyrY: "); p.print(((ICMData *)data)->gyrY, 4); p.print(", ");
      p.print("gyrZ: "); p.print(((ICMData *)data)->gyrZ, 4); p.print(", ");
      p.print("magX: "); p.print(((ICMData *)data)->magX, 4); p.print(", ");
      p.print("magY: "); p.print(((ICMData *)data)->magY, 4); p.print(", ");
      p.print("magZ: "); p.print(((ICMData *)data)->magZ, 4); p.print(", ");
      p.print("temp: "); p.print(((ICMData *)data)->temp, 4); p.println();
    }

    void logCsvHeader(Print& p) override {
      p.print("ICMaccelX,ICMaccelY,ICMaccelZ,ICMgyrX,ICMgyrY,ICMgyrZ,magX,magY,magZ,ICMtemp");
    }

    void logCsvRow(Print &p) override {
      p.print(((ICMData *)data)->accelX, 4); p.print(",");
      p.print(((ICMData *)data)->accelY, 4); p.print(",");
      p.print(((ICMData *)data)->accelZ, 4); p.print(",");
      p.print(((ICMData *)data)->gyrX, 4); p.print(",");
      p.print(((ICMData *)data)->gyrY, 4); p.print(",");
      p.print(((ICMData *)data)->gyrZ, 4); p.print(",");
      p.print(((ICMData *)data)->magX, 4); p.print(",");
      p.print(((ICMData *)data)->magY, 4); p.print(",");
      p.print(((ICMData *)data)->magZ, 4); p.print(",");
      p.print(((ICMData *)data)->temp, 4);
    }

  private:
    Adafruit_ICM20948 icm;

    bool init_impl() override {
        if (!icm.begin_I2C(0x68)) {
            icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
            icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
            uint16_t accelRateDiv = 1125 / ODR - 1; // Per datasheet: ODR = 1125 / (1 + div)
            uint8_t gyrRateDiv = 1100 / ODR - 1; // Per datasheet: ODR = 1100 / (1 + div)
            icm.setAccelRateDivisor(accelRateDiv);
            icm.setGyroRateDivisor(gyrRateDiv);
            return false;
        }
        return true;
    }

    void *poll() override {
        sensors_event_t accel, gyr, mag, temp;

        icm.getEvent(&accel, &gyr, &temp, &mag);

        ((ICMData *)data)->accelX = accel.acceleration.x;
        ((ICMData *)data)->accelY = accel.acceleration.y;
        ((ICMData *)data)->accelZ = accel.acceleration.z;

        ((ICMData *)data)->gyrX = gyr.gyro.x;
        ((ICMData *)data)->gyrY = gyr.gyro.y;
        ((ICMData *)data)->gyrZ = gyr.gyro.z;

        ((ICMData *)data)->magX = mag.magnetic.x;
        ((ICMData *)data)->magY = mag.magnetic.y;
        ((ICMData *)data)->magZ = mag.magnetic.z;

        ((ICMData *)data)->temp = temp.temperature;

        return data;
    }
};
