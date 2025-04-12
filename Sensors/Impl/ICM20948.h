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

class ICM20948 : public Sensor {
  public:
    ICM20948() : Sensor(sizeof(ICMData), 40), icm() {}

    ICMData getData() { return *(ICMData *)data; }

    void debugPrint(Print& p) {
      p.print("accelX: "); p.print(((ICMData *)data)->accelX); p.print(", ");
      p.print("accelY: "); p.print(((ICMData *)data)->accelY); p.print(", ");
      p.print("accelZ: "); p.print(((ICMData *)data)->accelZ); p.print(", ");
      p.print("gyrX: "); p.print(((ICMData *)data)->gyrX); p.print(", ");
      p.print("gyrY: "); p.print(((ICMData *)data)->gyrY); p.print(", ");
      p.print("gyrZ: "); p.print(((ICMData *)data)->gyrZ); p.print(", ");
      p.print("magX: "); p.print(((ICMData *)data)->magX); p.print(", ");
      p.print("magY: "); p.print(((ICMData *)data)->magY); p.print(", ");
      p.print("magZ: "); p.print(((ICMData *)data)->magZ); p.print(", ");
      p.print("temp: "); p.print(((ICMData *)data)->temp); p.println();
    }

  private:
    Adafruit_ICM20948 icm;

    bool init_impl() override {
        if (!icm.begin_I2C()) {
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
