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

    void debugPrint() {
      Serial.print("accelX"); Serial.print(((ICMData *)data)->accelX); Serial.print(", ");
      Serial.print("accelY"); Serial.print(((ICMData *)data)->accelY); Serial.print(", ");
      Serial.print("accelZ"); Serial.print(((ICMData *)data)->accelZ); Serial.print(", ");
      Serial.print("gyrX"); Serial.print(((ICMData *)data)->gyrX); Serial.print(", ");
      Serial.print("gyrY"); Serial.print(((ICMData *)data)->gyrY); Serial.print(", ");
      Serial.print("gyrZ"); Serial.print(((ICMData *)data)->gyrZ); Serial.print(", ");
      Serial.print("magX"); Serial.print(((ICMData *)data)->magX); Serial.print(", ");
      Serial.print("magY"); Serial.print(((ICMData *)data)->magY); Serial.print(", ");
      Serial.print("magZ"); Serial.print(((ICMData *)data)->magZ); Serial.print(", ");
      Serial.print("temp"); Serial.print(((ICMData *)data)->temp); Serial.println();
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
