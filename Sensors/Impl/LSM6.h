#pragma once

#include ".../SensorManager/SensorBase.h"
#include <LSM6.h>
#include <Arduino.h>
#include <Wire.h>

struct LSM6Data {
    float accel0, accel1, accel2, gyr0, gyr1, gyr2;
};

class LSM6 : public Sensor<LSM6, LSM6Data> {
    public:
        LSM6() 
        : Sensor(5) //made up number 
          {};

        bool init_impl() {
            Serial.print("Initializing for LSM6");

            if (!imu.init()) {
                Serial.println("FAILED");
                return false;
            }
            Serial.println("OK");
            return true;
        }

        void poll_impl(uint32_t now_ms, LSM6Data &out) {
            imu.read();

            out.accel0 = (float)imu.a.x;
            out.accel1 = (float)imu.a.y;
            out.accel2 = (float)imu.a.z;

            out.gyr0 = (float)imu.g.x;
            out.gyr1 = (float)imu.g.y;
            out.gyr2 = (float)imu.g.z;
        }

    private:
        LSM6 imu;
}