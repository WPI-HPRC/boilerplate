#pragma once

#include "../SensorManager/SensorBase.h"
#include <LSM6DSO32Sensor.h>
#include <Arduino.h>
#include <Wire.h>

struct LSM6Data {
    float accel0, accel1, accel2, gyr0, gyr1, gyr2;
};

#define LSM6_ODR 208.0f
#define LSM6_X_FS 32
#define LSM6_G_FS 2000

class LSM6 : public Sensor<LSM6, LSM6Data> {
    public:
        LSM6(SPIClass *spi, uint32_t cs) 
        : Sensor(1000.0 / LSM6_ODR), imu(spi, cs), cs(cs)
          {};

        bool begin_impl() {
            Serial.print("Beginning for LSM6... ");

            if (imu.begin() != LSM6DSO32_OK) {
                Serial.println("FAILED");
                return false;
            }

            Serial.println("OK");

            return true;
        }

        bool init_impl() {
            Serial.print("Initializing for LSM6... ");

            if (imu.begin() != LSM6DSO32_OK) {
                Serial.println("FAILED");
                return false;
            }

            imu.Set_G_FS(LSM6_G_FS);
            imu.Set_X_FS(LSM6_X_FS);
            imu.Set_G_ODR(LSM6_ODR);
            imu.Set_X_ODR(LSM6_ODR);
            imu.Enable_G();
            imu.Enable_X();

            Serial.println("OK");

            return true;
        }

        void poll_impl(uint32_t now_ms, LSM6Data &out) {
            int32_t acc[3], gyr[3];
            imu.Get_X_Axes(acc);
            imu.Get_G_Axes(gyr);

            out.accel0 = (float)acc[0] / 1000.0f;
            out.accel1 = (float)acc[1] / 1000.0f;
            out.accel2 = (float)acc[2] / 1000.0f;

            out.gyr0 = (float)gyr[0] / 1000.0f;
            out.gyr1 = (float)gyr[1] / 1000.0f;
            out.gyr2 = (float)gyr[2] / 1000.0f;
        }

    private:
        LSM6DSO32Sensor imu;
        uint32_t cs;
};
