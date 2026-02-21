#pragma once

#include "../Sensor/Sensor.h"
#include "ASM330LHHSensor.h"
#include "Print.h"
#include "boilerplate/Logging/Loggable.h"
#include <Arduino.h>

#define X_FS 16
#define G_FS 4000

struct ASM330Data {
    float accelX;
    float accelY;
    float accelZ;

    float gyrX;
    float gyrY;
    float gyrZ;
};

#define ASM330_LOG_DESC(X)                                                     \
    X(0, "ASMaccelX", p.print(getData()->accelX, 3))                           \
    X(1, "ASMaccelY", p.print(getData()->accelY, 3))                           \
    X(2, "ASMaccelZ", p.print(getData()->accelZ, 3))                           \
    X(3, "ASMgyrX", p.print(getData()->gyrX, 3))                               \
    X(4, "ASMgyrY", p.print(getData()->gyrY, 3))                               \
    X(5, "ASMgyrZ", p.print(getData()->gyrZ, 3))

class ASM330 : public Sensor, public Loggable {
public:
  ASM330(SPIClass *dev_spi, uint32_t CS, uint32_t POLLING_RATE, bool inPayload = false)
      : Sensor(sizeof(ASM330Data), 0), Loggable(NUM_FIELDS(ASM330_LOG_DESC)),
        AccGyr(dev_spi, CS), inPayload(inPayload),
        CS(CS) {}

        const TimedPointer<ASM330Data> getData() const {
          return static_cast<TimedPointer<ASM330Data>>(data);
        }
        TimedPointer<ASM330Data> setData() {
          return static_cast<TimedPointer<ASM330Data>>(data);
        }
  
  bool inPayload;

  uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

  bool init_impl() {
    Serial.print("Initializing ASM330... ");

    pinMode(CS, OUTPUT);
    digitalWrite(CS, LOW);

    if (AccGyr.begin() != 0) {
      Serial.println("FAILED ASM");
      return false;
    }

    if (AccGyr.Set_X_ODR(104.0) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING X ODR");
      return false;
    }

    if (AccGyr.Set_G_ODR(104.0) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING G ODR");
      return false;
    }

    if (AccGyr.Set_X_FS(X_FS) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING X FS");
      return false;
    }
    
    if (AccGyr.Set_G_FS(G_FS) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING G FS");
      return false;
    }

    if (AccGyr.Enable_X() != ASM330LHH_OK) {
      Serial.println("ERROR ENABLING X");
      return false;
    }

    if (AccGyr.Enable_G() != ASM330LHH_OK) {
      Serial.println("ERROR ENABLING G");
      return false;
    }

    uint8_t status;

    if (AccGyr.Get_X_DRDY_Status(&status) != ASM330LHH_OK) {
      Serial.println("ERROR IN X DRDY STATUS");
      return false;
    }

    if (status != 0) {
      Serial.println("X NOT DATA READY");
      return false;
    }
    
   if (AccGyr.Get_G_DRDY_Status(&status) != ASM330LHH_OK) {
      Serial.println("ERROR IN G DRDY STATUS");
      return false;
    }

    if (status != 0) {
      Serial.println("G NOT DATA READY");
      return false;
    }

    int32_t st;
    if (AccGyr.Get_X_FS(&st) != ASM330LHH_OK)
    {
      Serial.println("ERROR GETTING X FS");
      return false;
    }

    if (st != X_FS)
    {
      Serial.printf("X FS NOT SET PROPERLY. EXPECTED {%d}, GOT {%d}", X_FS, st);
    }

    if (AccGyr.Get_X_FS(&st) != ASM330LHH_OK)
    {
      Serial.println("ERROR GETTING G FS");
      return false;
    }

    if (st != G_FS)
    {
      Serial.printf("G FS NOT SET PROPERLY. EXPECTED {%d}, GOT {%d}", G_FS, st);
    }

    Serial.println("OK");
    return true;
  }

  void poll() override {
        static int32_t accelerometer[3] = {};
        static int32_t gyroscope[3] = {};
        AccGyr.Get_X_Axes(accelerometer);
        AccGyr.Get_G_Axes(gyroscope);

        // X and Y axes rotated to match ICM orientation
        float aX = -(float)accelerometer[1] / 1000.0;
        float aY = (float)accelerometer[0] / 1000.0;
        float aZ = (float)accelerometer[2] / 1000.0;

        // X and Y axes rotated to match ICM orientation
        float gX = -(float)gyroscope[1] / 1000.0;
        float gY = (float)gyroscope[0] / 1000.0;
        float gZ = (float)gyroscope[2] / 1000.0;

        if (inPayload) {
            // Axes rotated to match rocket (z up)
            setData()->accelX = aX;
            setData()->accelY = aZ;
            setData()->accelZ = -aY;

            setData()->gyrX = gX;
            setData()->gyrY = gZ;
            setData()->gyrZ = -gY;
        } else {
            setData()->accelX = aX;
            setData()->accelY = aY;
            setData()->accelZ = aZ;

            setData()->gyrX = gX;
            setData()->gyrY = gY;
            setData()->gyrZ = gZ;
        }

    // MAKE_LOGGABLE(ASM330_LOG_DESC);
  }

private:
  MAKE_LOGGABLE(ASM330_LOG_DESC)
  ASM330LHHSensor AccGyr;
  uint32_t CS;
};
