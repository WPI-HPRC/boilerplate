#pragma once

#include "../SensorManager/SensorBase.h"
#include "ASM330LHHSensor.h"
#include "Print.h"
#include "boilerplate/Logging/Loggable.h"
#include <Arduino.h>

#define X_FS 16
#define G_FS 4000

struct ASM330Data {
    float accel0;
    float accel1;
    float accel2;

    float gyr0;
    float gyr1;
    float gyr2;
};

class ASM330 : public Sensor<ASM330, ASM330Data> {
public:
  ASM330(SPIClass *dev_spi, uint32_t CS, uint32_t POLLING_RATE)
      : Sensor(POLLING_RATE),
        AccGyr(dev_spi, CS, 2000000),
        CS(CS) {}

  bool init_impl() {
    Serial.print("Initializing ASM330... ");

    pinMode(CS, OUTPUT);
    digitalWrite(CS, LOW);

    bool ret = true;

    if (AccGyr.begin() != 0) {
      Serial.println("FAILED ASM");
      ret = false;
      goto end;
    }

    delay(500);

    if (AccGyr.Set_X_ODR(104.0) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING X ODR");
      ret = false;
      goto end;
    }

    if (AccGyr.Set_G_ODR(104.0) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING G ODR");
      ret = false;
      goto end;
      }

    if (AccGyr.Set_X_FS(X_FS) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING X FS");
      ret = false;
      goto end;
      }
    
    if (AccGyr.Set_G_FS(G_FS) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING G FS");
      ret = false;
      goto end;
      }

    if (AccGyr.Enable_X() != ASM330LHH_OK) {
      Serial.println("ERROR ENABLING X");
      ret = false;
      goto end;
      }

    if (AccGyr.Enable_G() != ASM330LHH_OK) {
      Serial.println("ERROR ENABLING G");
       ret = false;
      goto end;
      }

      delay(500);

    uint8_t status;

    if (AccGyr.Get_X_DRDY_Status(&status) != ASM330LHH_OK) {
      Serial.println("ERROR IN X DRDY STATUS");
      ret = false;
      goto end;
      }
    
   if (AccGyr.Get_G_DRDY_Status(&status) != ASM330LHH_OK) {
      Serial.println("ERROR IN G DRDY STATUS");
      ret = false;
      goto end;
      }
      
    delay(500);

    int32_t st;
    if (AccGyr.Get_X_FS(&st) != ASM330LHH_OK)
    {
      Serial.println("ERROR GETTING X FS");
      ret = false;
      goto end;
      }

    if (st != X_FS)
    {
      Serial.printf("X FS NOT SET PROPERLY. EXPECTED {%d}, GOT {%d}\n", X_FS, st);
      ret = false;
      goto end;
      }

    if (AccGyr.Get_G_FS(&st) != ASM330LHH_OK)
    {
      Serial.println("ERROR GETTING G FS");
      goto end;
    }

    if (st != G_FS)
    {
      Serial.printf("G FS NOT SET PROPERLY. EXPECTED {%d}, GOT {%d}\n", G_FS, st);
      ret = false;
      goto end;
      }

    Serial.println("OK");

    end:
        digitalWrite(CS, HIGH);
        return ret;
  }

  void poll_impl(uint32_t now_ms, ASM330Data &out) {
    // unsigned long now = millis();

    int32_t accel[3] = {0};
    int32_t gyro[3] = {0};

    int status;
    status = AccGyr.Get_X_Axes(accel);
    if(status != ASM330LHHStatusTypeDef::ASM330LHH_OK){
      SerialUSB.println("HELP");
    }
    status = AccGyr.Get_G_Axes(gyro);
    if(status != ASM330LHHStatusTypeDef::ASM330LHH_OK){
      SerialUSB.println("HELP");
    }

    out.accel0 = accel[0] / 1000.f;
    out.accel1 = accel[1] / 1000.f;
    out.accel2 = accel[2] / 1000.f;

    out.gyr0 = gyro[0] / 1000.f;
    out.gyr1 = gyro[1] / 1000.f;
    out.gyr2 = gyro[2] / 1000.f;
  }

private:
  ASM330LHHSensor AccGyr;
  uint32_t CS;
};