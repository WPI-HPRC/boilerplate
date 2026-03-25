#pragma once

#include "../SensorManager/SensorBase.h"
#include "ASM330LHHSensor.h"
#include <Arduino.h>
#include <Wire.h>

#define ASM330_ODR 104.0f
#define ASM330_X_FS 16
#define ASM330_G_FS 4000

struct ASM330Data {
  float accel0, accel1, accel2, gyr0, gyr1, gyr2;
};

class ASM330 : public Sensor<ASM330, ASM330Data> {
public:
  ASM330(SPIClass *dev_spi, uint32_t cs)
      : Sensor(1000.0 / ASM330_ODR),
        AccGyr(dev_spi, cs),
        cs(cs) {}

  bool init_impl() {
    Serial.print("Initializing ASM330... ");

    if (AccGyr.begin() != 0) {
      Serial.println("FAILED ASM");
      return false;
    }

    if (AccGyr.Set_G_FS(ASM330_G_FS) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING G FS");
      return false;
    }

    if (AccGyr.Set_G_ODR(ASM330_ODR) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING G ODR");
      return false;
    }

    if (AccGyr.Set_X_FS(ASM330_X_FS) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING X FS");
      return false;
    }

    if (AccGyr.Set_X_ODR(ASM330_ODR) != ASM330LHH_OK) {
      Serial.println("ERROR SETTING X ODR");
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

    if (status != 1) {
      Serial.printf("X NOT DATA READY, status %d\n", status);
      return false;
    }
    
   if (AccGyr.Get_G_DRDY_Status(&status) != ASM330LHH_OK) {
      Serial.println("ERROR IN G DRDY STATUS");
      return false;
    }

    if (status != 1) {
      Serial.println("G NOT DATA READY");
      return false;
    }

    int32_t st;
    if (AccGyr.Get_X_FS(&st) != ASM330LHH_OK)
    {
      Serial.println("ERROR GETTING X FS");
      return false;
    }

    if (st != ASM330_X_FS)
    {
      Serial.printf("X FS NOT SET PROPERLY. EXPECTED {%d}, GOT {%d}\n", ASM330_X_FS, st);
    }

    if (AccGyr.Get_G_FS(&st) != ASM330LHH_OK)
    {
      Serial.println("ERROR GETTING G FS");
      return false;
    }

    if (st != ASM330_G_FS)
    {
      Serial.printf("G FS NOT SET PROPERLY. EXPECTED {%d}, GOT {%d}\n", ASM330_G_FS, st);
    }

    Serial.println("OK");
    return true;
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

    out.accel0 = (float)accel[0];
    out.accel1 = (float)accel[1]; 
    out.accel2 = (float)accel[2];    
    out.gyr0 = (float)gyro[0];
    out.gyr1 = (float)gyro[1];
    out.gyr2 = (float)gyro[2]; 
  }

private:
  ASM330LHHSensor AccGyr;
  uint32_t cs;
};
