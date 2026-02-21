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

class ASM330 : public Sensor<ASM330, ASM330Data> {
public:
  ASM330(SPIClass *dev_spi, uint32_t CS, uint32_t POLLING_RATE)
      : Sensor(POLLING_RATE),
        AccGyr(dev_spi, CS),
        CS(CS) {}

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

    MAKE_LOGGABLE(ASM330_LOG_DESC)

private:
  ASM330LHHSensor AccGyr;
  uint32_t CS;
};
