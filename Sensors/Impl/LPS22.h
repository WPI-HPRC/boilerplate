#pragma once

#include "../SensorManager/SensorBase.h"
#include <LPS22HBSensor.h>
#include <Arduino.h>
#include <Wire.h>

struct LPS22Data {
  double pressure;
  double temp;
};

#define LPS22_ODR 75.0f

class LPS22 : public Sensor<LPS22, LPS22Data> {
public:
  LPS22(SPIClass* spi, uint32_t cs)
      : Sensor(1000.0 / LPS22_ODR), lps(spi, cs), cs(cs) {}

  bool init_impl() {
    Serial.print("Initializing LPS22... ");

    LPS22HBStatusTypeDef status = lps.begin();
    if (status != LPS22HB_STATUS_OK) {
      Serial.println("FAILED to begin");
      return false;
    }

    status = lps.SetODR(LPS22_ODR);
    if (status != LPS22HB_STATUS_OK) {
      Serial.println("FAILED to set ODR");
      return false;
    }


    status = lps.Enable();
    if (status != LPS22HB_STATUS_OK) {
      Serial.println("FAILED to enable");
      return false;
    }

    float realOdr;
    lps.GetODR(&realOdr);

    pollingPeriodMs_ = 1000 / realOdr;

    Serial.println("OK");

    return true;
  }

  void poll_impl(uint32_t now_ms, LPS22Data &out) {
    float pressure, temperature;
    lps.GetPressure(&pressure);
    lps.GetTemperature(&temperature);

    out.pressure = pressure;
    out.temp = temperature;
  }

private:
   LPS22HBSensor lps;
   uint32_t cs;
};
