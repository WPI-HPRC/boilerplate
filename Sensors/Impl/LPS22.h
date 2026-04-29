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

  bool begin_impl() {
    Log.infoln("Beginning LPS22");

    if (lps.begin() != LPS22HB_STATUS_OK) {
      return false;
    }
    return true;
  }

  bool init_impl() {
    Log.infoln("Initializing LPS22");

    LPS22HBStatusTypeDef status = lps.SetODR(LPS22_ODR);
    Log.traceln("\tSetting ODR");
    if (status != LPS22HB_STATUS_OK) {
      return false;
    }


    status = lps.Enable();
    Log.traceln("\tSetting enable");
    if (status != LPS22HB_STATUS_OK) {
      return false;
    }

    float realOdr;
    lps.GetODR(&realOdr);

    pollingPeriodMs_ = 1000 / realOdr;

    return true;
  }

  bool poll_impl(uint32_t now_ms, LPS22Data &out) {
    float pressure, temperature;
    lps.GetPressure(&pressure);
    lps.GetTemperature(&temperature);

    out.pressure = pressure;
    out.temp = temperature;

    return true;
  }

private:
   LPS22HBSensor lps;
   uint32_t cs;
};
