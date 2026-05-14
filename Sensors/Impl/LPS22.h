#pragma once

#include "../SensorManager/SensorBase.h"
#include <LPS22HBSensor.h>
#include <Arduino.h>
#include <Wire.h>

struct LPS22Data {
  float pressure;
  float temp;
};

#define LPS22_ODR 75.0f
// #define LPS22_ODR 50.0f

class LPS22 : public Sensor<LPS22, LPS22Data> {
public:
  LPS22(SPIClass* spi, uint32_t cs)
      // : Sensor(digitalPinToPinName(LPS_INT)), lps(spi, cs), cs(cs) {}
      : Sensor(1000.0f / LPS22_ODR), lps(spi, cs), cs(cs) {}

  bool begin_impl() {
    Log.infoln("Beginning LPS22");

    if (lps.begin() != LPS22HB_STATUS_OK) {
      return false;
    }
    return true;
  }

  bool init_impl() {
    Log.infoln("Initializing LPS22");

    // lps.Disable();
    // lps.WriteReg(0x12, 0x00);

    // uint8_t status_reg;
    // lps.ReadReg(0x27, &status_reg);
    // while ((status_reg & 0b11) != 0) {
    //   Log.infoln("\tTrying to clear status register");
    //   float val;
    //   lps.GetPressure(&val);
    //   lps.GetTemperature(&val);
    //   lps.ReadReg(0x27, &status_reg);
    // }

    LPS22HBStatusTypeDef status = lps.SetODR(LPS22_ODR);
    Log.traceln("\tSetting ODR");
    if (status != LPS22HB_STATUS_OK) {
      return false;
    }

    // Log.traceln("\tEnabling interrupt");
    // status = lps.WriteReg(0x12, 0x04);
    // if (status != LPS22HB_STATUS_OK) {
    //   return false;
    // }

    Log.traceln("\tSetting enable");
    status = lps.Enable();
    if (status != LPS22HB_STATUS_OK) {
      return false;
    }

    // float realOdr;
    // lps.GetODR(&realOdr);

    // pollingPeriodMs_ = 1000 / realOdr;

    return true;
  }

  bool poll_impl(uint32_t now_ms, LPS22Data &out) {
    float pressure = 0.0f, temperature = 0.0f;
    int status;
    status = lps.GetPressure(&pressure);
    if (status != LPS22HB_STATUS_OK) {
      Log.errorln("Failed to get pressure");
    }
    status = lps.GetTemperature(&temperature);
    if (status != LPS22HB_STATUS_OK) {
      Log.errorln("Failed to get temperature");
    }

    out.pressure = pressure;
    out.temp = temperature;

    return true;
  }

private:
   LPS22HBSensor lps;
   uint32_t cs;
};
