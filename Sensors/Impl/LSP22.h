#pragma once

#include "../SensorManager/SensorBase.h"
#include <LPS22HBSensor.h>
#include <Arduino.h>
#include <SPI.h>

struct LPS22Data {
  double pressure;
  double temp;
};

class LPS22 : public Sensor<LPS22, LPS22Data> {
public:
  LPS22(SPIClass *dev_spi, uint32_t CS, uint32_t POLLING_RATE): 
  dev_spi(dev_spi), CS(CS), polling_rate(POLLING_RATE), Sensor(POLLING_RATE), lps(dev_spi, CS, 2000000) {}

  bool init_impl() {
    Serial.print("Initializing LPS22... ");
    delay(100);

    pinMode(CS, OUTPUT);
    digitalWrite(CS, LOW);

    LPS22HBStatusTypeDef status = lps.begin();
    if (status != LPS22HB_STATUS_OK) {
      Serial.printf("FAILED TO BEGIN. STATUS = %02X\n", status);
      return false;
    }

    uint8_t id = 0;
    if (lps.ReadID(&id) != 0) {
      Serial.println("FAILED TO READ ID");
      // return false;
    }

    Serial.printf("LPS ID: %02X\n", id);

    status = lps.Reset();
    if (status != LPS22HB_STATUS_OK) {
      Serial.printf("FAILED TO RESET. STATUS = %02X\n", status);
      return false;
    }


    delay(1000);

    status = lps.Enable();
    if (status != LPS22HB_STATUS_OK) {
      Serial.printf("FAILED TO ENABLE. STATUS = %02X\n", status);
      return false;
    }

    Serial.println("OK");
    return true;
  }

  void poll_impl(uint32_t now_ms, LPS22Data &out) {
    
    float pressure;
    float temperature;
    LPS22HBStatusTypeDef status = lps.GetPressure(&pressure); 
    
    if (status != LPS22HB_STATUS_OK)
    {
      Serial.printf("ERROR GETTING LPS PRESSURE: %d\n", status);
    }

    status = lps.GetTemperature(&temperature);
    if (status != LPS22HB_STATUS_OK)
    {
      Serial.printf("ERROR GETTING LPS TEMPERATURE: %d\n", status);
    }

    out.pressure = pressure;
    out.temp = temperature;
  }

private:
  LPS22HBSensor lps;
  SPIClass *dev_spi;
  uint32_t CS;
  uint32_t polling_rate;
};
