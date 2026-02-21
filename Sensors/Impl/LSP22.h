#pragma once

#include "../SensorManager/SensorBase.h"
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>

struct LPS22Data {
  double pressure;
  double temp;
};

#define LPS22_CS_PIN 6

class LPS22 : public Sensor<LPS22, LPS22Data> {
public:
  LPS22() // 50
      : Sensor(50), lps() {}

  bool init_impl() {
    Serial.print("Initializing LPS22... ");

    SPI.begin();
    pinMode(LPS22_CS_PIN, OUTPUT);
    digitalWrite(LPS22_CS_PIN, HIGH);

    if (!lps.begin_SPI(LPS22_CS_PIN, &SPI)) {
      Serial.println("FAILED");
      return false;
    }

    lps.setDataRate(LPS22_RATE_50_HZ);
    //poll_interval_ms_ = 1000 / info_.poll_rate_hz;
    Serial.println("OK");

    return true;
  }

  void poll_impl(uint32_t now_ms, LPS22Data &out) {
    sensors_event_t pressure, temperature;
    if (lps.getEvent(&pressure, &temperature)) {
      out.pressure = pressure.pressure;
      out.temp = temperature.temperature;
    }
  }

private:
  Adafruit_LPS22 lps;
};
