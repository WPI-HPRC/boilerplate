#pragma once

#include "../SensorManager/SensorBase.h"
#include "../Impl/LPS22.h"
#include "boilerplate/Utilities/CSVParser.h"
#include <Arduino.h>

class MockLPS22 : public Sensor<MockLPS22, LPS22Data> {
public:
  MockLPS22(Stream *dataFile, float rate)
      : Sensor(1000.0 / rate), dataFile(dataFile) {}

  bool begin_impl() {
    Serial.println("Beginning LPS22");
    return true;
  }

  bool init_impl() {
    Serial.println("Initializing LPS22");
    return skipCSVRow(dataFile);
  }

  bool poll_impl(uint32_t now_ms, LPS22Data &out) {
    float vals[2];
    loadCSVRow(dataFile, 2, vals);

    out.pressure = vals[0];
    out.temp = vals[1];

    return true;
  }

private:
  Stream *dataFile;
};
