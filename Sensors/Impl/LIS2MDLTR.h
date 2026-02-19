#pragma once

#include "../SensorManager/SensorBase.h"
#include "Wire.h"
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <cstdint>

struct LISM2Data {
  float max0, mag1, mag2;
};

#define LISM2_POLLING_RATE 100 // not sure on this, should be checked

class LISM2 : public Sensor<LISM2, LISM2Data> {
public:
  LISM2() : Sensor(LISM2_POLLING_RATE), Adafruit_LIS2MDL(1234) {}

  bool init_impl() {
    Serial.print("Initialising LISM2... ");

    // docs have some pin definitions, might have to do same?
    
    if(!lis2mdl.begin()) {
      Serial.println("FAILED");
      return false;
    }

    lis2mdl.enableAutoRange(true);
    // not sure that  this rly means but we can do this

    // lis2mdl.printSensorDetails();
    // idk this seems like it might help debugging
  }

  void poll_impl(uint32_t now_ms, LISM2Data &out) {
    sensors_event_t event;
    lis2mdl.getEvent(&event);

    out.max0 = (float)event.magnetic.x;
    out.mag1 = (float)event.magnetic.y;
    out.mag2 = (float)event.magnetic.z;
    // these are in uT ?

    // can also do
    // lis2mdl.raw.x
    
  }

private:
  Adafruit_LIS2MDL lis2mdl;
}
