#pragma once

#include "../../Sensor/Sensor.h"
#include "../../boilerplate/Logging/Loggable.h"
#include "Print.h"
#include <Arduino.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

// TODO: Was this calibrated?

// Please note: to properly correct and calibrate the X, Y and Z channels, you need to determine true
// offsets (zero points) and scale factors (gains) for all three channels. Futher details can be found at:
// https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/

struct MMC5983Data {
  float magX;
  float magY;
  float magZ;
};

#define MMC_LOG_DESC(X)                                                            \
    X(0, "MMCmagX", p.print(getData()->magX, 4))                               \
    X(1, "MMCmagY", p.print(getData()->magY, 4))                               \
    X(2, "MMCmagZ", p.print(getData()->magZ, 4))

#define ODR 40

class MMC5983 : public Sensor, public Loggable {
  public:
    MMC5983() : Sensor(sizeof(MMC5983Data), ODR), mag(), Loggable(NUM_FIELDS(MMC_LOG_DESC)) {}

    const TimedPointer<MMC5983Data> getData() const {
        return static_cast<TimedPointer<MMC5983Data>>(data);
    }

  private:
    SFE_MMC5983MA mag;

    MAKE_LOGGABLE(MMC_LOG_DESC)

    TimedPointer<MMC5983Data> setData() {
        return static_cast<TimedPointer<MMC5983Data>>(data);
    }

    uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

    bool init_impl() override {
      if (mag.begin()) {
        mag.softReset();

        return true;
      }
      return false;
    }

    void poll() override {
      uint32_t rawX = mag.getMeasurementX();
      uint32_t rawY = mag.getMeasurementY();
      uint32_t rawZ = mag.getMeasurementZ();

      // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17 (131072).
      // Here we scale each field to +/- 1.0 to make it easier to convert to Gauss.
      double scaledX = (double)rawX - 131072.0;
      scaledX /= 131072.0;
      double scaledY = (double)rawY - 131072.0;
      scaledY /= 131072.0;
      double scaledZ = (double)rawZ - 131072.0;
      scaledZ /= 131072.0;

      // The magnetometer full scale is +/- 8 Gauss
      // Multiply the scaled values by 8 to convert to Gauss
      setData()->magX = scaledX * 8.0;
      setData()->magY = scaledY * 8.0;
      setData()->magZ = scaledZ * 8.0;
    }
};
