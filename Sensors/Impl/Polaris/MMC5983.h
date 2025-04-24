#pragma once

#include "../../Sensor/Sensor.h"
#include <Arduino.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

struct MMC5983Data {
  float magX;
  float magY;
  float magZ;
};

class MMC5983 : public Sensor {
  public:
    MMC5983() : Sensor(sizeof(MMC5983Data), 40), mag() {}

    void debugPrint(Print& p) override {
      p.print("magX: "); p.print(((MMC5983Data *)data)->magX, 4); p.print(", ");
      p.print("magY: "); p.print(((MMC5983Data *)data)->magY, 4); p.print(", ");
      p.print("magZ: "); p.print(((MMC5983Data *)data)->magZ, 4); p.println();
    }

    void logCsvHeader(Print& p) override {
      p.print("magX,magY,magZ");
    }

    void logCsvRow(Print& p) override {
      p.print(((MMC5983Data *)data)->magX, 4); p.print(",");
      p.print(((MMC5983Data *)data)->magY, 4); p.print(",");
      p.print(((MMC5983Data *)data)->magZ, 4);
    }

  private:
    SFE_MMC5983MA mag;

    bool init_impl() override {
      if (mag.begin()) {
        mag.softReset();

        return true;
      }
      return false;
    }

    void *poll() override {
      uint32_t rawX = mag.getMeasurementX();
      uint32_t rawY = mag.getMeasurementY();
      uint32_t rawZ = mag.getMeasurementZ();

      double scaledX = (double)rawX - 131072.0;
      scaledX /= 131072.0;
      double scaledY = (double)rawY - 131072.0;
      scaledY /= 131072.0;
      double scaledZ = (double)rawZ - 131072.0;
      scaledZ /= 131072.0;

      ((MMC5983Data *)data)->magX = scaledX * 8.0;
      ((MMC5983Data *)data)->magY = scaledY * 8.0;
      ((MMC5983Data *)data)->magZ = scaledZ * 8.0;

      return data;
    }
};
