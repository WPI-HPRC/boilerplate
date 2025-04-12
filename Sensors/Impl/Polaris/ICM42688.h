#pragma once

#include "../../Sensor/Sensor.h"
#include "Print.h"
#include <Arduino.h>
#include <ICM42688.h>

struct ICM42688Data {
    double xAcc;
    double yAcc;
    double zAcc;

    double xGyr;
    double yGyr;
    double zGyr;
};

class ICM42688_ : public Sensor {
  public:
    ICM42688_() : Sensor(sizeof(ICM42688Data), 40), icm(Wire, 0x68) {}

    ICM42688Data getData() { return *(ICM42688Data *)data; }

    void debugPrint(Print &p) {
        p.print("xAcc: "); p.print(((ICM42688Data *)data)->xAcc, 4); p.print(", ");
        p.print("yAcc: "); p.print(((ICM42688Data *)data)->yAcc, 4); p.print(", ");
        p.print("zAcc: "); p.print(((ICM42688Data *)data)->zAcc, 4); p.print(", ");

        p.print("xGyr: "); p.print(((ICM42688Data *)data)->xGyr, 4); p.print(", ");
        p.print("yGyr: "); p.print(((ICM42688Data *)data)->yGyr, 4); p.print(", ");
        p.print("zGyr: "); p.print(((ICM42688Data *)data)->zGyr, 4); p.println();
    };
  private:
    ICM42688 icm;

    bool init_impl() override {
      if (icm.begin() < 0) {
        return false;
      }
      if (icm.setAccelFS(ICM42688::gpm16) < 0) {
        return false;
      }
      if (icm.setGyroFS(ICM42688::dps2000) < 0) {
        return false;
      }
    }

    void *poll() override {
      icm.getAGT();

      ((ICM42688Data *)data)->xAcc = icm.accX();
      ((ICM42688Data *)data)->yAcc = icm.accY();
      ((ICM42688Data *)data)->zAcc = icm.accZ();

      ((ICM42688Data *)data)->xGyr = icm.gyrX();
      ((ICM42688Data *)data)->yGyr = icm.gyrY();
      ((ICM42688Data *)data)->zGyr = icm.gyrZ();

      return data;
    }
};
