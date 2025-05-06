#pragma once

#include "../../Sensor/Sensor.h"
#include "Print.h"
#include <Arduino.h>
#include <ICM42688.h>

struct ICM42688Data {
    double accelX;
    double accelY;
    double accelZ;

    double gyrX;
    double gyrY;
    double gyrZ;
};

class ICM42688_ : public Sensor {
  public:
    ICM42688_() : Sensor(sizeof(ICM42688Data), 40), icm(Wire, 0x68) {}

    ICM42688Data getData() { return *(ICM42688Data *)data; }

    void debugPrint(Print &p) override {
        p.print("accelX: "); p.print(((ICM42688Data *)data)->accelX, 4); p.print(", ");
        p.print("accelY: "); p.print(((ICM42688Data *)data)->accelY, 4); p.print(", ");
        p.print("accelZ: "); p.print(((ICM42688Data *)data)->accelZ, 4); p.print(", ");

        p.print("gyrX: "); p.print(((ICM42688Data *)data)->gyrX, 4); p.print(", ");
        p.print("gyrY: "); p.print(((ICM42688Data *)data)->gyrY, 4); p.print(", ");
        p.print("gyrZ: "); p.print(((ICM42688Data *)data)->gyrZ, 4); p.println();
    }

    void logCsvHeader(Print& p) override {
      p.print("accelX,accelY,accelZ,gyrX,gyrY,gyrZ");
    }

    void logCsvRow(Print &p) override {
        p.print(((ICM42688Data *)data)->accelX, 4); p.print(",");
        p.print(((ICM42688Data *)data)->accelY, 4); p.print(",");
        p.print(((ICM42688Data *)data)->accelZ, 4); p.print(",");

        p.print(((ICM42688Data *)data)->gyrX, 4); p.print(",");
        p.print(((ICM42688Data *)data)->gyrY, 4); p.print(",");
        p.print(((ICM42688Data *)data)->gyrZ, 4);
    }

  private:
    ICM42688 icm;

    bool init_impl() override {
        return icm.begin() > 0 && icm.setAccelFS(ICM42688::gpm16) > 0 &&
               icm.setGyroFS(ICM42688::dps2000) > 0;
    }

    void poll() override {
        icm.getAGT();

        ((ICM42688Data *)data)->accelX = icm.accX();
        ((ICM42688Data *)data)->accelY = icm.accY();
        ((ICM42688Data *)data)->accelZ = icm.accZ();

        ((ICM42688Data *)data)->gyrX = icm.gyrX();
        ((ICM42688Data *)data)->gyrY = icm.gyrY();
        ((ICM42688Data *)data)->gyrZ = icm.gyrZ();
    }
};
