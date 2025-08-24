#pragma once

#include "../../Sensor/Sensor.h"
#include "../../boilerplate/Logging/Loggable.h"
#include "boilerplate/StateEstimator/kfConsts.h"
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

#define ICM_LOG_DESC(X)                                                            \
    X(0, "ICMaccelX", p.print(getData()->accelX, 4))                           \
    X(1, "ICMaccelY", p.print(getData()->accelY, 4))                           \
    X(2, "ICMaccelZ", p.print(getData()->accelZ, 4))                           \
    X(3, "ICMgyrX", p.print(getData()->gyrX, 4))                               \
    X(4, "ICMgyrY", p.print(getData()->gyrY, 4))                               \
    X(5, "ICMgyrZ", p.print(getData()->gyrZ, 4))                               


#define ODR 40

class ICM42688_ : public Sensor, public Loggable {
  public:
    ICM42688_() : Sensor(sizeof(ICM42688Data), ODR), icm(Wire, 0x68),
     Loggable(NUM_FIELDS(ICM_LOG_DESC)) {}

    const TimedPointer<ICM42688Data> getData() const {
        return static_cast<TimedPointer<ICM42688Data>>(data);
    }

  private:
    ICM42688 icm;

    MAKE_LOGGABLE(ICM_LOG_DESC)

    TimedPointer<ICM42688Data> setData() {
        return static_cast<TimedPointer<ICM42688Data>>(data);
    }

    uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

    bool init_impl() override {
        return icm.begin() > 0 && icm.setAccelFS(ICM42688::gpm16) > 0 &&
               icm.setGyroFS(ICM42688::dps2000) > 0;
    }

    void poll() override {
        icm.getAGT();

        setData()->accelX = icm.accX() / g; // Convert to gs from m/s^2
        setData()->accelY = icm.accY() / g;
        setData()->accelZ = icm.accZ() / g;

        setData()->gyrX = icm.gyrX(); // Already in dps
        setData()->gyrY = icm.gyrY();
        setData()->gyrZ = icm.gyrZ();
    }
};
