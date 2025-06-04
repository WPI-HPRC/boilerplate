#pragma once

#define kUBLOXGNSSDefaultMaxWait 250

#include "../Sensor/Sensor.h"
#include "Print.h"
#include "boilerplate/Logging/Loggable.h"
#include <SparkFun_u-blox_GNSS_v3.h>

struct MAX10SData {
    int32_t lat = 0;
    int32_t lon = 0;
    float altMSL = 0.0;
    float altEllipsoid = 0.0;
    int32_t velN = 0;
    int32_t velE = 0;
    int32_t velD = 0;
    uint32_t epochTime = 0;
    uint8_t satellites = 0;
    uint8_t gpsLockType = 0;
};

#define MAX10S_LOG_DESC(X)                                                     \
    X(0, "lat", p.print(getData()->lat))                                       \
    X(1, "lon", p.print(getData()->lon))                                       \
    X(2, "altMSL", p.print(getData()->altMSL, 3))                              \
    X(3, "altEll", p.print(getData()->altEllipsoid, 3))                        \
    X(4, "velN", p.print(getData()->velN))                                     \
    X(5, "velE", p.print(getData()->velE))                                     \
    X(6, "velD", p.print(getData()->velD))                                     \
    X(7, "epochTime", p.print(getData()->epochTime))                           \
    X(8, "satellites", p.print(getData()->satellites))                         \
    X(9, "gpsLockType", p.print(getData()->gpsLockType))

class MAX10S : public Sensor, public Loggable {
  public:
    MAX10S()
        : Sensor(sizeof(MAX10SData), 40), Loggable(NUM_FIELDS(MAX10S_LOG_DESC)),
          GPS() {} // This gps initialization defaults to i2c

    const TimedPointer<MAX10SData> getData() const {
        return static_cast<TimedPointer<MAX10SData>>(data);
    }

  private:
    SFE_UBLOX_GNSS GPS;

    MAKE_LOGGABLE(MAX10S_LOG_DESC)

    uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

    TimedPointer<MAX10SData> setData() {
        return static_cast<TimedPointer<MAX10SData>>(data);
    }

    bool init_impl() override {
        if (GPS.begin()) {
            GPS.setNavigationFrequency(25);
            GPS.setAutoPVT(true);
            return true;
        }
        return false;
    }

    void poll() override {
        setData()->gpsLockType = GPS.getFixType();
        setData()->lat = GPS.getLatitude();
        setData()->lon = GPS.getLongitude();
        setData()->altMSL = (float)GPS.getAltitudeMSL() / 1000.0;
        setData()->altEllipsoid = (float)GPS.getAltitude() / 1000.0;
        setData()->velN = GPS.getNedNorthVel();
        setData()->velE = GPS.getNedEastVel();
        setData()->velD = GPS.getNedDownVel();
        setData()->epochTime = GPS.getUnixEpoch();
        setData()->satellites = GPS.getSIV(); // Satellites In View
    }
};
