#pragma once

#include "../Sensor/Sensor.h"
#include "Print.h"
#include <SparkFun_u-blox_GNSS_v3.h>

struct MAX10SData {
    float lat = 0.0;
    float lon = 0.0;
    float altMSL = 0.0;
    float altEllipsoid = 0.0;
    int32_t velN = 0;
    int32_t velE = 0;
    int32_t velD = 0;
    uint32_t epochTime = 0;
    uint8_t satellites = 0;
    uint8_t gpsLockType = 0;
};

class MAX10S : public Sensor {
  public:
    MAX10S()
        : Sensor(sizeof(MAX10SData), 25), GPS() {
    } // This gps initialization defaults to i2c

    const TimedPointer<MAX10SData> getData() const {
        return static_cast<TimedPointer<MAX10SData>>(data);
    }

    // clang-format off
    void debugPrint(Print& p) override {
       p.print("lat: "); p.print(getData()->lat); p.print(", ");
       p.print("lon: "); p.print(getData()->lon); p.print(", ");
       p.print("altMSL: "); p.print(getData()->altMSL); p.print(", ");
       p.print("altEll: "); p.print(getData()->altEllipsoid); p.print(", ");
       p.print("velN: "); p.print(getData()->velN); p.print(", ");
       p.print("velE: "); p.print(getData()->velE); p.print(", ");
       p.print("velD: "); p.print(getData()->velD); p.print(", ");
       p.print("epochTime: "); p.print(getData()->epochTime); p.print(", ");
       p.print("satellites: "); p.print(getData()->satellites); p.print(", ");
       p.print("gpsLockType: "); p.print(getData()->gpsLockType); p.println();
    }

    void logCsvHeader(Print& p) override {
       p.print("lat,lon,altMSL,altEll,velN,velE,velD,epochTime,satellites,gpsLockType");
    }

    void logCsvRow(Print& p, uint32_t lastLoggedAt = 0) override {
       IF_NEW(p.print(getData()->lat)); p.print(",");
       IF_NEW(p.print(getData()->lon)); p.print(",");
       IF_NEW(p.print(getData()->altMSL)); p.print(",");
       IF_NEW(p.print(getData()->altEllipsoid)); p.print(",");
       IF_NEW(p.print(getData()->velN)); p.print(",");
       IF_NEW(p.print(getData()->velE)); p.print(",");
       IF_NEW(p.print(getData()->velD)); p.print(",");
       IF_NEW(p.print(getData()->epochTime)); p.print(",");
       IF_NEW(p.print(getData()->satellites)); p.print(",");
       IF_NEW(p.print(getData()->gpsLockType));
    }
    // clang-format on

  private:
    SFE_UBLOX_GNSS GPS;

    TimedPointer<MAX10SData> setData() {
        return static_cast<TimedPointer<MAX10SData>>(data);
    }

    bool init_impl() override {
        if (GPS.begin()) {
            // GPS.setI2CpollingWait(40);
            GPS.setNavigationFrequency(40);
            GPS.setAutoPVT(true);
            return true;
        }
        return false;
    }

    void poll() override {
        setData()->gpsLockType = GPS.getFixType();
        setData()->lat = GPS.getLatitude();
        setData()->lon = GPS.getLongitude();
        setData()->altMSL = GPS.getAltitudeMSL();
        setData()->altEllipsoid = GPS.getAltitude();
        setData()->velN = GPS.getNedNorthVel();
        setData()->velE = GPS.getNedEastVel();
        setData()->velD = GPS.getNedDownVel();
        setData()->epochTime = GPS.getUnixEpoch();
        setData()->satellites = GPS.getSIV(); // Satellites In View
    }
};
