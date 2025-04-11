#pragma once

#include "../Sensor/Sensor.h"
#include "SdFat.h"
#include <SparkFun_u-blox_GNSS_v3.h>

struct MAX10SData {
     float lat = 0.0;
     float lon = 0.0;
     float altMSL = 0.0;
     float altAGL = 0.0;
     int32_t velN = 0;
     int32_t velE = 0;
     int32_t velD = 0;
     uint32_t epochTime = 0;
     uint8_t satellites = 0;
     uint8_t gpsLockType = 0;
};

class MAX10S : public Sensor {
     public:
          MAX10S() : Sensor(sizeof(MAX10SData), 25), GPS() {} // This gps initialization defaults to i2c
          // the 40 is arbitrary

          void debugPrint(Print& p) {
               p.print("lat: "); p.print(((MAX10SData *)data)->lat); p.print(", ");
               p.print("lon: "); p.print(((MAX10SData *)data)->lon); p.print(", ");
               p.print("altMSL: "); p.print(((MAX10SData *)data)->altMSL); p.print(", ");
               p.print("altAGL: "); p.print(((MAX10SData *)data)->altAGL); p.print(", ");
               p.print("velN: "); p.print(((MAX10SData *)data)->velN); p.print(", ");
               p.print("velE: "); p.print(((MAX10SData *)data)->velE); p.print(", ");
               p.print("velD: "); p.print(((MAX10SData *)data)->velD); p.print(", ");
               p.print("epochTime: "); p.print(((MAX10SData *)data)->epochTime); p.print(", ");
               p.print("satellites: "); p.print(((MAX10SData *)data)->satellites); p.print(", ");
               p.print("gpsLockType: "); p.print(((MAX10SData *)data)->gpsLockType); p.println();
          }

     private:
          SFE_UBLOX_GNSS GPS;
          bool init_impl () override {
               if (GPS.begin()) {
                    // GPS.setI2CpollingWait(40);
                    GPS.setNavigationFrequency(40);
                    GPS.setAutoPVT(true);
                    return true;
               }
               return false;
          }

          void* poll() override {
               ((MAX10SData*)data)->gpsLockType = GPS.getFixType();
               ((MAX10SData*)data)->lat = GPS.getLatitude();
               ((MAX10SData*)data)->lon = GPS.getLongitude();
               ((MAX10SData*)data)->altMSL = GPS.getAltitudeMSL();
               ((MAX10SData*)data)->altAGL = GPS.getAltitude(); // not actually AGL, returns ellipsoid height (source ChatGPT)
               ((MAX10SData*)data)->velN = GPS.getNedNorthVel();
               ((MAX10SData*)data)->velE = GPS.getNedEastVel();
               ((MAX10SData*)data)->velD = GPS.getNedDownVel();
               ((MAX10SData*)data)->epochTime = GPS.getUnixEpoch();
               ((MAX10SData*)data)->satellites = GPS.getSIV(); // Satellites In View

               return data;
          }
};
