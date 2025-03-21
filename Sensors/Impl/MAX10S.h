#pragma once

#include "../Sensor/Sensor.h"
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
     bool gpsLock = false;
};

class MAX10S : public Sensor {
     public:
          MAX10S() : Sensor(sizeof(MAX10SData), 40), GPS() {} // This gps initialization defaults to i2c
          // the 40 is arbitrary

     private:
          SFE_UBLOX_GNSS GPS;
          bool init_impl () override {
               if (GPS.begin()) {
                    return true;
               }
               return false;
          }

          void* poll() override {
               ((MAX10SData*)data)->lat = GPS.getLatitude();
               ((MAX10SData*)data)->lon = GPS.getLongitude();
               ((MAX10SData*)data)->altMSL = GPS.getAltitudeMSL();
               ((MAX10SData*)data)->altAGL = GPS.getAltitude(); // not actually AGL, returns ellipsoid height (source ChatGPT)
               ((MAX10SData*)data)->velN = GPS.getNedNorthVel();
               ((MAX10SData*)data)->velE = GPS.getNedEastVel();
               ((MAX10SData*)data)->velD = GPS.getNedDownVel();
               ((MAX10SData*)data)->epochTime = GPS.getUnixEpoch();
               ((MAX10SData*)data)->satellites = GPS.getSIV(); // Satellites In View

               uint8_t fix_type = GPS.getFixType();
               ((MAX10SData*)data)->gpsLock = (fix_type == 3 || fix_type == 4);
               // there are 6 types of locks, 3 or 4 are needed for 3D positioning
               // may be useful to store fix type not just true/false

               return data;
          }
};