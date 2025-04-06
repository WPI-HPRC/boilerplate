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
     uint8_t gpsLockType = 0;
};

class MAX10S : public Sensor {
     public:
          MAX10S() : Sensor(sizeof(MAX10SData), 40), GPS() {} // This gps initialization defaults to i2c
          // the 40 is arbitrary

          void debugPrint() {
               Serial.print("lat: "); Serial.print(((MAX10SData *)data)->lat); Serial.print(", ");
               Serial.print("lon: "); Serial.print(((MAX10SData *)data)->lon); Serial.print(", ");
               Serial.print("altMSL: "); Serial.print(((MAX10SData *)data)->altMSL); Serial.print(", ");
               Serial.print("altAGL: "); Serial.print(((MAX10SData *)data)->altAGL); Serial.print(", ");
               Serial.print("velN: "); Serial.print(((MAX10SData *)data)->velN); Serial.print(", ");
               Serial.print("velE: "); Serial.print(((MAX10SData *)data)->velE); Serial.print(", ");
               Serial.print("velD: "); Serial.print(((MAX10SData *)data)->velD); Serial.print(", ");
               Serial.print("epochTime: "); Serial.print(((MAX10SData *)data)->epochTime); Serial.print(", ");
               Serial.print("satellites: "); Serial.print(((MAX10SData *)data)->satellites); Serial.print(", ");
               Serial.print("gpsLockType: "); Serial.print(((MAX10SData *)data)->gpsLockType); Serial.println();
          }

     private:
          SFE_UBLOX_GNSS GPS;
          bool init_impl () override {
               if (GPS.begin()) {
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
