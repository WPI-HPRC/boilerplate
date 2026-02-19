#pragma once

#include <stdint.h>
#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include <Teseo.h>

struct MAX10SData {
    float lat;
    float lon;
    float altMSL;
    float altEllipsoid;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t epochTime;
    uint8_t satellites;
    uint8_t gpsLockType;
};

typedef LIV3FData GPSProvider::LocationUpdateParams_t;

class LIV3F: public Sensor<LIV3F, LIV3FData> {
public:
    using DataType = LIV3FData;
    // static constexpr SensorDataType TYPE = SensorDataType::GPS;

    LIV3F() // 25
        : Sensor(40),
        {
        }

    bool init_impl() {
        Serial.print("Initializing LIV3F...");
        GPS = Teseo(PinName resetPin,
          PinName wakeupPin,
          PinName ppsPin,
          PinName uartTxPin,
          PinName uartRxPin,
          I2C    *i2cBus,
          Serial *serialDebug = NULL
        );
        GPS.reset();
        GPS.start();
        if (GPS.haveDeviceInfo()){
            Serial.println("OK");
            return true;
        } else {
            Serial.println("FAILED");
            return false;
        }
    }

    void poll_impl(uint32_t now_ms,  LIV3FData &out) {
       // GPS.process() must be "called frequently"
        GPS.process();
        const auto locPointer = GPS.getLastLocation();
        out.version = locPointer->version;
        out.valid = locPointer->valid;
        out.lat = locPointer->lat;
        out.lon = locPointer->lon;
        out.altitude = locPointer->altitude;
        out.numGPSSVs = locPointer->numGPSSVs;
        out.numGLOSVs = locPointer->numGLOSVs;
        out.gpsTime = locPointer->gpsTime;
        out.utcTime = locPointer->utcTime;
    }
    
private:
    Teseo GPS;
};
