#pragma once

#include <stdint.h>
#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

struct MAX10SData {
    float lat;
    float lon;
    float ecefX;
    float ecefY;
    float ecefZ;
    float altMSL;
    float altEllipsoid;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t epochTime;
    uint8_t satellites;
    uint8_t gpsLockType;
};

class MAX10S: public Sensor<MAX10S, MAX10SData> {
public:
    //using DataType = MAX10SData;
    // static constexpr SensorDataType TYPE = SensorDataType::GPS;

    explicit MAX10S(TwoWire &wire = Wire) // 25
        : Sensor(40),
          GPS(),
          wire_(wire)//, last_update_ms_(0), poll_interval_ms_(1000 / info_.poll_rate_hz) {}
          {}

    bool begin_impl() {
        Serial.print("Initializing MAX10S...");

        pinMode(GPS_RESET, OUTPUT);
        digitalWrite(GPS_RESET, HIGH);

        wire_.beginTransmission(0x42);
        uint8_t i2cErr = wire_.endTransmission();
        Serial.print("I2C probe @0x42 result: ");
        Serial.println(i2cErr); // 0 = ACK/success, 2 = NACK on address (no device), others = bus error

        if (GPS.begin(wire_)) {
            GPS.setNavigationFrequency(25);
            GPS.setAutoPVT(true);
            Serial.println("OK");
            return true;
        } else {
            Serial.println("FAILED");
            return false;
        }
    }

    bool init_impl() {
        Serial.print("Initializing MAX10S... done or sometjgkasldg");
        return true;
    }

    bool poll_impl(uint32_t now_ms,  MAX10SData &out) {
        GPS.checkUblox();

        out.gpsLockType = GPS.getFixType();
        out.lat = (float)GPS.getLatitude() / 1e7;
        out.lon = (float)GPS.getLongitude() / 1e7;

        if (GPS.getNAVPOSECEF(20)) {
            out.ecefX = GPS.packetUBXNAVPOSECEF->data.ecefX * 0.01f;
            out.ecefY = GPS.packetUBXNAVPOSECEF->data.ecefY * 0.01f;
            out.ecefZ = GPS.packetUBXNAVPOSECEF->data.ecefZ * 0.01f;
        }

        out.altMSL = (float) GPS.getAltitudeMSL() / 1000.0;
        out.altEllipsoid = (float) GPS.getAltitude() / 1000.0;
        out.velN = GPS.getNedNorthVel();
        out.velE = GPS.getNedEastVel();
        out.velD = GPS.getNedDownVel();
        out.epochTime = 0;//GPS.getUnixEpoch();
        out.satellites = 0;//GPS.getSIV(); // sat in view
        return true;
    }
    
private:
    SFE_UBLOX_GNSS GPS;
    TwoWire &wire_;
};