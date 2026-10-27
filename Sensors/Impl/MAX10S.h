#pragma once

#include <stdint.h>
#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

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

class MAX10S: public Sensor<MAX10S, MAX10SData> {
public:

    MAX10S() // 25
        : Sensor(40),
          GPS()//, last_update_ms_(0), poll_interval_ms_(1000 / info_.poll_rate_hz) {}
          {}

    bool begin_impl() {
        // WE are not using this sensor 
    }

    bool init_impl() {
        pinMode(GPS_RESET, OUTPUT);
        digitalWrite(GPS_RESET, HIGH);

        wire_.beginTransmission(0x42);
        uint8_t i2cErr = wire_.endTransmission();
        Serial.print("I2C probe @0x42 result: ");
        Serial.println(i2cErr); // 0 = ACK/success, 2 = NACK on address (no device), others = bus error

        if (GPS.begin(wire_)) {
            // M10 can't hit 25Hz with the default multi-constellation config, so ask for 10Hz
            if (!GPS.setNavigationFrequency(10)) {
                Serial.println("MAX10S: setNavigationFrequency rejected");
            }
            GPS.setAutoPVT(true);
            GPS.setAutoNAVPOSECEF(true); // non-blocking ECEF instead of polling every loop
            GPS.setAutoNAVSATrate(10);   // NAV-SAT once every 10 nav epochs (1Hz) for printSatellites()
            Serial.println("OK");
            return true;
        } else {
            return false;
        }
    }

    bool init_impl() {
        Serial.println("max10s init impl called!");
        return true;
    }

    bool poll_impl(uint32_t now_ms,  MAX10SData &out) {
        GPS.checkUblox();

        // only copy out data when a fresh PVT has arrived, otherwise we read stale/garbage buffers
        if (!GPS.getPVT()) {
            return false;
        }

        out.gpsLockType = GPS.getFixType();
        out.lat = (double)GPS.getLatitude() / 1e7;
        out.lon = (double)GPS.getLongitude() / 1e7;

        if (GPS.getNAVPOSECEF()) {
            out.ecefX = GPS.packetUBXNAVPOSECEF->data.ecefX * 0.01f;
            out.ecefY = GPS.packetUBXNAVPOSECEF->data.ecefY * 0.01f;
            out.ecefZ = GPS.packetUBXNAVPOSECEF->data.ecefZ * 0.01f;
        }

        out.altMSL = (float) GPS.getAltitudeMSL() / 1000.0;
        out.altEllipsoid = (float) GPS.getAltitude() / 1000.0;
        out.velN = GPS.getNedNorthVel();
        out.velE = GPS.getNedEastVel();
        out.velD = GPS.getNedDownVel();
        out.epochTime = GPS.getUnixEpoch();
        out.satellites = GPS.getSIV(); // sats used in fix
        return true;
    }

    // Debug: dump every satellite the receiver knows about with its signal strength (C/N0).
    // Non-blocking; prints nothing if no new NAV-SAT has arrived since the last call.
    void printSatellites() {
        if (!GPS.getNAVSAT()) {
            return;
        }

        const UBX_NAV_SAT_data_t &sat = GPS.packetUBXNAVSAT->data;
        uint8_t tracked = 0, used = 0, maxCno = 0;
        for (uint8_t i = 0; i < sat.header.numSvs; i++) {
            const UBX_NAV_SAT_block_t &b = sat.blocks[i];
            if (b.cno > 0) tracked++;
            if (b.flags.bits.svUsed) used++;
            if (b.cno > maxCno) maxCno = b.cno;
        }

        Serial.printf("MAX10S NAV-SAT: %u known, %u with signal, %u used, max C/N0 %u dB-Hz\r\n",
                      sat.header.numSvs, tracked, used, maxCno);

        for (uint8_t i = 0; i < sat.header.numSvs; i++) {
            const UBX_NAV_SAT_block_t &b = sat.blocks[i];
            if (b.cno == 0) continue; // skip almanac-only entries with no signal
            Serial.printf("  %c%-3u cno=%2u elev=%3d azim=%3d q=%u%s\r\n",
                          gnssLetter(b.gnssId), b.svId, b.cno, b.elev, b.azim,
                          b.flags.bits.qualityInd, b.flags.bits.svUsed ? " USED" : "");
        }
    }
    
private:
    static char gnssLetter(uint8_t gnssId) {
        switch (gnssId) {
            case 0: return 'G'; // GPS
            case 1: return 'S'; // SBAS
            case 2: return 'E'; // Galileo
            case 3: return 'B'; // BeiDou
            case 5: return 'Q'; // QZSS
            case 6: return 'R'; // GLONASS
            default: return '?';
        }
    }

    SFE_UBLOX_GNSS GPS;
    TwoWire &wire_;
};
