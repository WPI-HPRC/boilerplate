#pragma once

#include "../SensorManager/SensorBase.h"
#include "wiring_digital.h"
#include <Arduino.h>
#include <stdint.h>

#include <MicroNMEA.h>

struct LIV3FData {
    bool lock;
    float lat;
    float lon;
    float alt;
    uint32_t epochTime;
    uint8_t satellites;
};

// typedef LIV3FData GPSProvider::LocationUpdateParams_t;

class LIV3F : public Sensor<LIV3F, LIV3FData> {
  public:
    using DataType = LIV3FData;
    // static constexpr SensorDataType TYPE = SensorDataType::GPS;

    LIV3F(HardwareSerial &gps)
        : Sensor(25), gps(gps), nmea(nmea_buf, sizeof(nmea_buf)) {}

    bool begin_impl() {
        // something here
        Serial.println("Beginning LIV3F");

        pinMode(GPS_RESET, OUTPUT);
        digitalWrite(GPS_RESET, HIGH);

        gps.begin(9600);

        while (gps.available()) {
            gps.read();
        }

        digitalWrite(GPS_RESET, LOW);
        delay(50);
        digitalWrite(GPS_RESET, HIGH);
        delay(100);

        // Set the enabled messages to the ones MicroNMEA understands
        MicroNMEA::sendSentence(gps, "$PSTMSETPAR,1201,0x00000042");
        // Set the CPU clock speed to max
        MicroNMEA::sendSentence(gps, "$PSTMSETPAR,1130,0");
        // Set the baud rate to 115200
        MicroNMEA::sendSentence(gps, "$PSTMSETPAR,1102,0xA");
        // Set the fix rate to 10Hz (0.1s period)
        MicroNMEA::sendSentence(gps, "$PSTMSETPAR,1303,0.5");
        MicroNMEA::sendSentence(gps, "$PSTMSAVEPAR");

        // Reset the device so that the changes can take place
        MicroNMEA::sendSentence(gps, "$PSTMSRR");

        delay(200);

        // Reinitialize serial port with 115200 baud rate
        gps.end();
        gps.begin(115200);

        // We assume this succeeds, we technically cannot determine that at this
        // point though
        return true;
    }

    bool init_impl() {
        Serial.println("Initializing LIV3F");
        return true;
    }

    bool poll_impl(uint32_t now_ms, LIV3FData &out) {
        process_all_serial();

        if (nmea.isValid()) {
            out.lock = true;
            out.lat = (float)nmea.getLatitude() / 1e6;
            out.lon = (float)nmea.getLongitude() / 1e6;
            long alt;
            bool alt_valid = nmea.getAltitude(alt);
            if (alt_valid) {
                out.alt = (float)alt / 1e3;
            } else {
                out.alt = NAN;
            }
            out.satellites = nmea.getNumSatellites();

            nmea.clear();
            return true;
        }

        out.lock = false;

        return false;
    }

    void process_all_serial() {
        while (gps.available()) {
            char c = gps.read();
            nmea.process(c);
        }
    }

  private:
    HardwareSerial &gps;
    char nmea_buf[255];
    MicroNMEA nmea;
};
