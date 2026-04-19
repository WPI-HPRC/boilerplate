#pragma once

#include "../SensorManager/SensorBase.h"
#include <Arduino.h>
#include <MicroNMEA.h>
#include "teseo_liv3f_class.h"
#include <stdint.h>

#define MSG_SZ 256
#define waitForRequest 0
#define waitForAnswer 1

// typedef LIV3FData GPSProvider::LocationUpdateParams_t;

struct LIV3FData {
    bool valid;
    double lat;
    double lon;
    float alt;
    int sats;
};
class LIV3F : public Sensor<LIV3F, LIV3FData> {
  public:
    using DataType = LIV3FData;

    LIV3F()
        : Sensor(40),
          gpsSerial((PinName)GPS_SERIAL_RX, (PinName)GPS_SERIAL_TX) {}

    /*
    #define GPS_SERIAL_TX       PB14
    #define GPS_SERIAL_RX       PB15
    */

    bool begin_impl() {
        Serial.println("Beginning LIV3F");
        // placeholder, this sensor does not use spi so this can be ignored?
        // :sparkles:
        return true;
    }

    bool init_impl() {
        Serial.println("Initializing LIV3F");

        gpsSerial.begin(115200);
        Serial.println("Setup begin");
        // Create the device object passing to it the serial interface
        // #define GPS_RESET           PB8
        // #define GPS_INT             PB9
        gps = new TeseoLIV3F(&gpsSerial, GPS_RESET, GPS_INT);
        // Initialize the device
        gps->init();
        return true; // the init func only retunrs ok? look into
    }

    void poll_impl(uint32_t now_ms, LIV3FData &out) {
        gps->update();

        if (gpsSerial.available()) {
            Serial.print((char)gpsSerial.read());
        }

        // Get latest parsed data
        data = gps->getData();
        // Check if we have a valid fix
        if (data.gpgga_data.valid == 1) {
            out.valid = true;

            // Convert from NMEA format (ddmm.mmmm) to something usable
            double rawLat = data.gpgga_data.xyz.lat;
            double rawLon = data.gpgga_data.xyz.lon;

            int latDeg = (int)(rawLat / 100);
            double latMin = rawLat - (latDeg * 100);

            int lonDeg = (int)(rawLon / 100);
            double lonMin = rawLon - (lonDeg * 100);

            out.lat = latDeg + (latMin / 60.0);
            out.lon = lonDeg + (lonMin / 60.0);

            // Apply hemisphere
            if (data.gpgga_data.xyz.ns == 'S')
                out.lat *= -1;
            if (data.gpgga_data.xyz.ew == 'W')
                out.lon *= -1;

            // Other useful data
            out.alt = data.gpgga_data.xyz.alt;
            out.sats = data.gpgga_data.sats;
        } else {
            out.valid = false;
        }
    }

  private:
    HardwareSerial gpsSerial;
    TeseoLIV3F *gps;
    int incomingByte;
    GNSSParser_Data_t data;
    char command[32] = {0};
    char msg[256];
    int cmdType = 0;
    uint32_t t, s;
    int tracked;
    GPGGA_Info_t stored_positions[64];
    int status = 0;
    uint32_t stime = 0;
    int waitType = 0;
};
