#pragma once

#include "../Sensor/Sensor.h"
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <cmath>

struct BarometerData {
    float pressure;
    float altitude;
};

class Barometer : public Sensor {
  public:
    Barometer() : Sensor(sizeof(BarometerData), 40), lps() {}

    BarometerData getData() { return *(BarometerData *)data; }

    void debugPrint() {
        Serial.print("pressure: "); Serial.print(((BarometerData *)data)->pressure); Serial.print(", ");
        Serial.print("altitude: "); Serial.print(((BarometerData *)data)->altitude); Serial.println();
    }

  private:
    Adafruit_LPS25 lps;

    bool init_impl() override {
        if (!lps.begin_I2C(0x5c)) {
            return false;
        }
        lps.setDataRate(LPS25_RATE_25_HZ);
        return true;
    }

    void *poll() override {
        sensors_event_t pressure;
        lps.getEvent(&pressure, nullptr);
        ((BarometerData *)data)->pressure = pressure.pressure;
        ((BarometerData *)data)->altitude = solveAltitude(pressure.pressure);
        return data;
    }

    float solveAltitude(float pressure) {
        // physical parameters for model
        const float pb = 101325;   // [Pa] pressure at sea level
        const float Tb = 288.15;   // [K] temperature at seal level
        const float Lb = -0.0065;  // [K/m] standard temperature lapse rate
        const float hb = 0;        // [m] height at bottom of atmospheric layer (sea level)
        const float R = 8.31432;   // [N*m/mol*K] universal gas constant
        const float g0 = 9.80665;  // [m/s^2] Earth standard gravity
        const float M = 0.0289644; // [kg/mol] molar mass of Earth's air

        float pressure_Pa = pressure * 100;

        return hb + (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
    }
};
