#pragma once

#include "../Sensor/Sensor.h"
#include "Print.h"
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <cmath>

struct LPS22Data {
    double pressure;
    double temperature;
    double altitude;
};

class LPS22 : public Sensor {
  public:
    LPS22() : Sensor(sizeof(LPS22Data), 40), lps() {}

    LPS22Data getData() { return *(LPS22Data *)data; }

    void debugPrint(Print& p) override {
        p.print("pressure: "); p.print(((LPS22Data *)data)->pressure, 4); p.print(", ");
        p.print("temperature: "); p.print(((LPS22Data *)data)->temperature, 4); p.print(", ");
        p.print("altitude: "); p.print(((LPS22Data *)data)->altitude, 4); p.println();
    }

    void logCsvHeader(Print &p) override {
        p.print("pressure,temperature,altitude");
    }

    void logCsvRow(Print &p) override {
        p.print(((LPS22Data *)data)->pressure, 4); p.print(",");
        p.print(((LPS22Data *)data)->temperature, 4); p.print(",");
        p.print(((LPS22Data *)data)->altitude, 4);
    }

  private:
    Adafruit_LPS22 lps;

    bool init_impl() override {
        if (!lps.begin_I2C(0x5c)) {
            return false;
        }
        lps.setDataRate(LPS22_RATE_25_HZ);
        return true;
    }

    void poll() override {
        sensors_event_t pressure, temperature;
        lps.getEvent(&pressure, &temperature);
        ((LPS22Data *)data)->pressure = pressure.pressure;
        ((LPS22Data *)data)->temperature = temperature.temperature;
        ((LPS22Data *)data)->altitude = solveAltitude(pressure.pressure);
    }

    double solveAltitude(double pressure) {
        // physical parameters for model
        const double pb = 101325;   // [Pa] pressure at sea level
        const double Tb = 288.15;   // [K] temperature at seal level
        const double Lb = -0.0065;  // [K/m] standard temperature lapse rate
        const double hb = 0;        // [m] height at bottom of atmospheric layer (sea level)
        const double R = 8.31432;   // [N*m/mol*K] universal gas constant
        const double g0 = 9.80665;  // [m/s^2] Earth standard gravity
        const double M = 0.0289644; // [kg/mol] molar mass of Earth's air

        double pressure_Pa = pressure * 100;

        return hb + (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
    }
};
