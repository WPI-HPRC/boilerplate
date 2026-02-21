#pragma once

#include "../Sensor/Sensor.h"
#include "Print.h"
#include "boilerplate/Logging/Loggable.h"
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <cmath>

struct LPS22Data {
    double pressure;
    double temperature;
    double altitude;
};

#define LPS22_LOG_DESC(X)                                                            \
    X(0, "pressure", p.print(getData()->pressure, 4))                          \
    X(1, "temperature", p.print(getData()->temperature, 4))                    \
    X(2, "altitude", p.print(getData()->altitude, 4))

class LPS22 : public Sensor, public Loggable {
  public:
    LPS22()
        : Sensor(sizeof(LPS22Data), 20), Loggable(NUM_FIELDS(LPS22_LOG_DESC)), lps() {
    }

    const TimedPointer<LPS22Data> getData() const {
        return static_cast<TimedPointer<LPS22Data>>(data);
    }

    uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

  private:
    Adafruit_LPS22 lps;

    MAKE_LOGGABLE(LPS22_LOG_DESC)


    TimedPointer<LPS22Data> setData() {
        return static_cast<TimedPointer<LPS22Data>>(data);
    }

    bool init_impl() override {
        return false;
        if (!lps.begin_I2C(0x5c)) {
            return false;
        }
        lps.setDataRate(LPS22_RATE_50_HZ);
        return true;
    }

    void poll() override {
        sensors_event_t pressure, temperature;
        lps.getEvent(&pressure, &temperature);
        setData()->pressure = pressure.pressure;
        setData()->temperature = temperature.temperature;
        setData()->altitude = solveAltitude(pressure.pressure);
    }

    double solveAltitude(double pressure) {
        // physical parameters for model
        const double pb = 101325;  // [Pa] pressure at sea level
        const double Tb = 288.15;  // [K] temperature at seal level
        const double Lb = -0.0065; // [K/m] standard temperature lapse rate
        const double hb =
            0; // [m] height at bottom of atmospheric layer (sea level)
        const double R = 8.31432;   // [N*m/mol*K] universal gas constant
        const double g0 = 9.80665;  // [m/s^2] Earth standard gravity
        const double M = 0.0289644; // [kg/mol] molar mass of Earth's air

        double pressure_Pa = pressure * 100;

        return hb +
               (Tb / Lb) * (pow((pressure_Pa / pb), (-R * Lb / (g0 * M))) - 1);
    }
};
