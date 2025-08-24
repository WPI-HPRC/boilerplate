#pragma once

/*
    Code in this file copied from
   https://github.com/WPI-HPRC/rocket-software-2024/tree/LIFT/PolarisLTS/src/SensorBoardLibraries/Barometer
*/

#include "../../Sensor/Sensor.h"
#include "../../boilerplate/Logging/Loggable.h"
#include "Print.h"
#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t RESET = 0x1E;
constexpr uint8_t ADC_READ = 0x00;
constexpr uint8_t NUM_BYTES_ADC_READ = 3;
constexpr uint8_t NUM_BYTES_PROM_DATA = 2;

constexpr uint8_t PROM_READ_ADDRESS_0 = 0xA0;
constexpr uint8_t PROM_READ_ADDRESS_1 = 0xA2;
constexpr uint8_t PROM_READ_ADDRESS_2 = 0xA4;
constexpr uint8_t PROM_READ_ADDRESS_3 = 0xA6;
constexpr uint8_t PROM_READ_ADDRESS_4 = 0xA8;
constexpr uint8_t PROM_READ_ADDRESS_5 = 0xAA;
constexpr uint8_t PROM_READ_ADDRESS_6 = 0xAC;
constexpr uint8_t PROM_READ_ADDRESS_7 = 0xAE;

constexpr uint8_t D1_OSR = 0x40; // 0100 0000 = 256 oversampling Pressure
constexpr uint8_t D2_OSR = 0x50; // 0101 0000 = 256 oversampling Temperature

struct MS5611Data {
    float pressure;
    float temperature;
    float altitude;
};

#define MS5611_LOG_DESC(X)                                                            \
    X(0, "MS5611pressure", p.print(getData()->pressure, 4))                     \
    X(1, "MS5611temperature", p.print(getData()->temperature, 4))               \
    X(2, "MS5611altitude", p.print(getData()->altitude, 4))

#define ODR 40

class MS5611 : public Sensor, public Loggable {
  public:
    MS5611(uint8_t addr = 0x77) : Sensor(sizeof(MS5611Data), ODR), addr(addr), Loggable(NUM_FIELDS(MS5611_LOG_DESC)) {}

    const TimedPointer<MS5611Data> getData() const {
        return static_cast<TimedPointer<MS5611Data>>(data);
    }

  private:
    uint8_t addr;

    MAKE_LOGGABLE(MS5611_LOG_DESC)

    struct {
        uint16_t C1 = 0;
        uint16_t C2 = 0;
        uint16_t C3 = 0;
        uint16_t C4 = 0;
        uint16_t C5 = 0;
        uint16_t C6 = 0;
    } calibrationData;

    TimedPointer<MS5611Data> setData() {
        return static_cast<TimedPointer<MS5611Data>>(data);
    }

    uint32_t dataUpdatedAt() override { return getLastTimePolled(); }

    bool init_impl() override {
        if (!readCalibrationData())
            return false;
        if (!sendCommand(RESET))
            return false;
        delay(10);

        return true;
    }

    void poll() override {
        sendCommand(D1_OSR);
        // 600 us delay for 256 oversampling ratio change as per datasheet, this
        // is what the datasheet says to be the max ADC conversion time for 256
        // oversampling ratio
        delayMicroseconds(600);
        sendCommand(ADC_READ);

        Wire.beginTransmission(addr);
        Wire.requestFrom(addr, NUM_BYTES_ADC_READ);
        uint32_t D1 = 0;
        for (int i = NUM_BYTES_ADC_READ - 1; i >= 0; i--) {
            D1 |= (uint32_t)Wire.read() << (i * 8);
        }
        Wire.endTransmission();

        sendCommand(D2_OSR);
        delayMicroseconds(600); // 600 us delay for 256 oversampling ratio
                                // change as per datasheet
        sendCommand(ADC_READ);

        Wire.beginTransmission(addr);
        Wire.requestFrom(addr, NUM_BYTES_ADC_READ);
        uint32_t D2 = 0;
        for (int i = NUM_BYTES_ADC_READ - 1; i >= 0; i--) {
            D2 |= (uint32_t)Wire.read() << (i * 8);
        }
        Wire.endTransmission();

        int32_t dT = D2 - (uint32_t)calibrationData.C5 * (1ul << 8);
        int32_t TEMP =
            2000 + (int32_t)dT * (int32_t)calibrationData.C6 / (1ll << 23);
        int64_t OFF = (int64_t)calibrationData.C2 * (1ul << 16) +
                      (int64_t)dT * (int64_t)calibrationData.C4 / (1l << 7);
        int64_t SENS = (int64_t)calibrationData.C1 * (1ul << 15) +
                       (int64_t)dT * (int64_t)calibrationData.C3 / (1l << 8);
        int32_t P = (int32_t)((D1 * SENS / (1ul << 21) - OFF) / (1ul << 15));
        setData()->pressure = (float)P / 100.0;
        setData()->temperature = (float)TEMP / 100.0;
        setData()->altitude = solveAltitude(getData()->pressure);
    }

    bool sendCommand(uint8_t command) {
        Wire.beginTransmission(addr);
        Wire.write(command);
        return Wire.endTransmission() == 0;
    }

    bool readCalibrationData() {
        uint8_t Buffer[2] = {};

        sendCommand(PROM_READ_ADDRESS_1);
        Wire.beginTransmission(this->addr);
        if (Wire.requestFrom(this->addr, NUM_BYTES_PROM_DATA) !=
            NUM_BYTES_PROM_DATA)
            return false;
        Buffer[0] = Wire.read();
        Buffer[1] = Wire.read();
        this->calibrationData.C1 = ((uint16_t)((Buffer[0] << 8) | Buffer[1]));

        sendCommand(PROM_READ_ADDRESS_2);
        Wire.beginTransmission(this->addr);
        if (Wire.requestFrom(this->addr, NUM_BYTES_PROM_DATA) !=
            NUM_BYTES_PROM_DATA)
            return false;
        Buffer[0] = Wire.read();
        Buffer[1] = Wire.read();
        this->calibrationData.C2 = ((uint16_t)((Buffer[0] << 8) | Buffer[1]));

        sendCommand(PROM_READ_ADDRESS_3);
        Wire.beginTransmission(this->addr);
        if (Wire.requestFrom(this->addr, NUM_BYTES_PROM_DATA) !=
            NUM_BYTES_PROM_DATA)
            return false;
        Buffer[0] = Wire.read();
        Buffer[1] = Wire.read();
        this->calibrationData.C3 = ((uint16_t)((Buffer[0] << 8) | Buffer[1]));

        sendCommand(PROM_READ_ADDRESS_4);
        Wire.beginTransmission(this->addr);
        if (Wire.requestFrom(this->addr, NUM_BYTES_PROM_DATA) !=
            NUM_BYTES_PROM_DATA)
            return false;
        Buffer[0] = Wire.read();
        Buffer[1] = Wire.read();
        this->calibrationData.C4 = ((uint16_t)((Buffer[0] << 8) | Buffer[1]));

        sendCommand(PROM_READ_ADDRESS_5);
        Wire.beginTransmission(this->addr);
        if (Wire.requestFrom(this->addr, NUM_BYTES_PROM_DATA) !=
            NUM_BYTES_PROM_DATA)
            return false;
        Buffer[0] = Wire.read();
        Buffer[1] = Wire.read();
        this->calibrationData.C5 = ((uint16_t)((Buffer[0] << 8) | Buffer[1]));

        sendCommand(PROM_READ_ADDRESS_6);
        Wire.beginTransmission(this->addr);
        if (Wire.requestFrom(this->addr, NUM_BYTES_PROM_DATA) !=
            NUM_BYTES_PROM_DATA)
            return false;
        Buffer[0] = Wire.read();
        Buffer[1] = Wire.read();
        this->calibrationData.C6 = ((uint16_t)((Buffer[0] << 8) | Buffer[1]));

        return Wire.endTransmission() == 0;
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
