#pragma once

#include "BasicLinearAlgebra.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/MAX10S.h"

class PVStateEstimator {
  public:
    // Remember to make the necessary variables private...

    PVStateEstimator(const TimedPointer<LPS22Data> baroData,
                     const TimedPointer<ICMData> magData,
                     const TimedPointer<MAX10SData> gpsData, float dt)
        : baroData(baroData), magData(magData), gpsData(gpsData), dt(dt) {
        // clang-format off
        F = {
            1,0,0,dt,0,0,
            0,1,0,0,dt,0,
            0,0,1,0,0,dt,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1
        };

        B = {
            0.5f*dt*dt, 0, 0,
            0,0.5*dt*dt,0,
            0,0,0.5*dt*dt,
            dt,0,0,
            0,dt,0, 
            0,0,dt 
        };
        // clang-format on
    }

    void init(BLA::Matrix<6, 1> initial); // Do I also need initial orientation?

    BLA::Matrix<6,1> onLoop(); // This should be called every loop

    // Global Variables
    float dt;                      // Loop time
    const float accelXYZ_var = 60; // Accel X,Y,Z Variance microg's/sqrt(Hz)
    const float gps_var =
        0.0; // This may need to be split into different axis, GPS Variance
    const float baro_var = 0.0; // Barometer Variance

    // Helper Constants
    constexpr static double a = 6378137.0;           // WGS-84 semi-major axis
    constexpr static double f = 1.0 / 298.257223563; // flattening
    constexpr static double e2 = f * (2 - f);        // eccentricity squared
    constexpr static double pi = 3.14159265358979323846;

  private:
    // Required Sensor data (passed in through constructor)
    const TimedPointer<LPS22Data> baroData;
    const TimedPointer<ICMData> magData;
    const TimedPointer<MAX10SData> gpsData;

    // State vector
    BLA::Matrix<6, 1> initialPV; // For reference
    BLA::Matrix<6, 1> x;

    // Kalman Gain
    BLA::Matrix<6, 3> K;

    // Estimate Covariance
    BLA::Matrix<6, 6> P;

    // Sensor Covariance Matrices, these will be defined based on sensor
    // variance
    BLA::Matrix<6, 6> Q;
    BLA::Matrix<3, 3> R;

    // F and B Matrices, make sure dt is declared beforehand in init or this
    // will fail
    BLA::Matrix<6, 6> F;

    BLA::Matrix<6, 3> B;

    // Observation Matrix H
    // clang-format off
    const BLA::Matrix<3,6> H = {
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0
    };

    const BLA::Matrix<6,6> I = {
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1
    };
    // clang-format on
    
    BLA::Matrix<6,1> updateState(BLA::Matrix<3,1> z);

    BLA::Matrix<6, 1> ned2ecef(BLA::Matrix<6, 1> state);
};
