#pragma once

#include "BasicLinearAlgebra.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/MAX10S.h"
#include "kfConsts.h"
#include <cstdint>

// Helper Constants
constexpr static double a = 6378137.0;           // WGS-84 semi-major axis
constexpr static double f = 1.0 / 298.257223563; // flattening
constexpr static double e2 = f * (2 - f);        // eccentricity squared

class PVStateEstimator {
  public:
    // TODO: Add descriptors to each of these: 

    PVStateEstimator(const TimedPointer<LPS22Data>,
                     const TimedPointer<ICMData>,
                     const TimedPointer<MAX10SData>, 
                     float dt); 

    void init(BLA::Matrix<6, 1> initialPV, BLA::Matrix<13,1> initialQuat); 
    
    BLA::Matrix<6,1> onLoop(); 

  private:

    // Loop time
    float dt = 0.0f;
    // Last GPS logged
    uint32_t lastGPSlogged = 0;  

    // Required Sensor data 
    const TimedPointer<LPS22Data> baroData;
    const TimedPointer<ICMData> accelData;
    const TimedPointer<MAX10SData> gpsData;

    // State vectors
    BLA::Matrix<6, 1> initialPV; // Initial Position 
    BLA::Matrix<13,1> initialQuat; // Initial Orientation 
    BLA::Matrix<6, 1> x; // Current PV

    // Kalman Gain
    BLA::Matrix<6, 3> K;

    // Estimate Covariance
    BLA::Matrix<6, 6> P;

    // Sensor Covariance Matrices
    BLA::Matrix<6, 6> Q;
    BLA::Matrix<3, 3> R;

    // F and G Matrices
    BLA::Matrix<6, 6> F;

    BLA::Matrix<6, 3> G;

    // Observation Matrix H and corresponding Identity 
    BLA::Matrix<3,6> H; 

    BLA::Matrix<6,6> I6 = BLA::Eye<6, 6>(); 

    
    BLA::Matrix<6,1> updateState(BLA::Matrix<3,1> z);

    BLA::Matrix<6, 1> ned2ecef(BLA::Matrix<6, 1> state);
    BLA::Matrix<3,1> lla2ecef(BLA::Matrix<3,1> lla); 
    BLA::Matrix<3,3> getRotM(BLA::Matrix<3,1> lla); 
    BLA::Matrix<3,1> body2ned(BLA::Matrix<13,1> orientation, float accelX, float accelY, float accelZ); 
};
