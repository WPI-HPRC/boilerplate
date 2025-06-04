#pragma once

#include "BasicLinearAlgebra.h"
#include "boilerplate/Logging/Loggable.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/Sensors/Impl/LPS22.h"
#include "boilerplate/Sensors/Impl/MAX10S.h"
#include "kfConsts.h"
#include <cstdint>

#define PVKF_LOG_DESC(X)                                                            \
    X(0, "posX", p.print(state(0), 4))                                             \
    X(1, "posY", p.print(state(1), 4))                                             \
    X(2, "posZ", p.print(state(2), 4))                                             \
    X(3, "velX", p.print(state(3), 4))                                             \
    X(4, "velY", p.print(state(4), 4))                                             \
    X(5, "velZ", p.print(state(5), 4))

class PVEkfLogger : public Loggable {
  public:
    PVEkfLogger() : Loggable(NUM_FIELDS(PVKF_LOG_DESC)) {}

    void newState(BLA::Matrix<6, 1> s) {
      initialized = true;
      state = s;
      updatedAt = millis();
    }

    const BLA::Matrix<6, 1>& getState() { return state; }

  private:
    BLA::Matrix<6, 1> state;
    uint32_t updatedAt = 0;
    bool initialized = false;

    MAKE_LOGGABLE(PVKF_LOG_DESC)

    uint32_t dataUpdatedAt() override {
      if (!initialized) return 0; // Data will not be logged
      return updatedAt;
    }
};

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
