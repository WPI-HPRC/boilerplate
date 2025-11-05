#include "AttMEKF.h"
#include <Arduino.h>
#include <QuaternionUtils.h>

AttStateEstimator::AttStateEstimator(const TimedPointer<ICMData> magData, float dt) : magData(magData), dt(dt) {
    P.Fill(0.0f);
    for(uint8_t idx : AttKFInds::quat) {
        P(idx, idx) = 1e-8;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        P(idx, idx) = powf(icm20948_const::gyro_VRW, 2.0f);
    }
    // P(AttKFInds::gb_x, AttKFInds::gb_x) = 0.0000068469;
    // P(AttKFInds::gb_y, AttKFInds::gb_y) = 0.0000076689;
    // P(AttKFInds::gb_z, AttKFInds::gb_z) = 0.0000076652;
    for(uint8_t idx: AttKFInds::accelBias) {
        P(idx, idx) = powf(icm20948_const::accelXY_VRW, 2.0f);
    }
    // P(AttKFInds::ab_x, AttKFInds::ab_x) = 0.0000095337;
    // P(AttKFInds::ab_y, AttKFInds::ab_y) = 0.0000102062;
    // P(AttKFInds::ab_z, AttKFInds::ab_z) = 0.0000108632;
    for(uint8_t idx : AttKFInds::magBias) {
        // P(idx, idx) = icm20948_const.magXYZ_var;
        P(idx, idx) = powf(0.1f, 2);
    }

    P_ = P;


    //ToDo: Look at this alter, big ass matrix or integration
    Q.Fill(0.0f);
    for(uint8_t idx : AttKFInds::quat) { 
        Q(idx, idx) = 1e-8;
    }
    for(uint8_t idx : AttKFInds::gyroBias) {
        Q(idx, idx) = powf(0.1f,2);
    }
    for(uint8_t idx : AttKFInds::accelBias) {
        Q(idx, idx) = powf(0.00f,2);
    }
    for(uint8_t idx : AttKFInds::magBias) {
        Q(idx, idx) = powf(0.1f, 2);
    }

#ifdef DBG
    Serial.println("<----- Process Noise ----->");
    for (int i = 0; i < Q.Rows; i++) {
        for (int j = 0; j < Q.Cols; j++) {
            Serial.print(String(Q(i,j)) + "\t");
        }
        Serial.println("");
    }

    Serial.println("<----- Initial Error Cov ----->");
    for (int i = 0; i < P.Rows; i++) {
        for (int j = 0; j < P.Cols; j++) {
            Serial.print(String(P(i,j)) + "\t");
        }
        Serial.println("");
    }
#endif

}
void AttStateEstimator::init(){
    // Accelerometer
    BLA::Matrix<3,1> a_b = {
        magData->accelX,
        magData->accelY,
        magData->accelZ
    };
    a_b = a_b / BLA::Norm(a_b);

    // Flip gravity direction: ensure z points DOWN in body frame
    float ax = a_b(0), ay = a_b(1), az = a_b(2);

    // Compute roll (phi) and pitch (theta) assuming NED frame
    float roll  = atan2(-ay, -az);  // Flip signs to match z-down NED
    float pitch = atan2(ax, sqrt(ay*ay + az*az));

    // Magnetometer
    BLA::Matrix<3,1> m_b = {
        magData->magX,
        magData->magY,
        magData->magZ
    };

    // Tilt compensation
    float mx = m_b(0), my = m_b(1), mz = m_b(2);

    float mx2 = mx * cos(pitch) + mz * sin(pitch);
    float my2 = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    float yaw = atan2(-my2, mx2);  // Heading angle (NED convention)

    // Convert roll/pitch/yaw to quaternion (ZYX convention)
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    BLA::Matrix<4,1> q0 = {
        cr * cp * cy + sr * sp * sy,  // w
        sr * cp * cy - cr * sp * sy,  // x
        cr * sp * cy + sr * cp * sy,  // y
        cr * cp * sy - sr * sp * cy   // z
    };

    this->x = {q0(0), q0(1), q0(2), q0(3),
        0, 0, 0,   // gyro bias
        0, 0, 0,   // accel bias
        0, 0, 0};  // mag bias

    this->x_min = x;

    lastTimeGrav = millis();
    lastTimeMag  = millis();
}


BLA::Matrix<13,1> AttStateEstimator::onLoop(bool inPrelaunch) {
    // Read data from sensors and convert values
    
    float gyrX = magData->gyrX;
    float gyrY = magData->gyrY;
    float gyrZ = magData->gyrZ;

    float aclX = magData->accelX;
    float aclY = magData->accelY;
    float aclZ = magData->accelZ;

    float magX = magData->magX;
    float magY = magData->magY;
    float magZ = magData->magZ;

    BLA::Matrix<3,1> u = {gyrX, gyrY, gyrZ};   // [rads]
    BLA::Matrix<3,1> a_b = {aclX, aclY, aclZ}; // [g]
    BLA::Matrix<3,1> m_b = {magX, magY, magZ}; // [uT]

    // Update u_prev if first iteration
    

    // x_min = x + predictionFunction(x, u) * dt;
    x_min = propRK4(u);

    // Measurement Jacobian Matrix
    BLA::Matrix<13,13> F = predictionJacobian(u);

    // Discretize Measurement
    BLA::Matrix<13,13> phi = I_13 + F * dt;

    // Predict Error Covariance
    P_min = phi * P * BLA::MatrixTranspose<BLA::Matrix<13,13>>(phi) + Q;

    x = x_min;

    P = P_min;

    if(millis() - lastTimeGrav >= 1000 && inPrelaunch) {
        applyGravUpdate(x, a_b);

        lastTimeGrav = millis();

    } else if(millis() - lastTimeMag >= 10000 && inPrelaunch) {
        // applyMagUpdate(x, m_b);

        lastTimeMag = millis();
    }

    // === Ensure P is Symmetric ===
    P = (P + BLA::MatrixTranspose<BLA::Matrix<13,13>>(P)) * 0.5f;

    // Update previous gyro reading
    u_prev = u;

    return x;
}

BLA::Matrix<13,1> AttStateEstimator::propGyro(BLA::Matrix<3,1> u) 
{
    float p = u(0) - x(AttKFInds::gb_x);
    float q = u(1) - x(AttKFInds::gb_y);
    float r = u(2) - x(AttKFInds::gb_z);

    BLA::Matrix<3,1> gyr = {p*dt, q*dt, r*dt};

    BLA::Matrix<3,1> rotVecNorm = BLA::Norm(gyr);
    BLA::Matrix<3,1> axis = gyr / rotVecNorm;
    BLA::Matrix<3,1> dq = 
    {
        cosf(rotVecNorm(0)/2.0f)
        axis(0) * sinf(rotVecNorm(0)/2.0f),
        axis(1) * sinf(rotVecNorm(0)/2.0f),
        axis(2) * sinf(rotVecNorm(0)/2.0f),
    };

    BLA::Matrix<4,1> q = 
    {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };

    BLA::Matrix<4,1> q = QuaternionUtils::quatMultiply(q, dq);

    BLA:Matrix<4,1> qNorm = q / BLA::Norm(q);


