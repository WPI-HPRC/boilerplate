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
}

BLA::Matrix<12, 12> AttStateEstimator::predictionFunction(float dt) {
    float gyrX = magData->gyrX;
    float gyrY = magData->gyrY;
    float gyrZ = magData->gyrZ;

    /*
    float aclX = magData->accelX;
    float aclY = magData->accelY;
    float aclZ = magData->accelZ;

    BLA::Matrix<4, 1> q =
    {
    x(AttMEKFInds::q_w),
    x(AttMEKFInds::q_x),
    x(AttMEKFInds::q_y),
    x(AttMEKFInds::q_z)
    }
    BLA::Matrix<3, 3> rotMatrix = QuaternionUtils::quatToRot(q);
    
    BLA::Matrix<3, 1> accelVec = {accelX, accelY, accelZ};
    BLA::Matrix<3,3> accelSkew = QuaternionUtils::skewSymmetric(accelVec);
    */

    BLA::Matrix<3, 1> gyroVec = {gyrX, gyrY, gyrZ};
    BLA::Matrix<3,3> gyroSkew = QuaternionUtils::skewSymmetric(gyroVec);

    BLA::Matrix<12, 12> F;
    F = F.Fill(0);

    //Row 1 - 3
    F.subMatrix<3, 3>(0, AttMEKFInds::q_w) = -1 * gyroSkew; 
    F.subMatrix<3, 3>(0, AttMEKFInds::gb_x) = -1 * I_3;
    F()

    BLA::Matrix<12, 12> phi;
    P = P.Fill(0);

    phi = I_12 + (F * dt) + (0.5 * F * F * pow(dt, 2));
}

BLA::Matrix<12, 12> AttStateEstimator::measurementFunction(BLA::Matrix<3, 1> u, float dt, bool inPreLaunch) {
    float magX = magData->magX;
    float magY = magData->magY;
    float magZ = magData->magZ;

    float aclX = magData->accelX;
    float aclY = magData->accelY;
    float aclZ = magData->accelZ;   

    BLA::Matrix<3, 1> accelVec = {accelX, accelY, accelZ};
    BLA::Matrix<3, 1> magVec = {magX, magY, magZ};


    BLA::Matrix<3,1> g = {0, 0, -9.81};

    BLA::Matrix<4,1> q = {
        x_in(AttMEKFInds::q_w),
        x_in(AttMEKFInds::q_x),
        x_in(AttMEKFInds::q_y),
        x_in(AttMEKFInds::q_z)
    };

    BLA::Matrix<3,3> rotMatrix = quat2rot(q);

    BLA::Matrix<3, 12> H_accel;
    H_accel = H_accel.fill(0);
    H_accel.subMatrix<3, 3>(0, AttMEKFInds::q_w) = rotMatrix * g;
    H_accel.subMatrix<3, 3>(0, AttMEKFInds::ab_x - 1) = I_3;

    BLA::Matrix<3, 12> H_mag;
    H_mag = H_mag.fill(0);
       //TODO
    H_mag.subMatrix<3, 3>(0, AttMEKFInds::q_w) = rotMatrix * 67; //come back and add igrm_model       H_mag.subMatrix<3 ,3>(0, AttMEKFInds::mb_x - 1) = I_3;

    BLA::Matrix<3, 3> accel_h;
    accel_h = accel_h.fill(0);
    accel_h = QuaternionUtils::quat2DCM(q) * g

    BLA::Matrix<3, 3> mag_h;
    mag_h = mag_h.fill(0);
    //Come back igrm whatever
    mag_h = QuaternionUtils::quat2DCM(q) * 67 

    if(inPreLaunch){
        BLA::Matrix<6, 12> H;
        H = H.fill(0);
        H.subMatrix<3, 12>(0, 0) = H_accel;
        H.subMatrix<3, 12>(3, 0) = H_mag;
        
        BLA::Matrix<6, 1> h;
        h = h.fill(0);
        H.subMatrix<3, 1>(0, 0) = accel_h;
        H.subMatrix<3, 1>(3, 0) = mag_h;

        BLA::Matrix<6, 1> residual;
        residual.subMatrix<3, 1>(0, 0) = accelVec;
        residual.subMatrix<3, 1>(3, 0) = magVec;
        residual = residual - h;

        BLA::Matrix<6, 6> R;
        R = R.fill(0);
        R(0, 0) = 67;
        R(1, 1) = 67;
        R(2, 2) = 67;
        R(3 ,3) = 67;
        R(4, 4) = 67;
        R(5, 5) = 67

    }
    else{
        BLA::Matrix<3, 12> H = H_mag;
        BLA::Matrix<3, 1> h = mag_h;
        BLA::Matrix<3, 1> residual = magVec - h;

        BLA::Matrix<3, 3> R;
        R = R.fill(0);
        R(0, 0) = 67;
        R(1, 1) = 67;
        R(2, 2) = 67;
    }

    
}
