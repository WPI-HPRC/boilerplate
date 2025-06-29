#include "AttEkf.h"
#include <Arduino.h>

// #define DBG

AttStateEstimator::AttStateEstimator(const TimedPointer<ICMData> magData, float dt) : magData(magData), dt(dt) {
    // Initialize Error Covariance

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
    
    // P(AttKFInds::accelBias[0], AttKFInds::accelBias[0]) = icm20948_const.accelXY_var;
    // P(AttKFInds::accelBias[1], AttKFInds::accelBias[1]) = icm20948_const.accelXY_var;
    // P(AttKFInds::accelBias[2], AttKFInds::accelBias[2]) = icm20948_const.accelZ_var;
    // P(AttKFInds::accelBias[0], AttKFInds::accelBias[0]) = 0.01;
    // P(AttKFInds::accelBias[1], AttKFInds::accelBias[1]) = 0.01;
    // P(AttKFInds::accelBias[2], AttKFInds::accelBias[2]) = 0.01;
    
    P_min = P;

    // Initialize Process Noise
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

void AttStateEstimator::init() {
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
    if(!hasPassedGo) {
        u_prev = u;

        hasPassedGo = true;
    }

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

BLA::Matrix<13,1> AttStateEstimator::propRK4(BLA::Matrix<3,1> u) {

    BLA::Matrix<3,1> u_k1  = u_prev;
    BLA::Matrix<3,1> u_k   = u;
    BLA::Matrix<3,1> u_k12 = 0.5f * (u_k1 + u_k);

    BLA::Matrix<13,1> k1 = dt * predictionFunction(x, u_k1);
    BLA::Matrix<13,1> k2 = dt * predictionFunction(x + k1*0.5f, u_k12);
    BLA::Matrix<13,1> k3 = dt * predictionFunction(x + k2*0.5f, u_k12);
    BLA::Matrix<13,1> k4 = dt * predictionFunction(x + k3, u_k);

    x_min = x + (k1*(1.0f/6.0f) + k2*(1.0f/3.0f) + k3*(1.0f/3.0f) + k4*(1.0f/6.0f));

    // Force Normalize Unit Quaternion
    BLA::Matrix<4,1> quat = {
        x_min(AttKFInds::q_w),
        x_min(AttKFInds::q_x),
        x_min(AttKFInds::q_y),
        x_min(AttKFInds::q_z)
    };

    quat = quat / BLA::Norm(quat);

    x_min(AttKFInds::q_w) = quat(0);
    x_min(AttKFInds::q_x) = quat(1);
    x_min(AttKFInds::q_y) = quat(2);
    x_min(AttKFInds::q_z) = quat(3);

    return x_min;
}

BLA::Matrix<13,1> AttStateEstimator::predictionFunction(BLA::Matrix<13,1> x, BLA::Matrix<3,1> u) {
    float p = u(0) - x(AttKFInds::gb_x);
    float q = u(1) - x(AttKFInds::gb_y);
    float r = u(2) - x(AttKFInds::gb_z);

    BLA::Matrix<4,3> quatMat = {
        -x(1), -x(2), -x(3),
         x(0), -x(3),  x(2),
         x(3),  x(0), -x(1), 
        -x(2),  x(1),  x(0), 
    };
    
    BLA::Matrix<3,1> gyr = {p, q, r};

    BLA::Matrix<4, 1> f_q = (quatMat * gyr) * 0.5f;

    // Assume constant bias
    BLA::Matrix<13,1> f = {
        f_q(0), f_q(1), f_q(2), f_q(3), 0, 0, 0, 0, 0, 0, 0, 0, 0
    };

    return f;
}

BLA::Matrix<13,13> AttStateEstimator::predictionJacobian(BLA::Matrix<3,1> u) {
    float gbx = x(AttKFInds::gb_x);
    float gby = x(AttKFInds::gb_y);
    float gbz = x(AttKFInds::gb_z);
    
    float p = u(0) - gbx;
    float q = u(1) - gby;
    float r = u(2) - gbz;

    float qw = x(AttKFInds::q_w);
    float qx = x(AttKFInds::q_x);
    float qy = x(AttKFInds::q_y);
    float qz = x(AttKFInds::q_z);

    // clang-format off
    BLA::Matrix<13,13> F = {
        0, gbx/2 - p/2, gby/2 - q/2, gbz/2 - r/2,  qx/2,  qy/2,  qz/2, 0, 0, 0, 0, 0, 0,
        p/2 - gbx/2,           0, r/2 - gbz/2, gby/2 - q/2, -qw/2,  qz/2, -qy/2, 0, 0, 0, 0, 0, 0,
        q/2 - gby/2, gbz/2 - r/2,           0, p/2 - gbx/2, -qz/2, -qw/2,  qx/2, 0, 0, 0, 0, 0, 0,
        r/2 - gbz/2, q/2 - gby/2, gbx/2 - p/2,           0,  qy/2, -qx/2, -qw/2, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0, 0, 0, 0,
    };
    // clang-format on

    return F;
}

void AttStateEstimator::applyGravUpdate(BLA::Matrix<13,1> &x_in, BLA::Matrix<3,1> a_b) {
    BLA::Matrix<3,1> G_NED = {0, 0, -1};

    BLA::Matrix<4,1> q = {
        x_in(AttKFInds::q_w),
        x_in(AttKFInds::q_x),
        x_in(AttKFInds::q_y),
        x_in(AttKFInds::q_z)
    };

    BLA::Matrix<3,3> R_TB = quat2rot(q);

    BLA::Matrix<3,1> bias = {
        x_in(AttKFInds::ab_x),
        x_in(AttKFInds::ab_y),
        x_in(AttKFInds::ab_z)
    };
    
    BLA::Matrix<3,1> h_grav = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB) * G_NED + bias;

    BLA::Matrix<3,1> z_grav = a_b - bias;

    float qw = x_in(AttKFInds::q_w);
    float qx = x_in(AttKFInds::q_x);
    float qy = x_in(AttKFInds::q_y);
    float qz = x_in(AttKFInds::q_z);

    float abx = x_in(AttKFInds::ab_x);
    float aby = x_in(AttKFInds::ab_y);
    float abz = x_in(AttKFInds::ab_z);

    BLA::Matrix<3, 13> H_grav = {
         2*qy, -2*qz,  2*qw, -2*qx, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        -2*qx, -2*qw, -2*qz, -2*qy, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        -4*qw,  0,     0,    -4*qz, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    };

    BLA::Matrix<3,3> S = H_grav * P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_grav) + R_grav;

    BLA::Matrix<13,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,13>>(H_grav) * BLA::Inverse(S);

    BLA::Matrix<3,1> y = z_grav - h_grav;

#ifdef DBG
    // Print innovation to teleplot three line series
    Serial.print(">Grav Innovation X: ");
    Serial.println(y(0));
    Serial.print(">Grav Innovation Y: ");
    Serial.println(y(1));
    Serial.print(">Grav Innovation Z: ");
    Serial.println(y(2));
#endif
    
    x = x_in + K * (z_grav - h_grav);

    // Normalize quaternion
    BLA::Matrix<4,1> quat = {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };

    quat = quat / BLA::Norm(quat);
    x(AttKFInds::q_w) = quat(0);
    x(AttKFInds::q_x) = quat(1);
    x(AttKFInds::q_y) = quat(2);
    x(AttKFInds::q_z) = quat(3);

    P = (I_13 - K * H_grav) * P_min;
}

void AttStateEstimator::applyMagUpdate(BLA::Matrix<13,1> &x_in, BLA::Matrix<3,1> m_b) {
    // Reference horizontal magnetic field in NED [µT]
    float B_N = 19.9583f;
    float B_E = -4.8770f;

    // Extract quaternion
    BLA::Matrix<4,1> q = {
        x_in(AttKFInds::q_w),
        x_in(AttKFInds::q_x),
        x_in(AttKFInds::q_y),
        x_in(AttKFInds::q_z)
    };

    // Extract magnetometer bias
    BLA::Matrix<3,1> bias = {
        x_in(AttKFInds::mb_x),
        x_in(AttKFInds::mb_y),
        x_in(AttKFInds::mb_z)
    };

    // Rotate reference horizontal magnetic field to body frame
    BLA::Matrix<3,3> R_TB = quat2rot(q);
    BLA::Matrix<3,1> B_ref_NED = {B_N, B_E, 0.0f};
    BLA::Matrix<3,1> h_b = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB) * B_ref_NED + bias;

    // Bias-corrected measured magnetic field
    m_b = m_b - bias;

    // Compute yaw from projected fields
    float yaw_pred = atan2f(h_b(1), h_b(0));
    float yaw_meas = atan2f(m_b(1), m_b(0));
    float yaw_err = yaw_meas - yaw_pred;

    // Wrap angle to [-pi, pi]
    while (yaw_err > M_PI) yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    // Define observation model H for heading: only q_z and gyroBias_z affect yaw
    BLA::Matrix<1,13> H_yaw = {0, 0, 0, 2.0f,   // quaternion: only qz
                               0, 0, 1.0f,      // gyroBias_z
                               0, 0, 0,         // accelBias
                               0, 0, 0};        // magBias (already removed from measurement)

    // Compute Kalman gain
    auto H_yaw_T = BLA::MatrixTranspose<BLA::Matrix<1,13>>(H_yaw);
    BLA::Matrix<1,1> S_mat = H_yaw * P_min * H_yaw_T;
    float S = S_mat(0,0) + R_mag;  // R_mag is yaw measurement variance [rad^2]
    BLA::Matrix<13,1> K = (P_min * H_yaw_T) * (1.0f / S);

#ifdef DBG
    // Print innovation to teleplot
    Serial.print(">Yaw Innovation: ");
    Serial.println(yaw_err);

    Serial.print(">Yaw: ");
    Serial.println(yaw_meas);
    Serial.print(">Yaw Pred: ");
    Serial.println(yaw_pred);
#endif

    // Update state
    x = x_in + K * yaw_err;

    // Normalize quaternion
    BLA::Matrix<4,1> quat = {
        x(AttKFInds::q_w),
        x(AttKFInds::q_x),
        x(AttKFInds::q_y),
        x(AttKFInds::q_z)
    };
    quat = quat / BLA::Norm(quat);
    x(AttKFInds::q_w) = quat(0);
    x(AttKFInds::q_x) = quat(1);
    x(AttKFInds::q_y) = quat(2);
    x(AttKFInds::q_z) = quat(3);

    // Update covariance
    P = (I_13 - K * H_yaw) * P_min;
}


BLA::Matrix<3,3> quat2rot(const BLA::Matrix<4,1> &q) {

    float q0 = q(0);
    float q1 = q(1);
    float q2 = q(2);
    float q3 = q(3);

    BLA::Matrix<3,3> rotm = {
        2*(q0*q0 + q1*q1)-1, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2),
        2*(q1*q2 + q0*q3), 2*(q0*q0 + q2*q2)-1, 2*(q2*q3 - q0*q1),
        2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 2*(q0*q0 + q3*q3)-1,
    };

    return rotm;

}
