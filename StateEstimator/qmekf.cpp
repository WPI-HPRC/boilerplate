#include "QMEKF.h"
#include <Arduino.h>
#include <QuaternionUtils.h>




StateEstimator::StateEstimator(const TimedPointer<ICMData> IMUData, float dt) : IMUData(IMUData), dt(dt) {
    P.Fill(0.0f);
    for(uint8_t idx : AttKFInds::quat) {
        P(idx, idx) = 1e-8;
    }
	for (uint8_t idx : QMEKFInds::vel) {
		P(idx, idx) = 1e-8;
	}
	for (uint8_t idx : QMEKFInds::pos) {
		P(idx, idx) = 1e-8;
	}
    for(uint8_t idx : QMEKFInds::gyroBias) {
        P(idx, idx) = powf(icm20948_const::gyro_VRW, 2.0f);
    }
    for(uint8_t idx: QMEKFInds::accelBias) {
        P(idx, idx) = powf(icm20948_const::accelXY_VRW, 2.0f);
    }
    for(uint8_t idx : QMEKFInds::magBias) {
        // P(idx, idx) = icm20948_const.magXYZ_var;
        P(idx, idx) = powf(0.1f, 2);
    }
	for (uint8_t idx : QMEKFInds::baroBias) {
		P(idx, idx) = powf(0.1f, 2);
	}

    P_ = P;


    //ToDo: Look at this alter, big ass matrix or integration
    Q.Fill(0.0f);
    for(uint8_t idx : QMEKFInds::quat) { 
        Q(idx, idx) = 1e-8;
    }
	for(uint8_t idx : QMEKFInds::vel) { 
        Q(idx, idx) = 1e-8;
    }
	for(uint8_t idx : QMEKFInds::pos) { 
        Q(idx, idx) = 1e-8;
    }
    for(uint8_t idx : QMEKFInds::gyroBias) {
        Q(idx, idx) = powf(0.1f,2);
    }
    for(uint8_t idx : QMEKFInds::accelBias) {
        Q(idx, idx) = powf(0.00f,2);
    }
    for(uint8_t idx : QMEKFInds::magBias) {
        Q(idx, idx) = powf(0.1f, 2);
    }
	for(uint8_t idx : QMEKFInds::baroBias) {
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
void StateEstimator::init(){
	// Abhay TODO: Replace w/ TRIAD once we verify this works
    // Accelerometer
    BLA::Matrix<3,1> a_b = {
        IMUData->accelX,
        IMUData->accelY,
        IMUData->accelZ
    };
    a_b = a_b / BLA::Norm(a_b);

    // Flip gravity direction: ensure z points DOWN in body frame
    float ax = a_b(0), ay = a_b(1), az = a_b(2);

    // Compute roll (phi) and pitch (theta) assuming NED frame
    float roll  = atan2(-ay, -az);  // Flip signs to match z-down NED
    float pitch = atan2(ax, sqrt(ay*ay + az*az));

    // Magnetometer
    BLA::Matrix<3,1> m_b = {
        IMUData->magX,
        IMUData->magY,
        IMUData->magZ
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
		0, 0, 0,   // velocity
		0, 0, 0,   // position
        0, 0, 0,   // gyro bias
        0, 0, 0,   // accel bias
        0, 0, 0,   // mag bias
		0};        // baro bias

    this->x_min = x;

    lastTimeAccel = millis();
    lastTimeMag  = millis();
}


BLA::Matrix<20,1> stateEstimator::onLoop(int state) {
    // Read data from sensors and convert values
    
    float gyrX = IMUData->gyrX;
    float gyrY = IMUData->gyrY;
    float gyrZ = IMUData->gyrZ;

    float aclX = IMUData->accelX;
    float aclY = IMUData->accelY;
    float aclZ = IMUData->accelZ;

    float magX = IMUData->magX;
    float magY = IMUData->magY;
    float magZ = IMUData->magZ;
	
	// TODO get in the GPS data and baro data from somewhere
    BLA::Matrix<3,1> gyro = {gyrX, gyrY, gyrZ};   // [rad/s]
    BLA::Matrix<3,1> accel = {aclX, aclY, aclZ}; // [m/s^s] NOT G
    BLA::Matrix<3,1> mag = {magX, magY, magZ}; // [uT]
	BLA::Matrix<3,1> gps = {gpsX, gpsY, gpsZ}; // [m]
	BLA::Matrix<1,1> baro = {baroZ};
	
	// Remove biases from each measurement
	BLA::Matrix<3,1> unbiased_gyro = {gyro(0) - x(10), gyro(1) - x(11), gyro(2) - x(12)};
	BLA::Matrix<3,1> unbiased_accel = {accel(0) - x(13), accel(1) - x(14), accel(2) - x(15)};
	BLA::Matrix<3,1> umbiased_mag = {mag(0) - x(16), mag(1) - x(17), mag(2) - x(18)};
	BLA::Matrix<1,1> unbiased_baro = {baro(0) - x(20)};
	
	
	time = millis();
	run_priori = time - lastTimes(0) >= frequencies(0);
	run_accel_update = time - lastTimes(1) >= frequencies(1);
	run_mag_update = time - lastTimes(2) >= frequencies(2);
	run_gps_update = time - lastTimes(3) >= frequencies(3);
	run_baro_update = time - lastTimeBaro(4) >= frequencies(4);
	
	if(run_priori) {
		// TODO eventually implement RK4 here, but I don't understand it yet
		lastAttUpdate = // Maximum of the lastTimes(0, 1, 2)
		lastPVUpdate = // Maximum of the lastTimes(0, 3, 4)
		dt_att = millis() - lastAttUpdate;
		dt_pv = millis() - lastPVUpdate;
		
		
		
		x = x;
		P = P;
		
		
		lastTimePriori = time;
		u_prev = u;
	}
	
	if (run_accel_update || run_mag_update || run_gps_update || run_baro_update) {
		P = something;
	}
	
	if (run_accel_update) {
		x = x;
		P = P;
		lastTimeAccel = time;
	}
	
	if (run_mag_update) {
		x = x;
		P = P;
		lastTimeMag = time;
	}
	
	if (run_gps_update) {
		x = x;
		P = P;
		lastTimeGPS = time;
	}
	
	if (run_baro_update) {
		x = x;
		P = P;
		lastTimeBaro = time;
	}

    // Update previous gyro reading

    return x;
}

BLA::Matrix<20,1> StateEstimator::fastIMUProp(BLA::Matrix<3,1> gyro,
											BLA::Matrix<3, 1> accel,
											att_dt,
											pv_dt) {
												
	g = [0, 0, 9.8]
	// TODO change to from world model when go to ECEF
	
	
    BLA::Matrix<3,1> gyro_int = {gyro(0)*att_dt, gyro(1)*att_dt, gyro(2)*att_dt};
    BLA::Matrix<3,1> rotVecNorm = BLA::Norm(gyro_int);
    BLA::Matrix<3,1> axis = gyro_int / rotVecNorm;
    BLA::Matrix<3,1> dq = 
    {
        cosf(rotVecNorm(0)/2.0f)
        axis(0) * sinf(rotVecNorm(0)/2.0f),
        axis(1) * sinf(rotVecNorm(0)/2.0f),
        axis(2) * sinf(rotVecNorm(0)/2.0f),
    };
    BLA::Matrix<4,1> q = 
    {
        x(QMEKFInds::q_w),
        x(QMEKFInds::q_x),
        x(QMEKFInds::q_y),
        x(QMEKFInds::q_z)
    };
    BLA::Matrix<4,1> q = QuaternionUtils::quatMultiply(q, dq);
    BLA:Matrix<4,1> qNorm = q / BLA::Norm(q);
	
	
	
	BLA::Matrix<3,1> v_dot = QuaternionUtils::quatToRot(q) * accel + g_i;
	BLA::Matrix<3,1> old_v = {
		x(QMEKFInds::v_x), x(QMEKFInds::v_y), x(QMEKFInds::v_z)
	}
	BLA::Matrix<3,1> old_p = {
		x(QMEKFInds::p_x), x(QMEKFInds::p_y), x(QMEKFInds::p_z)
	}
	v = old_v + v_dot * pv_dt;
	p = old_p + v * dt;
	
	x(QMEKFInds::v_x) = v(0);
	x(QMEKFInds::v_y) = v(1);
	x(QMEKFInds::v_z) = v(2);
	x(QMEKFInds::p_x) = p(0);
	x(QMEKFInds::p_y) = p(1);
	x(QMEKFInds::p_z) = p(2);
}

BLA::Matrix<19, 1> AttStateEstimator::predictionFunction(BLA::Matrix<3, 1> u, BLA::Matrix<3, 1> accel) {
    
    float gyrX = IMUData->gyrX;
    float gyrY = IMUData->gyrY;
    float gyrZ = IMUData->gyrZ;   
    
    BLA::Matrix<3, 1> gyroVec = {gyrX, gyrY, gyrZ};
    BLA::Matrix<3,3> gyroSkew = QuaternionUtils::skewSymmetric(gyroVec);

    float aclX = IMUData->accelX;
    float aclY = IMUData->accelY;
    float aclZ = IMUData->accelZ;

    BLA::Matrix<3, 1> accelVec = {accelX, accelY, accelZ};
    BLA::Matrix<3,3> accelSkew = QuaternionUtils::skewSymmetric(accelVec);

    BLA::Matrix<4, 1> q =
    {
    x(AttMEKFInds::q_w),
    x(AttMEKFInds::q_x),
    x(AttMEKFInds::q_y),
    x(AttMEKFInds::q_z)
    }
    
    BLA::Matrix<3, 3> rotMatrix = QuaternionUtils::quatToRot(q);

    BLA::Matrix<19, 19> F;
    F = F.Fill(0);

    //Row 1 - 3
    F.subMatrix<3, 3>(0, QMEKFInds::q_w) = -1 * gyroSkew; 
    F.subMatrix<3, 3>(0, QMEKFInds::gb_x) = -1 * I_3;

    //Row 4 - 6
    F.subMatrix<3, 3>(QMEKFInds::v_x - 1, QMEKFInds::q_w) = -1 * rotMatrix * accelSkew;
    F.subMatrix<3, 3>(QMEKFInds::v_x - 1, QMEKFInds::ab_x) = -1 * rotMatrix;

    //Row 7 - 9
    F.subMatrix<3, 3>(QMEKDInds::P_x -1, 3) = I_3;

    F()
    
    BLA::Matrix<19, 19> phi;
    phi = phi.Fill(0);

    phi = I_19 + (F * dt) + (0.5 * F * F * pow(dt, 2));

    BLA::Matrix<19, 19> Q_d;
    Q_d = Q_d.Fill(0);

    BLA::Matrix<3, 3> gyro_var_diag;
    gyro_var_diag = gyro_var_diag.Fill(0);
    gyro_var_diag(0, 0) = QMEKFInds::gyro_var;
    gyro_var_diag(1, 1) = QMEKFInds::gyro_var;
    gryo_var_diag(2, 2) = QMEKFInds::gyro_var;

    BLA::Matrix<3, 3> gyro_bias_var_diag;
    gyro_bias_var_diag = gyro_bias_var_diag.Fill(0);
    gyro_bias_var_diag(0, 0) = QMEKFInds::gyro_bias_var;
    gyro_bias_var_diag(1, 1) = QMEKFInds::gyro_bias_var;
    gyro_bias_var_diag(2, 2) = QMEKFInds::gyro_bias_var;

    BLA::Matrix<3, 3> accel_bias_var_diag;
    accel_bias_var_diag = accel_bias_var_diag.Fill(0);
    accel_bias_var_diag(0, 0) = QMEKFInds::accel_bias_var;
    accel_bias_var_diag(1, 1) = QMEKFInds::accel_bias_var;
    accel_bias_var_diag(2, 2) = QMEKFInds::accel_bias_var;

    Q_d.subMatrix<3, 3>(QMEKFInds::q_w, QMEKFInds::q_w) = (gyro_var_diag * dt) + (gyro_bias_var_diag * (pow(dt, 3) / 10));
    Q_d.subMatrix<3, 3>(QMEKFInds::q_w, 9) = -1 * gyro_bias_var_diag * (pow(dt, 2) / 2);

    Q_d.subMatrix<3 ,3>(3, 3) = QMEKF::Inds::R_Grav * dt + aaccel_bias_var_diag * (pow(dt, 3) / 3);
    Q_d.subMatrix<3, 3>(3, 6) = accel_bias_var_diag * (pow(dt ,4) / 8.0) + QMEKFInds::R_grav * (pow(dt, 2) / 2.0);
    Q_d.subMatrix<3, 3>(3, 10) = -1.0 * accel_bias_var_diag * (pow(dt, 2) / 2.0);

    Q_d.subMatrix<3, 3>(6, 3) = QMEKFInds::R_grav * (pow(dt, 2) / 2) + accel_bias_var_diag * (pow(dt, 4) / 8.0);
    Q_d.subMatrix<3, 3>(6, 6) = QMEKFInds::R_grav * (pow(dt, 3) / 3.0) + accel_bias_var_diag * (pow(dt, 5) / 20.0);
    Q_d.subMatrix<3, 3>(6, 10) = -1.0 * accel_bias_var_diag * (pow(dt, 3) / 6.0);

    Q_d.subMatix<3, 3>(9, 0) = -1.0 * gyro_bias_var_diag * (pow(dt, 2) / 2.0);
    Q_d.subMatrix<3, 3>(9, 9) = gyro_bias_var_diag * (pow(dt, 2) / 2.0);

    Q_d.subMatrix<3, 3>(12, 3) = -1.0 * accel_bias_var_diag * (pow(dt, 2) / 2.0);
    Q_d.subMatrix<3, 3>(12, 6) = -1.0 * accel_bias_var_diag * (pow(dt, 2) / 2.0);
    Q_d.subMatrix<3, 3>(12, 12) = accel_bias_var_diag * dt;

    Q_d(15, 15) = mag_bias_var * dt;

    Q_d(18, 18) = baro_bias_var * dt;

    BLA::Matrix<19, 19> P;
    
    P = phi * P_ * phi + Q_D;

}

BLA::Matrix<20, 1> AttStateEstimator::run_accel_update(BLA::Matrix<3,1> mag_meas)
{
    BLA::Matrix<4,1> q = 
    {
        x(AttMEKFInds::q_w),
        x(AttMEKFInds::q_x),
        x(AttMEKFInds::q_y),
        x(AttMEKFInds::q_z),
    }
}

BLA::Matrix<20, 1> AttStateEstimator::run_mag_update(BLA::Matrix<3, 1> mag_meas) {
    // TODO input igrm model somehow figure out
    BLA::Matrix<4, 1> q =
    {
        x(AttMEKFInds::q_w),
        x(AttMEKFInds::q_x),
        x(AttMEKFInds::q_y),
        x(AttMEKFInds::q_z)
    };

    BLA::Matrix<3, 20> H_mag;
    H_mag = H_mag.Fill(0);
    H_mag.subMatrix<3, 3>(0, 0) = QuaternionUtils::quat2DCM(q) * igrm_model;
    H_mag.subMatrix<3, 3>(0, QMEKFInds::gb_x) = I_3;

    h_mag = QuaternionUtils::quat2DCM(q) * igrm_model;

    BLA::Matrix<3, 3> R = subMatrix(R_all); // IDK something

    EKFCalcErrorInject(oldState, oldP, mag_meas, H_mag, h_mag, R);



}

BLA::Matrix<20, 1> AttStateEstimator::run_mag_update(BLA::Matrix<20, 1> oldState, BLA::Matrix<19, 19> oldP, BLA::Matrix sens_reading, BLA::Matrix H_matrix, BLA::Matrix h, BLA::Matrix R) {
    residual = sens - h_matrix;

    S = H_matrix

}

