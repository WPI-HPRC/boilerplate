#if 1
#include "PVKF.h"
#include "impl/NotSoBasicLinearAlgebra.h"
#include <boilerplate/Sensors/SensorManager/SensorManager.h>
#include "boilerplate/Sensors/Sensor/Sensor.h"
#include <Arduino.h>

PVStateEstimator::PVStateEstimator(const TimedPointer<LPS22Data> baroData,
                 const TimedPointer<ICMData> accelData,
                 const TimedPointer<MAX10SData> gpsData,
                 float dt): baroData(baroData), accelData(accelData), gpsData(gpsData), dt(dt) {

    // clang-format off
    F = {
        1,0,0,dt,0,0,
        0,1,0,0,dt,0,
        0,0,1,0,0,dt,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1,
    };

    G = {
        0.5f*dt*dt, 0, 0,
        0,0.5f*dt*dt,0,
        0,0,0.5f*dt*dt,
        dt,0,0,
        0,dt,0, 
        0,0,dt, 
    };

    H = {
        1,0,0,0,0,0,
        0,1,0,0,0,0,
        0,0,1,0,0,0,
    };

    // GG^T * accel_var^2
    Q = {
        (1.0f/3.0f) * powf(dt, 3) * icm20948_const::accelXY_var, 0, 0, 0.5f * dt * dt * icm20948_const::accelXY_var, 0, 0,
        0, (1.0f/3.0f) * powf(dt, 3) * icm20948_const::accelXY_var, 0, 0, 0.5f * dt * dt * icm20948_const::accelXY_var, 0,
        0, 0, (1.0f/3.0f) * powf(dt, 3) * icm20948_const::accelZ_var, 0, 0, 0.5f * dt * dt * icm20948_const::accelZ_var,
        0.5f * dt * dt * icm20948_const::accelXY_var, 0, 0, dt * icm20948_const::accelXY_var, 0, 0,
        0, 0.5f * dt * dt * icm20948_const::accelXY_var, 0, 0, dt * icm20948_const::accelXY_var, 0,
        0, 0, 0.5f * dt * dt * icm20948_const::accelZ_var, 0, 0, dt * icm20948_const::accelZ_var, 
    };

    // Check this matrix 
    P = {
        1e-8, 0, 0, 0, 0, 0, 
        0, 1e-8, 0, 0, 0, 0,
        0, 0, 1e-8, 0, 0, 0, 
        0, 0, 0, 1e-8, 0, 0, 
        0, 0, 0, 0, 1e-8, 0, 
        0, 0, 0, 0, 0, 1e-8,
    }; 

    R = {
        2.25, 0, 0,
        0, 2.25, 0,
        0, 0, lps22_const::baro_var,
    }; 
    // clang-format on
} 

void PVStateEstimator::init(BLA::Matrix<6,1> initialPV, BLA::Matrix<13,1> initialQuat){
    this->initialPV = initialPV; 
    this->initialQuat = initialQuat; 
    this->x = initialPV; 
}

/*Converts accel x,y,z readings from Body frame to NED*/
BLA::Matrix<3,1> PVStateEstimator::body2ned(BLA::Matrix<13,1> orientation, float accelX, float accelY, float accelZ){

    // Set quaternion values 
    float qw = orientation(0); 
    float qx = orientation(1); 
    float qy = orientation(2); 
    float qz = orientation(3); 

    BLA::Matrix<3, 3> rotm = {
        qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), qw*qw - qx*qx - qy*qy + qz*qz
    };

    // Check axes
    static BLA::Matrix<3,1> grav = {0,0,g}; 
    BLA::Matrix<3,1> accel_body = {accelX, accelY, accelZ}; 
    BLA::Matrix<3,1> accel_ned = g * rotm * accel_body; 
    return accel_ned + grav; //account for gravity, assumes accel in m/s^2
}

/*Converts predicted state from lla to ECEF*/
BLA::Matrix<3,1> PVStateEstimator::lla2ecef(BLA::Matrix<3,1> lla){

    float lat = lla(0) * DEG_TO_RAD; // Convert to radians 
    float lon = lla(1) * DEG_TO_RAD; // Convert to radians 
    float alt = lla(2); 

    // Convert reference lla to ecef first 
    double N = a / sqrt(1 - e2 * powf(sin(lat), 2));

    double x = (N + alt) * cos(lat) * cos(lon);
    double y = (N + alt) * cos(lat) * sin(lon);
    double z = ((1 - e2) * N + alt) * sin(lat);

    BLA::Matrix<3,1> ecef_coords = {(float)x, (float)y, (float)z}; 

    return ecef_coords; 

} 

BLA::Matrix<3,3> PVStateEstimator::getRotM(BLA::Matrix<3,1> lla){
    float lat = lla(0) * DEG_TO_RAD; // Convert to radians 
    float lon = lla(1) * DEG_TO_RAD; // Convert to radians 

    BLA::Matrix<3,3> Rot = {
        (float)(-sin(lat) * cos(lon)), -sin(lon), -cos(lat) * cos(lon),
        -sin(lat) * sin(lon),  cos(lon), -cos(lat) * sin(lon),
         cos(lat),                 0.0,            -sin(lat)
    };

    return Rot;
}

/*Converts predicted state from NED to ECEF*/
BLA::Matrix<6,1> PVStateEstimator::ned2ecef(BLA::Matrix<6,1> state) { // change so just gps readings 
    BLA::Matrix<3,1> ref_lla = {initialPV(0),initialPV(1),initialPV(2)}; 
    BLA::Matrix<3,1> ref_ecef = lla2ecef(ref_lla);

    BLA::Matrix<3,3> Rot = getRotM(ref_lla); 

    BLA::Matrix<3,1> pos = {state(0), state(1), state(2)}; 
    BLA::Matrix<3,1> ecef_pos = ref_ecef + Rot*pos; 

    BLA::Matrix<3,1> vel = {state(3), state(4), state(5)};  
     
    //BLA::Matrix<3,1> currGPS = {gpsData->lat, gpsData->lon, 0}; 
     
    BLA::Matrix<3,1> ecef_vel = Rot * vel; // multiply by the reference rotation matrix, is this really the right thing? 

    BLA::Matrix<6,1> x_ecef = {ecef_pos(0), ecef_pos(1), ecef_pos(2),ecef_vel(0),ecef_vel(1), ecef_vel(2)}; 
    
    return x_ecef;
} 
    

BLA::Matrix<6,1> PVStateEstimator::onLoop() {

    //Convert accel readings from body2ned, depending on quat rep will need to change this 
    BLA::Matrix<3,1> accel_ned = body2ned(initialQuat, accelData->accelX, accelData->accelY, accelData->accelZ); // This will need to be fixed to align with current sensor stuff, also check correct units
    
    /*Prediction Step*/
    // Use the measurement model to predict the next state 
    BLA::Matrix<6,1> x_ned = F*x + G*accel_ned; // FIXME: Convert things here
    x = ned2ecef(x_ned);

    // Propogate Covariance
    P = F * P * BLA::MatrixTranspose(F) + Q;

    if(lastGPSlogged < gpsData.getLastUpdated()){ 

        lastGPSlogged = gpsData.getLastUpdated(); 

       // Convert barometer to correct units 
        float alt = baroData->altitude; 
        // Compile z Matrix 
        BLA::Matrix<3,1> z = {gpsData->lat, gpsData->lon, alt}; 

        //Convert to m
        z = lla2ecef(z); 

        // Now do the update step
        x = updateState(z);    
    }
   
    //x = lla2ecef(x); 

    #if defined(DEBUG)
        Serial.println("Position X: " + String(x(0))); 
        Serial.println("Position Y: " + String(x(1)));
        Serial.println("Position Z: " + String(x(2)));
        Serial.println("Velocity X: " + String(x(3)));
        Serial.println("Velocity Y: " + String(x(4)));
        Serial.println("Velocity Z: " + String(x(5))+ "\n");
      
    #endif 

    return x; 
}


BLA::Matrix<6,1> PVStateEstimator::updateState(BLA::Matrix<3,1> z){ 

    // Compute the innovation 
    BLA::Matrix<3,1> y = z - H*x; 

    // Compute S matrix 
    BLA::Matrix<3,3> S = H*P*BLA::MatrixTranspose(H) + R; 

    // Compute the Kalman Gain 
    K = P*BLA::MatrixTranspose(H)*BLA::Inverse(S); 

    //Compute updated state
    BLA::Matrix<6,1> updated_x = x + K*y; 

    // Update state covariance
    P = (I6 - K*H)*P; 

    return updated_x; 
}
#endif
