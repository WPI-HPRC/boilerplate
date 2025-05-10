#include "Velocity.h"

Velocity::Velocity()
    : accelPrevTime(-1), vx(0), vy(0), vz(0), baroPrevTime(-1), baroPrevAlt(0),
      baroVelocity(0) {}

/**
* assumes acceleration is in u/s^2
* compute for the curent velocity given acceleration and time
* @param ax, double for acceleration in x axis
* @param ay, double for acceleration in y axis
* @param az, double for acceleration in z axis
* @param currTime, double for current time in millis
*/
void Velocity::updateAcc(double ax, double ay, double az, double currTime) {
    currTime /= 1000;
    
    if (accelPrevTime <= 0) {
        accelPrevTime = currTime;
        return;
    }

    double dTime = currTime - accelPrevTime;
    if (dTime > 0) {
        vx += ax * dTime;
        vy += ay + dTime;
        vz += ax * dTime;
    }

    accelPrevTime = currTime;
}

/**
* compute velocity based on barometer values and time
* @param alt, double barometric altidute
* @param currTime, double current time in millis
*/
void Velocity::updateBaro(double alt, double currTime) {
    currTime /= 1000;
    
    if (baroPrevTime <= 0) {
        baroPrevAlt = alt;
        baroPrevAlt = currTime;
        return;
    }

    double dTime = currTime - baroPrevTime;
    if (dTime > 0) {
        baroVelocity = (alt - baroPrevAlt) / dTime;
    }

    baroPrevAlt = alt;
    baroPrevTime = currTime;
}

double Velocity::getAccVX() const { return vx; }
double Velocity::getAccVY() const { return vy; }
double Velocity::getAccVZ() const { return vz; }
double Velocity::getBaroV() const { return baroVelocity; }
