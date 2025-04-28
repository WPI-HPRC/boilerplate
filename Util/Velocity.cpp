#include "Velocity.h"

Velocity::Velocity()
    : accelPrevTime(-1), vx(0), vy(0), vz(0), baroPrevTime(-1), baroPrevAlt(0),
      baroVelocity(0) {}

void Velocity::updateAcc(double ax, double ay, double az, double currTime) {
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

void Velocity::updateBaro(double alt, double currTime) {
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
