#include "AngularPosition.h";

/**
* assumes angular velocity is in u/s
* compute the angluar position given angular velocity
* @param vx, angular velocity in the x axis
* @param vy, angular velocity in the y axis
* @param vz, angular velocity in the z axis
* @param currTime, current time in millis
*/
void AngularPosition::updatePosition(double vx, double vy, double vz, double currTime) {
  currTime /= 1000;

  if(prevTime <= 0) {
    prevTime = currTime;
    return;
  }

  double dTime = currTime - prevTime;
  if(dTime > 0) {
    px = vx * dTime;
    py = vy * dTime;
    pz = vz * dTime;
  }

  prevTime = currTime;
}

double AngularPosition::getPX() { return px; }
double AngularPosition::getPY() { return py; }
double AngularPosition::getPZ() { return pz; }
