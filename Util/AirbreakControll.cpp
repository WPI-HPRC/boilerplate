#include <cmath>
#include "AirbreakControll.h"

// assume z is the vertical direction, this should be correct

/**
* convert degree to radian
* @param double deg: degree value
* @returns double: value in radians
*/
double AirbreakController::degToRad(double deg) {
  return deg * PI / 180.0;
}

/**
* coordinate system
*    x: right
*    y: up
*    z: forwards
*
* compute the curnet angle of attack
* @param double rollDeg: angle x in degree
* @param double pitchDeg: angle y in degree
* @param double yawDeg: angle z in degree
* @param double vx: velocity x
* @param double vy: velocity y
* @param double vz: velocity z
* @return double angle between the velocity vector, and the forwards direction of the rocket
*/
double AirbreakController::currentAOA(double rollDeg, double pitchDeg, double yawDeg, double vx, double vy, double vz) {
  // convert to radian
  double radX = degToRad(rollDeg);
  double radY = degToRad(pitchDeg);
  double radZ = degToRad(yawDeg);

  // compute sin and cos of angles
  double cosX = std::cos(radX);
  double sinX = std::sin(radX);
  double cosY = std::cos(radY);
  double sinY = std::sin(radY);
  double cosZ = std::cos(radZ);
  double sinZ = std::sin(radZ);

  // compute body forward vector in world frame
  //     we only need third col as the vector of motion for the
  //     forwards direction matters to us
  // https://en.wikipedia.org/wiki/Rotation_matrix#Basic_3D_rotations
  // https://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html
  dirX = cosZ * sinY * cosX + sinZ * sinX;
  dirY = sinZ * sinY * cosX - cosZ * sinX;
  dirZ = cosY * cosX;

  // normalize forwards vector 
  double dirMag = std::sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
  if(dirMag > 0) {
    dirX /= dirMag;
    dirY /= dirMag;
    dirZ /= dirMag;
  }

  // compute dot product of velocity and forwards direction
  double dot = vx * dirX + vy * dirY + vz * dirZ;

  // get magnitude of velocity
  double vMag = std::sqrt(vx * vx + vy * vy + vz * vz);

  // AOA = angle between velocity and forward: arccos((v*d)/|v|)
  // if velocity is 0 return 0
  return (vMag != 0) ? std::acos(dot / vMag) : 0;
}

/**
* compute the curent vertical velocity
* @param double aoa: curent angle of attack
* @param double vx: velocity x
* @param double vy: velocity y
* @param double vz: velocity z;
* @return double; curent vertical velocity
*/
double AirbreakController::currentVerticalVelocity(double vx, double vy, double vz) {
  double dot = vx * dirX + vy * dirY + vz * dirZ;
  return vz - dot * dirZ;
}
