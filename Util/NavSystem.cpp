#include "NavSystem.h"
#include "wiring_time.h"

NavSystem::NavSystem() : currentAOA(0), verticalVelocity(0) {}

bool NavSystem::init() {
    // return sensor manager init? not sure what to do here
    return 0;
}

void NavSystem::updateAll() {
    // get sensor data
    const double currTime = millis();
    double ax, ay, az, gx, gy, gz, alt;
    // populate these from sensor data ^

    velocityTracker.updateAcc(ax, ay, az, currTime);
    // units might need to be adjusted, sensors are in mg/s^2 ?
    velocityTracker.updateBaro(
        alt, currTime); // units might be ok here, still worth checking

    orientationTracker.updatePosition(gx, gy, gz, currTime);

    // velocity components
    const double vx = velocityTracker.getAccVX();
    const double vy = velocityTracker.getAccVY();
    const double vz = velocityTracker.getAccVZ();

    // ornetation in degree
    const double rollDeg = orientationTracker.getPX();
    const double pitchDeg = orientationTracker.getPY();
    const double yawDeg = orientationTracker.getPZ();

    currentAOA =
        airbreakLogic.currentAOA(rollDeg, pitchDeg, yawDeg, vx, vy, vz);
    verticalVelocity = airbreakLogic.currentVerticalVelocity(
        vx, vy, vz); // do we average this with barometric too?
}

double NavSystem::getAngleOfAttack() const { return currentAOA; }
double NavSystem::getVerticalVelocity() const { return verticalVelocity; }
double NavSystem::getBaroVelocity() const { return velocityTracker.getBaroV(); }

void NavSystem::getVelocity(double &x,double &y, double &z) const {
    x = velocityTracker.getAccVX();
    y = velocityTracker.getAccVY();
    z = velocityTracker.getAccVZ();
}

void NavSystem::getOrientation(double &roll, double &pitch, double &yaw) {
    roll = orientationTracker.getPX();
    pitch = orientationTracker.getPY();
    yaw = orientationTracker.getPZ();
}
