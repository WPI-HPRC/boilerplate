#pragma once

#include "AirbreakControll.h"
#include "AngularPosition.h"
#include "Velocity.h"

class NavSystem {
  public:
    NavSystem();

    bool init();
    void updateAll();

    double getAngleOfAttack() const;
    double getVerticalVelocity() const;
    double getBaroVelocity() const;

    void getVelocity(double &x, double &y, double &z) const;
    void getOrientation(double &roll, double &pitch, double &yaw);

  private:
    // will there be a pointer to sensor manager somewhere here?
    // also what about time
    Velocity velocityTracker;
    AngularPosition orientationTracker;
    AirbreakController airbreakLogic;

    double currentAOA;
    double verticalVelocity;
};
