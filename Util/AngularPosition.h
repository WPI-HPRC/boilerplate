#pragma once

#include <cmath>

class AngularPosition {
  private:
    double prevTime;
    double px, py, pz;

  public:
    void updatePosition(double vx, double vy, double vz, double currTime);
    double getPX();
    double getPY();
    double getPZ();
};
