#pragma once

#include <cmath>

class Velocity {
  private:
    // acc
    double accelPrevTime;
    double vx, vy, vz;

    // baro
    double baroPrevTime;
    double baroPrevAlt;
    double baroVelocity;
    
    public:
      Velocity();

      // acc
      void updateAcc(double ax, double ay, double az, double currTime);
      double getAccVX() const;
      double getAccVY() const;
      double getAccVZ() const;

      // baro
      void updateBaro(double alt, double currTime);
      double getBaroV() const;    
};
