// compute curent angle of rocket
// compute curent upwards velocity

class AirbreakController {
  public:
    // pass in arguments in radian
    double currentAOA(double ax, double ay, double az, double vx, double vy, double vz);
    double currentVerticalVelocity(double vx, double vy, double vz);

  private:
    double degToRad(double deg);

    double dirX = 0.0;
    double dirY = 0.0;
    double dirZ = 0.0;
};
