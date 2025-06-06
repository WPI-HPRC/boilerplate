#pragma once

#include "BasicLinearAlgebra.h"

#include "boilerplate/Logging/Loggable.h"
#include "boilerplate/Sensors/Impl/ICM20948.h"
#include "boilerplate/TimedPointer/TimedPointer.h"
#include "kfConsts.h"
#include <cstdint>

/**
 * @name AttKfInds
 * @brief Struct holding the indices of the Attitude EKF
 */
namespace AttKFInds {
constexpr uint8_t q_w = 0;
constexpr uint8_t q_x = 1;
constexpr uint8_t q_y = 2;
constexpr uint8_t q_z = 3;
constexpr std::array<uint8_t, 4> quat = {q_w, q_x, q_y, q_z};

constexpr uint8_t gb_x = 4;
constexpr uint8_t gb_y = 5;
constexpr uint8_t gb_z = 6;
constexpr std::array<uint8_t, 3> gyroBias = {gb_x, gb_y, gb_z};

constexpr uint8_t ab_x = 7;
constexpr uint8_t ab_y = 8;
constexpr uint8_t ab_z = 9;
constexpr std::array<uint8_t, 3> accelBias = {ab_x, ab_y, ab_z};

constexpr uint8_t mb_x = 10;
constexpr uint8_t mb_y = 11;
constexpr uint8_t mb_z = 12;
constexpr std::array<uint8_t, 3> magBias = {mb_x, mb_y, mb_z};
}; // namespace AttKFInds

#define ATTKF_LOG_DESC(X)                                                      \
    X(0, "w", p.print(state(AttKFInds::q_w), 4))                               \
    X(1, "i", p.print(state(AttKFInds::q_x), 4))                               \
    X(2, "j", p.print(state(AttKFInds::q_y), 4))                               \
    X(3, "k", p.print(state(AttKFInds::q_z), 4))

class AttEkfLogger : public Loggable {
  public:
    AttEkfLogger() : Loggable(NUM_FIELDS(ATTKF_LOG_DESC)) {}

    void newState(BLA::Matrix<13, 1> s) {
      initialized = true;
      state = s;
      updatedAt = millis();
    }

    const BLA::Matrix<13, 1>& getState() { return state; }

  private:
    BLA::Matrix<13, 1> state;
    uint32_t updatedAt = 0;
    bool initialized = false;

    MAKE_LOGGABLE(ATTKF_LOG_DESC)

    uint32_t dataUpdatedAt() override {
      if (!initialized) return 0; // Data will not be loggged
      return updatedAt;
    }
};

/**
 * @name AttStateEstimator
 * @author @frostydev99
 * @brief Quaternion Attitude State Estimator
 */
class AttStateEstimator {

  public:
    AttStateEstimator(const TimedPointer<ICMData>, float dt);

    /**
     * @name init
     * @author @frostydev99
     * @param x_0 - Initial Quaternion State
     * @param dt  - Discrete time step
     */
    void init();

    /**
     * @name onLoop
     * @author @frostydev99
     * @brief Run Every Loop
     * @paragraph This method should run every loop of the expected prediction
     * update rate given by dt
     */
    BLA::Matrix<13, 1> onLoop(bool inPrelaunch);

  private:
    const TimedPointer<ICMData> magData;

    // Prediction Functions
    BLA::Matrix<13, 1> predictionFunction(BLA::Matrix<13, 1> x,
                                          BLA::Matrix<3, 1> u);
    BLA::Matrix<13, 13> predictionJacobian(BLA::Matrix<3, 1> u);

    // Update Functions
    void applyGravUpdate(BLA::Matrix<13, 1> &x, BLA::Matrix<3, 1> a_b);

    void applyMagUpdate(BLA::Matrix<13, 1> &x, BLA::Matrix<3, 1> m_b);

    /**
     * @name propRK4
     * @author @frostydev99
     * @brief Runs a RKF propagation algorithm to predict state
     * @param u - [3x1] Vector of gyro readings
     */
    BLA::Matrix<13, 1> propRK4(BLA::Matrix<3, 1> u);

    float dt = 0.0f;

    // State Vector Allocation
    BLA::Matrix<13, 1> x_min;

    BLA::Matrix<13, 1> x;

    // Error Covariance Allocation
    BLA::Matrix<13, 13> P;

    BLA::Matrix<13, 13> P_min;

    // Process Noise Covariance Allocation
    BLA::Matrix<13, 13> Q;

    // Previous control input
    BLA::Matrix<3, 1> u_prev = {0, 0, 0};

    // Identity Matrices
    BLA::Matrix<13, 13> I_13 = BLA::Eye<13, 13>();
    BLA::Matrix<3, 3> I_3 = BLA::Eye<3, 3>();

    // Gravity Update
    // BLA::Matrix<3,3> R_grav = BLA::Eye<3>() * asm330_const.accelXYZ_var;

    // clang-format off
    BLA::Matrix<3, 3> R_grav = {
        asm330_const::accelXY_var + 0.01, 0, 0,
        0, asm330_const::accelXY_var + 0.01, 0,
        0, 0, asm330_const::accelZ_var + 0.01
    };
    // clang-format on

    // BLA::Matrix<3,3> R_mag = I_3 * icm20948_const.magXYZ_var;
    float R_mag = icm20948_const::magXYZ_var;

    bool hasPassedGo = 0;

    long lastTimeGrav = 0;
    long lastTimeMag = 0;
};

template <size_t N, size_t M>
BLA::Matrix<M, 1> extractSub(const BLA::Matrix<N, 1> &x,
                             const std::array<uint8_t, M> &inds) {
    BLA::Matrix<M, 1> sub;
    for (int i = 0; i < M; i++) {
        sub(i) = x(inds[i]);
    }
    return sub;
}

BLA::Matrix<3, 3> quat2rot(const BLA::Matrix<4, 1> &q);
