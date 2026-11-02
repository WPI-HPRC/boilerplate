#pragma once

#include "BasicLinearAlgebra.h"

#include <array>
#include <cstddef>

class SimpleEstimator {
  public:
    // Calibration keeps the most recent gyro samples when the buffer fills.
    static constexpr std::size_t gyro_buffer_size = 128;

    // Gyroscope values are in rad/s; initialize_loop should run during rest.
    bool initialize_loop(const BLA::Matrix<3, 1> &gyro);
    
    // Assumes a stationary accelerometer reads [0, 0, -1] in NED.
    bool initialize(const BLA::Matrix<3, 1> &accelerometer,
                    const BLA::Matrix<3, 1> &magnetometer,
                    float barometer_pressure_pa);
    // dt_seconds is the elapsed time since the previous gyroscope sample.
    bool flight_loop(const BLA::Matrix<3, 1> &gyro, float dt_seconds);

    BLA::Matrix<3, 1> unbiased_gyro(const BLA::Matrix<3, 1> &gyro) const;
    // Returns roll, pitch, yaw in radians.
    BLA::Matrix<3, 1> get_orientation_rpy() const;
    BLA::Matrix<3, 1> get_gyro_bias() const;
    // Uses 101325 Pa as reference pressure and subtracts the initialized height.
    float get_height(float pressure_pa) const;

  private:
    std::array<BLA::Matrix<3, 1>, gyro_buffer_size> gyro_buffer = {};
    std::size_t gyro_buffer_count = 0;
    std::size_t next_gyro_index = 0;
    BLA::Matrix<3, 1> gyro_bias = {0, 0, 0};
    BLA::Matrix<4, 1> orientation = {1, 0, 0, 0};
    float barometer_bias_meters = 0.0f;
    bool initialized = false;
};
