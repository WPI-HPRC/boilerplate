#include "../include/simple_estimator.h"

#include <cmath>
#include <limits>

namespace {

bool is_finite_vector(const BLA::Matrix<3, 1> &vector) {
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(vector(axis))) {
            return false;
        }
    }
    return true;
}

float vector_norm(const BLA::Matrix<3, 1> &vector) {
    return std::sqrt(vector(0) * vector(0) +
                     vector(1) * vector(1) +
                     vector(2) * vector(2));
}

bool normalize_vector(BLA::Matrix<3, 1> &vector) {
    const float norm = vector_norm(vector);
    if (!std::isfinite(norm) || norm <= 1.0e-6f) {
        return false;
    }
    vector = vector / norm;
    return true;
}

BLA::Matrix<3, 1> cross_product(const BLA::Matrix<3, 1> &left,
                                const BLA::Matrix<3, 1> &right) {
    return {left(1) * right(2) - left(2) * right(1),
            left(2) * right(0) - left(0) * right(2),
            left(0) * right(1) - left(1) * right(0)};
}

BLA::Matrix<4, 1> matrix_to_quaternion(const BLA::Matrix<3, 3> &matrix) {
    BLA::Matrix<4, 1> quaternion;
    const float trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);

    if (trace > 0.0f) {
        const float scale = 2.0f * std::sqrt(trace + 1.0f);
        quaternion(0) = 0.25f * scale;
        quaternion(1) = (matrix(2, 1) - matrix(1, 2)) / scale;
        quaternion(2) = (matrix(0, 2) - matrix(2, 0)) / scale;
        quaternion(3) = (matrix(1, 0) - matrix(0, 1)) / scale;
    } else if (matrix(0, 0) > matrix(1, 1) &&
               matrix(0, 0) > matrix(2, 2)) {
        const float scale =
            2.0f * std::sqrt(1.0f + matrix(0, 0) - matrix(1, 1) - matrix(2, 2));
        quaternion(0) = (matrix(2, 1) - matrix(1, 2)) / scale;
        quaternion(1) = 0.25f * scale;
        quaternion(2) = (matrix(0, 1) + matrix(1, 0)) / scale;
        quaternion(3) = (matrix(0, 2) + matrix(2, 0)) / scale;
    } else if (matrix(1, 1) > matrix(2, 2)) {
        const float scale =
            2.0f * std::sqrt(1.0f + matrix(1, 1) - matrix(0, 0) - matrix(2, 2));
        quaternion(0) = (matrix(0, 2) - matrix(2, 0)) / scale;
        quaternion(1) = (matrix(0, 1) + matrix(1, 0)) / scale;
        quaternion(2) = 0.25f * scale;
        quaternion(3) = (matrix(1, 2) + matrix(2, 1)) / scale;
    } else {
        const float scale =
            2.0f * std::sqrt(1.0f + matrix(2, 2) - matrix(0, 0) - matrix(1, 1));
        quaternion(0) = (matrix(1, 0) - matrix(0, 1)) / scale;
        quaternion(1) = (matrix(0, 2) + matrix(2, 0)) / scale;
        quaternion(2) = (matrix(1, 2) + matrix(2, 1)) / scale;
        quaternion(3) = 0.25f * scale;
    }

    return quaternion;
}

BLA::Matrix<4, 1> multiply_quaternions(const BLA::Matrix<4, 1> &left,
                                      const BLA::Matrix<4, 1> &right) {
    BLA::Matrix<4, 1> result;
    result(0) = left(0) * right(0) - left(1) * right(1) -
                left(2) * right(2) - left(3) * right(3);
    result(1) = left(0) * right(1) + left(1) * right(0) +
                left(2) * right(3) - left(3) * right(2);
    result(2) = left(0) * right(2) - left(1) * right(3) +
                left(2) * right(0) + left(3) * right(1);
    result(3) = left(0) * right(3) + left(1) * right(2) -
                left(2) * right(1) + left(3) * right(0);
    return result;
}

bool normalize_quaternion(BLA::Matrix<4, 1> &quaternion) {
    float norm_squared = 0.0f;
    for (int component = 0; component < 4; ++component) {
        norm_squared += quaternion(component) * quaternion(component);
    }

    const float norm = std::sqrt(norm_squared);
    if (!std::isfinite(norm) || norm <= 1.0e-6f) {
        return false;
    }
    quaternion = quaternion / norm;
    return true;
}

}  // namespace

bool SimpleEstimator::initialize_loop(const BLA::Matrix<3, 1> &gyro) {
    if (!is_finite_vector(gyro)) {
        return false;
    }

    gyro_buffer[next_gyro_index] = gyro;
    next_gyro_index = (next_gyro_index + 1) % gyro_buffer_size;
    if (gyro_buffer_count < gyro_buffer_size) {
        ++gyro_buffer_count;
    }
    return true;
}

bool SimpleEstimator::initialize(const BLA::Matrix<3, 1> &accelerometer,
                                 const BLA::Matrix<3, 1> &magnetometer,
                                 float barometer_pressure_pa) {
    if (gyro_buffer_count == 0 || !is_finite_vector(accelerometer) ||
        !is_finite_vector(magnetometer) ||
        !std::isfinite(barometer_pressure_pa) ||
        barometer_pressure_pa <= 0.0f) {
        return false;
    }

    BLA::Matrix<3, 1> accel_body = accelerometer;
    BLA::Matrix<3, 1> mag_body = magnetometer;
    if (!normalize_vector(accel_body) || !normalize_vector(mag_body)) {
        return false;
    }

    BLA::Matrix<3, 1> accel_reference = {0, 0, -1};
    BLA::Matrix<3, 1> mag_reference = {1, 0, 0};
    BLA::Matrix<3, 1> body_cross = cross_product(accel_body, mag_body);
    BLA::Matrix<3, 1> reference_cross =
        cross_product(accel_reference, mag_reference);
    if (!normalize_vector(body_cross) || !normalize_vector(reference_cross)) {
        return false;
    }

    const BLA::Matrix<3, 1> body_third = cross_product(accel_body, body_cross);
    const BLA::Matrix<3, 1> reference_third =
        cross_product(accel_reference, reference_cross);

    BLA::Matrix<3, 3> body_basis;
    BLA::Matrix<3, 3> reference_basis;
    for (int row = 0; row < 3; ++row) {
        body_basis(row, 0) = accel_body(row);
        body_basis(row, 1) = body_cross(row);
        body_basis(row, 2) = body_third(row);
        reference_basis(row, 0) = accel_reference(row);
        reference_basis(row, 1) = reference_cross(row);
        reference_basis(row, 2) = reference_third(row);
    }

    BLA::Matrix<3, 3> body_to_ned = reference_basis * ~body_basis;
    BLA::Matrix<4, 1> initial_orientation =
        matrix_to_quaternion(body_to_ned);
    if (!normalize_quaternion(initial_orientation)) {
        return false;
    }

    gyro_bias.Fill(0.0f);
    for (std::size_t sample = 0; sample < gyro_buffer_count; ++sample) {
        for (int axis = 0; axis < 3; ++axis) {
            gyro_bias(axis) += gyro_buffer[sample](axis);
        }
    }
    gyro_bias = gyro_bias / static_cast<float>(gyro_buffer_count);

    barometer_bias_meters =
        44330.0f *
        (1.0f - std::pow(barometer_pressure_pa / 101325.0f, 1.0f / 5.255f));
    orientation = initial_orientation;
    initialized = true;
    return true;
}

bool SimpleEstimator::flight_loop(const BLA::Matrix<3, 1> &gyro,
                                  float dt_seconds) {
    if (!initialized || !is_finite_vector(gyro) ||
        !std::isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        return false;
    }

    const BLA::Matrix<3, 1> corrected_gyro = unbiased_gyro(gyro);
    const BLA::Matrix<3, 1> rotation_vector = corrected_gyro * dt_seconds;
    const float angle = vector_norm(rotation_vector);
    if (!std::isfinite(angle)) {
        return false;
    }

    BLA::Matrix<4, 1> delta_orientation;
    if (angle <= 1.0e-6f) {
        delta_orientation(0) = 1.0f;
        delta_orientation(1) = 0.5f * rotation_vector(0);
        delta_orientation(2) = 0.5f * rotation_vector(1);
        delta_orientation(3) = 0.5f * rotation_vector(2);
    } else {
        const float half_angle = 0.5f * angle;
        const float vector_scale = std::sin(half_angle) / angle;
        delta_orientation(0) = std::cos(half_angle);
        delta_orientation(1) = vector_scale * rotation_vector(0);
        delta_orientation(2) = vector_scale * rotation_vector(1);
        delta_orientation(3) = vector_scale * rotation_vector(2);
    }

    BLA::Matrix<4, 1> updated_orientation =
        multiply_quaternions(orientation, delta_orientation);
    if (!normalize_quaternion(updated_orientation)) {
        return false;
    }
    orientation = updated_orientation;
    return true;
}

BLA::Matrix<3, 1> SimpleEstimator::unbiased_gyro(
    const BLA::Matrix<3, 1> &gyro) const {
    return gyro - gyro_bias;
}

BLA::Matrix<3, 1> SimpleEstimator::get_orientation_rpy() const {
    BLA::Matrix<3, 1> rpy;
    const float w = orientation(0);
    const float x = orientation(1);
    const float y = orientation(2);
    const float z = orientation(3);
    const float sin_pitch = 2.0f * (w * y - z * x);
    const float pitch = std::asin(sin_pitch < -1.0f ? -1.0f :
                                  (sin_pitch > 1.0f ? 1.0f : sin_pitch));

    rpy(0) = std::atan2(2.0f * (w * x + y * z),
                        1.0f - 2.0f * (x * x + y * y));
    rpy(1) = pitch;
    rpy(2) = std::atan2(2.0f * (w * z + x * y),
                        1.0f - 2.0f * (y * y + z * z));
    return rpy;
}

BLA::Matrix<3, 1> SimpleEstimator::get_gyro_bias() const {
    return gyro_bias;
}

float SimpleEstimator::get_height(float pressure_pa) const {
    if (!std::isfinite(pressure_pa) || pressure_pa <= 0.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const float pressure_altitude =
        44330.0f *
        (1.0f - std::pow(pressure_pa / 101325.0f, 1.0f / 5.255f));
    return pressure_altitude - barometer_bias_meters;
}
