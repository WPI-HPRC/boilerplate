#include "QuaternionUtils.h"

BLA::Matrix<3, 3> QuaternionUtils::quatToRot(const BLA::Matrix<13, 1> &quat) {
    float w = quat(0);
    float x = quat(1);
    float y = quat(2);
    float z = quat(3);

    BLA::Matrix<3, 3> rot;

    // Row1
    rot(0, 0) = 1 - 2 * y * y - 2 * z * z;
    rot(0, 1) = 2 * x * y - 2 * w * z;
    rot(0, 2) = 2 * x * z + 2 * w * y;

    // Row2
    rot(1, 0) = 2 * x * y + 2 * w * z;
    rot(1, 1) = 1 - 2 * x * x - 2 * z * z;
    rot(1, 2) = 2 * y * z - 2 * w * x;

    // Row3
    rot(2, 0) = 2 * x * z - 2 * w * y;
    rot(2, 1) = 2 * y * z + 2 * w * x;
    rot(2, 2) = 1 - 2 * x * x - 2 * y * y;

    return rot;
}

BLA::Matrix<3, 3> QuaternionUtils::quat2DCM(const BLA::Matrix<13, 1> &quat) {
    float w = quat(0);
    float x = quat(1);
    float y = quat(2);
    float z = quat(3);

    BLA::Matrix<3, 3> rot;

    // Row1
    rot(0, 0) = w * w + x * x - y * y - z * z;
    rot(0, 1) = 2 * x * y + 2 * w * z;
    rot(0, 2) = 2 * x * z - 2 * w * y;

    // Row2
    rot(1, 0) = 2 * x * y - 2 * w * z;
    rot(1, 1) = w * w - x * x + y * y - z * z;
    rot(1, 2) = 2 * y * z + 2 * w * x;

    // Row3
    rot(2, 0) = 2 * x * z + 2 * w * y;
    rot(2, 1) = 2 * y * z - 2 * w * x;
    rot(2, 2) = w * w - x * x - y * y + z * z;

    return rot;
}

BLA::Matrix<3, 1> QuaternionUtils::getUpVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 1), rot(1, 1), rot(2, 1));
}

BLA::Matrix<3, 1> QuaternionUtils::getForwardVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 2), rot(1, 2), rot(2, 2));
}

BLA::Matrix<3,1> QuaternionUtils::getRightVector(const BLA::Matrix<3,3>& rot) {
    return BLA::Matrix<3,1>(rot(0,0), rot(1,0), rot(2,0));
}

BLA::Matrix<3, 1> QuaternionUtils::skewSymmetric(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<3, 3> mat;

    mat(0, 0) = 0;
    mat(0, 1) = -1 * rot(2, 0);
    mat(0, 2) = rot(1, 0);

    mat(1, 0) = rot(2, 0);
    mat(1, 1) = 0;
    mat(1, 2) = -1 * rot(0, 0);

    mat(2, 0) = -1 * rot(1, 0);
    mat(2, 1) = rot(0, 0);
    mat(2, 2) = 0;

}


BLA::Matrix<4, 1> QuaternionUtils::rotVec2Quat(const BLA::Matrix<3, 1> &vec) {
    BLA::Matrix<4, 1> quat;

    norm = BLA::Norm(vec);
    vec_rot_normed = vec / norm;

    quat(0, 0) = cos(norm / 2);
    quat(1, 0) = vec_rot_normed(0, 0) * sin(norm / 2);
    quat(2, 0) = vec_rot_normed(1, 0) * sin(norm / 2);
    quat(3, 0) = vec_rot_normed(2, 0) * sin(norm / 2);
}

BLA::Matrix<4, 1> QuaternionUtils::quatMultiply(const BLA::Matrix<4, 1> &p, const BLA::Matrix<4, 1> &q) {
    BLA::Matrix<4, 1> quat;

    quat(0, 0) = p(0, 0) * q(0, 0) - p(1, 0) * q(1, 0) - p(2, 0) * q(2, 0) - p(3, 0) * q(3, 0);
    quat(1, 0) = p(0, 0) * q(1, 0) + p(1, 0) * q(0, 0) + p(2, 0) * q(3, 0) - p(3, 0) * q(2, 0);
    quat(2, 0) = p(0, 0) * q(2, 0)  - p(1, 0) * q(3, 0) + p(2, 0) * q(1, 0) + p(3, 0) * q(1, 0);
    quat(3, 0) = p(0, 0) * q(3, 0) + p(1, 0) * q(2, 0) - p(2, 0) * q(1, 0) + p(3, 0) * q(3, 0);



}


