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

BLA::Matrix<3, 1> QuaternionUtils::getUpVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 1), rot(1, 1), rot(2, 1));
}

BLA::Matrix<3, 1> QuaternionUtils::getForwardVector(const BLA::Matrix<3, 3> &rot) {
    return BLA::Matrix<3, 1>(rot(0, 2), rot(1, 2), rot(2, 2));
}

BLA::Matrix<3,1> QuaternionUtils::getRightVector(const BLA::Matrix<3,3>& rot) {
    return BLA::Matrix<3,1>(rot(0,0), rot(1,0), rot(2,0));
}
