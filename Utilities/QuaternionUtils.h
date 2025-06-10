#pragma once

#include "BasicLinearAlgebra.h"

namespace QuaternionUtils {
    // Convert quaternion (w,x,y,z) to rotation matrix
    // quat SHOULD be normalized
    BLA::Matrix<3,3> quatToRot(const BLA::Matrix<13,1>& quat);

    // Get the up vector from the rotation matrix for the payload
    // In NED coordinates, this will be the second column of the rotation matrix
    BLA::Matrix<3,1> getUpVector(const BLA::Matrix<3,3>& rot);

    // Get the forward vector from the rotation matrix for the payload
    // NED coord == third col
    BLA::Matrix<3,1> getForwardVector(const BLA::Matrix<3,3>& rot);

    // Get the right vector from the rotation matrix for the payload
    // NED coord == first col
    BLA::Matrix<3,1> getRightVector(const BLA::Matrix<3,3>& rot);
}
