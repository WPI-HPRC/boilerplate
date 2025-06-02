#pragma once

#include "BasicLinearAlgebra.h"

namespace QuaternionUtils {
    // Convert quaternion (w,x,y,z) to rotation matrix
    // quat SHOULD be normalized
    BLA::Matrix<3,3> quatToRot(const BLA::Matrix<13,1>& quat) {
        float w = quat(0);
        float x = quat(1);
        float y = quat(2);
        float z = quat(3);

        BLA::Matrix<3,3> rot;
        
        //Row1
        rot(0,0) = 1 - 2*y*y - 2*z*z;
        rot(0,1) = 2*x*y - 2*w*z;
        rot(0,2) = 2*x*z + 2*w*y;
        
        //Row2
        rot(1,0) = 2*x*y + 2*w*z;
        rot(1,1) = 1 - 2*x*x - 2*z*z;
        rot(1,2) = 2*y*z - 2*w*x;
        
        //Row3
        rot(2,0) = 2*x*z - 2*w*y;
        rot(2,1) = 2*y*z + 2*w*x;
        rot(2,2) = 1 - 2*x*x - 2*y*y;

        return rot;
    }

    // Get the up vector (y-axis) from the rotation matrix for the payload
    // In NED coordinates, this will be the second column of the rotation matrix
    BLA::Matrix<3,1> getUpVector(const BLA::Matrix<3,3>& rot) {
        return BLA::Matrix<3,1>(rot(0,1), rot(1,1), rot(2,1));
    }

    // Get the forward vector (z-axis) from the rotation matrix for the payload
    // NED coord == third col
    BLA::Matrix<3,1> getForwardVector(const BLA::Matrix<3,3>& rot) {
        return BLA::Matrix<3,1>(rot(0,2), rot(1,2), rot(2,2));
    }

    // Get the right vector (x-axis) from the rotation matrix for the payload
    // NED coord == first col
    BLA::Matrix<3,1> getRightVector(const BLA::Matrix<3,3>& rot) {
        return BLA::Matrix<3,1>(rot(0,0), rot(1,0), rot(2,0));
    }

    // check if a vector is pointing mostly downward (in NED coords)
    // true if the vector is within 45 degrees of pointing down
    bool isPointingDown(const BLA::Matrix<3,1>& vec) {
        // In NED, "down" means positive z component
        // cos(45°) = 0.707
        return vec(2) > 0.707;
    }

    // Enum to represent which side is on the ground
    enum class GroundSide {
        TOP,        // Y+ side down
        BOTTOM,     // Y- side down
        FRONT,      // Z+ side down
        BACK,       // Z- side down
        LEFT,       // X- side down
        RIGHT,      // X+ side down
        UNKNOWN     // No side is clearly down
    };

    // Determine which side of the robot is on the ground
    GroundSide getGroundSide(const BLA::Matrix<3,3>& rot) {

        BLA::Matrix<3,1> up = getUpVector(rot);
        BLA::Matrix<3,1> down = -up;
        BLA::Matrix<3,1> forward = getForwardVector(rot);
        BLA::Matrix<3,1> back = -forward;
        BLA::Matrix<3,1> right = getRightVector(rot);
        BLA::Matrix<3,1> left = -right;

        // check each side
        if (isPointingDown(up)) return GroundSide::TOP;
        if (isPointingDown(down)) return GroundSide::BOTTOM;
        if (isPointingDown(forward)) return GroundSide::FRONT;
        if (isPointingDown(back)) return GroundSide::BACK;
        if (isPointingDown(right)) return GroundSide::RIGHT;
        if (isPointingDown(left)) return GroundSide::LEFT;

        return GroundSide::UNKNOWN;
    }

    // convert GroundSide enum to string for debug if needed idk
    const char* groundSideToString(GroundSide side) {
        switch (side) {
            case GroundSide::TOP: return "TOP";
            case GroundSide::BOTTOM: return "BOTTOM";
            case GroundSide::FRONT: return "FRONT";
            case GroundSide::BACK: return "BACK";
            case GroundSide::LEFT: return "LEFT";
            case GroundSide::RIGHT: return "RIGHT";
            case GroundSide::UNKNOWN: return "UNKNOWN";
            default: return "INVALID";
        }
    }
}