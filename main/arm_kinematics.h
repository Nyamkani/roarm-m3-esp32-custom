#pragma once

#include "joint_mapper.h"

// RoArm-M3 manufacturer's Cartesian command convention.
// x/y/z: millimeters in the arm base coordinate system.
// t: tool tilt (radians), r: tool roll (radians),
// g: gripper joint angle (radians). This is not a generic 6-DoF pose.
struct ArmPose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double t = 0.0;
    double r = 0.0;
    double g = 0.0;
};

class ArmKinematics final {
public:
    // Manufacturer's single-branch IK. Outputs only reachable, mapper-valid joints.
    // No clamping; on failure the caller's output is unchanged.
    static bool Inverse(const ArmPose& target, JointAngles& joints);

    // Manufacturer's FK for joint positions. Does not access servo hardware.
    // On failure the caller's output is unchanged.
    static bool Forward(const JointAngles& joints, ArmPose& pose);
};
