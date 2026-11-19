#include "joint_mapper.h"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    const double pi = 3.14159265358979323846;
    JointAngles q{};
    q.base = 0; q.shoulder = static_cast<float>(pi / 4);
    q.elbow = static_cast<float>(pi / 2);
    q.wrist = 0; q.roll = 0; q.gripper = static_cast<float>(pi);
    ServoPositions p{};
    assert(JointMapper::ToServoPositions(q, p));
    assert(p.counts[0] == 2047 && p.counts[1] == 2559 && p.counts[2] == 1535);
    assert(p.counts[3] == 2048 && p.counts[4] == 2047 && p.counts[5] == 2047);
    assert(p.counts[6] == 2048);
    JointAngles back{};
    assert(JointMapper::ToJointAngles(p, back));
    assert(std::fabs(back.shoulder - q.shoulder) < 0.002);
    p.counts[2] = 2000;
    assert(!JointMapper::ToJointAngles(p, back));
    JointAngles bad = q; bad.shoulder = 2.0f;
    assert(!JointMapper::ToServoPositions(bad, p));
    std::cout << "JointMapper host tests PASS\n";
}
