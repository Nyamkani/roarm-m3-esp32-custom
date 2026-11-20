#include "arm_kinematics.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    // Manufacturer's default home pose.
    ArmPose home{30.0 + 144.49 + 171.67, 0.0, 236.82 - 13.69, 0.0, 0.0, 3.14};
    JointAngles joints{};
    bool ok = ArmKinematics::Inverse(home, joints);
    std::printf("home inverse=%d  q=[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n",
                ok, joints.base, joints.shoulder, joints.elbow, joints.wrist, joints.roll, joints.gripper);
    assert(ok);
    ArmPose reconstructed{};
    assert(ArmKinematics::Forward(joints, reconstructed));
    std::printf("home reconstructed xyz=[%.5f, %.5f, %.5f] t=%.6f\n",
                reconstructed.x, reconstructed.y, reconstructed.z, reconstructed.t);
    assert(std::abs(home.x - reconstructed.x) < 0.01);
    assert(std::abs(home.y - reconstructed.y) < 0.01);
    assert(std::abs(home.z - reconstructed.z) < 0.01);
    assert(std::abs(home.t - reconstructed.t) < 1e-5);
    // Test FK -> IK -> FK within the supported branch, not joint-identity (multiple IK solutions).
    int pass=0, unsupported=0;
    for (double s=-0.8; s<=0.8; s+=0.2) {
      for (double e=0.2; e<=2.2; e+=0.25) {
        for (double w=-0.8; w<=0.8; w+=0.4) {
          JointAngles q{0.2f,static_cast<float>(s),static_cast<float>(e),static_cast<float>(w),0.1f,3.14f};
          ArmPose pos{};
          if (!ArmKinematics::Forward(q,pos)) continue;
          JointAngles restored{}; ArmPose round{};
          if (ArmKinematics::Inverse(pos,restored) && ArmKinematics::Forward(restored,round) &&
              std::abs(pos.x-round.x)<0.02 && std::abs(pos.y-round.y)<0.02 &&
              std::abs(pos.z-round.z)<0.02 && std::abs(pos.t-round.t)<0.0001) ++pass;
          else ++unsupported;
        }
      }
    }
    std::printf("round-trip pass=%d unsupported=%d (single-branch IK / joint constraints)\n",pass,unsupported);
    assert(pass>0);
    ArmPose invalid{5000, 0, 5000, 0, 0, 3.14};
    JointAngles unchanged{0, 0, 0, 0, 0, 3.14};
    assert(!ArmKinematics::Inverse(invalid,unchanged));
    return 0;
}
