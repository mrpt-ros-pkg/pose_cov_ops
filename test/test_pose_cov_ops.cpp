/*
 * test_pose_cov_ops.cpp
 *
 *  Created on: Dec 23, 2019
 *      Author: Jose Luis Blanco Claraco
 */

#include <gtest/gtest.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/version.h>
#include <pose_cov_ops/pose_cov_ops.h>

using namespace std;

TEST(PoseCovOps, composition) {
  // Test added whiel debugging report:
  // https://github.com/mrpt-ros-pkg/pose_cov_ops/issues/7

  geometry_msgs::PoseWithCovariance a;
  a.pose.position.x = -0.333330;
  a.pose.position.y = 0;
  a.pose.position.z = 0.100000;

  a.pose.orientation.x = 0.707107;
  a.pose.orientation.y = 0;
  a.pose.orientation.z = 0.707107;
  a.pose.orientation.w = -0.000005;

  geometry_msgs::PoseWithCovariance b;
  b.pose.position.x = 1.435644;
  b.pose.position.y = 0;
  b.pose.position.z = 0;
  b.pose.orientation.x = 1.000000;
  b.pose.orientation.y = 0;
  b.pose.orientation.z = 0;
  b.pose.orientation.w = 0;

  geometry_msgs::PoseWithCovariance ab;
  pose_cov_ops::compose(a, b, ab);

  mrpt::poses::CPose3D a_mrpt, b_mrpt, ab_mrpt;
  mrpt_bridge::convert(a.pose, a_mrpt);
  mrpt_bridge::convert(b.pose, b_mrpt);
  mrpt_bridge::convert(ab.pose, ab_mrpt);
#if MRPT_VERSION >= 0x199
  std::cout << "a: " << a_mrpt.asString() << "\nRot:\n"
            << a_mrpt.getRotationMatrix() << "\n";
  std::cout << "b: " << b_mrpt.asString() << "\nRot:\n"
            << b_mrpt.getRotationMatrix() << "\n";
  std::cout << "a+b: " << ab_mrpt.asString() << "\nRot:\n"
            << ab_mrpt.getRotationMatrix() << "\n";
#endif
  std::cout << "a: " << a << "\n";
  std::cout << "b: " << b << "\n";
  std::cout << "a(+)b: " << ab << "\n";

  EXPECT_NEAR(ab.pose.position.x, -0.3333333, 0.01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
