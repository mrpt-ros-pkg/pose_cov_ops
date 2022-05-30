/* pose_cov_ops
 *
 * Copyright 2012-2022, Jose Luis Blanco Claraco
 * License: BSD 3-Clause License
 */

#include <pose_cov_ops/pose_cov_ops.h>
//
#include <gtest/gtest.h>
#include <iostream>
#include <mrpt/poses/CPose3D.h>

#if PACKAGE_ROS_VERSION == 1
#include <mrpt/ros1bridge/pose.h>
namespace m2r = mrpt::ros1bridge;
#else
#include <mrpt/ros2bridge/pose.h>
namespace m2r = mrpt::ros2bridge;
#endif

TEST(PoseCovOps, composition) {
  using namespace std;
  using namespace pose_cov_ops;

  // Test added while debugging report:
  // https://github.com/mrpt-ros-pkg/pose_cov_ops/issues/7

  PoseWithCovariance a;
  a.pose.position.x = -0.333330;
  a.pose.position.y = 0;
  a.pose.position.z = 0.100000;

  a.pose.orientation.x = 0.707107;
  a.pose.orientation.y = 0;
  a.pose.orientation.z = 0.707107;
  a.pose.orientation.w = -0.000005;

  PoseWithCovariance b;
  b.pose.position.x = 1.435644;
  b.pose.position.y = 0;
  b.pose.position.z = 0;
  b.pose.orientation.x = 1.000000;
  b.pose.orientation.y = 0;
  b.pose.orientation.z = 0;
  b.pose.orientation.w = 0;

  PoseWithCovariance ab;
  pose_cov_ops::compose(a, b, ab);

  const mrpt::poses::CPose3D a_mrpt = m2r::fromROS(a.pose);
  const mrpt::poses::CPose3D b_mrpt = m2r::fromROS(b.pose);
  const mrpt::poses::CPose3D ab_mrpt = m2r::fromROS(ab.pose);

  std::cout << "a: " << a_mrpt.asString() << "\nRot:\n"
            << a_mrpt.getRotationMatrix() << "\n";
  std::cout << "b: " << b_mrpt.asString() << "\nRot:\n"
            << b_mrpt.getRotationMatrix() << "\n";
  std::cout << "a+b: " << ab_mrpt.asString() << "\nRot:\n"
            << ab_mrpt.getRotationMatrix() << "\n";

#if 0
  std::cout << "a: " << a << "\n";
  std::cout << "b: " << b << "\n";
  std::cout << "a(+)b: " << ab << "\n";
#endif

  EXPECT_NEAR(ab.pose.position.x, -0.3333333, 0.01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
