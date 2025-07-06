/* pose_cov_ops
 *
 * Copyright 2012-2025, Jose Luis Blanco Claraco
 * License: BSD 3-Clause License
 */

#include "pose_cov_ops/pose_cov_ops.h"

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

#if PACKAGE_ROS_VERSION == 1
#include <mrpt/ros1bridge/pose.h>
namespace m2r = mrpt::ros1bridge;
#else
#include <mrpt/ros2bridge/pose.h>
namespace m2r = mrpt::ros2bridge;
#endif

void pose_cov_ops::compose(const Pose &a, const Pose &b, Pose &out) {

  out = m2r::toROS_Pose(m2r::fromROS(a) + m2r::fromROS(b));
}

void pose_cov_ops::compose(const PoseWithCovariance &a,
                           const PoseWithCovariance &b,
                           PoseWithCovariance &out) {
  using namespace mrpt::poses;

  const CPose3DPDFGaussian A = m2r::fromROS(a);
  const CPose3DPDFGaussian B = m2r::fromROS(b);

  const CPose3DPDFGaussian OUT = A + B;
  out = m2r::toROS_Pose(OUT);
}

void pose_cov_ops::compose(const PoseWithCovariance &a, const Pose &b,
                           PoseWithCovariance &out) {
  using namespace mrpt::poses;

  CPose3DPDFGaussian A = m2r::fromROS(a);
  const CPose3D B = m2r::fromROS(b);

  A += B;
  out = m2r::toROS_Pose(A);
}

void pose_cov_ops::compose(const Pose &a, const PoseWithCovariance &b,
                           PoseWithCovariance &out) {
  using namespace mrpt::poses;

  const CPose3D A = m2r::fromROS(a);
  CPose3DPDFGaussian B = m2r::fromROS(b);

  B.changeCoordinatesReference(A); // b = a (+) b
  out = m2r::toROS_Pose(B);
}

pose_cov_ops::PoseWithCovariance
pose_cov_ops::compose(const pose_cov_ops::PoseWithCovariance &a,
                      const tf2::Transform &b) {
  using namespace mrpt::poses;

  CPose3DPDFGaussian A = m2r::fromROS(a);
  const CPose3D B = m2r::fromROS(b);

  A += B;
  return m2r::toROS_Pose(A);
}

void pose_cov_ops::inverseCompose(const Pose &a, const Pose &b, Pose &out) {
  out = m2r::toROS_Pose(m2r::fromROS(a) - m2r::fromROS(b));
}

void pose_cov_ops::inverseCompose(const PoseWithCovariance &a,
                                  const PoseWithCovariance &b,
                                  PoseWithCovariance &out) {
  using namespace mrpt::poses;

  const CPose3DPDFGaussian A = m2r::fromROS(a);
  const CPose3DPDFGaussian B = m2r::fromROS(b);

  const CPose3DPDFGaussian OUT = A - B;
  out = m2r::toROS_Pose(OUT);
}
void pose_cov_ops::inverseCompose(const PoseWithCovariance &a, const Pose &b,
                                  PoseWithCovariance &out) {
  using namespace mrpt::poses;
  using namespace mrpt::math;

  const CPose3DPDFGaussian A = m2r::fromROS(a);
  const CPose3D B_mean = m2r::fromROS(b);

  const CPose3DPDFGaussian B(B_mean, CMatrixDouble66::Zero());

  const CPose3DPDFGaussian OUT = A - B;
  out = m2r::toROS_Pose(OUT);
}

void pose_cov_ops::inverseCompose(const Pose &a, const PoseWithCovariance &b,
                                  PoseWithCovariance &out) {
  using namespace mrpt::poses;
  using namespace mrpt::math;

  const CPose3D A_mean = m2r::fromROS(a);
  const CPose3DPDFGaussian B = m2r::fromROS(b);

  const CPose3DPDFGaussian A(A_mean, CMatrixDouble66::Zero());

  const CPose3DPDFGaussian OUT = A - B;
  out = m2r::toROS_Pose(OUT);
}

pose_cov_ops::PoseWithCovariance
pose_cov_ops::inverseCompose(const pose_cov_ops::PoseWithCovariance &a,
                             const tf2::Transform &b) {
  using namespace mrpt::poses;
  using namespace mrpt::math;

  const CPose3DPDFGaussian A = m2r::fromROS(a);
  const CPose3D B_mean = m2r::fromROS(b);

  const CPose3DPDFGaussian B(B_mean, CMatrixDouble66::Zero());

  const CPose3DPDFGaussian OUT = A - B;
  return m2r::toROS_Pose(OUT);
}
