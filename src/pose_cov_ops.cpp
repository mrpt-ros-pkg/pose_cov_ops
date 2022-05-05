/*
 * pose_cov_ops.cpp
 *
 * License: BSD 3-Clause License
 * Created on: Mar 25, 2012
 * Author: JLBC
 */

#include "pose_cov_ops/pose_cov_ops.h"

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/ros1bridge/pose.h>

using namespace mrpt::poses;
using namespace mrpt::math;

void pose_cov_ops::compose(const geometry_msgs::Pose &a,
                           const geometry_msgs::Pose &b,
                           geometry_msgs::Pose &out) {

  out = mrpt::ros1bridge::toROS_Pose(mrpt::ros1bridge::fromROS(a) +
                                     mrpt::ros1bridge::fromROS(b));
}

void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance &a,
                           const geometry_msgs::PoseWithCovariance &b,
                           geometry_msgs::PoseWithCovariance &out) {

  const CPose3DPDFGaussian A = mrpt::ros1bridge::fromROS(a);
  const CPose3DPDFGaussian B = mrpt::ros1bridge::fromROS(b);

  const CPose3DPDFGaussian OUT = A + B;
  out = mrpt::ros1bridge::toROS_Pose(OUT);
}

void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance &a,
                           const geometry_msgs::Pose &b,
                           geometry_msgs::PoseWithCovariance &out) {
  CPose3DPDFGaussian A = mrpt::ros1bridge::fromROS(a);
  const CPose3D B = mrpt::ros1bridge::fromROS(b);

  A += B;
  out = mrpt::ros1bridge::toROS_Pose(A);
}

void pose_cov_ops::compose(const geometry_msgs::Pose &a,
                           const geometry_msgs::PoseWithCovariance &b,
                           geometry_msgs::PoseWithCovariance &out) {
  const CPose3D A = mrpt::ros1bridge::fromROS(a);
  CPose3DPDFGaussian B = mrpt::ros1bridge::fromROS(b);

  B.changeCoordinatesReference(A); // b = a (+) b
  out = mrpt::ros1bridge::toROS_Pose(B);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::Pose &a,
                                  const geometry_msgs::Pose &b,
                                  geometry_msgs::Pose &out) {
  out = mrpt::ros1bridge::toROS_Pose(mrpt::ros1bridge::fromROS(a) -
                                     mrpt::ros1bridge::fromROS(b));
}

void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  const CPose3DPDFGaussian A = mrpt::ros1bridge::fromROS(a);
  const CPose3DPDFGaussian B = mrpt::ros1bridge::fromROS(b);

  const CPose3DPDFGaussian OUT = A - B;
  out = mrpt::ros1bridge::toROS_Pose(OUT);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::Pose &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  const CPose3DPDFGaussian A = mrpt::ros1bridge::fromROS(a);
  const CPose3D B_mean = mrpt::ros1bridge::fromROS(b);

  const CPose3DPDFGaussian B(B_mean, CMatrixDouble66::Zero());

  const CPose3DPDFGaussian OUT = A - B;
  out = mrpt::ros1bridge::toROS_Pose(OUT);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::Pose &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  const CPose3D A_mean = mrpt::ros1bridge::fromROS(a);
  const CPose3DPDFGaussian B = mrpt::ros1bridge::fromROS(b);

  const CPose3DPDFGaussian A(A_mean, CMatrixDouble66::Zero());

  const CPose3DPDFGaussian OUT = A - B;
  out = mrpt::ros1bridge::toROS_Pose(OUT);
}
