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
  CPose3DPDFGaussian A(UNINITIALIZED_POSE);
  CPose3D B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  A += B;
  mrpt_bridge::convert(A, out);
}
void pose_cov_ops::compose(const geometry_msgs::Pose &a,
                           const geometry_msgs::PoseWithCovariance &b,
                           geometry_msgs::PoseWithCovariance &out) {
  CPose3D A(UNINITIALIZED_POSE);
  CPose3DPDFGaussian B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  B.changeCoordinatesReference(A); // b = a (+) b
  mrpt_bridge::convert(B, out);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::Pose &a,
                                  const geometry_msgs::Pose &b,
                                  geometry_msgs::Pose &out) {
  CPose3D A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE), OUT(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  OUT.inverseComposeFrom(A, B);
  mrpt_bridge::convert(OUT, out);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  CPose3DPDFGaussian A(UNINITIALIZED_POSE), B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                                  const geometry_msgs::Pose &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  CPose3DPDFGaussian A(UNINITIALIZED_POSE);
  CPose3D B_mean(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A);
  mrpt_bridge::convert(b, B_mean);

  const CPose3DPDFGaussian B(B_mean, CMatrixDouble66()); // Cov=zeros

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}
void pose_cov_ops::inverseCompose(const geometry_msgs::Pose &a,
                                  const geometry_msgs::PoseWithCovariance &b,
                                  geometry_msgs::PoseWithCovariance &out) {
  CPose3D A_mean(UNINITIALIZED_POSE);
  CPose3DPDFGaussian B(UNINITIALIZED_POSE);

  mrpt_bridge::convert(a, A_mean);
  mrpt_bridge::convert(b, B);

  const CPose3DPDFGaussian A(A_mean, CMatrixDouble66()); // Cov=zeros

  const CPose3DPDFGaussian OUT = A - B;
  mrpt_bridge::convert(OUT, out);
}
