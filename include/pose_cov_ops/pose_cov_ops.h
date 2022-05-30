/* pose_cov_ops
 *
 * Copyright 2012-2022, Jose Luis Blanco Claraco
 * License: BSD 3-Clause License
 */

#pragma once

#if PACKAGE_ROS_VERSION == 1
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#else
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#endif

namespace pose_cov_ops {

using Pose = geometry_msgs::Pose;
using PoseWithCovariance = geometry_msgs::PoseWithCovariance;


/** @name  Pose composition: out = a (+) b
    @{ */
void compose(const Pose &a, const Pose &b, Pose &out);
void compose(const PoseWithCovariance &a, const PoseWithCovariance &b,
             PoseWithCovariance &out);
void compose(const PoseWithCovariance &a, const Pose &b,
             PoseWithCovariance &out);
void compose(const Pose &a, const PoseWithCovariance &b,
             PoseWithCovariance &out);
/** @} */

/** @name  Pose inverse composition (a "as seen from" b): out = a (-) b
    @{ */
void inverseCompose(const Pose &a, const Pose &b, Pose &out);
void inverseCompose(const PoseWithCovariance &a, const PoseWithCovariance &b,
                    PoseWithCovariance &out);
void inverseCompose(const PoseWithCovariance &a, const Pose &b,
                    PoseWithCovariance &out);
void inverseCompose(const Pose &a, const PoseWithCovariance &b,
                    PoseWithCovariance &out);
/** @} */

} // namespace pose_cov_ops
