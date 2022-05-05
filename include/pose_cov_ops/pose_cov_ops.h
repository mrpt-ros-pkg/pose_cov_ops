/*
 * pose_cov_ops.h
 *
 * License: BSD 3-Clause License
 * Created on: Mar 25, 2012
 * Author: JLBC
 */

#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace pose_cov_ops {
/** @name  Pose composition: out = a (+) b
    @{ */
void compose(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b,
             geometry_msgs::Pose &out);
void compose(const geometry_msgs::PoseWithCovariance &a,
             const geometry_msgs::PoseWithCovariance &b,
             geometry_msgs::PoseWithCovariance &out);
void compose(const geometry_msgs::PoseWithCovariance &a,
             const geometry_msgs::Pose &b,
             geometry_msgs::PoseWithCovariance &out);
void compose(const geometry_msgs::Pose &a,
             const geometry_msgs::PoseWithCovariance &b,
             geometry_msgs::PoseWithCovariance &out);
/** @} */

/** @name  Pose inverse composition (a "as seen from" b): out = a (-) b
    @{ */
void inverseCompose(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b,
                    geometry_msgs::Pose &out);
void inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                    const geometry_msgs::PoseWithCovariance &b,
                    geometry_msgs::PoseWithCovariance &out);
void inverseCompose(const geometry_msgs::PoseWithCovariance &a,
                    const geometry_msgs::Pose &b,
                    geometry_msgs::PoseWithCovariance &out);
void inverseCompose(const geometry_msgs::Pose &a,
                    const geometry_msgs::PoseWithCovariance &b,
                    geometry_msgs::PoseWithCovariance &out);
/** @} */

} // namespace pose_cov_ops
