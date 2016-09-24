/*
 * math_utils.h
 *
 *  Created on: December 4, 2015
 *      Author: Pavel Vechersky
 *
 *  Class for performing various math helper functions. Since none of the
 *  funtions require any internal parameters, all the functions are static for
 *  ease of utilizing it.
 */

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

#include <math.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>

class MathUtils {
 public:
  // Compute a quaternion from roll, pitch, and yaw angles
  static Eigen::Quaterniond quaternionFromEuler(double roll, double pitch, double yaw) {
    geometry_msgs::Quaternion q_tf = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    return Eigen::Quaterniond(q_tf.w, q_tf.x, q_tf.y, q_tf.z);
  }

  // Return a random number of given type in specified range
  template<typename T> static T randNumInRange(T min, T max) {
    T range = max - min;
    return (min + static_cast <T> (rand()) / (static_cast <T> (RAND_MAX/(range))));
  }

  // Check if the two poses are equal
  static bool arePosesEqual(geometry_msgs::Pose pose_1, geometry_msgs::Pose pose_2) {
    // Check if the difference between two poses is zero
    double sum_diff = (pose_1.position.x - pose_2.position.x) +
            (pose_1.position.y - pose_2.position.y) +
            (pose_1.position.z - pose_2.position.z) +
            (pose_1.orientation.w - pose_2.orientation.w) +
            (pose_1.orientation.x - pose_2.orientation.x) +
            (pose_1.orientation.y - pose_2.orientation.y) +
            (pose_1.orientation.z - pose_2.orientation.z);

    return (sum_diff == 0.0) ? true : false;
  }

  // Wrap the angle to be from -PI to PI
  static double wrapToPi (double angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;

    while (angle < -M_PI)
      angle += 2 * M_PI;

    return angle;
  }
};

#endif /* MATH_UTILS_H__ */
