#!/usr/bin/env python  
import roslib
import rospy

import tf
import turtlesim.msg



if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = "turtle_tf"
    while not rospy.is_shutdown():
      br = tf.TransformBroadcaster()
      br.sendTransform((1, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "robot",
                     "odom")


    rospy.spin()
