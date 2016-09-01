#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
write a tf listener
source:
    http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20(Python)
"""

import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv


if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,
            queue_size = 1)

    rate = rospy.Rate(10.0)

    listener.waitForTransform("/turtle2", "/carrot1", rospy.Time(),
            rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("/turtle2", "/carrot1", now,
                    rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/turtle2', '/carrot1',
                    now)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException), e:
            raise e
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0]**2 + trans[1]**2)
        # pstr= "x= %3.5f, y=%3.5f" %(trans[0], trans[1])
        # rospy.loginfo(pstr)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()

