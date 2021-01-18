#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from math import pi

# Initialize ROS
rospy.init_node('one')
rospy.loginfo("Starting")

pub = rospy.Publisher('one_joint_states', JointState, queue_size=10)

# Control loop
front_legs_position = pi/3
back_legs_position = -pi/3
neck_position = 0.0

while not rospy.is_shutdown():
    
    # Prepare joint state message
    jointstates = JointState()
    jointstates.header.stamp = rospy.Time.now()
    jointstates.name = ['front_joint_one', 'front_joint_two', 'back_joint_one',
                        'back_joint_two', 'base_to_neck']

    # Setting joint state positions
    while (front_legs_position > -pi/3) and (back_legs_position < pi/3):
        front_legs_position -= pi/12
        back_legs_position += pi/12

        # Publish the joint states
        jointstates.position = [front_legs_position, front_legs_position,
                                back_legs_position, back_legs_position,
                                neck_position]
        pub.publish(jointstates)

        # Wait
        rospy.sleep(0.15)

    # Setting the joint state positions (going in the opposite direction)
    while (front_legs_position < pi/3) and (back_legs_position > -pi/3):
        front_legs_position += pi/12
        back_legs_position -= pi/12

        # Publish the joint states
        jointstates.position = [front_legs_position, front_legs_position,
                                back_legs_position, back_legs_position,
                                neck_position]
        pub.publish(jointstates)

        # Wait
        rospy.sleep(0.15)
