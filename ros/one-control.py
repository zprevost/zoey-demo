#!/usr/bin/env python

import pygame
import rospy
from sensor_msgs.msg import JointState
from math import pi

# Initialize joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Initialize ROS
rospy.init_node('one')
pub = rospy.Publisher('one_joint_states', JointState, queue_size=10)
rate = rospy.Rate(10) # hz

# Control loop
front_legs_position = 0.0
back_legs_position = 0.0
neck_position = 0.0
while not rospy.is_shutdown():

    # Get joystick values
    pygame.event.pump()
    front_legs_axis = joystick.get_axis(0) # [-1, 1]
    back_legs_axis = joystick.get_axis(1) # [-1, 1]
    neck_axis = joystick.get_axis(2) # [-1, 1]

    # Prepare joint state message
    jointstates = JointState()
    jointstates.header.stamp = rospy.Time.now()
    jointstates.name = ['front_joint_one', 'front_joint_two', 'back_joint_one',
                        'back_joint_two', 'base_to_neck']

    # Compute front leg positions based on joystick
    front_legs_position = front_legs_position + 0.01 * (-front_legs_axis)
    front_legs_position = min(max(front_legs_position, -pi/2), pi/2)

    # Compute back leg positions based on joystick
    back_legs_position = back_legs_position + 0.01 * (-back_legs_axis)
    back_legs_position = min(max(back_legs_position, -pi/2), pi/2)

    # Compute neck position based on joystick
    neck_position = neck_position + pi/20 * neck_axis

    # Publish the joint states
    jointstates.position = [front_legs_position, front_legs_position,
                            back_legs_position, back_legs_position,
                            neck_position]
    pub.publish(jointstates)

    # Control the loop rate
    rate.sleep()

# Cleanup
pygame.quit()
