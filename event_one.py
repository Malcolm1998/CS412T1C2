import signal
import rospy
import smach
import smach_ros
import math
from math import tanh
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

import time

global shutdown_requested

class RotateLeft(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done1', 'count'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:

            target_heading = (self.callbacks.heading + 90) % 360

            turning = True
            previous_difference = None
            while turning:
                difference = minimum_angle_between_headings(target_heading, self.callbacks.heading)

                if previous_difference is None:
                    self.twist.angular.z = 0.4
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 0.5:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = 0.4
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference
            return 'count'
        return 'done1'


class Count(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done1', 'rotate_right'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            time.sleep(5)
            return 'rotate_right'
        return 'done1'


class RotateRight(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done1', 'success1'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:

            target_heading = self.callbacks.heading - 90
            if target_heading < 0:
                target_heading = target_heading + 360

            turning = True
            previous_difference = None
            while turning:
                difference = minimum_angle_between_headings(target_heading, self.callbacks.heading)

                if previous_difference is None:
                    self.twist.angular.z = -0.4
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 0.5:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = -0.4
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference
            return 'success1'
        return 'done1'


def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
    return heading_difference


def get_state_machine(callbacks):
    sm_event_1 = smach.StateMachine(outcomes=['DONE1', 'SUCCESS1'])
    with sm_event_1:
        smach.StateMachine.add('ROTATE_LEFT', RotateLeft(callbacks),
                               transitions={'done1': 'DONE1', 'count': 'COUNT'})
        smach.StateMachine.add('COUNT', Count(callbacks),
                               transitions={'done1': 'DONE1', 'rotate_right': 'ROTATE_RIGHT'})
        smach.StateMachine.add('ROTATE_RIGHT', RotateRight(callbacks),
                               transitions={'done1': 'DONE1', 'success1': 'SUCCESS1'})
    return sm_event_1
