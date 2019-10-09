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
            print("Rotate left")
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
            print("count")
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
            print("Rotate right")
        return 'done1'


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
