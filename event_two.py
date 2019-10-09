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
        smach.State.__init__(self, outcomes=['done2', 'check'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            print("Rotate left")
        return 'done2'


class Check(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'rotate_right'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            print("count")
        return 'done2'


class RotateRight(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'success2'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            print("Rotate right")
        return 'done2'


def get_state_machine(callbacks):
    sm_event_2 = smach.StateMachine(outcomes=['DONE2', 'SUCCESS2'])
    with sm_event_2:
        smach.StateMachine.add('ROTATE_LEFT', RotateLeft(callbacks),
                               transitions={'done2': 'DONE2', 'check': 'CHECK'})
        smach.StateMachine.add('CHECK', Check(callbacks),
                               transitions={'done2': 'DONE2', 'rotate_right': 'ROTATE_RIGHT'})
        smach.StateMachine.add('ROTATE_RIGHT', RotateRight(callbacks),
                               transitions={'done2': 'DONE2', 'success2': 'SUCCESS2'})
    return sm_event_2
