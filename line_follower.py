#!/usr/bin/env python

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
from tf.transformations import euler_from_quaternion

import cv2
import cv_bridge
import numpy
import time
import event_one
import event_two
import event_three
import detect_shape
from nav_msgs.msg import Odometry

from kobuki_msgs.msg import Sound


global shutdown_requested
global red_count


class Stop(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['follow_line', 'done', 'event_one', 'event_two', 'event_three'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.prev_error = None
        self.Kp = 1.0 / 50.0
        self.Ki = 1.0 / 50.0
        self.Kd = 1.0 / 50.0
        self.speed = 0.8

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        global red_count

        red_count = (red_count + 1) % 6

        red_events = [1, 3]
        red_stops = [0, 2, 4, 5]
        if red_count in red_stops:
            distance = 0.1
        else:
            distance = 0.35

        while self.callbacks.pose is None:
            time.sleep(1)

        sp = self.callbacks.pose
        ep = sp

        while math.sqrt((sp.x - ep.x) ** 2 + (sp.y - ep.y) ** 2) < distance:
            if shutdown_requested:
                return 'done'
            h = self.callbacks.h
            w = self.callbacks.w
            search_top = 3 * h / 4
            search_bot = h
            bottom_white_mask = self.callbacks.white_mask.copy()
            bottom_white_mask[0:search_top, 0:w] = 0
            bottom_white_mask[search_bot:h, 0:w] = 0

            M = cv2.moments(bottom_white_mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # BEGIN CONTROL
                if self.prev_error is None:
                    error = cx - self.callbacks.w / 2
                    rotation = -(self.Kp * float(error))
                    self.prev_error = error
                else:
                    error = cx - self.callbacks.w / 2
                    rotation = -(self.Kp * float(error) + self.Kd * (error - self.prev_error))
                    self.prev_error = error
                self.twist.linear.x = self.speed
                self.twist.angular.z = rotation
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
                ep = self.callbacks.pose

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

        if red_count in red_events:
            if red_count == 1:
                #return 'follow_line'
                return 'event_one'
            elif red_count == 3:
                #return 'follow_line'
                return 'event_two'

        start = time.time()
        while time.time() - start < 5:
            if shutdown_requested:
                return 'done'

        if red_count == 5:
            #return 'follow_line'
            return 'event_three'

        return 'follow_line'


class FollowLine(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['stop', 'done'])
        self.callbacks = callbacks
        self.prev_error = None
        self.Kp = 1.0 / 50.0
        self.Ki = 1.0 / 50.0
        self.Kd = 1.0 / 50.0
        self.speed = 0.8
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global button_start
        global shutdown_requested
        while not shutdown_requested:
            if self.callbacks.white_mask is not None and self.callbacks.red_mask is not None:

                bottom_white_mask = self.callbacks.white_mask.copy()
                bottom_red_mask = self.callbacks.red_mask.copy()

                # Check if a long red strip has been detected
                h = self.callbacks.h
                w = self.callbacks.w
                search_top = 3 * h / 4
                search_bot = h
                bottom_white_mask[0:search_top, 0:w] = 0
                bottom_white_mask[search_bot:h, 0:w] = 0
                bottom_red_mask[0:search_top, 0:w] = 0
                bottom_red_mask[search_bot:h, 0:w] = 0
                red_pixel_count = cv2.sumElems(bottom_red_mask)[0] / 255
                white_pixel_count = cv2.sumElems(bottom_white_mask)[0] / 255

                # Check if a half red strip, on the left, has been detected
                left_red_mask = bottom_red_mask.copy()
                right_red_mask = bottom_red_mask.copy()
                left_red_mask[0:h, w / 2: w] = 0
                right_red_mask[0:h, 0: w / 2] = 0
                left_red_pixel_count = cv2.sumElems(left_red_mask)[0] / 255
                right_red_pixel_count = cv2.sumElems(right_red_mask)[0] / 255
                # cv2.imshow("left window", left_red_mask)
                # cv2.imshow("right window", right_red_mask)
                # print(red_pixel_count)
                #print(left_red_pixel_count - right_red_pixel_count)

                RM = cv2.moments(bottom_red_mask)
                if RM['m00'] > 0:
                    ry = int(RM['m01'] / RM['m00'])
                    # print(" RedY: " + str(ry) + " Red Pixel: " + str(red_pixel_count))  # ----------

                    #if red_pixel_count > 3000 and ry > 430:
                    #    print(red_pixel_count)
                    #    print(ry)
                    #    print("Full red found")
                    #    return 'stop'

                    if red_pixel_count > 1000 and ry > 430:
                        print(red_pixel_count)
                        print(ry)
                        print("Right red found")
                        return 'stop'


                # If there is no significant red line, follow white line
                WM = cv2.moments(bottom_white_mask)

                if WM['m00'] > 0:
                    cx = int(WM['m10'] / WM['m00'])
                    cy = int(WM['m01'] / WM['m00'])

                    # BEGIN CONTROL
                    if self.prev_error is None:
                        error = cx - self.callbacks.w / 2
                        rotation = -(self.Kp * float(error))
                        self.prev_error = error
                    else:
                        error = cx - self.callbacks.w / 2
                        rotation = -(self.Kp * float(error) + self.Kd * (error - self.prev_error))
                        self.prev_error = error
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = rotation
                    self.cmd_vel_pub.publish(self.twist)
                    # END CONTROL
        return 'done'


def request_shutdown(sig, frame):
    global shutdown_requested
    event_one.shutdown_requested = True
    event_two.shutdown_requested = True
    event_three.shutdown_requested = True
    shutdown_requested = True


class Callbacks:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.Kp = 1.0/200.0
        self.Ki = 1.0/200.0
        self.Kd = 1.0/200.0
        self.prev_error = None
        self.past_error = []

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)

        self.twist = Twist()

        self.red_mask = None
        self.white_mask = None
        self.symbol_red_mask = None
        self.symbol_green_mask = None

        self.h = None
        self.w = None
        self.d = None

        self.pose = None

        self.heading = None

        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def odometry_callback(self, msg):
        self.pose = msg.pose.pose.position
        #print(self.pose)
        yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])[2]
        self.heading = (yaw + math.pi) * (180 / math.pi)
        return

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #upper_white = numpy.array([360, 20, 255])
        #lower_white = numpy.array([0,  0,  240])

        #upper_white = numpy.array([360, 25, 255])
        #lower_white = numpy.array([0, 0, 200])

        self.h, self.w, self.d = image.shape

        upper_white = numpy.array([360, 30, 255])
        lower_white = numpy.array([0, 0, 230])
        self.white_mask = cv2.inRange(hsv, lower_white, upper_white)

        upper_red_a = numpy.array([20, 255, 255])
        lower_red_a = numpy.array([0, 100, 100])
        red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

        upper_red_b = numpy.array([255, 255, 255])
        lower_red_b = numpy.array([150, 100, 100])
        red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)
        self.red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

        upper_red_a = numpy.array([20, 255, 255])
        lower_red_a = numpy.array([0, 200, 60])
        red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

        upper_red_b = numpy.array([255, 255, 255])
        lower_red_b = numpy.array([150, 200, 60])
        red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)

        self.symbol_red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

        upper_green = numpy.array([136, 255, 255])
        lower_green = numpy.array([56, 43, 90])
        self.symbol_green_mask = cv2.inRange(hsv, lower_green, upper_green)

        bottom_white_mask = self.white_mask.copy()
        bottom_red_mask = self.red_mask.copy()
        h = self.h
        w = self.w
        search_top = 3 * h / 4
        search_bot = h
        bottom_white_mask[0:search_top, 0:w] = 0
        bottom_white_mask[search_bot:h, 0:w] = 0
        bottom_red_mask[0:search_top, 0:w] = 0
        bottom_red_mask[search_bot:h, 0:w] = 0
        red_pixel_count = cv2.sumElems(bottom_red_mask)[0] / 255
        white_pixel_count = cv2.sumElems(bottom_white_mask)[0] / 255

        # self.white_mask = white_mask
        # self.red_mask = red_mask

        symbol_red_mask = self.symbol_red_mask.copy()
        symbol_red_mask[0:h/4, 0:w] = 0
        symbol_red_mask[3*h/4:h, 0:w] = 0

        symbol_green_mask = self.symbol_green_mask.copy()
        #symbol_green_mask[0:h / 4, 0:w] = 0
        #symbol_green_mask[0:h, 0:w / 4] = 0
        #symbol_green_mask[0:h, 3 * w / 4:w] = 0
        #symbol_green_mask[3 * h / 4:h, 0:w] = 0

        symbol_green_mask[0:h / 4, 0:w] = 0
        symbol_green_mask[3 * h / 4:h, 0:w] = 0

        shapes = detect_shape.detect_shape(symbol_green_mask)[0]
        #print(shapes)

        # print(cv2.sumElems(red_mask)[0] / 255)
        cv2.imshow("green window", symbol_green_mask)
        cv2.imshow("white window", self.white_mask)
        cv2.waitKey(3)


def main():
    global button_start
    global shutdown_requested
    global red_count

    event_two.previous_shape = 4

    #red_count = 0
    red_count = 2
    # red_count = 4

    button_start = False

    shutdown_requested = False
    event_one.shutdown_requested = False
    event_two.shutdown_requested = False
    event_three.shutdown_requested = False

    rospy.init_node('line_follow_bot')

    callbacks = Callbacks()
    #rospy.Subscriber('camera/rgb/image_raw', Image, callbacks.image_callback)
    rospy.Subscriber('/camera/image_raw', Image, callbacks.image_callback)
    rospy.Subscriber("odom", Odometry, callbacks.odometry_callback)

    # Create done outcome which will stop the state machine
    sm_turtle = smach.StateMachine(outcomes=['DONE'])

    with sm_turtle:
        smach.StateMachine.add('FOLLOW_LINE', FollowLine(callbacks),
                               transitions={'stop': 'STOP', 'done': 'DONE'})
        smach.StateMachine.add('STOP', Stop(callbacks),
                               transitions={'follow_line': 'FOLLOW_LINE', 'done': 'DONE',
                                            'event_one': 'EVENT_ONE',
                                            'event_two': 'EVENT_TWO',
                                            'event_three': 'EVENT_THREE'})

        sm_event_1 = event_one.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_ONE', sm_event_1,
                               transitions={'DONE1': 'DONE', 'SUCCESS1': 'FOLLOW_LINE'})

        sm_event_2 = event_two.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_TWO', sm_event_2,
                               transitions={'DONE2': 'DONE', 'SUCCESS2': 'FOLLOW_LINE'})

        sm_event_3 = event_three.get_state_machine(callbacks)
        smach.StateMachine.add('EVENT_THREE', sm_event_3,
                               transitions={'DONE3': 'DONE', 'SUCCESS3': 'FOLLOW_LINE'})



    # Create and start the instrospection server - needed for smach_viewer
    sis = smach_ros.IntrospectionServer('TRAVELLER_server', sm_turtle, 'STATEMACHINE')
    sis.start()

    # Start state machine and run until SIGINT received
    signal.signal(signal.SIGINT, request_shutdown)
    sm_turtle.execute()

    # Stop server
    sis.stop()


if __name__ == '__main__':
    main()
