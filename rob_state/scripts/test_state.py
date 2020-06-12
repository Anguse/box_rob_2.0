#!/usr/bin/env python
import rospy
import smach
import smach_ros
import cv2
import time
import numpy as np
import math
#import imutils
import std_msgs
from collections import deque
from std_msgs.msg import String, Int16, Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
#from imutils.video import VideoStream

#Global variables for publishing, used in several states.
g_leftWheel_pub = rospy.Publisher('left_wheel', Int16, queue_size=-1)
g_rightWheel_pub = rospy.Publisher('right_wheel', Int16, queue_size=-1)

g_pose = Pose()
g_box_finder = String()
g_box_eval = String()
g_home = Marker()

centered_box = False
box_place = 0
box_one = False
ir_sens = False

class glob_listen:
    def __init__(self):
        self.box_found_sub = rospy.Subscriber('box_finder', String, self.foundbox_cb)
        self.eval_box_sub = rospy.Subscriber('camera', String, self.evalbox_cb)
        self.ir_sub = rospy.Subscriber('ir', String, self.ir_cb)
        self.pos_sub = rospy.Subscriber('slam_out_pose', Pose, self.pos_cb)
        self.init_sub = rospy.Subscriber('home', Marker, self.home_cb)

    def foundbox_cb(self, data):
        print("This is the data: " + str(data))
        if data.data == 'Centered box':
            centered_box = True
        if data.data == 'Box to the right':
            box_place = 1
        if data.data == 'Box to the left':
            box_place = -1

    def evalbox_cb(self, data):
        #Subscriber of topic that says wheter zero or one
        print(data)
        if data.data == 'one':
            box_one = True

    def ir_cb(self, data):
        print(data)
        if data.data == 'box':
            ir_sens = True

    def pos_cb(self, data):
        #Make data into rob_pos
        rospy.loginf(data.data)
        g_pose = data.data

    def home_cb(self, data):
        g_home = data



class drive_straight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['middle','driving'])

    def execute(self, userdata):

        rospy.loginfo(g_pose.position.x)
        if g_pose.position.x > .05:
            return 'middle'
        else:
            g_leftWheel_pub.publish(30)
            g_rightWheel_pub.publish(30)
           # g_leftWheel_pub.publish(0)
           # g_rightWheel_pub.publish(0)
            return 'driving'
class drive_home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['driving_home', 'home'])

    def travel_home(self):
        #rob_pos = self.rob_pos
        #home_pos = init_pos
        #TODO: Introduce init_pos x,y
        dX = g_home.pose.position.x - g_pose.position.x
        dY = g_home.pose.position.y - g_pose.position.y
        dist = math.sqrt(pow(dX,2)+pow(dY,2))
        direction = math.atan2(dX,dY)


        robot_angle = math.degrees(float(g_pose.orientation.w))
        angle_difference = (robot_angle - math.degrees(float(direction)))

        if abs(angle_difference) > 10.0 and dist > 30 :
            if angle_difference < 0:
                #turn right
                g_leftWheel_pub.publish(-30)
                g_rightWheel_pub.publish(30)
                g_leftWheel_pub.publish(0)
                g_rightWheel_pub.publish(0)
            elif(angle_difference > 0):
                #turn left
                g_leftWheel_pub.publish(30)
                g_rightWheel_pub.publish(-30)
                g_leftWheel_pub.publish(0)
                g_rightWheel_pub.publish(0)
        elif dist > 30:
            #move straight
            g_leftWheel_pub.publish(30)
            g_rightWheel_pub.publish(30)
            g_leftWheel_pub.publish(0)
            g_rightWheel_pub.publish(0)


    def execute(self, userdata):
        if g_pose.position.x == g_home.pose.position.x:
            return 'home'
        else:
            travel_home()
            return 'driving_home'


def main():
    rospy.init_node('robotics_state_machine')

    global_listener = glob_listen()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])
    # Open the container
    with sm:
        #TODO:Might add state and class for when collected two boxes and "script" is done.
        # Add states to the container
        smach.StateMachine.add('DRIVE_STRAIGHT', drive_straight(),
                               transitions={'driving':'DRIVE_STRAIGHT', 'middle':'DRIVE_HOME'})
        smach.StateMachine.add('DRIVE_HOME', drive_home(),
                               transitions={'driving_home':'DRIVE_HOME','home':'done'})


    # Execute SMACH plan
    outcome = sm.execute()
    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    r.sleep()



if __name__ == '__main__':
    main()

