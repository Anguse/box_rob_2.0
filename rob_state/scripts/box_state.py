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
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from imutils.video import VideoStream

#Global variables for publishing, used in several states.
g_leftWheel_pub = rospy.Publisher('left_wheel', Int16, queue_size=1)
g_rightWheel_pub = rospy.Publisher('right_wheel', Int16, queue_size=1)

g_pose = Pose()
g_box_finder = String()
g_box_eval = String()
g_home = Marker()

collected = False
centered_box = False
box_place = 0
box_one = False
turned_left = False
turned_right = False

class glob_listen:
    def __init__(self):
        self.box_found_sub = rospy.Subscriber('box_location', String, self.foundbox_cb)
        self.eval_box_sub = rospy.Subscriber('box_digit', String, self.evalbox_cb)
        self.pos_sub = rospy.Subscriber('slam_out_pose', PoseStamped, self.pos_cb)
        #self.init_sub = rospy.Subscriber('home', Marker, self.home_cb)
        self.collected_sub = rospy.Subscriber('box_collected',String, self.collected_cb)

    def foundbox_cb(self, data):
        global centered_box
        global box_place
        if data.data == 'Centered box':
            centered_box = True
        elif data.data == 'Box to the right':
            centered_box = False
            box_place = 1
        elif data.data == 'Box to the left':
            box_place = -1
            centered_box = False
        else:
            box_place = 0
            centered_box = False

    def evalbox_cb(self, data):
        #Subscriber of topic that says wheter zero or one
        global box_one
        if data.data == 'one':
            box_one = True

    def collected_cb(self, data):
        global collected
        if data.data == 'collected':
            collected = True
        else:
            collected = False

    def pos_cb(self, data):
        global g_pose
        g_pose = data.pose
        #rospy.loginfo(g_pose)
'''
    def home_cb(self, data):
        global g_home
        g_home = data
'''


class drive_straight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['middle','driving'])

    def execute(self, userdata):
        global turned_right
        global turned_left

        r = rospy.Rate(10)
        r.sleep()
        if g_pose.position.x > 1:
            return 'middle'
        else:
            if turned_right:
                g_leftWheel_pub.publish(60)
                g_rightWheel_pub.publish(80)
                rospy.sleep(1)
                turned_right = False
            elif turned_left:
                g_leftWheel_pub.publish(80)
                g_rightWheel_pub.publish(60)
                rospy.sleep(1)
                turned_left = False
            else:
                g_leftWheel_pub.publish(70)
                g_rightWheel_pub.publish(70)
            return 'driving'


class drive_home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['driving_home', 'home'])
        self.dist = 100

    def travel_home(self):
        global g_pose
        global turned_right
        global turned_left
        r = rospy.Rate(40)
        r.sleep()

        orientation_q = g_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        start_x = 0
        start_y = 0

        dX = start_x - g_pose.position.x
        dY = start_y - g_pose.position.y
        self.dist = math.sqrt(pow(dX,2)+pow(dY,2))
        direction = math.degrees(math.atan2(dY,dX)) # swapped dx, dy


        robot_angle = math.degrees(float(yaw))
        rospy.loginfo("yaw:%s"%str(robot_angle)) # radians [pi, -pi]
        rospy.loginfo("direction:%s"%str(direction))
        angle_difference = (float(direction)-robot_angle+180)%(360)-180
        rospy.loginfo("angle_diff:%s"%str(angle_difference))
        #rospy.loginfo(self.dist)

        if abs(angle_difference) > 20.0 and self.dist > 0.08:
            if angle_difference < 0:
                #turn right
                turned_right = True
                turned_left = False
                rospy.loginfo("right")
                g_leftWheel_pub.publish(20)
                g_rightWheel_pub.publish(-20)
            elif angle_difference > 0:
                #turn left
                turned_right = False
                turned_left = True
                rospy.loginfo("left")
                g_leftWheel_pub.publish(-20)
                g_rightWheel_pub.publish(20)
        elif self.dist > 0.08:
            if turned_left:
                turned_left = False
                g_leftWheel_pub.publish(60)
                g_rightWheel_pub.publish(40)
                rospy.sleep(1)

            elif turned_right:
                turned_right = False
                g_leftWheel_pub.publish(40)
                g_rightWheel_pub.publish(60)
                rospy.sleep(1)
            else:
                g_leftWheel_pub.publish(40)
                g_rightWheel_pub.publish(40)
        else:
            rospy.loginfo("done")


    def execute(self, userdata):
        global g_pose
        start_x = 0
        start_y = 0

        dX = start_x - g_pose.position.x
        dY = start_y - g_pose.position.y
        self.dist = math.sqrt(pow(dX,2)+pow(dY,2))

        if self.dist <= 0.08:
            g_leftWheel_pub.publish(0)
            g_rightWheel_pub.publish(0)
            return 'home'
        else:
            self.travel_home()
            return 'driving_home'

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered_box','no_box','box_right','box_left'])

    #  The four possible outcomes from frame is , box centered, no box, to the right or to the left.
    # This will produce different transitions, which eventually will continue to Evalbox state.


    def execute(self, userdata):
        rospy.loginfo('Executing state Search')
        #Is there a box in frame, if so, centered left or right?
        #g_leftWheel_pub.publish(0)
        #g_rightWheel_pub.publish(0)
        if centered_box:
            g_leftWheel_pub.publish(0)
            g_rightWheel_pub.publish(0)
            rospy.sleep(1)
            return 'centered_box'
        elif box_place == 1:
            return 'box_right'
        elif box_place == -1:
            return 'box_left'
        else:
            return 'no_box'

class Evalbox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['zero','one'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Evalbox')
        #is the box of the type zero or one?
        if box_one:
            return 'one'
        else:
            return 'zero'

class Collect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['collected','collecting','transition','lost_box'])
    #Drive straight
    def go_to_box(self):
        global turned_right
        global turned_left
        # if last turned, compensate
        if turned_left:
            g_leftWheel_pub.publish(80)
            g_rightWheel_pub.publish(40)
            turned_left = False
            rospy.sleep(.5)
        elif turned_right:
            g_rightWheel_pub.publish(80)
            g_leftWheel_pub.publish(40)
            turned_right = False
            rospy.sleep(.5)
        else:
            g_leftWheel_pub.publish(30)
            g_rightWheel_pub.publish(30)
            rospy.sleep(0.1)

    def execute(self, userdata):
        #Publish to vel for motors, drive forward until ir sensor detects box
        #Can the sensor detect a box
        g_leftWheel_pub.publish(0)
        g_rightWheel_pub.publish(0)
        if collected:
            g_leftWheel_pub.publish(40)
            g_rightWheel_pub.publish(40)
            rospy.sleep(3.0)
            return 'collected'
        elif centered_box:
            self.go_to_box()
            return 'collecting'
        else:
            return 'lost_box'



class Drop_Box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropped'])

    def execute(self, userdata):
        #Check if no contour is taking up "majority" of frame, can assume that box is dropped.
        global g_pose
        global collected
        g_leftWheel_pub.publish(-40)
        g_rightWheel_pub.publish(-40)
        rospy.sleep(3)
        collected = False
        orientation_q = g_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        while math.degrees(abs(yaw)) > 60:
            rospy.loginfo(math.degrees(yaw))
            # turn right
            g_leftWheel_pub.publish(10)
            g_rightWheel_pub.publish(-10)
            rospy.sleep(.2)
            orientation_q = g_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return 'dropped'




class Turn_Right(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turned'])

    def execute(self, userdata):
         #turn right
        global turned_right
        global turned_left
        turned_right = True
        turned_left = False
        g_leftWheel_pub.publish(20)
        g_rightWheel_pub.publish(-20)
        rospy.sleep(0.3)
        g_leftWheel_pub.publish(0)
        g_rightWheel_pub.publish(0)
        rospy.sleep(1.5)
        return 'turned'



class Turn_Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turned'])

    def execute(self, userdata):
         #turn left
        global turned_right
        global turned_left
        turned_right = False
        turned_left = True
        g_leftWheel_pub.publish(-20)
        g_rightWheel_pub.publish(20)
        rospy.sleep(0.3)
        g_leftWheel_pub.publish(0)
        g_rightWheel_pub.publish(0)
        rospy.sleep(1.5)
        return 'turned'



def main():
    rospy.init_node('robotics_state_machine')

    global_listener = glob_listen()

    #r = rospy.Rate(20)
    #while not rospy.is_shutdown():
    #    r.sleep()
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['DONE'])
    # Open the container
    with sm:
        #TODO:Might add state and class for when collected two boxes and "script" is done.


        #As this should work atm the robot should start with drive_straint to search for a box.
        # when box is found in search turn til centered
        # when centered evaluation of box
        # will close if zero, if one next is collect phase
        # will drive straight until "majority" of camera frame is blue box, if zero appears during collection then also done
        # when collected drive_home until in home_position.

        #This can then be further improved by ignoring zeros rather than "completing", dropping box and searching for a new one, repeat process.

        # Add states to the container
        smach.StateMachine.add('DRIVE_STRAIGHT', drive_straight(),
                               transitions={'driving':'DRIVE_STRAIGHT', 'middle':'SEARCH'})
        smach.StateMachine.add('DRIVE_HOME', drive_home(),
                               transitions={'driving_home':'DRIVE_HOME','home':'DROP_BOX'})
        smach.StateMachine.add('DROP_BOX', Drop_Box(),
                               transitions={'dropped':'DRIVE_STRAIGHT'})
        smach.StateMachine.add('SEARCH', Search(),
                               transitions={'centered_box':'EVALBOX', 'no_box':'TURN_RIGHT','box_right':'TURN_RIGHT','box_left':'TURN_LEFT'})
        smach.StateMachine.add('EVALBOX', Evalbox(),
                               transitions={'zero':'COLLECT','one':'COLLECT'})
        smach.StateMachine.add('TURN_RIGHT', Turn_Right(),
                               transitions={'turned':'SEARCH'})
        smach.StateMachine.add('TURN_LEFT', Turn_Left(),
                               transitions={'turned':'SEARCH'})
        smach.StateMachine.add('COLLECT', Collect(),
                               transitions={'collected':'DRIVE_HOME','collecting':'COLLECT','transition':'DONE','lost_box':'SEARCH'})

    # Execute SMACH plan
    outcome = sm.execute()
    #r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    r.sleep()



if __name__ == '__main__':
    main()

