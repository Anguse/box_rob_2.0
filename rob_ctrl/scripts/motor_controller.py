#!/usr/bin/env python
import rospy
from ctypes import *
import sys
import wiringpi
from math import sin, cos, pi, degrees
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int16, Float64MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from visualization_msgs.msg import Marker
import tf
import numpy as np

# dc motor with 19:1 gear

# CONSTANTS
ENCODER = 128
ENCODER_COUNTER = 4
SAMPLE_TIME = 0.01
GEAR_REDUCTION = 15.5/24.0
GEARBOX_RATIO = (24.0/1.0)*(1.0/2.5)
WHEEL_CIRC = 42.0*pi
WHEELBASE_MM = 225.0
PULSES_PER_REVOLUTION = ENCODER*ENCODER_COUNTER*GEAR_REDUCTION
MM_PER_PULSE = WHEEL_CIRC/(PULSES_PER_REVOLUTION*GEARBOX_RATIO)
PULSES_PER_MM = PULSES_PER_REVOLUTION*GEARBOX_RATIO/WHEEL_CIRC
SIGMA_WB = 13.0*4
SIGMA_RW = 1*4
SIGMA_LW = 1*4


class TYPE_White_Board_TX(Structure):
    _fields_ = [('Status', c_ubyte),
                ('Control', c_ubyte),
                ('Current_Loop', c_ushort),
                ('Speed_Loop', c_ushort),
                ('Set_PWM_M0', c_short),
                ('Set_PWM_M1', c_short),
                ('Set_Speed_M0', c_short),
                ('Set_Speed_M1', c_short),
                ('Digital_Out', c_ubyte),
                ('Spare1', c_ubyte),
                ('P_Value_Current', c_float),
                ('P_Value_Speed', c_float),
                ('I_Value_Speed', c_float),
                ('Speed_Stepper', c_short),
                ('Position_Servo', c_ubyte),
                ('Heart_Beat', c_ubyte)]


class TYPE_White_Board_RX(Structure):
    _fields_ = [('Status', c_ubyte),
                ('Digital_In', c_ubyte),
                ('Current_M0', c_ushort),
                ('Current_M1', c_ushort),
                ('Speed_M0', c_short),
                ('Speed_M1', c_short),
                ('Spare0', c_ushort),
                ('Position_M0', c_int),
                ('Position_M1', c_int)]

# class for steering motors


class Motorcontroller():
    def __init__(self):
        # subscribers & publishers
        self.sub_left = rospy.Subscriber('left_wheel', Int16, self.left_cb)
        self.sub_right = rospy.Subscriber('right_wheel', Int16, self.right_cb)
        self.marker = rospy.Publisher('home', Marker, queue_size=1)
        self.pub_uncertainty = rospy.Publisher(
            'odom_uncertainty', numpy_msg(Floats), queue_size=5)

        # publish start position
        home = Marker()
        home.header.frame_id = "base_link"
        home.header.stamp = rospy.Time.now()
        #home.ns = "beizer"
        home.type = home.CUBE
        home.action = home.ADD
        home.id = 0
        home.scale.x = 0.1
        home.scale.y = 0.1
        home.scale.z = 0.1
        home.pose.position.x = 0
        home.pose.position.y = 0
        home.pose.position.z = 0
        home.color.a = 1.0
        home.color.g = 0.5
        home.color.b = 0.5
        self.home = home
        # self.marker.publish(home)

        # spi comm
        self.SPI_CHANNEL = 1
        self.SPI_SPEED = 4000000
        wiringpi.wiringPiSPISetup(self.SPI_CHANNEL, self.SPI_SPEED)
        self.pWhite_Board_RX = TYPE_White_Board_RX()
        self.pWhite_Board_TX = TYPE_White_Board_TX()

        # odom
        self.deltaw = 0  # change since last odom update
        self.delta0 = 0  # right wheel
        self.delta1 = 0
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.C = np.array(([1, 0, 0], [0, 1, 0], [0, 0, 1]))
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time = rospy.Time.now()

        # storage
        self.x_store = []
        self.y_store = []
        self.th_store = []

    def left_cb(self, msg):
        rospy.loginfo(msg.data)
        self.pWhite_Board_TX.Set_Speed_M1 = int(msg.data)
        self.pWhite_Board_TX.Status = 1
        self.pWhite_Board_TX.Heart_Beat = 1
        self.pWhite_Board_TX.Digital_Out = 0x00

    def right_cb(self, msg):
        rospy.loginfo(msg.data)
        self.pWhite_Board_TX.Set_Speed_M0 = int(msg.data)
        self.pWhite_Board_TX.Status = 1
        self.pWhite_Board_TX.Heart_Beat = 1
        self.pWhite_Board_TX.Digital_Out = 0x00

    def update_odom(self):
        current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - self.last_time).to_sec()
        self.vx = ((self.delta0+self.delta1)/2)/dt
        self.vy = 0.0
        self.vth = ((self.delta0-self.delta1)/(WHEELBASE_MM/1000)) / \
            dt  # estimation a bit slow

        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.),
                              Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(
            Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # uncertainty
        Sx = self.C
        Sm = np.array(([SIGMA_RW*abs(self.delta0), 0],
                       [0, SIGMA_LW*abs(self.delta1)]))

        Js = np.array(([1, 0, -sin(self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))*(self.delta1/2 + self.delta0/2)],
                       [0, 1, cos(self.th-(self.delta1 - self.delta0) /
                                  (2*WHEELBASE_MM))*(self.delta1/2 + self.delta0/2)],
                       [0, 0, 1]))
        Jm = np.array(([cos(self.th - (self.delta1 - self.delta0)/(2*WHEELBASE_MM))/2 - (sin(self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))*(self.delta1/2+self.delta0/2))/(2*WHEELBASE_MM), cos(self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))/2 + (sin(self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))*(self.delta1/2+self.delta0/2))/(2*WHEELBASE_MM)],
                       [sin(self.th - (self.delta1 - self.delta0)/(2*WHEELBASE_MM))/2 - (cos(self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))*(self.delta1/2+self.delta0/2))/(2*WHEELBASE_MM), sin(
                           self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))/2 + (cos(self.th-(self.delta1-self.delta0)/(2*WHEELBASE_MM))*(self.delta1/2+self.delta0/2))/(2*WHEELBASE_MM)],
                       [1/WHEELBASE_MM, -1/WHEELBASE_MM]))

        Jb = np.array(([-(sin(self.th - (self.delta1 - self.delta0)/(2*WHEELBASE_MM)) * (self.delta1 - self.delta0)*(self.delta1/2 + self.delta0/2))/(2*pow(WHEELBASE_MM, 2))],
                       [(cos(self.th - (self.delta1 - self.delta0)/(2*WHEELBASE_MM)) * (self.delta1 -
                                                                                        self.delta0)*(self.delta1/2 + self.delta0/2))/(2*pow(WHEELBASE_MM, 2))],
                       [(self.delta1 - self.delta0)/pow(WHEELBASE_MM, 2)]))

        # self.C = (Js*Sx*Js.transpose()) #+ (Jm*Sm*Jm.transpose()) #+ (Jb*SIGMA_WB*Jb.transpose())

        self.x_store.append(self.x)
        self.y_store.append(self.y)
        self.th_store.append(self.th)

        data = np.array([self.x_store, self.y_store, self.th_store])
        self.C = np.cov(data, bias=True)
        odom.pose.covariance[0] = self.C[0, 0]
        odom.pose.covariance[1] = self.C[0, 1]
        odom.pose.covariance[2] = self.C[0, 2]
        odom.pose.covariance[3] = self.C[1, 0]
        odom.pose.covariance[4] = self.C[1, 1]
        odom.pose.covariance[5] = self.C[1, 2]
        odom.pose.covariance[6] = self.C[2, 0]
        odom.pose.covariance[7] = self.C[2, 1]
        odom.pose.covariance[8] = self.C[2, 2]

        # publish the message
        self.odom_pub.publish(odom)
        self.last_time = current_time

        self.deltaw = 0
        self.delta0 = 0
        self.delta1 = 0

    def get_encoder_values(self):
        pass

    def set_speed(self, speed_M1, speed_M0):
        self.pWhite_Board_TX.Status = 1
        self.pWhite_Board_TX.Set_Speed_M0 = speed_M0
        self.pWhite_Board_TX.Set_Speed_M1 = speed_M1
        self.pWhite_Board_TX.Heart_Beat = 1
        self.pWhite_Board_TX.Digital_Out = 0x00

    def reset_encoders(self):
        self.pWhite_Board_TX.Status = 0
        self.pWhite_Board_TX.Control = 0
        self.pWhite_Board_TX.Current_Loop = 0
        self.pWhite_Board_TX.Speed_Loop = 0
        self.pWhite_Board_TX.Set_PWM_M0 = 0
        self.pWhite_Board_TX.Set_PWM_M1 = 0
        self.pWhite_Board_TX.Set_Speed_M0 = 0
        self.pWhite_Board_TX.Set_Speed_M1 = 0
        self.pWhite_Board_TX.Digital_Out = 0
        self.pWhite_Board_TX.Spare1 = 0
        self.pWhite_Board_TX.P_Value_Current = 0.0
        self.pWhite_Board_TX.P_Value_Speed = 0.0
        self.pWhite_Board_TX.I_Value_Speed = 0.0
        self.pWhite_Board_TX.Speed_Stepper = 0
        self.pWhite_Board_TX.Position_Servo = 0
        self.pWhite_Board_TX.Heart_Beat = 0

        self.pWhite_Board_TX.Status = 1
        self.pWhite_Board_TX.Control = 1
        self.pWhite_Board_TX.Set_Speed_M0 = 0
        self.pWhite_Board_TX.Set_Speed_M1 = 0
        self.pWhite_Board_TX.Position_Servo = 0
        self.pWhite_Board_TX.Heart_Beat = 1

        # serialize data and send over SPI
        buf = cast(byref(self.pWhite_Board_TX), POINTER(
            c_char * sizeof(self.pWhite_Board_TX)))
        retlen, retdata = wiringpi.wiringPiSPIDataRW(
            self.SPI_CHANNEL, buf.contents.raw)
        memmove(pointer(self.pWhite_Board_RX),
                retdata, sizeof(self.pWhite_Board_RX))
        print(self.pWhite_Board_RX.Position_M0)

    def send_encoder_values(self):
        pWhite_Board_RX_update = TYPE_White_Board_RX()
        buf = cast(byref(self.pWhite_Board_TX), POINTER(
            c_char * sizeof(self.pWhite_Board_TX)))
        retlen, retdata = wiringpi.wiringPiSPIDataRW(
            self.SPI_CHANNEL, buf.contents.raw)
        memmove(pointer(pWhite_Board_RX_update),
                retdata, sizeof(pWhite_Board_RX_update))
        diff0_mm = (pWhite_Board_RX_update.Position_M0 -
                    self.pWhite_Board_RX.Position_M0)/PULSES_PER_MM
        diff1_mm = (pWhite_Board_RX_update.Position_M1 -
                    self.pWhite_Board_RX.Position_M1)/PULSES_PER_MM
        self.deltaw += (diff0_mm-diff1_mm)/WHEELBASE_MM
        self.delta0 += diff0_mm/1000  # meters
        self.delta1 += diff1_mm/1000
        self.pWhite_Board_RX = pWhite_Board_RX_update


def main():
    rospy.init_node('rob_ctrl')
    controller = Motorcontroller()
    controller.reset_encoders()
    r = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        controller.send_encoder_values()
        controller.update_odom()
        # controller.marker.publish(controller.home)
        r.sleep()
    return


if __name__ == "__main__":
    main()
