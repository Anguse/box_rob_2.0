#!/usr/bin/env python

# A node for logging odometry, scan match and combined position from both modalities using kf
# Format is: Time[s], X[mm], Y[mm], theta[rad], sigma_11, sigma_12, ..., sigma_33

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import std_msgs
import numpy as np


class Logger():
    def __init__(self, t_0):

        self.t_0 = t_0

        # odom
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        self.odom = Odometry()
        self.odom_file = open("odom_log.txt", "w")
        self.odom_sigma = np.zeros((3, 3))
        self.odom_th = 0.0

        # scan
        self.scan_sub = rospy.Subscriber(
            "poseupdate", PoseWithCovarianceStamped, self.scan_cb)
        self.scan = Odometry()
        self.scan_file = open("scan_log.txt", "w")
        self.scan_sigma = np.zeros((3, 3))
        self.scan_th = 0.0

        # kalman
        self.kalman = Odometry()
        self.kalman_file = open("kalman_log.txt", "w")
        self.kalman_sigma = np.zeros((3, 3))
        self.kalman_th = 0.0

        # subscribers
        # calculate new uncertainty in callbacks
        # store position from each modality

    def odom_cb(self, msg):
        self.odom = msg

    def scan_cb(self, msg):
        self.scan.header = msg.header
        self.scan.pose = msg.pose

    def log_odom(self):
        self.odom_file.write("{:.2f} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(self.odom.header.stamp.to_sec(),
                                                                                   self.odom.pose.pose.position.x,
                                                                                   self.odom.pose.pose.position.y,
                                                                                   self.odom_th,
                                                                                   self.odom_sigma[0, 0],
                                                                                   self.odom_sigma[0, 1],
                                                                                   self.odom_sigma[0, 2],
                                                                                   self.odom_sigma[1, 0],
                                                                                   self.odom_sigma[1, 1],
                                                                                   self.odom_sigma[1, 2],
                                                                                   self.odom_sigma[2, 0],
                                                                                   self.odom_sigma[2, 1],
                                                                                   self.odom_sigma[2, 2]))
        self.odom.header.stamp = rospy.Time.now() - self.t_0

    def log_kalman(self):
        self.kalman_file.write("{:.2f} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(self.kalman.header.stamp.to_sec(),
                                                                                     self.kalman.pose.pose.position.x,
                                                                                     self.kalman.pose.pose.position.y,
                                                                                     self.kalman_th,
                                                                                     self.kalman_sigma[0, 0],
                                                                                     self.kalman_sigma[0, 1],
                                                                                     self.kalman_sigma[0, 2],
                                                                                     self.kalman_sigma[1, 0],
                                                                                     self.kalman_sigma[1, 1],
                                                                                     self.kalman_sigma[1, 2],
                                                                                     self.kalman_sigma[2, 0],
                                                                                     self.kalman_sigma[2, 1],
                                                                                     self.kalman_sigma[2, 2]))
        self.kalman.header.stamp = rospy.Time.now() - self.t_0
        return

    def log_scan(self):
        self.scan_file.write("{:.2f} {} {} {} {} {} {} {} {} {} {} {} {}\n".format(self.scan.header.stamp.to_sec(),
                                                                                   self.scan.pose.pose.position.x,
                                                                                   self.scan.pose.pose.position.y,
                                                                                   self.scan_th,
                                                                                   self.scan_sigma[0, 0],
                                                                                   self.scan_sigma[0, 1],
                                                                                   self.scan_sigma[0, 2],
                                                                                   self.scan_sigma[1, 0],
                                                                                   self.scan_sigma[1, 1],
                                                                                   self.scan_sigma[1, 2],
                                                                                   self.scan_sigma[2, 0],
                                                                                   self.scan_sigma[2, 1],
                                                                                   self.scan_sigma[2, 2]))
        self.scan.header.stamp = rospy.Time.now() - self.t_0
        return


def main():
    rospy.init_node("logger")
    r = rospy.Rate(2)
    t_0 = rospy.Time.now()
    logger = Logger(t_0)

    while(not rospy.is_shutdown()):
        logger.log_odom()
        logger.log_scan()
        logger.log_kalman()
        r.sleep()
    return


if __name__ == '__main__':
    main()
