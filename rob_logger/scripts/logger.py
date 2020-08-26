#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from rospy_tutorials.msg import Floats
import std_msgs
import numpy as np


class Logger():
    def __init__(self, t_0):

        self.t_0 = t_0

        # odom
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_cb)
        #self.odom_unc_sub = rospy.Subscriber("odom_uncertainty", Floats, self.odom_unc_cb)
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

    def odom_unc_cb(self, msg):
        self.odom_sigma = msg.data
        # rospy.loginfo(msg.data)

    def odom_cb(self, msg):
        self.odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.odom_th = yaw
        self.odom_sigma[0, 0] = msg.pose.covariance[0]
        self.odom_sigma[0, 1] = msg.pose.covariance[1]
        self.odom_sigma[0, 2] = msg.pose.covariance[2]
        self.odom_sigma[1, 0] = msg.pose.covariance[3]
        self.odom_sigma[1, 1] = msg.pose.covariance[4]
        self.odom_sigma[1, 2] = msg.pose.covariance[5]
        self.odom_sigma[2, 0] = msg.pose.covariance[6]
        self.odom_sigma[2, 1] = msg.pose.covariance[7]
        self.odom_sigma[2, 2] = msg.pose.covariance[8]

    def scan_cb(self, msg):
        self.scan.header = msg.header
        self.scan.pose = msg.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.scan_th = yaw
        # uncertainty
        self.scan_sigma[0, 0] = msg.pose.covariance[0]
        self.scan_sigma[0, 1] = msg.pose.covariance[1]
        self.scan_sigma[0, 2] = msg.pose.covariance[5]
        self.scan_sigma[1, 0] = msg.pose.covariance[6]
        self.scan_sigma[1, 1] = msg.pose.covariance[7]
        self.scan_sigma[1, 2] = msg.pose.covariance[11]
        self.scan_sigma[2, 0] = msg.pose.covariance[30]
        self.scan_sigma[2, 1] = msg.pose.covariance[31]
        self.scan_sigma[2, 2] = msg.pose.covariance[35]
        self.scan_sigma = np.linalg.pinv(self.scan_sigma)

    def log_odom(self):
        self.odom_file.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e}\n".format(self.odom.header.stamp.to_sec() - self.t_0.to_sec(),
                                                                                                                                   self.odom.pose.pose.position.x,
                                                                                                                                   self.odom.pose.pose.position.y,
                                                                                                                                   self.odom_th,
                                                                                                                                   self.odom_sigma[
                                                                                                                                       0, 0],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       0, 1],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       0, 2],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       1, 0],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       1, 1],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       1, 2],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       2, 0],
                                                                                                                                   self.odom_sigma[
                                                                                                                                       2, 1],
                                                                                                                                   self.odom_sigma[2, 2]))

    def log_kalman(self):
        self.kalman_file.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e}\n".format(self.kalman.header.stamp.to_sec() - self.t_0.to_sec(),
                                                                                                                                     self.kalman.pose.pose.position.x,
                                                                                                                                     self.kalman.pose.pose.position.y,
                                                                                                                                     self.kalman_th,
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         0, 0],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         0, 1],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         0, 2],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         1, 0],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         1, 1],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         1, 2],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         2, 0],
                                                                                                                                     self.kalman_sigma[
                                                                                                                                         2, 1],
                                                                                                                                     self.kalman_sigma[2, 2]))
        return

    def log_scan(self):
        self.scan_file.write("{:.2f} {:.2f} {:.2f} {:.2f} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e} {:.2e}\n".format(self.scan.header.stamp.to_sec() - self.t_0.to_sec(),
                                                                                                                                   self.scan.pose.pose.position.x,
                                                                                                                                   self.scan.pose.pose.position.y,
                                                                                                                                   self.scan_th,
                                                                                                                                   self.scan_sigma[
                                                                                                                                       0, 0],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       0, 1],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       0, 2],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       1, 0],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       1, 1],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       1, 2],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       2, 0],
                                                                                                                                   self.scan_sigma[
                                                                                                                                       2, 1],
                                                                                                                                   self.scan_sigma[2, 2]))
        return

    def kalman_update(self):

        lidar_var = self.scan_sigma
        odom_var = self.odom_sigma
        lidar_pos = np.array(
            [self.scan.pose.pose.position.x, self.scan.pose.pose.position.y, self.scan_th])
        odom_pos = np.array(
            [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom_th])

        pseudo_inverse = np.linalg.pinv(lidar_var+odom_var)

        inv_lidar_var = np.linalg.pinv(lidar_var)
        inv_odom_var = np.linalg.pinv(odom_var)

        new_pos = ((lidar_var.dot(pseudo_inverse).dot(odom_pos)) +
                   (odom_var.dot(pseudo_inverse).dot(lidar_pos)))

        updated_var = np.linalg.pinv(inv_lidar_var + inv_odom_var)
        # vandans
        #pos_cov_kalman = pinv()
        self.kalman.pose.pose.position.x = new_pos[0]
        self.kalman.pose.pose.position.y = new_pos[1]
        self.kalman.header.stamp = rospy.Time.now()
        #self.kalman_th = new_pos[0]
        self.kalman_sigma = updated_var


def main():
    rospy.init_node("logger")
    r = rospy.Rate(1)
    t_0 = rospy.Time.now()
    logger = Logger(t_0)

    while(not rospy.is_shutdown()):
        logger.kalman_update()
        logger.log_odom()
        logger.log_scan()
        logger.log_kalman()
        r.sleep()
    logger.kalman_file.close()
    logger.odom_file.close()
    logger.scan_file.close()
    return


if __name__ == '__main__':
    main()
