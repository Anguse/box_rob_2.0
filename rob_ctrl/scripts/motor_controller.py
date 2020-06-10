import rospy
from ctypes import *
import sys
import wiringpi
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


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
		self.SPI_CHANNEL = 1
		self.SPI_SPEED = 4000000
		wiringpi.wiringPiSPISetup(self.SPI_CHANNEL, self.SPI_SPEED
		self.pWhite_Board_RX=TYPE_White_Board_RX()
		self.pWhite_Board_TX=TYPE_White_Board_TX()

		# odom
		self.odom_pub=rospy.Publisher("odom", Odometry, queue_size=50)
		self.odom_broadcast=tf.TransformBroadcaster()
		self.x=0.0
		self.y=0.0
		self.th=0.0
		self.vx=0.1
		self.vy=-0.1
		self.vth=0.1
		self.last_time=rospy.Time.now()
		pass

	def update_odom(self):
		current_time=rospy.Time.now()
		# compute odometry in a typical way given the velocities of the robot
		dt=(current_time - self.last_time).to_sec()
		delta_x=(self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
		delta_y=(self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
		delta_th=self.vth * dt

		self.x += delta_x
		self.y += delta_y
		self.th += delta_th

		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat=tf.transformations.quaternion_from_euler(0, 0, self.th)

		# first, we'll publish the transform over tf
		self.odom_broadcaster.sendTransform(
			(self.x, self.y, 0.),
			odom_quat,
			current_time,
			"base_link",
			"odom"
		)

		# next, we'll publish the odometry message over ROS
		odom=Odometry()
		odom.header.stamp=current_time
		odom.header.frame_id="odom"

		# set the position
		odom.pose.pose=Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

		# set the velocity
		odom.child_frame_id="base_link"
		odom.twist.twist=Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

		# publish the message
		self.odom_pub.publish(odom)
		self.last_time=current_time

	def get_encoder_values(self):
		pass

	def set_speed(self, speed_M0, speed_M1):
		self.pWhite_Board_TX.Status=1
		self.pWhite_Board_TX.Set_Speed_M0=speed_M0
		self.pWhite_Board_TX.Set_Speed_M1=speed_M1
		self.pWhite_Board_TX.Heart_Beat=1
		self.pWhite_Board_TX.Digital_Out=0x00

	def reset_encoders(self):
		self.pWhite_Board_TX.Status=0
		self.pWhite_Board_TX.Control=0
		self.pWhite_Board_TX.Current_Loop=0
		self.pWhite_Board_TX.Speed_Loop=0
		self.pWhite_Board_TX.Set_PWM_M0=0
		self.pWhite_Board_TX.Set_PWM_M1=0
		self.pWhite_Board_TX.Set_Speed_M0=0
		self.pWhite_Board_TX.Set_Speed_M1=0
		self.pWhite_Board_TX.Digital_Out=0
		self.pWhite_Board_TX.Spare1=0
		self.pWhite_Board_TX.P_Value_Current=0.0
		self.pWhite_Board_TX.P_Value_Speed=0.0
		self.pWhite_Board_TX.I_Value_Speed=0.0
		self.pWhite_Board_TX.Speed_Stepper=0
		self.pWhite_Board_TX.Position_Servo=0
		self.pWhite_Board_TX.Heart_Beat=0

		self.pWhite_Board_TX.Status=1
		self.pWhite_Board_TX.Control=1
		self.pWhite_Board_TX.Set_Speed_M0=0
		self.pWhite_Board_TX.Set_Speed_M1=0
		self.pWhite_Board_TX.Position_Servo=0
		self.pWhite_Board_TX.Heart_Beat=1

		# serialize data and send over SPI
		buf=cast(byref(self.pWhite_Board_TX), POINTER(
		    c_char * sizeof(self.pWhite_Board_TX)))
		retlen, retdata=wiringpi.wiringPiSPIDataRW(
		    self.SPI_CHANNEL, buf.contents.raw)
		memmove(pointer(self.pWhite_Board_RX), retdata, sizeof(self.pWhite_Board_RX))
		print(self.pWhite_Board_RX.Position_M0)

def main():
	controller=Motorcontroller()
	controller.reset_encoders()
	r=rospy.rate(1.0)
	while not rospy.is_shutdown():
		controller.update_odom()
		r.sleep()
	'''
	controller.set_speed(-30, -30)
	while(controller.pWhite_Board_RX.Position_M0 > -10000):
		buf=cast(byref(controller.pWhite_Board_TX), POINTER(
		    c_char * sizeof(controller.pWhite_Board_TX)))
		retlen, retdata=wiringpi.wiringPiSPIDataRW(
		    controller.SPI_CHANNEL, buf.contents.raw)
		memmove(pointer(controller.pWhite_Board_RX),
		        retdata, sizeof(controller.pWhite_Board_RX))
		print(controller.pWhite_Board_RX.Position_M0)
	controller.set_speed(0, -30)
	while(controller.pWhite_Board_RX.Position_M1 > -20000):
		buf=cast(byref(controller.pWhite_Board_TX), POINTER(
		    c_char * sizeof(controller.pWhite_Board_TX)))
		retlen, retdata=wiringpi.wiringPiSPIDataRW(
		    controller.SPI_CHANNEL, buf.contents.raw)
		memmove(pointer(controller.pWhite_Board_RX),
		        retdata, sizeof(controller.pWhite_Board_RX))
		print(controller.pWhite_Board_RX.Position_M1)
	controller.set_speed(-30, -30)
	while(controller.pWhite_Board_RX.Position_M1 > -30000):
		buf=cast(byref(controller.pWhite_Board_TX), POINTER(
		    c_char * sizeof(controller.pWhite_Board_TX)))
		retlen, retdata=wiringpi.wiringPiSPIDataRW(
		    controller.SPI_CHANNEL, buf.contents.raw)
		memmove(pointer(controller.pWhite_Board_RX),
		        retdata, sizeof(controller.pWhite_Board_RX))
		print(controller.pWhite_Board_RX.Position_M0)
	'''
	return

if __name__ == "__main__":
	main()
