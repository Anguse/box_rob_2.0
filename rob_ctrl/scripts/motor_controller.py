import rospy
from ctypes import *
import sys

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
		self.pWhite_Board_RX = TYPE_White_Board_RX()
		self.pWhite_Board_TX = TYPE_White_Board_TX()
		pass

	def get_encoder_values(self):
		pass

	def set_speed(self, speed):
		pass

def main():
	controller = Motorcontroller()
	size = sys.getsizeof(controller.pWhite_Board_TX)
	print(size)
	return

if __name__ == "__main__":
	main()
