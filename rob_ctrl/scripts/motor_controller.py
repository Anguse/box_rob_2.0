import rospy
from ctypes import *
import sys
import wiringpi

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
		wiringpi.wiringPiSPISetup(self.SPI_CHANNEL, self.SPI_SPEED)	
		self.pWhite_Board_RX = TYPE_White_Board_RX()
		self.pWhite_Board_TX = TYPE_White_Board_TX()
		pass

	def get_encoder_values(self):
		pass

	def set_speed(self, speed_M0, speed_M1):
		self.pWhite_Board_TX.Status = 1
		self.pWhite_Board_TX.Set_Speed_M0 = speed_M0
		self.pWhite_Board_TX.Set_Speed_M1 = speed_M1
		self.pWhite_Board_TX.Heart_Beat = 1
		#self.pWhite_Board_TX.Digital_Out = 0x01
		buf = cast(byref(self.pWhite_Board_TX), POINTER(c_char * sizeof(self.pWhite_Board_TX)))
		print(sizeof(self.pWhite_Board_TX))
		retlen, retdata = wiringpi.wiringPiSPIDataRW(self.SPI_CHANNEL, buf.contents.raw)
		print(retlen)
		print(retdata)
		pass
	

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
		buf = cast(byref(self.pWhite_Board_TX), POINTER(c_char * sizeof(self.pWhite_Board_TX)))
		retlen, retdata = wiringpi.wiringPiSPIDataRW(self.SPI_CHANNEL, buf.contents.raw)
				

def main():
	controller = Motorcontroller()
	controller.reset_encoders()
	while(True):
	    controller.set_speed(-30,-30)
	size = sizeof(controller.pWhite_Board_TX)
	print(size)
	return

if __name__ == "__main__":
	main()
