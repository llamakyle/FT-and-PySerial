import serial
import multiprocessing
import time
#import FT_sensor.FT_ros_interface as ros_interface
#import FT_sensor.launch_FT_sensor as launch_ft_sensor
#from FT_sensor.events import event_shutdown_FT
ser = serial.Serial('/dev/ttyACM0')	#This will change depending on the port the stm is connected to
ser.flushInput()
#ser2 = serial.Serial('/dev/ttyACM1')	#This will change depending on the port the ft sensor is connected to
#ser2.flushInput()






# Start FT sensor ROS node =====================================================================================
def startFT():
	p_launch = multiprocessing.Process(target=launch_ft_sensor.launch_ft_node, args=(event_shutdown_FT,))
	p_launch.start()
	time.sleep(10)  # Wait until the FT node is fully launched
	p_FT = multiprocessing.Process(target=ros_interface.measure_force, args=(event_shutdown_FT, time_begin_global))
	p_FT.start()
	time.sleep(10)  # Wait until FT sensor calibration is finished
	atexit.register(exit_handler)

# Terminate FT sensor ROS node =====================================================================================
def endFT():	
	event_shutdown_FT.set()
	time.sleep(8)
	print("Shut down FT launch node!!!")
	p_FT.terminate()
	p_launch.terminate()
	
# Read Bota FT sensor and translate into N	
def readFT():
	try:
		ser2.reset_input_buffer()
		ser_bytes2 = ser2.readline()
		
		try:
			meas = float(ser_bytes2[0:len(ser_bytes2)-2].decode("utf-8"))*9.81*.001
		except:
			print('Read Error from FT sensor message')
	except:
		print('Connection Error to FT sensor')
		
# Read 4 tab delimited values from the STM	
def readSTM():
	Curr=0
	Pos=0
	Stif=0
	Etc=0
	ser_bytes = ser.readline()
	decoded = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
	Curr,Pos,Stif,Etc=decoded.split("\t")


	return Curr,Pos,Stif,Etc
	
# Write desired current and position to STM	
def writeSTM(d_Current,d_Position):
	try:
		x=str(d_Current) +','+str(d_Position)+'\r\n'

		ser.write(bytes(x,'utf-8'))

	except :
		print('Write Failed')	
	
	
if __name__ == '__main__':
	while True:
		try:
			Curr,Pos,Stif,Etc=readSTM()
		except:
			continue
		print(Curr,Pos,Stif,Etc)
		dc=1
		dp=2
		writeSTM(dc,dp)
		time.sleep(.01)
			
	
	
