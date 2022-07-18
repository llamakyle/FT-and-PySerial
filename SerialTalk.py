import serial
import signal
import time
import csv
#import FT_sensor.FT_ros_interface as ros_interface
#import FT_sensor.launch_FT_sensor as launch_ft_sensor
#from FT_sensor.events import event_shutdown_FT
try:
	ser = serial.Serial('/dev/ttyACM0')	#This will change depending on the port the stm is connected to
	ser.flushInput()
except:
	print('No STM Connected')
try:
	serFT = serial.Serial(port='/dev/ttyUSB0',
							baudrate=460800,
							timeout=1,
							parity=serial.PARITY_NONE,
    						stopbits=serial.STOPBITS_ONE,
    						bytesize=serial.EIGHTBITS)
	serFT.flushInput()
except:
	print('No FT Connected')
CalVal=0


# Start FT sensor ROS node =====================================================================================
def startFT():
	time.sleep(2)
	serFT.write(b'C')
	serFT.write(b'c,0,1,1,0')
	serFT.write(b'R')

# Terminate FT sensor ROS node =====================================================================================
def endFT():	
	serFT.write(b'C')
	serFT.write(b'c,0,1,0,4')
	serFT.write(b'R')
	
# Read Bota FT sensor and translate into N	
def readFT():
	try:
		#serFT.flushInput()
		remove_trash = serFT.read_until(b'\n1\t')
		ser_bytes2 = serFT.read_until(b'\n1\t')

		try:
			meas = ser_bytes2[0:len(ser_bytes2)].decode("utf-8")
			meas=meas.strip('\n1\t')
			meas=meas.replace('\x00','')
			meas=meas.strip('')
			FT=[float(x) for x in meas.split('\t')]
			#print(FT)
		except Exception as e:
			print(meas)
			print('Read Error from FT sensor message')
			print(e)
	except:
		print('Connection Error to FT sensor')
	return FT

def Calibrate():
	Cal=0
	for i in range(0,99):
		F=readFT()
		Fz=F[2]
		Cal=Cal+Fz
	Cal=Cal/100
	return Cal

def readFTCal(CalibrationVal):
	F=readFT()
	Fz=F[2]-CalibrationVal
	print(Fz)
	return Fz

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

def safe2csv():
	with open("cal_data_"+moment+".csv","a") as f:
	writer = csv.writer(f,delimiter=",")
	writer.writerow([sv1,cv])

	
# Write desired current and position to STM	
def writeSTM(d_Current,d_Position):
	try:
		x=str(d_Current) +','+str(d_Position)+'\r\n'

		ser.write(bytes(x,'utf-8'))

	except :
		print('Write Failed')	
	
def handler(signum,frame):
	endFT()	
	exit(1)

signal.signal(signal.SIGINT, handler)

if __name__ == '__main__':
	startFT()
	CalVal=Calibrate()
	while True:
		readFTCal(CalVal)
		#time.sleep(1)
					# STM READ/WRITE
		# try:
		# 	Curr,Pos,Stif,Etc=readSTM()
		# except:
		# 	continue
		# print(Curr,Pos,Stif,Etc)
		# dc=1
		# dp=2
		# writeSTM(dc,dp)
		# time.sleep(.01)
			
	
	
