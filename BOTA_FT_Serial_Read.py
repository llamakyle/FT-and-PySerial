import serial
import signal
import time
import csv

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


# Change FT Sensor Data Sending Mode =====================================================================================
def startFT():
	time.sleep(2)
	serFT.write(b'C')
	serFT.write(b'c,0,1,1,4')
	serFT.write(b'R')

# Change FT Sensor Data Sending Mode Back to Original Settings =====================================================================================
def endFT():	
	serFT.write(b'C')
	serFT.write(b'c,0,1,0,4')
	serFT.write(b'R')
	
# Read Bota FT sensor	
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
		try:
			F=readFT()
			Fz=F[2]
			Cal=Cal+Fz
			c=c+1
		except Exception as e:
			print('Read Error from FT sensor message')
			print(e)
	Cal=Cal/c
	return Cal

def readFTCal(CalibrationVal):
	try:
		F=readFT()
		Fz=F[2]-CalibrationVal
	except Exception as e:
			print('Read Error from FT sensor message')
			print(e)
	print(Fz)
	return Fz

def handler(signum,frame):
	endFT()	
	exit(1)

signal.signal(signal.SIGINT, handler)

if __name__ == '__main__':
	startFT()
	CalVal=Calibrate()
	while True:
		try:
			readFTCal(CalVal)
		except:
			continue
