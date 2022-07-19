import serial
import multiprocessing
import time
import numpy as np
import csv
import matplotlib.pyplot as plt

# import FT_sensor.FT_ros_interface as ros_interface
# import FT_sensor.launch_FT_sensor as launch_ft_sensor
# from FT_sensor.events import event_shutdown_FT


##############################################

#parse incoming data from
def parseSerial(data):
    mid = data.find(',')
    current = float(data[0:mid-1])
    position = float(data[mid+1: len(data)-2])
    return [current, position]

#Read from STM32 MCU based on the dilimiters
#Cur,Pos\n
def readSTM(ser):
    Curr = 0
    Pos = 0
    ser_bytes = ser.readline()
    decoded = ser_bytes[0:len(ser_bytes) - 2].decode("utf-8")
    Curr, Pos = decoded.split(",")
    return float(Curr), float(Pos)


# Write desired current and position to STM32 MCU
#Create and send control instructions string based on following format
#<CUR,POS>-- Each packet must be 13 bytes no less no more
def writeSTM(d_Current, d_Position, ser):
    try:
        x = '<' + str(d_Current) + ',' + str(d_Position) + '>'
        while(len(x)<13):
            x = x +'-'
        ser.write(bytes(x, 'utf-8'))
    except:
        print('Write Failed')

#sine function returns value of sine at elasped time with specified
#Frequency f (Hz)
#A for amplitude
def sine(time, amplitude, frequency):
    return "{:0.1f}".format(amplitude*np.sin(2*np.pi*frequency*time))


# Start FT sensor ROS node =====================================================================================
def startFT(serFT):
    time.sleep(2)
    serFT.write(b'C')
    serFT.write(b'c,0,1,1,0')
    serFT.write(b'R')


# Terminate FT sensor ROS node =====================================================================================
def endFT(serFT):
    serFT.write(b'C')
    serFT.write(b'c,0,1,0,4')
    serFT.write(b'R')


# Read Bota FT sensor and translate into N
def readFT(serFT):
    try:
        # serFT.flushInput()
        remove_trash = serFT.read_until(b'\n1\t')
        ser_bytes2 = serFT.read_until(b'\n1\t')

        try:
            meas = ser_bytes2[0:len(ser_bytes2)].decode("utf-8")
            meas = meas.strip('\n1\t')
            meas = meas.replace('\x00', '')
            meas = meas.strip('')

            FT = [float(x) for x in meas.split('\t')]
        # print(FT)
        except Exception as e:
            print(meas)
            print('Read Error from FT sensor message')
            print(e)
    except:
        print('Connection Error to FT sensor')
    return FT


def Calibrate(serFT):
    Cal = 0
    c = 0
    for i in range(0, 99):
        try:
            F = readFT(serFT)
            Fz = F[2]
            Cal = Cal + Fz
            c = c + 1
        except Exception as e:
            print('Read Error from FT sensor message')
            print(e)
    Cal = Cal / c
    return Cal


def readFTCal(serFT,CalibrationVal):
    try:
        F = readFT(serFT)
        Fz = F[2] - CalibrationVal
    except Exception as e:
        print('Read Error from FT sensor message')
        print(e)
    #print(Fz)
    return Fz

# def safe2csv():
#     with open("cal_data_" + moment + ".csv", "a") as f:
#         writer = csv.writer(f, delimiter=",")
#     writer.writerow([sv1, cv])

def handler(signum, frame, serFT):
    endFT(serFT)
    exit(1)

def save2csv(filename,data2save):
	with open(filename,"a") as f:
		writer = csv.writer(f,delimiter=",")
		writer.writerow(data2save)
