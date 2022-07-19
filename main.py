from builtins import print

import numpy
import serial
import argparse
import time
from itertools import count
import csv
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np

#Local Imports
from methods import parseSerial, readSTM, writeSTM, sine, startFT,Calibrate, handler, readFTCal,readFT,endFT,save2csv


###plotting variable
from matplotlib.animation import FuncAnimation

plot_window = 200
#Y variable assignment, empty buffer
y_var = np.array(np.zeros([plot_window]))       #Current
y_var2 = np.array(np.zeros([plot_window]))      #Position

plt.ion()
fig, ax = plt.subplots()

#Color choice for plots
line, = ax.plot(y_var,'r')                      #RED current
line2, = ax.plot(y_var2,'g')                    #Green Poistion

##instances
ser = serial.Serial('com3', 115200)             #Open Serial port to STM32 MCU
                                                #MCU Baud rate 115200

serFT = serial.Serial('com6', 460800)           #FT snesor serial port open

serFT.flush()                                   #flush serial port for FT sensor
ser.flush()                                     #Flush Serial mode


#OPen CSV file
f = open(r'C:\Users\fadir\OneDrive\Desktop\RoMeLa\My Projects\mVSA_GRIPPER\mVSA_GRIPPER\Testing\Data\Force Profile TEsts.csv','w')

writer = csv.writer(f)

while 1:
    print('To acces the following modes \n'
          '(1) Idle mode\n'
          '(2) Start-up mode\n'
          '(3) Control mode\n'
          '(4) Terminate\n')

    mode = input("Enter Mode: ")

    if mode == '1':
        ser.flush()
        ser.write("----idle-----".encode())
        time.sleep(0.05)
    if mode == '2':

        ser.flush()
        ser.write("---enstup----".encode())
        time.sleep(0.05)

        while 1:
            print('System Start-up: \n'
                  '(1) Initialize System\n'
                  '(2) Enable Pump\n'
                  '(3) Enable Motor Torque\n'
                  'Press Enter to Go back\n')

            submode = input('Start up mode:')
            if submode == '1':
                ser.flush()
                ser.write("--initsys----".encode())
                print(ser.readline())
                time.sleep(0.5)
            elif submode == '2':
                ser.flush()
                ser.write("---enPmp-----".encode())
                print(ser.readline())
                time.sleep(0.5)
            elif submode == '3':
                ser.flush()
                ser.write("---enMtr-----".encode())
                print(ser.readline())
                time.sleep(0.5)
            elif submode =='':
                break

    if mode == '3':
        ser.flush()
        ser.write("---enCtrl----".encode())


        ####ionitialize FT sesor
        startFT(serFT)  # Start FT sensor
        cal_val = Calibrate(serFT)  # Califbratiobn value for Z-axis

        time.sleep(0.1)
        while 1:
            print('Control Options: \n'
                  '(1) Manual Control\n'
                  '(2) Force/ Displacement Profile test\n'
                  '(3) High Impedence Test\n'
                  '(4) Direct Force Control\n'
                  '(5) Frequency responce/ Bandwidth'
                  'Press Enter to Go back\n')
            submode = input('Control Mode:')
            if submode == '1':                      #Manual Control
                timeStart = time.time()
                i=1
                while 1:
                    format_sin = "{:0.1f}".format(9*numpy.sin(0.01*i))
                    writeSTM(0*format_sin, format_sin)
                    i+=1
                    cur, pos = readSTM()
                    print(pos)

                    Force = readFTCal(serFT,cal_val)

                    y_var = np.append(y_var, pos)
                    y_var2 = np.append(y_var2, Force)
                    y_var = y_var[1:plot_window + 1]
                    y_var2 = y_var2[1:plot_window + 1]
                    line.set_ydata(y_var)
                    line2.set_ydata(y_var2)

                    ax.relim()
                    ax.autoscale_view()
                    fig.canvas.draw()
                    fig.canvas.flush_events()

            elif submode == '2':
                #Current Cycle parameters
                cur_start = -6.5                                #start value of desired current(AMPs)
                cur_end = 6.5                                   #End value of desired current
                cur_step = 1                                    #Current step after each iteration

                time.sleep(0.3)                                 #wait for current Transient

                cur_d = cur_start                               #Current itteration desired current
                while cur_d <= 0.4:#cur_end:                         #Current Loop
                    #Position cycle parameters
                    time_max = 10                               #position loop cycle max time (s)
                    freq = 0.05                                 #frequency of position loop (1/s)
                    amp = 12                                    #Amplitude of sine wave, max defelction of spring in either
                                                                #direction is 12.7, 0.2 accounting for a buffer, to avoid
                                                                #rigid contact
                    cur_d = 0

                    timeStart = time.time()                     #Start time before loop starts
                    timeElapsed = 0                             #Define elapsed time for while loop

                    while timeElapsed < time_max:               #Position Loop

                        timeElapsed = time.time()-timeStart     #update time
                        pos_d = sine(timeElapsed, amp, freq)    #Desired position based on sine wave

                        writeSTM(cur_d, pos_d,ser)              #write desired values to position and current
                        cur, pos = readSTM(ser)                 #read sensor values for position and current


                        Force = readFTCal(serFT, cal_val)

                        writer = csv.writer(f, delimiter=",")
                        data2save = [Force, pos, cur]
                        writer.writerow(data2save)
                        # y_var = np.append(y_var, Force)
                        # y_var2 = np.append(y_var2, pos)
                        # y_var = y_var[1:plot_window + 1]
                        # y_var2 = y_var2[1:plot_window + 1]
                        # line.set_ydata(y_var)
                        # line2.set_ydata(y_var2)
                        #
                        # ax.relim()
                        # ax.autoscale_view()
                        # fig.canvas.draw()
                        # fig.canvas.flush_events()

                    cur_d += cur_step
                break
            elif submode == '3':
                break
            elif submode == '4':
                break
            elif submode == '5':
                break
            elif submode =='':
                break

    if mode == '4':
        ser.flush()
        submode = input('Terminete? (Y/N)')
        if submode == 'Y':
            ser.write("----term-----".encode())
            print(ser.readline())
            time.sleep(0.5)
            print(ser.readline())
            break



ser.close()     #close Serial Connection
f.close()
endFT()


