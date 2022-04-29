from __future__ import division
import os
curr_dir = os.path.dirname(os.path.realpath(__file__))
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time
import scipy.io
import csv
import serial

#SG Setup
ser = serial.Serial('/dev/ttyACM0')	
ser.flushInput()
moment=time.strftime("%b-%d__%I:%M",time.localtime())
SGmap=[0,35,64,135,219,273]	
FTmap=[0,6,14,28,46,81]
F=[0,0]
SG1f=[0,0]
SG2f=[0,0]
SG1av=[0,0]
SG2av=[0,0]
c=0

# This file will provide an interface with the BOTA force / torque sensors to convert ROS topics provided by the
# BOTA launch files into python for pySiLVIA


class ROSInterface:

    def __init__(self, sensor_id):
        topic_name_FT = "/rokubimini/" + sensor_id + "/ft_sensor_readings/wrench"

        self.current_FT = WrenchStamped()
        self.sub_FT = rospy.Subscriber(topic_name_FT, WrenchStamped, self.receive_FT)
        self.dt_time_vec = []

        # specify what delta_t we want, where we average the forces (delta_t MUST be bigger than dt specified in BOTA ros launch file)
        self.dt_limit = 0.003

        self.force_mag_vec = []

        self.offset_calibrated = False
        self.offset = np.zeros(6)

        self.FT_meas_log = []

    def receive_FT(self, msg):
        self.current_FT = msg

    def get_current_FT(self):
        #F_T = np.zeros((6,1))
        F_T = np.zeros(6)

        if self.offset_calibrated:
            F_T[0] = self.current_FT.wrench.force.x - self.offset[0]
            F_T[1] = self.current_FT.wrench.force.y - self.offset[1]
            F_T[2] = self.current_FT.wrench.force.z - self.offset[2]
            F_T[3] = self.current_FT.wrench.torque.x - self.offset[3]
            F_T[4] = self.current_FT.wrench.torque.y - self.offset[4]
            F_T[5] = self.current_FT.wrench.torque.z - self.offset[5]
        else:
            F_T[0] = self.current_FT.wrench.force.x
            F_T[1] = self.current_FT.wrench.force.y
            F_T[2] = self.current_FT.wrench.force.z
            F_T[3] = self.current_FT.wrench.torque.x
            F_T[4] = self.current_FT.wrench.torque.y
            F_T[5] = self.current_FT.wrench.torque.z

        return F_T

    def calibrate_offset(self):
        self.offset_calibrated = False
        data_cal = []
        num_pt_cal = 300
        dt_cal = 0.01  # unit:sec
        print("Calibration will take {} seconds".format(num_pt_cal*dt_cal))

        for iterr in range(num_pt_cal):
            measurement = self.get_current_FT()
            data_cal.append(measurement)
            time.sleep(dt_cal)
        data_cal_np = np.array(data_cal)

        self.offset = np.average(data_cal_np, axis=0)
        var_offset = np.var(data_cal_np, axis=0)
        max_offset = np.ndarray.max(data_cal_np, axis=0)
        print("Offsets calibrated to be Fx: {}, Fy: {}, Fz: {}, Tx: {}, Ty: {}, Tz: {}".format(
            self.offset[0], self.offset[1], self.offset[2], self.offset[3], self.offset[4], self.offset[5]))
        print("Variances of offsets are Fx: {}, Fy: {}, Fz: {}, Tx: {}, Ty: {}, Tz: {}".format(
            var_offset[0], var_offset[1], var_offset[2], var_offset[3], var_offset[4], var_offset[5]))
        print("Max offsets are Fx: {}, Fy: {}, Fz: {}, Tx: {}, Ty: {}, Tz: {}".format(
            max_offset[0], max_offset[1], max_offset[2], max_offset[3], max_offset[4], max_offset[5]))
        self.offset_calibrated = True


def measure_force(event_shutdown, time_begin_global):
    time_seq = []
    data_FT = []
    #SG Setup
    ser = serial.Serial('/dev/ttyACM0')	
    ser.flushInput()
    moment=time.strftime("%b-%d__%I:%M",time.localtime())
    #SGmap=[0,35,64,135,219,273]	
    #FTmap=[0,6,14,28,46,81]
    #SGmap=[-3,3.94,9.57,12.17,17.72,40.82,142.9,260]	
    #FTmap=[0.02,0.57,1.01,2.85,7.04,15.01,29.20,48.32]
    #SGmap=[1.69,9.87,13.28,15.33,18.38,35.62,142.9,260]
    #FTmap=[0.01,0.46,0.92,2.53,6.43,13.29,29.20,48.32]
    #SGmap=[-1.69,0.06,5.15,9.92,36.075,47.26,142.9,341.83]
    #FTmap=[0.01,0.53,1.1,2.96,6.69,14.42,29.20,69.32]	
    SGmap=[-1.60,0.06,5.15,9.92,12.63,34.5,133.92,242.4,322.09]
    FTmap=[0.02,0.47,1.1,2.96,7.12,14.9,29.59,45.68,69.75]
    F=[0,0]
    SG1f=[0,0]
    SG2f=[0,0]
    SG1av=[0,0]
    SG2av=[0,0]
    c=0
    FT = ROSInterface("ft_sensor0")
    rospy.init_node('ros_interface')
    measl=0
    count=0
    sgval=0
    calval=0
    cv=0
    sv=0
    print("Beginning Calibration")
    for f in range(200):
        try:
            ser_bytes = ser.readline()	
            try:
                decoded = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
                try:
            	    SG1,SG2,SG3,SG4=decoded.split("\t")	
            	    SG1av[0]+=float(SG1)
            	    SG2av[0]+=float(SG2)
            	    SG1av[1]+=float(SG3)
            	    SG2av[1]+=float(SG4)
            	    c+=1
                except:
            	    continue
            except:
               continue
        except:
    	    print("error opening sg port")        
    	    continue
    	
    	
    SG1av[0]=SG1av[0]/c
    SG2av[0]=SG2av[0]/c
    SG1av[1]=SG1av[1]/c
    SG2av[1]=SG2av[1]/c

    time.sleep(5)  # Wait for ROS node to full initialize, otherwise the first FT sensor's calibration will be off

    FT.calibrate_offset()

    print("Begin to measure force ...")
    print("Begining SG Test")
    while not event_shutdown.is_set():
           
        meas = FT.get_current_FT()
        measure=meas[2]
        try:
                FTchange=abs(measure-measl)        
        except:
                continue                       

        measl=measure
        data_FT.append(meas)



        curr_time = time.time()

        time_seq.append(curr_time - time_begin_global)
 
        try:
                ser_bytes = ser.readline()

                try:
## Reads the data from the arduino's serial port then divides them into the two strain gauges for each finger based on tab delimation 
                    decoded = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")
            #print(decoded)
                    SG1,SG2,SG3,SG4=decoded.split("\t")
                    SG1f[0]=(float(SG1)-SG1av[0])

                    SG2f[0]=(float(SG2)-SG2av[0])

                    SG1f[1]=(float(SG3)-SG1av[1])

                    SG2f[1]=(float(SG4)-SG2av[1])
                    #print(SG1f)
                    #print(SG2f)

                    print(FTchange)	
                    if FTchange<measl*0.01 or FTchange<0.15:

            	        count=count+1                    
            	        print(count)
            	        if count>4:         
	        
            		        calval=calval+measure
            		        sgval=SG1f[1]+sgval

                    elif count>10:
            	        cv=calval/(count-4)
            	        sv=sgval/(count-4)

            	        with open("cal_data_"+moment+".csv","a") as f:
            		        writer = csv.writer(f,delimiter=",")
            		        writer.writerow([sv,cv,count])
            	        count=0
            	        calval=0
            	        sgval=0
                    else:
            	        count=0
            	        calval=0
            	        sgval=0





                
                                
## Linear interpolation based on calibration data to convert from SG values to Force (N) for each finger
                    for k in range(2):
            	        if SG1f[k]<SGmap[1]:	# Determines which bounds to interpolate with
            		        y2=SGmap[1]
            		        y1=SGmap[0]
            		        yf2=FTmap[1]
            		        yf1=FTmap[0]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]<SGmap[2] and SG1f[k]>SGmap[1]:
            		        y2=SGmap[2]
            		        y1=SGmap[1]
            		        yf2=FTmap[2]
            		        yf1=FTmap[1]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]<SGmap[3] and SG1f[k]>SGmap[2]:
            		        y2=SGmap[3]
            		        y1=SGmap[2]
            		        yf2=FTmap[3]
            		        yf1=FTmap[2]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]>SGmap[3] and SG1f[k]<SGmap[4]:
            		        y2=SGmap[4]
            		        y1=SGmap[3]
            		        yf2=FTmap[4]
            		        yf1=FTmap[3]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]>SGmap[4] and SG1f[k]<SGmap[5]:
            		        y2=SGmap[5]
            		        y1=SGmap[4]
            		        yf2=FTmap[5]
            		        yf1=FTmap[4]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]>SGmap[5] and SG1f[k]<SGmap[6]:
            		        y2=SGmap[6]
            		        y1=SGmap[5]
            		        yf2=FTmap[6]
            		        yf1=FTmap[5]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]>SGmap[6] and SG1f[k]<SGmap[7]:
            		        y2=SGmap[7]
            		        y1=SGmap[6]
            		        yf2=FTmap[7]
            		        yf1=FTmap[6]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
            	        elif SG1f[k]>SGmap[7]:
            		        y2=SGmap[8]
            		        y1=SGmap[7]
            		        yf2=FTmap[8]
            		        yf1=FTmap[7]
            		        x=(SG1f[k]-y1)/(y2-y1)
            		        F[k]=yf1+x*(yf2-yf1)
		  
            	        else:
            		        F[k]=0
            		        print("Possible error in interpolating data")
            		  	
            	# x=(SG1f[k]-y1)/(y2-y1) 					# Determines distance between bounds on SG curve
            	# F[k]=yf1+x*(yf2-yf1)						# Applies that distance on the FT curve to approximate force per finger
            	        #print("F:")
            	        #print(F)	
            		
                except:
                    print("Error in sorting data")
                    continue
##  Saves the data to a new , delimited csv file, the name of the file is based on the date and time of the experiment      
                print('SG:' +str(F[1]) + '/t FT:' + str(meas[2]*-1) + '/t Error:' +str(meas[2]*-1-F[1]))
                with open("test_data_"+moment+".csv","a") as f:
                    writer = csv.writer(f,delimiter=",")
                    writer.writerow([time.time()-time_begin_global,SG1f[1],SG2f[1],F[1],meas[2]*-1])
        except:
            print("Keyboard Interrupt")
            break 
        	
    print("Saving the FT sensor readings ...")
    #if not os.path.exits(curr_dir + '/FT_results'):
       # os.makedirs(curr_dir + '/FT_results')
    
    scipy.io.savemat(curr_dir + '/FT_results/FT_readings.mat', mdict={'FT_time_seq': np.array(time_seq),
                                                                      'FT_meas_log': np.array(data_FT)})

    # ROS.__del__()


if __name__ == "__main__":
    measure_force(1)
