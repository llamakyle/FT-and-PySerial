from __future__ import division
import rospy
from geometry_msgs.msg import WrenchStamped
import numpy as np
import scipy.io
import serial
import multiprocessing
import time
import FT_sensor.launch_FT_sensor as launch_ft_sensor
from FT_sensor.events import event_shutdown_FT

#ser = serial.Serial('/dev/ttyACM0')	#This will change depending on the port the stm is connected to
#ser.flushInput()


#----------------------------------FT Sensor-------------------------------------
# Ros Node for BOTA sensor
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




# Start FT sensor ROS node 
def startFT():
	global FT
	p_launch = multiprocessing.Process(target=launch_ft_sensor.launch_ft_node, args=(event_shutdown_FT,))
	p_launch.start()
	time.sleep(10)  # Wait until the FT node is fully launched
	time_seq = []
	data_FT = []

	FT = ROSInterface("ft_sensor0")
	rospy.init_node('ros_interface')

	time.sleep(5)  # Wait for ROS node to full initialize, otherwise the first FT sensor's calibration will be off

	FT.calibrate_offset()

	print("Begin to measure force ...")


# Terminate FT sensor ROS node 
def endFT():	
	event_shutdown_FT.set()
	time.sleep(8)
	print("Shut down FT launch node!!!")

	
# Read Bota FT sensor	
def readFT():
	global FT
	meas = FT.get_current_FT()
	print(meas[2])
	
	
	
#-------------------------------------Serial Read/Write----------------------------------		
	
	
	

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
	
#------------------------------------Main---------------------------------------------------------	
if __name__ == '__main__':
	global use_FT_sensor
	global p_launch
	global p_FT
	global FT
	time_begin_global = time.time()
	startFT()
	for k in range(0,100):
		#try:
		#	Curr,Pos,Stif,Etc=readSTM()
		#except:
		#	continue
		#print(Curr,Pos,Stif,Etc)
		#dc=1
		#dp=2
		#writeSTM(dc,dp)
		readFT()
		time.sleep(.1)
		
	endFT()
		
			
	
	
