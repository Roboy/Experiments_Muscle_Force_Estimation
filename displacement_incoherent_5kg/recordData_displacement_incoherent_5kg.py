from os import read
import re
import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.msg import MotorState
from std_msgs.msg import String
from datetime import datetime
from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *

import numpy as np
import csv
from time import time, sleep

class ConnectionError(Exception):
	"""Raised when connection to load cell channels fails."""
	def __init__(self, message):
		self.message = message


class LoadCell(VoltageRatioInput):
	"""This class handles a single load cell channel.
	
	"""
	def __init__(self, conf):
		"""
		Args:
			conf (dict): Dictionary with initial channel configuration:
							'tendon_id' 	(int): Tendon id to which the cell is attached.
							'cal_offset'  (float): Calibration offset value of the load cell.
							'cal_factor'  (float): Calibration factor value of the load cell.
							'serial' 		(int): Serial number of the phidget to which the cell is connected.
							'channel' 		(int): Number of the phidget channel to which the cell is connected.
		
		"""
		super().__init__()
		self.id = conf['tendon_id']
		self.cal_offset = conf['cal_offset']
		self.cal_factor = conf['cal_factor']
		self.setDeviceSerialNumber(conf['serial'])
		self.setChannel(conf['channel'])

	@property
	def ready(self):
		return self.cal_offset is not None and self.cal_factor is not None

	def getAddress(self):
		return f"{self.getDeviceSerialNumber()}/{self.getChannel()}"

	def openChannel(self):
		"""Opens and attaches to phidget channel.
		
		Args:
			-
		Returns:
		   	-
		"""
		if not self.getAttached():
			try:
				self.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
			except PhidgetException as e:
				raise ConnectionError(e.details)

	def closeChannel(self):
		"""Closes phidget channel.
		
		Args:
			-
		Returns:
		   	-
		"""
		self.close()

	def readForce(self):
		"""Reads force value from load cell in Newtons.
		
		Args:
			-
		Returns:
		   	float: Force value in Newtons.
		"""
		force = 0
		if self.ready:
			force = self.cal_factor * (self.getVoltageRatio() + self.cal_offset)
		return force * 9.81

#initial parameters
EXPERIMENT_ID = "displacement_incoherent_5kg"

MOTOR_NUM = 7

MAX_ROTATIONS = 10

STEP_ROTATIONS = MAX_ROTATIONS
#STEP_ROTATIONS = 2.5

TICKS_PER_ROTATION = 4096

MOTOR_H_ID = 3

WAIT_TIME = 20

WEIGHT = 10

time_now = datetime.now()

readValuesLabel = [ "protocol",
                    "starttime",
                    "timestamp",
                    "encoder0_scaled_old",
                    "encoder1_scaled_old",
                    "displacement_old",
                    "loadcell_force"]

readValues = []

readValuesAtDistanceMarks = []


writtenSetpoinits = []

loadcellforce = 0

#creat protocol label
protocol = "lengthControl_"+ str(MOTOR_H_ID) + "motorHardwareID_" + str(MAX_ROTATIONS)+"maxRotations_"+str(STEP_ROTATIONS)+"stepRotations_"+str(WAIT_TIME)+"s_waitTime_"+str(WEIGHT)+"kg_weight"
print(protocol)
#define subscriber callback to append data to readValues
def callback(data):
    #read data from load cell
    loadcellforce = channels[0].readForce()
    readValues.append([protocol,time_now,str(datetime.now()),data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],data.displacement[MOTOR_NUM],loadcellforce])


#load cell parameters
phidget_serial = 585671
cal_offsets = [-1.0473654e-05]
cal_factors = [-7956.613617122953]

#loadcell config
configuration = [{'tendon_id': i, 'cal_offset': o, 'cal_factor': f, 'serial': phidget_serial, 'channel': 1} for i, (o, f) in enumerate(zip(cal_offsets, cal_factors))]

def main():
    rospy.init_node(EXPERIMENT_ID)
	
    pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)

    setpoints = np.arange(0, MAX_ROTATIONS*TICKS_PER_ROTATION + 1, STEP_ROTATIONS*TICKS_PER_ROTATION)
    cmd = MotorCommand()
    cmd.global_id = [MOTOR_NUM]

    rate = rospy.Rate(600)

    try:
        for s in setpoints:
            writtenSetpoinits.append(s)
            cmd.setpoint = [s]
            pub.publish(cmd)
            rate.sleep()
            #make sure the movment has stoped
            while(len(readValues) > 1 and readValues[len(readValues)-1][3] != readValues[len(readValues)-2][3]):
                rate.sleep()
            print(s)
            #wait for WAIT_TIME
            sleep(WAIT_TIME)
    except Exception as e:
        print(e.message,e.args)
        print("Save Data? (y/anything else)")
        if(input() != "y"):
            return
    #check if file already exists if not create it with colum labels
    newFile = True
    try:
        f = open(EXPERIMENT_ID + ".csv", 'x')
        print("create file")
    except:
        print("Append to existing file")
        newFile = False
    #append to file the collected data
    with open(EXPERIMENT_ID + ".csv", 'a') as f: 
        write = csv.writer(f)
        if newFile: 
            write.writerow(readValuesLabel) 
        write.writerows(readValues)
    print("SAVED")
    #wait till return is pressed so the weight doesn't hit the ground grab it by hand !
    print("Bitte Taste zum Ablassen drücken")
    input()
    cmd.setpoint = [0.0]
    pub.publish(cmd)
    rate.sleep()

if __name__ == '__main__':
    #open phigets channel
    channels = []
    for i, conf in enumerate(configuration):
        new_channel = LoadCell(conf)
        try:
            new_channel.openChannel()
            new_channel.setDataInterval(10)
            channels.append(new_channel)
        except ConnectionError as e:
            print(f"Failed to open load cell {i}: {e.message}")   
    main()
    # Close your Phidgets once the program is done.
    for channel in channels:
        channel.closeChannel()
        print("closed")