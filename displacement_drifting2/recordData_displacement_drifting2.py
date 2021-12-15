from os import read
import re
import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.msg import MotorState
from roboy_middleware_msgs.srv import ControlMode
from std_msgs.msg import String
from datetime import datetime

import numpy as np
import csv
from time import time, sleep

#initial parameters
EXPERIMENT_ID = "displacement_drifting2"

MOTOR_NUM = 7

WEIGHT = 0

MOTOR_H_ID = 3

PWM = 2

NUMBER_OF_ROTATIONS = 200

MAX_TICKS = NUMBER_OF_ROTATIONS * 4096

time_now = datetime.now()

readValuesLabel = [ "protocol",
                    "starttime",
                    "timestamp",
                    "encoder0_scaled_old",
                    "encoder1_scaled_old",
                    "displacement_old"]
readValues = []

readValuesAtDistanceMarks = []

writtenSetpoinits = []

global stop
stop = False
#create protocol lable
protocol = "pwm" + str(PWM) + "_" + str(NUMBER_OF_ROTATIONS) +"rotations_" + str(MOTOR_H_ID) + "motorHardwareID_" + str(WEIGHT) + "kg_weight" 

#creat global publisher and massege
pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)

cmd = MotorCommand()
cmd.global_id = [MOTOR_NUM]

#define subscriber callback to append data to readValues
def callback(data):
    readValues.append([protocol,time_now,str(datetime.now()),data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],data.displacement[MOTOR_NUM]])
    #stop at 200 rotations
    if(data.encoder0_pos[MOTOR_NUM] >= MAX_TICKS):
        cmd.setpoint = [0]
        pub.publish(cmd)
        global stop
        stop = True

def main():
    rospy.init_node(EXPERIMENT_ID)
    print(protocol)
    print("Set Motor Control Mode")

    rospy.wait_for_service('/roboy/pinky/middleware/ControlMode')
    #call service to set control mode to pwm
    try:
           call_service = rospy.ServiceProxy('/roboy/pinky/middleware/ControlMode', ControlMode)
           resp1 = call_service(3,[0], [MOTOR_NUM])
           print("Control Mode Set")
    except rospy.ServiceException as e:
           print("Service call failed: %s"%e)

    #initialize publisher and subscriber
    writtenSetpoinits.append(0)
    
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)

    

    rate = rospy.Rate(600)

    #start measurment
    try:
        print("to start press any key")
        input()
        cmd.setpoint = [PWM]
        pub.publish(cmd)
        writtenSetpoinits.append(PWM)
        while not stop:
            sleep(0.1)
    except Exception as e:
        print(e.message,e.args)
        print("Save Data? (y/anything else)")
        if(input() != "y"):
            return
    #wait a little so it stoped turning for sure
    sleep(0.5)
    
    newFile = True
    #check if file already exists if not create it with colum labels
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
    rate.sleep()

if __name__ == '__main__':

    main()
 