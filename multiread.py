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
EXPERIMENT_ID = "displacement_drifting_multi_read_6_30mm_distance_sensor_magnet_30"

MOTOR_NUM = 7

WEIGHT = 0

MOTOR_H_ID = 1

global distance
distance = 630

global PWMs
PWMs = []

global PWM
PWM = 3

NUMBER_OF_ROTATIONS = 200

global MAX_TICKS
MAX_TICKS = NUMBER_OF_ROTATIONS * 216000

TICKS_PER_RUN = MAX_TICKS

global ENC0_OFFSET
ENC0_OFFSET = 0

global ENC1_OFFSET
ENC1_OFFSET = 0

global ENC0_CORR
ENC0_CORR = 0

global ENC1_CORR
ENC1_CORR = 0

global ENC0_SIGN
ENC0_SIGN = 1

global ENC1_SIGN
ENC1_SIGN = 1

global time_now
time_now = datetime.now()

readValuesLabel = [ "protocol",
                    "starttime",
                    "timestamp",
                    "encoder0_scaled2",
                    "encoder1_scaled2",
                    "displacement2",
                    "pwm",
                    "motorHardwareID",
                    "distance"]
readValues = []


global stop
stop = False
#create protocol lable
protocol = "pwm" + str(PWM) + "_" + "runFor_2400s_" + str(MOTOR_H_ID) + "motorHardwareID_" + str(WEIGHT) + "kg_weight_"+str(distance)+"mm_distance" 

#creat global publisher and massege
pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)

cmd = MotorCommand()
cmd.global_id = [MOTOR_NUM]

#define subscriber callback to append data to readValues
def callback(data):
    global MAX_TICKS
    global ENC0_SIGN
    global ENC1_SIGN

    if(data.encoder0_pos[MOTOR_NUM]*ENC0_SIGN < 0):
        if(data.encoder0_pos[MOTOR_NUM] < 0):
            global ENC0_CORR
            ENC0_CORR += 16777216
        ENC0_SIGN = -ENC0_SIGN

    if(data.encoder1_pos[MOTOR_NUM]*ENC1_SIGN < 0):
        if(data.encoder1_pos[MOTOR_NUM] < 0):
            global ENC1_CORR
            ENC1_CORR += 16777216 
        ENC1_SIGN = -ENC1_SIGN

    enc0 = data.encoder0_pos[MOTOR_NUM] + ENC0_CORR
    enc1 = data.encoder1_pos[MOTOR_NUM] + ENC1_CORR

    # if(enc0 >= MAX_TICKS):
    #     global PWMs
    #     if(len(PWMs) == 0):
    #         cmd.setpoint = [0]
    #         pub.publish(cmd)
    #         global stop
    #         stop = True

    #     else:
    #         sleep(5)
    #         global time_now
    #         time_now = datetime.now()


    #         global PWM
    #         PWM = PWMs[0]

    #         PWMs = PWMs[1:]

    #         cmd.setpoint = [PWM]
    #         pub.publish(cmd)

    #         global protocol
    #         protocol = "pwm" + str(PWM) + "_" + str(NUMBER_OF_ROTATIONS) +"rotations_" + str(MOTOR_H_ID) + "motorHardwareID_" + str(WEIGHT) + "kg_weight_"+str(distance)+"mm_distance" 

    #         global TICKS_PER_RUN
    #         MAX_TICKS += TICKS_PER_RUN

    #         global ENC0_OFFSET
    #         ENC0_OFFSET = enc0

    #         global ENC1_OFFSET
    #         ENC1_OFFSET = enc1

    #         print(protocol)
    #print(enc0)
    readValues.append([protocol,time_now,str(datetime.now()),enc0-ENC0_OFFSET ,enc1-ENC1_OFFSET ,data.displacement[MOTOR_NUM],PWM,MOTOR_H_ID])


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

    
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)

    rate = rospy.Rate(600)

    #start measurment
    try:
        print("to start press any key")
        input()
        global PWM
        cmd.setpoint = [PWM]
        pub.publish(cmd)
        print(readValues[0])
        # while not stop:
        #     sleep(0.1)
        sleep(2400)
        cmd.setpoint = [0]
        pub.publish(cmd)
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
 