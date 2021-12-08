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

MOTOR_NUM = 7

TICKS_PER_ROTATION_DIV = 512 * 4 * 53

NUMBER_OF_ROTATIONS = 15

WEIGHT = 0

PWM = 10

EXPERIMENT_ID = "fpga_division_impact0"

time_now = datetime.now()

print(time_now)

maxTicks = NUMBER_OF_ROTATIONS * TICKS_PER_ROTATION_DIV

readValuesLabel = [ "protocol",
                    "starttime",
                    "encoder0_raw",
                    "encoder1_raw",
                    "displacement_old"]


readValues = []

cmd = MotorCommand()
cmd.global_id = [MOTOR_NUM]

pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
protocol = "pwm" + str(PWM) + "_"+str(WEIGHT) + "kg_weight"

def callback(data):
    readValues.append([protocol,time_now,data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],data.displacement[MOTOR_NUM]])
    if(data.encoder0_pos[MOTOR_NUM] >= maxTicks):
        cmd.setpoint = [0]
        pub.publish(cmd)

def main():
    rospy.init_node("experimant_fpga_division_impact0")

    print("Set Motor Control Mode")

    rospy.wait_for_service('/roboy/pinky/middleware/ControlMode')

    try:
           call_service = rospy.ServiceProxy('/roboy/pinky/middleware/ControlMode', ControlMode)
           resp1 = call_service(3,[0], [MOTOR_NUM])
           print("Control Mode Set")
    except rospy.ServiceException as e:
           print("Service call failed: %s"%e)
    print("START")
    
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)
    
    rate = rospy.Rate(600)

    sleep(1)

    cmd.setpoint = [PWM]
    pub.publish(cmd)
    
    while 1:
        if readValues[len(readValues) - 1][2] >= maxTicks:
            print("Type y to save.")
            userInput = input()
            if userInput == 'y':
                newFile = True
                try:
                    f = open(EXPERIMENT_ID + ".csv", 'x')
                    print("create file")
                except:
                    print("Append to existing file")
                    newFile = False
                with open(EXPERIMENT_ID + ".csv", 'a') as f: 
                    write = csv.writer(f)
                    if newFile: 
                        write.writerow(readValuesLabel) 
                    write.writerows(readValues)
                print("SAVED")
                return
            else:
                print("ABORT")
                return
        sleep(0.1)

if __name__ == '__main__':

    main()
