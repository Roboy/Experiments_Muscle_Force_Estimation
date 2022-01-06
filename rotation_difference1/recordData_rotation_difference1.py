from os import read
import re
import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.msg import MotorState
from roboy_middleware_msgs.msg import MotorConfig
from roboy_middleware_msgs.srv import MotorConfigService, MotorConfigServiceRequest
from std_msgs.msg import String
from datetime import datetime

import numpy as np
import csv
from time import time, sleep

MOTOR_NUM = 7

TICKS_PER_ROTATION_DIV = 512 * 4 * 2

NUMBER_OF_ROTATIONS = 200

WEIGHT = 0

PWM_LIMIT = 3

EXPERIMENT_ID = "rotation_difference1"

DT_THRESHHOLD = 0.5

time_now = datetime.now()

print("Current time:", time_now)

ticksToSend =  NUMBER_OF_ROTATIONS * TICKS_PER_ROTATION_DIV

readValuesLabel = [ "protocol",
                    "starttime",
                    "timestamp",
                    "encoder0_scaled_old",
                    "encoder1_scaled_old",
                    "total_rotations"]

totalRotations = 0

readValues = []

cmd = MotorCommand()    
cmd.global_id = [MOTOR_NUM]

pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
protocol = "lengthControl_pwmLimit"+str(PWM_LIMIT)+"_"+str(NUMBER_OF_ROTATIONS)+"rotations_"+str(WEIGHT) + "kg_weight"

def callback(data):
    readValues.append([protocol,time_now,str(datetime.now()),data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],totalRotations])

def main():
    rospy.init_node("experimant_fpga_division_impact0")

    print("Set PWM Limit")

    rospy.wait_for_service('/roboy/pinky/middleware/MotorConfig')

    try:
            call_service = rospy.ServiceProxy('/roboy/pinky/middleware/MotorConfig', MotorConfigService)
            cmd_Limit = MotorConfigServiceRequest()
            cmd_Limit.legacy = False
            cmd_Limit.config.global_id = [MOTOR_NUM]
            cmd_Limit.config.PWMLimit = [PWM_LIMIT]
            cmd_Limit.config.setpoint = [0.0]
            cmd_Limit.config.control_mode = [0]
            cmd_Limit.config.update_frequency = [500]
            cmd_Limit.config.IntegralLimit = [25]
            cmd_Limit.config.Kp = [4]
            cmd_Limit.config.Ki = [0]
            cmd_Limit.config.Kd = [0]
            cmd_Limit.config.deadband = [0]
            resp1 = call_service(cmd_Limit)
            print("PWM Limit Set")
    except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return

    global totalRotations
    totalRotations = 0
    
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)
    
    rate = rospy.Rate(600)
    t1 = datetime.now()

    print("Press return every rotation.")
    print("Input s to stop the data collection. Does NOT increase the rotation count!")
    print("Press return to verify and start")
    input()
    print("START")
    sleep(1)

    cmd.setpoint = [ticksToSend]
    pub.publish(cmd)
    
    while 1:
        print("Press return to incerease rotation count")
        stop = input()
        
        if((datetime.now()-t1).total_seconds() < DT_THRESHHOLD):
            continue

        if stop == 's':
            print("Number of rotations counted:",totalRotations)
            while(1):
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
                    print("You want to throw away the data?!")
                    print("Are you sure? yes to abort!")
                    userInput = input()
                    sleep(0.2)
                    
                    if(userInput == "yes"):
                        print("ABORT")
                        return

        t1 = datetime.now()
        totalRotations = totalRotations + 1
        print("Current number of rotations:",totalRotations)

if __name__ == '__main__':

    main()
