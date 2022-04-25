from os import read
import re
import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.msg import MotorState
from roboy_middleware_msgs.srv import ControlMode
from std_msgs.msg import String
from datetime import datetime
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
import numpy as np
import csv
from time import time, sleep

#initial parameters
EXPERIMENT_ID = "live_magnetic_field_correction"
MOTOR_NUM = 7
WEIGHT = 0
MOTOR_H_ID = 1
PWM = 3
TOTAL_TIME = 30
ONE_ROTATION = 216000
time_now = datetime.now()

#polynomial correction
fitting_functions = {
    3:np.poly1d([1])
}


calibration_encoder0 = []
calibration_displacement= []
calibrartion_running = False
calibration_rotations = 25

max_rotations = 38 #before overflow

readValuesLabel = [ "protocol",
                    "starttime",
                    "timestamp",
                    "encoder0_scaled_old",
                    "encoder1_scaled_old",
                    "displacement_old"]
readValues = []
readValuesAtDistanceMarks = []
writtenSetpoinits = []



#create protocol lable
protocol = "pwm" + str(PWM) + "_" + str(TOTAL_TIME) +"s_duration_" + str(MOTOR_H_ID) + "motorHardwareID_" + str(WEIGHT) + "kg_weight" 

#live plot prep
style.use('fivethirtyeight')
fig = plt.figure(dpi=100)

ax1 = fig.add_subplot(1,1,1)

plot_data = []

def calibration_fit(x):
    return fitting_functions[PWM](x%ONE_ROTATION)


def find_positon_in_fitting_function():
    x = np.array(calibration_encoder0)%ONE_ROTATION
    y = np.array(calibration_displacement)
    global fitting_functions
    res = np.polyfit(x,y,14)
    print(res)
    fitting_functions[PWM] = np.poly1d(res)
    return


def animate(i):
    xs = []
    ys = []
    zs = []
    global plot_data
    if(len(plot_data) > 1):
            for x,y,z in plot_data:
                xs.append(x)
                ys.append(y)
                zs.append(z)
    ax1.clear()

    ax1.scatter(xs, ys,label="displacement")
    ax1.scatter(xs, zs,label="corrected")
    ax1.set_xlabel("encoder0%216000")
    ax1.set_ylabel("displacement")
    ax1.legend()

#define subscriber callback to append data to readValues
def callback(data):
    if(calibrartion_running):
        calibration_encoder0.append(data.encoder0_pos[MOTOR_NUM])
        calibration_displacement.append(data.displacement[MOTOR_NUM])
    else:
        displacement = data.displacement[MOTOR_NUM]
        plot_data.append([data.encoder0_pos[MOTOR_NUM]%216000, displacement, displacement - calibration_fit(data.encoder0_pos[MOTOR_NUM]) ])
    readValues.append([protocol,time_now,str(datetime.now()),data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],data.displacement[MOTOR_NUM]])
    

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
    pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)

    cmd = MotorCommand()
    cmd.global_id = [MOTOR_NUM]

    rate = rospy.Rate(600)

    #start measurment
    try:
        print("to start press any key")
        input()

        print("Calibration start")
        end_point = readValues[-1][3] + calibration_rotations * ONE_ROTATION
        cmd.setpoint = [PWM]
        pub.publish(cmd)
        writtenSetpoinits.append(PWM)

        global calibrartion_running

        calibrartion_running = True
        while(readValues[-1][3] <= end_point):

            print(readValues[-1][3],end_point)
            sleep(0.1)
        calibrartion_running = False
        
        cmd.setpoint = [0]
        pub.publish(cmd)
        writtenSetpoinits.append(0)
        print("calibration stop")

        find_positon_in_fitting_function()
        global plot_data
        sleep(1)
        print("start live correction plotting")

        cmd.setpoint = [PWM]
        pub.publish(cmd)
        writtenSetpoinits.append(PWM)
        end_point = readValues[-1][3] + (max_rotations - calibration_rotations) * ONE_ROTATION
        
        ani = animation.FuncAnimation(fig, animate, interval=1000)
        plt.show()

        while(readValues[-1][3] <= end_point):

            print(readValues[-1][3],end_point)
            sleep(0.1)

        print("stop")
        cmd.setpoint = [0]
        pub.publish(cmd)
        writtenSetpoinits.append(0)

    except Exception as e:
        print(e.message,e.args)

        print("Save Data? (y/anything else)")
        if(input() != "y"):
            return

    #wait a little so it stoped turning for sure
    sleep(1)    
    
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
 