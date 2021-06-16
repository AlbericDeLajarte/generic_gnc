#!/usr/bin/env python3
from numpy.lib.type_check import real
import rospy
import rospkg

import numpy as np
import math
import serial
from scipy.interpolate import interp1d 
import time

from real_time_simulator.msg import Control
from real_time_simulator.msg import FSM
from real_time_simulator.msg import State
from real_time_simulator.msg import Sensor

import msv2
import struct

CM4_H2C_PING        =   0x00
CM4_H2C_SHUTDOWN    =   0x01
CM4_H2C_PAYLOAD     =   0x02
CM4_H2C_SENSORS     =   0x03
CM4_H2C_FEEDBACK    =   0x04

CM4_C2H_PING        =   0x80 
CM4_C2H_COMMAND     =   0x81

CONVERSION = 76.233

THROTTLING = True


hb = None
current_kalman = State()

def shutdown():
    print("shutting down...")
    command = "/usr/bin/sudo /sbin/shutdown -h now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print(output)
    
def ping():
    print("ping")
    hb.send_from_slave(CM4_H2C_PING, [0xc5, 0x5c])
    
def sensors(data):
    if data and len(data) == 32:
        sens_data = struct.unpack("I"+"iii"+"iii"+"i", bytes(data))
        #print("payload data: ", sens_data)

        # Send the received sensor and actuators data to ROS
        sensor_data = Sensor()
        
        timestamp = sens_data[0]

        sensor_data.IMU_acc.x = 9.81*sens_data[1]/1000.0
        sensor_data.IMU_acc.y = 9.81*sens_data[2]/1000.0
        sensor_data.IMU_acc.z = 9.81*sens_data[3]/1000.0

        sensor_data.IMU_gyro.x = sens_data[4]/1000.0
        sensor_data.IMU_gyro.y = sens_data[5]/1000.0
        sensor_data.IMU_gyro.z = sens_data[6]/1000.0

        sensor_data.baro_height = sens_data[7]/1000.0
        

        hb.send_from_slave(CM4_H2C_SENSORS, [0xc5, 0x5c])

        sensor_pub.publish(sensor_data)
        
    else:
        # Send error code
        hb.send_from_slave(CM4_H2C_SENSORS, [0xce, 0xec])


def feedback(data):
    if data and len(data) == 24:
        feedback_data = struct.unpack("I"+"i"+"iiii", bytes(data))


        measured_control = Control()

        measured_control.force.z = feedback_data[1]/CONVERSION

        actuator_pub.publish(measured_control)





def recv_data(opcode, data):
    #print('message ({}): [{}]'.format(opcode, ', '.join(hex(x) for x in data)))
    if opcode == CM4_H2C_PING:
        ping()
    if opcode == CM4_H2C_SHUTDOWN:
        shutdown()
    if opcode == CM4_H2C_SENSORS:
        sensors(data)
    if opcode == CM4_H2C_FEEDBACK:
        feedback(data)
   #print('{} at {:.3f}'.format(opcode, rospy.get_time()))

    
    

def send_control(control):
    fsm_states = ["", "Idle", "Rail", "Launch", "Coast"]

    if current_fsm.state_machine == "Rail":
        gpio.write(17, 0)
    if current_fsm.state_machine == "Launch":
        gpio.write(27, 0)
    if current_fsm.state_machine == "Coast":
        gpio.write(22, 0)

    try:
        true_state = fsm_states.index(current_fsm.state_machine)
    except:
        true_state = 0

    # Send the last commanded control to the AV
    s_data = struct.pack("I"+"i"+"iiii"+"iii"+"iii"+"H",
              0,
              int(control.force.z),
              0,
              0,
              0,
              0,
              int(current_kalman.pose.position.x*1000),
              int(current_kalman.pose.position.y*1000),
              int(current_kalman.pose.position.z*1000),
              int(current_kalman.twist.linear.x*1000),
              int(current_kalman.twist.linear.y*1000),
              int(current_kalman.twist.linear.z*1000),
              true_state
              )
    hb.send_from_slave(CM4_C2H_COMMAND, s_data)

def control_callback(control):
    global current_control
    current_control = control

    #print(current_control)

    if GNC_mode != "SIL":
        send_control(control)

def simu_sensor_callback(sensor):
    # Send back sensors and control as official flight data for GNC
    sensor_pub.publish(sensor)

def kalman_callback(kalman):
    global current_kalman
    current_kalman = kalman

def fsm_callback(fsm):
    global current_fsm
    current_fsm = fsm
    


if __name__ == '__main__':

    GNC_mode = ["Flight", "HIL", "PIL", "SIL"][rospy.get_param("/simulation")]

    THROTTLING = rospy.get_param("/rocket/throttling")

    # Create global variable
    current_control = Control()

    current_fsm = FSM()
    current_fsm.state_machine = "Idle"

    if GNC_mode == "Flight":
        import pigpio
        
        gpio = pigpio.pi()

        gpio.write(17, 1)
        gpio.write(27, 1)
        gpio.write(22, 1)

    

    # Init ROS
    rospy.init_node('av_interface', anonymous=True)
    
    # Subscribed topics: control, navigation state, fsm  
    rospy.Subscriber("control_pub", Control, control_callback)
    rospy.Subscriber("kalman_rocket_state", State, kalman_callback)
    rospy.Subscriber("gnc_fsm_pub", FSM, fsm_callback)

    # Published topics: sensor data, actuator feedback
    sensor_pub = rospy.Publisher('sensor_pub', Sensor, queue_size=10)
    actuator_pub = rospy.Publisher('control_measured', Control, queue_size=1)

    # Connect to AV if in flight mode (simulation = 0)
    if GNC_mode != "SIL":
        hb = msv2.msv2()
        if hb.connect("/dev/serial0"):
            print("connected")
            hb.slave(recv_data)
            quit(0)
        else:
            print("\033[91m" + " Error opening serial port. Shutting down" + '\033[0m')
            rospy.signal_shutdown("No connection with Avionics")

    # If not in flight mode, we enter the ROS loop to remap sensor data from simulation to real sensor data       
    else:
        measured_control = Control()
        rospy.Subscriber("simu_sensor_pub", Sensor, simu_sensor_callback)

        # Load motor thrust curve to get real thrust (for control_measured)
        rospack = rospkg.RosPack()
        thrust_curve = np.loadtxt(rospack.get_path("real_time_simulator") + "/config/thrust_curve/motor_file.txt")
        f_thrust = interp1d(thrust_curve[:,0], thrust_curve[:,1])

        # Init motor force to the one after one integration period
        current_control.force.z = rospy.get_param("/rocket/maxThrust")[2]

        rate = rospy.Rate(1.0/(2*rospy.get_param("/rocket/output_delay")))

        while not rospy.is_shutdown():
        
            # Thread sleep time defined by rate
            rate.sleep()

            if current_fsm.state_machine != "Idle":
                real_thrust = 0.0

                if THROTTLING:
                    real_thrust = current_control.force.z

                else:
                    if current_control.force.z != 0.0 and current_fsm.time_now >= thrust_curve[0,0] and current_fsm.time_now <= thrust_curve[-1,0]:
                        real_thrust = float(f_thrust(current_fsm.time_now))

                measured_control.force.z = real_thrust
                actuator_pub.publish(measured_control)
