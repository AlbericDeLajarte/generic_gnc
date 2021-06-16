#!/usr/bin/env python3

# -----------------------
#
# Node to add variation between monte carlo simulations, and log data
#
# Input:
#	 - Full simulated state from integrator node:    			\rocket_state
#    - 3D force and torque from the control node:    			\control_pub
#	 - Finite state machine and time since start of simulation:	\fsm_pub
#
# Parameters:
#    - Rocket model: 		/config/rocket_parameters.yaml
#    - Environment model: 	/config/environment_parameters.yaml
#	 - Perturbations model: /config/perturbations_parameters.yaml
#
# Outputs:
#    - Aerodynamic 3D force and torque:  \rocket_aero
#
# -----------------------

import rospy
import rospkg

import numpy as np
import math
from scipy.spatial.transform import Rotation as R

from real_time_simulator.msg import Control
from real_time_simulator.msg import FSM
from real_time_simulator.msg import State

def fsm_callback(fsm):
	global current_fsm
	current_fsm.time_now = fsm.time_now
	current_fsm.state_machine = fsm.state_machine


def rocket_state_callback(state):
	global rocket_state
	rocket_state = state

def rocket_state_est_callback(state):
	global rocket_est_state
	rocket_est_state = state


def control_callback(control):
	global current_control
	current_control = control




if __name__ == '__main__':

    rocket_data = rospy.get_param("/rocket")
    env_data = rospy.get_param("/environment")

    # Define parameters from uniform random distributions
    rail_zenith = np.random.uniform(0, 10)
    rail_azimuth = np.random.uniform(0, 180)
    wind_angle = env_data["wind_direction"] - rail_azimuth
    
    wind_speed = np.random.uniform(0, 15)

    acc_noise = np.random.uniform(0, 0.2)
    gyro_noise = np.random.uniform(0, 0.05)
    baro_noise = np.random.uniform(0, 1)

    wind_gust_x = np.random.uniform(0, 10)
    wind_gust_y = np.random.uniform(0, 10)

    sensor_period = np.random.uniform(40e-3, 56e-3)

    # Set parameters for others node
    rospy.set_param("/environment/rail_zenith", rail_zenith)
    rospy.set_param("/environment/rail_azimuth", rail_azimuth)

    rospy.set_param("/environment/wind_speed", wind_speed)

    rospy.set_param("/perturbation/acc_noise", acc_noise)
    rospy.set_param("/perturbation/gyro_noise", gyro_noise)
    rospy.set_param("/perturbation/baro_noise", baro_noise)

    rospy.set_param("/perturbation/sensor_period", sensor_period)
    
    rospy.set_param("/perturbation/wind_gust_intensity", [wind_gust_x, wind_gust_y])



    # Create global variable
    current_fsm = FSM()
    rocket_state = State()
    rocket_est_state = State()
    current_control = Control()

    # Init ROS
    rospy.init_node('monte_carlo_logger', anonymous=True)

    # Init fsm
    current_fsm.time_now = 0
    current_fsm.state_machine = "Idle"

    # Subscribe to fsm 
    rospy.Subscriber("gnc_fsm_pub", FSM, fsm_callback)

    # Subscribe to rocket_control 
    rospy.Subscriber("control_measured", Control, control_callback)

    # Subscribe to rocket_state 
    rospy.Subscriber("rocket_state", State, rocket_state_callback)

    # Subscribe to rocket estimated state
    rospy.Subscriber("kalman_rocket_state", State, rocket_state_est_callback)

    
    # Log parameters
    rospack = rospkg.RosPack()
    file = open(rospack.get_path('generic_gnc') + "/log/monte_carlo_results.txt", "a+")

    file.write("{:.2f} {:.2f} {:.2f} {:.5f} {:.5f} {:.2f} {:.2f} {:.2f} ".format(rail_zenith, wind_angle, wind_speed,
                                                acc_noise, gyro_noise, baro_noise,
                                                wind_gust_x, wind_gust_y ))

    apogee_reached = False
    end_burn = False

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()

        if (not end_burn) and current_fsm.state_machine == "Coast":
            file.write("{:.2f} {:.2f} ".format(current_fsm.time_now, rocket_data["propellant_mass"]-rocket_state.propeller_mass))
            speed_error_x = rocket_est_state.twist.linear.x-rocket_state.twist.linear.x
            speed_error_y = rocket_est_state.twist.linear.y-rocket_state.twist.linear.y
            speed_error_z = rocket_est_state.twist.linear.z-rocket_state.twist.linear.z
            end_burn = True

        if (not apogee_reached) and rocket_state.twist.linear.z < -0.1:
            apogee_error = rocket_state.pose.position.z - env_data["apogee"][2]


            file.write("{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.5f}\n".format(apogee_error,
                                    rocket_est_state.pose.position.x - rocket_state.pose.position.x,
                                    rocket_est_state.pose.position.y - rocket_state.pose.position.y,
                                    rocket_est_state.pose.position.z - rocket_state.pose.position.z,
                                    speed_error_x, speed_error_y, speed_error_z,
                                    sensor_period))
            file.close()

            apogee_reached = True



    