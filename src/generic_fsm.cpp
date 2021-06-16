/*
* Node to synchronize the GNC algorithms by estimating the current state of the rocket
* (Idle, Rail, Launch, Coast)
*
* Inputs: 
*   - Sensor data (IMU and barometer):					\sensor_pub
*   - Estimated state from generic_navigation:		    \kalman_rocket_state
*
* Parameters:
*   - Threshold for rocket ignition detection (Rail phase): in FSM_thread
*
* Outputs:
*   - Estimated finite state machine from flight data:	\gnc_fsm_pub
*
*/

#include "ros/ros.h"
#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Sensor.h"
#include "real_time_simulator/Control.h"

#include <time.h>

#include <sstream>
#include <string>

#include "real_time_simulator/GetFSM.h"
#include "std_msgs/String.h"

class FSM_publisher
{
	public:
		// global variable with time and state machine
		real_time_simulator::FSM current_fsm;
		double time_zero;
		float rail_length = 0;

		// Global variable with last received sensor data
		real_time_simulator::Sensor current_sensor;

		// global variable with last received rocket state
		real_time_simulator::State current_rocket_state;

		// global variable with last received rocket state
		real_time_simulator::Control current_control;



		// Create Subscribers and Publishers
		ros::ServiceServer timer_service;

		ros::Publisher timer_pub;

		ros::Subscriber rocket_state_sub;

		ros::Subscriber rocket_control_sub;

		ros::Subscriber sensor_sub;

		// Callback function to store last received sensor data
		void sensorCallback(const real_time_simulator::Sensor::ConstPtr& sensor)
		{
			current_sensor.IMU_acc = sensor->IMU_acc;
			current_sensor.IMU_gyro = sensor->IMU_gyro;
			current_sensor.baro_height = sensor->baro_height;

			if (current_fsm.state_machine.compare("Idle") == 0)
			{
				if(current_sensor.IMU_acc.z > 10.5)
				{
					current_fsm.state_machine = "Rail";
					time_zero = ros::Time::now().toSec();
					timer_pub.publish(current_fsm);
				}
			}
		}

		void rocket_stateCallback(const real_time_simulator::State::ConstPtr& rocket_state)
		{
			current_rocket_state.pose = rocket_state->pose;
			current_rocket_state.twist = rocket_state->twist;
			current_rocket_state.propeller_mass = rocket_state->propeller_mass;
		}

		void rocket_controlCallback(const real_time_simulator::Control::ConstPtr& control_law)
		{
			current_control = *control_law;
		}

		// Service function: send back fsm (time + state machine)
		bool sendFSM(real_time_simulator::GetFSM::Request &req, real_time_simulator::GetFSM::Response &res)
		{
			// Update current time
			if (current_fsm.state_machine.compare("Idle") != 0) current_fsm.time_now = ros::Time::now().toSec() - time_zero;

			res.fsm.time_now = current_fsm.time_now;
			res.fsm.state_machine = current_fsm.state_machine;
			
			return true;
		}

		void init(ros::NodeHandle n)
		{
			n.getParam("/environment/rail_length", rail_length);

			// Initialize fsm
			current_fsm.time_now = 0;
			current_fsm.state_machine = "Idle";

			// Initialize rocket state
			n.getParam("/rocket/propellant_mass", current_rocket_state.propeller_mass); // To stay in launch mode at first iteration

			current_control.force.z = 3000;
			
			timer_pub.publish(current_fsm);
		}

		void update_fsm()
		{
			if(current_fsm.state_machine.compare("Idle")!=0)
			{
				// Update current time
				current_fsm.time_now = ros::Time::now().toSec() - time_zero;
				
				if (current_fsm.state_machine.compare("Rail") == 0)
				{
					// End of rail
					if(current_rocket_state.pose.position.z > rail_length)
					{
						current_fsm.state_machine = "Launch";
					}
				}

				else if (current_fsm.state_machine.compare("Launch") == 0)
				{
					// End of burn -> no more thrust
					if(current_rocket_state.propeller_mass < 0 || current_control.force.z == 0)
					{
						current_fsm.state_machine = "Coast";
					}

				}

				else if (current_fsm.state_machine.compare("Coast") == 0)
				{
				// Do nothing for now
				}

				// Publish time + state machine    
				timer_pub.publish(current_fsm);
			}

		}
};



int main(int argc, char **argv)
{

	// Init ROS time keeper node
	ros::init(argc, argv, "generic_fsm");
	ros::NodeHandle n;

	FSM_publisher fsm;

	// Create timer service
	fsm.timer_service = n.advertiseService("getFSM_gnc",  &FSM_publisher::sendFSM, &fsm);

	// Create timer publisher
	fsm.timer_pub = n.advertise<real_time_simulator::FSM>("gnc_fsm_pub", 10);

	// Subscribe to state message
	fsm.rocket_state_sub = n.subscribe("kalman_rocket_state", 100, &FSM_publisher::rocket_stateCallback, &fsm);

	// Subscribe to control message
	fsm.rocket_control_sub = n.subscribe("control_pub", 100, &FSM_publisher::rocket_controlCallback, &fsm);

	// Subscribe to sensors message
	fsm.sensor_sub = n.subscribe("sensor_pub", 100, &FSM_publisher::sensorCallback, &fsm);

	fsm.init(n);

	ros::Timer FSM_thread = n.createTimer(ros::Duration(0.01),
	[&](const ros::TimerEvent&) 
	{
		// Update FSM
		fsm.update_fsm();

	});

	// Automatic callback of service and publisher from here
	ros::spin();

}
