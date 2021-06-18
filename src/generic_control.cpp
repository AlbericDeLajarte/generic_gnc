/*
* Node to send control commands to the rocket engine. 
* Can also be used by the simulation for SIL and PIL tests.
*
* Inputs: 
*   - Finite state machine from the generic_gnc package:	\gnc_fsm_pub
*   - Estimated state from generic_navigation:		        \kalman_rocket_state
*
* Parameters:
*   - Rocket model: 		  /config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
#	  - P gain: 		        PD_control
*
* Outputs:
*   - Commanded 3D force and torque for the rocket engine:  \control_pub
*
*/

#include "ros/ros.h"

#include <ros/package.h> 

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Waypoint.h"
#include "real_time_simulator/Trajectory.h"

#include "real_time_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include "real_time_simulator/GetFSM.h"
#include "real_time_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#define CONTROL_HORIZON 2 // In seconds

#include "../submodule/polympc/src/polynomials/ebyshev.hpp"
#include "../submodule/polympc/src/control/continuous_ocp.hpp"
#include "../submodule/polympc/src/polynomials/splines.hpp"

#include "../submodule/polympc/src/solvers/sqp_base.hpp"
//#include "../submodule/polympc/src/solvers/osqp_interface.hpp"

#include "../submodule/polympc/src/control/mpc_wrapper.hpp"

#include <iomanip>
#include <iostream>
#include <chrono>

#include "rocket_model.hpp"
#include "polympc_redef.hpp" 
#include "rocket_mpc.hpp"

using namespace Eigen;  
 
using admm = boxADMM<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t,
                control_ocp::MATRIXFMT, linear_solver_traits<control_ocp::MATRIXFMT>::default_solver>;
                
//using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;


// Creates solver
using mpc_t = MPC<control_ocp, MySolver, admm>;
mpc_t mpc;
 
 
// Global variable with last received rocket state
real_time_simulator::State current_state;

// Global variable with last received control
real_time_simulator::Control current_control;

// Global variable with last receive reference trajectory
real_time_simulator::Trajectory current_trajectory;

// Global variable with last requested fsm 
real_time_simulator::FSM current_fsm;

// Callback function to store last received state
void rocket_stateCallback(const real_time_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;
}

// Callback function to store last received measured control
void controlCallback(const real_time_simulator::Control::ConstPtr& control)
{
	current_control.torque = control->torque;
	current_control.force = control->force;
}

// Callback function to store last received measured control
void guidanceCallback(const real_time_simulator::Trajectory::ConstPtr& trajectory)
{
	current_trajectory = *trajectory;
  //std::cout << current_trajectory.trajectory[0] << "\n";
}

void fsm_Callback(const real_time_simulator::FSM::ConstPtr& fsm)
{
  current_fsm.state_machine = fsm->state_machine;
  current_fsm.time_now = fsm->time_now;
}

Eigen::Matrix<double, 4,1> convertControl_SI(const Eigen::Matrix<double, 4,1>& u)
{
  Eigen::Matrix<double, 4,1> input; input << rocket.maxThrust[0]*u(0), rocket.maxThrust[1]*u(1), 0.5*((rocket.maxThrust[2]-rocket.minThrust[2])*u(2) + rocket.maxThrust[2]+rocket.minThrust[2] ), rocket.maxTorque*u(3);
  return input;
}

void trajectory_interpolate()
{
  // Find closest point in time compared to requested time
  int i = 0;
  for(i = 0; i<mpc.ocp().NUM_NODES; i++)
  {
    if(current_trajectory.trajectory[i].time > current_fsm.time_now+CONTROL_HORIZON)
    {
      break;
    }
  } 
  int prev_point = i-1;
  //std::cout << prev_point << "\n";
  
  // If requested time lies inside array of points:
  if (prev_point >= 0 && prev_point < mpc.ocp().NUM_NODES -1)
  {  
    float ratio = ((current_fsm.time_now+CONTROL_HORIZON - current_trajectory.trajectory[prev_point].time)/(current_trajectory.trajectory[prev_point+1].time - current_trajectory.trajectory[prev_point].time));

    mpc.ocp().xs << current_trajectory.trajectory[prev_point].position.x +  ratio* (current_trajectory.trajectory[prev_point+1].position.x - current_trajectory.trajectory[prev_point].position.x),
                    current_trajectory.trajectory[prev_point].position.y +  ratio* (current_trajectory.trajectory[prev_point+1].position.y - current_trajectory.trajectory[prev_point].position.y),
                    current_trajectory.trajectory[prev_point].position.z +  ratio* (current_trajectory.trajectory[prev_point+1].position.z - current_trajectory.trajectory[prev_point].position.z),

                    current_trajectory.trajectory[prev_point].speed.x +  ratio* (current_trajectory.trajectory[prev_point+1].speed.x - current_trajectory.trajectory[prev_point].speed.x),
                    current_trajectory.trajectory[prev_point].speed.y +  ratio* (current_trajectory.trajectory[prev_point+1].speed.y - current_trajectory.trajectory[prev_point].speed.y),
                    current_trajectory.trajectory[prev_point].speed.z +  ratio* (current_trajectory.trajectory[prev_point+1].speed.z - current_trajectory.trajectory[prev_point].speed.z),

                    0, 0, 0, 1, 
                    
                    0, 0, 0,

                    current_trajectory.trajectory[prev_point].propeller_mass +  ratio* (current_trajectory.trajectory[prev_point+1].propeller_mass - current_trajectory.trajectory[prev_point].propeller_mass);
    
    //mpc.ocp().xs.head(6) << 0,0,rocket.target_apogee[2],0,0,0;

    mpc.ocp().xs.head(6) *= 1e-3;

    float optimal_thrust = current_trajectory.trajectory[prev_point].thrust +  ratio* (current_trajectory.trajectory[prev_point+1].thrust - current_trajectory.trajectory[prev_point].thrust);

    mpc.ocp().us << 0.0, 0.0, 
                    (2*optimal_thrust - rocket.maxThrust[2]- rocket.minThrust[2])/(rocket.maxThrust[2]- rocket.minThrust[2]),
                    0.0;
  }
  // If asked for a time before first point, give first point
  else if (prev_point <0)
  {
    //res.target_point = trajectory[1];
  }
  // If asked for a time after first point, give last point
  else
  {
    //res.target_point = trajectory[N_POINT -1];
  }
}

void init_weight(ros::NodeHandle n)
{
  std::vector<double> r(4);
  std::vector<double> q(14);

  n.getParam("/mpc/R", r);
  n.getParam("/mpc/Q", q);

  mpc.ocp().R.diagonal() << r[0], r[1], r[2], r[3];

  mpc.ocp().Q.diagonal() << q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8], q[9], q[10], q[11], q[12], q[13];

  mpc.ocp().QN = mpc.ocp().Q;

  n.getParam("/mpc/theta", mpc.ocp().weight_vertical_angle);
}




int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
 
	// Create control publisher
	ros::Publisher control_pub = n.advertise<real_time_simulator::Control>("control_pub", 1);

  // Create path publisher
  ros::Publisher MPC_horizon_pub = n.advertise<real_time_simulator::Trajectory>("mpc_horizon", 10);

	// Subscribe to state message from generic_gnc
  ros::Subscriber rocket_state_sub = n.subscribe("kalman_rocket_state", 100, rocket_stateCallback);

  // Subscribe to measured control for MPC initialization
	ros::Subscriber control_sub = n.subscribe("control_measured", 100, controlCallback);

  // Subscribe to guidance reference trajectory
	ros::Subscriber guidance_sub = n.subscribe("target_trajectory", 100, guidanceCallback);

  // Subscribe to fsm and time from time_keeper
  ros::Subscriber fsm_sub = n.subscribe("gnc_fsm_pub", 100, fsm_Callback);

	// Setup Time_keeper client and srv variable for FSM and time synchronization
	ros::ServiceClient client_fsm = n.serviceClient<real_time_simulator::GetFSM>("getFSM_gnc");
  real_time_simulator::GetFSM srv_fsm;

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

  // Initialize trajectory
  int i;
  real_time_simulator::Waypoint waypoint0;
  for(i = 0; i<mpc.ocp().NUM_NODES; i++)
  {
    current_trajectory.trajectory.push_back(waypoint0);
  }
	
  // Initialize rocket class with useful parameters
  rocket.init(n);

  // Init MPC ----------------------------------------------------------------------------------------------------------------------
	
  mpc.settings().max_iter = 2; 
  mpc.settings().line_search_max_iter = 10;
  //mpc.qp_settings().max_iter = 25;
  //mpc.m_solver.settings().max_iter = 1000;
  //mpc.m_solver.settings().scaling = 10;

  // Cost weight
  init_weight(n);

  mpc.set_time_limits(0, CONTROL_HORIZON);


  // Input constraints
  const double inf = std::numeric_limits<double>::infinity();
  mpc_t::control_t lbu; 
  mpc_t::control_t ubu;
  mpc_t::control_t u0; 
  
  lbu << -1, -1, -1, -1; // lower bound on control
  ubu << 1, 1, 1, 1; // upper bound on control
  u0 <<  0, 0, 1, 0; // Ideal control for init
  
  //lbu << -inf, -inf, -inf, -inf;
  //ubu <<  inf,  inf,  inf,  inf;
  mpc.control_bounds(lbu, ubu); 

  // Initial control
  mpc.u_guess(u0.replicate(mpc.ocp().NUM_NODES,1));

  // State constraints
  const double eps = 1e-1;
  mpc_t::state_t lbx; 
  mpc_t::state_t ubx; 
  
  lbx << -inf, -inf, 0,     -inf, -inf, 0-eps,     -0.183-eps, -0.183-eps, -0.183-eps, -1-eps,   -inf, -inf, -inf,     0-eps;
  ubx <<  inf,  inf, inf,    inf,  inf, 330+eps,    0.183+eps,  0.183+eps,  0.183+eps,  1+eps,    inf,  inf,  inf,     rocket.propellant_mass+eps;
  
  //lbx << -inf, -inf, -inf,   -inf, -inf, -inf,   -inf, -inf, -inf, -inf,   -inf, -inf, -inf,     -inf;
  //ubx << inf,  inf, inf,     inf,  inf, inf,    inf,  inf,  inf,  inf,     inf,  inf,  inf,      inf;
  mpc.state_bounds(lbx, ubx);

  // Initial state
  mpc_t::state_t x0;
  x0 << 0, 0, 0,
        0, 0, 0,
        0, 0, 0, 1, 
        0, 0, 0,
        rocket.propellant_mass;
  mpc.x_guess(x0.replicate(14,1));	

  // Variables to track performance over whole simulation
	std::vector<float> average_time;
  std::vector<int> average_status;
   
  // Init default control to zero
  real_time_simulator::Control control_law;

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.050), [&](const ros::TimerEvent&) 
	{
    //Get current FSM and time 
    if(client_fsm.call(srv_fsm))
    {
      current_fsm = srv_fsm.response.fsm;
    }
   
    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}
 
    else if (current_fsm.state_machine.compare("Coast") != 0)
    {

      if (current_fsm.state_machine.compare("Rail") == 0)// || current_fsm.state_machine.compare("Launch") == 0)
      {
        control_law.force.z = rocket.maxThrust[2];
      }     
 
      else if (current_fsm.state_machine.compare("Launch") == 0)
			{
        // Initialize current state and target state
        x0 <<   current_state.pose.position.x/1000, current_state.pose.position.y/1000, current_state.pose.position.z/1000,
                current_state.twist.linear.x/1000, current_state.twist.linear.y/1000, current_state.twist.linear.z/1000,
                current_state.pose.orientation.x, current_state.pose.orientation.y, current_state.pose.orientation.z, current_state.pose.orientation.w, 
                current_state.twist.angular.x, current_state.twist.angular.y, current_state.twist.angular.z,
                current_state.propeller_mass;

        mpc.initial_conditions(x0);
        rocket.update_CM(x0(13));


        trajectory_interpolate();

        // Solve problem and save solution
        double time_now = ros::Time::now().toSec();
        mpc.solve();
        time_now = 1000*(ros::Time::now().toSec()-time_now);

        ROS_INFO("Ctr T= %.2f ms, st: %d, iter: %d, end time: %.2f", time_now , mpc.info().status.value,  mpc.info().iter, mpc.time_grid[mpc.ocp().NUM_NODES-1]);
        //average_status.push_back(mpc.info().status.value);
        //average_time.push_back(time_now);

        // Get state and control solution
        Eigen::Matrix<double, 4, 1> control_MPC;
        control_MPC =  mpc.solution_u_at(0);

        Eigen::Matrix<double, 4,1> input = convertControl_SI(control_MPC);
        //ROS_INFO("Fx: %f, Fy: %f, Fz: %f, Mx: %f \n",  input[0], input[1], input[2], input[3]);

        // Apply MPC control
        control_law.force.x = input[0];
        control_law.force.y = input[1];
        control_law.force.z = input[2];

        control_law.torque.x = control_law.force.y*rocket.total_CM; 
        control_law.torque.y = -control_law.force.x*rocket.total_CM; 
        control_law.torque.z = input[3];

        //control_law.force.z = current_trajectory.trajectory[0].thrust;

        //control_law.force.z = rocket.maxThrust[2];

        // Send optimal trajectory computed by control. Send only position for now
        real_time_simulator::Trajectory trajectory_msg;
        for(int i = 0; i<mpc.ocp().NUM_NODES ;i++){
          real_time_simulator::Waypoint point;
          point.position.x = 1000*mpc.solution_x_at(i)[0];
          point.position.y = 1000*mpc.solution_x_at(i)[1];
          point.position.z = 1000*mpc.solution_x_at(i)[2];
          trajectory_msg.trajectory.push_back(point);
        }

        MPC_horizon_pub.publish(trajectory_msg);
      }
    
    //Last check in case fsm changed during control computation 
    if(client_fsm.call(srv_fsm))
    {
      current_fsm = srv_fsm.response.fsm;
      if (current_fsm.state_machine.compare("Coast") != 0)
      {
          control_pub.publish(control_law);
      }
    }
       
    }
  });
 
	// Automatic callback of service and publisher from here
	ros::spin(); 
}
