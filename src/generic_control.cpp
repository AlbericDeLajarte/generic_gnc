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

#define CONTROL_HORIZON 30 // In seconds

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
 
real_time_simulator::Control passive_control();

using admm = boxADMM<guidance_ocp::VAR_SIZE, guidance_ocp::NUM_EQ, guidance_ocp::scalar_t,
                guidance_ocp::MATRIXFMT, linear_solver_traits<guidance_ocp::MATRIXFMT>::default_solver>;
                
//using osqp_solver_t = polympc::OSQP<control_ocp::VAR_SIZE, control_ocp::NUM_EQ, control_ocp::scalar_t>;


// Creates solver
using mpc_t = MPC<guidance_ocp, MySolver, admm>;
mpc_t mpc;
 
 
// Global variable with last received rocket state
real_time_simulator::State current_state;

// Global variable with last received control
real_time_simulator::Control current_control;

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

void fsm_Callback(const real_time_simulator::FSM::ConstPtr& fsm)
{
  current_fsm.state_machine = fsm->state_machine;
  current_fsm.time_now = fsm->time_now;
}


int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "control");
  ros::NodeHandle n;
 
	// Create control publisher
	ros::Publisher control_pub = n.advertise<real_time_simulator::Control>("control_pub", 1);

	// Subscribe to state message from generic_gnc
  ros::Subscriber rocket_state_sub = n.subscribe("kalman_rocket_state", 100, rocket_stateCallback);

  // Subscribe to measured control for MPC initialization
	ros::Subscriber control_sub = n.subscribe("control_measured", 100, controlCallback);

  // Subscribe to fsm and time from time_keeper
  ros::Subscriber fsm_sub = n.subscribe("gnc_fsm_pub", 100, fsm_Callback);

	// Setup Time_keeper client and srv variable for FSM and time synchronization
	ros::ServiceClient client_fsm = n.serviceClient<real_time_simulator::GetFSM>("getFSM_gnc");
  real_time_simulator::GetFSM srv_fsm;

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";
	
  // Initialize rocket class with useful parameters
  rocket.init(n);

  // Init MPC ----------------------------------------------------------------------------------------------------------------------
	
  mpc.settings().max_iter = 10; 
  mpc.settings().line_search_max_iter = 10;
  //mpc.m_solver.settings().max_iter = 1000;
  //mpc.m_solver.settings().scaling = 10;


  // Input constraints and initialisation -------------
  const double inf = std::numeric_limits<double>::infinity();
  mpc_t::control_t lbu; 
  mpc_t::control_t ubu; 

  lbu << -3000; // lower bound on control
	ubu << 0; // upper bound on control
  mpc.control_bounds(lbu, ubu);  

  // Initial control
  mpc.u_guess(ubu.replicate(mpc.ocp().NUM_NODES,1));

  // State constraints and initialisation ---------------
  const double eps = 1e-1;
  mpc_t::state_t lbx; 
  mpc_t::state_t ubx; 
  
  lbx << -inf, -inf, 0,                           -inf, -inf, -inf,      0-eps,                      rocket.minThrust[2]-eps;
  ubx <<  inf,  inf, rocket.target_apogee[2],      inf,  inf, 330+eps,   rocket.propellant_mass+eps, rocket.maxThrust[2];
  
  // lbx << -inf, -inf, -inf,   -inf, -inf, -inf,   -inf;
  // ubx <<  inf,  inf, inf,     inf,  inf, inf,     inf;
  mpc.state_bounds(lbx, ubx);
   
  // Final state
  mpc_t::state_t lbx_f; lbx_f << -inf, -inf, rocket.target_apogee[2],     -inf, -inf, 0-1,   0-eps,                     -rocket.minThrust[2]-eps; // lower bound on final state
  mpc_t::state_t ubx_f; ubx_f <<  inf,  inf, rocket.target_apogee[2]+50,   inf,  inf, 0+1,   rocket.propellant_mass+eps, rocket.maxThrust[2]; // upper bound on final state
  mpc.final_state_bounds(lbx_f, ubx_f);

  // Initial state
  mpc_t::state_t x0_inf, x0_sup;  
  x0_inf << 0, 0, 0,
            0, 0, 0,
            rocket.propellant_mass,
            rocket.minThrust[2]; 
  x0_sup = x0_inf;
  x0_sup(7) = rocket.maxThrust[2];
  
  mpc.x_guess(x0_sup.replicate(mpc.ocp().NUM_NODES,1));	
 
  // Parameters
  mpc_t::parameter_t lbp; lbp << 0.0;                 // lower bound on time
  mpc_t::parameter_t ubp; ubp << CONTROL_HORIZON;     // upper bound on time
  mpc_t::parameter_t p0; p0 << CONTROL_HORIZON/1.5;   // very important to set initial time estimate

  mpc.parameters_bounds(lbp, ubp);
  mpc.p_guess(p0);
  
  // Init default control to zero
  real_time_simulator::Control control_law;

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.200), [&](const ros::TimerEvent&) 
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
        control_law.force.z = 2000;
      }    
 
      else if (current_fsm.state_machine.compare("Launch") == 0)
			{
        control_law.force.z = 2000;
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
