/*
* Node to send control commands to the rocket engine. 
* Can also be used by the simulation for SIL and PIL tests.
*
* Inputs: 
*   - Finite state machine from the basic_gnc package:	\gnc_fsm_pub
*   - Estimated state from basic_navigation:		        \kalman_rocket_state
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

real_time_simulator::Trajectory create_trajectory()
{
  real_time_simulator::Trajectory trajectory_msg;    
  for(int i=0; i< mpc.ocp().NUM_NODES; i++)
  {
    mpc_t::state_t guidance_point; guidance_point = mpc.solution_x_at(i);
    real_time_simulator::Waypoint waypoint;

    waypoint.position.x = guidance_point(0);
    waypoint.position.y = guidance_point(1);
    waypoint.position.z = guidance_point(2);

    waypoint.speed.x = guidance_point(3);
    waypoint.speed.y = guidance_point(4);
    waypoint.speed.z = guidance_point(5);

    waypoint.propeller_mass = guidance_point(6);

    waypoint.time = current_fsm.time_now + mpc.solution_p()[0]*mpc.time_grid(i);

    waypoint.thrust = guidance_point(7); 


    trajectory_msg.trajectory.push_back(waypoint);
  } 

  return trajectory_msg;
}



int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "guidance");
  ros::NodeHandle n;

  // Create waypoint trajectory publisher
	ros::Publisher target_trajectory_pub = n.advertise<real_time_simulator::Trajectory>("target_trajectory", 10);

	// Subscribe to state message from basic_gnc
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

  lbu << -1000; // lower bound on control
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

  // Solve first guidance for intialization, and send it
  mpc.initial_conditions(x0_inf, x0_sup); 
  mpc.solve(); 
  target_trajectory_pub.publish(create_trajectory()); 
  
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
        control_law.force.z = rocket.maxThrust[2];
      }    
 
      else if (current_fsm.state_machine.compare("Launch") == 0)
			{

        mpc.ocp().cos_vertical_angle =  current_state.pose.orientation.w*current_state.pose.orientation.w 
                                      - current_state.pose.orientation.x*current_state.pose.orientation.x
                                      - current_state.pose.orientation.y*current_state.pose.orientation.y
                                      + current_state.pose.orientation.z*current_state.pose.orientation.z;

        x0_inf <<   current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z,
                    current_state.twist.linear.x, current_state.twist.linear.y, current_state.twist.linear.z,
                    current_state.propeller_mass,
                    rocket.minThrust[2];
        x0_sup = x0_inf;
        x0_sup(7) = current_control.force.z;

        mpc.initial_conditions(x0_inf, x0_sup); 
        //mpc.x_guess(x0_sup.replicate(mpc.ocp().NUM_NODES,1));	

        // Solve problem and save solution 
        double time_now = ros::Time::now().toSec(); 
        mpc.solve(); 
        double dT = ros::Time::now().toSec()-time_now;
        ROS_INFO("Gdc T= %.2f ms, st: %d, iter: %d",  1000*dT, mpc.info().status.value,  mpc.info().iter);
        

        double body_thrust = mpc.solution_x_at(0)[7];
        body_thrust = std::min(body_thrust, (double)rocket.maxThrust[2]);
        body_thrust = std::max(body_thrust, (double)rocket.minThrust[2]);

        float predicted_apogee = rocket.check_apogee(x0_inf(6)+rocket.dry_mass, x0_inf(2), x0_inf(5));
        if(predicted_apogee>rocket.target_apogee[2])
        {
          body_thrust = 0.0;
        }

        std::cout << "Thrust: " << body_thrust << "N | Apogee: " << predicted_apogee << "m | Time: " << mpc.solution_p() << "s\n\n";

        if( (body_thrust == 0.0 && predicted_apogee > rocket.target_apogee[2])
          ||((int)body_thrust != 0 && predicted_apogee < rocket.target_apogee[2]) )
        {

          if(!isnan(body_thrust) && !isinf(body_thrust))
          {
            // Send full optimal state as waypoint trajectory     
            target_trajectory_pub.publish(create_trajectory()); 

            control_law.force.z = body_thrust;
          }
        }
      }
       
    }
  });
 
	// Automatic callback of service and publisher from here
	ros::spin(); 
}
