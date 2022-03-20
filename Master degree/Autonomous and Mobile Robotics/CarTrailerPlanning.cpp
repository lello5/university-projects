#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Planner.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <iostream>
#include <valarray>
#include <fstream>
#include <limits>

#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

namespace ob = ompl::base;
// assuming we do the control planning
namespace oc = ompl::control;
namespace po = boost::program_options;

// constant values of the robot structure
const double carLength_ = 0.25;
const double carWidth_ = 0.125;
const double m1_ = 0.07;
const double trailerLength_ = 0.26;
const double trailerWidth_ = 0.125;


// definition of the ODE for the car-trailer.
void CarTrailerODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
	// extracting control inputs u = [v w], with v = driving vel. and w = steering vel.
	const double v = 0.25 * control->as<oc::CompoundControlSpace::ControlType>()[0].as<oc::DiscreteControlSpace::ControlType>(0)->value;
	const double w = 0.2 * control->as<oc::CompoundControlSpace::ControlType>()[0].as<oc::DiscreteControlSpace::ControlType>(1)->value;

	// extracting the parameters required in the KM
	const double theta = q[2];
	const double psi = q[3];
	const double phi = q[4];

	// Zero out qdot
	qdot.resize(q.size (), 0);

	// KM of the car-trailer of Equation 1 from "Anti-Jackknifing Control of Tractor-Trailer Vehicles
	// via Intrinsically Stable MPC" by Beglini et al.
	qdot[0] = v * cos(theta);
	qdot[1] = v * sin(theta);
	qdot[2] = v * tan(phi) / carLength_;
	qdot[3] = - v * tan(phi) / carLength_ * (1 + m1_ * cos(psi) / trailerLength_) - v * sin(psi) / trailerLength_;
	qdot[4] = w;
}

// definition of the ODE for the car-trailer with a Control-Loop (CL).
void CarTrailerODE_CL(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
{
	// extracting control inputs u = [v phi_des], with v = driving vel. and phi_des = desired steering angle
	const double v = 0.25 * control->as<oc::CompoundControlSpace::ControlType>()[0].as<oc::DiscreteControlSpace::ControlType>(0)->value;
	// phi_des should be scaled s.t. it is in [-25°, +25°] but remember that the control input takes integer values in [-5, +5] rad/s
	// => a conversion from deg to rad would ease the scaling, taking into account the "displacement" of 5° between primitives
	const double phi_des_deg = 5 * control->as<oc::CompoundControlSpace::ControlType>()[0].as<oc::DiscreteControlSpace::ControlType>(1)->value;
	const double phi_des = phi_des_deg * boost::math::constants::pi<double>() / 180.;
	
	// extracting the parameters required in the KM
	const double theta = q[2];
	const double psi = q[3];
	const double phi = q[4];

	// Zero out qdot
	qdot.resize(q.size (), 0);
	
	// defining the required gains in the CL:
	// - gain for the regulation of the reference steering angle by means of steering velocity
	const double K_reg = 2.;
	// - gain for the "stabilizing" control action (or how much do we have to steer to counter the divergence of the hitch angle)
	double K_stab = 1.;
	
	// remember that there is no need to stabilize during forward motion
	if (v > 0.) K_stab = 0.;
	
	// phi_ref = phi_des - K_stab * psi
	double phi_ref = phi_des - K_stab * psi;
	
	// phi_dot = w = K_reg * (phi_ref - phi)
	const double w = K_reg * (phi_ref - phi);
	
	// KM of the car-trailer of Equation 1 from "Anti-Jackknifing Control of Tractor-Trailer Vehicles
	// via Intrinsically Stable MPC" by Beglini et al.
	qdot[0] = v * cos(theta);
	qdot[1] = v * sin(theta);
	qdot[2] = v * tan(phi) / carLength_;
	qdot[3] = - v * tan(phi) / carLength_ * (1 + m1_ * cos(psi) / trailerLength_) - v * sin(psi) / trailerLength_;
	qdot[4] = w;
}

// this is a callback method invoked after numerical integration
void CarTrailerPostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
	// normalize orientation between 0 and 2*pi for:
	// - theta
	ob::SO2StateSpace SO2;
	SO2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SE2StateSpace::StateType>(0)->as<ob::SO2StateSpace::StateType>(1));
	// - psi
	ob::SO2StateSpace SO2_1;
	SO2_1.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SO2StateSpace::StateType>(1));
	// - phi
	ob::SO2StateSpace SO2_2;
	SO2_2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SO2StateSpace::StateType>(2));
}

bool jackknifing(const double psi)
{
	// we ASSUME that we are in the jackknifing situation if |psi| > pi/4 = 45°
	return fabs(psi) > boost::math::constants::pi<double>()/4;
}

// returns the distance from the given state's position to the boundary of the circular obstacle
double clearance(const ob::State* state, double x_obs, double y_obs, double radius, int cp_num)
{
    // downcast state into the specific type
    const ob::RealVectorStateSpace::StateType* state2D =
        state->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SE2StateSpace::StateType>(0)->as<ob::RealVectorStateSpace::StateType>(0);
    // extract the robot's (x,y) position from its state
    double x = state2D->values[0];
    double y = state2D->values[1];

    // extract theta
    const ob::SO2StateSpace::StateType* SO2 =
		state->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SE2StateSpace::StateType>(0)->as<ob::SO2StateSpace::StateType>(1);
    double theta = SO2->value;

    // extract psi
	const ob::SO2StateSpace::StateType* SO2_1 = state->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SO2StateSpace::StateType>(1);
    const double psi = SO2_1->value;

	double d;
    // clearance computation between a specific control points and a given circular obstacle
    switch (cp_num)
    {
        case 1:
        {	
            double x1 = x - carWidth_ / 2 * cos(boost::math::constants::pi<double>()/2 - theta);
            double y1 = y + carWidth_ / 2 * sin(boost::math::constants::pi<double>()/2 - theta);
            d = sqrt((x1-x_obs)*(x1-x_obs) + (y1-y_obs)*(y1-y_obs)) - radius;
            break;
        }
        case 2:
        {
            double x2 = x - carWidth_ / 2 * cos(boost::math::constants::pi<double>()/2 - theta) + carLength_ / 2 * cos(theta);
            double y2 = y + carWidth_ / 2 * sin(boost::math::constants::pi<double>()/2 - theta) + carLength_ / 2 * sin(theta);
            d = sqrt((x2-x_obs)*(x2-x_obs) + (y2-y_obs)*(y2-y_obs)) - radius;
            break;
        }
        case 3:
        {
			double x3 = x - carWidth_ / 2 * cos(boost::math::constants::pi<double>()/2 - theta) + carLength_ * cos(theta);
            double y3 = y + carWidth_ / 2 * sin(boost::math::constants::pi<double>()/2 - theta) + carLength_ * sin(theta);
            d = sqrt((x3-x_obs)*(x3-x_obs) + (y3-y_obs)*(y3-y_obs)) - radius;
            break;
        }
        case 4:
        {
            double x4 = x + carLength_ * cos(theta);
            double y4 = y + carLength_ * sin(theta);
            d = sqrt((x4-x_obs)*(x4-x_obs) + (y4-y_obs)*(y4-y_obs)) - radius;
            break;
        }
        case 5:
        {
            double x5 = x + carWidth_ / 2 * cos(boost::math::constants::pi<double>()/2 - theta) + carLength_ * cos(theta);
            double y5 = y - carWidth_ / 2 * sin(boost::math::constants::pi<double>()/2 - theta) + carLength_ * sin(theta);
            d = sqrt((x5-x_obs)*(x5-x_obs) + (y5-y_obs)*(y5-y_obs)) - radius;
            break;
        }
        case 6:
        {
            double x6 = x + carWidth_ / 2 * cos(boost::math::constants::pi<double>()/2 - theta) + carLength_ / 2 * cos(theta);
            double y6 = y - carWidth_ / 2 * sin(boost::math::constants::pi<double>()/2 - theta) + carLength_ / 2 * sin(theta);
            d = sqrt((x6-x_obs)*(x6-x_obs) + (y6-y_obs)*(y6-y_obs)) - radius;
            break;
        }
        case 7:
        {	
            double x7 = x + carWidth_ / 2 * cos(boost::math::constants::pi<double>()/2 - theta);
            double y7 = y - carWidth_ / 2 * sin(boost::math::constants::pi<double>()/2 - theta);
            d = sqrt((x7-x_obs)*(x7-x_obs) + (y7-y_obs)*(y7-y_obs)) - radius;
            break;
        }
        case 8:
        {
            double x8 = x - m1_ * cos(theta) + trailerWidth_/ 2 * cos(boost::math::constants::pi<double>()/2-psi-theta);
            double y8 = y - m1_ * sin(theta) - trailerWidth_ / 2 * sin(boost::math::constants::pi<double>()/2-psi-theta);
            d = sqrt((x8-x_obs)*(x8-x_obs) + (y8-y_obs)*(y8-y_obs)) - radius;
            break;
        }
        case 9:
        {
            double x9 = x - m1_ * cos(theta) - trailerLength_ / 2 * cos(psi+theta) + trailerWidth_ / 2 * cos(boost::math::constants::pi<double>()/2-psi-theta);
            double y9 = y - m1_ * sin(theta) - trailerLength_ / 2 * sin(psi+theta) - trailerWidth_ / 2 * sin(boost::math::constants::pi<double>()/2-psi-theta);
            d = sqrt((x9-x_obs)*(x9-x_obs) + (y9-y_obs)*(y9-y_obs)) - radius;
            break;
        }
        case 10:
        {
            double x10 = x - m1_ * cos(theta) - trailerLength_ * cos(psi+theta) + trailerWidth_ / 2 * cos(boost::math::constants::pi<double>()/2-psi-theta);
            double y10 = y - m1_ * sin(theta) - trailerLength_ * sin(psi+theta) - trailerWidth_ / 2 * sin(boost::math::constants::pi<double>()/2-psi-theta);
            d = sqrt((x10-x_obs)*(x10-x_obs) + (y10-y_obs)*(y10-y_obs)) - radius;
            break;
        }
        case 11:
        {
            double x11 = x - m1_ * cos(theta) - trailerLength_ * cos(psi+theta);
            double y11 = y - m1_ * sin(theta) - trailerLength_ * sin(psi+theta);
            d = sqrt((x11-x_obs)*(x11-x_obs) + (y11-y_obs)*(y11-y_obs)) - radius;
            break;
        }
        case 12:
        {
            double x12 = x - m1_ * cos(theta) - trailerLength_ * cos(psi+theta) - trailerWidth_ / 2 * cos(boost::math::constants::pi<double>()/2-psi-theta);
            double y12 = y - m1_ * sin(theta) - trailerLength_ * sin(psi+theta) + trailerWidth_ / 2 * sin(boost::math::constants::pi<double>()/2-psi-theta);
            d = sqrt((x12-x_obs)*(x12-x_obs) + (y12-y_obs)*(y12-y_obs)) - radius;
            break;
        }
        case 13:
        {
            double x13 = x - m1_ * cos(theta) - trailerLength_ / 2 * cos(psi+theta) - trailerWidth_ / 2 * cos(boost::math::constants::pi<double>()/2-psi-theta);
            double y13 = y - m1_ * sin(theta) - trailerLength_ / 2 * sin(psi+theta) + trailerWidth_ / 2 * sin(boost::math::constants::pi<double>()/2-psi-theta);
            d = sqrt((x13-x_obs)*(x13-x_obs) + (y13-y_obs)*(y13-y_obs)) - radius;
            break;
        }
        case 14:
        {
			double x14 = x - m1_ * cos(theta) - trailerWidth_ / 2 * cos(boost::math::constants::pi<double>()/2-psi-theta);
            double y14 = y - m1_ * sin(theta) + trailerWidth_ / 2 * sin(boost::math::constants::pi<double>()/2-psi-theta);
            d = sqrt((x14-x_obs)*(x14-x_obs) + (y14-y_obs)*(y14-y_obs)) - radius;
            break;
        }
        case 15:
        {
            double x15 = x - m1_ * cos(theta);
            double y15 = y - m1_ * sin(theta);
            d = sqrt((x15-x_obs)*(x15-x_obs) + (y15-y_obs)*(y15-y_obs)) - radius;
            break;
        }
        default:
        {
        	// distance formula between two points, offset by the circle's radius
    		d = sqrt((x-x_obs)*(x-x_obs) + (y-y_obs)*(y-y_obs)) - radius;
    		break;
        }
    }
    return d;    
}

// check if any of the cp_num = 16 control points is in collision with a given obstacles
bool check_obstacle(const ob::State* state, double x_obs, double y_obs, double radius)
{
	for (int cp_num = 0; cp_num < 16; cp_num++) {
		if (clearance(state, x_obs, y_obs, radius, cp_num) < 0.05) {
			return false;
		}
	}
	return true;
}

// return the minimum clearance between any of the cp_num = 16 control points and a given obstacles
double compute_clearance_for_obstacle(const ob::State* state, double x_obs, double y_obs, double radius)
{
    double min  = clearance(state, x_obs, y_obs, radius, 0);
    for (int cp_num = 1; cp_num < 16; cp_num++) {
        if (clearance(state, x_obs, y_obs, radius, cp_num) < min) {
            min = clearance(state, x_obs, y_obs, radius, cp_num);
        }
    }
    return min;
}

// return the minimum clearance of a given experiment
double compute_clearance(const ob::State* state, const int obstacles)
{
	double d;
	switch(obstacles)
	{
		// case 1: circle obstacles
		case 1:
		{
			// 6 circular obstacles
			double obs_clearances[6];
			obs_clearances[0] = compute_clearance_for_obstacle(state, 0.0, 0.0, 1.0);
			obs_clearances[1] = compute_clearance_for_obstacle(state, -1.5, 2.0, 0.5);
			obs_clearances[2] = compute_clearance_for_obstacle(state, 1.0, -2.0, 0.25);
			obs_clearances[3] = compute_clearance_for_obstacle(state, -2.0, -1.0, 0.75);
			obs_clearances[4] = compute_clearance_for_obstacle(state, 2.2, 1.0, 0.3);
			obs_clearances[5] = compute_clearance_for_obstacle(state, -2.1, 0.6, 0.7);  
			double min = obs_clearances[0];
			for (int i = 1; i < 6; i++) {
				if (obs_clearances[i] < min) {
					min = obs_clearances[i]; 
				}
			}
			d = min;
			break;
		}

		// case 2: three point turn
		case 2:
		{
			double min = 10., c = 10.;
			// brutal way for computing clearance: the obstacles are modeled
			// as control points along the perimeter of the environment
			double x_obs = 0., y_obs = 0.;
			for (int i = 0; i <= 24; ++i){
				x_obs = i/8.;
				if (x_obs == 0. || x_obs == 3.){
					for (int j = 0; j <= 4; ++j){
						y_obs = j/8.;
						c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
						if (c < min) min = c;
					}
				}
				else if ((x_obs > 0 && x_obs < 1.125) || (x_obs > 1.875 && x_obs < 3)){
					y_obs = 0.;
					c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
					if (c < min) min = c;
					y_obs = 0.5;
					c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
					if (c < min) min = c;
				}
				else if (x_obs == 1.125 || x_obs == 1.875){
					y_obs = 0.;
					c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
					if (c < min) min = c;
					
					for (int j = 4; j <= 8; ++j){
						y_obs = j/8.;				
						c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
						if (c < min) min = c;
					}
				}
				else {
					y_obs = 0.;
					c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
					if (c < min) min = c;
					y_obs = 1.;
					c = compute_clearance_for_obstacle(state, x_obs, y_obs, 0.);
					if (c < min) min = c;
				}
			}
			d = min;
			break;
		}

		// case 3: real parking
		case 3:
		{
			// simplification: rectangular obstacles modeled as a set of circles
			double obs_clearances[10];
			obs_clearances[0] = compute_clearance_for_obstacle(state, 0.21, 0.9, 0.21);
			obs_clearances[1] = compute_clearance_for_obstacle(state, 0.63, 0.9, 0.21);
			obs_clearances[2] = compute_clearance_for_obstacle(state, 1.05, 0.9, 0.21);
			obs_clearances[3] = compute_clearance_for_obstacle(state, 1.95, 0.9, 0.21);
			obs_clearances[4] = compute_clearance_for_obstacle(state, 2.37, 0.9, 0.21);
			obs_clearances[5] = compute_clearance_for_obstacle(state, 2.79, 0.9, 0.21);
			obs_clearances[6] = compute_clearance_for_obstacle(state, 0.75, 0.5, 0.5);
			obs_clearances[7] = compute_clearance_for_obstacle(state, 2.25, 0.5, 0.5);
			obs_clearances[8] = compute_clearance_for_obstacle(state, 1.1, 0.2, 0.21);
			obs_clearances[9] = compute_clearance_for_obstacle(state, 1.9, 0.2, 0.21);
			double min = obs_clearances[0];
			for (int i = 1; i < 10; i++) {
				if (obs_clearances[i] < min) {
					min = obs_clearances[i]; 
				}
			}
			d = min;
			break;
		}

		default:
		{
			std::cout << "Invalid clearance definition.\n" << std::endl;
			exit(1);
		}
	}
	return d;
}

// whether the given state's position avoids overlapping with some circular obstacles
// any states lying in this circular region are considered "in collision"
bool collision_check(const ob::State* state, const int obstacles)
{
	switch(obstacles)
	{
		// case 1: circle obstacles
		case 1:
		{
			// 6 circular obstacles:
			// 		centered at (0.0, 0.0) of radius 1.0
			// 		centered at (-1.5, 2.0) of radius 0.5
			// 		centered at (1.0, -2.0) of radius 0.25
			// 		centered at (-2.0, -1.0) of radius 0.75
			// 		centered at (2.2, 1.0) of radius 0.3
			//      centered at (-2.1, 0.6) of radius 0.7
			return check_obstacle(state, 0.0, 0.0, 1.0)
				   && check_obstacle(state, -1.5, 2.0, 0.5)
				   && check_obstacle(state, 1.0, -2.0, 0.25)
				   && check_obstacle(state, -2.0, -1.0, 0.75)
				   && check_obstacle(state, 2.2, 1.0, 0.3)
                   && check_obstacle(state, -2.1, 0.6, 0.7);
		}

		// case 2: three point turn
		case 2:
		{
			// brutal way for collision checking: the obstacles are modeled
			// as control points along the perimeter of the environment;
			// these points are very close to each other such that the
			// probability to have an undetected collision is extremely low
			double x_obs = 0., y_obs = 0.;
			for (int i = 0; i <= 24; ++i){
				x_obs = i/8.;
				if (x_obs == 0. || x_obs == 3.){
					for (int j = 0; j <= 4; ++j){
						y_obs = j/8.;
						if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
					}
				}
				else if ((x_obs > 0 && x_obs < 1.125) || (x_obs > 1.875 && x_obs < 3)){
					y_obs = 0.;
					if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
					y_obs = 0.5;
					if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
				}
				else if (x_obs == 1.125 || x_obs == 1.875){
					y_obs = 0.;
					if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
					
					for (int j = 4; j <= 8; ++j){
						y_obs = j/8.;
						
						if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
					}
				}
				else {
					y_obs = 0.;
					if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
					y_obs = 1.;
					if (!check_obstacle(state, x_obs, y_obs, 0.)) return false;
				}
			}
			
			return true;
		}

		// case 3: real parking
		case 3:
		{
			// simplification: rectangular obstacles modeled as a set of circles
			return check_obstacle(state, 0.21, 0.9, 0.21)
				   && check_obstacle(state, 0.63, 0.9, 0.21)
				   && check_obstacle(state, 1.05, 0.9, 0.21)

				   && check_obstacle(state, 1.95, 0.9, 0.21)
				   && check_obstacle(state, 2.37, 0.9, 0.21)
				   && check_obstacle(state, 2.79, 0.9, 0.21)

				   && check_obstacle(state, 0.75, 0.5, 0.5)
				   && check_obstacle(state, 2.25, 0.5, 0.5)
				   
				   && check_obstacle(state, 1.1, 0.2, 0.21)
				   && check_obstacle(state, 1.9, 0.2, 0.21);
		}

		default:
		{
			std::cout << "Invalid collision check.\n" << std::endl;
			exit(1);
		}
	}
	return true;
}

// state validity checking function
// obstacles is a flag which represents if we want to plan with obstacles or in the free environment.
// in particular:
// 		- 0, for forward and backward
// 		- 1, for obstacles
// 		- 2, for turn
// 		- 3, for real
bool isStateValid(const ob::SpaceInformation *si, const ob::State *state, const int obstacles)
{
	// extracting psi to handle jackknifing
	const auto *so2state1 = state->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SO2StateSpace::StateType>(1);
    const double psi = so2state1->value;
    
    // extracting phi as well
    const auto *so2state2 = state->as<ob::CompoundStateSpace::StateType>()[0].as<ob::SO2StateSpace::StateType>(2);
	const double phi = so2state2->value;
	
	// we know that in order to render a realistic motion of the front wheels of 
	// the car, phi should be bounded (let's say that |phi| < pi/6 = 30°)
	const double real_motion = fabs(phi) < boost::math::constants::pi<double>() / 6.;

	// if planning with obstacles is desired, calls collision checking, otherwise is true (no collisions for free environment)
	auto avoid_obstacles = obstacles ? collision_check(state, obstacles) : true;
									
	return si->satisfiesBounds(state) && real_motion && (!jackknifing(psi)) && avoid_obstacles;
}

// utility for stack of planner in benchmarking
void addPlanner(ompl::tools::Benchmark& benchmark, const ob::PlannerPtr& planner, double goal_bias)
{
     ob::ParamSet& params = planner->params();
     if (params.hasParam(std::string("goal_bias")))
     	params.setParam(std::string("goal_bias"), ompl::toString(goal_bias));
     benchmark.addPlanner(planner);
}

// ompl::base::StateCostIntegralObjective represents objectives as summations of state costs
// we need to inherit from it and specify our state cost function by overriding the ompl::base::OptimizationObjective::stateCost() method
class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
	
	int obstacles;
    
    ClearanceObjective(const ob::SpaceInformationPtr& si, const int obs):
    // true enables to use motion cost interpolation when summing up state costs along the path
    // by default, ompl::base::StateCostIntegralObjective would simply take the individual states that make up a given path, and sum up those costs
    // the interpolation of states along a path is the same as the one used in ompl::base::DiscreteMotionValidator
    ob::StateCostIntegralObjective(si, true)
    {
		obstacles = obs;
    }
  
    // we want to represent the objective as a minimization
    // we set each state's cost to be the reciprocal of its clearance, so that as state clearance increases, the state cost decreases
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / (compute_clearance(s, obstacles) + std::numeric_limits<double>::min()));
    }
};

// returns a structure representing the optimization objective to use for optimal motion planning
// weighted length and clearance combo: same as 1.0*lengthObj + 0.0*clearObj
// THE FOLLOWING COMBINATIONS ARE POSSIBLE:
// 		1) Path Length Objective:
//			desc: returns an objective which attempts to minimize the length in configuration space of computed paths
// 		2) Path Clearance Objective:
//			desc: maximizes path clearance from obstacles
//		3) Weighted Combination between 1) and 2)
// 		4) Threshold Path Length Objective:
//			desc: stops planning as soon as it has found a path shorter than the given threshold, if didn't find -> shortest possible
ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si, const int obstacles)
{
    // Path Length Objective without threshold
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    // Path Clearance Objective
    // exploits custom class ClearanceObjective
    auto clearObj(std::make_shared<ClearanceObjective>(si, obstacles));
    // Weighted Objective pointer
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));

    // SET DESIRED WEIGHTS
    // if Path Length Objective is desired
    // 		*) set Clearance Objective weight equal to 0.0
    // if Path Clearance Objective is desired
    // 		*) set Length Objective weight equal to 0.0
    // if Combination of Path Clearance and Length Objectives is desired
    // 		*) distribute weights between them in a desired way
    // if Threshold Path Length Objective is desired:
    //		*1) set Clearance Objective weight equal to 0.0
    //		*2) uncomment the last line before return and set desired threshold
    opt->addObjective(lengthObj, 1.0);
    
    // add clearance only if we are in a non-free environment
    if (clearObj->obstacles > 0)
		opt->addObjective(clearObj, 0.0);
	opt->setCostThreshold(ob::Cost(1000.0));
  
    return ob::OptimizationObjectivePtr(opt);
}

// generic plan function
void planWithSimpleSetup(oc::SimpleSetup ss,
						 const std::string output_path,
                         const std::string graph_path,
						 const int run_count,
						 const double runtime_limit)
{
	// set the propagation step size ...
	ss.getSpaceInformation()->setPropagationStepSize(.1);
	// ... and the duration (in time) of the applied controls
	ss.getSpaceInformation()->setMinMaxControlDuration(1, 10);

	// if you specify a number of desired runs equal to just 1, it will solve the
    // problem just once with RRT and print out the solution path
    if (run_count < 2)
    {	
        ss.setup();
        ss.print();

		// attempt to solve the problem within runtime_limit seconds of planning time
		// intrinsically calls SimpleSetup
		// regular planners stop when they've found a path from start to goal
		// optimizing planners stop when they've found a path from start to goal that satisfies the optimization objective
		ob::PlannerStatus solved = ss.solve(runtime_limit);

		// if the solution has been found, we can simplify it and print on the screen
		if(solved)
		{
			std::cout << "Found solution." << std::endl;

			oc::PathControl path = ss.getSolutionPath();

			/*
			 * // routine to run the algorithm until a minimum desired solution path is found
            std::cout << "With state count: " << path.asGeometric().getStateCount() << std::endl;
            int len = (int) path.asGeometric().getStateCount();
            while (len > 175|| !ss.haveExactSolutionPath()) {
                ss.clear();
                ss.solve(runtime_limit);
                path = ss.getSolutionPath();
                std::cout << "With state count: " << path.asGeometric().getStateCount() << std::endl;
                len = (int) path.asGeometric().getStateCount();
            }
            */

            if (!ss.haveExactSolutionPath())
            {
                std::cout << "Solution is approximate. Distance to actual goal is "
                       << ss.getProblemDefinition()->getSolutionDifference() << "." << std::endl;
            }

            // extracting planner data from most recent solve attempt
            ob::PlannerData pd(ss.getSpaceInformation());
            ss.getPlannerData(pd);

            if (graph_path != "")
			{
                std::ofstream graph_file;
                graph_file.open(graph_path);
                pd.printGraphML(graph_file);
                graph_file.close();
                std::cout << "Planner data was written in the graphml file: " << graph_path << "." << std::endl;
			}
            
            // print the solution to the output file rather than to stdout
            std::ofstream output_file;
			output_file.open(output_path);
			path.asGeometric().printAsMatrix(output_file);
			// also the desired goal
			ob::CompoundState *goal = ss.getGoal()->as<ob::GoalState>()->getState()->as<ob::CompoundState>();
			const double x = goal->as<ob::SE2StateSpace::StateType>(0)->getX();
			const double y = goal->as<ob::SE2StateSpace::StateType>(0)->getY();
			const double theta = goal->as<ob::SE2StateSpace::StateType>(0)->getYaw();
			const double psi = goal->as<ob::SO2StateSpace::StateType>(1)->value;
			const double phi = goal->as<ob::SO2StateSpace::StateType>(2)->value;
			output_file << x << " " << y << " " << theta << " " << psi << " " << phi;
			output_file.close();
			
			std::cout << "Solution was written in the output file: " << output_path << "." << std::endl;
		}
		else
			std::cout << "No solution found." << std::endl;

        exit(0);
    }

	// if run_count > 1 we get here.
	// planning parameters:
	// 	    runtime_limit: experiment execution time
	//   	memory_limit:  experiment memory limit
	//   	run_count:  how many times to run each planner
  	double memory_limit = 10000;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(ss); 

	// add here all the planners you want to test
    addPlanner(b, std::make_shared<oc::RRT>(ss.getSpaceInformation()), 0.3);
    addPlanner(b, std::make_shared<oc::SST>(ss.getSpaceInformation()), 0.3);

    b.benchmark(request);
    b.saveResultsToFile("solution.log");
}

void planForward(ob::StateSpacePtr space,
				 const std::string output_path,
                 const std::string graph_path,
				 const std::string planner_name,
				 const int run_count,
				 const double runtime_limit,
				 const bool use_cl)
{							 
	// set up the bound for the Cartesian components
	ob::RealVectorBounds bounds(2); // (x, y)
	bounds.setLow(-3);
	bounds.setHigh(3);

	space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(0)->setBounds(bounds);
	
	/* discretizing the control space to model Dubins motion primitives on our own.
	 * the control space is now compounded of two discretized sub-spaces;
	 * in particular:
	 * - the driving velocity input space (Dvel) is limited to [v_min, v_max]
	 * - the steering velocity input space (Svel) is limited to [-w, +w]
	 * 		so w takes value among {-w, -w+1, ..., -1, 0, 1, ..., +w-1, +w}
	*/
	
	auto Dvel(std::make_shared<oc::DiscreteControlSpace>(space, -1, 2));
	auto Svel(std::make_shared<oc::DiscreteControlSpace>(space, -5, 5));
	
	oc::CompoundControlSpace *ccs = new oc::CompoundControlSpace(space);
	ccs->addSubspace(oc::ControlSpacePtr(Dvel));
	ccs->addSubspace(oc::ControlSpacePtr(Svel));
	oc::ControlSpacePtr cspace(ccs);
	
	// create an instance of SimpleSetup which internally contains SpaceInformation and ProblemDefinition
	oc::SimpleSetup ss(cspace);
	
	// set the state validity checker
	ss.setStateValidityChecker([&ss](const ob::State *state)
		{ 
			return isStateValid(ss.getSpaceInformation().get(), state, 0); 
		});

	// use the ODESolver to propagate the system
	// call CarTrailerPostIntegration when integration has finished to normalize the orientation values
	auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &CarTrailerODE));
	if (use_cl) odeSolver->setODE(&CarTrailerODE_CL);	
	ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &CarTrailerPostIntegration));

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// simple forward motion
	start[0] = -2.; // x
	start[1] = -2.; // y
	start[2] = 0.;  // theta
	start[3] = 0.;	// psi
	start[4] = 0.;	// phi

	goal[0] = 2.;	// x
	goal[1] = 2.;   // y
	goal[2] = boost::math::constants::pi<double>()/2; // theta
	goal[3] = 0.;	// psi
	goal[4] = 0.;	// phi
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.2);
	
	// by default the bias is 0.05 (5% of probability to pick as random state the goal)
	// here we increase this probability s.t. we can move faster to the goal
    double goal_bias = 0.3;
	if (planner_name == "RRT" || planner_name == "CLRRT")
	{
		oc::RRT *rrt = new oc::RRT(ss.getSpaceInformation());
		rrt->setGoalBias(goal_bias);
		ob::PlannerPtr planner(rrt);
		ss.setPlanner(planner);
	}
	else
	{
		// sets Optimization Objective of interest: 4 different objectives are possible
		// for the details check commentaries to getBalancedObjective function
		ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), 0));
	
		oc::SST *sst = new oc::SST(ss.getSpaceInformation());
		sst->setGoalBias(goal_bias);
		ob::PlannerPtr planner(sst);
		ss.setPlanner(planner);
	}

	planWithSimpleSetup(ss, output_path, graph_path, run_count, runtime_limit);
}

void planBackward(ob::StateSpacePtr space,
				  const std::string output_path,
                  const std::string graph_path,
				  const std::string planner_name,
				  const int run_count,
				  const double runtime_limit,
				  const bool use_cl)
{							 
	// set up the bound for the Cartesian components
	ob::RealVectorBounds bounds(2); // (x, y)
	bounds.setLow(-3);
	bounds.setHigh(3);

	space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(0)->setBounds(bounds);
	
	/* discretizing the control space to model Dubins motion primitives on our own.
	 * the control space is now compounded of two discretized sub-spaces;
	 * in particular:
	 * - the driving velocity input space (Dvel) is limited to [v_min, v_max]
	 * 		according to the desired maneuever
	 * - the steering velocity input space (Svel) is limited to [-w, +w]
	 * 		so w takes value among {-w, -w+1, ..., -1, 0, 1, ..., +w-1, +w}
	*/
	
	auto Dvel(std::make_shared<oc::DiscreteControlSpace>(space, -2, 1));
	auto Svel(std::make_shared<oc::DiscreteControlSpace>(space, -5, 5));
	
	oc::CompoundControlSpace *ccs = new oc::CompoundControlSpace(space);
	ccs->addSubspace(oc::ControlSpacePtr(Dvel));
	ccs->addSubspace(oc::ControlSpacePtr(Svel));
	oc::ControlSpacePtr cspace(ccs);
	
	// create an instance of SimpleSetup which internally contains SpaceInformation and ProblemDefinition
	oc::SimpleSetup ss(cspace);
	
	// set the state validity checker
	ss.setStateValidityChecker([&ss](const ob::State *state)
		{ 
			return isStateValid(ss.getSpaceInformation().get(), state, 0); 
		});

	// use the ODESolver to propagate the system
	// call CarTrailerPostIntegration when integration has finished to normalize the orientation values
	auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &CarTrailerODE));
	if (use_cl) odeSolver->setODE(&CarTrailerODE_CL);
	ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &CarTrailerPostIntegration));

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// simple backward motion
	start[0] = 2.;	// x
	start[1] = 2.;	// y
	start[2] = boost::math::constants::pi<double>()/2;  // theta
	start[3] = 0.;	// psi
	start[4] = 0.;	// phi

	goal[0] = -2.;	// x
	goal[1] = -2.;	// y
	goal[2] = 0.;	// theta
	goal[3] = 0.;	// psi
	goal[4] = 0.;	// phi
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.3);
	
	// by default the bias is 0.05 (5% of probability to pick as random state the goal)
	// here we increase this probability s.t. we can move faster to the goal
    double goal_bias = 0.3;
	if (planner_name == "RRT" || planner_name == "CLRRT")
	{
		oc::RRT *rrt = new oc::RRT(ss.getSpaceInformation());
		rrt->setGoalBias(goal_bias);
		ob::PlannerPtr planner(rrt);
		ss.setPlanner(planner);
	}
	else
	{
		// sets Optimization Objective of interest: 4 different objectives are possible
		// for the details check commentaries to getBalancedObjective function
		ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), 0));

		oc::SST *sst = new oc::SST(ss.getSpaceInformation());
		sst->setGoalBias(goal_bias);
		ob::PlannerPtr planner(sst);
		ss.setPlanner(planner);
	}

	planWithSimpleSetup(ss, output_path, graph_path, run_count, runtime_limit);
}

void planObstacles(ob::StateSpacePtr space,
				   const std::string output_path,
                   const std::string graph_path,
				   const std::string planner_name,
				   const int run_count,
				   const double runtime_limit,
				   const bool use_cl)
{
	// set up the bound for the Cartesian components
	ob::RealVectorBounds bounds(2); // (x, y)
	bounds.setLow(-3);
	bounds.setHigh(3);

	space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(0)->setBounds(bounds);
	
	/* discretizing the control space to model Dubins motion primitives on our own.
	 * the control space is now compounded of two discretized sub-spaces;
	 * in particular:
	 * - the driving velocity input space (Dvel) is is limited to [v_min, v_max]
	 * 		according to the desired maneuever
	 * - the steering velocity input space (Svel) is limited to [-w, +w]
	 * 		so w takes value among {-w, -w+1, ..., -1, 0, 1, ..., +w-1, +w}
	*/

	auto Dvel(std::make_shared<oc::DiscreteControlSpace>(space, -2, 2));
	auto Svel(std::make_shared<oc::DiscreteControlSpace>(space, -5, 5));
	
	oc::CompoundControlSpace *ccs = new oc::CompoundControlSpace(space);
	ccs->addSubspace(oc::ControlSpacePtr(Dvel));
	ccs->addSubspace(oc::ControlSpacePtr(Svel));
	oc::ControlSpacePtr cspace(ccs);
	
	// create an instance of SimpleSetup which internally contains SpaceInformation and ProblemDefinition
	oc::SimpleSetup ss(cspace);
	
	// set the state validity checker
	ss.setStateValidityChecker([&ss](const ob::State *state)
		{ 
			return isStateValid(ss.getSpaceInformation().get(), state, 1); 
		});

	// use the ODESolver to propagate the system
	// call CarTrailerPostIntegration when integration has finished to normalize the orientation values
	auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &CarTrailerODE));
	if (use_cl) odeSolver->setODE(&CarTrailerODE_CL);
	ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &CarTrailerPostIntegration));

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// circular obstacles avoidance start and goal
	start[0] = -2.; // x
	start[1] = -2.; // y
	start[2] = boost::math::constants::pi<double>()/6;  // theta
	start[3] = 0.;	// psi
	start[4] = 0.;	// phi

    goal[0] = -2.1; // x
    goal[1] = 1.5;  // y
    goal[2] = 7*boost::math::constants::pi<double>()/8; // theta
	goal[3] = 0.;	// psi
	goal[4] = 0.;	// phi
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.25);
	
	// by default the bias is 0.05 (5% of probability to pick as random state the goal)
	// here we increase this probability s.t. we can move faster to the goal
    double goal_bias = 0.3;
	if (planner_name == "RRT" || planner_name == "CLRRT")
	{
		oc::RRT *rrt = new oc::RRT(ss.getSpaceInformation());
		rrt->setGoalBias(goal_bias);
		ob::PlannerPtr planner(rrt);
		ss.setPlanner(planner);
	}
	else
	{
		// sets Optimization Objective of interest: 4 different objectives are possible
		// for the details check commentaries to getBalancedObjective function
		ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), 1));

		oc::SST *sst = new oc::SST(ss.getSpaceInformation());
		sst->setGoalBias(goal_bias);
		ob::PlannerPtr planner(sst);
		ss.setPlanner(planner);
	}

	planWithSimpleSetup(ss, output_path, graph_path, run_count, runtime_limit);
}

void planTurn(ob::StateSpacePtr space,
			  const std::string output_path,
              const std::string graph_path,
			  const std::string planner_name,
			  const int run_count,
			  const double runtime_limit,
			  const bool use_cl)
{
	// set up the bound for the Cartesian components
	ob::RealVectorBounds bounds(2); // (x, y)
	bounds.high[0] = 3;
	bounds.high[1] = 1;

	space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(0)->setBounds(bounds);
	
	/* discretizing the control space to model Dubins motion primitives on our own.
	 * the control space is now compounded of two discretized sub-spaces;
	 * in particular:
	 * - the driving velocity input space (Dvel) is limited to [v_min, v_max]
	 * 		according to the desired maneuever
	 * - the steering velocity input space (Svel) is limited to [-w, +w]
	 * 		so w takes value among {-w, -w+1, ..., -1, 0, 1, ..., +w-1, +w}
	*/

	auto Dvel(std::make_shared<oc::DiscreteControlSpace>(space, -2, 2));
	auto Svel(std::make_shared<oc::DiscreteControlSpace>(space, -5, 5));
	
	oc::CompoundControlSpace *ccs = new oc::CompoundControlSpace(space);
	ccs->addSubspace(oc::ControlSpacePtr(Dvel));
	ccs->addSubspace(oc::ControlSpacePtr(Svel));
	oc::ControlSpacePtr cspace(ccs);
	
	// create an instance of SimpleSetup which internally contains SpaceInformation and ProblemDefinition
	oc::SimpleSetup ss(cspace);
	
	// set the state validity checker
	ss.setStateValidityChecker([&ss](const ob::State *state)
		{ 
			return isStateValid(ss.getSpaceInformation().get(), state, 2); 
		});

	// use the ODESolver to propagate the system
	// call CarTrailerPostIntegration when integration has finished to normalize the orientation values
	auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &CarTrailerODE));
	if (use_cl) odeSolver->setODE(&CarTrailerODE_CL);
	ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &CarTrailerPostIntegration));

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// parking test in real world start and goal
	start[0] = 0.5;	 // x
	start[1] = 0.25; // y
	start[2] = -boost::math::constants::pi<double>();  // theta
	start[3] = 0.;	 // psi
	start[4] = 0.;	 // phi

	goal[0] = 2.5;	 // x
	goal[1] = 0.25;  // y
	goal[2] = 0.; 	 // theta
	goal[3] = 0.;	 // psi
	goal[4] = 0.;	 // phi
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.25);
	
	// by default the bias is 0.05 (5% of probability to pick as random state the goal)
	// here we increase this probability s.t. we can move faster to the goal
    double goal_bias = 0.3;
	if (planner_name == "RRT" || planner_name == "CLRRT")
	{
		oc::RRT *rrt = new oc::RRT(ss.getSpaceInformation());
		rrt->setGoalBias(goal_bias);
		ob::PlannerPtr planner(rrt);
		ss.setPlanner(planner);
	}
	else
	{
		// sets Optimization Objective of interest: 4 different objectives are possible
		// for the details check commentaries to getBalancedObjective function
		ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), 2));

		oc::SST *sst = new oc::SST(ss.getSpaceInformation());
		sst->setGoalBias(goal_bias);
		ob::PlannerPtr planner(sst);
		ss.setPlanner(planner);
	}
	
	planWithSimpleSetup(ss, output_path, graph_path, run_count, runtime_limit);
}

void planReal(ob::StateSpacePtr space,
			  const std::string output_path,
              const std::string graph_path,
			  const std::string planner_name,
			  const int run_count,
			  const double runtime_limit,
			  const bool use_cl)
{
	// set up the bound for the Cartesian components
	ob::RealVectorBounds bounds(2); // (x, y)
	bounds.setLow(0);
	bounds.setHigh(3);

	space->as<ob::CompoundStateSpace>()->as<ob::SE2StateSpace>(0)->setBounds(bounds);
	
	/* discretizing the control space to model Dubins motion primitives on our own.
	 * the control space is now compounded of two discretized sub-spaces;
	 * in particular:
	 * - the driving velocity input space (Dvel) is limited to [v_min, v_max]
	 * 		according to the desired maneuever
	 * - the steering velocity input space (Svel) is limited to [-w, +w]
	 * 		so w takes value among {-w, -w+1, ..., -1, 0, 1, ..., +w-1, +w}
	*/

	auto Dvel(std::make_shared<oc::DiscreteControlSpace>(space, -2, 2));
	auto Svel(std::make_shared<oc::DiscreteControlSpace>(space, -5, 5));
	
	oc::CompoundControlSpace *ccs = new oc::CompoundControlSpace(space);
	ccs->addSubspace(oc::ControlSpacePtr(Dvel));
	ccs->addSubspace(oc::ControlSpacePtr(Svel));
	oc::ControlSpacePtr cspace(ccs);
	
	// create an instance of SimpleSetup which internally contains SpaceInformation and ProblemDefinition
	oc::SimpleSetup ss(cspace);
	
	// set the state validity checker
	ss.setStateValidityChecker([&ss](const ob::State *state)
		{ 
			return isStateValid(ss.getSpaceInformation().get(), state, 3); 
		});

	// use the ODESolver to propagate the system
	// call CarTrailerPostIntegration when integration has finished to normalize the orientation values
	auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &CarTrailerODE));
	if (use_cl) odeSolver->setODE(&CarTrailerODE_CL);
	ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &CarTrailerPostIntegration));

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);

	// parking test in real world start and goal
	start[0] = 2;   // x
	start[1] = 2.3; // y
	start[2] = boost::math::constants::pi<double>()/4;  // theta
	start[3] = 0.;	// psi
	start[4] = 0.;	// phi

	goal[0] = 1.5;	// x
	goal[1] = 0.5;  // y
	goal[2] = boost::math::constants::pi<double>()/2; // theta
	goal[3] = 0.;	// psi
	goal[4] = 0.;	// phi
	
	// set these states as start and goal for the SimpleSetup
	// the third parameter denotes the threshold
	ss.setStartAndGoalStates(start, goal, 0.25);
	
	// by default the bias is 0.05 (5% of probability to pick as random state the goal)
	// here we increase this probability s.t. we can move faster to the goal
    double goal_bias = 0.3;
	if (planner_name == "RRT" || planner_name == "CLRRT")
	{
		oc::RRT *rrt = new oc::RRT(ss.getSpaceInformation());
		rrt->setGoalBias(goal_bias);
		ob::PlannerPtr planner(rrt);
		ss.setPlanner(planner);
	}
	else
	{
		// sets Optimization Objective of interest: 4 different objectives are possible
		// for the details check commentaries to getBalancedObjective function
		ss.setOptimizationObjective(getBalancedObjective(ss.getSpaceInformation(), 3));

		oc::SST *sst = new oc::SST(ss.getSpaceInformation());
		sst->setGoalBias(goal_bias);
		ob::PlannerPtr planner(sst);
		ss.setPlanner(planner);
	}
	
	planWithSimpleSetup(ss, output_path, graph_path, run_count, runtime_limit);
}

int main(int argc, char **argv)
{
	try
	{
		// parsing args from command line
		std::string path;
        std::string graph_path;
        std::string exp;
        std::string planner;
        int run_count;
        double run_limit;

        po::options_description desc("Options");
        desc.add_options()
            ("help,h", "show help message")
            ("outfile", po::value<std::string>(&path)->default_value("solution.txt"), "path of the file to save the solution")
            ("graphfile", po::value<std::string>(&graph_path)->default_value(""), "path to the graphml representation of the planner data")
            ("exp", po::value<std::string>(&exp)->default_value("forward"), "type of experiment: {forward, backward, obstacles, turn, real}")
            ("planner", po::value<std::string>(&planner)->default_value("RRT"), "type of planner: {RRT, SST, CLRRT}")
            ("run_count", po::value<int>(&run_count)->default_value(1), "number of experiments you want to run")
            ("run_limit", po::value<double>(&run_limit)->default_value(45.0), "time limit (in seconds) allocated for computation of exact solution");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if ((vm.count("help") != 0u))
        {
            std::cout << desc << "\n";
            return 1;
        }
        
        // checking arguments validity
        if (planner != "RRT" && planner != "SST" && planner != "CLRRT")
		{
			std::cout << "Invalid argument for planner type (--planner). Please, retry." << std::endl;
			return 1;
		}
		
		// construct the state space itself
		auto SE2(std::make_shared<ob::SE2StateSpace>());
		auto psi(std::make_shared<ob::SO2StateSpace>());
		auto phi(std::make_shared<ob::SO2StateSpace>());

		ob::CompoundStateSpace *cs = new ob::CompoundStateSpace();
		// SE2 + psi + phi;
		cs->addSubspace(ob::StateSpacePtr(SE2), 1.0);
		cs->addSubspace(ob::StateSpacePtr(psi), 0.5);
		cs->addSubspace(ob::StateSpacePtr(phi), 0.0);
		
		ob::StateSpacePtr space(cs);
		
		bool use_cl = false;
		if (planner == "CLRRT") use_cl = true;
		// redirecting to the desired type of experiment
		if (exp == "forward")
			planForward(space, path, graph_path, planner, run_count, run_limit, use_cl);
		else if (exp == "backward")
			planBackward(space, path, graph_path, planner, run_count, run_limit, use_cl);
		else if (exp == "obstacles")
			planObstacles(space, path, graph_path, planner, run_count, run_limit, use_cl);
		else if (exp == "turn")
			planTurn(space, path, graph_path, planner, run_count, run_limit, use_cl);
		else if (exp == "real")
			planReal(space, path, graph_path, planner, run_count, run_limit, use_cl);
		else {
			std::cout << "Invalid argument for experiment type (--exp). Please, retry." << std::endl;
			return 1;
		}
	}
	catch(std::exception& e)
	{
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}
	
	return 0;
}
