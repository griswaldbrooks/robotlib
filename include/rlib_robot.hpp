// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the basic robot.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _RLIB_ROBOT_HPP
#define _RLIB_ROBOT_HPP

// Standard Includes
#include <list>
// Special Includes

// Robot lib Includes
#include "rlib_actuator.hpp"
// #include "rlib_sensor.hpp"
// #include "rlib_perceiver.hpp"
// #include "rlib_planner.hpp"

namespace rlib{

	// Class for aggregating robot centric objects.
	class Robot: public Actuator{
	public:	
		Robot();
	private:
		// List of actuators that belong to the robot.
		std::list<Actuator*> _actuators;
		// List of sensors that belong to the robot.
		// List of perceivers that belong to the robot.
		// List of planners that belong to the robot.
	};

}

#endif