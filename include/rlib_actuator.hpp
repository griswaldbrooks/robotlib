// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the actuator classes.
// One of the main ideas in this library is that actuators can be
// constructed into chains of actuators using the parent child 
// structure. If the actuator needs more than one child then 
// the robot class in the rlib_robot.cpp library can be used
// to allow multiple children and to add additional structures
// such and sensors and planners.
//
// Actuator is an abstract class to allow for the creation of 
// specific actuators with their own movement needs such as mobile
// bases, servos, and manipulators.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _RLIB_ACTUATOR_HPP
#define _RLIB_ACTUATOR_HPP

// Standard Includes
#include <string>

// Special Includes

// Robot lib Includes
#include "rlib_state.hpp"

namespace rlib{

	// Class for building basic actuators.
	class Actuator{
	public:	
		// Constructor for the actuator class.
		Actuator(const char * name);
		// Set the pose of the actuator with respect to its origin.
		virtual void setPosition(Pose& pose) = 0;
		// Set the velocity of the actuator in the local frame.
		virtual void setVelocity(Vel& velocity) = 0;
		// Return name of actuator
		std::string getName();
		// Set the parent actuator
		void setParent(Actuator* parent);
	protected:
		// 6D description of actuator position with respect to the actuator origin.
		// (Do pose maxes and mins make sense?)
		Pose pose, posMax, posMin;
		// 6D description of actuator velocity with respect to the actuator origin.
		Vel velocity, velMax, velMin;
	private:
		// Unique identifier for the actuator
		std::string _name;
		// The actuator object that this actuator is a child of.
		Actuator* _parent;
		// The actuator object that is the child of this one.
		Actuator* _child;
		// Transformation that the parent must do on its origin to
		// reach the origin of the child (the child is this actuator).
		Transform _tf;
		
	};

	/* Some basic derivatives */

	// Mobile base class that takes linear velocity and angular velocity commands.
	class MobileBase : public Actuator{
	public:
		MobileBase(const char * name);
		// Set the pose of the actuator with respect to its origin.
		void setPosition(Pose& pose);
		// Set the velocity of the actuator in the local frame.
		void setVelocity(Vel& velocity);
	};

}

#endif