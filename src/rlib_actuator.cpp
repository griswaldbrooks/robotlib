// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the actuator classes.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Standard Includes
#include <iostream>

// Robotlib Includes
#include <rlib_actuator.hpp>

namespace rlib{
	
	// Constructor for actuator class.
	Actuator::Actuator(const char * name):_name(name){
		// More things to construct.
		_parent = this;
		_child = this;
	}
	// Return name of actuator	
	std::string Actuator::getName(){
		return _name;
	}

	MobileBase::MobileBase(const char * name): Actuator(name){
		// Do more constructory things.
		std::cout << "Mobile Base " << getName() << " constructed." << std::endl;
	}
	// Set the pose of the actuator with respect to its origin.
	void MobileBase::setPosition(Pose& pose){
		// Set limits on the pose
		pose = pose;
		std::cout << "Position set!" << std::endl;
	}
	// Set the velocity of the actuator in the local frame.
	void MobileBase::setVelocity(Vel& velocity){
		// Set limits on the velocity
		velocity = velocity;
		std::cout << "Velocity set!" << std::endl;
	}
}