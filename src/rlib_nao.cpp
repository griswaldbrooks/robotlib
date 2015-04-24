// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the Nao H4 from
// Aldebaran Robotics.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Standard Includes
#include <iostream>

// Special Includes

// Robot lib Includes
// #include "rlib_robot.hpp"
// #include "rlib_sensor.hpp"
// #include "rlib_actuator.hpp"
// #include "rlib_perceiver.hpp"
// #include "rlib_planner.hpp"
#include <rlib_nao.hpp>


namespace rlib{

	NaoBasicWalker::NaoBasicWalker(const char * name): Actuator(name){
		// Do more constructory things.
		std::cout << "Basic walker " << getName() << " constructed." << std::endl;
	}
	// Set the pose of the actuator with respect to its origin.
	void NaoBasicWalker::setPosition(Pose& pose){
		// Set limits on the pose
		pose = pose;
		std::cout << "Position set!" << std::endl;

		
	}
	// Set the velocity of the actuator in the local frame.
	void NaoBasicWalker::setVelocity(Vel& velocity){
		// Set limits on the velocity
		velocity = velocity;
		std::cout << "Velocity set!" << std::endl;
		
	}

	// TODO: Write spinOnce/Run function that executes the implementation.
	// Most likely it should take an implementation object that has yet to be
	// defined but should belong to the Robot object and send that commands.
	//
	// Which means the below code should actually be in main as which implementation object...
	// Or probably a better idea would be to switch in the Nao constructor object which 
	// implementation object gets used. That would be a good idea.
	//
	// #if defined RUNNING_NAO
	// 		std::cout << "Velocity set Nao!" << std::endl;
	// 	#elif defined RUNNING_VREP
	// 		std::cout << "Velocity set VREP!" << std::endl;
	// 	#endif

}
