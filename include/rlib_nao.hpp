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

#ifndef _RLIB_NAO_HPP
#define _RLIB_NAO_HPP

// Standard Includes

// Special Includes

// Robot lib Includes
#include "rlib_robot.hpp"
// #include "rlib_sensor.hpp"
#include "rlib_actuator.hpp"
// #include "rlib_perceiver.hpp"
// #include "rlib_planner.hpp"

/*
If using V-REP, you must include 
#include "extApi.h"
before including this header in main
and #define RUNNING_VREP

If using the actual NAO, you must include
...some files
and #define RUNNING_NAO
*/

//#define RUNNING_VREP
#define RUNNING_NAO


#if defined RUNNING_NAO
  // #include <alproxies/almotionproxy.h>
  // #include <alproxies/almemoryproxy.h>
#elif defined RUNNING_VREP
  extern "C" {
    // #include "extApi.h"
  }
#else
  #error Either RUNNING_VREP or RUNNING_NAO must be defined!
#endif

namespace rlib{

	// Define simple mobile object for walking
	class NaoBasicWalker : public Actuator{
	public:
		NaoBasicWalker(const char * name);
		// Set the pose of the actuator with respect to its origin.
		void setPosition(Pose pose);
		// Set the velocity of the actuator in the local frame.
		void setVelocity(Vel velocity);
	};

	// TODO: Add builder class that returns a robot of type Nao.

}
#endif