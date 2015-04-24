// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for basic state information such 
// as poses, transformations, and velocities.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _RLIB_STATE_HPP
#define _RLIB_STATE_HPP

// Standard Includes

// Special Includes
#include <eigen3/Eigen/Geometry>
// #include <eigen3/Eigen/Dense>

// Robot lib Includes

namespace rlib{

	// Class for defining velocities.
	class Vel{
	public:
		// Linear velocities in m/s
		float vx, vy, vz;
		// Angular velocities in rad/s
		float wx, wy, wz;
	};

	// Class for defining pose.
	class Pose{
	public:
		// Linear pose represented as x,y,z
		Eigen::Translation<float, 3> p;
		// Angular pose represented as w x y z
		Eigen::Quaternionf q;
	};

	// Class for defining rigid body transformations.
	class Transform{
	public:
		// Rigid body relationships are represented as a 4x4 Homogeneous
		// Transformation matrix.
		Eigen::Transform<float, 3, Eigen::Affine> T;
	};


}

#endif