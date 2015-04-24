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
		// Default constructor
		Pose();
		// Constructor with initial values as xyz and rpy
		// rpy is Roll, Pitch, Yaw, or Tait-Brian angles. Roll is about the
		// x axis, then pitch is about the new y axis, and yaw is about the final
		// z axis.
		Pose(float x, float y = 0, float z = 0, float roll = 0, float pitch = 0, float yaw = 0);
		// Set linear pose.
		void setXYZ(float x, float y, float z);
		// Set angular position by setting rpy as in the constructor.
		void setRPY(float roll, float pitch, float yaw);
		// Linear pose represented as x, y, z.
		Eigen::Translation<float, 3> p;
		// Angular pose represented as w, x, y, z.
		Eigen::Quaternion<float> q;
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