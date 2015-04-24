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

#include <rlib_state.hpp>

namespace rlib{
	// Default constructor
	Pose::Pose(){}

	// Constructor with initial values as xyz and rpy
	// rpy is Roll, Pitch, Yaw, or Tait-Brian angles. Roll is about the
	// x axis, then pitch is about the new y axis, and yaw is about the final
	// z axis.
	Pose::Pose(float x, float y, float z, float roll, float pitch, float yaw){
		// Set linear part.
		setXYZ(x,y,z);
		// Set angular part.
		setRPY(roll, pitch, yaw);
	}

	// Set linear pose.
	void Pose::setXYZ(float x, float y, float z){
		// Apply any limits.
		// Set the linear elements.
		p.x() = x; p.y() = y; p.z() = z;
	}

	// Set angular position by setting rpy as in the constructor.
	void Pose::setRPY(float roll, float pitch, float yaw){
		// Apply any limits.

		// Create angle axis representations.
		Eigen::AngleAxis<float> rollAngle(roll, Eigen::Vector3f::UnitX());
		Eigen::AngleAxis<float> pitchAngle(pitch, Eigen::Vector3f::UnitY());
		Eigen::AngleAxis<float> yawAngle(yaw, Eigen::Vector3f::UnitZ());

		// Compose them in the rpy order and then make them a quaternion.
		q = rollAngle * pitchAngle * yawAngle;
	}

}