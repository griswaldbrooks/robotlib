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

	/*! 
	* \brief Class for defining velocities.
	*/
	class Vel{
	public:
		float vx; /**< Linear velocities in the x-axis in m/s. */
		float vy; /**< Linear velocities in the y-axis in m/s. */
		float vz; /**< Linear velocities in the z-axis in m/s. */
		float wx; /**< Angular velocities in the x-axis in rad/s. */
		float wy; /**< Angular velocities in the y-axis in rad/s. */
		float wz; /**< Angular velocities in the z-axis in rad/s. */
	};

	/*!
	* \brief Class for defining pose.
	*/
	/** This class uses three translational elements and one quaternion to represent
	* 	pose information.
	*/
	class Pose{
	public:
		/*!
		* \brief Default constructor.
		*/
		Pose();

		/*!
		* \brief Constructor with initial values as xyz and rpy.
		*/
		/** rpy is Roll, Pitch, Yaw, or Tait-Brian angles. Roll is about the
		* 	x axis, then pitch is about the new y axis, and yaw is about the final
		* 	z axis.
		* 	\param[in] 	x 		Translation in the x direction in meters.
		* 	\param[in] 	y 		Translation in the y direction in meters.
		* 	\param[in] 	z 		Translation in the z direction in meters.
		* 	\param[in] 	roll 	Rotation about the x direction in radians.
		* 	\param[in] 	pitch 	Rotation about the y' direction in radians where y' is the new y axis after 
		*						performing the roll operation.
		* 	\param[in] 	yaw 	Rotation about the z'' direction in radians where z'' is the new z axis after 
		*						performing the pitch operation.
		*/
		Pose(float x, float y = 0, float z = 0, float roll = 0, float pitch = 0, float yaw = 0);
		
		/*! 
		* \brief Set linear pose.
		* 	\param[in] 	x 		Translation in the x direction in meters.
		* 	\param[in] 	y 		Translation in the y direction in meters.
		* 	\param[in] 	z 		Translation in the z direction in meters.
		*/
		void setXYZ(float x, float y, float z);
		
		/*!
		* \brief Set angular position by setting rpy as in the constructor.
		* 	\param[in] 	roll 	Rotation about the x direction in radians.
		* 	\param[in] 	pitch 	Rotation about the y' direction in radians where y' is the new y axis after 
		*						performing the roll operation.
		* 	\param[in] 	yaw 	Rotation about the z'' direction in radians where z'' is the new z axis after 
		*						performing the pitch operation.
		*/
		void setRPY(float roll, float pitch, float yaw);

		Eigen::Translation<float, 3> p; 	/**< Linear pose represented as x, y, z. */
		Eigen::Quaternion<float> q; 		/**< Angular pose represented as w, x, y, z. */
	};

	/*!
	* \brief Class for defining rigid body transformations.
	*/
	class Transform{
	public:
		Eigen::Transform<float, 3, Eigen::Affine> T; /**< Rigid body relationships are represented as a 4x4 Homogeneous
															Transformation matrix. */
	};


}

#endif