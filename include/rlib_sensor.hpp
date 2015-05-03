// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the sensor classes.
//
// Sensor is an abstract class to allow for the creation of 
// specific sensors with their own sensing needs such as Lidars, Sonars,
// Cameras, etc.
//
// Sensors can be attached to actuators or their derivative classes (such
// as the robot class) but have no children objects.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _RLIB_SENSOR_HPP
#define _RLIB_SENSOR_HPP

// Standard Includes
#include <string>
#include <vector>
#include <utility> // std::pair

// Special Includes

// Robot lib Includes
#include <rlib_state.hpp>
#include <rlib_actuator.hpp>

namespace rlib{
	// Class for containing sensor data. Top level class made for compatibility
	// with other classes. Implementation of each will look very different from 
	// each other.
	class SensorData{};

	class SonarData : public SensorData{};

	class InfraredData : public SensorData{};

	class ImageData : public SensorData{};

	class PointCloudData : public SensorData{};

	class LidarData : public SensorData{
	public:
		// Constructor with ranges in meters and angles in radians. Angles
		// should correspond to each range by index.
		LidarData(std::vector<float> &ranges, std::vector<float> &angles);

		// Returns number of data elements.
		size_t size();

		// Get a range for a given angle. If no angle is found, method will return
		// zero.
		// TODO: If no angle is found, method will return range to the closest angle.
		float getRange(float angle);

		// TODO: Change this to use iterators instead of indices.
		
		// Get range and angle pair at a particular index. Puts the result in data. 
		// If the data doesn't exist, returns false. 
		bool getData(int ndx, std::pair<float, float> &data);

	private:
		// Lidar data in pair format. First element in pair corresponds to range in
		// meters. Second element in pair in the angle of that range in radians.
		std::vector<std::pair<float,float> > _data;
	};

	// Class for building basic sensors.
	class Sensor{
	public:	
		// Constructor for the sensor class.
		Sensor(const char * name);
		// Return name of sensor
		std::string getName();
		// Set the transformation of the sensor with respect to its parent.
		void setFrame(Transform tf);
		// Start running the sensor
		virtual void startSensor() = 0;
		// Stop running the sensor
		virtual void stopSensor() = 0;
		// Get sensor data
		virtual SensorData getData() = 0;
	private:
		// Unique identifier for the sensor
		std::string _name;
		// The sensor object that this sensor is a child of.
		Actuator* _parent;
		// Transformation that the parent must do on its origin to
		// reach the origin of the child (the child is this sensor).
		Transform _tf;
	};

	// Class for building sensor implementations
	class SensorImp{
		// Constructor
		SensorImp();
	};

	/* Some basic derivatives */

	// Lidar class for making sensors that produce planar point clouds.
	class Lidar : public Sensor{
	public:
		// Lidar constructor
		Lidar(const char * name);
		// Set the transformation of the sensor with respect to its parent.
		void setFrame(Pose pose);
		// Start running the sensor
		void startSensor();
		// Stop running the sensor
		void stopSensor();
		// Get Lidar Data

	};

}

#endif