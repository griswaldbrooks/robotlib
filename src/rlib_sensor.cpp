// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the sensor classes.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Standard Includes
#include <iostream>

// Robotlib Includes
#include <rlib_sensor.hpp>

namespace rlib{
	///*** Lidar Data Class ***///
	
	// Constructor with ranges in meters and angles in radians. Angles
	// should correspond to each range by index.
	LidarData::LidarData(std::vector<float> &ranges, std::vector<float> &angles){
		// Add the data to the data vector.
		putData(ranges, angles);
	}

	// Returns number of data elements.
	size_t LidarData::size(){}

	// Get a range for a given angle. If no angle is found, method will return
	// zero.
	// TODO: If no angle is found, method will return range to the closest angle.
	float LidarData::getRange(float angle){}


	///*** Sensor Abstract Class ***///

	// Constructor for sensor class.
	Sensor::Sensor(const char * name):_name(name){
		// More things to construct.
		_parent = NULL;
	}
	// Return name of sensor	
	std::string Sensor::getName(){
		return _name;
	}
	// Set the transformation of the sensor with respect to its parent.
	void Sensor::setFrame(Transform tf){
	}

	Lidar::Lidar(const char * name): Sensor(name){
		// Do more constructory things.
		std::cout << "Lidar: " << getName() << " constructed." << std::endl;
	}
	
	// Start sensor
	void Lidar::startSensor(){
	}
}