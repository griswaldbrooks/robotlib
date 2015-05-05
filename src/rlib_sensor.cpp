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
	// Lidar Data Class ***/
	
	// Constructor with ranges in meters and angles in radians. Angles
	// should correspond to each range by index.
	LidarData::LidarData(std::vector<float> &ranges, std::vector<float> &angles){
		// Set the ranges into the first row of the data.
		for(int j = 0; j < ranges.size(); j++){
			_data.row(0) << ranges[j];
		}
		// Set the angles into the second row of the data.
		for(int j = 0; j < angles.size(); j++){
			_data.row(1) << angles[j];
		}
	}

	// Returns number of data elements.
	size_t LidarData::size(){
		return _data.rows();
	}

	// Get a range for a given angle. If no angle is found, method will return range to the closest angle.
	float LidarData::getRange(float angle){
		// Take the difference of all of the angle elements, get their magnitude
		// and find the smallest one's index.
		int minIndex;
		Eigen::ArrayXf offsetAngles = _data.row(1);
		(offsetAngles - angle).abs().minCoeff(&minIndex);

		// Return the matching distance
		return _data(0, minIndex);
	}

	// Get the sensor data. Returns a const reference to the data.
	const Eigen::Matrix<float, 2, Eigen::Dynamic> & LidarData::getData(){
		return _data;
	}

	///*** Sensor Abstract Class ***///

	// Constructor for sensor class.
	Sensor::Sensor(const char * name):_name(name){
		// More things to construct.
		_parent = NULL;
		_handler = NULL;
		_tf.T.setIdentity();
	}

	// Return name of sensor.
	std::string Sensor::getName(){
		return _name;
	}
	// Set the transformation of the sensor with respect to its parent.
	void Sensor::setFrame(Transform tf){
		// Do some frame checking.

		// Assign frame to our frame.
		_tf = tf;
	}

	///*** Lidar Class ***///

	Lidar::Lidar(const char * name): Sensor(name){
		// Do more constructory things.
		std::cout << "Lidar: " << getName() << " constructed." << std::endl;
	}
	
	// Start sensor
	void Lidar::startSensor(){
		// Call our implementation object to do the work for us.
	}

	// Stop sensor
	void Lidar::stopSensor(){
		// Call our implementation object to do the work for us.
	}

	// Get Lidar Data
	SensorData getData(){
		// Get the data from the sensor via the implementation.
		// Package it into Lidar Data and send it out.
		std::vector<float> v, w;
		return LidarData(v,w);
	}
}