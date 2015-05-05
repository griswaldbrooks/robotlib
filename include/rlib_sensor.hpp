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

// Special Includes
#include <eigen3/Eigen/Core>
#include "hokuyo_client.hpp"

// Robot lib Includes
#include <rlib_state.hpp>
#include <rlib_actuator.hpp>

namespace rlib{
	///*** Sensor Data Abstract Class ***///

	// Class for containing sensor data. Top level class made for compatibility
	// with other classes. Implementation of each will look very different from 
	// each other.
	class SensorData{};

	///*** Sensor Data Classes ***///
	
	class SonarData : public SensorData{};

	class InfraredData : public SensorData{};

	class ImageData : public SensorData{};

	class PointCloudData : public SensorData{};

	///*** Lidar Data Class ***///

	class LidarData : public SensorData{
	public:
		// Constructor with ranges in meters and angles in radians. Angles
		// should correspond to each range by index.
		LidarData(std::vector<float> &ranges, std::vector<float> &angles);

		// Returns number of data elements.
		size_t size();

		// Get a range for a given angle. If no angle is found, method will return range to the closest angle.
		float getRange(float angle);

		// Get the sensor data.
		const Eigen::Matrix<float, 2, Eigen::Dynamic> & getData();

	private:
		// Lidar data in pair format. First element in pair corresponds to range in
		// meters. Second element in pair in the angle of that range in radians.
		Eigen::Matrix<float, 2, Eigen::Dynamic> _data;
	};

	///*** Sensor Handler Abstract Class ***///

	// Class for building sensor implementations
	class SensorHandler{
	public:
		// Constructor
		SensorHandler();
	};

	//* HokuyoLidarHandlerIP
	/**
	* This is a handler class derived from the rlib::SensorHandler class.
	* It is designed for use with the Hokuyo Lidar URG-04LX-UG01. The Lidar
	* is started using the Python lidar_server_tester.py written by Abhimanyu
	* Ghosh. This client handler is a wrapper class for the client library
	* hokuyo_client.cpp written by Brian Cairl.
	*/
	class HokuyoLidarHandlerIP : public SensorHandler {
	public:
		/*!
		* \brief Constructor for IP version of Hokoyu Lidar Handler.
		*
		* The constructor for the handler needs the IP address and port number
		* that the Lidar is running on.
		* \param[in] ip			This is the IP address of the computer that the Lidar is connected to.
		* \param[in] port 		This is the port number of the Lidar on the computer.
		*/
		HokuyoLidarHandlerIP(char *ip, char *port);

		/*!
		* \brief Initiates communication with the Lidar.
		*/
		void startSensor();

		/*!	
		* \brief Terminates Lidar connection.
		*/
		void stopSensor();

		/*!	
		* \brief Gets range and angle data from the sensor.
		*
		* Requests new sensor data from the Lidar. This is a blocking function.
		* \return SensorData	The range and angle data packed into the rlib::SensorData format.
		*/
		SensorData getData();
	private:
		// Create Hokuyo Laser Client
  		boost::asio::io_service _laserIOServ;	/**< The boost io_service object that the socket object needs.*/
  		boost::asio::ip::tcp::socket _laserSocket; /**< The boost socket object is used to communicate with the Lidar.*/
  		hokuyo::tcp_client _laserClient; /**< The hokuyo client object that parses the Lidar data. */
	};

	///*** Sensor Abstract Class ***///

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
		// Implementation (Bridge) object for interfacing with the physical sensor.
		SensorHandler* _handler;
	};

	///*** Some Basic Sensor Derivative Classes ***///

	// Lidar class for making sensors that produce planar point clouds.
	class Lidar : public Sensor{
	public:
		// Lidar constructor
		Lidar(const char * name);
		// Set the transformation of the sensor with respect to its parent.
		void setFrame(Transform tf);
		// Start running the sensor
		void startSensor();
		// Stop running the sensor
		void stopSensor();
		// Get Lidar Data
		SensorData getData();
	};

}

#endif