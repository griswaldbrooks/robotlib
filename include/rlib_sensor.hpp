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
#include <ostream>

// Special Includes
#include <eigen3/Eigen/Core>
#include "hokuyo_client.hpp"

// Robot lib Includes
#include <rlib_state.hpp>
#include <rlib_actuator.hpp>

/*! \brief Robotlib Namespace */
namespace rlib{

	/*! \brief SensorData class for getting data from Sensors. */
	/**
	* Abstract class for encapsulating sensor data.
	*/
	class SensorData{
	public:
		/*!
		* \brief Constructor for the SensorData class.
		* \param[in] timeStamp 		The time in seconds when the sensor data was retrieved. 
		*/
		SensorData(float timeStamp);
		
		/*! 
		* \brief Print operator for Sensor Data.
		*/
		friend std::ostream& operator<<( std::ostream& os, SensorData& data);

		/*!
		* \brief Print function for streams.
		* \param[in] os 	The output stream that the data should be printed to.
		*/
		virtual void printToStream(std::ostream& os) = 0;

	private:
		float _timeStamp; /**< Time in seconds when this data was generated. */

	};

	class SonarData : public SensorData{};

	class InfraredData : public SensorData{};

	class ImageData : public SensorData{};

	class PointCloudData : public SensorData{};

	/*! \brief LidarData class for getting data from a Lidar Sensor. */
	/** 
	* This class is for encapsulating the data produced by a Lidar Sensor and making it easy
	* to access the data in different ways.
	*/
	class LidarData : public SensorData{
	public:
		// Constructor with ranges in meters and angles in radians. Angles
		// should correspond to each range by index.
		/*!
		* \brief Constructor for LidarData objects.
		*
		* The constructor for the needs a std::vector of ranges and angles. Each vector must be the same length
		* and the indices must correspond with each others data. The vectors do not need to be sorted by angle,
		* and in general are not. 
		* \param[in] timeStamp 	The system time in seconds when the data was retrieved.
		* \param[in] ranges		This is the IP address of the computer that the Lidar is connected to.
		* \param[in] angles		This is the port number of the Lidar on the computer.
		*/
		LidarData(float timeStamp, std::vector<float> &ranges, std::vector<float> &angles);

		/*! 
		* \brief Returns number of data elements.
		* \return 	The number of range angle pairs in the set.
		*/
		size_t size();

		/*!
		* \brief Get a range for a given angle. If no angle is found, method will return range to the closest angle.
		* \param[in] angle 		The angle in radians to get a range for.
		* \return 				The range in meters.
		*/
		float getRange(float angle);

		/*!
		* \brief Get the sensor data.
		* \return 	A 2x2 Matrix of floats of ranges and angles in meters and radians. The ranges are on the
		*			first row and their corresponding angles are on the second row.
		*/
		const Eigen::Matrix<float, 2, Eigen::Dynamic> & getData();

		/*! 
		* \brief Print operator for Lidar Data.
		*/
		friend std::ostream& operator<<( std::ostream& os, LidarData& data);

		/*!
		* \brief Print function for streams.
		* \param[in] os 	The output stream that the data should be printed to.
		*/
		void printToStream(std::ostream& os);

	private:
		Eigen::Matrix<float, 2, Eigen::Dynamic> _data; /**< Lidar data in pair format. First element in pair corresponds to range in
															meters. Second element in pair in the angle of that range in radians. */
	};

	/*! \brief Abstract class for all Sensor Handler objects. */
	/** 
	* This is the abstract class for building all Sensor Handlers. These objects are meant
	* to deal with the implementation details of actually getting sensor data. The Sensor
	* Handler is kept by the Sensor object and uses it to get sensor data. 
	*/
	class SensorHandler{
	public:
		/*! \brief Default Handler constructor. */
		SensorHandler();
		
		/*!
		* \brief Initializes and starts sensor.
		*/
		virtual void start() = 0;

		/*!	
		* \brief Stops sensor.
		*/
		virtual void stop() = 0;

		/*!	
		* \brief Gets data from the sensor.
		*
		* Requests new data from the sensor. This is a blocking function.
		* \return 		The sensor data packed into the rlib::SensorData format.
		*/
		virtual SensorData* getData() = 0;
	};

	/*! \brief Hokuyo Lidar Handler class using IP connected Lidar. */
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
		HokuyoLidarHandlerIP(const char *ip, const char *port);

		/*!
		* \brief Initiates communication with the Lidar.
		*/
		void start();

		/*!	
		* \brief Terminates Lidar connection.
		*/
		void stop();

		/*!	
		* \brief Gets range and angle data from the sensor.
		*
		* Requests new sensor data from the Lidar. This is a blocking function.
		* \return 		The range and angle data packed into the rlib::SensorData format.
		*/
		SensorData* getData();
	private:
  		boost::asio::io_service _laserIOServ;	/**< The boost io_service object that the socket object needs.*/
  		boost::asio::ip::tcp::socket * _laserSocket; /**< The boost socket object is used to communicate with the Lidar.*/
  		hokuyo::tcp_client * _laserClient; /**< The hokuyo client object that parses the Lidar data. */
	};

	/*! \brief Abstract class for creating Sensor objects. */
	/**
	* This class defines the abstract class that all Sensor derivatives must follow.
	* It is intended for handling representation details of any sensor. It must have a SensorHandler
	* in order to interact with a physical sensor.
	*/
	class Sensor{
	public:	
		/*!
		* \brief Constructor for building Sensors.
		*
		* Each sensor must have a name.
		* \param[in] name 		The name the sensor will have, used as a unique ID.
		*/
		Sensor(const char * name);
		
		/*!
		* \brief Gets the name of the sensor.
		* \return 		The name of the sensor.
		*/
		std::string getName();

		/*!
		* \brief Sets the coordinate transformation between the parent and the sensor.
		* \param[in] tf 	The tranformation that brings the parent frame to the sensor frame.
		*/
		void setFrame(Transform tf);

		/*!
		* \brief Starts the sensor.
		*/
		virtual void start();

		/*!
		* \brief Stops the sensor.
		*/
		virtual void stop();

		/*!
		* \brief Gets data from the sensor.
		*/
		virtual void update();

		/*!
		* \brief Returns sensor data from last update.
		*/
		SensorData* getData();

		/*!
		* \brief Sets the SensorHandler for this sensor.
		* \param[in] handler 	The SensorHandler object that interacts with the physical sensor.
		*/
		void attachHandler(SensorHandler* handler);

		/*!
		* \brief Says if Sensor is associated with an Actuator object.
		* \return 		True if it has an association, False otherwise.
		*/
		bool isAttached();

	private:
		std::string _name; /**< Unique identifier for the sensor.*/
		Actuator* _parent; /**< The sensor object that this sensor is a child of.*/
		Transform _tf; /**< Transformation that the parent must do on its origin to reach the origin of the child (the child is this sensor).*/
		SensorHandler* _handler; /**< Implementation (Bridge) object for interfacing with the physical sensor.*/
		SensorData* _data; /**< Holds the current data for the sensor. */
	};

}

#endif