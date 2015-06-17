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
#include <time.h>

// Robotlib Includes
#include <rlib_sensor.hpp>

namespace rlib{

	/*!
	* \brief Constructor for the SensorData class.
	* \param[in] timeStamp 		The time in seconds when the sensor data was retrieved. 
	*/
	SensorData::SensorData(float timeStamp){
		_timeStamp = timeStamp;
	}

	/*! 
	* \brief Print operator for Lidar Data.
	*/
	std::ostream& operator<<( std::ostream& os, SensorData& data){
		data.printToStream(os);
		return os;
	}	

	///*** Lidar Data Class ***///
	
	// Constructor with ranges in meters and angles in radians. Angles
	// should correspond to each range by index.
	LidarData::LidarData(float timeStamp, std::vector<float> &ranges, std::vector<float> &angles):
		SensorData(timeStamp)
		{
		// Resize matrix
		_data.resize(2,ranges.size());

		// Set the ranges into the first row of the data.
		for(int j = 0; j < ranges.size(); j++){
			_data(0,j) = ranges[j];
		}
		// Set the angles into the second row of the data.
		for(int j = 0; j < angles.size(); j++){
			_data(1,j) = angles[j];
		}
	}

	// Returns number of data elements.
	size_t LidarData::size(){
		return _data.cols();
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

	/*! 
	* \brief Print operator for Lidar Data.
	*/

	std::ostream& operator<<( std::ostream& os, LidarData& data){
		for(size_t ndx = 0; ndx < data.size(); ndx++){
			os << "( " << data._data(0,ndx) << ", " << data._data(1,ndx) << ")";
		}

		return os;
	}

	/*!
	* \brief Print function for streams.
	* \param[in] os 	The output stream that the data should be printed to.
	*/
	void LidarData::printToStream(std::ostream& os){
		for(size_t ndx = 0; ndx < this->size(); ndx++){
			os << "( " << this->_data(0,ndx) << ", " << this->_data(1,ndx) << ")";
		}
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

	// Start sensor
	void Sensor::start(){
		// Call our implementation object to do the work for us.
		_handler->start();
	}

	// Stop sensor
	void Sensor::stop(){
		// Call our implementation object to do the work for us.
		_handler->stop();
	}

	// Get sensor data
	void Sensor::update(){
		// Get the data from the sensor via the handler.
		_data = _handler->getData();
	}

	// Return retrieved sensor data
	SensorData* Sensor::getData(){
		// Get the current sensor data.
		return _data;
	}

	/*!
	* \brief Sets the SensorHandler for this sensor.
	* \param[in] handler 	The SensorHandler object that interacts with the physical sensor.
	*/
	void Sensor::attachHandler(SensorHandler* handler){
		_handler = handler;
	}

	///*** Sensor Handler Class ***///

	// Default Constructor
	SensorHandler::SensorHandler(){

	}

	///*** HokuyoLidarHandlerIP Class ***///

	// Default Hokuyo Constructor.
	HokuyoLidarHandlerIP::HokuyoLidarHandlerIP(const char *ip, const char *port){
		// Create the socket object that the laser client uses to communicate with the
		// Hokuyo using the io service object from the class.
  		_laserSocket = new boost::asio::ip::tcp::socket(_laserIOServ);

  		// Create the laser client object.
  		// Currently there is no mechanism to check sucessful object creation.
  		_laserClient = new hokuyo::tcp_client(ip, port, *_laserSocket);
	}

	// Start sensor
	void HokuyoLidarHandlerIP::start(){
		// Start the Lidar
		_laserClient->start();
	}

	// Stop sensor
	void HokuyoLidarHandlerIP::stop(){
		// Start the Lidar
		_laserClient->stop();
	}

	// Grab the Lidar data.
	SensorData* HokuyoLidarHandlerIP::getData(){
		// Request new laser data
		_laserClient->request();

		// Wait until we get Lidar data.
		while(!_laserClient->valid());

		// Get time stamp.
		float timeStamp = ((float)clock())/CLOCKS_PER_SEC;

		// Grab the data.
		std::vector<float> ranges, angles;
		ranges = _laserClient->get_distances();
		angles = _laserClient->get_angles();

		// Remove spurious readings
		for(int ndx = ranges.size() - 1; ndx > -1; ndx--){
			// Check to see if the range is outside of the bounds.
			if(	(ranges[ndx] > _laserClient->get_max_range()) || 
				(ranges[ndx] < _laserClient->get_min_range()))
			{
				// If so, remove it.
				ranges.erase(ranges.begin() + ndx);
				angles.erase(angles.begin() + ndx);
			}
		}

		// Put the data into a LidarData object and return it.
		return new LidarData(timeStamp, ranges, angles);
	}


}