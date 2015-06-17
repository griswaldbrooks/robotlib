// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the basic robot.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Robotlib Includes
#include <rlib_robot.hpp>

namespace rlib{

	/*!
	* \brief Constructor for the Robot class.
	* \param[in] name 		The name of the robot, used as a unique identifier.
	*/
	Robot::Robot(const char * name):Actuator(name){
		
	}

	/*!
	* \brief Sets the pose of the robot.
	* \param[in] pose 	 The pose of the robot with respect to its origin.
	*/
	void Robot::setPosition(Pose pose){
		// _pose = _planner->setPosition(pose);
	}

	/*!
	* \brief Sets the velocity of the robot.
	* \param[in] pose 	 The velocity of the robot in the local frame.
	*/
	void Robot::setVelocity(Vel velocity){
		// _velocity = _planner->setVelocity(velocity);
	}

	/*!
	* \brief Assigns the child actuator in the kinematic chain.
	* \param[in] child 	Pointer to the child actuator.
	* \param[in] tf 	The coordinate transformation that beings the parent
	*					frame to the child frame.
	* \return 			True if the actuator could be attached. 
	*					False if it could not. Likely because it has not been
	*					properly detached.
	*/
	bool Robot::attachActuator(Actuator* child, Transform tf){
		// Check to see if the child already has a parent.
		if(child->isAttached()) return false;
		else{
			// If not, then add it to the list.
			_actuators.push_back(child);
		}
	}

	/*!
	* \brief Disassociate the child actuator from the parent.
	* \param[in] child 	Pointer to the child to be removed.
	*/
	void Robot::detachActuator(Actuator* child){
		// Probably use the std::list::remove function but I'm not sure if I should be 
		// calling the destructor of the object.
	}

	/*!
	* \brief Attaches a Sensor to the Robot. The Robot will manage the updating
	* of this Sensor.
	* \param[in] sensor	Pointer to the Sensor.
	* \param[in] tf 	The coordinate transformation that beings the Robot
	*					frame to the Sensor frame.
	* \return 			True if the Sensor could be attached. 
	*					False if it could not. Likely because it has not been
	*					properly detached.
	*/
	bool attachSensor(Sensor* sensor, Transform tf){

	}

	/*!
	* \brief Disassociate the Sensor from the Robot.
	* \param[in] sensor 	Pointer to the Sensor to be removed.
	*/
	void detachSensor(Sensor* sensor){

	}

	/*!
	* \brief Sets the Controller for this actuator.
	* \param[in] controller 	The Controller object that generates controller commands for the actuator.
	* \return 					True if the Observer could be attached. 
	*							False if it could not. Likely because it has not been
	*							properly detached.
	*/
	bool attachController(Controller* controller){

	}

	/*!
	* \brief Attaches an Observer to the Robot. The Robot will manage the updating
	* of this Observer.
	* \param[in] observer 	Pointer to the Observer.
	* \return 				True if the Observer could be attached. 
	*						False if it could not. Likely because it has not been
	*						properly detached.
	*/
	bool attachObserver(Observer* observer){

	}

	/*!
	* \brief Commands robot to read from sensors and update actuator commands.
	*/
	// Mediator function.
	void Robot::run(){
		// Update sensors.
		std::list<Sensor*>::iterator sensors;
		for(sensors = _sensors->begin(); sensors != _sensors->end(); sensors++){
			sensors->update();
		}

		// Update reference commands.
		Pose next_pose_ref = _planner->getNextPose();
		Vel  next_vel_ref  = _planner->getNextVel();

		// Update actuators.
		std::list<Actuator*>::iterator actuators;
		for(actuators = _actuators->begin(); actuators != _actuators->end(); actuators++){
			actuators->update();
		}

	}

}