// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the basic robot.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _RLIB_ROBOT_HPP
#define _RLIB_ROBOT_HPP

// Standard Includes
#include <list>
// Special Includes

// Robot lib Includes
#include <rlib_state.hpp>
#include <rlib_actuator.hpp>
#include <rlib_sensor.hpp>
#include <rlib_observer.hpp>
#include <rlib_controller.hpp>

namespace rlib{

	/*! 
	* \brief Class for aggregating rlib objects.
	*/
	/**
	* The Robot class is intended to aggregate all of the other types of objects
	* together into a collection that can work together. It inherits from the Actuator
	* interface so that it can be commanded like an Actuator. For example, the Robot class
	* contains a list of Actuators but since Robot derives from this class Robots can hold
	* other Robots. A mobile base might hold an arm and so on.
	*/
	class Robot: public Actuator{
	public:	
		/*!
		* \brief Constructor for the Robot class.
		* \param[in] name 		The name of the robot, used as a unique identifier.
		*/
		Robot(const char * name);

		/*!
		* \brief Sets the pose of the robot.
		* \param[in] pose 	 The pose of the robot with respect to its origin.
		*/
		void setPosition(Pose pose);

		/*!
		* \brief Sets the velocity of the robot.
		* \param[in] pose 	 The velocity of the robot in the local frame.
		*/
		void setVelocity(Vel velocity);

		/*!
		* \brief Attaches an Actuator to the Robot. Actuators attached to Robots are
		* not in a kinematics chain but are rather part of the set of Actuators for the
		* Robot to manage.
		* \param[in] child 	Pointer to the child actuator.
		* \param[in] tf 	The coordinate transformation that beings the parent
		*					frame to the child frame.
		* \return 			True if the actuator could be attached. 
		*					False if it could not. Likely because it has not been
		*					properly detached.
		*/
		bool attachActuator(Actuator* child, Transform tf);

		/*!
		* \brief Disassociate the child actuator from the parent.
		* \param[in] child 	Pointer to the child to be removed.
		*/
		void detachActuator(Actuator* child);

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
		bool attachSensor(Sensor* sensor, Transform tf);

		/*!
		* \brief Disassociate the Sensor from the Robot.
		* \param[in] sensor 	Pointer to the Sensor to be removed.
		*/
		void detachSensor(Sensor* sensor);

		/*!
		* \brief Sets the Controller for this actuator.
		* \param[in] controller 	The Controller object that generates controller commands for the actuator.
		* \return 					True if the Observer could be attached. 
		*							False if it could not. Likely because it has not been
		*							properly detached.
		*/
		bool attachController(Controller* controller);

		/*!
		* \brief Attaches an Observer to the Robot. The Robot will manage the updating
		* of this Observer.
		* \param[in] observer 	Pointer to the Observer.
		* \return 				True if the Observer could be attached. 
		*						False if it could not. Likely because it has not been
		*						properly detached.
		*/
		bool attachObserver(Observer* observer);

		/*!
		* \brief Commands robot to read from sensors and update actuator commands.
		*/
		void run();

	private:
		// Should these be lists or some other datatype?
		std::list<Actuator*> _actuators; 	/**< List of actuators that belong to the robot. */
		std::list<Sensor*> _sensors;		/**< List of sensors that belong to the robot. */
		std::list<Observer*> _observers; 	/**< List of observers that belong to the robot. */
		Controller* _controller; 			/**< Object that controls the actuator. */
	};

}

#endif