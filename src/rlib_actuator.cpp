// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the actuator classes.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Standard Includes
#include <iostream>
#include <limits>

// Robotlib Includes
#include <rlib_actuator.hpp>

namespace rlib{

	/*! \brief Default Handler constructor. */
	ActuatorHandler::ActuatorHandler(){
		// Set the default limits to no limits.
		_poseMin = Pose(-std::numeric_limits<float>::infinity(), 
						-std::numeric_limits<float>::infinity(), 
						-std::numeric_limits<float>::infinity(), 
						-std::numeric_limits<float>::infinity(), 
						-std::numeric_limits<float>::infinity(), 
						-std::numeric_limits<float>::infinity()
						);
		_poseMax = Pose(std::numeric_limits<float>::infinity(), 
						std::numeric_limits<float>::infinity(), 
						std::numeric_limits<float>::infinity(), 
						std::numeric_limits<float>::infinity(), 
						std::numeric_limits<float>::infinity(), 
						std::numeric_limits<float>::infinity()
						);
		
		_velMin.vx = -std::numeric_limits<float>::infinity();
		_velMin.vy = -std::numeric_limits<float>::infinity();
		_velMin.vz = -std::numeric_limits<float>::infinity();
		_velMin.wx = -std::numeric_limits<float>::infinity();
		_velMin.wy = -std::numeric_limits<float>::infinity();
		_velMin.wz = -std::numeric_limits<float>::infinity();

		_velMax.vx = std::numeric_limits<float>::infinity();
		_velMax.vy = std::numeric_limits<float>::infinity();
		_velMax.vz = std::numeric_limits<float>::infinity();
		_velMax.wx = std::numeric_limits<float>::infinity();
		_velMax.wy = std::numeric_limits<float>::infinity();
		_velMax.wz = std::numeric_limits<float>::infinity();
	}
	
	// Constructor for actuator class.
	Actuator::Actuator(const char * name):_name(name){
		// More things to construct.
		_parent = NULL;
		_child = NULL;
		_tf.T.setIdentity();
	}

	// Return name of actuator	
	std::string Actuator::getName(){
		return _name;
	}

	// Set the pose of the actuator with respect to its origin.
	void Actuator::setPosition(Pose pose){
		// Call handler.
		_pose = _handler->setPosition(pose);
	}

	// Set the velocity of the actuator in the local frame.
	void Actuator::setVelocity(Vel velocity){
		// Call handler. 
		_velocity = _handler->setVelocity(velocity);
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
	bool Actuator::attachActuator(Actuator* child, Transform tf){
		// Check to see if there is a current child or the prospective child has
		// a parent. If so, fail to attach.
		if(	(_child) ||
			( child->_parent)
			)
		{
			return false;
		}
		else{
			// Set child and its transform.
			_child = child;
			_child->_tf = tf;
			// Set the parent of the child.
			_child->_parent = this;
			return true;
		}
	}

	/*!
	* \brief Assigns the child actuator in the kinematic chain.
	* \param[in] child 	Pointer to the child actuator.
	*/
	void Actuator::detachActuator(){
		if(_child){
			// Detach the parent.
			_child->_parent = NULL;
			// Reset the transform.
			_child->_tf.T.setIdentity();
			// Detach child.
			_child = NULL;
		}
	}

	/*!
	* \brief Get pointer to the child actuator.
	* \return Actuator pointer to the child actuator.
	*/
	// Maybe replace this with iterator?
	Actuator* Actuator::next(){
		return _child;
	}

	/*!
	* \brief Says if Actuator is associated with another object.
	* \return 		True if it has an association, False otherwise.
	*/
	bool Actuator::isAttached(){
		return (bool)(_parent);
	}

	MobileBase::MobileBase(const char * name): Actuator(name){
		// Do more constructory things.
		std::cout << "Mobile Base " << getName() << " constructed." << std::endl;
	}

	// Set the pose of the actuator with respect to its origin.
	void MobileBase::setPosition(Pose pose){
		// Call default.
		Actuator::setPosition(pose);
	}

	// Set the velocity of the actuator in the local frame.
	void MobileBase::setVelocity(Vel velocity){
		// Call default.
		Actuator::setVelocity(velocity);
	}
}