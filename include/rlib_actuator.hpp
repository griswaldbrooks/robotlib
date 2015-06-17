// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This library file is for constructing the actuator classes.
// One of the main ideas in this library is that actuators can be
// constructed into chains of actuators using the parent child 
// structure. If the actuator needs more than one child then 
// the robot class in the rlib_robot.cpp library can be used
// to allow multiple children and to add additional structures
// such and sensors and planners.
//
// Actuator is an abstract class to allow for the creation of 
// specific actuators with their own movement needs such as mobile
// bases, servos, and manipulators.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _RLIB_ACTUATOR_HPP
#define _RLIB_ACTUATOR_HPP

// Standard Includes
#include <string>

// Special Includes

// Robot lib Includes
#include <rlib_state.hpp>
#include <rlib_controller.hpp>

namespace rlib{

	/*!
	* \brief Abstract handler class for implementing actuators.
	*/
	/** ActuatorHandler objects are used by the Actuator class for commanding
	* actuator hardware.
	*/
	class ActuatorHandler{
	public:
		/*! \brief Default Handler constructor. */
		ActuatorHandler();
		
		/*!
		* \brief Initializes and starts actuator.
		*/
		virtual void start() = 0;

		/*!	
		* \brief Stops actuator.
		*/
		virtual void stop() = 0;

		/*!	
		* \brief Commands actuator to a position.
		*
		* Commands actuator to a position setpoint.
		* \param[in] pose 	The commanded actuator pose.
		* \return 			The pose command sent to the actuator.
		*/
		/**
		* If the pose command contains elements outside the limits of the actuator
		* then the actuator will only be commanded to setpoints within those limits.
		*/
		virtual Pose setPosition(Pose pose) = 0;

		/*!	
		* \brief Commands actuator to a velocity.
		*
		* Commands actuator to a velocity setpoint.
		* \param[in] vel 	The commanded actuator velocity.
		* \return 			The velocity command sent to the actuator.
		*/
		/**
		* If the velocity command contains elements outside the limits of the actuator
		* then the actuator will only be commanded to setpoints within those limits.
		*/
		virtual Vel setVelocity(Vel velocity) = 0;

		/*!
		* \brief Sets the pose and velocity limits of the actuator.
		* \param[in] poseMin	The minimum pose of the actuator.
		* \param[in] poseMax	The maximum pose of the actuator.
		* \param[in] velMin	 	The minimum velocity of the actuator.
		* \param[in] velMax 	The maximum velocity of the actuator.
		* \return 				True if the limit are valid.
		*/
		virtual bool setLimits(Pose poseMin, Pose poseMax, Vel velMin, Vel velMax) = 0;

	private:
		Pose _poseMax;	/**< Pose maximum limits for the actuator. */
		Pose _poseMin;	/**< Pose minimum limits for the actuator. */
		Vel _velMax;		/**< Velocity maximum limits for the actuator. */ 
		Vel _velMin;		/**< Velocity minimum limits for the actuator. */
	};

	/*!
	* \brief Abstract class for implementing actuators.
	*/
	/** Actuator objects are used to command actuators to poses.
	*/
	class Actuator{
	public:	
		/*!
		* \brief Constructor for the actuator class.
		* \param[in] name 		The name for the actuator, used as a unique identifier.
		*/
		Actuator(const char * name);
		
		/*!
		* \brief Sets the pose of the actuator.
		* \param[in] pose 	 The pose of the actuator with respect to its origin.
		*/
		virtual void setPosition(Pose pose);

		/*!
		* \brief Sets the velocity of the actuator.
		* \param[in] pose 	 The velocity of the actuator in the local frame.
		*/
		virtual void setVelocity(Vel velocity);

		/*!
		* \brief Assigns the child actuator in the kinematic chain.
		* \param[in] child 	Pointer to the child actuator.
		* \param[in] tf 	The coordinate transformation that beings the parent
		*					frame to the child frame.
		* \return 			True if the actuator could be attached. 
		*					False if it could not. Likely because it has not been
		*					properly detached.
		*/
		virtual bool attachActuator(Actuator* child, Transform tf);

		/*!
		* \brief Disassociate the child actuator from the parent.
		*/
		virtual void detachActuator();

		/*!
		* \brief Get pointer to the child actuator.
		* \return Actuator pointer to the child actuator.
		*/
		// Maybe replace this with iterator?
		Actuator* next();

		/*!
		* \brief Says if Actuator is associated with another object.
		* \return 		True if it has an association, False otherwise.
		*/
		bool isAttached();

		/*!
		* \brief Sets the ActuatorHandler for this actuator.
		* \param[in] handler 		The ActuatorHandler object that interacts with the physical actuator.
		*/
		void attachHandler(ActuatorHandler* handler);

		/*!
		* \brief Gets the actuator unique identifier.
		* \return The name of the actuator.
		*/
		std::string getName();

	protected:
		Actuator* _parent; 	/**< The actuator object that this actuator is a child of. */
		Actuator* _child; 	/**< The actuator object that is the child of this one. */
		Transform _tf; 		/**< Transformation that the parent must do on its origin to
										reach the origin of the child (the child is this actuator). */
	private:
		Pose _pose; 		/**< 6D description of actuator position with respect to the actuator origin. */
		Vel _velocity; 		/**< 6D description of actuator velocity with respect to the actuator origin. */
	
		std::string _name; 			/**< Unique identifier for the actuator. */
		Controller* _controller; 	/**< Object that controls the actuator. */
		ActuatorHandler* _handler; 	/**< Implementation (Bridge) object for interfacing with the physical actuator.*/	
	};

	/* Some basic derivatives */

	// Mobile base class that takes linear velocity and angular velocity commands.
	class MobileBase : public Actuator{
	public:
		MobileBase(const char * name);
		// Set the pose of the actuator with respect to its origin.
		void setPosition(Pose pose);
		// Set the velocity of the actuator in the local frame.
		void setVelocity(Vel velocity);
	};

}

#endif