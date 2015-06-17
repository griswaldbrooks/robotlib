# Robotlib
C++ Library that contains many useful constructions for writing robot centric programs.

TODO: There will be more text here about the different structures and whatnot.

The point of the library is to have flexible and reusable robot objects that can be used for many applications. 
In short, I'm trying to rewrite as little code as possible when I do my robot projects and I'm sharing my results.

Currently the library is written in C++ and tested on Ubuntu.

The intent is that the developer builds a robot in code similar to how one might build a robot in hardware.
To facilitate constructing the robot there are four main abstract classes to use:

Actuator
Sensor
Controller
Observer

As you might expect, the Actuator class is an object that encapsulates the main ideas of actuators. They have commands for going places and doing things that would be common to most actuators. They also maintain references to a parent actuator and a child actuator in order to form simple kinematic chains. Each actuator maintains a homogeneous transformation that maps its origin to the origin of its parent.

TODO: Talk a bit about Sensors

TODO: Talk a bit about Controllers

TODO: Talk a bit about Observers

More specific properties of actuators, sensors, controllers, and observers should be subclassed to add the specific needs of that object.

In order to bring these objects together to do useful things, the Actuator class has been subclassed to make the Robot class. This class holds references to lists of actuators, sensors, controllers, and observers. It is a subclass of Actuator because the Robot class is meant to be able to hold references to other robots that it can command. For example, with a mobile manipulator the robot might hold a reference to a mobile base that it can command, but then it will likely hold a reference to a robot arm that will have its own controllers and sensors that the mobile manipulator can command as if it were a simple actuator. 

........

Robots contain Actuators, Sensors, Controllers, and Observers.

Any component can have only one parent, ie only one Robot object is allowed to update it.

Robots manage all of the components in their composition such as when things get updated.
	Sensors for example only get updated once per update.
	Only one Robot should manage/update any one Actuator or Sensor, though in the case
	of Sensors, many components should be able to read them.

Observers can be read by many components, but only updated by its Robot. Each Sensor should notify
its Observer when there is new data to retrieve.

Robots are subclasses of Actuators so they can be commanded in the same way.

Controllers and Observers are highly custom objects which can attach any object they want
	to themselves. Controllers might have multiple Actuators, Sensors, and Observers.
	Controllers command Actuators with either Pose or Velocity commands, though it the future there
	should be a provision for torque.

