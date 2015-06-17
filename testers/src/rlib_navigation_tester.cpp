// Standard Libraries
#include <iostream>
#include <math.h>
#include <string>
#include <stdlib.h>
#include <pthread.h>
#include <fstream>

// AL Libraries
// #include <alerror/alerror.h>
// #include <alproxies/almotionproxy.h>
// #include <alproxies/alrobotpostureproxy.h>
// #include <alproxies/almemoryproxy.h>
// #include <alproxies/alsensorsproxy.h>
// #include <alproxies/alsonarproxy.h>
// #include <alproxies/dcmproxy.h>
// #include <alproxies/albasicawarenessproxy.h>
// #include <qi/os.hpp>
// #include <alproxies/altexttospeechproxy.h>
// #include <alproxies/alredballtrackerproxy.h>

// Custom Libraries

// Robotlib Libraries
#include <rlib_nao.hpp>
#include <rlib_sensor.hpp>

int main(int argc, char* argv[]) {
  if(argc != 2){
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: test_navigation NAO_IP" << std::endl;
    exit(2);
  }

  // Create Nao robot
  rlib::Actuator* NaoBase = new rlib::NaoBasicWalker("mobile_base");

  // Create Lidar
  rlib::Sensor* Lidar = new rlib::Sensor("hokuyo");
  // Create Lidar Handler
  rlib::SensorHandler* LidarHandler = new rlib::HokuyoLidarHandlerIP(argv[1], "9002");
  // Set Handler
  Lidar->attachHandler(LidarHandler);
  // Start Lidar
  Lidar->start();


///*** Package this into the implementation object for the Robot Actuator Subclass ***///

  ///*** Initialize the Nao Proxies ***///

  // Initialize Memory Proxy to grab memory values.
  // AL::ALMemoryProxy memPrx(argv[1], 9559);
  // // Initialize Sensor Proxy to use head buttons.
  // AL::ALSensorsProxy headTouchPrx(argv[1], 9559);
  // // Initialize Sonar Proxy to initialze sonars.
  // AL::ALSonarProxy sonarPrx(argv[1], 9559);
  // // Initialize Motion Proxy for walking.
  // AL::ALMotionProxy motionPrx(argv[1], 9559);
  // // Initialize Posture Proxy for going to predefined poses.
  // AL::ALRobotPostureProxy pstrPrx(argv[1], 9559);
  // // Initialize Redball Tracking Proxy.
  // AL::ALRedBallTrackerProxy redBallPrx(argv[1], 9559);
  // // Initialize DCM Proxy, used to get time.
  // AL::DCMProxy dcm_proxy(argv[1], 9559);
  
  // ///*** Configure Proxies ***///
  // headTouchPrx.subscribe(prxHeadActivationButton);
  // sonarPrx.subscribe(prxSonarNavigation);


  // Values to store current distance measurements

  // Sonar Value keystrings (locations in memory)




  // Walk Velocity Variables
  

  
///*** ----------------------------------------- ***///

  // Disable Basic Awareness
  
  // Disable Collision Protection

  // Stand Nao
  // pstrPrx.goToPosture("Stand", 1.0f);

  // Initialize Walk

  try {

    std::cout << "Loop started." << std::endl;

    while(true){

      // Grab scan
      rlib::SensorData* scan = Lidar->getData();
      
      // Head Pressed?
      // if(memPrx.getData(prxHeadActivationButton)) robotHalted = true;

      // Should the robot stop?
      // if(!robotHalted){

        // Calculate Walk Variables
        // robotNav(ballPose, Om, Vx, rightDist, leftDist);

        // Scale speed because the laser backpack makes things unstable
        // Vx *= 0.15;

        // Constrain velocities
        // if(Vx < -1.0f) Vx = -1.0f;
        // if(Vx > 1.0f) Vx = 1.0f;
        // if(Om < -1.0f) Om = -1.0f;
        // if(Om > 1.0f) Om = 1.0f;
      
        // Update Walk
        // motionPrx.moveToward(Vx, Vy, Om);

      // }
      // else{
        // std::cout << "Robot Halted." << std::endl;
        // motionPrx.moveToward(0.0f, 0.0f, 0.0f);
      // }
    }

    Lidar->stopSensor();

  }
  // catch (const AL::ALError& e) {
  //   exit(1);
  // }
  catch(int e){
    Lidar->stopSensor();
    exit(1);
  }

  exit(0);
}
