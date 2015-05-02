// Standard Libraries
#include <iostream>
#include <math.h>
#include <string>

// AL Libraries
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsensorsproxy.h>
#include <alproxies/alsonarproxy.h>
#include <alproxies/dcmproxy.h>
// #include <alproxies/albasicawarenessproxy.h>
// #include <qi/os.hpp>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alredballtrackerproxy.h>

// Custom Libraries
#include "hokuyo-client-master/hokuyo_client.hpp"
#include "bRobot.hh"
#include "robotNav.h"

// Robotlib Libraries
#include <rlib_nao.hpp>

// Thresholds
#define BALL_DISTANCE_THRESHOLD 30.0f
#define BALL_COUNTER_THRESHOLD 15

// Global variable that definitely needs to go away
double time_from_start = 0;

int main(int argc, char* argv[]) {
  if(argc != 2){
    std::cerr << "Wrong number of arguments!" << std::endl;
    std::cerr << "Usage: nao_nav1 NAO_IP" << std::endl;
    exit(2);
  }

  // Strings for Proxy Identifications
  const std::string prxHeadActivationButton = "ActivationButton";
  const std::string prxSonarNavigation = "SonarNavigation";

  // String for Button Values
  const std::string frontButton = "FrontTactilTouched";

  // Strings for Sonar Values
  const std::string sonarRight = "Device/SubDeviceList/US/Right/Sensor/Value";
  const std::string sonarLeft = "Device/SubDeviceList/US/Left/Sensor/Value";

  // Welcome message
  // const std::string phraseToSay = "Good day.";
  // AL::ALTextToSpeechProxy tts("nao.local" , 9559);
  // tts.say(phraseToSay);

  // Create Nao robot
  rlib::Actuator* NaoBase = new rlib::NaoBasicWalker("mobile_base");


///*** Package this into the rlib sensors object ***///

  // // Create Hokuyo Laser Client
  // boost::asio::io_service laserIOServ;
  // boost::asio::ip::tcp::socket laserSocket(laserIOServ);
  // // hokuyo::tcp_client laserClient("128.238.31.109", "9002", laserSocket);
  // hokuyo::tcp_client laserClient(argv[1], "9002", laserSocket);
  // laserClient.start();
  // // Stop it later
  // std::cout << "Laser initialized." << std::endl;

///*** ----------------------------------------- ***///

///*** Package this into the implementation object for the Robot Actuator Subclass ***///

  ///*** Initialize the Nao Proxies ***///

  // Initialize Memory Proxy to grab memory values.
  AL::ALMemoryProxy memPrx(argv[1], 9559);
  // Initialize Sensor Proxy to use head buttons.
  AL::ALSensorsProxy headTouchPrx(argv[1], 9559);
  // Initialize Sonar Proxy to initialze sonars.
  AL::ALSonarProxy sonarPrx(argv[1], 9559);
  // Initialize Motion Proxy for walking.
  AL::ALMotionProxy motionPrx(argv[1], 9559);
  // Initialize Posture Proxy for going to predefined poses.
  AL::ALRobotPostureProxy pstrPrx(argv[1], 9559);
  // Initialize Redball Tracking Proxy.
  AL::ALRedBallTrackerProxy redBallPrx(argv[1], 9559);
  // Initialize DCM Proxy, used to get time.
  AL::DCMProxy dcm_proxy(argv[1], 9559);
  
  ///*** Configure Proxies ***///
  headTouchPrx.subscribe(prxHeadActivationButton);
  sonarPrx.subscribe(prxSonarNavigation);


  // Values to store current distance measurements

  // Sonar Value keystrings (locations in memory)




  // Walk Velocity Variables
  

  
///*** ----------------------------------------- ***///

  // Disable Basic Awareness
  
  // Disable Collision Protection

  // Stand Nao
  pstrPrx.goToPosture("Stand", 1.0f);

  // Initialize Red Ball Tracker
  redBallPrx.startTracker();
  // Initialize Red Ball variables
  // Delete this, this is terrible
  std::vector<float> ballPose;
  float ballMag = 1000.0f;
  int ballCounter = 0;
  ballPose.push_back(1000.0f);
  ballPose.push_back(0.0f);
  ballPose.push_back(0.0f);

  // Initialize Walk

  // Initialize Time varibles, which again, are terrible and we should get rid of them
  int t1 = dcm_proxy.getTime(0);    
  int t2 = 0;

  // Make some varibles that we should get rid of
  bool robotHalted = false;
  float rightDist, leftDist, Vx, Vy, Om;

  try {
    std::cout << "Loop started." << std::endl;
    // Poll Sonars
    while(true){
      // std::cout << "Read laser." << std::endl;
      // // Request new laser data
      // laserClient.request();
      // // If we got some data then update scan, etc.
      // if(laserClient.avail()){
      //   // Print laser
      //   // std::cout << laserClient << std::endl;

      //   // Basic testing
      //   // Get the center, right, and left beams
      //   const float angle_inc = 0.36; // Beam increment in degrees
      //   const int ndxOffset = floor(45/angle_inc);
      //   const int centerNdx = floor(laserClient.size()/2);

      //   // Grab the beams
      //   float center_beam, left_beam, right_beam;
      //   center_beam = laserClient.get_distance(centerNdx);
      //   left_beam = laserClient.get_distance(centerNdx + ndxOffset);
      //   right_beam = laserClient.get_distance(centerNdx - ndxOffset);

      //   // Print beams
      //   // std::cout << left_beam << " " << center_beam << " " << right_beam << std::endl;
      // }

      // Head Pressed?
      // if(memPrx.getData(prxHeadActivationButton)) robotHalted = true;

      // Should the robot stop?
      if(!robotHalted){

        // Update Time variable things
        int t3 = t2;
        t2 = dcm_proxy.getTime(0);      
        time_from_start = (t2-t1)/1000.0;

        // Update Ball Pose
        if(redBallPrx.isNewData()){
          ballPose = redBallPrx.getPosition();
          // Convert to Centimeters
          ballPose[0] *= 100;
          ballPose[1] *= 100;
          ballPose[2] *= 100;

          ballMag = sqrt(pow(ballPose[0],2) + pow(ballPose[1],2));
        }

        // Get Distances in centimeters
        rightDist = 100*float(memPrx.getData(sonarRight));
        leftDist = 100*float(memPrx.getData(sonarRight));
        
        // Calculate Walk Variables
        robotNav(ballPose, Om, Vx, rightDist, leftDist);

        // Check to see if the ball is close enough
        if(ballMag < BALL_DISTANCE_THRESHOLD){
          // Check to see if we've seen the ball enough times
          if(ballCounter > BALL_COUNTER_THRESHOLD){
            Vx = 0;
            Om = 0;
            robotHalted = true;
            std::cout << std::endl << " GOAL REACHED " << std::endl;
          }
          
          ballCounter++;
          std::cout << " Ball Counter: " << ballCounter;
        }

        // Scale speed because the laser backpack makes things unstable
        Vx *= 0.15;

        // Constrain velocities
        if(Vx < -1.0f) Vx = -1.0f;
        if(Vx > 1.0f) Vx = 1.0f;
        if(Om < -1.0f) Om = -1.0f;
        if(Om > 1.0f) Om = 1.0f;
      
        // Update Walk
        motionPrx.moveToward(Vx, Vy, Om);

      }
      else{
        std::cout << "Robot Halted." << std::endl;
        motionPrx.moveToward(0.0f, 0.0f, 0.0f);
        redBallPrx.stopTracker();
      }
    }

  }
  catch (const AL::ALError& e) {
    exit(1);
  }

  exit(0);
}
