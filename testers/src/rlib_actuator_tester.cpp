// Standard Libraries
#include <iostream>

// Robotlib Libraries
#include <rlib_actuator.hpp>
#include <rlib_state.hpp>

int main(int argc, char* argv[]) {

  // Create simple robot base
  rlib::Actuator* mobileBase1 = new rlib::MobileBase("mobile_base");

  // Set target position for it
  mobileBase1->setPosition(rlib::Pose(1, 1, 0));

}
