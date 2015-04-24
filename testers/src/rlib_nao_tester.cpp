// Robot Library v0.1
// This is a library for contructing robot objects for
// doing basic tasks such as sensing, navigation, and planning.
// 
// This is a unit tester for the Nao library.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Standard Libraries
#include <iostream>

// Rlib Libraries
#include <rlib_nao.hpp>


int main(int argc, char* argv[]) {

	rlib::Actuator* NaoBase = new rlib::NaoBasicWalker("mobile_base");

	std::cout << "Loop started." << std::endl;

	while(true){

    }

}
