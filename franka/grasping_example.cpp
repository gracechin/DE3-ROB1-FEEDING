// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>

/**
 * @example grasp_object.cpp
 * An example showing how to control FRANKA's gripper.
 */

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: ./grasp_object <gripper-hostname> <homing> <object-width>" << std::endl;
    return -1;
  }

  try {
    // gripper documentation: https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html
    franka::Gripper gripper(argv[1]);
    double grasping_width = std::stod(argv[3]);

    std::stringstream ss(argv[2]);

    // Note: homing has to be done only once so to know for the fingers we use what is the maximum width
    // bool homing;
    // if (!(ss >> homing)) {
    //   std::cerr << "<homing> can be 0 or 1." << std::endl;
    //   return -1;
    // }

    // if (homing) {
    //   // Do a homing in order to estimate the maximum grasping width with the current fingers.
    //   gripper.homing();
    // }

    // Check for the maximum grasping width.
    // franka::GripperState gripper_state = gripper.readOnce();
    // if (gripper_state.max_width < grasping_width) {
    //   std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
    //   return -1;
    // }

    // Grasp the object.
    // grasp documentation: https://frankaemika.github.io/libfranka/classfranka_1_1Gripper.html#a19b711cc7eb4cb560d1c52f0864fdc0d
    // bool franka::Gripper::grasp ( double  width,
    // double  speed,
    // double  force,
    // double  epsilon_inner = 0.005,
    // double  epsilon_outer = 0.005 
    // )   

    if (!gripper.grasp(grasping_width, 0.1, 60)) {  
    
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }

    // Wait 3s and check afterwards, if the object is still grasped.
    // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

    // gripper state documentation: https://frankaemika.github.io/libfranka/structfranka_1_1GripperState.html
    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {  // returns true if grasping_width-epsilon_inner < gripper_state.width < grasping_width-epsilon_outer
      std::cout << "Object lost." << std::endl;
      return -1;
    }

    // std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();  // releasing the object
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}