/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file autoscrubber_control.cpp
 * @brief autoscrubber op node
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-05-04
 */

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "autoscrubber_control");

  ros::NodeHandle n;
  n.setParam("/first_start", true);

  ros::spin();
  return 0;
}
