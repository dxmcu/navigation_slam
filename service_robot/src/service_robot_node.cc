/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file service_robot_node.cc
 * @brief service_robot control node
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-23
 */

#include <service_robot/service_robot.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_base_node");
  tf::TransformListener tf(ros::Duration(10));

  service_robot::ServiceRobot service_robot(&tf);

  ros::spin();

  return 0;
}
