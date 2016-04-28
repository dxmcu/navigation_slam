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

#ifdef VALIDATE_USB
#include <security/security_client.h>
#include <security/usb_validator.h>
void SecurityCallback(int ret) {
  if (0 != ret) exit(-1);
}
#endif

int main(int argc, char** argv) {
#ifdef VALIDATE_USB
  // start usb validator
  security::SecurityValidator* validator = new security::USBValidator(SecurityCallback);
  security::SecurityClient security_client(validator);
  security_client.Start();
#endif
  ros::init(argc, argv, "move_base_node");
  tf::TransformListener tf(ros::Duration(10));

  service_robot::ServiceRobot service_robot(&tf);

  ros::spin();

#ifdef VALIDATE_USB
  security_client.Stop();
#endif

  return 0;
}
