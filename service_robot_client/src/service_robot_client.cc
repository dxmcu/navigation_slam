/* Copyright(C) Gaussian Automation. All rights reserved.
 */

#include <ros/ros.h>
#include <autoscrubber_services/Start.h>
#include <autoscrubber_services/Pause.h>
#include <autoscrubber_services/Resume.h>
#include <autoscrubber_services/Terminate.h>

#include <iostream>

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "args not enough..." << std::endl;
    exit(-1);
  }

  ros::init(argc, argv, "service_robot_client");
  ros::NodeHandle service_robot_handle("service_robot_move_base");

  autoscrubber_services::Start start;
  autoscrubber_services::Pause pause;
  autoscrubber_services::Resume resume;
  autoscrubber_services::Terminate terminate;
  ros::ServiceClient start_client = service_robot_handle.serviceClient<autoscrubber_services::Start>("start");
  ros::ServiceClient pause_client = service_robot_handle.serviceClient<autoscrubber_services::Pause>("pause");
  ros::ServiceClient resume_client = service_robot_handle.serviceClient<autoscrubber_services::Resume>("resume");
  ros::ServiceClient terminate_client = service_robot_handle.serviceClient<autoscrubber_services::Terminate>("terminate");

  if (0 == strcmp(argv[1], "start")) {
    start_client.call(start);
  } else if (0 == strcmp(argv[1], "pause")) {
    pause_client.call(pause);
  } else if (0 == strcmp(argv[1], "resume")) {
    resume_client.call(resume);
  } else if (0 == strcmp(argv[1], "terminate")) {
    terminate_client.call(terminate);
  }

  ros::spinOnce();
  return 0;
}
