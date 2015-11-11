/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file autoscrubber_node.cc
 * @brief autoscrubber control node
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-23
 */

#include <autoscrubber/autoscrubber.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_base_node");
  tf::TransformListener tf(ros::Duration(10));

  autoscrubber::AutoScrubber autoscrubber(&tf);

  ros::spin();

  return 0;
}
