/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file base_controller.h
 * @brief base class of all controllers
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#ifndef SERVICEROBOT_INCLUDE_SERVICEROBOT_BASE_CONTROLLER_H_
#define SERVICEROBOT_INCLUDE_SERVICEROBOT_BASE_CONTROLLER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <string>

namespace service_robot {

typedef struct _ctrl_environment_t {
  bool launch_scrubber;
  bool run_flag;
  bool pause_flag;
} ControlEnvironment;

typedef struct _base_ctrl_option_t {
  tf::TransformListener* tf;

  costmap_2d::Costmap2DROS* planner_costmap_ros;
  costmap_2d::Costmap2DROS* controller_costmap_ros;

  std::string robot_base_frame;
  std::string global_frame;

  double planner_frequency;
  double controller_frequency;
  double inscribed_radius;
  double planner_patience;
  double controller_patience;
  double oscillation_timeout;
  double oscillation_distance;

  ros::Publisher* vel_pub;

  _base_ctrl_option_t(tf::TransformListener* tf,
                      costmap_2d::Costmap2DROS* planner_costmap_ros,
                      costmap_2d::Costmap2DROS* controller_costmap_ros,
                      const std::string& robot_base_frame, const std::string& global_frame,
                      double planner_frequency, double controller_frequency, double inscribed_radius,
                      double planner_patience, double controller_patience, double oscillation_timeout,
                      double oscillation_distance, ros::Publisher* vel_pub)
    : tf(tf), planner_costmap_ros(planner_costmap_ros), controller_costmap_ros(controller_costmap_ros),
      robot_base_frame(robot_base_frame), global_frame(global_frame),
      planner_frequency(planner_frequency), controller_frequency(controller_frequency), inscribed_radius(inscribed_radius),
      planner_patience(planner_patience), controller_patience(controller_patience),
      oscillation_timeout(oscillation_timeout), oscillation_distance(oscillation_distance), vel_pub(vel_pub) { }
} BaseControlOption;

class BaseController {
 public:
  virtual bool Control(BaseControlOption* option, ControlEnvironment* environment) = 0;
};

};  // namespace service_robot

#endif  // SERVICEROBOT_INCLUDE_SERVICEROBOT_BASE_CONTROLLER_H_
