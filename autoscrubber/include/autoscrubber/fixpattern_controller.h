/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file fixpattern_controller.h
 * @brief fixpattern controller flow
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#ifndef AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_FIXPATTERN_CONTROLLER_H_
#define AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_FIXPATTERN_CONTROLLER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <fixpattern_path/path.h>
#include <fixpattern_local_planner/trajectory_planner_ros.h>
#include <string>
#include <vector>

#include "autoscrubber/base_controller.h"
#include "autoscrubber/footprint_checker.h"

namespace autoscrubber {

typedef enum {
  F_CONTROLLING = 0,
  F_CLEARING    = 1
} FixPatternState;

typedef enum {
  F_CONTROLLING_R = 0,
  F_OSCILLATION_R = 1,
  F_FRONTSAFE_R   = 2
} FixPatternRecoveryTrigger;

struct FixPatternControlOption : BaseControlOption {
  double stop_duration;
  double max_offroad_dis;
  double front_safe_check_dis;
  int* fixpattern_reached_goal;
  double fixpattern_footprint_padding;
  fixpattern_path::Path* fixpattern_path;
  std::vector<geometry_msgs::Point> circle_center_points;
  boost::shared_ptr<fixpattern_local_planner::FixPatternTrajectoryPlannerROS> fixpattern_local_planner;
  geometry_msgs::PoseStamped* global_planner_goal;
  FixPatternControlOption(tf::TransformListener* tf,
                          costmap_2d::Costmap2DROS* planner_costmap_ros,
                          costmap_2d::Costmap2DROS* controller_costmap_ros,
                          const std::string& robot_base_frame, const std::string& global_frame,
                          double planner_frequency, double controller_frequency, double inscribed_radius,
                          double planner_patience, double controller_patience, double oscillation_timeout,
                          double oscillation_distance, ros::Publisher* vel_pub)
    : BaseControlOption(tf, planner_costmap_ros, controller_costmap_ros,
                        robot_base_frame, global_frame,
                        planner_frequency, controller_frequency, inscribed_radius,
                        planner_patience, controller_patience, oscillation_timeout,
                        oscillation_distance, vel_pub) { }
};

class FixPatternController : public BaseController {
 public:
   /**
    * @brief Constructor for the controller
    *
    * @param tf A pointer to a TransformListener
    * @param planner_costmap_ros A pointer to a Costmap2DROS of global frame
    * @param controller_costmap_ros A pointer to a Costmap2DROS of local frame
    */
  FixPatternController(tf::TransformListener* tf,
                       costmap_2d::Costmap2DROS* planner_costmap_ros,
                       costmap_2d::Costmap2DROS* controller_costmap_ros);
  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~FixPatternController();

  bool Control(BaseControlOption* option, ControlEnvironment* environment);

 private:
  /**
   * @brief  Performs a control cycle
   * @return True if processing of the fixpattern path is done, false otherwise
   */
  bool ExecuteCycle();
  /**
   * @brief Get a new fixpattern path or just publisher the old one
   */
  void MakePlan();
  /**
   * @brief  Publishes a velocity command of zero to the base
   */
  void PublishZeroVelocity();
  /**
   * @brief  Reset the state of the move_base action and send a zero velocity command to the base
   */
  void ResetState();
  /**
   * @brief Check if footprint of path is safe
   *
   * @param path
   * @param circle_center_points
   * @param length
   *
   * @return True if footprint is safe, false if not
   */
  bool IsPathFootprintSafe(const std::vector<geometry_msgs::PoseStamped>& path,
                           const std::vector<geometry_msgs::Point>& circle_center_points, double length);
  /**
   * @brief Check if front of machine is safe
   *
   * @return True if front is safe, return false if not
   */
  bool IsFrontSafe();
  /**
   * @brief Publish nav_msgs::Path for visualization
   *
   * @param pub Publisher that will use
   * @param plan Path that needs to be published
   */
  void PublishPlan(const ros::Publisher& pub, const std::vector<geometry_msgs::PoseStamped>& plan);
  /**
   * @brief Calculate distance of two geometry_msgs::PoseStamped
   *
   * @param p1 One of geometry_msgs::PoseStamped
   * @param p2 The other of geometry_msgs::PoseStamped
   *
   * @return Distance of two PoseStamped
   */
  double PoseStampedDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
  /**
   * @brief Transform geometry_msgs::PoseStamped to global_frame_ using tf
   *
   * @param pose_msg PoseStamped that needs to be transformed
   *
   * @return Transformed PoseStamped
   */
  geometry_msgs::PoseStamped PoseStampedToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg);

 private:
  tf::TransformListener& tf_;

  costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

  // footprint checker
  autoscrubber::FootprintChecker* footprint_checker_;

  ros::Publisher fixpattern_pub_;

  FixPatternState state_;
  FixPatternRecoveryTrigger recovery_trigger_;

  ros::Time last_valid_control_, last_oscillation_reset_;
  geometry_msgs::PoseStamped oscillation_pose_;
  geometry_msgs::PoseStamped planner_goal_;

  // global options given by autoscrubber
  FixPatternControlOption* co_;
  ControlEnvironment* env_;

  geometry_msgs::Twist cmd_vel_;
  bool switch_controller_;
  bool first_run_flag_;
};

};  // namespace autoscrubber

#endif  // AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_FIXPATTERN_CONTROLLER_H_
