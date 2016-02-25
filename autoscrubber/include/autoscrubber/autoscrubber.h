/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file autoscrubber.h
 * @brief controller node of autonomous scrubber
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#ifndef AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_AUTOSCRUBBER_H_
#define AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_AUTOSCRUBBER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt32.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_loader.h>
#include <autoscrubber_services/Start.h>
#include <autoscrubber_services/Pause.h>
#include <autoscrubber_services/Resume.h>
#include <autoscrubber_services/Terminate.h>
#include <autoscrubber_services/IsGoalReached.h>
#include <autoscrubber_services/LaunchScrubber.h>
#include <autoscrubber_services/StopScrubber.h>
#include <autoscrubber_services/GetCurrentPose.h>
#include <fixpattern_path/path.h>
#include <global_planner/planner_core.h>
// #include <sbpl_lattice_planner/sbpl_lattice_planner.h>
#include <search_based_global_planner/search_based_global_planner.h>
#include <fixpattern_local_planner/trajectory_planner_ros.h>
#include <vector>
#include <string>

#include "autoscrubber/base_controller.h"

namespace autoscrubber {

typedef enum {
  FIX_PATTERN = 0,
  AUTO_NAV    = 1
} NavigationMode;

enum AutoScrubberState {
  PLANNING,
  CONTROLLING,
  CLEARING
};

enum RecoveryTrigger {
  PLANNING_R,
  CONTROLLING_R,
  OSCILLATION_R
};

/**
 * @class AutoScrubber
 * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
 */
class AutoScrubber {
 public:
  /**
   * @brief  Constructor for the actions
   * @param name The name of the action
   * @param tf A pointer to a TransformListener
   */
  explicit AutoScrubber(tf::TransformListener* tf);
  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~AutoScrubber();
  bool Start(autoscrubber_services::Start::Request& req, autoscrubber_services::Start::Response& res);  // NOLINT
  bool Pause(autoscrubber_services::Pause::Request& req, autoscrubber_services::Pause::Response& res);  // NOLINT
  bool Resume(autoscrubber_services::Resume::Request& req, autoscrubber_services::Resume::Response& res);  // NOLINT
  bool Terminate(autoscrubber_services::Terminate::Request& req, autoscrubber_services::Terminate::Response& res);  // NOLINT
  bool IsGoalReached(autoscrubber_services::IsGoalReached::Request& req, autoscrubber_services::IsGoalReached::Response& res); // NOLINT
  bool GetCurrentPose(autoscrubber_services::GetCurrentPose::Request& req, autoscrubber_services::GetCurrentPose::Response& res); // NOLINT

 private:
  /**
   * @brief  Performs a control cycle
   * @return True if processing of the fixpattern path is done, false otherwise
   */
  bool ExecuteCycle();
  /**
   * @brief  Make a new global plan
   * @param  goal The goal to plan to
   * @param  plan Will be filled in with the plan made by the planner
   * @return  True if planning succeeds, false otherwise
   */
  bool MakePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>* plan);
  /**
   * @brief Load the global planner for the navigation stack
   * @return True if the global planner was loaded successfully, false otherwise
   */
  bool LoadGlobalPlanner();
  /**
   * @brief Load the local planner for the navigation stack
   * @return True if the local planner was loaded successfully, false otherwise
   */
  bool LoadLocalPlanner();
  /**
   * @brief  Load the recovery behaviors for the navigation stack from the parameter server
   * @param node The ros::NodeHandle to be used for loading parameters
   * @return True if the recovery behaviors were loaded successfully, false otherwise
   */
  bool LoadRecoveryBehaviors(ros::NodeHandle node);
  /**
   * @brief  Publishes a velocity command of zero to the base
   */
  void PublishZeroVelocity();
  /**
   * @brief  Reset the state of the move_base action and send a zero velocity command to the base
   */
  void ResetState();

  void SimpleGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void GoalCB(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal);
  void PauseCB(const std_msgs::UInt32::ConstPtr& param);
  void TerminateCB(const std_msgs::UInt32::ConstPtr& param);
  void ControlThread();
  void NotifyChassisThread();
  void SwitchNavigationMode();
  bool ReadCircleCenterFromParams(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* points);
  bool ReadBackwardCenterFromParams(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* points);
  bool ReadFootprintCenterFromParams(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* points);

 private:
  tf::TransformListener& tf_;

  boost::shared_ptr<fixpattern_local_planner::FixPatternTrajectoryPlannerROS> fixpattern_local_planner_;
  costmap_2d::Costmap2DROS *controller_costmap_ros_;

//  std::vector<BaseController*> controllers_;
//  std::vector<BaseControlOption*> options_;
  BaseController* controllers_;
  BaseControlOption* options_;
  ControlEnvironment environment_;

  boost::shared_ptr<nav_core::BaseGlobalPlanner> astar_global_planner_;
  boost::shared_ptr<search_based_global_planner::SearchBasedGlobalPlanner> sbpl_global_planner_;
  std::string robot_base_frame_, global_frame_;

  tf::Stamped<tf::Pose> global_pose_;
  double sbpl_max_distance_;
  double max_path_length_diff_;
  double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
  double planner_patience_, controller_patience_;
  double conservative_reset_dist_, clearing_radius_;
  ros::Publisher vel_pub_, goal_reached_pub_;
  ros::Subscriber simple_goal_sub_, goal_sub_, pause_sub_, terminate_sub_;
  ros::ServiceServer start_srv_, pause_srv_, resume_srv_, terminate_srv_, is_goal_reached_srv_;
  ros::ServiceServer get_current_pose_srv_;

  ros::ServiceClient launch_scrubber_client_;
  ros::ServiceClient stop_scrubber_client_;
  bool shutdown_costmaps_;
  double oscillation_timeout_, oscillation_distance_;

  bool enable_scrubber_;

  // fixpattern option
  double stop_duration_;
  double localization_duration_;
  double max_offroad_dis_;
  double max_offroad_yaw_;
  double front_safe_check_dis_;
  double backward_check_dis_;
  double goal_safe_dis_a_;
  double goal_safe_dis_b_;
  double goal_safe_check_dis_;	
  double goal_safe_check_duration_;
  double switch_corner_dis_diff_;
  double switch_corner_yaw_diff_;
  double switch_normal_dis_diff_;
  double switch_normal_yaw_diff_;
  double stop_to_zero_acc_;

  double fixpattern_footprint_padding_;
  double sbpl_footprint_padding_;

  // sbpl param
  std::vector<geometry_msgs::Point> circle_center_points_;
  std::vector<geometry_msgs::Point> backward_center_points_;
  std::vector<geometry_msgs::Point> footprint_center_points_;

  // fixpattern path, share between two controllers
  fixpattern_path::Path* fixpattern_path_;
  // fixpattern path, used only in astar controller
  fixpattern_path::Path* astar_path_;

  NavigationMode nav_mode_;
  AutoScrubberState state_;
  RecoveryTrigger recovery_trigger_;

  ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
  geometry_msgs::PoseStamped oscillation_pose_;
  geometry_msgs::PoseStamped global_planner_goal_;
 
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
  pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

  // set up plan triple buffer
  std::vector<geometry_msgs::PoseStamped>* planner_plan_;
  std::vector<geometry_msgs::PoseStamped>* latest_plan_;
  std::vector<geometry_msgs::PoseStamped>* controller_plan_;

  // control thread
  boost::thread* control_thread_;
  boost::thread* notify_chassis_thread_;

  bool new_global_plan_;
};

};  // namespace autoscrubber

#endif  // AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_AUTOSCRUBBER_H_
