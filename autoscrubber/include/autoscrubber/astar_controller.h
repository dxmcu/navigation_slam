/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file astar_controller.h
 * @brief astar mode controll flow
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#ifndef AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_ASTAR_CONTROLLER_H_
#define AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_ASTAR_CONTROLLER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_loader.h>
#include <fixpattern_path/path.h>
#include <search_based_global_planner/search_based_global_planner.h>
#include <fixpattern_local_planner/trajectory_planner_ros.h>
#include <string>
#include <vector>

#include "autoscrubber/base_controller.h"
#include "autoscrubber/footprint_checker.h"

namespace autoscrubber {

typedef enum {
  A_PLANNING    = 0,
  A_CONTROLLING = 1,
  A_CLEARING    = 2
} AStarState;

typedef enum {
  A_PLANNING_R    = 0,
  A_CONTROLLING_R = 1,
  A_OSCILLATION_R = 2,
  A_GOALSAFE_R    = 3
} AStarRecoveryTrigger;

struct AStarControlOption : BaseControlOption {
  double stop_duration;
  double sbpl_max_distance;
  double max_offroad_dis;
  double front_safe_check_dis;
  double goal_safe_dis_a;
  double goal_safe_dis_b;
  double goal_safe_check_dis;	
  double goal_safe_check_duration;
	int* fixpattern_reached_goal;
  fixpattern_path::Path* fixpattern_path;
  geometry_msgs::PoseStamped* global_planner_goal;
  
  boost::shared_ptr<nav_core::BaseGlobalPlanner> astar_global_planner;
  boost::shared_ptr<search_based_global_planner::SearchBasedGlobalPlanner> sbpl_global_planner;
  boost::shared_ptr<fixpattern_local_planner::FixPatternTrajectoryPlannerROS> fixpattern_local_planner;

  AStarControlOption(tf::TransformListener* tf,
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

class AStarController : public BaseController {
 public:
  /**
   * @brief Constructor for the controller
   *
   * @param tf A pointer to a TransformListener
   * @param planner_costmap_ros A pointer to a Costmap2DROS of global frame
   * @param controller_costmap_ros A pointer to a Costmap2DROS of local frame
   */
  AStarController(tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* planner_costmap_ros,
                  costmap_2d::Costmap2DROS* controller_costmap_ros);
  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~AStarController();

  bool Control(BaseControlOption* option, ControlEnvironment* environment);

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
  bool MakePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>* plan);
  /**
   * @brief  Publishes a velocity command of zero to the base
   */
  void PublishZeroVelocity();
  /**
   * @brief  Reset the state of the move_base action and send a zero velocity command to the base
   */
  void ResetState();
  /**
   * @brief This is used to wake the planner at periodic intervals.
   */
  void WakePlanner(const ros::TimerEvent& event);
  /**
   * @brief Transform geometry_msgs::PoseStamped to global_frame_ using tf
   *
   * @param pose_msg PoseStamped that needs to be transformed
   *
   * @return Transformed PoseStamped
   */
  geometry_msgs::PoseStamped PoseStampedToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg);
  geometry_msgs::PoseStamped PoseStampedToLocalFrame(const geometry_msgs::PoseStamped& pose_msg);
  bool IsPoseFootprintSafe(double front_safe_dis_a, double front_safe_dis_b, const geometry_msgs::PoseStamped& pose);
  bool IsPathFootprintSafe(const fixpattern_path::Path& fix_path, double length);
  bool IsGoalFootprintSafe(double goal_check_safe_dis, double goal_safe_dis_a, double goal_safe_dis_b, const geometry_msgs::PoseStamped& current_pose);
  bool NeedBackward(const geometry_msgs::PoseStamped& pose, double distance);
  bool GetAStarGoal();
  void HandleGoingBack(geometry_msgs::PoseStamped current_position);
  void PlanThread();
  double PoseStampedDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
	std::vector<fixpattern_path::PathPoint> CalculateStartCurvePath(const std::vector<fixpattern_path::PathPoint>& astar_path);
	std::vector<fixpattern_path::PathPoint> CalculateGoalCurvePath(const std::vector<fixpattern_path::PathPoint>& astar_path);

 private:
  tf::TransformListener& tf_;

  costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;

  tf::Stamped<tf::Pose> global_pose_;

  AStarState state_;
  AStarRecoveryTrigger recovery_trigger_;

  ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
  geometry_msgs::PoseStamped oscillation_pose_;

  // used in astar local planner
  fixpattern_path::Path astar_path_;
  // footprint checker
  autoscrubber::FootprintChecker* footprint_checker_;

  // index of planner_goal_ in fixpattern_path, record it so we can search new
  // goal from it
  unsigned int planner_goal_index_;

  // set up plan triple buffer
  std::vector<geometry_msgs::PoseStamped>* planner_plan_;
  std::vector<geometry_msgs::PoseStamped>* latest_plan_;
  std::vector<geometry_msgs::PoseStamped>* controller_plan_;

  // set up the planner's thread
  bool runPlanner_;
	bool sbpl_reached_goal_;
  boost::mutex planner_mutex_;
  boost::condition_variable planner_cond_;
  geometry_msgs::PoseStamped planner_goal_;
  boost::thread* planner_thread_;

  bool new_global_plan_;
  bool using_sbpl_directly_;

  AStarControlOption* co_;
  ControlEnvironment* env_;
};

};  // namespace autoscrubber

#endif  // AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_ASTAR_CONTROLLER_H_
