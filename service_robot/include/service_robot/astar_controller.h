/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file astar_controller.h
 * @brief astar mode controll flow
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#ifndef SERVICEROBOT_INCLUDE_SERVICEROBOT_ASTAR_CONTROLLER_H_
#define SERVICEROBOT_INCLUDE_SERVICEROBOT_ASTAR_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/base_global_planner.h>
#include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <autoscrubber_services/StartRotate.h>
#include <autoscrubber_services/StopRotate.h>
#include <autoscrubber_services/CheckRotate.h>
#include <autoscrubber_services/CheckGoal.h>
#include <fixpattern_path/path.h>
#include <search_based_global_planner/search_based_global_planner.h>
#include <fixpattern_local_planner/trajectory_planner_ros.h>
#include <gslib/gaussian_debug.h>
#include <string>
#include <vector>

#include "service_robot/base_controller.h"
#include "service_robot/footprint_checker.h"

namespace service_robot {

typedef enum {
  A_PLANNING    = 0,
  FIX_CONTROLLING = 1,
  FIX_CLEARING    = 2
} AStarState;

typedef enum {
  A_PLANNING_R    = 0,
  FIX_CONTROLLING_R = 1,
  PLANNER_RECOVERY_R= 2,
  FIX_GETNEWGOAL_R  = 3,
  FIX_FRONTSAFE_R   = 4,
  FIX_OSCILLATION_R = 5,
  LOCATION_RECOVERY_R = 6,
  BACKWARD_RECOVERY_R = 7
} AStarRecoveryTrigger;

typedef enum {
  P_INSERTING_NONE   = 0,
  P_INSERTING_BEGIN  = 1,
  P_INSERTING_END    = 2,
  P_INSERTING_MIDDLE = 3,
  P_INSERTING_SBPL   = 4
} AStarPlanningState;

typedef enum {
  E_NULL = 0,
  E_LOCATION_INVALID,
  E_GOAL_UNREACHABLE,
  E_PATH_NOT_SAFE,
  I_GOAL_HEADING,
  I_GOAL_REACHED,
  I_GOAL_UNREACHED,
  E_GOAL_NOT_SAFE,
  MAX_RET,
} StatusIndex;

struct AStarControlOption : BaseControlOption {
  double stop_duration;
  double localization_duration;
  double sbpl_max_distance;
  double max_path_length_diff;
  double max_offroad_dis;
  double max_offroad_yaw;
  double front_safe_check_dis;
  double backward_check_dis;
  double goal_safe_dis_a;
  double goal_safe_dis_b;
  double goal_safe_check_dis;
  double goal_safe_check_duration;
  double sbpl_footprint_padding;
  double fixpattern_footprint_padding;
  double switch_corner_dis_diff;
  double switch_corner_yaw_diff;
  double switch_normal_dis_diff;
  double switch_normal_yaw_diff;
  double stop_to_zero_acc;
  bool use_farther_planner;
  double init_path_sample_dis;
  double init_path_sample_yaw;
  double init_path_circle_center_extend_y;

  int* fixpattern_reached_goal;
  fixpattern_path::Path* fixpattern_path;
  geometry_msgs::PoseStamped* global_planner_goal;
  std::vector<geometry_msgs::Point> circle_center_points;
  std::vector<geometry_msgs::Point> backward_center_points;
  std::vector<geometry_msgs::Point> footprint_center_points;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> astar_global_planner;
  boost::shared_ptr<search_based_global_planner::SearchBasedGlobalPlanner> sbpl_global_planner;
  boost::shared_ptr<fixpattern_local_planner::FixPatternTrajectoryPlannerROS> fixpattern_local_planner;

  AStarControlOption(tf::TransformListener* tf,
                     costmap_2d::Costmap2DROS* controller_costmap_ros,
                     const std::string& robot_base_frame, const std::string& global_frame,
                     double planner_frequency, double controller_frequency, double inscribed_radius,
                     double planner_patience, double controller_patience, double oscillation_timeout,
                     double oscillation_distance, ros::Publisher* vel_pub)
    : BaseControlOption(tf, controller_costmap_ros, controller_costmap_ros,
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
                  costmap_2d::Costmap2DROS* controller_costmap_ros);
  /**
   * @brief  Destructor - Cleans up
   */
  virtual ~AStarController();

  bool Control(BaseControlOption* option, ControlEnvironment* environment);
  std::vector<geometry_msgs::Point> footprint_spec_;

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
   * @brief  Publishes a velocity command of zero to the base
   */

  void PublishVelWithAcc(geometry_msgs::Twist last_cmd_vel, double vel_acc);
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
  bool IsGoalFootprintSafe(double front_safe_dis_a, double front_safe_dis_b, const geometry_msgs::PoseStamped& pose);
  bool IsGoalSafe(const geometry_msgs::PoseStamped& goal_pose, double goal_front_check_dis, double goal_back_check_dis);
  bool IsGoalUnreachable(const geometry_msgs::PoseStamped& goal_pose);
  bool IsFixPathFrontSafe(double front_safe_check_dis);
  bool IsPathFootprintSafe(const fixpattern_path::Path& fix_path, double length);
  bool IsPathFootprintSafe(const std::vector<geometry_msgs::PoseStamped>& path,
                           const std::vector<geometry_msgs::Point>& circle_center_points, double length);
  bool IsGlobalGoalReached(const geometry_msgs::PoseStamped& current_position, const geometry_msgs::PoseStamped& global_goal,
                            double xy_goal_tolerance, double yaw_goal_tolerance);
  double CheckFixPathFrontSafe(const std::vector<geometry_msgs::PoseStamped>& path, double front_safe_check_dis, double extend_x, double extend_y);
  bool UpdateFixPath(const std::vector<geometry_msgs::PoseStamped>& path, const geometry_msgs::PoseStamped& global_start, double front_safe_check_dis, bool use_static_costmap);
  bool NeedBackward(const geometry_msgs::PoseStamped& pose, double distance);
  bool GetAStarInitailPath(const geometry_msgs::PoseStamped& global_start, const geometry_msgs::PoseStamped& global_goal);
  bool GetAStarGoal(const geometry_msgs::PoseStamped& cur_pose, int begin_index = 0);
  bool GetAStarTempGoal(geometry_msgs::PoseStamped& goal_pose, double offset_dis);
  bool GetAStarStart(double front_safe_check_dis);
  void SampleInitailPath(std::vector<geometry_msgs::PoseStamped>* planner_plan,
                         std::vector<fixpattern_path::PathPoint>& fix_path);
  bool GetCurrentPosition(geometry_msgs::PoseStamped& current_position);
  unsigned int GetPoseIndexOfPath(const std::vector<geometry_msgs::PoseStamped>& path, const geometry_msgs::PoseStamped& pose);
  bool HandleGoingBack(const geometry_msgs::PoseStamped& current_position, double backward_dis = 0.0);
  void PlanThread();
  double PoseStampedDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);

  void PublishPlan(const ros::Publisher& pub, const std::vector<geometry_msgs::PoseStamped>& plan);
  void PublishMovebaseStatus(unsigned int status_index);
  void PublishHeadingGoal(void);
  void PublishGoalReached(geometry_msgs::PoseStamped goal_pose);
  // rotate recovery
  void UpdateRecoveryYaw(geometry_msgs::PoseStamped current_position);
  bool CanRotate(double x, double y, double yaw, int dir);
  bool RotateToYaw(double target_yaw);
  bool RotateRecovery();
  bool HandleRecovery(geometry_msgs::PoseStamped current_pos);
  bool HandleLocalizationRecovery(void);
  bool HandleSwitchingPath(geometry_msgs::PoseStamped current_position);
  // forward
  bool GoingForward(double distance);
  bool CanForward(double distance);

  // backward
  bool GoingBackward(double distance);
  bool CanBackward(double distance);

  void LocalizationCallBack(const std_msgs::Int8::ConstPtr& param);
  bool CheckGoalIsSafe(autoscrubber_services::CheckGoal::Request& req, autoscrubber_services::CheckGoal::Response& res); // NOLINT

 private:
  tf::TransformListener& tf_;

  costmap_2d::Costmap2DROS* controller_costmap_ros_;

  tf::Stamped<tf::Pose> global_pose_;

  geometry_msgs::Twist last_valid_cmd_vel_;
  AStarState state_;
  AStarRecoveryTrigger recovery_trigger_;
  AStarPlanningState planning_state_;

  ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
  geometry_msgs::PoseStamped oscillation_pose_;

  // used in astar local planner
  fixpattern_path::Path astar_path_;
  // used for path switching and replacing
  fixpattern_path::Path front_path_;
  // footprint checker
  service_robot::FootprintChecker* footprint_checker_;

  // index of planner_goal_ in fixpattern_path, record it so we can search new
  // goal from it
  unsigned int planner_goal_index_;

  // set up plan triple buffer
  std::vector<geometry_msgs::PoseStamped>* planner_plan_;
  std::vector<fixpattern_path::PathPoint> fix_path_;

  // rotate direction of rotate_recovery
  int rotate_recovery_dir_;
  int rotate_failure_times_;
  int try_recovery_times_;

  // set up the planner's thread
  bool runPlanner_;
  bool sbpl_reached_goal_;
  bool taken_global_goal_;
  bool gotStartPlan_;
  bool gotInitPlan_;
  bool gotGoalPlan_;
  bool switch_path_;
  unsigned int origin_path_safe_cnt_;
  unsigned int astar_planner_timeout_cnt_;
  unsigned int local_planner_error_cnt_;
  unsigned int fix_local_planner_error_cnt_;
  unsigned int goal_not_safe_cnt_;
  unsigned int path_not_safe_cnt_;
  unsigned int front_safe_check_cnt_;
  unsigned int obstacle_index_;
  unsigned int front_goal_index_;
  double cmd_vel_ratio_;
  double rotate_recovery_target_yaw_[15];
  boost::mutex planner_mutex_;
  boost::condition_variable planner_cond_;
  geometry_msgs::PoseStamped planner_start_;
  geometry_msgs::PoseStamped planner_goal_;
  geometry_msgs::PoseStamped global_goal_;
  geometry_msgs::PoseStamped front_goal_;
  geometry_msgs::PoseStamped sbpl_planner_goal_;
  geometry_msgs::PoseStamped init_pose_;
  geometry_msgs::PoseStamped success_broader_goal_;
  boost::thread* planner_thread_;
  unsigned int planner_start_index_;
  bool new_global_plan_;
  bool using_sbpl_directly_;
  bool last_using_bezier_;
  bool replan_directly_;
  // broader sbpl start and goal entry
  bool sbpl_broader_;

  bool first_run_controller_flag_;
  bool localization_valid_;
  bool localization_start_;

  AStarControlOption* co_;
  ControlEnvironment* env_;

  // set for fixpattern
  ros::Publisher fixpattern_pub_;
  ros::Publisher goal_reached_pub_;
  ros::Publisher heading_goal_pub_;
  ros::Publisher init_finished_pub_;
  ros::Publisher astar_start_pub_;
  ros::Publisher astar_goal_pub_;
  ros::Publisher sbpl_goal_pub_;
  ros::Publisher move_base_status_pub_;
  ros::Subscriber set_init_sub_;
  ros::Subscriber localization_sub_;
  ros::ServiceServer check_goal_srv_;
  ros::ServiceClient start_rotate_client_;
  ros::ServiceClient stop_rotate_client_;
  ros::ServiceClient check_rotate_client_;
};

};  // namespace service_robot

#endif  // SERVICEROBOT_INCLUDE_SERVICEROBOT_ASTAR_CONTROLLER_H_
