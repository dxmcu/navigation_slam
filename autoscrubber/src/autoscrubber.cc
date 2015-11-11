/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file autoscrubber.cc
 * @brief scrub machine control node
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#include "autoscrubber/autoscrubber.h"

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <cmath>

#include "autoscrubber/fixpattern_controller.h"
#include "autoscrubber/astar_controller.h"

namespace autoscrubber {

AutoScrubber::AutoScrubber(tf::TransformListener* tf)
    : tf_(*tf), nav_mode_(FIX_PATTERN), options_{2}, controllers_{2},
      planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
      blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
      recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
      planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
      new_global_plan_(false), fixpattern_reached_goal_(0) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle fixpattern_nh("fixpattern");
  ros::NodeHandle astar_nh("astar");
  ros::NodeHandle nh;

  recovery_trigger_ = PLANNING_R;

  // get some parameters that will be global to the move base node
  private_nh.param("enable_scrubber", enable_scrubber_, false);
  private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));
  private_nh.param("planner_frequency", planner_frequency_, 0.0);
  private_nh.param("sbpl_max_distance", sbpl_max_distance_, 12.0);
  private_nh.param("controller_frequency", controller_frequency_, 20.0);
  private_nh.param("planner_patience", planner_patience_, 5.0);
  private_nh.param("controller_patience", controller_patience_, 1.0);

  private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
  private_nh.param("oscillation_distance", oscillation_distance_, 0.5);
  private_nh.param("stop_duration", stop_duration_, 4.0);

  private_nh.param("max_offroad_dis", max_offroad_dis_, 1.5);
  private_nh.param("front_safe_check_dis", front_safe_check_dis_, 2.5);
  private_nh.param("goal_safe_dis_a", goal_safe_dis_a_, 1.2);
  private_nh.param("goal_safe_dis_b", goal_safe_dis_b_, 2.5);
  private_nh.param("goal_safe_check_dis", goal_safe_check_dis_, 1.0);
  private_nh.param("goal_safe_check_duration", goal_safe_check_duration_, 5.0);

  // for comanding the base
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
  // they won't get any useful information back about its status, but this is useful for tools
  // like nav_view and rviz

  ros::NodeHandle simple_nh("move_base_simple");
  simple_goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&AutoScrubber::SimpleGoalCB, this, _1));
  goal_sub_ = private_nh.subscribe<move_base_msgs::MoveBaseActionGoal>("goal", 1, boost::bind(&AutoScrubber::GoalCB, this, _1));
  pause_sub_ = private_nh.subscribe<std_msgs::UInt32>("gaussian_pause", 1, boost::bind(&AutoScrubber::PauseCB, this, _1));
  terminate_sub_ = private_nh.subscribe<std_msgs::UInt32>("gaussian_cancel", 1, boost::bind(&AutoScrubber::TerminateCB, this, _1));
  goal_reached_pub_ = private_nh.advertise<std_msgs::UInt32>("/GUI/IS_GOAL_REACHED", 1);

  private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);

  // create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();

  // initialize the global planner
  if (!LoadGlobalPlanner()) {
    exit(1);
  }

  // create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
  controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
  controller_costmap_ros_->pause();

  // create a local planner
  if (!LoadLocalPlanner()) {
    exit(1);
  }

  // Start actively updating costmaps based on sensor data
  planner_costmap_ros_->start();
  controller_costmap_ros_->start();

  // if we shutdown our costmaps when we're deactivated... we'll do that now
  if (shutdown_costmaps_) {
    ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
    planner_costmap_ros_->stop();
    controller_costmap_ros_->stop();
  }

  // create fixpattern_path object
  fixpattern_path_ = new fixpattern_path::Path();
  astar_path_ = new fixpattern_path::Path();

  // create controller option and intialize controllers
  options_[FIX_PATTERN] = new FixPatternControlOption(&tf_, planner_costmap_ros_, controller_costmap_ros_,
                                                      robot_base_frame_, global_frame_, planner_frequency_,
                                                      controller_frequency_, inscribed_radius_, planner_patience_,
                                                      controller_patience_, oscillation_timeout_,
                                                      oscillation_distance_, &vel_pub_);
  reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->stop_duration = stop_duration_;
  reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->max_offroad_dis = max_offroad_dis_;
  reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->front_safe_check_dis = front_safe_check_dis_;
  reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->fixpattern_path = fixpattern_path_;
  reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->fixpattern_local_planner = fixpattern_local_planner_;
  reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->global_planner_goal = &global_planner_goal_;
	reinterpret_cast<FixPatternControlOption*>(options_[FIX_PATTERN])->fixpattern_reached_goal = &fixpattern_reached_goal_;
  options_[AUTO_NAV] = new AStarControlOption(&tf_, planner_costmap_ros_, controller_costmap_ros_,
                                              robot_base_frame_, global_frame_, planner_frequency_,
                                              controller_frequency_, inscribed_radius_, planner_patience_,
                                              controller_patience_, oscillation_timeout_,
                                              oscillation_distance_, &vel_pub_);
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->stop_duration = stop_duration_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->max_offroad_dis = max_offroad_dis_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->front_safe_check_dis = front_safe_check_dis_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->sbpl_max_distance = sbpl_max_distance_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->goal_safe_dis_a = goal_safe_dis_a_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->goal_safe_dis_b = goal_safe_dis_b_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->goal_safe_check_dis = goal_safe_check_dis_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->goal_safe_check_duration = goal_safe_check_duration_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->fixpattern_path = fixpattern_path_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->astar_global_planner = astar_global_planner_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->sbpl_global_planner = sbpl_global_planner_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->fixpattern_local_planner = fixpattern_local_planner_;
  reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->global_planner_goal = &global_planner_goal_;
	reinterpret_cast<AStarControlOption*>(options_[AUTO_NAV])->fixpattern_reached_goal = &fixpattern_reached_goal_;
  controllers_[FIX_PATTERN] = new FixPatternController(&tf_, planner_costmap_ros_, controller_costmap_ros_);
  controllers_[AUTO_NAV] = new AStarController(&tf_, planner_costmap_ros_, controller_costmap_ros_);

  // initialize environment_
  environment_.launch_scrubber = false;
  environment_.run_flag = false;
  environment_.pause_flag = false;

  // initially, we'll need to make a plan
  state_ = PLANNING;

  // controlling thread 
  control_thread_ = new boost::thread(boost::bind(&AutoScrubber::ControlThread, this));
  notify_chassis_thread_ = new boost::thread(boost::bind(&AutoScrubber::NotifyChassisThread, this));

  // install service client
  ros::NodeHandle chassis_nh;
  launch_scrubber_client_ = chassis_nh.serviceClient<autoscrubber_services::LaunchScrubber>("launch_scrubber");
  stop_scrubber_client_ = chassis_nh.serviceClient<autoscrubber_services::StopScrubber>("stop_scrubber");

  // start service when all done
  start_srv_ = private_nh.advertiseService("start", &AutoScrubber::Start, this);
  pause_srv_ = private_nh.advertiseService("pause", &AutoScrubber::Pause, this);
  resume_srv_ = private_nh.advertiseService("resume", &AutoScrubber::Resume, this);
  terminate_srv_ = private_nh.advertiseService("terminate", &AutoScrubber::Terminate, this);
  is_goal_reached_srv_ = private_nh.advertiseService("is_goal_reached", &AutoScrubber::IsGoalReached, this);
}

void AutoScrubber::SimpleGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and start ExecuteCycle.");
  ROS_INFO("[AUTOSCRUBBER] Get Goal!x = %.2f, y = %.2f, yaw = %.2f",goal->pose.position.x, goal->pose.position.y, tf::getYaw(goal->pose.orientation));
  global_planner_goal_.pose = goal->pose;
//  reinterpret_cast<AStarControlOption*>(options_)->settle_planner_goal_ = &astar_planner_goal_;
  autoscrubber_services::Start::Request req;
  autoscrubber_services::Start::Response res;
  Start(req, res);
}

void AutoScrubber::GoalCB(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal) {
  ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and start ExecuteCycle.");
  autoscrubber_services::Start::Request req;
  autoscrubber_services::Start::Response res;
  Start(req, res);
}

void AutoScrubber::PauseCB(const std_msgs::UInt32::ConstPtr& param) {
//  ROS_WARN("[AUTOSCRUBBER] Get Gaussian_Pause topic= %d", (int)param->data);
  if (param->data == 1) {
    autoscrubber_services::Pause::Request req;
    autoscrubber_services::Pause::Response res;
    Pause(req, res);
  } else if (param->data == 0) {
    autoscrubber_services::Resume::Request req;
    autoscrubber_services::Resume::Response res;
    Resume(req, res);
  }
}

void AutoScrubber::TerminateCB(const std_msgs::UInt32::ConstPtr& param) {
  autoscrubber_services::Terminate::Request req;
  autoscrubber_services::Terminate::Response res;
  Terminate(req, res);
}

AutoScrubber::~AutoScrubber() {
  delete control_thread_;

  if (planner_costmap_ros_ != NULL)
    delete planner_costmap_ros_;

  if (controller_costmap_ros_ != NULL)
    delete controller_costmap_ros_;

  delete planner_plan_;
  delete latest_plan_;
  delete controller_plan_;

  delete controllers_[FIX_PATTERN];
  delete controllers_[AUTO_NAV];

  delete fixpattern_path_;
  delete astar_path_;

  astar_global_planner_.reset();
  sbpl_global_planner_.reset();
  fixpattern_local_planner_.reset();
}

bool AutoScrubber::Start(autoscrubber_services::Start::Request& req, autoscrubber_services::Start::Response& res) {
  ROS_INFO("[AUTOSCRUBBER] Start called");
  environment_.run_flag = true;
  environment_.pause_flag = false;
  return true;
}

bool AutoScrubber::Pause(autoscrubber_services::Pause::Request& req, autoscrubber_services::Pause::Response& res) {
  ROS_INFO("[AUTOSCRUBBER] Pause called");
//  environment_.run_flag = true;
  environment_.pause_flag = true;
  return true;
}

bool AutoScrubber::Resume(autoscrubber_services::Resume::Request& req, autoscrubber_services::Resume::Response& res) {
  ROS_INFO("[AUTOSCRUBBER] Resume called");
//  environment_.run_flag = true;
  environment_.pause_flag = false;
  return true;
}

bool AutoScrubber::Terminate(autoscrubber_services::Terminate::Request& req, autoscrubber_services::Terminate::Response& res) {
  ROS_INFO("[AUTOSCRUBBER] Terminate called");
  environment_.run_flag = false;
  environment_.pause_flag = true;
  return true;
}

bool AutoScrubber::IsGoalReached(autoscrubber_services::IsGoalReached::Request& req, autoscrubber_services::IsGoalReached::Response& res) {
  res.reached.data = !environment_.run_flag && !environment_.pause_flag;
  return true;
}

void AutoScrubber::NotifyChassisThread() {
  ros::NodeHandle n;
  ros::Rate loop_rate(5.0);
  while (n.ok()) {
    if (enable_scrubber_ && environment_.launch_scrubber) {
      autoscrubber_services::LaunchScrubber launch;
      launch_scrubber_client_.call(launch);
    } else {
      autoscrubber_services::StopScrubber stop;
      stop_scrubber_client_.call(stop);
    }

    loop_rate.sleep();
  }
}

void AutoScrubber::ControlThread() {
  ros::NodeHandle n;
  while (n.ok()) {
    if (!environment_.run_flag) {
      usleep(500);
      continue;
    }

    // do some intialize things
    nav_mode_ = FIX_PATTERN;

    // loop below quits when navigation finishes
    while (!controllers_[nav_mode_]->Control(options_[nav_mode_], &environment_)) {
      if (nav_mode_ == FIX_PATTERN) {
        nav_mode_ = AUTO_NAV;
      } else {
        nav_mode_ = FIX_PATTERN;
      }
    }

    // notify GUI that goal is reached
    std_msgs::UInt32 msg;
    msg.data = 1;
    goal_reached_pub_.publish(msg);

    // do some clean things
    environment_.launch_scrubber = false;
    environment_.run_flag = false;
    environment_.pause_flag = false;
  }
}

bool AutoScrubber::LoadGlobalPlanner() {
  // check if a non fully qualified name has potentially been passed in
  astar_global_planner_ = boost::shared_ptr<nav_core::BaseGlobalPlanner>(new global_planner::GlobalPlanner());
  astar_global_planner_->initialize("astar_global_planner", planner_costmap_ros_);
  sbpl_global_planner_ = boost::shared_ptr<search_based_global_planner::SearchBasedGlobalPlanner>(
      new search_based_global_planner::SearchBasedGlobalPlanner());
  sbpl_global_planner_->initialize("sbpl_global_planner", planner_costmap_ros_);
  return true;
}

bool AutoScrubber::LoadLocalPlanner() {
  fixpattern_local_planner_ = boost::shared_ptr<fixpattern_local_planner::FixPatternTrajectoryPlannerROS>(
      new fixpattern_local_planner::FixPatternTrajectoryPlannerROS());
  fixpattern_local_planner_->initialize("fixpattern_local_planner", &tf_, controller_costmap_ros_);
  return true;
}

};  // namespace autoscrubber
