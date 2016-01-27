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
    : tf_(*tf), controller_costmap_ros_(NULL),
      controllers_(NULL), options_(NULL),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
      blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
      recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
      new_global_plan_(false) {
  ros::NodeHandle private_nh("~");
  ros::NodeHandle sbpl_nh("~/sbpl_global_planner");
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
  private_nh.param("localization_duration", localization_duration_, 5.0);

  private_nh.param("max_path_length_dif", max_path_length_diff_, 5.0);
  private_nh.param("max_offroad_dis", max_offroad_dis_, 0.7); //0.7 Lee
  private_nh.param("front_safe_check_dis", front_safe_check_dis_, 1.0);
  private_nh.param("goal_safe_dis_a", goal_safe_dis_a_, 0.5);	// distance after goal poin
  private_nh.param("goal_safe_dis_b", goal_safe_dis_b_, 0.3); // distance before goal point
  private_nh.param("goal_safe_check_dis", goal_safe_check_dis_, 1.0);
  private_nh.param("goal_safe_check_duration", goal_safe_check_duration_, 5.0);

  private_nh.param("fixpattern_footprint_padding", fixpattern_footprint_padding_, 0.2);
  private_nh.param("sbpl_footprint_padding", sbpl_footprint_padding_, 0.1);

  if (!ReadCircleCenterFromParams(private_nh, &circle_center_points_)) {
    ROS_ERROR("[AUTOSCRUBBER] read circle_center_point failed");
    exit(1);
  }

  if (!ReadFootprintCenterFromParams(private_nh, &footprint_center_points_)) {
    ROS_ERROR("[AUTOSCRUBBER] read footprint_center_point failed");
    exit(1);
  }
  // for comanding the base
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
  // they won't get any useful information back about its status, but this is useful for tools
  // like nav_view and rviz

  ros::NodeHandle simple_nh("move_base_simple");
  simple_goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, boost::bind(&AutoScrubber::SimpleGoalCB, this, _1));
  goal_sub_ = private_nh.subscribe<move_base_msgs::MoveBaseActionGoal>("goal", 10, boost::bind(&AutoScrubber::GoalCB, this, _1));
  pause_sub_ = simple_nh.subscribe<std_msgs::UInt32>("gaussian_pause", 10, boost::bind(&AutoScrubber::PauseCB, this, _1));
  terminate_sub_ = simple_nh.subscribe<std_msgs::UInt32>("gaussian_cancel", 10, boost::bind(&AutoScrubber::TerminateCB, this, _1));
  goal_reached_pub_ = simple_nh.advertise<std_msgs::UInt32>("/GUI/IS_GOAL_REACHED", 1);
  std::cout << "subscribe init finish here" << std::endl;
  private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);

  // create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
  controller_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  controller_costmap_ros_->pause();

  // initialize the global planner
  if (!LoadGlobalPlanner()) {
    exit(1);
  }

  // create a local planner
  if (!LoadLocalPlanner()) {
    exit(1);
  }

  // Start actively updating costmaps based on sensor data
  controller_costmap_ros_->start();
  controller_costmap_ros_->getCostmap();

  // if we shutdown our costmaps when we're deactivated... we'll do that now
  if (shutdown_costmaps_) {
    ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
    controller_costmap_ros_->stop();
   // controller_costmap_ros_->stop();
  }

  // create fixpattern_path object
  fixpattern_path_ = new fixpattern_path::Path();
  astar_path_ = new fixpattern_path::Path();

  // create controller option and intialize controllers
  options_ = new AStarControlOption(&tf_, controller_costmap_ros_,
                                              robot_base_frame_, global_frame_, planner_frequency_,
                                              controller_frequency_, inscribed_radius_, planner_patience_,
                                              controller_patience_, oscillation_timeout_,
                                              oscillation_distance_, &vel_pub_);
  reinterpret_cast<AStarControlOption*>(options_)->stop_duration = stop_duration_;
  reinterpret_cast<AStarControlOption*>(options_)->localization_duration = localization_duration_;
  reinterpret_cast<AStarControlOption*>(options_)->max_offroad_dis = max_offroad_dis_;
  reinterpret_cast<AStarControlOption*>(options_)->max_path_length_diff = max_path_length_diff_;
  reinterpret_cast<AStarControlOption*>(options_)->front_safe_check_dis = front_safe_check_dis_;
  reinterpret_cast<AStarControlOption*>(options_)->sbpl_max_distance = sbpl_max_distance_;
  reinterpret_cast<AStarControlOption*>(options_)->goal_safe_dis_a = goal_safe_dis_a_;
  reinterpret_cast<AStarControlOption*>(options_)->goal_safe_dis_b = goal_safe_dis_b_;
  reinterpret_cast<AStarControlOption*>(options_)->goal_safe_check_dis = goal_safe_check_dis_;
  reinterpret_cast<AStarControlOption*>(options_)->goal_safe_check_duration = goal_safe_check_duration_;
  reinterpret_cast<AStarControlOption*>(options_)->fixpattern_path = fixpattern_path_;
  reinterpret_cast<AStarControlOption*>(options_)->astar_global_planner = astar_global_planner_;
  reinterpret_cast<AStarControlOption*>(options_)->sbpl_global_planner = sbpl_global_planner_;
  reinterpret_cast<AStarControlOption*>(options_)->fixpattern_local_planner = fixpattern_local_planner_;
  reinterpret_cast<AStarControlOption*>(options_)->circle_center_points = circle_center_points_;
  reinterpret_cast<AStarControlOption*>(options_)->footprint_center_points = footprint_center_points_;
  reinterpret_cast<AStarControlOption*>(options_)->sbpl_footprint_padding = sbpl_footprint_padding_;
  reinterpret_cast<AStarControlOption*>(options_)->fixpattern_footprint_padding = fixpattern_footprint_padding_;
  reinterpret_cast<AStarControlOption*>(options_)->global_planner_goal = &global_planner_goal_;
  controllers_ = new AStarController(&tf_, controller_costmap_ros_);

  // initialize environment_
  environment_.launch_scrubber = false;
  environment_.run_flag = false;
  environment_.pause_flag = false;

  // initially, we'll need to make a plan
  state_ = PLANNING;

  // controlling thread
  control_thread_ = new boost::thread(boost::bind(&AutoScrubber::ControlThread, this));
/*
  notify_chassis_thread_ = new boost::thread(boost::bind(&AutoScrubber::NotifyChassisThread, this));

  // install service client
  ros::NodeHandle chassis_nh;
  launch_scrubber_client_ = chassis_nh.serviceClient<autoscrubber_services::LaunchScrubber>("launch_scrubber");
  stop_scrubber_client_ = chassis_nh.serviceClient<autoscrubber_services::StopScrubber>("stop_scrubber");
*/
  // start service when all done
  start_srv_ = private_nh.advertiseService("start", &AutoScrubber::Start, this);
  pause_srv_ = private_nh.advertiseService("pause", &AutoScrubber::Pause, this);
  resume_srv_ = private_nh.advertiseService("resume", &AutoScrubber::Resume, this);
  terminate_srv_ = private_nh.advertiseService("terminate", &AutoScrubber::Terminate, this);
  is_goal_reached_srv_ = private_nh.advertiseService("is_goal_reached", &AutoScrubber::IsGoalReached, this);
  get_current_pose_srv_ = private_nh.advertiseService("get_current_pose", &AutoScrubber::GetCurrentPose, this);
} 

void AutoScrubber::SimpleGoalCB(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and start ExecuteCycle.");
  ROS_INFO("[AUTOSCRUBBER] Get Goal!x = %.2f, y = %.2f, yaw = %.2f",goal->pose.position.x, goal->pose.position.y, tf::getYaw(goal->pose.orientation));
  global_planner_goal_.pose = goal->pose;
  global_planner_goal_.header.frame_id = global_frame_;
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
//  ROS_WARN("[AUTOSCRUBBER] Get Gaussian_Cancel");
  autoscrubber_services::Terminate::Request req;
  autoscrubber_services::Terminate::Response res;
  Terminate(req, res);
}

AutoScrubber::~AutoScrubber() {
  delete control_thread_;

  if (controller_costmap_ros_ != NULL)
    delete controller_costmap_ros_;

  delete controllers_;
//  delete options_ ;

  delete fixpattern_path_;
  delete astar_path_;

  astar_global_planner_.reset();
  sbpl_global_planner_.reset();
  fixpattern_local_planner_.reset();
}

bool AutoScrubber::Start(autoscrubber_services::Start::Request& req, autoscrubber_services::Start::Response& res) {
  ROS_INFO("[AUTOSCRUBBER] Start called");
  if (environment_.run_flag) {
    ROS_INFO("[AUTOSCRUBBER] Control Thread is Running, Stop it first!");
    environment_.run_flag = false;
    environment_.pause_flag = true;
    while (environment_.pause_flag) {
      usleep(500);
    }
    usleep(1000);
  }
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

bool AutoScrubber::GetCurrentPose(autoscrubber_services::GetCurrentPose::Request& req, autoscrubber_services::GetCurrentPose::Response& res) {
  geometry_msgs::PoseStamped cur_pos;
/*  cur_pos.pose.position.x = 0.0;
  cur_pos.pose.position.y = 0.0;
  cur_pos.pose.position.z = 0.0;
  cur_pos.pose.orientation.x = 0.0;
  cur_pos.pose.orientation.y = 0.0;
  cur_pos.pose.orientation.z = 0.0;
  cur_pos.pose.orientation.w = 0.0;
*/
  // TODO(lizhen) check localization_valid_, if false, return pose(0,0,0)
  if (controller_costmap_ros_ != NULL) {
    tf::Stamped<tf::Pose> global_pose;
    if (controller_costmap_ros_->getRobotPose(global_pose)) {
      tf::poseStampedTFToMsg(global_pose, cur_pos);
      res.current_pose = cur_pos; 
      double x = cur_pos.pose.position.x;
      double y = cur_pos.pose.position.y;
      double yaw = tf::getYaw(cur_pos.pose.orientation);
      ROS_WARN("[AUTOSCRUBBER] clearFootprintInCostmap!");
      controller_costmap_ros_->clearFootprintInCostmap(controller_costmap_ros_->getStaticCostmap(), x, y, yaw, 0.50);
      return true;
    }
  }
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
      usleep(50000);
      continue;
    }

    // do some intialize things
    bool first_run = true;
    // loop below quits when navigation finishes
//    while (!controllers_[nav_mode_]->Control(options_[nav_mode_], &environment_, first_run)) {
    while (!controllers_->Control(options_, &environment_, first_run)) {
      first_run = false; 
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
  astar_global_planner_->initialize("astar_global_planner", controller_costmap_ros_);
  sbpl_global_planner_ = boost::shared_ptr<search_based_global_planner::SearchBasedGlobalPlanner>(
      new search_based_global_planner::SearchBasedGlobalPlanner());
  sbpl_global_planner_->initialize("sbpl_global_planner", controller_costmap_ros_);
  return true;
}

bool AutoScrubber::LoadLocalPlanner() {
  fixpattern_local_planner_ = boost::shared_ptr<fixpattern_local_planner::FixPatternTrajectoryPlannerROS>(
      new fixpattern_local_planner::FixPatternTrajectoryPlannerROS());
  fixpattern_local_planner_->initialize("fixpattern_local_planner", &tf_, controller_costmap_ros_);
  return true;
}

double GetNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name) {
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    std::string& value_string = value;
    ROS_FATAL("Values in the circle_center specification (param %s) must be numbers. Found value %s.",
              full_param_name.c_str(), value_string.c_str() );
    throw std::runtime_error("Values in the circle_center specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(value) : static_cast<double>(value);
}

void ReadCircleCenterFromXMLRPC(XmlRpc::XmlRpcValue& circle_center_xmlrpc, const std::string& full_param_name, std::vector<geometry_msgs::Point>* points) {
  // Make sure we have an array of at least 3 elements.
  if (circle_center_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || circle_center_xmlrpc.size() < 1) {
    ROS_FATAL("The circle_center must be specified as list of lists on the parameter server, %s was specified as %s",
              full_param_name.c_str(), std::string(circle_center_xmlrpc).c_str());
    throw std::runtime_error("The circle_center must be specified as list of lists on the parameter server with at least 1 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  geometry_msgs::Point pt;

  for (int i = 0; i < circle_center_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = circle_center_xmlrpc[ i ];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2) {
      ROS_FATAL("The circle_center (parameter %s) must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.", full_param_name.c_str());
      throw std::runtime_error( "The circle_center must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = GetNumberFromXMLRPC(point[0], full_param_name);
    pt.y = GetNumberFromXMLRPC(point[1], full_param_name);
//    ROS_INFO("[AUTOSCRUBBER] get circle center[%d] px = %lf, py = %lf", i, pt.x, pt.y);
    points->push_back(pt);
  }
}

bool AutoScrubber::ReadCircleCenterFromParams(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* points) {
  std::string full_param_name;

  if (nh.searchParam("circle_center", full_param_name)) {
    XmlRpc::XmlRpcValue circle_center_xmlrpc;
    nh.getParam(full_param_name, circle_center_xmlrpc);
    if (circle_center_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      ReadCircleCenterFromXMLRPC(circle_center_xmlrpc, full_param_name, points);
      for (int i = 0; i < points->size(); ++i)
        ROS_INFO("[AUTOSCRUBBER] circle_center[%d].x = %lf; .y = %lf", i, points->at(i).x, points->at(i).y);
      return true;
    } else {
      ROS_ERROR("[AUTOSCRUBBER] circle_center param's type is not Array!");
      return false;
    }
  }
}

bool AutoScrubber::ReadFootprintCenterFromParams(ros::NodeHandle& nh, std::vector<geometry_msgs::Point>* points) {
  std::string full_param_name;

  if (nh.searchParam("footprint_center", full_param_name)) {
    XmlRpc::XmlRpcValue footprint_center_xmlrpc;
    nh.getParam(full_param_name, footprint_center_xmlrpc);
    if (footprint_center_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      ReadCircleCenterFromXMLRPC(footprint_center_xmlrpc, full_param_name, points);
      for (int i = 0; i < points->size(); ++i)
        ROS_INFO("[AUTOSCRUBBER] footprint_center[%d].x = %lf; .y = %lf", i, points->at(i).x, points->at(i).y);
      return true;
    } else {
      ROS_ERROR("[AUTOSCRUBBER] footprint_center param's type is not Array!");
      return false;
    }
  }
}

};  // namespace autoscrubber
