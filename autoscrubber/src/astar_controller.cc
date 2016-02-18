/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file astar_controller.cpp
 * @brief implementation of astar controller
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#include "autoscrubber/astar_controller.h"
#include "autoscrubber/bezier_planner.h"
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <std_msgs/UInt32.h>
#include <autoscrubber_services/StartRotate.h>
#include <autoscrubber_services/StopRotate.h>
#include <autoscrubber_services/CheckRotate.h>

namespace autoscrubber {

AStarController::AStarController(tf::TransformListener* tf,
                                 costmap_2d::Costmap2DROS* controller_costmap_ros)
    : tf_(*tf),
      controller_costmap_ros_(controller_costmap_ros), planner_plan_(NULL), 
      planner_goal_index_(0), sbpl_reached_goal_(false), 
      runPlanner_(false), new_global_plan_(false), first_run_controller_flag_(true),
      using_sbpl_directly_(false), sbpl_broader_(false), last_using_bezier_(false), replan_directly_(false),
      astar_planner_timeout_cnt_(0), local_planner_error_cnt_(0), goal_not_safe_cnt_(0), path_not_safe_cnt_(0){
  // set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  // create footprint_checker_
  footprint_checker_ = new autoscrubber::FootprintChecker(*controller_costmap_ros_->getCostmap());

  footprint_spec_ = controller_costmap_ros_->getRobotFootprint();
  // set up the planner's thread
  planner_thread_ = new boost::thread(boost::bind(&AStarController::PlanThread, this));

  // initially, we'll need to make a plan
  state_ = A_PLANNING;
  
  // disable terminate_controller_ when start	
  terminate_controller_ = false;
  // disable localization_recovery_ when start	
  localization_valid_ = false;
  // we'll start executing recovery behaviors at the beginning of our list
  recovery_trigger_ = A_PLANNING_R;

  switch_path_ = false;
  // set rotate_recovery_dir_
  rotate_recovery_dir_ = 0;
  rotate_failure_times_ = 0;
  cmd_vel_ratio_ = 1.0;
  // set for fixpattern_path

  ros::NodeHandle fixpattern_nh("~/fixpattern_global_planner");
  fixpattern_pub_ = fixpattern_nh.advertise<nav_msgs::Path>("plan", 1);
  ros::NodeHandle n;
  ros::NodeHandle device_nh("device");
  alarm_pub_ = n.advertise<std_msgs::UInt8>("alarm", 10);
  goal_reached_pub_ = n.advertise<geometry_msgs::PoseStamped>("goal_reached", 10);
  astar_goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("astar_goal", 10);
  astar_start_pub_ = n.advertise<geometry_msgs::PoseStamped>("astar_start", 10);

  localization_sub_ = n.subscribe<std_msgs::Int8>("/localization_valid", 100, boost::bind(&AStarController::LocalizationCallBack, this, _1));
  start_rotate_client_ = device_nh.serviceClient<autoscrubber_services::StartRotate>("start_rotate");
  stop_rotate_client_ = device_nh.serviceClient<autoscrubber_services::StopRotate>("stop_rotate");
  check_rotate_client_ = device_nh.serviceClient<autoscrubber_services::CheckRotate>("Check_rotate");
}

AStarController::~AStarController() {
  planner_thread_->interrupt();
  planner_thread_->join();

  delete footprint_checker_;

  delete planner_thread_;

  delete planner_plan_;
}

void AStarController::LocalizationCallBack(const std_msgs::Int8::ConstPtr& param) { 
  if (param->data == 1) {
    localization_valid_ = true;
  } else {
//    ROS_WARN("[ASTAR CONTROLLER] localization failed!"); 
    localization_valid_ = false;
  }
}

bool AStarController::MakePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>* plan) {

  // make sure to set the plan to be empty initially
  plan->clear();

  // since this gets called on handle activate
  if (controller_costmap_ros_ == NULL) {
    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
    return false;
  }
  replan_directly_ = false;

  if (PoseStampedDistance(start, goal) <= 0.25) {
    // set this to true as we'll use it afterwards
    using_sbpl_directly_ = true;
    last_using_bezier_ = false;

    ROS_INFO("[ASTAR PLANNER] too short, take start and goal as plan directly");
    // too short, plan direct path
    plan->clear();
    plan->push_back(start);
    plan->push_back(goal);

    // assign to astar_path_
    std::vector<fixpattern_path::PathPoint> path;
    for (const auto& p : *plan) {
      path.push_back(fixpattern_path::GeometryPoseToPathPoint(p.pose));
    }
    astar_path_.set_short_sbpl_path(start, path);
  } else if (!last_using_bezier_ && PoseStampedDistance(start, goal) <= 2.0) {
    ROS_INFO("[ASTAR PLANNER] use bezier planner");
    using_sbpl_directly_ = true;
    last_using_bezier_ = true;

    // get path limited length
    std::vector<fixpattern_path::PathPoint> bezier_path;
    if (!MakeBezierPlan(&bezier_path, start, goal, true) || bezier_path.size() == 0) {
      ROS_WARN("[ASTAR PLANNER] bezier failed to find a plan, replan directly");
      replan_directly_ = true;
      return false;
    }
    astar_path_.set_bezier_path(start, bezier_path, state_ == A_PLANNING);
    // if invalid, return false
    if (!IsPathFootprintSafe(astar_path_, co_->front_safe_check_dis)) {
      ROS_INFO("[ASTAR PLANNER] bezier_path not safe, replan directly");
      replan_directly_ = true;
      return false;
    }
/*
    // don't forget to fill plan
    ros::Time now = ros::Time::now();
    plan->push_back(start);
    for (const auto& p : bezier_path) {
      geometry_msgs::PoseStamped pp;
      pp.pose = p;
      pp.header.stamp = now;
      pp.header.frame_id = co_->global_frame;
      plan->push_back(pp);
    }
    plan->push_back(planner_goal_);
*/
  } else if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
    // too short, use sbpl directly
    ROS_INFO("[ASTAR PLANNER] use sbpl directly");
    using_sbpl_directly_ = true;
    last_using_bezier_ = false;
    // if last plan was success due to sbpl_broader_ and goal has not changed,
    // we'll still use sbpl_broader_
//    bool sbpl_broader = sbpl_broader_;
//    if (PoseStampedDistance(goal, success_broader_goal_) < GS_DOUBLE_PRECISION)
//      sbpl_broader = true;
    // if the planner fails or returns a zero length plan, planning failed
    if (!co_->sbpl_global_planner->makePlan(start, goal, *plan, astar_path_, sbpl_broader_, state_ != A_PLANNING) || plan->empty()) {
      ROS_ERROR("[ASTAR PLANNER] sbpl failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
  } else {
    // astar plan, it needs call set_fix_path to generate astar_path_
    ROS_INFO("[ASTAR PLANNER] astar plan to the global_goal_");

    using_sbpl_directly_ = false;
    last_using_bezier_ = false;
//    if (!co_->astar_global_planner->makePlan(start, goal, *plan) || plan->empty()) {
    if (!co_->astar_global_planner->makePlan(start, goal, *plan) || plan->empty()) {
      ROS_ERROR("[ASTAR PLANNER] astar failed to find a plan to point (%.2f, %.2f)", global_goal_.pose.position.x, global_goal_.pose.position.y);
      return false;
    }

    // assign to astar_path_
    std::vector<fixpattern_path::PathPoint> path;
    for (int i = 0; i < plan->size(); i += 3) {
      path.push_back(fixpattern_path::GeometryPoseToPathPoint(plan->at(i).pose));
    }
    path.push_back(fixpattern_path::GeometryPoseToPathPoint(plan->back().pose));
    ROS_INFO("[ASTAR PLANNER] got astar plan path size = %zu", path.size());
    astar_path_.set_fix_path(start, path, true); 
    double path_length_diff = astar_path_.Length() - co_->fixpattern_path->Length();
    ROS_INFO("[ASTAR PLANNER] length (astar_path_ - fix_path_) = %lf", path_length_diff);
    if (co_->fixpattern_path->Length() > 0.5 && path_length_diff > co_->max_path_length_diff) {
      ROS_ERROR("[ASTAR PLANNER] astar got a farther path, abandon it and return false");
      return false;
    }
    
/*
    // get sbpl goal and goal direction
    size_t i = 0;
    double dis_accu = 0.0;
    for (; i < plan->size() - 1; ++i) {
      dis_accu += PoseStampedDistance(plan->at(i), plan->at(i + 1));
      if (dis_accu > co_->sbpl_max_distance)
        break;
    }
    size_t end_i = std::min(i + 3, plan->size() - 1);
    double yaw = fixpattern_path::CalculateDirection(plan->at(i).pose, plan->at(end_i).pose);
    tf::Quaternion temp;
    temp.setRPY(0, 0, yaw);
    plan->at(i).pose.orientation.x = temp.getX();
    plan->at(i).pose.orientation.y = temp.getY();
    plan->at(i).pose.orientation.z = temp.getZ();
    plan->at(i).pose.orientation.w = temp.getW();
    // sbpl make plan, use plan->at(i) as goal
    geometry_msgs::PoseStamped temp_goal = plan->at(i);
    // if last plan was success due to sbpl_broader_ and new goal is not far
    // away, we'll still use sbpl_broader_
    bool sbpl_broader = sbpl_broader_;
    if (PoseStampedDistance(temp_goal, success_broader_goal_) < 0.1)
      sbpl_broader = true;
    if (!co_->sbpl_global_planner->makePlan(start, temp_goal, *plan, astar_path_, sbpl_broader, state_ != A_PLANNING) || plan->empty()) {
      ROS_ERROR("[ASTAR PLANNER] sbpl failed to find a plan to point (%.2f, %.2f)", temp_goal.pose.position.x, temp_goal.pose.position.y);
      return false;
    }
*/
  }

  return true;
}

void AStarController::PublishZeroVelocity() {
  geometry_msgs::Twist cmd_vel;
  cmd_vel_ratio_ = 1.0;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  co_->vel_pub->publish(cmd_vel);
  last_valid_cmd_vel_ = cmd_vel;
//  ROS_WARN("[ASTAR CONTROLLER] Publish Zero Velocity!");
}

void AStarController::WakePlanner(const ros::TimerEvent& event) {
  // we have slept long enough for rate
  planner_cond_.notify_one();
}

double GetTimeInSeconds() {
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec + 0.000001 * t.tv_usec;
}

void AStarController::PlanThread() {
  ROS_INFO("[ASTAR PLANNER] Starting planner thread...");
  ros::NodeHandle n;
  ros::Timer timer;
  bool wait_for_wake = false;
  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  double start_t;
  while (n.ok()) {
    // check if we should run the planner (the mutex is locked)
    while (wait_for_wake || !runPlanner_) {
      // if we should not be running the planner then suspend this thread
      ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
      planner_cond_.wait(lock);
      wait_for_wake = false;
      start_t = GetTimeInSeconds();
      last_valid_plan_ = ros::Time::now();
    }

    ROS_INFO("[ASTAR PLANNER] Plan Start!");
    ros::Time start_time = ros::Time::now();
//    controller_costmap_ros_->getCostmap();
 
    // time to plan! get a copy of the goal and unlock the mutex
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

    // get the starting pose of the robot
    geometry_msgs::PoseStamped start;
    tf::Stamped<tf::Pose> global_pose;
    bool gotStartPose = true;
    bool gotPlan = false;
    if (!controller_costmap_ros_->getRobotPose(global_pose)) {
      gotStartPose = false;
      ROS_ERROR("[ASTAR PLANNER]Unable to get starting pose of robot, unable to create global plan");
    } else {
      tf::poseStampedTFToMsg(global_pose, start);
      start.header.frame_id = co_->global_frame;
    }
    if (state_ == FIX_CONTROLLING && planning_state_ == P_INSERTING_MIDDLE) {
      if (!GetAStarStart(co_->front_safe_check_dis)) {
        ROS_WARN("[ASTAR PLANNER]Unable to get AStar start, take current pose in place, and planning_state_ = BEGIN ");
//        if (planning_state_ == P_INSERTING_MIDDLE) {
        planning_state_ = P_INSERTING_BEGIN;
//        }
      } else {
        start = planner_start_;
        gotStartPose = true;
      }
    }

    planner_start_ = start;
    if(gotStartPose) {
      // run planner
      planner_plan_->clear();
      gotPlan = n.ok() && MakePlan(start, temp_goal, planner_plan_) && !astar_path_.path().empty();
      if (replan_directly_) {
        // bezier failed, just replan
        replan_directly_ = false;
        gotPlan = n.ok() && MakePlan(start, temp_goal, planner_plan_) && !astar_path_.path().empty();
      }
    }

    if (gotPlan) {
      ROS_INFO("[ASTAR PLANNER] Got Plan with %zu points! cost: %lf secs", planner_plan_->size(), GetTimeInSeconds() - start_t);
      // check distance from current pose to the path.front() 
      geometry_msgs::PoseStamped cur_pos;
      controller_costmap_ros_->getRobotPose(global_pose);
      tf::poseStampedTFToMsg(global_pose, cur_pos);
      double distance_diff = PoseStampedDistance(cur_pos, astar_path_.GeometryPath().front());
      if (distance_diff > 0.3 && state_ == A_PLANNING) {
        ROS_WARN("[ASTAR PLANNER] Distance from start to path_front = %lf > 0.3m, continue", distance_diff);
      } else {
        last_valid_plan_ = ros::Time::now();
        new_global_plan_ = true;
        // reset rotate_recovery_dir_
        rotate_recovery_dir_ = 0;
        rotate_failure_times_ = 0;
        astar_planner_timeout_cnt_ = 0;
        lock.lock();
        front_path_.set_path(co_->fixpattern_path->path(), false, false);
        front_goal_ = temp_goal;
        if (taken_global_goal_ || planning_state_ == P_INSERTING_NONE) {
          taken_global_goal_ = false;
          if (using_sbpl_directly_) {
            co_->fixpattern_path->set_sbpl_path(start, astar_path_.path(), true);
          } else {
            co_->fixpattern_path->set_fix_path(start, astar_path_.path(), true);
          }
          first_run_controller_flag_ = true;
          switch_path_ = true;
        } else if (planning_state_ == P_INSERTING_BEGIN) {
          double corner_yaw_diff = state_ == A_PLANNING ? M_PI / 36.0 : M_PI / 3.0;
          co_->fixpattern_path->insert_begin_path(astar_path_.path(), start, temp_goal, false, corner_yaw_diff);
          first_run_controller_flag_ = true;
          switch_path_ = true;
        } else if (planning_state_ == P_INSERTING_END) {
          co_->fixpattern_path->insert_end_path(astar_path_.path());
          first_run_controller_flag_ = true;
        } else if (planning_state_ == P_INSERTING_MIDDLE) {
          co_->fixpattern_path->insert_middle_path(astar_path_.path(), start, temp_goal);
          front_safe_check_cnt_ = 0; // only set 0 after getting new fix_path
          switch_path_ = true;
          // first_run_controller_flag_ = true;
        } else { // unkonw state
          // switch to FIX_CLEARING state
          gotPlan = false;
          runPlanner_ = false;
          switch_path_ = false;
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_RECOVERY_R;
          ROS_ERROR("[ASTAR CONTROLLER] planning_state_ unknown, enter recovery");
        }

        if (gotPlan) {
          runPlanner_ = false;
          state_ = FIX_CONTROLLING;
        } 

        lock.unlock();
      }
    } else if (state_ == A_PLANNING) {  // if we didn't get a plan and we are in the planning state (the robot isn't moving)
      ROS_ERROR("[ASTAR PLANNER] No Plan...");
//      sbpl_broader_ = true;
      ros::Time attempt_end = last_valid_plan_ + ros::Duration(co_->planner_patience);
      // ros::Time attempt_end = ros::Time::now() + ros::Duration(co_->planner_patience);
      // check if we've tried to make a plan for over our time limit
      lock.lock();
      if (ros::Time::now() > attempt_end && runPlanner_) {
        // don't allow plan, as RotateRecovery needs global costmap
        runPlanner_ = false;
        PublishZeroVelocity();
        // switch to FIX_CLEARING state
        state_ = FIX_CLEARING;
        recovery_trigger_ = FIX_RECOVERY_R;
        planning_state_ = P_INSERTING_BEGIN;
        ++astar_planner_timeout_cnt_;
        ROS_ERROR("[ASTAR PLANNER] Alarm Here!!! Not got plan until planner_patience, enter recovery; timeout_cnt = %d", astar_planner_timeout_cnt_);
      } else if (runPlanner_) {
        // to update global costmap
        usleep(500000);
//        GetAStarGoal(start);
      }
      lock.unlock();
    } else if (state_ == FIX_CONTROLLING && planning_state_ == P_INSERTING_MIDDLE) { 
      ROS_WARN("[ASTAR PLANNER] Plan middle path failed, just return!");
      lock.lock();
      runPlanner_ = false;
      front_safe_check_cnt_ = 0; // only set 0 after getting new fix_path
      state_ = FIX_CONTROLLING;
      lock.unlock();
    }
    // take the mutex for the next iteration
    lock.lock();

    // setup sleep interface if needed
    if (co_->planner_frequency > 0) {
      ros::Duration sleep_time = (start_time + ros::Duration(1.0 / co_->planner_frequency)) - ros::Time::now();
      if (sleep_time > ros::Duration(0.0)) {
        wait_for_wake = true;
        timer = n.createTimer(sleep_time, &AStarController::WakePlanner, this);
      }
    }
    ROS_INFO("[ASTAR PLANNER] Plan End!");
  }
}

bool AStarController::Control(BaseControlOption* option, ControlEnvironment* environment, bool first_run_flag) {
  ROS_INFO("[ASTAR CONTROLLER] Switch to Astar Controller!");
  co_ = reinterpret_cast<AStarControlOption*>(option);
  env_ = environment;
  global_goal_.pose = co_->global_planner_goal->pose; 
  global_goal_.header.frame_id = co_->global_frame;

  while (!HandleLocalizationRecovery()) {
    ROS_WARN("[ASTAR CONTROLLER] localization failed! Recovery now by inplace_rotating");
    usleep(500000);
  }
  usleep(50000);

  taken_global_goal_ = false;
  bool start_path_got = false;
  double cur_goal_distance;
  unsigned int path_status; 
	
  controller_costmap_ros_->getCostmap(); // costmap only updated when we calling getCostmap()
  geometry_msgs::PoseStamped current_position;
  usleep(10000);
  tf::Stamped<tf::Pose> global_pose;
  if (!controller_costmap_ros_->getRobotPose(global_pose)) {
    ROS_WARN("Unable to get starting pose of robot, unable to create sbpl plan");
    terminate_controller_ = true;
    return true;
  } else {
    tf::poseStampedTFToMsg(global_pose, current_position);
    cur_goal_distance = PoseStampedDistance(current_position, global_goal_); 
  }
/*  
  while(1) {
    tf::poseStampedTFToMsg(global_pose, current_position);
    cur_goal_distance = PoseStampedDistance(current_position, global_goal_); 
    HandleGoingBack(current_position, 0.10);
    usleep(500000);
  }
*/

  cur_goal_distance = PoseStampedDistance(current_position, global_goal_);
  // check if current_position is invalid
  double x = current_position.pose.position.x;
  double y = current_position.pose.position.y;
  double yaw = tf::getYaw(current_position.pose.orientation);
  // check if oboscal in footprint, yes - recovery; no - get new goal and replan
  if (footprint_checker_->FootprintCenterCost(x, y, yaw, co_->footprint_center_points) < 0) {
//  if (footprint_checker_->FootprintCost(x, y, yaw, footprint_spec_, 0.0, 0.0) < 0) {
     ROS_WARN("[FIXPATTERN CONTROLLER] footprint cost check < 0!, switch to Recovery");
     if (!HandleRecovery(current_position)) {
       ROS_ERROR("[FIXPATTERN CONTROLLER] footprint not safe and recovery failed, terminate!");
       terminate_controller_ = true;
	     return true;
     }
  }
  // first run: just get sbpl Path, and insert to fixpattern_path 
  if (first_run_flag) {
    bool got_goal = false;
    // TODO(lizhen) if plan failed?
    if (cur_goal_distance < 2.5) { //co_->sbpl_max_distance
      ROS_INFO("[ASTAR CONTROLLER] distance from current pose to goal = %lf, take global goal as planner_goal", cur_goal_distance);
//      planner_goal_ = global_goal_;
//      taken_global_goal_ = true;
      if (GetAStarGoal(current_position)) {
        state_ = A_PLANNING;
        planning_state_ = P_INSERTING_NONE;
        got_goal = true;
//      } else {
//        state_ = FIX_CLEARING;
//        recovery_trigger_ = FIX_RECOVERY_R;
//        ROS_WARN("[ASTAR_CONTROLLER] get Astar goal fialed, siwtch to FIX_RECOVERY_R");
      }
    }

    if (!got_goal) {
      ROS_INFO("[ASTAR_CONTROLLER] get Astar Path, and set as fixpattern_path");
      // setSaticCostmap only for inital AStar Plan 
      co_->astar_global_planner->setStaticCosmap(true);
      start_path_got = GetAStarInitalPath(current_position, global_goal_);
      co_->astar_global_planner->setStaticCosmap(false);
      if (start_path_got) {
        std::vector<geometry_msgs::PoseStamped> fix_path = co_->fixpattern_path->GeometryPath();
        if (CheckFixPathFrontSafe(fix_path, co_->front_safe_check_dis) < 1.5) {
          ROS_WARN("[ASTAR_CONTROLLER] CheckFixPathFrontSafe failed, switch to A_PLANNING state");
          if (GetAStarGoal(current_position, obstacle_index_)) {
            state_ = A_PLANNING;
            planning_state_ = P_INSERTING_BEGIN;
          } else {
            state_ = FIX_CLEARING;
            recovery_trigger_ = FIX_RECOVERY_R;
            ROS_WARN("[ASTAR_CONTROLLER] get Astar goal fialed, siwtch to FIX_RECOVERY_R");
          }
        } else {
          state_ = FIX_CONTROLLING;
        }
      } else { 
        planner_goal_ = global_goal_;
        taken_global_goal_ = true;
        state_ = A_PLANNING;
        planning_state_ = P_INSERTING_NONE;
      }
    }
  }

  co_->fixpattern_local_planner->reset_planner();

  // initialize some flag
  first_run_controller_flag_ = true;
  using_sbpl_directly_ = false;
  last_using_bezier_ = false;
  replan_directly_ = false;
  // disable the planner
/*
  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  runPlanner_ = false;
//  planner_cond_.notify_one();
  lock.unlock();
*/
	// set state_ 

  ros::Rate r(co_->controller_frequency);

  // we want to make sure that we reset the last time we had a valid plan and control
  last_valid_control_ = ros::Time::now();
  last_valid_plan_ = ros::Time::now();
  last_oscillation_reset_ = ros::Time::now();

  ros::NodeHandle n;
  while (n.ok()) {
    // if is paused but run, continue
    if (env_->pause_flag && env_->run_flag) {
      ROS_WARN("[ASTAR CONTROLLER] Control Paused , just stop here!");
      PublishZeroVelocity();
      r.sleep();
      continue;
    }
    // if terminated, return true directly
    if (!env_->run_flag) {
      co_->fixpattern_path->EraseToPoint(fixpattern_path::GeometryPoseToPathPoint(global_goal_.pose));
      ResetState();

      // disable the planner thread
      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      runPlanner_ = false;
      lock.unlock();

      // TODO(chenkan): check if this is needed
      co_->fixpattern_local_planner->reset_planner();

      // we need to notify fixpattern_path
      co_->fixpattern_path->FinishPath();

      ROS_WARN("[ASTAR CONTROLLER] Control Teminated, stop and return");
      return true;
    }

    // for timing that gives real time even in simulation
    ros::WallTime start = ros::WallTime::now();

    // the real work on pursuing a goal is done here
    bool done = ExecuteCycle();

    // if we're done, then we'll return from execute
    if (done) {
      if (terminate_controller_) {
        terminate_controller_ = false;
        return true;
      } else {
        return false; // false = not terminate
      }
    }
    // check if execution of the goal has completed in some way

    ros::WallDuration t_diff = ros::WallTime::now() - start;
    ROS_DEBUG_NAMED("autoscrubber", "Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / co_->controller_frequency) && state_ == FIX_CONTROLLING)
      ROS_ERROR("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", co_->controller_frequency, r.cycleTime().toSec());
  } // while (n.ok())

  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  // wake up the planner thread so that it can exit cleanly
//  lock.lock();
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  // if !n.ok(), we don't want to switch controllers
  return true;
}

bool AStarController::IsGoalSafe(const geometry_msgs::PoseStamped& goal_pose, double goal_front_check_dis, double goal_back_check_dis) {
  if (!IsGoalFootprintSafe(0.5, 0.0, goal_pose)) {
    return false;
  }
  double resolution = controller_costmap_ros_->getCostmap()->getResolution();
  int front_num_step = goal_front_check_dis / resolution;
  int back_num_step = (-1) * goal_back_check_dis / resolution;
	
  double yaw = tf::getYaw(goal_pose.pose.orientation);
  std::vector<geometry_msgs::PoseStamped> path;
  for (int i = back_num_step; i <= front_num_step; ++i) {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = goal_pose.pose.position.x + i * resolution * cos(yaw);
    p.pose.position.y = goal_pose.pose.position.y + i * resolution * sin(yaw);
    p.pose.orientation = goal_pose.pose.orientation;
    path.push_back(p);
  }
  for (int i = 0; i < path.size(); ++i) {
    if (footprint_checker_->FootprintCost(path[i].pose.position.x, path[i].pose.position.y, yaw, footprint_spec_, 0.0, 0.0) < 0) {
      return false;
    }
  }
  return true;
}

bool AStarController::IsGoalFootprintSafe(double goal_safe_dis_a, double goal_safe_dis_b, const geometry_msgs::PoseStamped& pose) {
  std::vector<geometry_msgs::PoseStamped> fix_path = co_->fixpattern_path->GeometryPath();
  int goal_index = -1;
  for (int i = 0; i < static_cast<int>(fix_path.size()); ++i) {
    if (PoseStampedDistance(fix_path[i], pose) < 0.0001) {
      goal_index = i;
      break;
    }
  }
  if (goal_index == -1) {
    return true;
  }
  double free_dis_a = 0.0;
  for (int i = goal_index - 1; i >= 0; i -= 5) {
    double x = fix_path[i].pose.position.x;
    double y = fix_path[i].pose.position.y;
    double yaw = tf::getYaw(fix_path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0) {
//      ROS_WARN("[ASTAR CONTROLLER] goal front not safe");
      return false;
    }
    free_dis_a += PoseStampedDistance(fix_path[i], fix_path[i + 5]);
    if (free_dis_a >= goal_safe_dis_a) {
      break;
    }
  }
  double free_dis_b = 0.0;
  for (int i = goal_index + 1; i < fix_path.size(); i += 5) {
    double x = fix_path[i].pose.position.x;
    double y = fix_path[i].pose.position.y;
    double yaw = tf::getYaw(fix_path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0) {
//      ROS_WARN("[ASTAR CONTROLLER] goal back not safe");
      return false;
    }
    free_dis_b += PoseStampedDistance(fix_path[i], fix_path[i - 5]);
    if (free_dis_b >= goal_safe_dis_b) {
      break;
    }
  }
  return true;
}

bool AStarController::IsPathFootprintSafe(const std::vector<geometry_msgs::PoseStamped>& path,
                                          const std::vector<geometry_msgs::Point>& circle_center_points,
                                          double length) {
  double accu_dis = 0.0;
  for (int i = 0; i < path.size(); i += 5) {
    double yaw = tf::getYaw(path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(path[i].pose.position.x, path[i].pose.position.y,
                                             yaw, circle_center_points) < 0) {
      return false;
    }
    if (i != 0) accu_dis +=PoseStampedDistance(path[i], path[i - 5]);
    if (accu_dis >= length) return true;
  }
  return true;
}

bool AStarController::IsPathFootprintSafe(const fixpattern_path::Path& fix_path, double length) {
  std::vector<geometry_msgs::PoseStamped> path = fix_path.GeometryPath();
  if (IsPathFootprintSafe(path, co_->circle_center_points, length)) {
    return true;
  }

  if (fabs(co_->sbpl_footprint_padding) < GS_DOUBLE_PRECISION) return false;
  ROS_WARN("[ASTAR CONTROLLER] origin fix_path footprint is not safe");
	
  // if not safe, let's cast some padding to footprint
  std::vector<geometry_msgs::Point> circle_center_points_padding_1 = co_->circle_center_points;
  for (auto&& p : circle_center_points_padding_1) p.y +=co_->sbpl_footprint_padding;
  if (IsPathFootprintSafe(path, circle_center_points_padding_1, length)) {
    return true;
  }
  ROS_WARN("[ASTAR CONTROLLER] pandding up fix_path footprint is not safe");

  // okay okay, the other padding
  std::vector<geometry_msgs::Point> circle_center_points_padding_2 = co_->circle_center_points;
  for (auto&& p : circle_center_points_padding_2) p.y -= co_->sbpl_footprint_padding;
  if (IsPathFootprintSafe(path, circle_center_points_padding_2, length)) {
    return true;
  }
  ROS_WARN("[ASTAR CONTROLLER] pandding down fix_path footprint is not safe");

  // at last...
  return false;
}

double AStarController::CheckFixPathFrontSafe(const std::vector<geometry_msgs::PoseStamped>& path, double front_safe_check_dis) {
  double accu_dis = 0.0;
  double off_obstacle_dis = 0.0;
  bool cross_obstacle = false;
  int i, j;
  unsigned int temp_goal_index = 0;
  for (i = 0; i < path.size(); i += 5) {
    double yaw = tf::getYaw(path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(path[i].pose.position.x, path[i].pose.position.y,
                                             yaw, co_->circle_center_points) < 0) {
      cross_obstacle = true;
      obstacle_index_ = i;
      break;
    }
    if (i != 0) accu_dis += PoseStampedDistance(path[i], path[i - 5]);
    if (temp_goal_index ==0 && accu_dis >= 1.5) temp_goal_index = i;
    if (accu_dis >= front_safe_check_dis) break;
  }
  if (!cross_obstacle && i >= path.size())
    accu_dis = front_safe_check_dis + 0.001;

  front_goal_index_ = temp_goal_index;
  return accu_dis;
}

bool AStarController::GetAStarStart(double front_safe_check_dis) {
  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  double accu_dis = 0.0;
  double off_obstacle_dis = 0.0;
  bool cross_obstacle = false;
  bool start_got = false;
  int i, j;
  for (i = 0; i < path.size(); i += 5) {
    double yaw = tf::getYaw(path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(path[i].pose.position.x, path[i].pose.position.y,
                                             yaw, co_->circle_center_points) < 0) {
      cross_obstacle = true;
      obstacle_index_ = i;
      ROS_INFO("[ASTAR CONTROLLER] GetAStarStart: obstacle_index_ = %d", obstacle_index_);
      break;
    }
    if (i != 0) accu_dis += PoseStampedDistance(path[i], path[i - 5]);
    if (accu_dis >= front_safe_check_dis) break;
  }
  if (cross_obstacle) {
    double start_dis;
    if (accu_dis > 1.2) {
      start_dis = 1.0;
    } else if (accu_dis > 1.0) {
      start_dis = 0.8;
    } else if (accu_dis > 0.7) {
      start_dis = 0.6;
    } else {
      start_dis = 0.0;
    }
    if (start_dis > 0.0) {
      for (j = obstacle_index_; j > 2; j -= 2) {
        off_obstacle_dis += PoseStampedDistance(path[j], path[j - 2]);
        if (off_obstacle_dis > start_dis) {
          planner_start_ = path.at(j);
          start_got = true;
          ROS_INFO("[ASTAR CONTROLLER] GetAStarStart: taken point front dis = %lf", accu_dis - off_obstacle_dis);
          break;
        }
      }
    } else {
      planner_start_ = path.front();
      ROS_WARN("[ASTAR CONTROLLER] GetAStarStart: taken path.front as start point");
    }
/*
    if (accu_dis > 1.5) {
//      double start_dis = 1.3;
      double start_dis = accu_dis > 1.7 ?  accu_dis - 0.55 : 1.15;
      for (j = obstacle_index_; j > 2; j -= 2) {
        off_obstacle_dis += PoseStampedDistance(path[j], path[j - 2]);
        if (off_obstacle_dis > start_dis) {
          planner_start_ = path.at(j);
          start_got = true;
          ROS_INFO("[ASTAR CONTROLLER] GetAStarStart: taken point front dis = %lf", accu_dis - off_obstacle_dis);
          break;
        }
      }
   } else if (accu_dis > 1.0) {
      double start_dis = accu_dis > 1.3 ?  accu_dis - 0.35 : 0.95;
      for (j = obstacle_index_; j > 2; j -= 2) {
        off_obstacle_dis += PoseStampedDistance(path[j], path[j - 2]);
        if (off_obstacle_dis > start_dis) {
          planner_start_ = path.at(j);
          start_got = true;
          ROS_INFO("[ASTAR CONTROLLER] GetAStarStart: taken point front dis = %lf", accu_dis - off_obstacle_dis);
          break;
        }
      }
    } else if (accu_dis > 0.7) {
      double start_dis = accu_dis > 0.85 ?  accu_dis - 0.15 : 0.70;
      for (j = obstacle_index_; j > 2; j -= 2) {
        off_obstacle_dis += PoseStampedDistance(path[j], path[j - 2]);
        if (off_obstacle_dis > start_dis) {
          planner_start_ = path.at(j);
          start_got = true;
          ROS_INFO("[ASTAR CONTROLLER] GetAStarStart: taken point front dis = %lf", accu_dis - off_obstacle_dis);
          break;
        }
      }
    }else {
      if (path.size() > 20) {
        planner_start_ = path.at(10);
        ROS_WARN("[ASTAR CONTROLLER] GetAStarStart: taken point front index = 10");
      } else {
        planner_start_ = path.front();
        ROS_WARN("[ASTAR CONTROLLER] GetAStarStart: taken path.front as start point");
      }
    }
*/
  }
  planner_start_.header.frame_id = co_->global_frame;
  astar_start_pub_.publish(planner_start_);
  return start_got;
}

bool AStarController::IsFixPathFrontSafe(double front_safe_check_dis) {

  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  if (IsPathFootprintSafe(path, co_->circle_center_points, front_safe_check_dis)) {
    return true;
  }

  ROS_WARN("[Fixpattern_path] origin path is not safe");
  if (fabs(co_->fixpattern_footprint_padding) < GS_DOUBLE_PRECISION) return false;

  // if not safe, let's cast some padding to footprint
  std::vector<geometry_msgs::Point> circle_center_points_padding_1 = co_->circle_center_points;
  for (auto&& p : circle_center_points_padding_1) p.y +=co_->fixpattern_footprint_padding;
  if (IsPathFootprintSafe(path, circle_center_points_padding_1, front_safe_check_dis)) {
    return true;
  }
  ROS_WARN("[Fixpattern_path] Pandding up path is not safe");

  // okay okay, the other padding
  std::vector<geometry_msgs::Point> circle_center_points_padding_2 = co_->circle_center_points;
  for (auto&& p : circle_center_points_padding_2) p.y -= co_->fixpattern_footprint_padding;
  if (IsPathFootprintSafe(path, circle_center_points_padding_2, front_safe_check_dis)) {
    return true;
  }
  ROS_WARN("[Fixpattern_path] Pandding down path is not safe");

  // at last...
  return false;
}

bool AStarController::NeedBackward(const geometry_msgs::PoseStamped& pose, double distance) {
  double yaw = tf::getYaw(pose.pose.orientation);
  double resolution = controller_costmap_ros_->getCostmap()->getResolution() / 3.0;
  int num_step = distance / resolution;
//  ROS_INFO("[ASTAR CONTROLLER] needbackward check: distance = %lf, num_step = %d", distance, num_step);
	
  std::vector<geometry_msgs::PoseStamped> path;
  for (int i = 0; i <= num_step; ++i) {
    geometry_msgs::PoseStamped p;
    p.pose.position.x = pose.pose.position.x + i * resolution * cos(yaw);
    p.pose.position.y = pose.pose.position.y + i * resolution * sin(yaw);
    p.pose.orientation = pose.pose.orientation;
    path.push_back(p);
  }
  for (int i = 0; i < path.size(); ++i) {
    if (footprint_checker_->CircleCenterCost(path[i].pose.position.x, path[i].pose.position.y, yaw,
                                             co_->circle_center_points) < 0) {
      ROS_INFO("[ASTAR CONTROLLER] distance = %lf, not safe step = %d", distance, i);
      return true;
    }
  }
  return false;
}

double AStarController::PoseStampedDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

bool AStarController::ExecuteCycle() {
  // we need to be able to publish velocity commands
  double t0, t1, t2, t3, t4, t5;
  t0 = GetTimeInSeconds();

  geometry_msgs::Twist cmd_vel;
  // get curent position
  tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped current_position;
  if (!controller_costmap_ros_->getRobotPose(global_pose)) {
    ROS_ERROR("[ASTAR CONTROLLER] cannot get current position, terminate this ExecuteCycle");
    return false;
  } else {
    tf::poseStampedTFToMsg(global_pose, current_position);
  }
  double cur_goal_distance = PoseStampedDistance(current_position, global_goal_);
//  ROS_INFO("[ASTAR CONTROLLER]:cur_goal_distance = %lf", cur_goal_distance);
  // check to see if we've moved far enough to reset our oscillation timeout
  if (PoseStampedDistance(current_position, oscillation_pose_) >= co_->oscillation_distance) {
    last_oscillation_reset_ = ros::Time::now();
    oscillation_pose_ = current_position;
  }

  // check that the observation buffers for the costmap are current, we don't want to drive blind
  controller_costmap_ros_->getCostmap();
  if (!controller_costmap_ros_->isCurrent()) {
    ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
    PublishZeroVelocity();
    // TODO(chenkan) do something to notify GUI
    return false;
  }

  // if we have a new plan then grab it and give it to the controller
  // TODO(chenkan): need to check if planner_mutex_ needs to be locked
  // for new_global_plan_ here
  if (new_global_plan_) {
    // make sure to set the new plan flag to false
    new_global_plan_ = false;
    ROS_INFO("[ASTAR CONTROLLER] get new sbpl plan");
    ROS_DEBUG_NAMED("autoscrubber", "Got a new plan...swap pointers");
  
    // in case new plan has different rotate dir
    co_->fixpattern_local_planner->reset_planner();

    ROS_DEBUG_NAMED("autoscrubber", "pointers swapped!");
  }
  if (!localization_valid_) {
    state_ = FIX_CLEARING;
    recovery_trigger_ = LOCATION_RECOVERY_R;
  }
  
  t1 = GetTimeInSeconds();
  if (t1 - t0 > 0.02) {
    ROS_INFO("get costmap cost %lf sec", t1 - t0);
  }
  // the move_base state machine, handles the control logic for navigation
  switch (state_) {
    // if we are in a planning state, then we'll attempt to make a plan
    case A_PLANNING:
      ROS_INFO("[ASTAR CONTROLLER] in PLANNING state");
      {
        boost::mutex::scoped_lock lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
      }
      ROS_DEBUG_NAMED("autoscrubber", "Waiting for plan, in the planning state.");
      break;

    case FIX_CONTROLLING:
      ROS_INFO("[FIXPATTERN CONTROLLER] in FIX_CONTROLLING state");
      ROS_DEBUG_NAMED("autoscrubber", "In controlling state.");

      // check to see if we've reached our goal
      if (co_->fixpattern_local_planner->isGoalReached()) {
        ROS_WARN("[FIXPATTERN CONTROLLER] fixpattern goal reached");
        ROS_DEBUG_NAMED("autoscrubber", "Goal reached!");
        PublishZeroVelocity();
        ResetState();
        // reset fixpattern_local_planner
        co_->fixpattern_local_planner->reset_planner();	

        // we need to notify fixpattern_path
        co_->fixpattern_path->FinishPath();

        // publish goal reached 
//        PublishGoalReached();
//        ROS_WARN("[FIXPATTERN CONTROLLER] global goal reached, teminate controller");
//        terminate_controller_ = true;
//        return true;  //(lee)

        // check is global goal reached
        if (!IsGlobalGoalReached(current_position, global_goal_, 
                                 co_->fixpattern_local_planner->xy_goal_tolerance_, co_->fixpattern_local_planner->yaw_goal_tolerance_)) {
          ROS_WARN("[FIXPATTERN CONTROLLER] global goal not reached yet, swtich to PLANNING state");
          PublishZeroVelocity();
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
          ROS_ERROR("[FIXPATTERN CONTROLLER] HandleGoingBack, entering A_PLANNING state");
/*
          planner_goal_ = global_goal_;
          taken_global_goal_ = true;
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          planning_state_ = P_INSERTING_NONE;
          lock.unlock();
          state_ = A_PLANNING;
          recovery_trigger_ = A_PLANNING_R;
*/
          break;  //(lee)
        } else  {
          // publish goal reached 
          PublishGoalReached();
          ROS_WARN("[FIXPATTERN CONTROLLER] global goal reached, teminate controller");
          terminate_controller_ = true;
          return true;  //(lee)
        }
      }
/*      {
        bool need_backward = HandleGoingBack(current_position);
        // check if need going back
        if (need_backward) {
          PublishZeroVelocity();
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
          ROS_ERROR("[FIXPATTERN CONTROLLER] HandleGoingBack, entering A_PLANNING state");
          return false;
        }
      }
*/

    // check if swtich to origin path needed
    HandleSwitchingPath(current_position);

    t2 = GetTimeInSeconds();
    if (t2 - t1 > 0.04) {
      ROS_INFO("check reached goal and HandleSwitch cost %lf sec", t2 - t1);
    }
//    if (!co_->fixpattern_local_planner->isRotatingToGoal()) {
    {
        // we'll Prune the path first as we don't want to navigate back when
        // trigger front_safe while robot still moves
        // get current pose of the vehicle && prune fixpattern path
        // we'll not prune any point when first run 
        // TODO(lizhen) check if needed :first_run_controller_flag_

        if(first_run_controller_flag_) {
          first_run_controller_flag_ = false;
        } else {
          if (!co_->fixpattern_local_planner->isGoalXYLatched()) {
  //        if (cur_goal_distance > co_->fixpattern_local_planner->xy_goal_tolerance_) {
           if(co_->fixpattern_local_planner->isRotatingToGoalDone()) {
             co_->fixpattern_path->PruneCornerOnStart();
             co_->fixpattern_local_planner->resetRotatingToGoalDone();
             ROS_INFO("[FIXPATTERN CONTROLLER] Prune Corner Point On Start");  
           } else {
             t2 = GetTimeInSeconds();
             if (!co_->fixpattern_path->Prune(fixpattern_path::GeometryPoseToPathPoint(current_position.pose), co_->max_offroad_dis, co_->max_offroad_yaw, true)) {
               ROS_WARN("[FIXPATTERN CONTROLLER] Prune fix path failed, swtich to FIX_CLEARING");  
               PublishZeroVelocity();
               state_ = FIX_CLEARING;
               recovery_trigger_ = FIX_GETNEWGOAL_R;
               break;
             }
             t3 = GetTimeInSeconds();
             if (t3 - t2 > 0.04) {
               ROS_INFO("Prune Path cost %lf sec", t3 - t2);
             }
           }
          }
        }

      // check for an oscillation condition
      if (co_->oscillation_timeout > 0.0 &&
         last_oscillation_reset_ + ros::Duration(co_->oscillation_timeout) < ros::Time::now()) {
        ROS_INFO("[FIXPATTERN CONTROLLER] oscillation to CLEARING");
        PublishZeroVelocity();
        state_ = FIX_CLEARING;
        recovery_trigger_ = FIX_OSCILLATION_R;
      }

//     if (!co_->fixpattern_local_planner->isRotatingToGoal()) {
      {      
        cmd_vel_ratio_ = 1.0;
        std::vector<geometry_msgs::PoseStamped> fix_path = co_->fixpattern_path->GeometryPath();
        double front_safe_dis = CheckFixPathFrontSafe(fix_path, co_->front_safe_check_dis);
        if (cur_goal_distance < co_->goal_safe_check_dis && front_safe_dis < co_->front_safe_check_dis && !IsGoalSafe(global_goal_, 0.10, 0.15)) { // 
//        if ((cur_goal_distance < co_->front_safe_check_dis) && !IsGoalSafe(global_goal_)) { // 
//        if ((co_->fixpattern_path->Length() < co_->front_safe_check_dis) && !IsGoalSafe(global_goal_) && front_safe_dis < co_->front_safe_check_dis) {  
          if (front_safe_dis < 0.5) {
            PublishZeroVelocity();
            bool is_goal_safe = false;
            ros::Rate check_rate(10);
            ros::Time check_end_time = ros::Time::now() + ros::Duration(co_->goal_safe_check_duration);
            unsigned int check_goal_safe_cnt = 0;
            while (ros::Time::now() < check_end_time) {
              if (IsGoalSafe(global_goal_, 0.10, 0.15)) {
                if (++check_goal_safe_cnt > 5) {
                  is_goal_safe = true;
                  ROS_WARN("[FIXPATTERN CONTROLLER] Check global goal safe, continue!");
                  break;
                }
              } else {
                check_goal_safe_cnt = 0;
              }
              ROS_WARN("[FIXPATTERN CONTROLLER] Check global goal not safe, stop here!");
              check_rate.sleep();
            }
            if (!is_goal_safe){      
              ROS_ERROR("[FIXPATTERN CONTROLLER] Check global goal not safe, terminate!");
              // disable the planner thread
              boost::unique_lock<boost::mutex> lock(planner_mutex_);
              runPlanner_ = false;
              lock.unlock();

              ResetState();
              // we need to notify fixpattern_path
              co_->fixpattern_path->FinishPath();

              // publish goal reached 
              PublishGoalReached();

              // TODO(chenkan): check if this is needed
              co_->fixpattern_local_planner->reset_planner();
              // Goal not reached, but we will stop and exit ExecuteCycle
              terminate_controller_ = true;
              return true;
            }
          }
        } else if (front_safe_dis < co_->front_safe_check_dis) { // check front safe distance
          if (front_safe_dis <= 0.6) {
            front_safe_check_cnt_ = 0;
            PublishZeroVelocity();
            ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
            ros::Rate r(10);
            bool front_safe = false;
            unsigned int front_safe_cnt = 0;
            while (ros::Time::now() < end_time) {
              front_safe_dis = CheckFixPathFrontSafe(fix_path, co_->front_safe_check_dis);
              if (front_safe_dis > 0.7) {
                if (++front_safe_cnt > 2) {
                  front_safe = true;
                  break;
                }
              } 
              ROS_WARN("[FIXPATTERN CONTROLLER] path front not safe, dis = %lf <= 0.5, stop here until stop_duration", front_safe_dis);
              r.sleep();
            }
            if(!front_safe) {
//              if (front_safe_dis <= 0.25) {
                // check if need going back
              controller_costmap_ros_->getRobotPose(global_pose);
              tf::poseStampedTFToMsg(global_pose, current_position);
//                GetCurrentPosition(current_position);
              HandleGoingBack(current_position);
//              }
              ROS_ERROR("[FIXPATTERN CONTROLLER] !IsPathFrontSafe dis = %lf,stop and switch to CLEARING", front_safe_dis);
              state_ = FIX_CLEARING;
              recovery_trigger_ = FIX_GETNEWGOAL_R;
            } else {
              // clear local planner error cnt, to avoid it stop again
              fix_local_planner_error_cnt_ = 0;
            }
            break;
          } else {
            ROS_WARN("[FIXPATTERN CONTROLLER] !IsPathFrontSafe dis = %lf > 0.5, check_cnt = %d", front_safe_dis, front_safe_check_cnt_);
            if (front_safe_dis < 1.0) 
              cmd_vel_ratio_ = 0.5;
            else if (front_safe_dis < 1.7) 
              cmd_vel_ratio_ = 0.7;
            if (!runPlanner_ && ++front_safe_check_cnt_ > 10) {
              if (front_safe_dis < 1.5) {
                ROS_WARN("[FIXPATTERN CONTROLLER] Enable PlanThread and continue FIX_CONTROLLING");
//                GetAStarStart(co_->front_safe_check_dis);	
//                CheckFixPathFrontSafe(fix_path, co_->front_safe_check_dis, true); // get planner_start
                if (GetAStarGoal(current_position, obstacle_index_)) {
                  planning_state_ = P_INSERTING_MIDDLE;
                  // enable the planner thread in case it isn't running on a clock
                  boost::unique_lock<boost::mutex> lock(planner_mutex_);
                  runPlanner_ = true;
                  planner_cond_.notify_one();
                  lock.unlock();
                }
              } else {
                --front_safe_check_cnt_;
              }
            }
          }
        } else {
          front_safe_check_cnt_ = 0;
        }
      }
    } // is rotating to goal

      t4 = GetTimeInSeconds();
      if (t4 - t3 > 0.04) {
        ROS_INFO("Check front path cost %lf sec", t4 - t3);
      }

      {
        if (!co_->fixpattern_local_planner->setPlan(co_->fixpattern_path->path(), co_->global_frame)) {
          // ABORT and SHUTDOWN COSTMAPS
          ROS_ERROR("Failed to pass global plan to the controller, aborting.");
          ResetState();

          return true;
        }
        std::vector<geometry_msgs::PoseStamped> plan = co_->fixpattern_path->GeometryPath();
        for (auto&& p : plan) {  // NOLINT
          p.header.frame_id = co_->global_frame;
          p.header.stamp = ros::Time::now();
        }
        PublishPlan(fixpattern_pub_, plan);
      }

      {
        // get cmd_vel 
        bool local_planner_ret = co_->fixpattern_local_planner->computeVelocityCommands(fixpattern_local_planner::TRAJECTORY_PLANNER, &cmd_vel);    
        if (!local_planner_ret) {
          ++fix_local_planner_error_cnt_;
          cmd_vel = last_valid_cmd_vel_;
          ROS_WARN("[FIXPATTERN CONTROLLER] local_planner error count = %d", fix_local_planner_error_cnt_);
          // check if need going back
          if (cmd_vel.linear.x > 0.10 && NeedBackward(current_position, 0.05)) {
            ROS_ERROR("[FIXPATTERN CONTROLLER] !IsFrontSafe ,stop and switch to CLEARING");
            PublishZeroVelocity();
            state_ = FIX_CLEARING;
            recovery_trigger_ = FIX_GETNEWGOAL_R;
          }
        } else {
          fix_local_planner_error_cnt_ = 0;
          last_valid_cmd_vel_ = cmd_vel;
        }

        if (fix_local_planner_error_cnt_ < 3) {
          ROS_DEBUG_NAMED("autoscrubber", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
          last_valid_control_ = ros::Time::now();
          cmd_vel.linear.x *= cmd_vel_ratio_;	
          cmd_vel.angular.z *= cmd_vel_ratio_;	
          if (fix_local_planner_error_cnt_ > 0) {
            cmd_vel.linear.x *= 0.75;
            cmd_vel.angular.z *= 0.75;	
          }
          // make sure that we send the velocity command to the base
          co_->vel_pub->publish(cmd_vel);
          last_valid_cmd_vel_ = cmd_vel;
          // notify chassis to launch scrubber
          env_->launch_scrubber = true;
        } else {
          ROS_DEBUG_NAMED("autoscrubber", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(co_->controller_patience);

          // check if we've tried to find a valid control for longer than our time limit
          if (ros::Time::now() > attempt_end) {
            ROS_INFO("[FIXPATTERN CONTROLLER] CONTROLLING exceeds attempt_end");
            // we'll move into our obstacle clearing mode
            // TODO(lizhen): check this variable
            local_planner_error_cnt_ = 0;
            PublishZeroVelocity();
            state_ = FIX_CLEARING;
            recovery_trigger_ = FIX_RECOVERY_R;
          } else {
            // otherwise, if we can't find a valid control, we'll retry, until
            ROS_INFO("[FIXPATTERN CONTROLLER] wait for a valid control");
            // reach controller_patience
            state_ = FIX_CONTROLLING;
            PublishZeroVelocity();
          }
        }
      }

      t5 = GetTimeInSeconds();
      if (t5 - t4 > 0.06) {
        ROS_INFO("Local planner cost %lf sec", t5 - t4);
      }

      break;
			
    // we'll try to launch recovery behaviors
    case FIX_CLEARING:
      ROS_INFO("[FIX CONTROLLER] in FIX_CLEARING state");
      if (recovery_trigger_ == LOCATION_RECOVERY_R) {
        ROS_WARN("[FIX CONTROLLER] in LOCATION_RECOVERY_R state");
        ros::Time end_time = ros::Time::now() + ros::Duration(co_->localization_duration);
        ros::Rate r(10);
        while (ros::Time::now() < end_time && !localization_valid_) {
          ROS_WARN("[FIX CONTROLLER] CLEARING state: waiting for valid localization");
          r.sleep();
        }
        if (HandleLocalizationRecovery()) {
          PublishZeroVelocity();
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
        }
        break;
      }

      if (recovery_trigger_ == FIX_RECOVERY_R) {
        // we will try Going Back first
        HandleGoingBack(current_position);
        double x = current_position.pose.position.x;
        double y = current_position.pose.position.y;
        double yaw = tf::getYaw(current_position.pose.orientation);
        // check if oboscal in footprint, yes - recovery; no - get new goal and replan
//        if (footprint_checker_->FootprintCost(x, y, yaw, footprint_spec_, 0.0, 0.0) < 0) {
        if (footprint_checker_->BroaderFootprintCost(x, y, yaw, co_->footprint_center_points, 0.10, 0.05) < 0 ||
            footprint_checker_->FootprintCenterCost(x, y, yaw, co_->footprint_center_points) < 0) {
          ROS_WARN("[FIXPATTERN CONTROLLER] footprint cost check < 0!, switch to Recovery");
          HandleRecovery(current_position);
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
          break;
/*
          if (HandleRecovery(current_position)) {
            state_ = FIX_CLEARING;
            recovery_trigger_ = FIX_GETNEWGOAL_R;
          } else {
            // TODO(lizhen) Recovery Failed, Alarm here and terminate
            ResetState();
            // disable the planner thread
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            runPlanner_ = false;
            lock.unlock();

            // TODO(chenkan): check if this is needed
            co_->fixpattern_local_planner->reset_planner();

            // we need to notify fixpattern_path
            co_->fixpattern_path->FinishPath();

            // TODO(lizhen) Alarm here
            switch_controller_ = false;
            ROS_ERROR("[FIX CONTROLLER] Recovery failed, terminate");
            return true;
          }
*/
        } else {
          ROS_WARN("[FIXPATTERN CONTROLLER] footprint cost check OK!, switch to Replan");
          if (astar_planner_timeout_cnt_ > 2) {
            ROS_WARN("[FIXPATTERN CONTROLLER] Handle Rotate Recovery");
            RotateRecovery();
          }
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
        }
      }

      // we'll invoke recovery behavior
      if (recovery_trigger_ == FIX_GETNEWGOAL_R) {
        if (astar_planner_timeout_cnt_ > 5) {
//          astar_planner_timeout_cnt_ = 0;
          if (GetAStarTempGoal()) {
            state_ = A_PLANNING;
            recovery_trigger_ = A_PLANNING_R;
            ROS_INFO("[FIX CONTROLLER] CLEARING state: astar_planner_timeout_cnt_ > 3, got temp AStar Goal success! Switch to A_PLANNING");
            break;
          }
        }

        ROS_INFO("[FIX CONTROLLER] in CLEARING state: FIX_GETNEWGOAL_R");
        PublishZeroVelocity();
        bool new_goal_got = false; 
        // get a new astar goal
        ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration / 2.0);
        ros::Rate r(10);
        while (ros::Time::now() < end_time) {
          if (GetAStarGoal(current_position)) {
            new_goal_got = true;
            break;
          } else if (cur_goal_distance < co_->sbpl_max_distance
                      && IsGoalSafe(global_goal_, 0.10, 0.15)) {
            new_goal_got = true;
            planner_goal_ = global_goal_;
            taken_global_goal_ = true;
          }
          ROS_WARN("[FIX CONTROLLER] CLEARING state: getting new AStar Goal");
          last_valid_control_ = ros::Time::now();
          r.sleep();
        }
        // if get astar goal failed, try to get a temp goal
        if (!new_goal_got && GetAStarTempGoal()) {
          new_goal_got = true;
          ROS_INFO("[FIX CONTROLLER] CLEARING state: got temp AStar Goal success! Switch to A_PLANNING");
        }

        // find a new safe goal, use it to replan
        if (new_goal_got) {
          state_ = A_PLANNING;
          recovery_trigger_ = A_PLANNING_R;
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          if (taken_global_goal_) { 
            planning_state_ = P_INSERTING_NONE;
          } else {
            planning_state_ = P_INSERTING_BEGIN;
          }
          lock.unlock();
          ROS_INFO("[FIX CONTROLLER] CLEARING state: got AStar Goal success! Switch to A_PLANNING");
        } else {
          // TODO(lizhen) Alarm here, and try to get AStar goal again 
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
          ROS_ERROR("[FIX CONTROLLER] CLEARING state: got AStar Goal failed! Alarm and try again");
/*
          // no point is safe, so terminate the path
          ResetState();
          // disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          // TODO(chenkan): check if this is needed
          co_->fixpattern_local_planner->reset_planner();

          // we need to notify fixpattern_path
          co_->fixpattern_path->FinishPath();

          ROS_ERROR("[FIX CONTROLLER] GetAStarGoal failed, terminate path");
          // TODO(lizhen) Alarm here
          terminate_controller_ = true;
          return true;
*/
        }
      }

      break;

    default:
      ROS_ERROR("This case should never be reached, something is wrong, aborting");
      ResetState();
      // disable the planner thread
      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      runPlanner_ = false;
      lock.unlock();
      // Reached a case that should not be hit in autoscrubber. This is a bug, please report it.
      return true;
  }

  // we aren't done yet
  return false;
}

void AStarController::ResetState() {
  // Disable the planner thread
  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  runPlanner_ = false;
  lock.unlock();

  // Reset statemachine
  state_ = A_PLANNING;
  recovery_trigger_ = A_PLANNING_R;
  PublishZeroVelocity();
  front_path_.FinishPath();
  switch_path_ = false;
/*
  state_ = F_CONTROLLING;
  recovery_trigger_ = F_CONTROLLING_R;
  PublishZeroVelocity();
*/
//  ROS_INFO("[ASTAR CONTROLLER] ResetState");
  // search planner goal from start
  planner_goal_index_ = 0;
  cmd_vel_ratio_ = 1.0;
  astar_planner_timeout_cnt_ = 0;
  // reset some variables
  using_sbpl_directly_ = false;
  last_using_bezier_ = false;
  replan_directly_ = false;
  localization_valid_ = false;
  first_run_controller_flag_ = true;
}

bool AStarController::IsGlobalGoalReached(const geometry_msgs::PoseStamped& current_position, const geometry_msgs::PoseStamped& global_goal,
                                            double xy_goal_tolerance, double yaw_goal_tolerance) {
  double pose_diff = PoseStampedDistance(current_position, global_goal);
  double yaw_diff = angles::shortest_angular_distance(tf::getYaw(current_position.pose.orientation), tf::getYaw(global_goal.pose.orientation));
//  if ((co_->fixpattern_local_planner->isGoalXYLatched() || pose_diff < xy_goal_tolerance) && fabs(yaw_diff) < yaw_goal_tolerance) {
  if (pose_diff > 0.3 || fabs(yaw_diff) > M_PI / 6.0) {
    return false;
  } else {
    return true;
  }
}

bool AStarController::GetCurrentPosition(geometry_msgs::PoseStamped& current_position) {
  tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped cur_pos;
  if (!controller_costmap_ros_->getRobotPose(global_pose)) {
    ROS_WARN("Unable to get current_position");
    return false;
  } else {
    tf::poseStampedTFToMsg(global_pose, cur_pos);
    current_position = cur_pos;
    return true;
  }
}

unsigned int AStarController::GetPoseIndexOfPath(const std::vector<geometry_msgs::PoseStamped>& path, const geometry_msgs::PoseStamped& pose) {
  unsigned int index = 0x7FFFFFFF;
  for (unsigned int i = 0; i < static_cast<int>(path.size()); ++i) {
    if (PoseStampedDistance(path.at(i), pose) < 0.001) {
      index = i;
      break;
    }
  }
  return index;
}

bool AStarController::GetAStarGoal(const geometry_msgs::PoseStamped& cur_pose, int begin_index) {
  double start = GetTimeInSeconds();
  double cur_goal_dis = PoseStampedDistance(cur_pose, global_goal_);
 /* if (path.size() == 0) {
	  ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, path_size = %zu == 0", path.size());
    return false; 
  }
*/
//  if (planning_state_ == P_INSERTING_BEGIN) {
  if (true) {
    co_->fixpattern_path->Prune(fixpattern_path::GeometryPoseToPathPoint(cur_pose.pose), co_->max_offroad_dis, co_->max_offroad_yaw, false);
  }
  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  ROS_INFO("[ASTAR CONTROLLER] cur_goal_dis = %lf, path_size = %zu", cur_goal_dis, path.size());

  taken_global_goal_ = false;
  if(begin_index == 0 && (cur_goal_dis < 3.5 || // co_->sbpl_max_distance
     co_->fixpattern_path->Length() < co_->front_safe_check_dis ||
     path.size() <= 5)) {  
    ROS_INFO("[ASTAR CONTROLLER] taking global_goal_ as planner_goal_");
    if (IsGoalFootprintSafe(0.5, 0.0, global_goal_)) {
      planner_goal_ = global_goal_;
      taken_global_goal_ = true;
      planner_goal_index_ = (int)path.size() - 1;
      ROS_INFO("[ASTAR CONTROLLER] taking global_goal_ as planner_goal_");
      return true;
    } else {
      double acc_dis = 0.0;
      std::vector<geometry_msgs::PoseStamped>::iterator it;
      for (it = path.end() - 1; it >= path.begin() + 2; it -= 2) {
        if (IsGoalFootprintSafe(0.5 , 0.3, *it)) {
          planner_goal_ = *it;
          planner_goal_.header.frame_id = co_->global_frame;
          planner_goal_index_ = it - path.begin();
          ROS_INFO("[ASTAR CONTROLLER] taking global_goal_ as planner_goal_");
          return true;
        }
        acc_dis += PoseStampedDistance(*it, *(it - 2));
        if (acc_dis > cur_goal_dis) {
          ROS_WARN("[ASTAR CONTROLLER] Cur_goal_dis = %lf < 2.5m, but GetAStarGoal failed", cur_goal_dis);
          return false;
        }
      }
    }
  } else {
    bool cross_obstacle = false;
    double dis_accu = 0.0;
    int goal_index = -1;
    double goal_safe_dis_a, goal_safe_dis_b;
    int i, j;
    for (j = 0; j < 4; ++j) {
      cross_obstacle = false;
      dis_accu = 0.0;
      goal_index = -1;
      goal_safe_dis_a = co_->goal_safe_dis_a - j * 0.2;
      goal_safe_dis_b = co_->goal_safe_dis_b;
      ROS_INFO("[ASTAR CONTROLLER] get astar goal, round: %d", j);
      for (i = begin_index; i < path.size(); i += 2) {
        if (i > begin_index) dis_accu += PoseStampedDistance(path.at(i), path.at(i - 2));
        // we must enforce cross obstacle within front_safe_check_dis range
//        if (!cross_obstacle && dis_accu <= co_->front_safe_check_dis) continue;
        if (dis_accu <= goal_safe_dis_a) continue;
        if (PoseStampedDistance(cur_pose, path.at(i)) <= goal_safe_dis_a) continue;
//        ROS_INFO("[ASTAR CONTROLLER] dis_accu = %lf", dis_accu);
        double x = path[i].pose.position.x;
        double y = path[i].pose.position.y;
        double yaw = tf::getYaw(path[i].pose.orientation);
        if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0 ||
             !IsGoalFootprintSafe(goal_safe_dis_a, goal_safe_dis_b, path[i])) {
           cross_obstacle = true;
//           ROS_INFO("[ASTAR CONTROLLER] path[%d] not safe", i);
           continue;
        }
        goal_index = i;
        break;
      }
      if (goal_index != -1 || (!cross_obstacle && i >= path.size())) {
        if (i >= path.size()) goal_index = path.size() - 1;
        break;
      } 
    }
    if (goal_index == -1 || goal_index >= path.size()) {
      ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, cost: %lf secs", GetTimeInSeconds() - start);
      return false;
    }
    planner_goal_ = path[goal_index];
    planner_goal_.header.frame_id = co_->global_frame;
    planner_goal_index_ = goal_index;
  }
  astar_goal_pub_.publish(planner_goal_);
  ROS_INFO("[ASTAR CONTROLLER] GetAStarGoal cost: %lf secs", GetTimeInSeconds() - start);
  ROS_INFO("[ASTAR CONTROLLER] planner_goal_index_: %d", planner_goal_index_);
  return true;
}

bool AStarController::GetAStarTempGoal() {
  ROS_INFO("[ASTAR CONTROLLER] GetAStarTempGoal!");
  bool cross_obstacle = false;
  double dis_accu = 0.0;
  int goal_index = -1;
  double goal_safe_dis_a = 0.4;
  double goal_safe_dis_b = 0.3;
  int i;
  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  for (i = 0; i < path.size(); i += 1) {
    if (i > 0) dis_accu += PoseStampedDistance(path.at(i), path.at(i - 1));
    // we must enforce cross obstacle within front_safe_check_dis range
    if (dis_accu <= 1.0) continue;
    double x = path[i].pose.position.x;
    double y = path[i].pose.position.y;
    double yaw = tf::getYaw(path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0 ||
         !IsGoalFootprintSafe(goal_safe_dis_a, goal_safe_dis_b, path[i])) {
       cross_obstacle = true;
       continue;
    }
    goal_index = i;
    break;
  }
  if (!cross_obstacle && i >= path.size()) {
    goal_index = path.size() - 1;
  } 

  if (goal_index == -1 || goal_index >= path.size()) {
    ROS_WARN("[ASTAR CONTROLLER] GetAStarTempGoal failed");
    return false;
  }
  planner_goal_ = path[goal_index];
  planner_goal_.header.frame_id = co_->global_frame;
  planner_goal_index_ = goal_index;
  ROS_INFO("[ASTAR CONTROLLER] temp planner_goal_index_: %d", planner_goal_index_);
  return true;
}

void AStarController::PublishPlan(const ros::Publisher& pub, const std::vector<geometry_msgs::PoseStamped>& plan) {
  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(plan.size());

  if (!plan.empty()) {
    gui_path.header.frame_id = plan[0].header.frame_id;
    gui_path.header.stamp = plan[0].header.stamp;
  }

  for (unsigned int i = 0; i < plan.size(); i++) {
    gui_path.poses[i] = plan[i];
  }

  // publish
  pub.publish(gui_path);
}

void AStarController::PublishAlarm(unsigned char alarm_index) {
  // create a message for the plan
  std_msgs::UInt8 alarm_msg;
  alarm_msg.data = alarm_index;
  // publish
  alarm_pub_.publish(alarm_msg);
}

void AStarController::PublishGoalReached() {
  // publish
  goal_reached_pub_.publish(global_goal_);
}

bool AStarController::GetAStarInitalPath(const geometry_msgs::PoseStamped& global_start, const geometry_msgs::PoseStamped& global_goal) {
  if (!co_->astar_global_planner->makePlan(global_start, global_goal, *planner_plan_) || planner_plan_->empty()) {
    ROS_ERROR("[ASTAR CONTROLLER] astar failed to find a plan to point (%.2f, %.2f)", global_goal.pose.position.x, global_goal.pose.position.y);
    return false;
  } else {
     ROS_INFO("[ASTAR CONTROLLER] get initial path_size = %d", (int)planner_plan_->size());
     std::vector<fixpattern_path::PathPoint> fix_path;
     for (int i = 0; i < planner_plan_->size(); i += 5) {
       fix_path.push_back(fixpattern_path::GeometryPoseToPathPoint(planner_plan_->at(i).pose));
     }
     fix_path.push_back(fixpattern_path::GeometryPoseToPathPoint(planner_plan_->back().pose));
//     path_recorder::PathRecorder recorder;
//     recorder.CalculateCurvePath(&fix_path);
     co_->fixpattern_path->set_fix_path(global_start, fix_path, true); 

     std::vector<geometry_msgs::PoseStamped> plan = co_->fixpattern_path->GeometryPath();
     for (auto&& p : plan) {  // NOLINT
       p.header.frame_id = co_->global_frame;
       p.header.stamp = ros::Time::now();
     }
     PublishPlan(fixpattern_pub_, plan);

     ROS_INFO("[ASTAR CONTROLLER] After set_fix_path size = %d", (int)plan.size());
     return true;
  }
}

bool AStarController::HandleSwitchingPath(geometry_msgs::PoseStamped current_position) {
  if (switch_path_) {
    if (PoseStampedDistance(planner_start_, current_position) > 1.5 || PoseStampedDistance(front_goal_, current_position) < 1.0) {
      switch_path_ = false;
    } else {
      fixpattern_path::PathPoint start_pose = fixpattern_path::GeometryPoseToPathPoint(current_position.pose); 
      front_path_.Prune(start_pose, 0.8, M_PI / 2.0, false);
      if (front_path_.total_point_count() > 3 && 
          CheckFixPathFrontSafe(front_path_.GeometryPath(), co_->front_safe_check_dis) > 2.0) {
        double dis_diff, yaw_diff;
        bool detect_corner;
        if (co_->fixpattern_path->path().front().corner_struct.corner_point) {
          dis_diff = 0.10;
          yaw_diff = M_PI_4;
          detect_corner = true;
          if (front_path_.Prune(start_pose, dis_diff, yaw_diff, false) && 
               front_path_.Length() - co_->fixpattern_path->Length() < 0.0) {
            co_->fixpattern_path->set_fix_path(current_position, front_path_.path(), false, detect_corner); 
            first_run_controller_flag_ = true;
            ROS_INFO("[ASTAR CONTROLLER] switch origin path as fix path");
          }
        } else {
          double start_dis_diff = front_path_.path().front().DistanceToPoint(start_pose);
          if (start_dis_diff > 0.01) {
            dis_diff = 0.5;
            yaw_diff = M_PI / 2.0;
//            start_pose = co_->fixpattern_path->path().front(); 
            detect_corner = false;
            bool get_bezier_plan = false;
            std::vector<fixpattern_path::PathPoint> bezier_path;
            if (front_goal_index_ > 0 && front_goal_index_ < front_path_.GeometryPath().size()) {
              geometry_msgs::PoseStamped goal = front_path_.GeometryPath().at(front_goal_index_);
              if (MakeBezierPlan(&bezier_path, current_position, goal, false)) {
                astar_path_.set_bezier_path(current_position, bezier_path, false);
                front_path_.insert_begin_path(astar_path_.path(), current_position, goal, false, M_PI / 3.0);
                get_bezier_plan = true;
              }
            }
            if ((!get_bezier_plan && front_path_.Prune(start_pose, dis_diff, yaw_diff, false) || get_bezier_plan) && 
                 front_path_.Length() - co_->fixpattern_path->Length() < 0.0) {
              co_->fixpattern_path->set_fix_path(current_position, front_path_.path(), false, detect_corner); 
              first_run_controller_flag_ = true;
              ROS_INFO("[ASTAR CONTROLLER] switch origin path as fix path");
            }
          }
        }
   
        switch_path_ = false;
      }
    }
  }
  return true;
}

bool AStarController::HandleLocalizationRecovery() {
  if (!localization_valid_) {
    // TODO (lizhen) Alarm Hera!
    ROS_WARN("[ASTAR CONTROLLER] localization failed! Recovery now by inplace_rotating");
    autoscrubber_services::StartRotate start_rotate;
    autoscrubber_services::StopRotate stop_rotate;
    autoscrubber_services::CheckRotate check_rotate;
    start_rotate.request.rotateAngle.data = 360;
    start_rotate_client_.call(start_rotate);
    do {
      check_rotate_client_.call(check_rotate);
      usleep(1000);
    } while (!check_rotate.response.isFinished.data && !localization_valid_);

    stop_rotate_client_.call(stop_rotate);
    PublishZeroVelocity();
    usleep(500000);
  }
  return localization_valid_;
}


bool AStarController::HandleGoingBack(const geometry_msgs::PoseStamped& current_position, double backward_dis) {
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::PoseStamped cur_pos = current_position;
  if (backward_dis == 0.0) {
    backward_dis = co_->backward_check_dis;
  }
  // check if need backward
  ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration / 3);
  bool need_backward = true;
  ros::Rate r(10);
  while (ros::Time::now() < end_time) {
    if (!NeedBackward(cur_pos, backward_dis)) {
      need_backward = false;
      break;
    }
    ROS_INFO("[ASTAR CONTROLLER] Need Backward, Publish Zero Vel");
    // stop first, and set last_valid_control_
    PublishZeroVelocity();
    last_valid_control_ = ros::Time::now();
    r.sleep();
  }
  ros::Rate control_rate(co_->controller_frequency);
  tf::Stamped<tf::Pose> global_pose;
//  while (need_backward && NeedBackward(cur_pos, 0.15) && CanBackward(0.25)) {
  while (need_backward && NeedBackward(cur_pos, backward_dis + 0.05) && CanBackward(backward_dis + 0.15)) {
    ROS_INFO("[ASTAR CONTROLLER] going back");
    // get curent position
    controller_costmap_ros_->getRobotPose(global_pose);
    tf::poseStampedTFToMsg(global_pose, cur_pos);

    // make sure that we send the velocity command to the base
    cmd_vel.linear.x = -0.1;
    cmd_vel.angular.z = 0.0;
    co_->vel_pub->publish(cmd_vel);

    last_valid_control_ = ros::Time::now();
    control_rate.sleep();
  }
  return need_backward;
}

bool AStarController::HandleRecovery(geometry_msgs::PoseStamped current_pos) {
  ROS_INFO("[FIXPATTERN CONTROLLER] Handle Recovery!");
  bool ret = false;
  geometry_msgs::PoseStamped goal_pos;
  double target_yaw = footprint_checker_->RecoveryCircleCost(current_pos, footprint_spec_, &goal_pos);
  if(target_yaw < M_PI * 2.0) {
    double target_dis = PoseStampedDistance(current_pos, goal_pos);
    if (RotateToYaw(target_yaw)) {
      ROS_INFO("rotate to yaw done, next going forward dis = %lf", target_dis);
//      if (CanForward(target_dis)) {
      if (GoingForward(target_dis / 2.5)) {
        ROS_INFO("GoingForward done");
        ret = true;
      }
		}
  }
  return ret;
}

bool AStarController::CanRotate(double x, double y, double yaw, int dir) {
  // only check 0.4 radian, ignore current footprint
  for (int i = 1; i <= 4; ++i) {
    if (footprint_checker_->CircleCenterCost(x, y, yaw + dir * 0.1 * i,
        co_->circle_center_points) < 0.0) {
      ROS_INFO("[ASTAR CONTROLLER] CanRotate: false, yaw: %lf, dir: %d", yaw, dir);
      return false;
    }
  }
  ROS_INFO("[ASTAR CONTROLLER] CanRotate: true");
  return true;
}

bool AStarController::RotateToYaw(double target_yaw) {
  tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped current_position;

  controller_costmap_ros_->getRobotPose(global_pose);
  tf::poseStampedTFToMsg(global_pose, current_position);
  double x = current_position.pose.position.x;
  double y = current_position.pose.position.y;
  double yaw = tf::getYaw(current_position.pose.orientation);
  double angle_diff = angles::shortest_angular_distance(yaw, target_yaw);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;

  ros::Rate r(co_->controller_frequency);
//  while (fabs(angle_diff) > 0.1 && CanRotate(x, y, yaw, angle_diff > 0 ? 1 : -1)) {
  while (fabs(angle_diff) > 0.1) {
    ROS_INFO("rotate to yaw: cur_yaw = %lf, target_yaw = %lf, yaw_diff = %lf",yaw ,target_yaw, angle_diff);
    cmd_vel.angular.z = angle_diff > 0 ? 0.3 : -0.3;
    co_->vel_pub->publish(cmd_vel);
    last_valid_control_ = ros::Time::now();

    controller_costmap_ros_->getRobotPose(global_pose);
    tf::poseStampedTFToMsg(global_pose, current_position);
    x = current_position.pose.position.x;
    y = current_position.pose.position.y;
    yaw = tf::getYaw(current_position.pose.orientation);
    angle_diff = angles::shortest_angular_distance(yaw, target_yaw);

    r.sleep();
  }
  return true;
}

bool AStarController::CanBackward(double distance) {
  tf::Stamped<tf::Pose> global_pose;
  controller_costmap_ros_->getRobotPose(global_pose);
  geometry_msgs::PoseStamped current_position;
  tf::poseStampedTFToMsg(global_pose, current_position);

  double x = current_position.pose.position.x;
  double y = current_position.pose.position.y;
  double yaw = tf::getYaw(current_position.pose.orientation);
  double resolution = controller_costmap_ros_->getCostmap()->getResolution();
  int num_step = distance / resolution;

  std::vector<geometry_msgs::PoseStamped> path;
  // ignore current footprint
  for (int i = 1; i <= num_step; ++i) {
    double new_x = x - i * resolution * cos(yaw);
    double new_y = y - i * resolution * sin(yaw);
    if (footprint_checker_->CircleCenterCost(new_x, new_y, yaw,
                                             co_->circle_center_points) < 0) {
      ROS_WARN("[ASTAR CONTROLLER] CanBackward: false");
      return false;
    }
  }
  ROS_INFO("[ASTAR CONTROLLER] CanBackward: true");
  return true;
}

bool AStarController::GoingBackward(double distance) {
  if (!CanBackward(0.35)) return false;

  double backward_time = distance / 0.1;
  ros::Time end_time = ros::Time::now() + ros::Duration(backward_time);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = -0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  ros::Rate r(co_->controller_frequency);
  while (ros::Time::now() < end_time && CanBackward(0.35)) {
    co_->vel_pub->publish(cmd_vel);
    last_valid_control_ = ros::Time::now();

    r.sleep();
  }
  return true;
}

bool AStarController::CanForward(double distance) {
  tf::Stamped<tf::Pose> global_pose;
  controller_costmap_ros_->getRobotPose(global_pose);
  geometry_msgs::PoseStamped current_position;
  tf::poseStampedTFToMsg(global_pose, current_position);

  double x = current_position.pose.position.x;
  double y = current_position.pose.position.y;
  double yaw = tf::getYaw(current_position.pose.orientation);
  double resolution = controller_costmap_ros_->getCostmap()->getResolution();
  int num_step = distance / resolution;

  std::vector<geometry_msgs::PoseStamped> path;
  // ignore current footprint
  for (int i = 0; i <= num_step; ++i) {
    double new_x = x + i * resolution * cos(yaw);
    double new_y = y + i * resolution * sin(yaw);
    if (footprint_checker_->CircleCenterCost(new_x, new_y, yaw,
                                             co_->circle_center_points) < 0) {
      ROS_INFO("[ASTAR CONTROLLER] CanForward: false");
      return false;
    }
  }
  ROS_INFO("[ASTAR CONTROLLER] CanForward: true");
  return true;
}

bool AStarController::GoingForward(double distance) {
  if (!CanForward(0.15)) return false;

  double forward_time = distance / 0.1;
  ros::Time end_time = ros::Time::now() + ros::Duration(forward_time);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  ros::Rate r(co_->controller_frequency);
  while (ros::Time::now() < end_time && CanForward(0.15)) {
 // while (ros::Time::now() < end_time) {
    co_->vel_pub->publish(cmd_vel);
    last_valid_control_ = ros::Time::now();

    r.sleep();
  }
  return true;
}

bool AStarController::RotateRecovery() {
  // rotate to previous direction
  tf::Stamped<tf::Pose> global_pose;
  controller_costmap_ros_->getRobotPose(global_pose);
  geometry_msgs::PoseStamped current_position;
  tf::poseStampedTFToMsg(global_pose, current_position);

  double yaw = tf::getYaw(current_position.pose.orientation);
  if (rotate_recovery_dir_ == 0) rotate_recovery_dir_ = 1;
  double target_yaw = angles::normalize_angle(yaw + rotate_recovery_dir_ * M_PI / 6);
  double theta_sim_granularity = rotate_recovery_dir_ > 0 ? 0.1 : -0.1;

  int num_step = M_PI / 6 / fabs(theta_sim_granularity);
  if (num_step == 0) num_step = 1;

  bool footprint_safe = true;
  // ignore current footprint
  for (int i = 1; i <= num_step; ++i) {
    double sample_yaw = angles::normalize_angle(yaw + i * theta_sim_granularity);
    if (footprint_checker_->CircleCenterCost(current_position.pose.position.x, current_position.pose.position.y,
                                             sample_yaw, co_->circle_center_points) < 0) {
      footprint_safe = false;
      break;
    }
  }
  if (footprint_safe) {
    ROS_INFO("[ASTAR CONTROLLER] RotateToYaw, yaw: %lf, target_yaw: %lf", yaw, target_yaw);
    RotateToYaw(target_yaw);
    return true;
  }

  rotate_failure_times_++;
  // can rotate only if at least one direction is safe
  if (rotate_failure_times_ < 2) {
    // if cannot rotate to previous direction, change direction
    rotate_recovery_dir_ *= -1;
    target_yaw = angles::normalize_angle(yaw + rotate_recovery_dir_ * M_PI / 6);
    theta_sim_granularity = rotate_recovery_dir_ > 0 ? 0.1 : -0.1;

    footprint_safe = true;
    // ignore current footprint
    for (int i = 1; i <= num_step; ++i) {
      double sample_yaw = angles::normalize_angle(yaw + i * theta_sim_granularity);
      if (footprint_checker_->CircleCenterCost(current_position.pose.position.x, current_position.pose.position.y,
                                               sample_yaw, co_->circle_center_points) < 0) {
        footprint_safe = false;
        break;
      }
    }
    if (footprint_safe) {
      ROS_INFO("[ASTAR CONTROLLER] RotateToYaw, yaw: %lf, target_yaw: %lf", yaw, target_yaw);
      RotateToYaw(target_yaw);
      return true;
    }

    rotate_failure_times_++;
  }

  // we should reset recovery_dir_ and failure_times_ here
  rotate_recovery_dir_ = 0;
  rotate_failure_times_ = 0;

  // backward...
  if (GoingBackward(0.25)) {
    return true;
  }

  // go forward, if can
  if (GoingForward(0.25)) {
    return true;
  }

  // at last... have a try to going back?
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = -0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  co_->vel_pub->publish(cmd_vel);
}

};  // namespace autoscrubber
