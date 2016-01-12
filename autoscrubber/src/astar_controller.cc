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
#include "path_recorder/path_recorder.h"
#include <nav_msgs/Path.h>
#include <angles/angles.h>

namespace autoscrubber {

AStarController::AStarController(tf::TransformListener* tf,
                                 costmap_2d::Costmap2DROS* planner_costmap_ros,
                                 costmap_2d::Costmap2DROS* controller_costmap_ros)
    : tf_(*tf),
      planner_costmap_ros_(planner_costmap_ros), controller_costmap_ros_(controller_costmap_ros),
      planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL), sbpl_reached_goal_(false),
      planner_goal_index_(0), runPlanner_(false), new_global_plan_(false), first_run_controller_flag_(true),
      using_sbpl_directly_(false), sbpl_broader_(false), 
      local_planner_error_cnt_(0), goal_not_safe_cnt_(0), path_not_safe_cnt_(0){
  // set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  // create footprint_checker_
  footprint_checker_ = new autoscrubber::FootprintChecker(*planner_costmap_ros_->getCostmap());

  footprint_spec_ = planner_costmap_ros_->getRobotFootprint();
  // set up the planner's thread
  planner_thread_ = new boost::thread(boost::bind(&AStarController::PlanThread, this));

  // initially, we'll need to make a plan
  state_ = A_PLANNING;
  
  // disable switch_controller_ when start	
  switch_controller_ = false;
  // we'll start executing recovery behaviors at the beginning of our list
  recovery_trigger_ = A_PLANNING_R;

  // set rotate_recovery_dir_
  rotate_recovery_dir_ = 0;
  rotate_failure_times_ = 0;
  cmd_vel_ratio_ = 1.0;
  // set for fixpattern_path

  ros::NodeHandle fixpattern_nh("~/fixpattern_global_planner");
  fixpattern_pub_ = fixpattern_nh.advertise<nav_msgs::Path>("plan", 1);
}

AStarController::~AStarController() {
  planner_thread_->interrupt();
  planner_thread_->join();

  delete footprint_checker_;

  delete planner_thread_;

  delete planner_plan_;
  delete latest_plan_;
  delete controller_plan_;
}

bool AStarController::MakePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>* plan) {
  //boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

  // make sure to set the plan to be empty initially
  plan->clear();

  // since this gets called on handle activate
  if (planner_costmap_ros_ == NULL) {
    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
    return false;
  }
  if (PoseStampedDistance(start, goal) <= 0.25) {
    // set this to true as we'll use it afterwards
    using_sbpl_directly_ = true;

    ROS_INFO("[ASTAR CONTROLLER] too short, take start and goal as plan directly");
    // too short, plan direct path
    plan->clear();
    plan->push_back(start);
    plan->push_back(goal);

    // assign to astar_path_
    std::vector<fixpattern_path::PathPoint> path;
    for (const auto& p : *plan) {
      path.push_back(fixpattern_path::GeometryPoseToPathPoint(p.pose));
    }
    if (state_ == A_PLANNING) {
//      astar_path_.set_path(path, false);
      astar_path_.set_short_sbpl_path(start, path);
//      co_->fixpattern_path->set_short_sbpl_path(start, path);
    } else {
      fixpattern_path::Path temp_path;
      temp_path.set_path(path);
      astar_path_.ExtendPath(temp_path.path());
    }
  } else if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
    // too short, use sbpl directly
    ROS_INFO("[ASTAR CONTROLLER] use sbpl directly");
    using_sbpl_directly_ = true;
    // if last plan was success due to sbpl_broader_ and goal has not changed,
    // we'll still use sbpl_broader_
    bool sbpl_broader = sbpl_broader_;
    if (PoseStampedDistance(goal, success_broader_goal_) < GS_DOUBLE_PRECISION)
      sbpl_broader = true;
    // if the planner fails or returns a zero length plan, planning failed
    if (!co_->sbpl_global_planner->makePlan(start, goal, *plan, astar_path_, sbpl_broader, state_ != A_PLANNING) || plan->empty()) {
      ROS_ERROR("[ASTAR CONTROLLER] sbpl failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
  } else {
    // astar first, and then sbpl to a median point
    ROS_INFO("[ASTAR CONTROLLER] astar first, and then sbpl to a median point");
    using_sbpl_directly_ = false;
    if (!co_->astar_global_planner->makePlan(start, goal, *plan) || plan->empty()) {
      ROS_ERROR("[ASTAR CONTROLLER] astar failed to find a plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }
    // assign to astar_path_
    std::vector<fixpattern_path::PathPoint> path;
    for (const auto& p : *plan) {
      path.push_back(fixpattern_path::GeometryPoseToPathPoint(p.pose));
    }
    co_->fixpattern_path->set_fix_path(start, path); 
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
      ROS_ERROR("[ASTAR CONTROLLER] sbpl failed to find a plan to point (%.2f, %.2f)", temp_goal.pose.position.x, temp_goal.pose.position.y);
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
  ROS_INFO("[ASTAR CONTROLLER] Starting planner thread...");
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
    }
    ros::Time start_time = ros::Time::now();

    // time to plan! get a copy of the goal and unlock the mutex
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

    // get the starting pose of the robot
    geometry_msgs::PoseStamped start;
    start.header.frame_id = planner_costmap_ros_->getGlobalFrameID();
    tf::Stamped<tf::Pose> global_pose;
    bool gotStartPose = true;
    if (state_ == A_PLANNING) {
      if (!planner_costmap_ros_->getRobotPose(global_pose)) {
        gotStartPose = false;
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      }
      tf::poseStampedTFToMsg(global_pose, start);
    } else if (state_ == A_CONTROLLING) {
      // if MakePlan when not in A_PLANNING, we need to make plan based on last plan
      std::vector<geometry_msgs::PoseStamped> path = astar_path_.GeometryPath();
      // path shouldn't be empty
      if (path.size() == 0) 
         gotStartPose = false;
      else {
        start = path.back();
      }
    } else if (state_ == FIX_CONTROLLING) {
      if (!GetAStarStart(co_->front_safe_check_dis)) {
        if (planning_state_ == P_INSERTING_MIDDLE) {
          planning_state_ = P_INSERTING_BEGIN;
        }
      }
      start = planner_start_;
    }

    // run planner
    planner_plan_->clear();
    bool gotPlan = false;
    if(gotStartPose) 
        gotPlan = n.ok() && MakePlan(start, temp_goal, planner_plan_) && !astar_path_.path().empty();

    if (gotPlan) {
      ROS_INFO("[ASTAR CONTROLLER] Got Plan with %zu points! cost: %lf secs", planner_plan_->size(), GetTimeInSeconds() - start_t);
      planner_costmap_ros_->getRobotPose(global_pose);
      tf::poseStampedTFToMsg(global_pose, start);
      double distance_diff = PoseStampedDistance(start, astar_path_.GeometryPath().front());
      if (distance_diff > 0.3 && state_ == A_PLANNING) {
        planning_state_ = P_INSERTING_BEGIN;
        ROS_WARN("[ASTAR CONTROLLER] Distance from start to path_front = %lf > 0.1m, continue", distance_diff);
      } else {
        last_valid_plan_ = ros::Time::now();
        new_global_plan_ = true;
        if(using_sbpl_directly_) {  // get sbpl_plan, set or insert path
          if (taken_global_goal_ || planning_state_ == P_INSERTING_NONE) {
            taken_global_goal_ = false;
            co_->fixpattern_path->set_sbpl_path(astar_path_.path());
            state_ = FIX_CONTROLLING;
            runPlanner_ = false;
            first_run_controller_flag_ = true;
          } else if (planning_state_ == P_INSERTING_BEGIN) {
            co_->fixpattern_path->insert_begin_path(astar_path_.path(), false, temp_goal);
            state_ = FIX_CONTROLLING;
            runPlanner_ = false;
            first_run_controller_flag_ = true;
          } else if (planning_state_ == P_INSERTING_END) {
            co_->fixpattern_path->insert_end_path(astar_path_.path());
            state_ = FIX_CONTROLLING;
            runPlanner_ = false;
            first_run_controller_flag_ = true;
          } else if (planning_state_ == P_INSERTING_MIDDLE) {
            co_->fixpattern_path->insert_middle_path(astar_path_.path(), planner_start_, temp_goal);
            state_ = FIX_CONTROLLING;
            runPlanner_ = false;
            front_safe_check_cnt_ = 0; // only set 0 after getting new fix_path
            // first_run_controller_flag_ = true;
          } else { // unkonw state
            // switch to FIX_CLEARING state
            state_ = FIX_CLEARING;
            recovery_trigger_ = FIX_RECOVERY_R;
            planning_state_ = P_INSERTING_BEGIN;
            ROS_ERROR("[ASTAR CONTROLLER] planning_state_ unknown, enter recovery");
          }
        } else { // get astar plan, switch to Controlling, because we have set it as fix_path in MakePlan function
          state_ = FIX_CONTROLLING;
          runPlanner_ = false;
          first_run_controller_flag_ = true;
        }
        // pointer swap the plans under mutex (the controller will pull from latest_plan_)
        lock.lock();

        // reset rotate_recovery_dir_
        rotate_recovery_dir_ = 0;
        // reset sbpl_broader_
        ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

        if (co_->planner_frequency <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
    } else if (state_ == A_PLANNING) {  // if we didn't get a plan and we are in the planning state (the robot isn't moving)
      ROS_ERROR("[ASTAR CONTROLLER] No Plan...");
      // ros::Time attempt_end = last_valid_plan_ + ros::Duration(co_->planner_patience);
      ros::Time attempt_end = ros::Time::now() + ros::Duration(co_->planner_patience);
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
        ROS_ERROR("[ASTAR CONTROLLER] Not got plan until planner_patience, enter recovery");
      } else if (runPlanner_) {
        // to update global costmap
        usleep(500000);
//        GetAStarGoal(start);
      }
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
  }
}

bool AStarController::Control(BaseControlOption* option, ControlEnvironment* environment, bool first_run_flag) {
  ROS_INFO("[ASTAR_CTRL] Switch to Astar Controller!");
  co_ = reinterpret_cast<AStarControlOption*>(option);
  env_ = environment;
  global_goal_.pose = co_->global_planner_goal->pose; 
  global_goal_.header.frame_id = co_->global_frame;
	
  taken_global_goal_ = false;
  bool start_path_got = false;
  double cur_goal_distance;
  unsigned int path_status; 
	
  geometry_msgs::PoseStamped current_position;
  tf::Stamped<tf::Pose> global_pose;
  if (!planner_costmap_ros_->getRobotPose(global_pose)) {
    ROS_WARN("Unable to get starting pose of robot, unable to create sbpl plan");
    switch_controller_ = false;
    return true;
  } else {
    tf::poseStampedTFToMsg(global_pose, current_position);
    cur_goal_distance = PoseStampedDistance(current_position, global_goal_); 
  }
/*  
  while(1) {
    NeedBackward(current_position, 0.05);
    usleep(500000);
    NeedBackward(current_position, 0.10);
    usleep(500000);
    NeedBackward(current_position, 0.15);
    usleep(500000);
    NeedBackward(current_position, 0.20);
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
       switch_controller_ = false;
	     return true;
     }
  }
  // first run: just get sbpl Path, and insert to fixpattern_path 
  if (first_run_flag) {
    if (cur_goal_distance < co_->sbpl_max_distance) {
      ROS_INFO("[ASTAR CONTROLLER] distance from current pose to goal = %lf, take global goal as planner_goal", cur_goal_distance);
      planner_goal_ = global_goal_;
      state_ = A_PLANNING;
//      planning_state_ = P_INSERTING_NONE;
      taken_global_goal_ = true;
    } else {
      ROS_INFO("[ASTAR_CONTROLLER] get Astar Path, and set as fixpattern_path");
//      start_path_got = GetInitalPath(current_position, global_goal_);
      start_path_got = GetAStarInitalPath(current_position, global_goal_);
      if (start_path_got) {
        if (CheckFixPathFrontSafe(co_->front_safe_check_dis) < co_->front_safe_check_dis) {
          if (GetAStarGoal(current_position, obstacle_index_)) {
            state_ = A_PLANNING;
            planning_state_ = P_INSERTING_BEGIN;
          }
        } else {
          state_ = FIX_CONTROLLING;
        }
	   	} else { 
        planner_goal_ = global_goal_;
        state_ = A_PLANNING;
        planning_state_ = P_INSERTING_NONE;
        taken_global_goal_ = true;
/*
        if (GetAStarGoal(current_position)) {
          state_ = A_PLANNING;
          planning_state_ = P_INSERTING_BEGIN;
        } else {
          ROS_ERROR("[ASTAR CONTROLLER] cannot get astar goal from start point to fix_path, terminate");
          // we need to notify fixpattern_path
          co_->fixpattern_path->FinishPath();
          // TODO(lizhen): alert
          return true;
        }
*/
      }
    }
  }

  co_->fixpattern_local_planner->reset_planner();

  // initialize some flag
  first_run_controller_flag_ = true;
  using_sbpl_directly_ = false;

  // disable the planner
  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  runPlanner_ = false;
//  planner_cond_.notify_one();
  lock.unlock();
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
      if (switch_controller_) {
        switch_controller_ = false;
        return false;
      } else {
        return true;
      }
    }
    // check if execution of the goal has completed in some way

    ros::WallDuration t_diff = ros::WallTime::now() - start;
    ROS_DEBUG_NAMED("autoscrubber", "Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / co_->controller_frequency) && state_ == A_CONTROLLING)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", co_->controller_frequency, r.cycleTime().toSec());
  } // while (n.ok())

  // wake up the planner thread so that it can exit cleanly
  lock.lock();
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
  double resolution = planner_costmap_ros_->getCostmap()->getResolution();
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

double AStarController::CheckFixPathFrontSafe(double front_safe_check_dis) {
  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  double accu_dis = 0.0;
  double off_obstacle_dis = 0.0;
  bool cross_obstacle = false;
  int i, j;
  for (i = 0; i < path.size(); i += 5) {
    double yaw = tf::getYaw(path[i].pose.orientation);
    if (footprint_checker_->CircleCenterCost(path[i].pose.position.x, path[i].pose.position.y,
                                             yaw, co_->circle_center_points) < 0) {
      cross_obstacle = true;
      obstacle_index_ = i;
      break;
    }
    if (i != 0) accu_dis += PoseStampedDistance(path[i], path[i - 5]);
    if (accu_dis >= front_safe_check_dis) break;
  }
  if (!cross_obstacle && i >= path.size())
    accu_dis = front_safe_check_dis + 0.001;

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
      break;
    }
    if (i != 0) accu_dis += PoseStampedDistance(path[i], path[i - 5]);
    if (accu_dis >= front_safe_check_dis) break;
  }
  if (cross_obstacle) {
    if (accu_dis > 0.8) {
      for (j = obstacle_index_; j > 5; j -= 5) {
        off_obstacle_dis += PoseStampedDistance(path[i], path[i - 5]);
        if (off_obstacle_dis > 0.7) {
		      planner_start_ = path.at(j);
          start_got = true;
          break;
        }
      }
    } else if (accu_dis > 0.5) {
      if (path.size() > 20)
		    planner_start_ = path.at(10);
      else 
		    planner_start_ = path.front();
    }
  }
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
  double resolution = planner_costmap_ros_->getCostmap()->getResolution();
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
  geometry_msgs::Twist cmd_vel;

  // get curent position
  tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped current_position;
  if (!planner_costmap_ros_->getRobotPose(global_pose)) {
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

    // do a pointer swap under mutex, but it seems no use
    std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    controller_plan_ = latest_plan_;
    latest_plan_ = temp_plan;
    lock.unlock();
    ROS_DEBUG_NAMED("autoscrubber", "pointers swapped!");
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

      // if we're controlling, we'll attempt to find valid velocity commands
    case A_CONTROLLING:
      ROS_INFO("[ASTAR CONTROLLER] in Astar CONTROLLING state");
      // check if the global goal footprint is safe, if not, stop and wait until timeout
      {      
        if (cur_goal_distance < co_->goal_safe_check_dis
             && !IsGoalFootprintSafe(0.5, 0.0, global_goal_)) {
					bool is_goal_safe = false;
					ros::Rate check_rate(10);
					ros::Time check_end_time = ros::Time::now() + ros::Duration(co_->goal_safe_check_duration);
					while (ros::Time::now() < check_end_time) {
						if (IsGoalFootprintSafe(0.5, 0.0, global_goal_)) {
							is_goal_safe = true;
							break;
						}
					  PublishZeroVelocity();
						check_rate.sleep();
					}
					if (!is_goal_safe){      
						ROS_ERROR("[ASTAR CONTROLLER] Check global goal not safe, stop here!");

						// disable the planner thread
						boost::unique_lock<boost::mutex> lock(planner_mutex_);
						runPlanner_ = false;
						lock.unlock();

						ResetState();
            co_->fixpattern_path->fixpattern_path_reached_goal_ = false;
            switch_controller_ = false;

						// TODO(chenkan): check if this is needed
						co_->fixpattern_local_planner->reset_planner();
						// Goal not reached, but we will stop and exit ExecuteCycle
						return true;
					}
        }
      }

      // check to see if we've reached our goal
      if (co_->fixpattern_local_planner->isGoalReached()) {
        ROS_WARN("[ASTAR CONTROLLER] Goal reached!");
        co_->fixpattern_path->EraseToPoint(fixpattern_path::GeometryPoseToPathPoint(planner_goal_.pose));
        ResetState();

        // disable the planner thread
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // TODO(chenkan): check if this is needed
        co_->fixpattern_local_planner->reset_planner();

        if(co_->fixpattern_path->fixpattern_path_reached_goal_
            && cur_goal_distance < co_->fixpattern_local_planner->xy_goal_tolerance_
            || taken_global_goal_) {
/*
        if (taken_global_goal_ || !IsGlobalGoalReached(current_position, global_goal_, 
                                 co_->fixpattern_local_planner->xy_goal_tolerance_, co_->fixpattern_local_planner->yaw_goal_tolerance_)) {
*/
        // Sbpl Goal reached, and fix_pattern Goal reached, all path done 
				  ResetState();
          co_->fixpattern_path->fixpattern_path_reached_goal_ = false;
          ROS_WARN("[ASTAR CONTROLLER] Final Goal reached!");
          switch_controller_ = false;
        } else {
          switch_controller_ = true;
        }
        // Goal reached, switch to fixpattern controller
        return true;
      } else {
        switch_controller_ = true;
      }

      {
        bool need_backward = HandleGoingBack(current_position);
        // check if need going back
        if(!IsGoalFootprintSafe(co_->goal_safe_dis_a, co_->goal_safe_dis_b, planner_goal_)) {
          ++goal_not_safe_cnt_;
          ROS_WARN("[ASTAR CONTROLLER] IsGoalFootprintSafe count = %d", goal_not_safe_cnt_);
        } else {
          goal_not_safe_cnt_ = 0;
        }
        // check if goal footprint hits something alongside the road, after and before it 
        if (need_backward || goal_not_safe_cnt_ > 8) {
          goal_not_safe_cnt_ = 0;
          PublishZeroVelocity();
          state_ = A_CLEARING;
          recovery_trigger_ = A_GOALSAFE_R;
          ROS_ERROR("[ASTAR CONTROLLER] HandleGoingBack or !IsGoalfootprintSafe, entering A_PLANNING state");
          return false;
        }
      }

			{
        // get the current amcl pose of the robot, and prune fixpattern path
//        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        // check if under rotating_to_goal, if done, prune the first corner point on path
				if (!co_->fixpattern_local_planner->isRotatingToGoal()) {
          if (co_->fixpattern_local_planner->isRotatingToGoalDone()) {
            astar_path_.PruneCornerOnStart();
            co_->fixpattern_local_planner->resetRotatingToGoalDone();
            ROS_INFO("[Astar Controller] Prune Corner Point On Start");  
          } else {
            // do nothing when controller first start, to avoid prune the first corner point
            if(first_run_controller_flag_) {
              first_run_controller_flag_ = false;
            } else {
              tf::Stamped<tf::Pose> global_pose;
              if (controller_costmap_ros_->getRobotPose(global_pose)) {
                geometry_msgs::PoseStamped start;
                tf::poseStampedTFToMsg(global_pose, start);
                start = PoseStampedToGlobalFrame(start);
                astar_path_.Prune(fixpattern_path::GeometryPoseToPathPoint(start.pose), co_->max_offroad_dis, true);
              } else {
                ROS_WARN("[MOVE BASE] Unable to get robot pose, unable to calculate highlight length");
              }
            }
          }
        }
      }

      // check for an oscillation condition
      if (co_->oscillation_timeout > 0.0 &&
         last_oscillation_reset_ + ros::Duration(co_->oscillation_timeout) < ros::Time::now()) {
        PublishZeroVelocity();
        state_ = A_CLEARING;
        recovery_trigger_ = A_OSCILLATION_R;
        ROS_ERROR("[ASTAR CONTROLLER] oscillation, entering GOALSAFE_R");
      }

      // if astar_path_ is not long enough and new plan has not arrived, just wait
      if (!using_sbpl_directly_ &&
          (PoseStampedDistance(current_position, planner_goal_) <= co_->sbpl_max_distance ||
           astar_path_.Length() < 2.5)) {
        // just restart the planner, and we'll not stop during this time
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();
        ROS_INFO("[ASTAR CONTROLLER] distance to start point < 2.5, replan");
      }

      // if astar_path_ is not long enough and new plan has not arrived, just wait
      if (!using_sbpl_directly_ && astar_path_.Length() < 0.5) {
        PublishZeroVelocity();
        state_ = A_PLANNING;
        recovery_trigger_ = A_PLANNING_R;
        ROS_WARN("[ASTAR CONTROLLER] astar_path.Length() < 0.5, entering A_PLANNING state");

        // continue the planner
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        return false;
      }

      // check if footprint hits something alongside the road, in a limited distance
      if (!IsPathFootprintSafe(astar_path_, co_->front_safe_check_dis)) {
        ROS_WARN("[ASTAR CONTROLLER] IsPathFootprintSafe count = %d", path_not_safe_cnt_);
        ++path_not_safe_cnt_;
      } else {
        path_not_safe_cnt_ = 0;
      }
      if (path_not_safe_cnt_ > 5) {
        path_not_safe_cnt_ = 0;
        PublishZeroVelocity();
        state_ = A_PLANNING;
        recovery_trigger_ = A_PLANNING_R;
        ROS_ERROR("[ASTAR CONTROLLER] !IsPathFootprintSafe, entering A_PLANNING state");

        // continue the planner
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        return false;
      }

      // change highlight and set plan for fixpattern_local_planner
      {
        //boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        // change highlight to 1.5 meters for sbpl path and 1.0 for points that around corner
        std::vector<fixpattern_path::PathPoint> path = astar_path_.path();
        if (!co_->fixpattern_local_planner->setPlan(path, co_->global_frame)) {
          // ABORT and SHUTDOWN COSTMAPS
          ROS_ERROR("[ASTAR CONTROLLER] Failed to pass global plan to the controller, aborting.");
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ResetState();
          return true;
        }

        bool local_planner_ret = co_->fixpattern_local_planner->computeVelocityCommands(
            fixpattern_local_planner::TRAJECTORY_PLANNER, &cmd_vel);
        if (!local_planner_ret) {
          ++local_planner_error_cnt_;
          cmd_vel = last_valid_cmd_vel_;
        } else {
          local_planner_error_cnt_ = 0;
          last_valid_cmd_vel_ = cmd_vel;
        }

        if (local_planner_error_cnt_ < 3) {
          ROS_DEBUG_NAMED("autoscrubber", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
          last_valid_control_ = ros::Time::now();
          // make sure that we send the velocity command to the base
          co_->vel_pub->publish(cmd_vel);
        } else {
          
          
          ROS_WARN("[ASTAR CONTROLLER] The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(co_->controller_patience);

          // check if we've tried to find a valid control for longer than our time limit
          if (ros::Time::now() > attempt_end) {
            // we'll move into our obstacle clearing mode
            PublishZeroVelocity();
            // TODO(lizhen): check this variable
            local_planner_error_cnt_ = 0;
            state_ = A_CLEARING;
            recovery_trigger_ = A_CONTROLLING_R;
            ROS_WARN("[ASTAR CONTROLLER] controller_patience time out!");
          } else {
            // otherwise, if we can't find a valid control, we'll retry, until
            // reach controller_patience
            ROS_WARN("[ASTAR CONTROLLER] wait for a valid control!");
            state_ = A_CONTROLLING;
            PublishZeroVelocity();
            // // otherwise, if we can't find a valid control, we'll go back to planning
            // last_valid_plan_ = ros::Time::now();
            // state_ = A_PLANNING;
            // PublishZeroVelocity();

            // enable the planner thread in case it isn't running on a clock
            // boost::unique_lock<boost::mutex> lock(planner_mutex_);
            // runPlanner_ = true;
            // planner_cond_.notify_one();
            // lock.unlock();
          }
        }
      }

      break;

    case FIX_CONTROLLING:
      ROS_INFO("[FIXPATTERN CONTROLLER] in FIX_CONTROLLING state");
      ROS_DEBUG_NAMED("autoscrubber", "In controlling state.");

      // check to see if we've reached our goal
      if (co_->fixpattern_local_planner->isGoalReached()) {
        ROS_WARN("[FIXPATTERN CONTROLLER] fixpattern goal reached");
        ROS_DEBUG_NAMED("autoscrubber", "Goal reached!");
        ResetState();
        // reset fixpattern_local_planner
        co_->fixpattern_local_planner->reset_planner();	
        first_run_controller_flag_ = true;

        // we need to notify fixpattern_path
        co_->fixpattern_path->FinishPath();
        // check is global goal reached
        if (!IsGlobalGoalReached(current_position, global_goal_, 
                                 co_->fixpattern_local_planner->xy_goal_tolerance_, co_->fixpattern_local_planner->yaw_goal_tolerance_)) {
          ROS_WARN("[FIXPATTERN CONTROLLER] global goal not reached yet, swtich to PLANNING state");
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
          planning_state_ = P_INSERTING_BEGIN;
          break;  //(lee)
        } else  {
          ROS_WARN("[FIXPATTERN CONTROLLER] global goal reached, teminate controller");
          switch_controller_ = false;
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
          planning_state_ = P_INSERTING_BEGIN;
          ROS_ERROR("[FIXPATTERN CONTROLLER] HandleGoingBack, entering A_PLANNING state");
          return false;
        }
      }
*/
//		if (!co_->fixpattern_local_planner->isRotatingToGoal()) {
    {
        //boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));
        // we'll Prune the path first as we don't want to navigate back when
        // trigger front_safe while robot still moves
        // get current pose of the vehicle && prune fixpattern path
        // we'll not prune any point when first run 
        // TODO(lizhen) check if needed :first_run_controller_flag_

        if(first_run_controller_flag_) {
          first_run_controller_flag_ = false;
        } else {
         if (cur_goal_distance > co_->fixpattern_local_planner->xy_goal_tolerance_) {
           if(co_->fixpattern_local_planner->isRotatingToGoalDone()) {
             co_->fixpattern_path->PruneCornerOnStart();
             co_->fixpattern_local_planner->resetRotatingToGoalDone();
             ROS_INFO("[FIXPATTERN CONTROLLER] Prune Corner Point On Start");  
           } else {
             if (!co_->fixpattern_path->Prune(fixpattern_path::GeometryPoseToPathPoint(current_position.pose), co_->max_offroad_dis, true)) {
               PublishZeroVelocity();
               state_ = FIX_CLEARING;
               recovery_trigger_ = FIX_GETNEWGOAL_R;
               planning_state_ = P_INSERTING_BEGIN;
               break;
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
        double front_safe_dis = CheckFixPathFrontSafe(co_->front_safe_check_dis);
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
              if (IsGoalFootprintSafe(0.5, 0.0, global_goal_)) {
                if (++check_goal_safe_cnt > 5) {
                  is_goal_safe = true;
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

              // TODO(chenkan): check if this is needed
              co_->fixpattern_local_planner->reset_planner();
              // Goal not reached, but we will stop and exit ExecuteCycle
              switch_controller_ = false;
              return true;
            }
          }
        } else if (front_safe_dis < co_->front_safe_check_dis) { // check front safe distance
          if (front_safe_dis <= 0.5) {
            front_safe_check_cnt_ = 0;
            PublishZeroVelocity();
            ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
            ros::Rate r(10);
            bool front_safe = false;
            unsigned int front_safe_cnt = 0;
            while (ros::Time::now() < end_time) {
              front_safe_dis = CheckFixPathFrontSafe(co_->front_safe_check_dis);
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
                planner_costmap_ros_->getRobotPose(global_pose);
                tf::poseStampedTFToMsg(global_pose, current_position);
//                GetCurrentPosition(current_position);
                HandleGoingBack(current_position);
//              }
              ROS_ERROR("[FIXPATTERN CONTROLLER] !IsPathFrontSafe dis = %lf,stop and switch to CLEARING", front_safe_dis);
              state_ = FIX_CLEARING;
              recovery_trigger_ = FIX_GETNEWGOAL_R;
              planning_state_ = P_INSERTING_BEGIN;
            } else {
              // clear local planner error cnt, to avoid it stop again
              fix_local_planner_error_cnt_ = 0;
            }
            break;
          } else if (front_safe_dis > 0.5) {
            ROS_WARN("[FIXPATTERN CONTROLLER] !IsPathFrontSafe dis = %lf > 0.5, check_cnt = %d", front_safe_dis, front_safe_check_cnt_);
            if (front_safe_dis < 0.8) 
              cmd_vel_ratio_ = 0.5;
            else if (front_safe_dis < 1.5) 
              cmd_vel_ratio_ = 0.7;
            if (++front_safe_check_cnt_ > 7) {
              ROS_WARN("[FIXPATTERN CONTROLLER] Enable PlanThread and continue FIX_CONTROLLING");
//              GetAStarStart(co_->front_safe_check_dis);	
//              CheckFixPathFrontSafe(co_->front_safe_check_dis, true); // get planner_start
              if (GetAStarGoal(current_position, obstacle_index_)) {
                planning_state_ = P_INSERTING_MIDDLE;

                // enable the planner thread in case it isn't running on a clock
                boost::unique_lock<boost::mutex> lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();
              }
            }
          }
        } else {
          front_safe_check_cnt_ = 0;
        }
      }
    } // is rotating to goal

      {
       // boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

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
            planning_state_ = P_INSERTING_BEGIN;
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
            planning_state_ = P_INSERTING_BEGIN;
          } else {
            // otherwise, if we can't find a valid control, we'll retry, until
            ROS_INFO("[FIXPATTERN CONTROLLER] wait for a valid control");
            // reach controller_patience
            state_ = FIX_CONTROLLING;
            PublishZeroVelocity();
          }
        }
      }

      break;
			
    case FIX_CLEARING:
      ROS_INFO("[FIX CONTROLLER] in FIX_CLEARING state");

      if (recovery_trigger_ == FIX_RECOVERY_R) {
        double x = current_position.pose.position.x;
        double y = current_position.pose.position.y;
        double yaw = tf::getYaw(current_position.pose.orientation);
        if (footprint_checker_->FootprintCenterCost(x, y, yaw, co_->footprint_center_points) < 0) {
        // check if oboscal in footprint, yes - recovery; no - get new goal and replan
//        if (footprint_checker_->FootprintCost(x, y, yaw, footprint_spec_, 0.0, 0.0) < 0) {
          ROS_WARN("[FIXPATTERN CONTROLLER] footprint cost check < 0!, switch to Recovery");
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
        } else {
          ROS_WARN("[FIXPATTERN CONTROLLER] footprint cost check OK!, switch to Replan");
          state_ = FIX_CLEARING;
          recovery_trigger_ = FIX_GETNEWGOAL_R;
        }
      }
      // we'll invoke recovery behavior
      if (recovery_trigger_ == FIX_GETNEWGOAL_R) {
        ROS_INFO("[FIX CONTROLLER] in CLEARING state: FIX_GETNEWGOAL_R");
        PublishZeroVelocity();
        bool new_goal_got = false; 
        // get a new astar goal
        ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
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
//        new_goal_got = GetAStarGoal(current_position);
        // if get a new astar goal fail, try check global_goal_
        if (!new_goal_got) {
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

          // TODO(lizhen) Alarm here
          switch_controller_ = false;
          ROS_ERROR("[FIX CONTROLLER] GetAStarGoal failed, terminate path");
//          return true;
        } else {
          // find a new safe goal, so use it to replan
          state_ = A_PLANNING;
          recovery_trigger_ = A_PLANNING_R;

          // continue the planner
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
        }
      }

      break;

    // we'll try to launch recovery behaviors
    case A_CLEARING:
      ROS_INFO("[ASTAR CONTROLLER] in CLEARING state");
      // we'll invoke recovery behavior
      if (recovery_trigger_ == A_GOALSAFE_R) {
        ROS_INFO("[ASTAR CONTROLLER] in CLEARING state: A_GOALSAFE_R");
        ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
        ros::Rate r(10);
        bool goal_safe = false;
        while (ros::Time::now() < end_time) {
          if (IsGoalFootprintSafe(co_->goal_safe_dis_a, co_->goal_safe_dis_b, planner_goal_)) {
            goal_safe = true;
            break;
          }
          PublishZeroVelocity();
          last_valid_control_ = ros::Time::now();
          r.sleep();
        }
        if (goal_safe) {
          // we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          state_ = A_PLANNING;
          recovery_trigger_ = A_PLANNING_R;
          break;
        } else {
          ROS_WARN("[ASTAR CONTROLLER] in CLEARING state: A_GOALSAFE_R check goal footprint failed");
        }
      } else if (recovery_trigger_ == A_PLANNING_R) {
        ROS_INFO("[ASTAR CONTROLLER] in CLEARING state: A_PLANNING_R");
        // disable the planner thread, otherwise costmap will be locked
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        //HandleGoingBack(current_position);
        RotateRecovery();
      }

      {
        int begin_index = 0;
        if (recovery_trigger_ == A_PLANNING_R) {
          // if already sbpl_broader_, cut some points
          if (sbpl_broader_) {
            unsigned int size = co_->fixpattern_path->current_point_count();
            if (size > 0) size -= 1;
            begin_index = std::min(planner_goal_index_ + 50, size);
          } else {
            ROS_INFO("[ASTAR CONTROLLER] set sbpl_broader_ to true");
            sbpl_broader_ = true;
          }
        }
				bool new_goal_got = false; 
				geometry_msgs::PoseStamped start;
				tf::Stamped<tf::Pose> global_pose;
				if (!planner_costmap_ros_->getRobotPose(global_pose)) {
					ROS_WARN("Unable to get starting pose of robot, unable to create sbpl plan");
				} else {
					tf::poseStampedTFToMsg(global_pose, start);
					// get a new astar goal
					new_goal_got = GetAStarGoal(start, begin_index);
					// if get a new astar goal fail, try check global_goal_
					if (!new_goal_got && IsGoalFootprintSafe(0.5, 0.0, global_goal_)) {
						new_goal_got = true;
						planner_goal_ = global_goal_;
					}
					if (!new_goal_got) {
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

						switch_controller_ = false;
						ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, terminate path");

						return true;
					}
				}	
        if (new_goal_got) {
          // find a new safe goal, so use it to replan
          state_ = A_PLANNING;
          recovery_trigger_ = A_PLANNING_R;

          // continue the planner
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
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
/*
  state_ = F_CONTROLLING;
  recovery_trigger_ = F_CONTROLLING_R;
  PublishZeroVelocity();
*/
//  ROS_INFO("[ASTAR CONTROLLER] ResetState");
  // search planner goal from start
  planner_goal_index_ = 0;
  cmd_vel_ratio_ = 1.0;

  // reset some variables
  using_sbpl_directly_ = false;
}

bool AStarController::IsGlobalGoalReached(const geometry_msgs::PoseStamped& current_position, const geometry_msgs::PoseStamped& global_goal,
                                            double xy_goal_tolerance, double yaw_goal_tolerance) {
  double pose_diff = PoseStampedDistance(current_position, global_goal);
  double yaw_diff = angles::shortest_angular_distance(tf::getYaw(current_position.pose.orientation), tf::getYaw(global_goal.pose.orientation));
  if (pose_diff > xy_goal_tolerance && fabs(yaw_diff) > yaw_goal_tolerance) {
		return false;
	} else {
    return true;
  }
}

geometry_msgs::PoseStamped AStarController::PoseStampedToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg) {
  std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
  tf::Stamped<tf::Pose> goal_pose, global_pose;
  poseStampedMsgToTF(pose_msg, goal_pose);

  // just get the latest available transform... for accuracy they should send
  // goals in the frame of the planner
  goal_pose.stamp_ = ros::Time();

  try {
    co_->tf->transformPose(global_frame, goal_pose, global_pose);
  } catch(tf::TransformException& ex) {
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
             goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
    return pose_msg;
  }

  geometry_msgs::PoseStamped global_pose_msg;
  tf::poseStampedTFToMsg(global_pose, global_pose_msg);
  return global_pose_msg;
}

geometry_msgs::PoseStamped AStarController::PoseStampedToLocalFrame(const geometry_msgs::PoseStamped& pose_msg) {
  std::string local_frame = controller_costmap_ros_->getGlobalFrameID();
  tf::Stamped<tf::Pose> goal_pose, global_pose;
  poseStampedMsgToTF(pose_msg, goal_pose);

  // just get the latest available transform... for accuracy they should send
  // goals in the frame of the planner
  goal_pose.stamp_ = ros::Time();

  try {
    tf_.transformPose(local_frame, goal_pose, global_pose);
  } catch(tf::TransformException& ex) {
    ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
             goal_pose.frame_id_.c_str(), local_frame.c_str(), ex.what());
    return pose_msg;
  }

  geometry_msgs::PoseStamped global_pose_msg;
  tf::poseStampedTFToMsg(global_pose, global_pose_msg);
  return global_pose_msg;
}

bool AStarController::GetCurrentPosition(geometry_msgs::PoseStamped& current_position) {
  tf::Stamped<tf::Pose> global_pose;
  geometry_msgs::PoseStamped cur_pos;
  if (!planner_costmap_ros_->getRobotPose(global_pose)) {
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
  ROS_INFO("[ASTAR CONTROLLER] cur_goal_dis = %lf", cur_goal_dis);

  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  if (path.size() == 0) {
	  ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, path_size = %zu == 0", path.size());
    return false; 
  }

  if(begin_index == 0 && (cur_goal_dis < 2.5 || // co_->sbpl_max_distance
     co_->fixpattern_path->Length() < co_->front_safe_check_dis)) {  
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
      for (it = path.end() - 1; it >= path.begin() + 5; it -= 5) {
			  if (IsGoalFootprintSafe(0.5 , 0.3, *it)) {
			    planner_goal_ = *it;
			    planner_goal_.header.frame_id = co_->global_frame;
			    planner_goal_index_ = it - path.begin();
          ROS_INFO("[ASTAR CONTROLLER] taking global_goal_ as planner_goal_");
			    return true;
        }
        acc_dis += PoseStampedDistance(*it, *(it - 5));
        if (acc_dis > cur_goal_dis) {
			    ROS_WARN("[ASTAR CONTROLLER] Cur_goal_dis = %lf < 2.5m, but GetAStarGoal failed", cur_goal_dis);
			    return false;
        }
      }
    }
  } else {
    if (planning_state_ == P_INSERTING_BEGIN) {
      co_->fixpattern_path->Prune(fixpattern_path::GeometryPoseToPathPoint(cur_pose.pose), co_->max_offroad_dis, false);
    }
/*
    if (begin_index == 0) {
      unsigned int pre_goal_index = GetPoseIndexOfPath(path, planner_goal_);
      if (pre_goal_index != 0x7FFFFFFF)
        begin_index = pre_goal_index;
      else 
        begin_index = planner_goal_index_;
    } else {
        ROS_INFO("[ASTAR CONTROLLER] search astar goal from: %d", begin_index);
    }
*/
    // we don't want cross_obstacle take effect when planning to start point
    // if (path.size() == co_->fixpattern_path->total_point_count()) {
    //   cross_obstacle = true;
    // }

/*
    double dis_accu = 0.0;
    bool goal_found = false;

    double goal_safe_dis_a, goal_safe_dis_b;
    // TODO(lizhen): why this is needed
    goal_safe_dis_a = cur_goal_dis > co_->goal_safe_dis_a + co_->front_safe_check_dis + 0.10 ?
                        co_->goal_safe_dis_a : cur_goal_dis - co_->front_safe_check_dis;
    goal_safe_dis_a = goal_safe_dis_a < 0.3 ? 0.3 : goal_safe_dis_a;
    goal_safe_dis_b = cur_goal_dis > co_->goal_safe_dis_a + co_->front_safe_check_dis + 0.10 ?
                        co_->goal_safe_dis_b : 0.0;
    ROS_INFO("[ASTAR CONTROLLER] cur_goal_dis = %lf, goal_safe_dis_a = %lf, goal_safe_dis_b = %lf", cur_goal_dis, co_->goal_safe_dis_a, co_->goal_safe_dis_b);
*/
    bool cross_obstacle = false;
    double dis_accu = 0.0;
    int goal_index = -1;
    for (int i = begin_index; i < path.size(); i += 5) {
      double x = path[i].pose.position.x;
      double y = path[i].pose.position.y;
      double yaw = tf::getYaw(path[i].pose.orientation);
      if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0 ||
           !IsGoalFootprintSafe(co_->goal_safe_dis_a, co_->goal_safe_dis_b, path[i])) {
         cross_obstacle = true;
         continue;
       }
       if (i > begin_index) dis_accu += PoseStampedDistance(path.at(i), path.at(i - 5));
         // we must enforce cross obstacle within front_safe_check_dis range
         if (!cross_obstacle && dis_accu <= co_->front_safe_check_dis) continue;
         goal_index = i;
         break;
       }
//      if (!cross_obstacle) it = path.begin() + begin_index;
       if (goal_index == -1) {
         ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, cost: %lf secs", GetTimeInSeconds() - start);
         return false;
       }
    planner_goal_ = path[goal_index];
    planner_goal_.header.frame_id = co_->global_frame;
    planner_goal_index_ = goal_index;
  }
/*
    std::vector<geometry_msgs::PoseStamped>::iterator it;
    for (it = path.begin() + begin_index; it <= path.end(); it += 5) {
      double x = *it.pose.position.x;
      double y = *it.pose.position.y;
      double yaw = tf::getYaw(*it.pose.orientation);
      if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0) {
				continue;
			}
			if (!IsGoalFootprintSafe(co_->goal_safe_dis_a, co_->goal_safe_dis_b, *it)) {
				cross_obstacle = true;
				continue;
			}
			if (it != path.begin() + begin_index) dis_accu += PoseStampedDistance(*it, *(it - 5));
			// we must enforce cross obstacle within front_safe_check_dis range
			if (!cross_obstacle && dis_accu <= co_->front_safe_check_dis) continue;
			goal_found = true;
			break;
		}
		if (!cross_obstacle) it = path.begin() + begin_index;
		if (!goal_found || it > path.end()) {
			ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, cost: %lf secs", GetTimeInSeconds() - start);
			return false;
		}
    planner_goal_ = *it;
    planner_goal_.header.frame_id = co_->global_frame;
    planner_goal_index_ = it - path.begin();
  }
*/
  ROS_INFO("[ASTAR CONTROLLER] GetAStarGoal cost: %lf secs", GetTimeInSeconds() - start);
  ROS_INFO("[ASTAR CONTROLLER] planner_goal_index_: %d", planner_goal_index_);
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

bool AStarController::GetInitalPath(const geometry_msgs::PoseStamped& global_start, const geometry_msgs::PoseStamped& global_goal) {
  gotStartPlan_ = false;
  gotGoalPlan_ = false;
  std::vector<fixpattern_path::PathPoint> fix_path = co_->fixpattern_path->path();
	
  if (!co_->fixpattern_path->IsRunning()) {
    std::string file_name = "/home/gaussian/record.path";
    const char* runtime_dir = ::getenv("GAUSSIAN_RUNTIME_DIR");
    if (runtime_dir != NULL) {
      file_name = std::string(runtime_dir) + "/run.path";
    }
    co_->fixpattern_path->SyncFromFile(file_name.c_str());
    ROS_INFO("[ASTAR CONTROLLOR] path synced from file: %s", file_name.c_str());
    co_->fixpattern_path->PruneFromStartToGoal(fixpattern_path::GeometryPoseToPathPoint(global_start.pose), fixpattern_path::GeometryPoseToPathPoint(global_goal.pose));

    std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath(); 
    for (auto&& p : path) {  // NOLINT
      p.header.frame_id = co_->global_frame;
      p.header.stamp = ros::Time::now();
    }
    PublishPlan(fixpattern_pub_, path);

    // get sbpl path from current pose to fix_path.front, and then insert sbpl path into fix_path
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;
    start = global_start; 
    goal = path.front();
    ROS_INFO("[ASTAR CONTROLLOR] Start Sbpl Path Planning ...");
    //ROS_INFO("[ASTAR CONTROLLOR]Start Pose(%lf,%lf,%lf)",start.pose.position.x, start.pose.position.y, tf::getYaw(goal.pose.orientation));
    //ROS_INFO("[ASTAR CONTROLLOR]Goal Pose(%lf,%lf,%lf)",goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
    if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
      gotStartPlan_ = MakePlan(start, goal, planner_plan_);
    }
    else {
      gotStartPlan_ = false;
    }
    if(gotStartPlan_ && !astar_path_.path().empty()) {
      co_->fixpattern_path->insert_begin_path(astar_path_.path(), true, start);
//      fix_path.insert(fix_path.begin(), astar_path_.path().begin(), astar_path_.path().end() - 1);
      ROS_INFO("[ASTAR CONTROLLOR] Insert Start Path to fixpattern_path");
		}
	  	
    // get astar path from (end point of fixpattern_path) to the real goal point
    start = path.back(); 
    goal = global_goal;
    ROS_INFO("[ASTAR CONTROLLOR] Goal Sbpl Path Planning ...");
    //ROS_INFO("[ASTAR CONTROLLOR]Start Pose(%lf,%lf,%lf)",start.pose.position.x, start.pose.position.y, tf::getYaw(goal.pose.orientation));
    //ROS_INFO("[ASTAR CONTROLLOR]Goal Pose(%lf,%lf,%lf)",goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
    if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
      gotGoalPlan_ = MakePlan(start, goal, planner_plan_);
    }
    else
      gotGoalPlan_ = false;

    if(gotGoalPlan_ && !astar_path_.path().empty()) {
			  co_->fixpattern_path->insert_end_path(astar_path_.path());
//        fix_path.insert(fix_path.end(), astar_path_.path().begin(), astar_path_.path().end());
        ROS_INFO("[ASTAR CONTROLLOR]Insert Goal Path to fixpattern_path");
    }
  }

  if (gotStartPlan_ || gotGoalPlan_) {
//    co_->fixpattern_path->set_fix_path(fix_path);
  }

  std::vector<geometry_msgs::PoseStamped> plan = co_->fixpattern_path->GeometryPath();
  for (auto&& p : plan) {  // NOLINT
    p.header.frame_id = co_->global_frame;
    p.header.stamp = ros::Time::now();
  }

  PublishPlan(fixpattern_pub_, plan);
  return gotStartPlan_;
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
//       path_recorder::PathRecorder recorder;
//       recorder.CalculateCurvePath(&fix_path);
       co_->fixpattern_path->set_fix_path(global_start, fix_path); 

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

bool AStarController::HandleGoingBack(const geometry_msgs::PoseStamped& current_position) {
  geometry_msgs::Twist cmd_vel;
  geometry_msgs::PoseStamped cur_pos = current_position;
  // check if need backward
  ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration / 3);
  bool need_backward = true;
  ros::Rate r(10);
  while (ros::Time::now() < end_time) {
    if (!NeedBackward(cur_pos, 0.10)) {
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
  while (need_backward && NeedBackward(cur_pos, 0.15) && CanBackward(0.25)) {
    ROS_INFO("[ASTAR CONTROLLER] going back");
    // get curent position
    planner_costmap_ros_->getRobotPose(global_pose);
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
/*
  if (footprint_checker_->FootprintCenterCost(x, y, yaw, co_->footprint_center_points) < 0) {
    ROS_WARN("[FIXPATTERN CONTROLLER] footprint cost check < 0!");
  }
*/
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
  double resolution = planner_costmap_ros_->getCostmap()->getResolution();
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
  double resolution = planner_costmap_ros_->getCostmap()->getResolution();
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
