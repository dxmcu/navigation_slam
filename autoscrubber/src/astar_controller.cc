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

namespace autoscrubber {

AStarController::AStarController(tf::TransformListener* tf,
                                 costmap_2d::Costmap2DROS* planner_costmap_ros,
                                 costmap_2d::Costmap2DROS* controller_costmap_ros)
    : tf_(*tf),
      planner_costmap_ros_(planner_costmap_ros), controller_costmap_ros_(controller_costmap_ros),
      planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL), sbpl_reached_goal_(false),
      planner_goal_index_(0), runPlanner_(false), new_global_plan_(false),
      using_sbpl_directly_(false), sbpl_broader_(false) {
  // set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  // create footprint_checker_
  footprint_checker_ = new autoscrubber::FootprintChecker(*planner_costmap_ros_->getCostmap());

  // set up the planner's thread
  planner_thread_ = new boost::thread(boost::bind(&AStarController::PlanThread, this));

  // initially, we'll need to make a plan
  state_ = A_PLANNING;

  // we'll start executing recovery behaviors at the beginning of our list
  recovery_trigger_ = A_PLANNING_R;

  // set rotate_recovery_dir_
  rotate_recovery_dir_ = 0;
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
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

  // make sure to set the plan to be empty initially
  plan->clear();

  // since this gets called on handle activate
  if (planner_costmap_ros_ == NULL) {
    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
    return false;
  }
/*
  // get the starting pose of the robot
  geometry_msgs::PoseStamped start;

  if (state_ == A_PLANNING) {
    tf::Stamped<tf::Pose> global_pose;
    if (!planner_costmap_ros_->getRobotPose(global_pose)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }
    tf::poseStampedTFToMsg(global_pose, start);
  } else {
    // if MakePlan when not in A_PLANNING, we need to make plan based on last plan
    std::vector<geometry_msgs::PoseStamped> path = astar_path_.GeometryPath();
    // path shouldn't be empty
    if (path.size() == 0) return false;
    start = path.back();
    start.header.frame_id = planner_costmap_ros_->getGlobalFrameID();
  }
*/
  if (PoseStampedDistance(start, goal) <= 0.25) {
    // set this to true as we'll use it afterwards
    using_sbpl_directly_ = true;

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
      astar_path_.set_path(path, false);
    } else {
      fixpattern_path::Path temp_path;
      temp_path.set_path(path);
      astar_path_.ExtendPath(temp_path.path());
    }
  } else if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
    // too short, use sbpl directly
    ROS_INFO("[ASTAR CONTROLLER] too short, use sbpl directly");
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
  }

  return true;
}

void AStarController::PublishZeroVelocity() {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  co_->vel_pub->publish(cmd_vel);
}

void AStarController::WakePlanner(const ros::TimerEvent& event) {
  // we have slept long enough for rate
  planner_cond_.notify_one();
}

void AStarController::PlanThread() {
  ROS_INFO("[ASTAR CONTROLLER] Starting planner thread...");
  ros::NodeHandle n;
  ros::Timer timer;
  bool wait_for_wake = false;
  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  while (n.ok()) {
    // check if we should run the planner (the mutex is locked)
    while (wait_for_wake || !runPlanner_) {
      // if we should not be running the planner then suspend this thread
      ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
      planner_cond_.wait(lock);
      wait_for_wake = false;
    }
    ros::Time start_time = ros::Time::now();

    // time to plan! get a copy of the goal and unlock the mutex
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

    // get the starting pose of the robot
    geometry_msgs::PoseStamped start;
    bool gotStartPose = true;
    if (state_ == A_PLANNING) {
      tf::Stamped<tf::Pose> global_pose;
      if (!planner_costmap_ros_->getRobotPose(global_pose)) {
        gotStartPose = false;
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      }
      tf::poseStampedTFToMsg(global_pose, start);
    } else {
      // if MakePlan when not in A_PLANNING, we need to make plan based on last plan
      std::vector<geometry_msgs::PoseStamped> path = astar_path_.GeometryPath();
      // path shouldn't be empty
      if (path.size() == 0) 
         gotStartPose = false;
      else {
        start = path.back();
        start.header.frame_id = planner_costmap_ros_->getGlobalFrameID();
      }
  }
    // run planner
    planner_plan_->clear();
    bool gotPlan = false;
    if(gotStartPose) 
        gotPlan = n.ok() && MakePlan(start, temp_goal, planner_plan_);

    if (gotPlan) {
      ROS_INFO("[ASTAR CONTROLLER] Got Plan with %zu points!", planner_plan_->size());
      // pointer swap the plans under mutex (the controller will pull from latest_plan_)
      std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

      lock.lock();
      planner_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      last_valid_plan_ = ros::Time::now();
      new_global_plan_ = true;
      // reset rotate_recovery_dir_
      rotate_recovery_dir_ = 0;
      // reset sbpl_broader_
      if (sbpl_broader_) success_broader_goal_ = temp_goal;
      else success_broader_goal_.pose.position.x = success_broader_goal_.pose.position.y = 1000000;
      sbpl_broader_ = false;

      ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

      // make sure we only start the controller if we still haven't reached the goal
      if (runPlanner_)
        state_ = A_CONTROLLING;
      if (co_->planner_frequency <= 0)
        runPlanner_ = false;
      lock.unlock();
    } else if (state_ == A_PLANNING) {  // if we didn't get a plan and we are in the planning state (the robot isn't moving)
      ROS_ERROR("[ASTAR CONTROLLER] No Plan...");
      ros::Time attempt_end = last_valid_plan_ + ros::Duration(co_->planner_patience);

      // check if we've tried to make a plan for over our time limit
      lock.lock();
      if (ros::Time::now() > attempt_end && runPlanner_) {
        // don't allow plan, as RotateRecovery needs global costmap
        runPlanner_ = false;
        // we'll move into our obstacle clearing mode
        state_ = A_CLEARING;
        PublishZeroVelocity();
        recovery_trigger_ = A_PLANNING_R;
      } else if (runPlanner_) {
        // to update global costmap
        usleep(500000);
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

unsigned int CalculatePathLengthIndex(const std::vector<fixpattern_path::PathPoint>& path, double distance, bool direction) {
  double accumulated_dis = 0.0;
  unsigned int path_index;
  if(direction) {
    for (path_index = 0; path_index < path.size() - 2; ++path_index) {
      accumulated_dis += path.at(path_index + 1).DistanceToPoint(path.at(path_index));
      if(accumulated_dis > distance)
        break;
    }
  } else {
    for (path_index = path.size() - 1; path_index > 1; --path_index) {
      accumulated_dis += path.at(path_index).DistanceToPoint(path.at(path_index - 1));
      if(accumulated_dis > distance)
        break;
    }
  }
  return path_index;
}

void AStarController::CalculateStartCurvePath(const std::vector<fixpattern_path::PathPoint>& astar_path) {
  std::vector<fixpattern_path::PathPoint> path = astar_path;
  std::vector<fixpattern_path::PathPoint> fix_path = co_->fixpattern_path->path();
  fix_path.insert(fix_path.begin(), astar_path.begin(), astar_path.end() - 1);
  co_->fixpattern_path->set_fix_path(fix_path);
/*
  unsigned int index = CalculatePathLengthIndex(fix_path, 0.5, 1);
  path.insert(path.end(), fix_path.begin(), fix_path.begin() + index);
  ROS_INFO("[Astar Controller]Fix_start_smooth_index=%d, before smooth_path_size=%d", index, (int)path.size());
  path_recorder::PathRecorder recorder;
 // recorder.CalculateCurvePath(&path);
  ROS_INFO("[Astar Controller]After start_smooth_path_size=%d", (int)path.size());
  path.insert(path.end(), fix_path.begin() + index + 1, fix_path.end());
  co_->fixpattern_path->set_fix_path(path);
*/
  ROS_INFO("[Astar Controller]set start fix_path s");
}

void AStarController::CalculateGoalCurvePath(const std::vector<fixpattern_path::PathPoint>& astar_path) {
  std::vector<fixpattern_path::PathPoint> path = astar_path;
  std::vector<fixpattern_path::PathPoint> fix_path = co_->fixpattern_path->path();
  fix_path.insert(fix_path.end(), astar_path.begin() + 1, astar_path.end());
  co_->fixpattern_path->set_fix_path(fix_path);
/*
  unsigned int index = CalculatePathLengthIndex(fix_path, 0.5, 0);
  ROS_INFO("[Astar Controller]Fix_goal_smooth_index=%d, before smooth_path_size=%d", index, (int)path.size());
  path.insert(path.begin(), fix_path.begin() + index, fix_path.end());

  path_recorder::PathRecorder recorder;
 // recorder.CalculateCurvePath(&path);
  ROS_INFO("[Astar Controller]After goal_smooth_path_size=%d", (int)path.size());
  path.insert(path.begin(), fix_path.begin(), fix_path.begin() + index - 1);
  co_->fixpattern_path->set_fix_path(path);
*/
  ROS_INFO("[Astar Controller]set goal fix_path s");
}

bool AStarController::Control(BaseControlOption* option, ControlEnvironment* environment) {
  ROS_INFO("[ASTAR_CTRL] Switch to Astar Controller!");
  co_ = reinterpret_cast<AStarControlOption*>(option);
  env_ = environment;
  // first run: just get Astar Path, and insert to fixpattern_path, then switch controllor 
  bool gotStartPoint = false;
  geometry_msgs::PoseStamped start;
  geometry_msgs::PoseStamped goal;
  tf::Stamped<tf::Pose> global_pose;
	double start_goal_distance;
  if (!planner_costmap_ros_->getRobotPose(global_pose)) {
    ROS_WARN("Unable to get starting pose of robot, unable to create sbpl plan");
  } else {
		gotStartPoint = true;
    tf::poseStampedTFToMsg(global_pose, start);
    goal.pose = co_->global_planner_goal->pose;
    goal.header.frame_id = co_->global_frame;
		start_goal_distance = PoseStampedDistance(start, goal); 
  }
  if(co_->fixpattern_path->fixpattern_path_first_run_) {
    co_->fixpattern_path->fixpattern_path_first_run_ = false;
    bool gotPlan = false;
    if (gotStartPoint) {
      std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath(); 
      goal.pose = path.front().pose;
      goal.header.frame_id = co_->global_frame;
      ROS_INFO("[ASTAR CONTROLLOR]Start Path Planning ...");
      //ROS_INFO("[ASTAR CONTROLLOR]Start Pose(%lf,%lf,%lf)",start.pose.position.x, start.pose.position.y, tf::getYaw(goal.pose.orientation));
      //ROS_INFO("[ASTAR CONTROLLOR]Goal Pose(%lf,%lf,%lf)",goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
      if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
        gotPlan = MakePlan(start, goal, planner_plan_);
      }
      else
        gotPlan = false;
    }
    if(gotPlan && !astar_path_.path().empty()) {
      ROS_INFO("[ASTAR CONTROLLOR]Insert Start Path to fixpattern_path");
      //co_->fixpattern_path->insert_begin_path(astar_path_.path());
      //co_->fixpattern_path->set_path(pathpoint, true);
      //co_->fixpattern_path->fixpattern_path_start_updated_ = true;
      CalculateStartCurvePath(astar_path_.path());
      // get astar path from (end point of fixpattern_path) to the real goal point
      std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath(); 
      start.pose = path.back().pose; 
      start.header.frame_id = co_->global_frame;
      goal.pose = co_->global_planner_goal->pose;
      goal.header.frame_id = co_->global_frame;
      ROS_INFO("[ASTAR CONTROLLOR]Goal Path Planning ...");
      //ROS_INFO("[ASTAR CONTROLLOR]Start Pose(%lf,%lf,%lf)",start.pose.position.x, start.pose.position.y, tf::getYaw(goal.pose.orientation));
      //ROS_INFO("[ASTAR CONTROLLOR]Goal Pose(%lf,%lf,%lf)",goal.pose.position.x, goal.pose.position.y, tf::getYaw(goal.pose.orientation));
      if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
        gotPlan = MakePlan(start, goal, planner_plan_);
      }
      else
        gotPlan = false;

      if(gotPlan && !astar_path_.path().empty()) {
        ROS_INFO("[ASTAR CONTROLLOR]Insert Goal Path to fixpattern_path");
        CalculateGoalCurvePath(astar_path_.path());
        //co_->fixpattern_path->insert_end_path(astar_path_.path());
        //co_->fixpattern_path->insert_to_path(astar_path_.path().back());
        co_->fixpattern_path->fixpattern_path_goal_updated_ = true;
        ROS_INFO("[ASTAR CONTROLLOR]Set Curve Goal Path s");
      }
      return false;
    } else {
      std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath(); 
      planner_goal_.pose = path.front().pose;
      planner_goal_.header.frame_id = co_->global_frame;
      co_->fixpattern_local_planner->reset_planner();
    }
  } else if (start_goal_distance <= co_->sbpl_max_distance || 
             co_->fixpattern_path->fixpattern_path_reached_goal_) { 
    //insert astar goal path fail and now fixpattern_reached goal
    planner_goal_.pose = co_->global_planner_goal->pose;
    planner_goal_.header.frame_id = co_->global_frame;
    co_->fixpattern_local_planner->reset_planner();
    ROS_INFO("[ASTAR_CONTROLLER] take planner_goal as astar goal");
  } else if (!GetAStarGoal()) {  
    // get a goal first, if get failed, we'll terminate
    ROS_WARN("[MOVE BASE] cannot get astar goal, terminate path");

    // we need to notify fixpattern_path
    co_->fixpattern_path->FinishPath();

    return true;
  }

  // initialize some flag
  using_sbpl_directly_ = false;

  // start the planner
  boost::unique_lock<boost::mutex> lock(planner_mutex_);
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  ros::Rate r(co_->controller_frequency);

  // we want to make sure that we reset the last time we had a valid plan and control
  last_valid_control_ = ros::Time::now();
  last_valid_plan_ = ros::Time::now();
  last_oscillation_reset_ = ros::Time::now();

  ros::NodeHandle n;
  while (n.ok()) {
    // if is paused, continue
    if (env_->pause_flag) {
      r.sleep();
      continue;
    }
    // if terminated, return true directly
    if (!env_->run_flag) {
      co_->fixpattern_path->EraseToPoint(fixpattern_path::GeometryPoseToPathPoint(planner_goal_.pose));
      ResetState();

      // disable the planner thread
      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      runPlanner_ = false;
      lock.unlock();

      // TODO(chenkan): check if this is needed
      co_->fixpattern_local_planner->reset_planner();

      // we need to notify fixpattern_path
      co_->fixpattern_path->FinishPath();

      return true;
    }

    // for timing that gives real time even in simulation
    ros::WallTime start = ros::WallTime::now();

    // the real work on pursuing a goal is done here
    bool done = ExecuteCycle();

    // if we're done, then we'll return from execute
    if (done) {
      if(sbpl_reached_goal_)
        return true;
      else
        return false;
    }
    // check if execution of the goal has completed in some way

    ros::WallDuration t_diff = ros::WallTime::now() - start;
    ROS_DEBUG_NAMED("autoscrubber", "Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / co_->controller_frequency) && state_ == A_CONTROLLING)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", co_->controller_frequency, r.cycleTime().toSec());
  }

  // wake up the planner thread so that it can exit cleanly
  lock.lock();
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  // if !n.ok(), we don't want to switch controllers
  return true;
}

bool AStarController::IsPoseFootprintSafe(double goal_safe_dis_a, double goal_safe_dis_b, const geometry_msgs::PoseStamped& pose) {
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
    if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0)
      return false;
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
    if (footprint_checker_->CircleCenterCost(x, y, yaw, co_->circle_center_points) < 0)
      return false;
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

  // if not safe, let's cast some padding to footprint
  std::vector<geometry_msgs::Point> circle_center_points_padding_1 = co_->circle_center_points;
  for (auto&& p : circle_center_points_padding_1) p.y +=co_->sbpl_footprint_padding;
  if (IsPathFootprintSafe(path, circle_center_points_padding_1, length)) {
    return true;
  }

  // okay okay, the other padding
  std::vector<geometry_msgs::Point> circle_center_points_padding_2 = co_->circle_center_points;
  for (auto&& p : circle_center_points_padding_2) p.y -= co_->sbpl_footprint_padding;
  if (IsPathFootprintSafe(path, circle_center_points_padding_2, length)) {
    return true;
  }

  // at last...
  return false;
}

bool AStarController::IsGoalFootprintSafe(double goal_check_safe_dis, double goal_safe_dis_a, double goal_safe_dis_b, const geometry_msgs::PoseStamped& current_pose) {
  double goal_pose_dis = PoseStampedDistance(current_pose, planner_goal_); 
//  ROS_INFO("[Astar Controller] goal_pose_dis: %lf, th: %lf", goal_pose_dis, goal_check_safe_dis);
  if(fabs(goal_pose_dis) > goal_check_safe_dis)
    return true;
  
  double inscribed_radius, circumscribed_radius;
  std::vector<geometry_msgs::Point> footprint_spec = planner_costmap_ros_->getRobotFootprint();
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
  double yaw = tf::getYaw(planner_goal_.pose.orientation);
  double resolution = planner_costmap_ros_->getCostmap()->getResolution();
//  ROS_INFO("goal_x = %lf, goal_y = %lf, goal_yaw = %lf", planner_goal_.pose.position.x, planner_goal_.pose.position.y, tf::getYaw(planner_goal_.pose.orientation));
  std::vector<geometry_msgs::PoseStamped> path;
  geometry_msgs::PoseStamped p;
  int num_step = goal_safe_dis_a / resolution;
  for (int i = 1; i <= num_step; ++i) {
    p.pose.position.x = planner_goal_.pose.position.x + i * resolution * cos(yaw);
    p.pose.position.y = planner_goal_.pose.position.y + i * resolution * sin(yaw);
    p.pose.orientation = planner_goal_.pose.orientation;
    path.push_back(p);
//    ROS_WARN("check a: p_x = %lf, p_y = %lf, p_yaw = %lf", p.pose.position.x, p.pose.position.y, tf::getYaw(p.pose.orientation));
  }
  num_step = goal_safe_dis_b / resolution;
  for (int i = 1; i <= num_step; ++i) {
    p.pose.position.x = planner_goal_.pose.position.x - i * resolution * cos(yaw);
    p.pose.position.y = planner_goal_.pose.position.y - i * resolution * sin(yaw);
    p.pose.orientation = planner_goal_.pose.orientation;
    path.push_back(p);
//    ROS_WARN("check b: p_x = %lf, p_y = %lf, p_yaw = %lf", p.pose.position.x, p.pose.position.y, tf::getYaw(p.pose.orientation));
  }
  for (int i = 0; i < path.size(); ++i) {
    if (footprint_checker_->FootprintCost(path[i].pose.position.x, path[i].pose.position.y, yaw,
                                footprint_spec, inscribed_radius, circumscribed_radius) < 0){
      ROS_ERROR("[ASTAR CONTROLLER] check goal not safe!");
      return false;
    }
  }
  return true;
}

bool AStarController::NeedBackward(const geometry_msgs::PoseStamped& pose, double distance) {
  double yaw = tf::getYaw(pose.pose.orientation);
  double resolution = planner_costmap_ros_->getCostmap()->getResolution();
  int num_step = distance / resolution;

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
                                             co_->circle_center_points) < 0)
      return true;
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
  planner_costmap_ros_->getRobotPose(global_pose);
  geometry_msgs::PoseStamped current_position;
  tf::poseStampedTFToMsg(global_pose, current_position);

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

    ROS_DEBUG_NAMED("autoscrubber", "Got a new plan...swap pointers");

    // in case new plan has different rotate dir
    co_->fixpattern_local_planner->reset_planner();

    // do a pointer swap under mutex
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
      ROS_INFO("[ASTAR CONTROLLER] in CONTROLLING state");
/*      {      
        bool is_goal_safe = false;
        bool is_stop_here = false;
        ros::Rate check_rate(10);
        ros::Time check_end_time = ros::Time::now() + ros::Duration(co_->goal_safe_check_duration);
        while (ros::Time::now() < check_end_time) {
          if (IsGoalFootprintSafe(co_->goal_safe_check_dis, co_->goal_safe_dis_a, co_->goal_safe_dis_b, current_position)) {
            is_goal_safe = true;
            break;
          }
          else {
          //  PublishZeroVelocity();
            ResetState();
            is_goal_safe = false;
            is_stop_here = true;
          }
          check_rate.sleep();
        }
        if(is_goal_safe && is_stop_here){
          state_ = A_PLANNING;
          // enable the planner thread in case it isn't running on a clock
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();
          return false;
        }
        else if (!is_goal_safe){      

          ROS_ERROR("[ASTAR CONTROLLER] Check goal not safe, stop here!");
          ResetState();

          // disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          // TODO(chenkan): check if this is needed
          co_->fixpattern_local_planner->reset_planner();
          // Goal not reached, but we will stop and exit ExecuteCycle
          return true;
        }
      }
*/
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
            && PoseStampedDistance(current_position, planner_goal_) < 0.1) {
        // Sbpl Goal reached, and fix_pattern Goal reached, all path done 
          co_->fixpattern_path->fixpattern_path_reached_goal_ = false;
          sbpl_reached_goal_ = true;
				} else {
          sbpl_reached_goal_ = false;
        }
        // Goal reached, switch to fixpattern controller
        return true;
      }else
          sbpl_reached_goal_ = false;

      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        // get the current amcl pose of the robot, and prune fixpattern path
        tf::Stamped<tf::Pose> global_pose;
        if (controller_costmap_ros_->getRobotPose(global_pose)) {
          geometry_msgs::PoseStamped start;
          tf::poseStampedTFToMsg(global_pose, start);
          start = PoseStampedToGlobalFrame(start);
          astar_path_.Prune(fixpattern_path::GeometryPoseToPathPoint(start.pose), co_->max_offroad_dis);
          if(co_->fixpattern_local_planner->isPathRotateDone()) {
            astar_path_.PruneCornerOnStart();
          }
        } else {
          ROS_WARN("[MOVE BASE] Unable to get robot pose, unable to calculate highlight length");
        }
      }

      // check for an oscillation condition
      if (co_->oscillation_timeout > 0.0 &&
         last_oscillation_reset_ + ros::Duration(co_->oscillation_timeout) < ros::Time::now()) {
        PublishZeroVelocity();
        state_ = A_CLEARING;
        recovery_trigger_ = A_OSCILLATION_R;
      }

      if (!IsPoseFootprintSafe(co_->goal_safe_dis_a, co_->goal_safe_dis_b, planner_goal_)) {
        PublishZeroVelocity();
        state_ = A_CLEARING;
        recovery_trigger_ = A_GOALSAFE_R;
        ROS_ERROR("[ASTAR CONTROLLER] !IsPoseFootprintSafe, entering GOALSAFE_R");
        return false;
      }
      // check if need going back
      ;
      // check if footprint hits something alongside the road, in a limited distance
      if (HandleGoingBack(current_position) || !IsPathFootprintSafe(astar_path_, co_->front_safe_check_dis)) {
        // just restart the planner, and we'll not stop during this time
        // boost::unique_lock<boost::mutex> lock(planner_mutex_);
        // runPlanner_ = true;
        // planner_cond_.notify_one();
        // lock.unlock();

        PublishZeroVelocity();
        state_ = A_PLANNING;
        recovery_trigger_ = A_PLANNING_R;
        ROS_ERROR("[ASTAR CONTROLLER] HandleGoingBack or !IsPathFootprintSafe, entering A_PLANNING state");

        // continue the planner
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        return false;
      }

      if (!using_sbpl_directly_ &&
          (PoseStampedDistance(current_position, planner_goal_) <= co_->sbpl_max_distance ||
           astar_path_.Length() < 1.5)) {
        // just restart the planner, and we'll not stop during this time
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();
        ROS_INFO("[ASTAR CONTROLLER] distance to start point < 1.5, replan");
      }

      // if astar_path_ is not long enough and new plan has not arrived, just wait
      if (!using_sbpl_directly_ && astar_path_.Length() < 0.5) {
        PublishZeroVelocity();
        state_ = A_PLANNING;
        recovery_trigger_ = A_PLANNING_R;
        ROS_INFO("[ASTAR CONTROLLER] astar_path.Length() < 0.5, entering A_PLANNING state");

        // continue the planner
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        return false;
      }

      // check if need going back
//      HandleGoingBack(current_position);

      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        // we'll replan if path size is 0
        // if (astar_path_.path().size() <= 0) {
        //   state_ = A_PLANNING;
        //   recovery_trigger_ = A_PLANNING_R;
        //   ROS_ERROR("[MOVE BASE] fixpattern_path_ size is 0, CONTROLLING to PLANNING");
        //   PublishZeroVelocity();
        //   return false;
        // }

        // change highlight to 1.5 meters for sbpl path and 1.0 for points that around corner
        std::vector<fixpattern_path::PathPoint> path = astar_path_.path();
        for (auto&& p : path) p.highlight = 0.8;  // NOLINT
        for (auto it = path.begin(); it != path.end(); ++it) {
          if (it->corner_struct.corner_point) {
            double dis_accu = 0.0;
            for (auto i = it; i >= path.begin(); --i) {
              dis_accu += it->DistanceToPoint(*i);
              if (dis_accu > 0.2) break;
              i->highlight = 0.4;
            }
            dis_accu = 0.0;
            for (auto i = it; i < path.end(); ++i) {
               dis_accu += it->DistanceToPoint(*i);
               if (dis_accu > 0.2) break;
               i->highlight = 0.4;
            }
          }
        }
        // change highlight of points before end point
        double dis_accu = 0.0;
        if (path.size() > 0) path.back().highlight = 0.01;
        for (int i = path.size() - 2; i >= 0; --i) {
          dis_accu += path.at(i).DistanceToPoint(path.at(i + 1));
          if (path.at(i).highlight <= dis_accu) break;
          path.at(i).highlight = dis_accu;
        }
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

        // only after computeVelocityCommands is called can need_backward() work
        // if (co_->fixpattern_local_planner->getPlanner()->need_backward()) {
        //   ROS_INFO("[ASTAR CONTROLLER] going back");

        //   last_valid_control_ = ros::Time::now();

        //   cmd_vel.linear.x = -0.1;
        //   cmd_vel.angular.z = 0.0;
        //   co_->vel_pub->publish(cmd_vel);

        //   break;
        // }

        if (local_planner_ret) {
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
            state_ = A_CLEARING;
            recovery_trigger_ = A_CONTROLLING_R;
          } else {
            // otherwise, if we can't find a valid control, we'll retry, until
            // reach controller_patience
            ROS_INFO("[ASTAR CONTROLLER] wait for a valid control");
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

    // we'll try to launch recovery behaviors
    case A_CLEARING:
      ROS_INFO("[ASTAR CONTROLLER] in CLEARING state");
      // we'll invoke recovery behavior
      if (recovery_trigger_ == A_GOALSAFE_R) {
        ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
        ros::Rate r(10);
        bool goal_safe = false;
        while (ros::Time::now() < end_time) {
          if (IsPoseFootprintSafe(co_->goal_safe_dis_a, co_->goal_safe_dis_b, planner_goal_)) {
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
        }
      } else if (recovery_trigger_ == A_PLANNING_R) {
        // disable the planner thread, otherwise costmap will be locked
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        RotateRecovery();
      }

      {
        int begin_index = 0;
        if (recovery_trigger_ == A_PLANNING_R) {
          // if already sbpl_broader_, cut some points
          if (sbpl_broader_) {
            unsigned int size = co_->fixpattern_path->path().size();
            if (size > 0) size -= 1;
            begin_index = std::min(planner_goal_index_ + 50, size);
          } else {
            ROS_INFO("[ASTAR CONTROLLER] set sbpl_broader_ to true");
            sbpl_broader_ = true;
          }
        }
        // get a new astar goal
        if (!GetAStarGoal(begin_index)) {
          // no point is safe, so terminate the path
          ResetState();
          // disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_WARN("[ASTAR CONTROLLER] GetAStarGoal failed, switch controller");
          return true;
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

  // search planner goal from start
  planner_goal_index_ = 0;

  // reset some variables
  using_sbpl_directly_ = false;
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

double GetTimeInSeconds() {
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec + 0.000001 * t.tv_usec;
}

bool AStarController::GetAStarGoal(int begin_index) {
  double start = GetTimeInSeconds();

  if (begin_index == 0)
    begin_index = planner_goal_index_;
  else
    ROS_INFO("[ASTAR CONTROLLER] search astar goal from: %d", begin_index);

  bool cross_obstacle = false;
  double dis_accu = 0.0;
  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  // we don't want cross_obstacle take effect when planning to start point
  // if (path.size() == co_->fixpattern_path->total_point_count()) {
  //   cross_obstacle = true;
  // }

  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for (it = path.begin() + begin_index; it != path.end(); it += 5) {
    if (!IsPoseFootprintSafe(co_->goal_safe_dis_a + 0.5, co_->goal_safe_dis_b, *it)) {
      cross_obstacle = true;
      continue;
    }
    if (it >= path.begin() + 5) dis_accu += PoseStampedDistance(*it, *(it - 5));
    // we must enforce cross obstacle within front_safe_check_dis range
    if (!cross_obstacle && dis_accu <= co_->front_safe_check_dis) continue;
    break;
  }
  if (!cross_obstacle) it = path.begin() + begin_index;
  if (it >= path.end()) {
    ROS_INFO("[ASTAR CONTROLLER] GetAStarGoal cost: %lf secs", GetTimeInSeconds() - start);
    return false;
  }
  planner_goal_ = *it;
  planner_goal_.header.frame_id = co_->global_frame;
  planner_goal_index_ = it - path.begin();
  ROS_INFO("[ASTAR CONTROLLER] GetAStarGoal cost: %lf secs", GetTimeInSeconds() - start);
  ROS_INFO("[ASTAR CONTROLLER] planner_goal_index_: %d", planner_goal_index_);
  return true;
}

bool AStarController::HandleGoingBack(geometry_msgs::PoseStamped current_position) {
  geometry_msgs::Twist cmd_vel;

  // check if need backward
  ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
  bool need_backward = true;
  ros::Rate r(10);
  while (ros::Time::now() < end_time) {
    if (!NeedBackward(current_position, 0.15)) {
      need_backward = false;
      break;
    }
    // stop first, and set last_valid_control_
    PublishZeroVelocity();
    last_valid_control_ = ros::Time::now();
    r.sleep();
  }
  ros::Rate control_rate(co_->controller_frequency);
  while (need_backward && NeedBackward(current_position, 0.15) && CanBackward(0.25)) {
    ROS_INFO("[ASTAR CONTROLLER] going back");
    // get curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    tf::poseStampedTFToMsg(global_pose, current_position);

    // make sure that we send the velocity command to the base
    cmd_vel.linear.x = -0.1;
    cmd_vel.angular.z = 0.0;
    co_->vel_pub->publish(cmd_vel);

    last_valid_control_ = ros::Time::now();
    control_rate.sleep();
  }
  return need_backward;
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
  while (fabs(angle_diff) > 0.1 && CanRotate(x, y, yaw, angle_diff > 0 ? 1 : -1)) {
    cmd_vel.angular.z = angle_diff > 0 ? 0.1 : -0.1;
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
      ROS_INFO("[ASTAR CONTROLLER] CanBackward: false");
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
    rotate_recovery_dir_ = 0;

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
  for (int i = 1; i <= num_step; ++i) {
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
  if (!CanForward(0.35)) return false;

  double forward_time = distance / 0.1;
  ros::Time end_time = ros::Time::now() + ros::Duration(forward_time);

  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.1;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  ros::Rate r(co_->controller_frequency);
  while (ros::Time::now() < end_time && CanForward(0.35)) {
    rotate_recovery_dir_ = 0;

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
