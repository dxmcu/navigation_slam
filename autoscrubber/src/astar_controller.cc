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
      planner_goal_index_(0), runPlanner_(false), new_global_plan_(false), using_sbpl_directly_(false) {
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
  tf::Stamped<tf::Pose> global_pose;
  if (!planner_costmap_ros_->getRobotPose(global_pose)) {
    ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
    return false;
  }

  geometry_msgs::PoseStamped start;
  tf::poseStampedTFToMsg(global_pose, start);
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
    astar_path_.set_path(path, false);
  } else if (PoseStampedDistance(start, goal) <= co_->sbpl_max_distance) {
    // too short, use sbpl directly
    ROS_INFO("[ASTAR CONTROLLER] too short, use sbpl directly");
    using_sbpl_directly_ = true;
    // if the planner fails or returns a zero length plan, planning failed
    if (!co_->sbpl_global_planner->makePlan(start, goal, *plan, astar_path_) || plan->empty()) {
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
    if (!co_->sbpl_global_planner->makePlan(start, plan->at(i), *plan, astar_path_) || plan->empty()) {
      ROS_ERROR("[ASTAR CONTROLLER] sbpl failed to find a plan to point (%.2f, %.2f)", plan->at(i).pose.position.x, plan->at(i).pose.position.y);
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

		bool gotPlan = false;
		// get the starting pose of the robot
		tf::Stamped<tf::Pose> global_pose;
		if (!planner_costmap_ros_->getRobotPose(global_pose)) {
			ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
		}else {
			geometry_msgs::PoseStamped start;
			tf::poseStampedTFToMsg(global_pose, start);

			ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

			// run planner
			planner_plan_->clear();
			gotPlan = n.ok() && MakePlan(start, temp_goal, planner_plan_);
		}
    if (gotPlan) {
      ROS_INFO("[ASTAR CONTROLLER] Got Plan with %zu points!", planner_plan_->size());
      // pointer swap the plans under mutex (the controller will pull from latest_plan_)
      std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

      lock.lock();
      planner_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      last_valid_plan_ = ros::Time::now();
      new_global_plan_ = true;

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
        // we'll move into our obstacle clearing mode
        state_ = A_CLEARING;
        PublishZeroVelocity();
        recovery_trigger_ = A_PLANNING_R;
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

std::vector<fixpattern_path::PathPoint> AStarController::CalculateStartCurvePath(const std::vector<fixpattern_path::PathPoint>& astar_path) {
	std::vector<fixpattern_path::PathPoint> path = astar_path;
	std::vector<fixpattern_path::PathPoint> fix_path = co_->fixpattern_path->path();
	unsigned int index = CalculatePathLengthIndex(fix_path, 0.5, 1);
	path.insert(path.end(), fix_path.begin(), fix_path.begin() + index);

	path_recorder::PathRecorder recorder;
	recorder.CalculatePath(&path);
	path.insert(path.end(), fix_path.begin() + index + 1, fix_path.end());
	co_->fixpattern_path->set_fix_path(path);
}

std::vector<fixpattern_path::PathPoint> AStarController::CalculateGoalCurvePath(const std::vector<fixpattern_path::PathPoint>& astar_path) {
	std::vector<fixpattern_path::PathPoint> path = astar_path;
	std::vector<fixpattern_path::PathPoint> fix_path = co_->fixpattern_path->path();
	unsigned int index = CalculatePathLengthIndex(fix_path, 0.5, 0);
	path.insert(path.begin(), fix_path.begin() + index, fix_path.end());

	path_recorder::PathRecorder recorder;
	recorder.CalculatePath(&path);
	path.insert(path.begin(), fix_path.begin(), fix_path.begin() + index - 1);
	co_->fixpattern_path->set_fix_path(path);
}

bool AStarController::Control(BaseControlOption* option, ControlEnvironment* environment) {
  co_ = reinterpret_cast<AStarControlOption*>(option);
  env_ = environment;
	ROS_INFO("[ASTAR_CTRL] Switch to Astar Controller!");
	// first run: just get Astar Path, and insert to fixpattern_path, then switch controllor 
	if(co_->fixpattern_path->fixpattern_path_first_run_) {
		co_->fixpattern_path->fixpattern_path_first_run_ = false;
		geometry_msgs::PoseStamped start;
		geometry_msgs::PoseStamped goal;
		std::vector<geometry_msgs::PoseStamped> path;
		tf::Stamped<tf::Pose> global_pose;
		bool gotPlan = false;
		if (!planner_costmap_ros_->getRobotPose(global_pose)) {
			ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
		} else {
			tf::poseStampedTFToMsg(global_pose, start);
			path = co_->fixpattern_path->GeometryPath();
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
			path = co_->fixpattern_path->GeometryPath();
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
			}

				return false;
		} else {
			path = co_->fixpattern_path->GeometryPath();
			planner_goal_.pose = path.front().pose;
			planner_goal_.header.frame_id = co_->global_frame;
			co_->fixpattern_local_planner->reset_planner();
		}
	} else if(!co_->fixpattern_path->fixpattern_path_goal_updated_ 
			&& co_->fixpattern_path->fixpattern_path_reached_goal_) { 
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
  double inscribed_radius, circumscribed_radius;
  std::vector<geometry_msgs::Point> footprint_spec = planner_costmap_ros_->getRobotFootprint();
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);

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
  for (int i = goal_index - 1; i >= 0; --i) {
    fix_path[i].header.frame_id = co_->global_frame;
    geometry_msgs::PoseStamped pose_tmp = PoseStampedToLocalFrame(fix_path[i]);
    double x = pose_tmp.pose.position.x;
    double y = pose_tmp.pose.position.y;
    double yaw = tf::getYaw(pose_tmp.pose.orientation);
    if (footprint_checker_->FootprintCost(x, y, yaw, footprint_spec, inscribed_radius, circumscribed_radius) < 0)
      return false;
    free_dis_a += PoseStampedDistance(fix_path[i], fix_path[i + 1]);
    if (free_dis_a >= goal_safe_dis_a) {
      break;
    }
  }
  double free_dis_b = 0.0;
  for (int i = goal_index + 1; i < fix_path.size(); ++i) {
    fix_path[i].header.frame_id = co_->global_frame;
    geometry_msgs::PoseStamped pose_tmp = PoseStampedToLocalFrame(fix_path[i]);
    double x = pose_tmp.pose.position.x;
    double y = pose_tmp.pose.position.y;
    double yaw = tf::getYaw(pose_tmp.pose.orientation);
    if (footprint_checker_->FootprintCost(x, y, yaw, footprint_spec, inscribed_radius, circumscribed_radius) < 0)
      return false;
    free_dis_b += PoseStampedDistance(fix_path[i], fix_path[i - 1]);
    if (free_dis_b >= goal_safe_dis_b) {
      break;
    }
  }
  return true;
}

bool AStarController::IsPathFootprintSafe(const fixpattern_path::Path& fix_path, double length) {
  double accu_dis = 0.0;
  double inscribed_radius, circumscribed_radius;
  std::vector<geometry_msgs::Point> footprint_spec = planner_costmap_ros_->getRobotFootprint();
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
  std::vector<geometry_msgs::PoseStamped> path = fix_path.GeometryPath();
  for (int i = 0; i < path.size(); ++i) {
    double yaw = tf::getYaw(path[i].pose.orientation);
    if (footprint_checker_->FootprintCost(path[i].pose.position.x, path[i].pose.position.y, yaw,
                                          footprint_spec, inscribed_radius, circumscribed_radius) < 0) {
      return false;
    }
    if (i != 0)
      accu_dis += PoseStampedDistance(path[i], path[i - 1]);
    if (accu_dis >= length) return true;
  }
  return true;
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
  double inscribed_radius, circumscribed_radius;
  std::vector<geometry_msgs::Point> footprint_spec = planner_costmap_ros_->getRobotFootprint();
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
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
    if (footprint_checker_->FootprintCost(path[i].pose.position.x, path[i].pose.position.y, yaw,
                                          footprint_spec, inscribed_radius, circumscribed_radius) < 0)
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
  /*    {			
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
						&& !co_->fixpattern_path->fixpattern_path_goal_updated_)
				//if(co_->fixpattern_reached_goal[0])
					sbpl_reached_goal_ = true;
				else
					sbpl_reached_goal_ = false;
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
        ROS_INFO("[ASTAR CONTROLLER] !IsPoseFootprintSafe, entering GOALSAFE_R");
        return false;
      }

      // check if footprint hits something alongside the road, in a limited distance
      if (!IsPathFootprintSafe(astar_path_, co_->front_safe_check_dis)) {
        // just restart the planner, and we'll not stop during this time
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();
        ROS_INFO("[ASTAR CONTROLLER] !IsPathFootprintSafe, replan");
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
      if (!using_sbpl_directly_ && astar_path_.Length() < 0.3) {
        ROS_INFO("[ASTAR CONTROLLER] waitting for new sbpl plan");
        return false;
      }

      // check if need going back
      HandleGoingBack(current_position);

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
        for (auto&& p : path) p.highlight = 1.0;  // NOLINT
        for (auto it = path.begin(); it != path.end(); ++it) {
          if (it->corner_struct.corner_point) {
            double dis_accu = 0.0;
            for (auto i = it; i >= path.begin(); --i) {
              dis_accu += it->DistanceToPoint(*i);
              if (dis_accu > 0.2) break;
              i->highlight = 0.5;
            }
            dis_accu = 0.0;
            for (auto i = it; i < path.end(); ++i) {
               dis_accu += it->DistanceToPoint(*i);
               if (dis_accu > 0.2) break;
               i->highlight = 0.5;
            }
          }
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

        if (co_->fixpattern_local_planner->computeVelocityCommands(fixpattern_local_planner::TRAJECTORY_PLANNER, &cmd_vel)) {
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
            // otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            state_ = A_PLANNING;
            PublishZeroVelocity();

            // enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
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
          r.sleep();
        }
        if (goal_safe) {
          // we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          state_ = A_PLANNING;
          recovery_trigger_ = A_PLANNING_R;
          break;
        }
      }

      {
        // get a new astar goal
        if (!GetAStarGoal()) {
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

bool AStarController::GetAStarGoal() {
  std::vector<geometry_msgs::PoseStamped> path = co_->fixpattern_path->GeometryPath();
  std::vector<geometry_msgs::PoseStamped>::iterator it;
  for (it = path.begin() + planner_goal_index_; it != path.end(); ++it) {
    if (!IsPoseFootprintSafe(co_->goal_safe_dis_a + 0.5, co_->goal_safe_dis_b, *it)) continue;
    break;
  }
  if (it == path.end()) return false;
  planner_goal_ = *it;
  planner_goal_.header.frame_id = co_->global_frame;
  planner_goal_index_ = it - path.begin();
  return true;
}

void AStarController::HandleGoingBack(geometry_msgs::PoseStamped current_position) {
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
    r.sleep();
  }
  ros::Rate control_rate(co_->controller_frequency);
  while (need_backward && NeedBackward(current_position, 0.25)) {
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
}

};  // namespace autoscrubber
