/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file fixpattern_controller.cc
 * @brief fixpattern controller
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-24
 */

#include "autoscrubber/fixpattern_controller.h"
#include <path_recorder/path_recorder.h>
#include <nav_msgs/Path.h>

namespace autoscrubber {

FixPatternController::FixPatternController(tf::TransformListener* tf,
                                           costmap_2d::Costmap2DROS* planner_costmap_ros,
                                           costmap_2d::Costmap2DROS* controller_costmap_ros)
    : tf_(*tf),
      planner_costmap_ros_(planner_costmap_ros), controller_costmap_ros_(controller_costmap_ros) {
  // create footprint_checker_
  footprint_checker_ = new autoscrubber::FootprintChecker(*controller_costmap_ros_->getCostmap());

  // initially, we'll need to make a plan
  state_ = F_CONTROLLING;

  // setup global_plan publisher
  ros::NodeHandle fixpattern_nh("~/fixpattern_global_planner");
  fixpattern_pub_ = fixpattern_nh.advertise<nav_msgs::Path>("plan", 1);

  // we'll start executing recovery behaviors at the beginning of our list
  recovery_trigger_ = F_CONTROLLING_R;

  // we'll not switch controller when start
  switch_controller_ = false;
	first_run_flag_ = true;
}

FixPatternController::~FixPatternController() {
  delete footprint_checker_;
}

void FixPatternController::PublishZeroVelocity() {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  co_->vel_pub->publish(cmd_vel);
}

bool FixPatternController::Control(BaseControlOption* option, ControlEnvironment* environment) {
  co_ = reinterpret_cast<FixPatternControlOption*>(option);
  env_ = environment;
	planner_goal_.pose = co_->global_planner_goal->pose;
  // reset fixpattern_local_planner
  co_->fixpattern_local_planner->reset_planner();
	ROS_INFO("[FIXPATTERN_PATH_CTRL] Switch to Fixpattern_Path Controller!");
  ros::Rate r(co_->controller_frequency);

  // we want to make sure that we reset the last time we had a valid plan and control
  last_valid_control_ = ros::Time::now();
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
      ResetState();

      // reset fixpattern_local_planner
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
    if (r.cycleTime() > ros::Duration(1 / co_->controller_frequency) && state_ == F_CONTROLLING)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", co_->controller_frequency, r.cycleTime().toSec());
  }

  // if !n.ok(), we don't want to switch controllers
  return true;
}

double FixPatternController::PoseStampedDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

bool FixPatternController::ExecuteCycle() {
  // we need to be able to publish velocity commands
  geometry_msgs::Twist cmd_vel;

  // get our curent position
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
    return false;
  }

  // sync path from file when start, publish path each cycle
  MakePlan();

  // the move_base state machine, handles the control logic for navigation
  switch (state_) {
    // if we're controlling, we'll attempt to find valid velocity commands
    case F_CONTROLLING:
      ROS_INFO("[FIXPATTERN CONTROLLER] in CONTROLLING state");
      ROS_DEBUG_NAMED("autoscrubber", "In controlling state.");

			if(first_run_flag_) {
				first_run_flag_ = false;
        switch_controller_ = true;
				co_->fixpattern_path->fixpattern_path_reached_goal_ = false;
				co_->fixpattern_path->fixpattern_path_first_run_ = true;
				return true;
			} else
				co_->fixpattern_path->fixpattern_path_first_run_ = false;

			if(co_->fixpattern_path->fixpattern_path_start_updated_) {
				co_->fixpattern_path->fixpattern_path_start_updated_ = false;
				//std::vector<fixpattern_path::PathPoint> pathpoint = co_->fixpattern_path->path(); 
				//path_recorder::PathRecorder recorder;
				//recorder.CalculatePath(&pathpoint);
				//co_->fixpattern_path->set_path(pathpoint, true);
			}
      // check to see if we've reached our goal
      if (co_->fixpattern_local_planner->isGoalReached()) {
        ROS_INFO("[FIXPATTERN CONTROLLER] fixpattern goal reached");
        ROS_DEBUG_NAMED("autoscrubber", "Goal reached!");
        ResetState();

        // reset fixpattern_local_planner
        co_->fixpattern_local_planner->reset_planner();	
				co_->fixpattern_path->fixpattern_path_reached_goal_ = true;
				first_run_flag_ = true;
				
        // we need to notify fixpattern_path
        co_->fixpattern_path->FinishPath();
        // Goal reached
				if(co_->fixpattern_path->fixpattern_path_goal_updated_) {
					co_->fixpattern_path->fixpattern_path_goal_updated_ = false;
				  switch_controller_ = false;
				} else  {
				  switch_controller_ = true;
				}
				return true;  //(lee)
      }
			if(co_->fixpattern_path->fixpattern_path_reached_goal_)	
				co_->fixpattern_path->fixpattern_path_reached_goal_ = false;
      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        // we'll Prune the path first as we don't want to navigate back when
        // trigger front_safe while robot still moves
        // get current pose of the vehicle && prune fixpattern path
        tf::Stamped<tf::Pose> global_pose;
        if (controller_costmap_ros_->getRobotPose(global_pose) ||
            planner_costmap_ros_->getRobotPose(global_pose)) {
          geometry_msgs::PoseStamped start;
          tf::poseStampedTFToMsg(global_pose, start);
          start = PoseStampedToGlobalFrame(start);
          co_->fixpattern_path->Prune(fixpattern_path::GeometryPoseToPathPoint(start.pose), co_->max_offroad_dis);
        } else {
          ROS_WARN("[MOVE BASE] Unable to get robot pose, unable to calculate highlight length");
        }
      }

      // check for an oscillation condition
      if (co_->oscillation_timeout > 0.0 &&
         last_oscillation_reset_ + ros::Duration(co_->oscillation_timeout) < ros::Time::now()) {
        ROS_INFO("[FIXPATTERN CONTROLLER] oscillation to CLEARING");
        PublishZeroVelocity();
        state_ = F_CLEARING;
        recovery_trigger_ = F_OSCILLATION_R;
      }

      // check front safe. If not, we'll go to recovery
      if (!IsFrontSafe()) {
        ROS_INFO("[FIXPATTERN CONTROLLER] !IsFrontSafe to CLEARING");
        PublishZeroVelocity();
        state_ = F_CLEARING;
        recovery_trigger_ = F_FRONTSAFE_R;
        break;
      }

      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        if (!co_->fixpattern_local_planner->setPlan(co_->fixpattern_path->path(), co_->global_frame)) {
          // ABORT and SHUTDOWN COSTMAPS
          ROS_ERROR("Failed to pass global plan to the controller, aborting.");
          ResetState();

          return true;
        }

        if (co_->fixpattern_local_planner->computeVelocityCommands(fixpattern_local_planner::TRAJECTORY_PLANNER, &cmd_vel)) {
          ROS_DEBUG_NAMED("autoscrubber", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
          last_valid_control_ = ros::Time::now();
          // make sure that we send the velocity command to the base
          co_->vel_pub->publish(cmd_vel);

          // notify chassis to launch scrubber
          env_->launch_scrubber = true;
        } else {
          ROS_DEBUG_NAMED("autoscrubber", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(co_->controller_patience);

          // check if we've tried to find a valid control for longer than our time limit
          if (ros::Time::now() > attempt_end) {
            ROS_INFO("[FIXPATTERN CONTROLLER] CONTROLLING exceeds attempt_end");
            // we'll move into our obstacle clearing mode
            PublishZeroVelocity();
            state_ = F_CLEARING;
            recovery_trigger_ = F_CONTROLLING_R;
          } else {
            // otherwise, if we can't find a valid control, we'll retry, until
            ROS_INFO("[FIXPATTERN CONTROLLER] wait for a valid control");
            // reach controller_patience
            state_ = F_CONTROLLING;
            PublishZeroVelocity();
          }
        }
      }

      break;

      // we'll try to clear out space with any user-provided recovery behaviors
    case F_CLEARING:
      ROS_INFO("[FIXPATTERN CONTROLLER] in CLEARING state, recovery_trigger_: %d", static_cast<int>(recovery_trigger_));
      ROS_DEBUG_NAMED("autoscrubber", "In clearing/recovery state");
      // we'll stop first, and then see if need to avoid obstacle
      if (recovery_trigger_ == F_FRONTSAFE_R) {
        ros::Time end_time = ros::Time::now() + ros::Duration(co_->stop_duration);
        ros::Rate r(10);
        bool front_safe = false;
        while (ros::Time::now() < end_time) {
          if (IsFrontSafe()) {
            front_safe = true;
            break;
          }
          r.sleep();
        }
        if (front_safe) {
          // we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now();

          state_ = F_CONTROLLING;
          break;
        }
      }
      // we are going to switch controller
      ResetState();
      switch_controller_ = true;
      return true;
    default:
      ROS_ERROR("This case should never be reached, something is wrong, aborting");
      ResetState();
      // Reached a case that should not be hit in autoscrubber. This is a bug, please report it.
      return true;
  }

  // we aren't done yet
  return false;
}

void FixPatternController::ResetState() {
  // Reset statemachine
  state_ = F_CONTROLLING;
  recovery_trigger_ = F_CONTROLLING_R;
  PublishZeroVelocity();
}

geometry_msgs::PoseStamped FixPatternController::PoseStampedToGlobalFrame(const geometry_msgs::PoseStamped& pose_msg) {
  std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
  tf::Stamped<tf::Pose> goal_pose, global_pose;
  poseStampedMsgToTF(pose_msg, goal_pose);

  // just get the latest available transform... for accuracy they should send
  // goals in the frame of the planner
  goal_pose.stamp_ = ros::Time();

  try {
    co_->tf->transformPose(global_frame, goal_pose, global_pose);
  } catch(tf::TransformException& ex) {
    ROS_WARN("[Fixpattern_path]Failed to transform the goal pose from %s into the %s frame: %s",
             goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
    return pose_msg;
  }

  geometry_msgs::PoseStamped global_pose_msg;
  tf::poseStampedTFToMsg(global_pose, global_pose_msg);
  return global_pose_msg;
}

bool FixPatternController::IsFootprintSafe(const geometry_msgs::Pose& pose) {
  double inscribed_radius, circumscribed_radius;
  std::vector<geometry_msgs::Point> footprint_spec = controller_costmap_ros_->getRobotFootprint();
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
  double yaw = tf::getYaw(pose.orientation);
  return footprint_checker_->FootprintCost(
      pose.position.x, pose.position.y, yaw, footprint_spec, inscribed_radius, circumscribed_radius) >= 0;
}

bool FixPatternController::IsFrontSafe() {
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

  std::vector<fixpattern_path::PathPoint> path = co_->fixpattern_path->path();

  if (path.size() <= 1) {
    return true;
  }
  double total_dis_accum = 0.0;
  for (auto it = path.begin() + 1; it != path.end(); ++it) {
    geometry_msgs::PoseStamped geometry_point = fixpattern_path::PathPointToGeometryPoseStamped(*it);
    geometry_msgs::PoseStamped pre_geometry_point = fixpattern_path::PathPointToGeometryPoseStamped(*(it - 1));

    geometry_point.header.frame_id = co_->global_frame;
    geometry_msgs::PoseStamped pose_tmp = PoseStampedToGlobalFrame(geometry_point);

    unsigned int cell_x, cell_y;
    if (!controller_costmap_ros_->getCostmap()->worldToMap(
            pose_tmp.pose.position.x, pose_tmp.pose.position.y, cell_x, cell_y)) {
      return true;
    }
    total_dis_accum += PoseStampedDistance(pre_geometry_point, geometry_point);
    if (!IsFootprintSafe(*it)) {
      return false;
    }
    // only check front_safe_check_dis_ meters
    if (total_dis_accum >= co_->front_safe_check_dis) {
      break;
    }
  }
  return true;
}

void FixPatternController::PublishPlan(const ros::Publisher& pub, const std::vector<geometry_msgs::PoseStamped>& plan) {
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

void FixPatternController::MakePlan() {
  if (!co_->fixpattern_path->IsRunning() || co_->fixpattern_path->fixpattern_path_reached_goal_) {
    std::string file_name = "/home/gaussian/record.path";
    const char* runtime_dir = ::getenv("GAUSSIAN_RUNTIME_DIR");
    if (runtime_dir != NULL) {
      file_name = std::string(runtime_dir) + "/run.path";
    }
		
    co_->fixpattern_path->SyncFromFile(file_name.c_str());
    ros::NodeHandle nh;
    bool first_start = true;
    if (nh.hasParam("/first_start")) {
      nh.getParam("/first_start", first_start);
    }
    if (!first_start) {
      int fixpattern_index = 0;
      nh.getParam("/fixpattern_index", fixpattern_index);
      if (static_cast<unsigned int>(fixpattern_index) < co_->fixpattern_path->GeometryPath().size() - 8) {
        co_->fixpattern_path->EraseToIndex(static_cast<unsigned int>(fixpattern_index));
      }
    }
    ROS_INFO("[GLOBAL PLANNER] path synced from file: %s", file_name.c_str());
  }

	if(first_run_flag_) {
		tf::Stamped<tf::Pose> global_pose;
		if (controller_costmap_ros_->getRobotPose(global_pose)/*|| planner_costmap_ros_->getRobotPose(global_pose)*/) {
				geometry_msgs::PoseStamped start;
				tf::poseStampedTFToMsg(global_pose, start);
				start.header.frame_id = co_->global_frame;
				start = PoseStampedToGlobalFrame(start);
				co_->fixpattern_path->PruneFromStartToGoal(fixpattern_path::GeometryPoseToPathPoint(start.pose), fixpattern_path::GeometryPoseToPathPoint(planner_goal_.pose));
		} else {
				ROS_WARN("[MOVE BASE] Unable to get robot pose, unable to calculate highlight length");
			}
	}

  std::vector<geometry_msgs::PoseStamped> plan = co_->fixpattern_path->GeometryPath();
  for (auto&& p : plan) {  // NOLINT
    p.header.frame_id = co_->global_frame;
    p.header.stamp = ros::Time::now();
  }

  PublishPlan(fixpattern_pub_, plan);
}

};  // namespace autoscrubber
