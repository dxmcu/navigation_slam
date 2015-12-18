/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file trajectory_planner.cpp
 * @brief trajectory planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-11-05
 */

#include <fixpattern_local_planner/trajectory_planner.h>
#include <costmap_2d/footprint.h>
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>
#include <ros/console.h>
#include <fixpattern_path/path.h>
#include <math.h>

#include <string>
#include <sstream>
#include <queue>
#include <vector>
#include <algorithm>

namespace fixpattern_local_planner {

void TrajectoryPlanner::reconfigure(BaseLocalPlannerConfig &cfg) {
  BaseLocalPlannerConfig config(cfg);

  boost::mutex::scoped_lock l(configuration_mutex_);

  acc_lim_x_ = config.acc_lim_x;
  acc_lim_y_ = config.acc_lim_y;
  acc_lim_theta_ = config.acc_lim_theta;

  max_vel_x_ = config.max_vel_x;
  min_vel_x_ = config.min_vel_x;

  max_vel_th_ = config.max_vel_theta;
  min_vel_th_ = config.min_vel_theta;
  min_in_place_vel_th_ = config.min_in_place_vel_theta;

  num_calc_footprint_cost_ = config.num_calc_footprint_cost;

  sim_time_ = config.sim_time;
  sim_granularity_ = config.sim_granularity;
  angular_sim_granularity_ = config.angular_sim_granularity;
  front_safe_sim_time_ = config.front_safe_sim_time;
  front_safe_sim_granularity_ = config.front_safe_sim_granularity;

  pdist_scale_ = config.pdist_scale;
  gdist_scale_ = config.gdist_scale;
  occdist_scale_ = config.occdist_scale;

  if (meter_scoring_) {
    // if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
    double resolution = costmap_.getResolution();
    gdist_scale_ *= resolution;
    pdist_scale_ *= resolution;
    occdist_scale_ *= resolution;
  }

  oscillation_reset_dist_ = config.oscillation_reset_dist;
  escape_reset_dist_ = config.escape_reset_dist;
  escape_reset_theta_ = config.escape_reset_theta;

  vx_samples_ = config.vx_samples;
  vtheta_samples_ = config.vtheta_samples;

  if (vx_samples_ <= 0) {
    config.vx_samples = 1;
    vx_samples_ = config.vx_samples;
    ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
  }
  if (vtheta_samples_ <= 0) {
    config.vtheta_samples = 1;
    vtheta_samples_ = config.vtheta_samples;
    ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
  }

  heading_lookahead_ = config.heading_lookahead;

  holonomic_robot_ = config.holonomic_robot;

  backup_vel_ = config.escape_vel;

  dwa_ = config.dwa;

  heading_scoring_ = config.heading_scoring;
  heading_scoring_timestep_ = config.heading_scoring_timestep;

  simple_attractor_ = config.simple_attractor;

  // y-vels
  std::string y_string = config.y_vels;
  std::vector<std::string> y_strs;
  boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

  std::vector<double>y_vels;
  for (std::vector<std::string>::iterator it = y_strs.begin(); it != y_strs.end(); ++it) {
    std::istringstream iss(*it);
    double temp;
    iss >> temp;
    y_vels.push_back(temp);
    // ROS_INFO("Adding y_vel: %e", temp);
  }

  y_vels_ = y_vels;
}

TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
                                     const costmap_2d::Costmap2D& costmap,
                                     std::vector<geometry_msgs::Point> footprint_spec,
                                     double acc_lim_x, double acc_lim_y, double acc_lim_theta,
                                     int num_calc_footprint_cost, int trajectory_range_check_obstacle_avoidance,
                                     int avoid_obstacle_traj_num,
                                     double sim_time, double sim_granularity,
                                     double front_safe_sim_time, double front_safe_sim_granularity,
                                     int vx_samples, int vtheta_samples,
                                     double pdist_scale, double gdist_scale, double occdist_scale,
                                     double heading_lookahead, double oscillation_reset_dist,
                                     double escape_reset_dist, double escape_reset_theta,
                                     bool holonomic_robot,
                                     double max_vel_x, double min_vel_x,
                                     double max_vel_th, double min_vel_th, double min_in_place_vel_th,
                                     double backup_vel,
                                     bool dwa, bool heading_scoring, double heading_scoring_timestep, bool meter_scoring, bool simple_attractor,
                                     std::vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
  : costmap_(costmap),
    world_model_(world_model), footprint_spec_(footprint_spec),
    num_calc_footprint_cost_(num_calc_footprint_cost),
    trajectory_range_check_obstacle_avoidance_(trajectory_range_check_obstacle_avoidance),
    avoid_obstacle_traj_num_(avoid_obstacle_traj_num),
    sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
    front_safe_sim_time_(front_safe_sim_time), front_safe_sim_granularity_(front_safe_sim_granularity),
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), occdist_scale_(occdist_scale),
    acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
    prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead),
    oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
    escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
    max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
    backup_vel_(backup_vel),
    dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
    simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period) {
  if (meter_scoring) {
    ROS_INFO("[FIXPATTERN LOCAL PLANNER] meter_scoring set to true");
  } else {
    ROS_INFO("[FIXPATTERN LOCAL PLANNER] meter_scoring set to false");
  }
  final_goal_position_valid_ = false;

  costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
}

TrajectoryPlanner::~TrajectoryPlanner() { }

void TrajectoryPlanner::CalculatePathCost(
    double x, double y, double theta,
    double vx, double vy, double vtheta,
    double vx_samp, double vy_samp, double vtheta_samp,
    double acc_x, double acc_y, double acc_theta,
    double impossible_cost,
    Trajectory& traj, double sim_time) {

  // make sure the configuration doesn't change mid run
  boost::mutex::scoped_lock l(configuration_mutex_);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i, vy_i, vtheta_i;

  vx_i = vx;
  vy_i = vy;
  vtheta_i = vtheta;

  // discard trajectory that is circle
  if (fabs(vtheta_samp) > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
    traj.cost_ = DBL_MAX;
    return;
  }

  double sim_granularity = sim_time / sim_time_ * sim_granularity_;
  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = static_cast<int>(sim_time / sim_granularity + 0.5);

  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) {
    num_steps = 1;
  }

  double dt = sim_time / num_steps;
  double time = 0.0;

  // create a potential trajectory
  traj.resetPoints();
  traj.xv_ = vx_samp;
  traj.yv_ = vy_samp;
  traj.thetav_ = vtheta_samp;
  traj.cost_ = -1.0;

  // initialize the costs for the trajectory
  double path_dist = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // update path and goal distances
    double point_cost = DBL_MAX;
    for (auto it = global_plan_.begin(); it != global_plan_.end(); ++it) {
      double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
      if (loop_cost < point_cost) {
        point_cost = loop_cost;
      }
    }
    path_dist += point_cost;

    // the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    // calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    // increment time
    time += dt;
  }

  traj.cost_ = pdist_scale_ * path_dist;
}

/**
 * create and score a trajectory given the current pose of the robot and selected velocities
 */
void TrajectoryPlanner::generateTrajectory(
    double x, double y, double theta,
    double vx, double vy, double vtheta,
    double vx_samp, double vy_samp, double vtheta_samp,
    double acc_x, double acc_y, double acc_theta,
    double impossible_cost,
    Trajectory& traj, double sim_time) {

  // make sure the configuration doesn't change mid run
  boost::mutex::scoped_lock l(configuration_mutex_);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i, vy_i, vtheta_i;

  vx_i = vx;
  vy_i = vy;
  vtheta_i = vtheta;

  // discard trajectory that is circle
  if (fabs(vtheta_samp) - 0.0 > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
    traj.cost_ = -1.0;
    // ROS_WARN("[TRAJECTORY PLANNER] trajectory is circle, cost = -1.0, vtheta_samp: %lf, sim_time: %lf", vtheta_samp, sim_time);
    return;
  }

  double sim_granularity = sim_time / sim_time_ * sim_granularity_;
  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = static_cast<int>(sim_time / sim_granularity + 0.5);

  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) {
    num_steps = 1;
  }

  double dt = sim_time / num_steps;
  double time = 0.0;

  // create a potential trajectory
  traj.resetPoints();
  traj.xv_ = vx_samp;
  traj.yv_ = vy_samp;
  traj.thetav_ = vtheta_samp;
  traj.cost_ = -1.0;

  // initialize the costs for the trajectory
  double path_dist = 0.0;
  double heading_diff = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
      ROS_WARN("[LOCAL PLANNER] world to map failed");
      traj.cost_ = -1.0;
      return;
    }
    // TODO(lizhen) check if it is needed
    if (i < 3) {
      double footprint_cost = 0.0;
      if (i < num_calc_footprint_cost_) {
        // check the point on the trajectory for legality
        footprint_cost = footprintCost(x_i, y_i, theta_i);

        // if the footprint hits an obstacle this trajectory is invalid
        if (footprint_cost < 0) {
          traj.cost_ = -1.0;
          return;
        }
      }
    }

    // update path and goal distances
    double point_cost = 0x7ffffff;
    for (auto it = global_plan_.begin(); it != global_plan_.end(); ++it) {
      double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
      if (loop_cost < point_cost) {
        point_cost = loop_cost;
      }
    }
    path_dist += point_cost;

    // if a point on this trajectory has no clear path it is invalid
    if (impossible_cost <= path_dist) {
      traj.cost_ = -2.0;
      ROS_WARN("[TRAJECTORY PLANNER] impossible_cost <= path_dist, cost = -2.0");
      return;
    }

    // the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    // calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    // increment time
    time += dt;
  }  //  end for i < numsteps

  traj.cost_ = pdist_scale_ * path_dist;
}

/**
 * create and score a trajectory given the current pose of the robot and selected velocities
 */
void TrajectoryPlanner::generateTrajectoryWithoutCheckingFootprint(
    double x, double y, double theta,
    double vx, double vy, double vtheta,
    double vx_samp, double vy_samp, double vtheta_samp,
    double acc_x, double acc_y, double acc_theta,
    double impossible_cost,
    Trajectory& traj, double sim_time) {
  // make sure the configuration doesn't change mid run
  boost::mutex::scoped_lock l(configuration_mutex_);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i, vy_i, vtheta_i;

  vx_i = vx;
  vy_i = vy;
  vtheta_i = vtheta;

  // discard trajectory that is circle
  if (fabs(vtheta_samp) - 0.0 > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
    traj.cost_ = -1.0;
    return;
  }

  double sim_granularity = sim_time / sim_time_ * sim_granularity_;
  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = static_cast<int>(sim_time / sim_granularity + 0.5);

  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) {
    num_steps = 1;
  }

  double dt = sim_time / num_steps;
  double time = 0.0;

  // create a potential trajectory
  traj.resetPoints();
  traj.xv_ = vx_samp;
  traj.yv_ = vy_samp;
  traj.thetav_ = vtheta_samp;
  traj.cost_ = -1.0;

  // initialize the costs for the trajectory
  double path_dist = 0.0;
  double goal_dist = 0.0;
  double occ_cost = 0.0;
  double heading_diff = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
      ROS_WARN("[LOCAL PLANNER] world to map failed");
      traj.cost_ = -1.0;
      return;
    }

    // update path and goal distances
    double point_cost = 0x7ffffff;
    for (auto it = global_plan_.begin(); it != global_plan_.end(); ++it) {
      double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
      if (loop_cost < point_cost) {
        point_cost = loop_cost;
      }
    }
    path_dist += point_cost;

    // if a point on this trajectory has no clear path it is invalid
    if (impossible_cost <= path_dist) {
      traj.cost_ = -2.0;
      return;
    }

    // the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    // calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    // increment time
    time += dt;
  }

  traj.cost_ = pdist_scale_ * path_dist;
}

/**
 * create and score a trajectory given the current pose of the robot and selected velocities
 */
void TrajectoryPlanner::generateTrajectoryForRecovery(
    double x, double y, double theta,
    double vx, double vy, double vtheta,
    double vx_samp, double vy_samp, double vtheta_samp,
    double acc_x, double acc_y, double acc_theta,
    double impossible_cost,
    Trajectory& traj, double sim_time, int within_obs_thresh) {

  // make sure the configuration doesn't change mid run
  boost::mutex::scoped_lock l(configuration_mutex_);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i, vy_i, vtheta_i;

  vx_i = vx;
  vy_i = vy;
  vtheta_i = vtheta;

  // discard trajectory that is circle
  if (fabs(vtheta_samp) - 0.0 > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
    traj.cost_ = -1.0;
    return;
  }

  double sim_granularity = sim_time / sim_time_ * sim_granularity_;
  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = static_cast<int>(sim_time / sim_granularity + 0.5);

  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) {
    num_steps = 1;
  }

  double dt = sim_time / num_steps;
  double time = 0.0;

  // create a potential trajectory
  traj.resetPoints();
  traj.xv_ = vx_samp;
  traj.yv_ = vy_samp;
  traj.thetav_ = vtheta_samp;
  traj.cost_ = -1.0;

  // initialize the costs for the trajectory
  double path_dist = 0.0;

  // recovery specific, record if first some points's footprint are in obstacle
  bool within_obs = true;
  int within_obs_num = 0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
      ROS_WARN("[LOCAL PLANNER] world to map failed");
      traj.cost_ = -1.0;
      return;
    }

    double footprint_cost = 0.0;
    if (i < num_calc_footprint_cost_) {
      // check the point on the trajectory for legality
      footprint_cost = footprintCost(x_i, y_i, theta_i);

      // if the footprint hits an obstacle this trajectory is invalid
      if (footprint_cost < 0) {
        traj.cost_ = -1.0;
        if (within_obs)
          within_obs_num++;
        if (!within_obs || within_obs_num > within_obs_thresh)
          return;
      } else {
        within_obs = false;
      }
    }

    // update path and goal distances
    double point_cost = 0x7ffffff;
    for (auto it = global_plan_.begin(); it != global_plan_.end(); ++it) {
      double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
      if (loop_cost < point_cost) {
        point_cost = loop_cost;
      }
    }
    path_dist += point_cost;

    // if a point on this trajectory has no clear path it is invalid
    if (impossible_cost <= path_dist) {
      traj.cost_ = -2.0;
      return;
    }

    // the point is legal... add it to the trajectory
    traj.addPoint(x_i, y_i, theta_i);

    // calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    // increment time
    time += dt;
  }  // end for i < numsteps

  traj.cost_ = pdist_scale_ * path_dist;
}

void TrajectoryPlanner::updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan) {
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i) {
    global_plan_[i] = new_plan[i];
  }

  if (global_plan_.size() > 0) {
    geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
    final_goal_x_ = final_goal_pose.pose.position.x;
    final_goal_y_ = final_goal_pose.pose.position.y;
    final_goal_position_valid_ = true;
  } else {
    final_goal_position_valid_ = false;
  }

  // set need_backward_ to false every time
  need_backward_ = false;
}

bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy,
                                        double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
  Trajectory t;

  double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

  // if the trajectory is a legal one... the check passes
  if (cost >= 0) {
    return true;
  }
  ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

  // otherwise the check fails
  return false;
}

double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy,
                                          double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
  Trajectory t;
  double impossible_cost = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  generateTrajectory(x, y, theta,
                     vx, vy, vtheta,
                     vx_samp, vy_samp, vtheta_samp,
                     acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                     impossible_cost, t, sim_time_);

  // return the cost.
  return static_cast<double>(t.cost_);
}

/**
 * check front safe
 */
bool TrajectoryPlanner::checkFrontSafe(
    double x, double y, double theta,
    double vx, double vy, double vtheta) {

  // make sure the configuration doesn't change mid run
  boost::mutex::scoped_lock l(configuration_mutex_);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  // compute the number of steps
  int num_steps = static_cast<int>(front_safe_sim_time_ / front_safe_sim_granularity_ + 0.5);

  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) {
    num_steps = 1;
  }

  double dt = front_safe_sim_time_ / num_steps;
  double time = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
      return false;
    }

    // check the point on the trajectory for legality
    double footprint_cost = footprintCost(x_i, y_i, theta_i);

    // if the footprint hits an obstacle this trajectory is invalid
    if (footprint_cost < 0) {
      return false;
    }

    // calculate positions
    x_i = computeNewXPosition(x_i, vx, vy, theta_i, dt);
    y_i = computeNewYPosition(y_i, vx, vy, theta_i, dt);
    theta_i = computeNewThetaPosition(theta_i, vtheta, dt);

    // increment time
    time += dt;
  }  // end for i < numsteps

  return true;
}  // End of checkFrontSafe

void TrajectoryPlanner::SetNeedBackward(double x, double y, double theta,
                                        double vx, double vy, double vtheta,
                                        double vx_samp, double vy_samp, double vtheta_samp,
                                        double acc_x, double acc_y, double acc_theta,
                                        double impossible_cost, double sim_time) {
  // make sure the configuration doesn't change mid run
  // boost::mutex::scoped_lock l(configuration_mutex_);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i = vx;
  double vy_i = vy;
  double vtheta_i = vtheta;

  double sim_granularity = sim_time / sim_time_ * sim_granularity_;
  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = static_cast<int>(sim_time / sim_granularity + 0.5);
  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) num_steps = 1;

  double dt = sim_time / num_steps;
  double time = 0.0;

  // set safe distance
  double safe_dis = 0.15;
  if (vx < 0) safe_dis = 0.25;
  double dis_accu = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) break;

    // if the footprint hits an obstacle this trajectory is invalid
    if (footprintCost(x_i, y_i, theta_i) < 0) {
      Trajectory traj;
      generateTrajectoryForRecovery(x, y, theta, vx, vy, vtheta, -0.1, 0.0, 0.0,
                                    acc_x, acc_y, acc_theta, impossible_cost, traj, sim_time_, 5);
      if (traj.cost_ < 0) {
        need_backward_ = false;
      } else {
        need_backward_ = true;
      }
      return;
    }

    // calculate velocities
    vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    double new_x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    double new_y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    double new_theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

    // check safe_dis
    dis_accu += hypot(x_i - new_x_i, y_i - new_y_i);
    if (dis_accu > safe_dis) break;

    x_i = new_x_i;
    y_i = new_y_i;
    theta_i = new_theta_i;

    // increment time
    time += dt;
  }  //  end for i < numsteps

  need_backward_ = false;
}

/*
 * create the trajectories we wish to score
 */
Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
                                                 double max_vel, double highlight, double current_point_dis,
                                                 double vx, double vy, double vtheta,
                                                 double acc_x, double acc_y, double acc_theta, std::vector<Trajectory>* all_explored) {
  // compute feasible velocity limits in robot space
  double max_vel_x = max_vel_x_, max_vel_theta;
  double min_vel_x, min_vel_theta;

  if (final_goal_position_valid_) {
    double final_goal_dist = hypot(final_goal_x_ - x, final_goal_y_ - y);
    max_vel_x = std::min(max_vel_x, final_goal_dist / sim_time_);
  }

  max_vel_x = std::max(std::min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
  min_vel_x = std::max(min_vel_x_, vx - acc_x * sim_time_);

  max_vel_theta = std::min(max_vel_th_, vtheta + acc_theta * sim_time_);
  min_vel_theta = std::max(min_vel_th_, vtheta - acc_theta * sim_time_);

  // we want to sample the velocity space regularly
  double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

  double vx_samp = min_vel_x;
  double vtheta_samp = min_vel_theta;
  double vy_samp = 0.0;

  // keep track of the best trajectory seen so far
  Trajectory* best_traj = &traj_one;
  best_traj->cost_ = -1.0;

  Trajectory* comp_traj = &traj_two;
  comp_traj->cost_ = -1.0;

  Trajectory* swap = NULL;

  // check front safe first, if not safe, return best->cost_ = -1
  if (!checkFrontSafe(x, y, theta, vx, vy, vtheta)) {
    ROS_ERROR("[LOCAL PLANNER] checkFrontSafe failed! vx: %lf, vtheta: %lf", vx, vtheta);
    return *best_traj;
  }

  // any cell with a cost greater than the size of the map is impossible
  double impossible_cost = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();

  if (highlight < 0.5) highlight = 0.5;
  vx_samp = max_vel;
  if (vx_samp > max_vel_x) vx_samp = max_vel_x;
  if (vx_samp < min_vel_x_) vx_samp = min_vel_x_;
  double temp_sim_time = sim_time_;
  temp_sim_time = highlight * 0.8 / vx_samp;
  // velocity goes higher if current_point_dis is too far
  if (current_point_dis > 0.12) {
    double distance_to_goal = hypot(global_plan_.back().pose.position.x - x, global_plan_.back().pose.position.y - y);
    if (distance_to_goal / highlight > 1.0) temp_sim_time *= (distance_to_goal / highlight);
  }
  if (temp_sim_time < 2.0) temp_sim_time = 2.0;

  vtheta_samp = 0;
  // first sample the straight trajectory
  generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                     acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
  all_explored->push_back(*comp_traj);

  // if the new trajectory is better... let's take it
  if (comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)) {
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
  }

  vtheta_samp = min_vel_theta;
  // next sample all theta trajectories
  int begin = 0;
  int end = vtheta_samples_ - 1;
  std::vector<double> costs;
  costs.resize(vtheta_samples_);
  for (auto&& i : costs) i = -100.0;  // NOLINT
  while (begin < end) {
    int mid = (begin + end) / 2;
    if (fabs(costs[mid]  + 100) < GS_DOUBLE_PRECISION) {
      CalculatePathCost(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * mid,
                        acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
      costs[mid] = comp_traj->cost_;
    }
    int right = mid;
    while (++right <= end) {
      if (fabs(costs[right] + 100) < GS_DOUBLE_PRECISION) {
        CalculatePathCost(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * right,
                          acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
        costs[right] = comp_traj->cost_;
        if (fabs(costs[right] - costs[mid]) > GS_DOUBLE_PRECISION) break;
      }
    }
    if (right > end) right = end;
    if (costs[mid] > costs[right])
      begin = right;
    else
      end = mid;
  }
  int best_index = end;

  // now check footprint
  begin = end = best_index;
  for (auto&& i : costs) i = -100.0;  // NOLINT
  generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * best_index,
                     acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
  all_explored->push_back(*comp_traj);
  costs[best_index] = comp_traj->cost_;
  if (costs[best_index] >= 0.0) {
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
  }
  while (best_traj->cost_ < 0.0 && (begin > 0 || end < vtheta_samples_ - 1)) {
    begin = begin >= 1 ? begin - 1 : 0;
    end = end < vtheta_samples_ - 1 ? end + 1 : vtheta_samples_ - 1;
    // if begin reaches 0 and end hasn't reached last, we'll not generate and wait
    if (fabs(costs[begin] + 100) < GS_DOUBLE_PRECISION) {
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * begin,
                         acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
      all_explored->push_back(*comp_traj);
      costs[begin] = comp_traj->cost_;
    }
    if (costs[begin] >= 0.0) {
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;
    }
    // if end reaches last and begin hasn't reached 0, we'll not generate and wait
    if (fabs(costs[end] + 100) < GS_DOUBLE_PRECISION) {
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * end,
                         acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
      all_explored->push_back(*comp_traj);
      costs[end] = comp_traj->cost_;
    }
    if (costs[end] >= 0.0 && (best_traj->cost_ < 0 || costs[end] < best_traj->cost_)) {
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;
      // we want to sample more trajectories to keep a distance from obstacle
      int search_index = end + 1;
      while (search_index <= end + avoid_obstacle_traj_num_ && search_index <= vtheta_samples_ - 1) {
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * search_index,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
        all_explored->push_back(*comp_traj);
        costs[search_index] = comp_traj->cost_;
        if (costs[search_index] >= 0.0) {
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
        search_index++;
      }
    } else if (costs[begin] >= 0.0) {
      // we want to sample more trajectories to keep a distance from obstacle
      int search_index = begin - 1;
      while (search_index >= begin - avoid_obstacle_traj_num_ && search_index >= 0) {
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * search_index,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
        all_explored->push_back(*comp_traj);
        costs[search_index] = comp_traj->cost_;
        if (costs[search_index] >= 0.0) {
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
        search_index--;
      }
    }
  }

  // if best_traj is valid, just return, as we don't want to rotate in place
  if (best_traj->cost_ >= 0.0) {
    return *best_traj;
  }

  // next we want to generate trajectories for rotating in place
  vtheta_samp = std::max(min_vel_theta, -1 * min_in_place_vel_th_);
  vx_samp = 0.0;
  vy_samp = 0.0;

  // rotate to right
  generateTrajectoryForRecovery(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                                acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, sim_time_, 5);
  if (comp_traj->cost_ >= 0.0) {
    ROS_INFO("[FIXPATTERN LOCAL PLANNER] rotate to right");
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
    return *best_traj;
  }

  // rotate to left
  vtheta_samp = std::min(max_vel_theta, min_in_place_vel_th_);
  generateTrajectoryForRecovery(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                                acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, sim_time_, 5);
  if (comp_traj->cost_ >= 0.0) {
    ROS_INFO("[FIXPATTERN LOCAL PLANNER] rotate to left");
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
    return *best_traj;
  }

  // and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
  ROS_INFO("[FIXPATTERN LOCAL PLANNER] going back with vel: %lf", backup_vel_);
  vtheta_samp = 0.0;
  vx_samp = backup_vel_;
  vy_samp = 0.0;
  generateTrajectoryForRecovery(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                                acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, sim_time_, 5);

  // our last chance
  swap = best_traj;
  best_traj = comp_traj;
  comp_traj = swap;
  return *best_traj;
}

// given the current state of the robot, find a good trajectory
Trajectory TrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, double traj_vel,
                                           double highlight, double current_point_dis,
                                           tf::Stamped<tf::Pose> global_vel,
                                           tf::Stamped<tf::Pose>& drive_velocities, std::vector<Trajectory>* all_explored) {
  Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
  Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));

  // temporarily remove obstacles that are within the footprint of the robot
  // std::vector<fixpattern_local_planner::Position2DInt> footprint_list =
  //     footprint_helper_.getFootprintCells(pos, footprint_spec_, costmap_, true);

  // rollout trajectories and find the minimum cost one
  Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
                                       traj_vel, highlight, current_point_dis,
                                       vel[0], vel[1], vel[2],
                                       acc_lim_x_, acc_lim_y_, acc_lim_theta_, all_explored);
  ROS_DEBUG("Trajectories created");

  if (best.cost_ < 0) {
    drive_velocities.setIdentity();
  } else {
    tf::Vector3 start(best.xv_, best.yv_, 0);
    drive_velocities.setOrigin(start);
    tf::Matrix3x3 matrix;
    matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
    drive_velocities.setBasis(matrix);
  }

  return best;
}

// we need to take the footprint of the robot into account when we calculate cost to obstacles
double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i) {
  // check if the footprint is legal
  return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
}

};  // namespace fixpattern_local_planner


