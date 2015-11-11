/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <fixpattern_local_planner/trajectory_planner.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>


#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>

#include <fixpattern_path/path.h>

using namespace std;
using namespace costmap_2d;

namespace fixpattern_local_planner {

  void TrajectoryPlanner::reconfigure(BaseLocalPlannerConfig &cfg)
  {
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
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
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
      if(vtheta_samples_ <= 0) {
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

      //y-vels
      string y_string = config.y_vels;
      vector<string> y_strs;
      boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>y_vels;
      for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          y_vels.push_back(temp);
          //ROS_INFO("Adding y_vel: %e", temp);
      }

      y_vels_ = y_vels;

  }

  TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
      const Costmap2D& costmap,
      std::vector<geometry_msgs::Point> footprint_spec,
      double acc_lim_x, double acc_lim_y, double acc_lim_theta,
      int num_calc_footprint_cost, int trajectory_range_check_obstacle_avoidance,
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
      vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
    : costmap_(costmap),
    world_model_(world_model), footprint_spec_(footprint_spec), num_calc_footprint_cost_(num_calc_footprint_cost),
    trajectory_range_check_obstacle_avoidance_(trajectory_range_check_obstacle_avoidance),
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
    simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period)
  {
    if (meter_scoring) {
      ROS_INFO("[FIXPATTERN LOCAL PLANNER] meter_scoring set to true");
    } else {
      ROS_INFO("[FIXPATTERN LOCAL PLANNER] meter_scoring set to false");
    }
    final_goal_position_valid_ = false;

    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

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

    // compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    // discard trajectory that is circle
    if (fabs(vtheta_samp) > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
      traj.cost_ = DBL_MAX;
      return;
    }

    double sim_granularity = sim_time / sim_time_ * sim_granularity_;
    // compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(sim_time / sim_granularity + 0.5);

    // we at least want to take one step... even if we won't move, we want to score our current position
    if (num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time / num_steps;
    double time = 0.0;

    //create a potential trajectory
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

    //compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    // discard trajectory that is circle
    if (fabs(vtheta_samp) - 0.0 > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
      traj.cost_ = -1.0;
      // ROS_WARN("[TRAJECTORY PLANNER] trajectory is circle, cost = -1.0, vtheta_samp: %lf, sim_time: %lf", vtheta_samp, sim_time);
      return;
    }

    double sim_granularity = sim_time / sim_time_ * sim_granularity_;
    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(sim_time / sim_granularity + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        ROS_WARN("[LOCAL PLANNER] world to map failed");
        traj.cost_ = -1.0;
        return;
      }

      double footprint_cost = 0.0;
      if (i < num_calc_footprint_cost_) {
        //check the point on the trajectory for legality
        footprint_cost = footprintCost(x_i, y_i, theta_i);

        //if the footprint hits an obstacle this trajectory is invalid
        if(footprint_cost < 0){
          // ROS_WARN("[LOCAL PLANNER] footprint_cost < 0, num_steps: %d", i);
          traj.cost_ = -1.0;
          return;
          //TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
          //it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to
          //come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
          //but safe.
          /*
             double max_vel_x, max_vel_y, max_vel_th;
          //we want to compute the max allowable speeds to be able to stop
          //to be safe... we'll make sure we can stop some time before we actually hit
          getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

          //check if we can stop in time
          if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
          ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
          //if we can stop... we'll just break out of the loop here.. no point in checking future points
          break;
          }
          else{
          traj.cost_ = -1.0;
          return;
          }
          */
        }
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      } else {

        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          double point_cost = 0x7ffffff;
          for (std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan_.begin();
               it != global_plan_.end(); ++it) {
            double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
            if (loop_cost < point_cost) {
              point_cost = loop_cost;
            }
          }
          path_dist += point_cost;

          //if a point on this trajectory has no clear path it is invalid
          if (impossible_cost <= path_dist) {
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            ROS_WARN("[TRAJECTORY PLANNER] impossible_cost <= path_dist, cost = -2.0");
            return;
          }
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if (!heading_scoring_) {
      cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost;
    } else {
      cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;
    }
    traj.cost_ = cost;
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

    // compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    // discard trajectory that is circle
    if (fabs(vtheta_samp) - 0.0 > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
      traj.cost_ = -1.0;
      return;
    }

    double sim_granularity = sim_time / sim_time_ * sim_granularity_;
    // compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(sim_time / sim_granularity + 0.5);

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

      // do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      } else {
        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          // update path and goal distances
          double point_cost = 0x7ffffff;
          for (std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan_.begin();
               it != global_plan_.end(); ++it) {
            double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
            if (loop_cost < point_cost) {
              point_cost = loop_cost;
            }
          }
          path_dist += point_cost;

          // if a point on this trajectory has no clear path it is invalid
          if (impossible_cost <= path_dist) {
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            return;
          }
        }
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

    double cost = -1.0;
    if (!heading_scoring_) {
      cost = pdist_scale_ * path_dist;
    } else {
      cost = pdist_scale_ * path_dist + 0.3 * heading_diff;
    }
    traj.cost_ = cost;
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

    //compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    // discard trajectory that is circle
    if (fabs(vtheta_samp) - 0.0 > 0.00001 && sim_time > M_PI / fabs(vtheta_samp)) {
      traj.cost_ = -1.0;
      return;
    }

    double sim_granularity = sim_time / sim_time_ * sim_granularity_;
    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps = int(sim_time / sim_granularity + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    // recovery specific, record if first some points's footprint are in obstacle
    bool within_obs = true;
    int within_obs_num = 0;

    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        ROS_WARN("[LOCAL PLANNER] world to map failed");
        traj.cost_ = -1.0;
        return;
      }

      double footprint_cost = 0.0;
      if (i < num_calc_footprint_cost_) {
        //check the point on the trajectory for legality
        footprint_cost = footprintCost(x_i, y_i, theta_i);

        //if the footprint hits an obstacle this trajectory is invalid
        if(footprint_cost < 0){
          traj.cost_ = -1.0;
          if (within_obs)
            within_obs_num++;
          if (!within_obs || within_obs_num > within_obs_thresh)
            return;
        } else {
          within_obs = false;
        }
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      //do we want to follow blindly
      if (simple_attractor_) {
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) *
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) +
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) *
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
      } else {

        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          double point_cost = 0x7ffffff;
          for (std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan_.begin();
               it != global_plan_.end(); ++it) {
            double loop_cost = hypot(x_i - it->pose.position.x, y_i - it->pose.position.y);
            if (loop_cost < point_cost) {
              point_cost = loop_cost;
            }
          }
          path_dist += point_cost;

          //if a point on this trajectory has no clear path it is invalid
          if (impossible_cost <= path_dist) {
            traj.cost_ = -2.0;
            return;
          }
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    if (!heading_scoring_) {
      cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost;
    } else {
      cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;
    }
    traj.cost_ = cost;
  }

  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    const double v2_x = cos(heading);
    const double v2_y = sin(heading);

    //find a clear line of sight from the robot's cell to a point on the path
    for (int i = global_plan_.size() - 1; i >=0; --i) {
      if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) {
        if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = fabs(vector_angle);
          return heading_diff;
        }
      }
    }
    return heading_diff;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1,
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
      point_cost = pointCost(x, y); //Score the current point

      if (point_cost < 0) {
        return -1;
      }

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den) {           // Check if numerator >= denominator
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    if( global_plan_.size() > 0 ){
      geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
      final_goal_x_ = final_goal_pose.pose.position.x;
      final_goal_y_ = final_goal_pose.pose.position.y;
      final_goal_position_valid_ = true;
    } else {
      final_goal_position_valid_ = false;
    }
  }

  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t;

    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //otherwise the check fails
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
    return double( t.cost_ );
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

    //compute the number of steps
    int num_steps = int(front_safe_sim_time_ / front_safe_sim_granularity_ + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if (num_steps == 0) {
      num_steps = 1;
    }

    double dt = front_safe_sim_time_ / num_steps;
    double time = 0.0;

    for (int i = 0; i < num_steps; ++i) {
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
        return false;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        return false;
      }

      //calculate positions
      x_i = computeNewXPosition(x_i, vx, vy, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx, vy, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    return true;

  } // End of checkFrontSafe


  void TrajectoryPlanner::CheckNeedObstacleAvoidance(std::vector<double>* costs,
                                                     std::vector<double>* costs_without_footprint,
                                                     int check_range) {
    unsigned int min_cost_index = 0;
    unsigned int min_cost_num = 0;
    double min_cost = 0x7ffffff;
    for (unsigned int i = 0; i < costs_without_footprint->size(); ++i) {
      if (costs_without_footprint->at(i) < min_cost) {
         min_cost = costs_without_footprint->at(i);
         min_cost_index = i;
         min_cost_num = 0;
      } else if (costs_without_footprint->at(i) == min_cost) {
        min_cost_num++;
      }
    }
    min_cost_index += min_cost_num / 2;

    unsigned int begin = min_cost_index >= check_range ? min_cost_index - check_range : 0;
    unsigned int end = min_cost_index + check_range < costs_without_footprint->size() ?
        min_cost_index + check_range : costs_without_footprint->size() - 1;

    for (unsigned int i = begin; i <= end; ++i) {
      if (costs->at(i) != -1) {
        need_avoid_obstacle_ = false;
        return;
      }
    }
    need_avoid_obstacle_ = true;
  }

  /*
   * create the trajectories we wish to score
   */
  Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta,
      double max_vel, double highlight, double current_point_dis,
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta, std::vector<Trajectory>* all_explored, int log_id) {

    //compute feasible velocity limits in robot space
    double max_vel_x = max_vel_x_, max_vel_theta;
    double min_vel_x, min_vel_theta;

    if( final_goal_position_valid_ ){
      double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
      max_vel_x = min( max_vel_x, final_goal_dist / sim_time_ );
    }

    //should we use the dynamic window approach?
    if (dwa_) {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);
    } else {
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_time_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_time_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_time_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_time_);
    }


    //we want to sample the velocity space regularly
    double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
    double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
    double vy_samp = 0.0;

    //keep track of the best trajectory seen so far
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

    //any cell with a cost greater than the size of the map is impossible
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
      // ROS_INFO("[LOCAL PLANNER] distance_to_goal / highlight: %lf", distance_to_goal / highlight);
    }
    if (temp_sim_time < 2.0) temp_sim_time = 2.0;
    // ROS_INFO("[LOCAL PLANNER] real highlight: %lf, vx: %lf, temp_sim_time: %lf", highlight, vx_samp, temp_sim_time);

    vtheta_samp = 0;
    //first sample the straight trajectory
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
        acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
    all_explored->push_back(*comp_traj);

    //if the new trajectory is better... let's take it
    if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;
    }

    vtheta_samp = min_vel_theta;
    // next sample all theta trajectories
    // calculate average theta if lots of best_traj->thetav_ is equal
    int begin = 0;
    int end = vtheta_samples_ - 1;
    std::vector<double> costs;
    costs.resize(vtheta_samples_);
    for (auto&& i : costs) i = -100.0;
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
    for (auto&& i : costs) i = -100;
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * best_index,
                       acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
    all_explored->push_back(*comp_traj);
    double cost = comp_traj->cost_;
    costs[best_index] = cost;
    if (cost >= 0.0) {
      swap = best_traj;
      best_traj = comp_traj;
      comp_traj = swap;
    }
    while (cost < 0.0 && !(begin == 0 && end == vtheta_samples_ - 1)) {
      begin = begin >= 1 ? begin - 1 : 0;
      end = end < vtheta_samples_ - 1 ? end + 1 : vtheta_samples_ - 1;
      // if begin reaches 0 and end hasn't reached last, we'll not generate and wait
      if (fabs(costs[begin] + 100) < GS_DOUBLE_PRECISION) {
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * begin,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
        all_explored->push_back(*comp_traj);
        cost = comp_traj->cost_;
        costs[begin] = cost;
        if (cost >= 0.0) {
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      } else {
        cost = costs[begin];
      }
      // if end reaches last and begin hasn't reached 0, we'll not generate and wait
      if (fabs(costs[end] + 100) < GS_DOUBLE_PRECISION) {
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp + dvtheta * end,
                           acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, temp_sim_time);
        all_explored->push_back(*comp_traj);
        double temp_cost = comp_traj->cost_;
        costs[end] = temp_cost;
        if (temp_cost >= 0.0 && temp_cost < cost) {
          cost = temp_cost;
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
        }
      } else {
         cost = costs[end];
      }
    }

    // if best_traj is valid, just return, as we don't want to rotate in place
    if (best_traj->cost_ >= 0.0) {
      return *best_traj;
    }

    //next we want to generate trajectories for rotating in place
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

    //and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
    ROS_INFO("[FIXPATTERN LOCAL PLANNER] going back with vel: %lf", backup_vel_);
    vtheta_samp = 0.0;
    vx_samp = backup_vel_;
    vy_samp = 0.0;
    generateTrajectoryForRecovery(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
                                  acc_x, acc_y, acc_theta, impossible_cost, *comp_traj, sim_time_, 5);

    // if cost < 0, return anyway
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;
    best_traj->cost_ = 0.0;
    return *best_traj;
  }

  //given the current state of the robot, find a good trajectory
  Trajectory TrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, double traj_vel,
                                             double highlight, double current_point_dis,
                                             tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities, std::vector<Trajectory>* all_explored, int log_id){

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<fixpattern_local_planner::Position2DInt> footprint_list =
        footprint_helper_.getFootprintCells(
            pos,
            footprint_spec_,
            costmap_,
            true);

    //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectories(pos[0], pos[1], pos[2],
        traj_vel, highlight, current_point_dis,
        vel[0], vel[1], vel[2],
        acc_lim_x_, acc_lim_y_, acc_lim_theta_, all_explored, log_id);
    ROS_DEBUG("Trajectories created");

    /*
    //If we want to print a ppm file to draw goal dist
    char buf[4096];
    sprintf(buf, "base_local_planner.ppm");
    FILE *fp = fopen(buf, "w");
    if(fp){
      fprintf(fp, "P3\n");
      fprintf(fp, "%d %d\n", map_.size_x_, map_.size_y_);
      fprintf(fp, "255\n");
      for(int j = map_.size_y_ - 1; j >= 0; --j){
        for(unsigned int i = 0; i < map_.size_x_; ++i){
          int g_dist = 255 - int(map_(i, j).goal_dist);
          int p_dist = 255 - int(map_(i, j).path_dist);
          if(g_dist < 0)
            g_dist = 0;
          if(p_dist < 0)
            p_dist = 0;
          fprintf(fp, "%d 0 %d ", g_dist, 0);
        }
        fprintf(fp, "\n");
      }
      fclose(fp);
    }
    */

    if(best.cost_ < 0){
      drive_velocities.setIdentity();
    }
    else{
      tf::Vector3 start(best.xv_, best.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }

};


