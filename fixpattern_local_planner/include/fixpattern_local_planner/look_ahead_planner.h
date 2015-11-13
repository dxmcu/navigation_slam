/* Copyright(C) Gaussian Robot. All rights reserved.
*/

/**
 * @file look_ahead_planner.h
 * @brief look ahead planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-04
 */

#ifndef FIXPATTERN_LOCAL_PLANNER_INCLUDE_FIXPATTERN_LOCAL_PLANNER_LOOK_AHEAD_PLANNER_H_
#define FIXPATTERN_LOCAL_PLANNER_INCLUDE_FIXPATTERN_LOCAL_PLANNER_LOOK_AHEAD_PLANNER_H_

#include <costmap_2d/costmap_2d.h>
#include <fixpattern_local_planner/world_model.h>
#include <fixpattern_local_planner/trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <algorithm>
#include <cmath>

namespace fixpattern_local_planner {

class LookAheadPlanner {
 public:
  // front safe check removed, should be put in move_base
  /**
   * @brief Constructs a look-ahead controller
   *
   * @param world_model The WorldModel the look-ahead controller uses to check for collisions
   * @param costmap A reference to the Costmap the controller should use
   * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
   * @param sample_granularity The distance between simulation points, should be small enough that the robot doesn't hit things
   * @param acc_lim_x The acceleration limit of the robot in the x direction
   * @param acc_lim_y The acceleration limit of the robot in the y direction
   * @param acc_lim_theta The acceleration limit of the robot in the theta direction
   * @param max_vel_x The maximum x velocity the controller will explore
   * @param min_vel_x The minimum x velocity the controller will explore
   * @param max_vel_th The maximum rotational velocity the controller will explore
   * @param min_vel_th The minimum rotational velocity the controller will explore
   * @param min_in_place_vel_th The absolute value of the minimum in-place rotational velocity the controller will explore
   */
  LookAheadPlanner(WorldModel& world_model,  // NOLINT
                   const costmap_2d::Costmap2D& costmap,
                   const std::vector<geometry_msgs::Point>& footprint_spec,
                   double sample_granularity = 0.025,
                   double acc_lim_x = 1.0, double acc_lim_y = 1.0, double acc_lim_theta = 1.0,
                   double max_vel_x = 0.5, double min_vel_x = 0.1,
                   double max_vel_th = 1.0, double min_vel_th = -1.0, double min_in_place_vel_th = 0.4);
  /**
   * @brief Destructs a look-ahead controller
   */
  ~LookAheadPlanner();
  /**
   * @brief  Update the plan that the controller is following
   * @param new_plan A new plan for the controller to follow
   */
  void UpdatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan);
  /**
   * @brief Given the current position, orientation, and velocity of the robot, return a trajectory to follow
   *
   * @param global_pose The current pose of the robot in world space
   * @param global_vel The current velocity of the robot in world space
   * @param traj_vel Max velocity of tarjectory
   * @param highlight Highlight of local plan, determin look ahead point
   * @param drive_velocities Will be set to velocities to send to the robot base
   *
   * @return The selected path or trajectory
   */
  Trajectory GeneratePath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
                          double traj_vel, double highlight, tf::Stamped<tf::Pose>* drive_velocities);
  /**
   * @brief  Generate and score a single trajectory
   *
   * @param x The x position of the robot
   * @param y The y position of the robot
   * @param theta The orientation of the robot
   * @param vx The x velocity of the robot
   * @param vy The y velocity of the robot
   * @param vtheta The theta velocity of the robot
   * @param vx_samp The x velocity used to seed the trajectory
   * @param vy_samp The y velocity used to seed the trajectory
   * @param vtheta_samp The theta velocity used to seed the trajectory
   *
   * @return True if the trajectory is legal, false otherwise
   */
  bool CheckTrajectory(double x, double y, double theta, double vx, double vy,
                       double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

 private:
  /**
   * @brief Generate look ahead path
   *
   * @param x Current x pos of vehicle
   * @param y Current y pos of vehicle
   * @param theta Current theta of vehicle
   * @param look_ahead_distance Distance to look ahead
   * @param traj_vel Velocity to trajectory
   * @param w Theta velocity to return
   */
  void LookAhead(double x, double y, double theta, double look_ahead_distance, double traj_vel, double* w);
  /**
   * @brief Generte a trajectory, will check if it's valid
   *
   * @param x Vehicle start x position
   * @param y Vehicle start y position
   * @param theta Vehicle start theta
   * @param vx Velocity in x direction of vehicle
   * @param vy Velocity in y direction of vehicle
   * @param vtheta Theta velocity of vehicle
   * @param vx_samp The x velocity used to seed the trajectory
   * @param vy_samp The y velocity used to seed the trajectory
   * @param vtheta_samp The theta velocity used to seed the trajectory
   * @param acc_x The x acceleration limit of the robot
   * @param acc_y The y acceleration limit of the robot
   * @param acc_theta The theta acceleration limit of the robot
   * @param sample_time The number of seconds the trajectory is sampled
   * @param sample_granularity The granularity of points in the trajectory
   * @param traj The trajectory generated
   */
  void GenerateTrajectory(double x, double y, double theta,
                          double vx, double vy, double vtheta,
                          double vx_samp, double vy_samp, double vtheta_samp,
                          double acc_x, double acc_y, double acc_theta,
                          double sample_time, double sample_granularity, Trajectory* traj);
  /**
   * @brief Given vehicle position, returns cost of footprint
   *
   * @param x_i X position of vehicle
   * @param y_i Y position of vehicle
   * @param theta_i Direction of vehicle
   *
   * @return Cost of footprint
   */
  double FootPrintCost(double x_i, double y_i, double theta_i);
  // compute velocity based on acceleration
  /**
   * @brief Compute velocity based on acceleration
   *
   * @param vg The desired velocity, what we're accelerating up to
   * @param vi The current velocity
   * @param a_max An acceleration limit
   * @param dt The timestep to take
   *
   * @return The new velocity
   */
  inline double ComputeNewVelocity(double vg, double vi, double a_max, double dt) {
    if ((vg - vi) >= 0) {
      return std::min(vg, vi + a_max * dt);
    }
    return std::max(vg, vi - a_max * dt);
  }
  /**
   * @brief  Compute x position based on velocity
   *
   * @param  xi The current x position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   *
   * @return The new x position
   */
  inline double ComputeNewXPosition(double xi, double vx, double vy, double theta, double dt) {
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
  }
  /**
   * @brief  Compute y position based on velocity
   *
   * @param  yi The current y position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   *
   * @return The new y position
   */
  inline double ComputeNewYPosition(double yi, double vx, double vy, double theta, double dt) {
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
  }
  /**
   * @brief  Compute orientation based on velocity
   *
   * @param  thetai The current orientation
   * @param  vth The current theta velocity
   * @param  dt The timestep to take
   *
   * @return The new orientation
   */
  inline double ComputeNewThetaPosition(double thetai, double vth, double dt) {
    return thetai + vth * dt;
  }

  WorldModel& world_model_;               ///< @brief The world model that the controller uses for collision detection
  const costmap_2d::Costmap2D& costmap_;  ///< @brief Provides access to cost map information
  std::vector<geometry_msgs::Point> footprint_spec_;     ///< @brief The footprint specification of the robot
  std::vector<geometry_msgs::PoseStamped> global_plan_;  ///< @brief The global path for the robot to follow

  double sample_granularity_;  ///< @brief The distance between sample points
  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;  ///< @brief The acceleration limits of the robot
  double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_;  ///< @brief Velocity limits for the controller

  double inscribed_radius_, circumscribed_radius_;
};

};  // namespace fixpattern_local_planner

#endif  // FIXPATTERN_LOCAL_PLANNER_INCLUDE_FIXPATTERN_LOCAL_PLANNER_LOOK_AHEAD_PLANNER_H_
