/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file look_ahead_planner.cpp
 * @brief look ahead planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-04
 */

#include <fixpattern_local_planner/look_ahead_planner.h>

#include <gslib/gslib.h>
#include <vector>
#include <Eigen/Dense>

namespace fixpattern_local_planner {

LookAheadPlanner::LookAheadPlanner(WorldModel& world_model,
                 const costmap_2d::Costmap2D& costmap,
                 const std::vector<geometry_msgs::Point>& footprint_spec,
                 double sample_granularity,
                 double acc_lim_x, double acc_lim_y, double acc_lim_theta,
                 double max_vel_x, double min_vel_x,
                 double max_vel_th, double min_vel_th, double min_in_place_vel_th)
  : world_model_(world_model), costmap_(costmap), footprint_spec_(footprint_spec),
    sample_granularity_(sample_granularity), acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y),
    acc_lim_theta_(acc_lim_theta), max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th) {
  costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
}

LookAheadPlanner::~LookAheadPlanner() { }

void LookAheadPlanner::UpdatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan) {
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i) {
    global_plan_[i] = new_plan[i];
  }
}

Trajectory LookAheadPlanner::GeneratePath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
                                          double traj_vel, double highlight, tf::Stamped<tf::Pose>* drive_velocities) {
  double x = global_pose.getOrigin().getX();
  double y = global_pose.getOrigin().getY();
  double theta = tf::getYaw(global_pose.getRotation());

  double v_x = global_vel.getOrigin().getX();
  double v_y = global_vel.getOrigin().getY();
  double v_theta = tf::getYaw(global_vel.getRotation());

  double w;
  LookAhead(x, y, theta, highlight, traj_vel, &w);

  Trajectory traj;
  // needs to compute
  double sample_time = 2.0;
  GenerateTrajectory(x, y, theta, v_x, v_y, v_theta,
                     traj_vel, 0.0, w, acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                     sample_time, sample_granularity_, &traj);

  if (traj.cost_ < 0) {
    drive_velocities->setIdentity();
  } else {
    tf::Vector3 start(traj_vel, 0, 0);
    drive_velocities->setOrigin(start);
    tf::Matrix3x3 matrix;
    matrix.setRotation(tf::createQuaternionFromYaw(w));
    drive_velocities->setBasis(matrix);
  }

  return traj;
}

bool LookAheadPlanner::CheckTrajectory(double x, double y, double theta, double vx, double vy,
                                        double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
  Trajectory t;
  double sample_time = 2.0;
  GenerateTrajectory(x, y, theta,
                     vx, vy, vtheta,
                     vx_samp, vy_samp, vtheta_samp,
                     acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                     sample_time, sample_granularity_, &t);

  // if the trajectory is a legal one... the check passes
  if (static_cast<double>(t.cost_) >= 0) {
    return true;
  }
  ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, static_cast<double>(t.cost_));

  // otherwise the check fails
  return false;
}

double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
  return hypot(p1.pose.position.x - p2.pose.position.x, p2.pose.position.y - p2.pose.position.y);
}

void LookAheadPlanner::LookAhead(double x, double y, double theta,
                                 double look_ahead_distance, double traj_vel, double* w) {
  if (global_plan_.size() == 0) {
    *w = 0;
    return;
  }

  // look ahead position is global_plan_'s back
  geometry_msgs::PoseStamped look_ahead_pose = global_plan_.back();

  // calculate which direction should go
  Eigen::Vector3d R_transform;
  Eigen::Vector3d start_pos(look_ahead_pose.pose.position.x, look_ahead_pose.pose.position.y,
                            tf::getYaw(look_ahead_pose.pose.orientation));
  Eigen::Vector3d current_pos(x, y, theta);
  R_transform << -sin(theta), cos(theta), 0;
  double p_y_e = R_transform.transpose() * (start_pos - current_pos);

  gslib::Point2D current_point(x, y);
  gslib::Point2D look_ahead_point(look_ahead_pose.pose.position.x, look_ahead_pose.pose.position.y);
  gslib::Point2D R_center(-10, -10);
  if (gslib::Line2D::CalculateCircleCenter(current_point, theta, look_ahead_point, &R_center)) {
    double radius = current_point.DistanceToPoint(R_center);
    if (p_y_e >= 0) {
      *w = traj_vel / radius;
    } else {
      *w = -traj_vel / radius;
    }
  } else {
    *w = 0;
  }
}

void LookAheadPlanner::GenerateTrajectory(double x, double y, double theta,
                                      double vx, double vy, double vtheta,
                                      double vx_samp, double vy_samp, double vtheta_samp,
                                      double acc_x, double acc_y, double acc_theta,
                                      double sample_time, double sample_granularity, Trajectory* traj) {
  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  double vx_i = vx;
  double vy_i = vy;
  double vtheta_i = vtheta;

  // compute the magnitude of the velocities
  double vmag = hypot(vx_samp, vy_samp);

  // compute the number of steps we must take along this trajectory to be "safe"
  int num_steps = static_cast<int>(sample_time / sample_granularity + 0.5);

  // we at least want to take one step... even if we won't move, we want to score our current position
  if (num_steps == 0) num_steps = 1;

  double dt = sample_time / num_steps;
  double time = 0.0;

  // create a potential trajectory
  traj->resetPoints();
  traj->xv_ = vx_samp;
  traj->yv_ = vy_samp;
  traj->thetav_ = vtheta_samp;
  traj->cost_ = -1.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;

    // we don't want a path that goes off the know map
    if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)) {
      ROS_WARN("[LOCAL PLANNER] world to map failed");
      traj->cost_ = -1.0;
      return;
    }

    double footprint_cost = FootPrintCost(x_i, y_i, theta_i);

    // if the footprint hits an obstacle this trajectory is invalid
    if (footprint_cost < 0) {
      ROS_WARN("[LOCAL PLANNER] footprint_cost < 0, num_steps: %d", i);
      traj->cost_ = -1.0;
      return;
    }

    // the point is legal... add it to the trajectory
    traj->addPoint(x_i, y_i, theta_i);

    // calculate velocities
    vx_i = ComputeNewVelocity(vx_samp, vx_i, acc_x, dt);
    vy_i = ComputeNewVelocity(vy_samp, vy_i, acc_y, dt);
    vtheta_i = ComputeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

    // calculate positions
    x_i = ComputeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
    y_i = ComputeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
    theta_i = ComputeNewThetaPosition(theta_i, vtheta_i, dt);

    // increment time
    time += dt;
  }

  traj->cost_ = 1.0;
}

double LookAheadPlanner::FootPrintCost(double x_i, double y_i, double theta_i) {
  return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
}

};  // namespace fixpattern_local_planner
