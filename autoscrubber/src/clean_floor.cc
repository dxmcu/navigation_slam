/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file clean_floor.cc
 * @brief functions of cleaning floor
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-11-15
 */

#include "autoscrubber/clean_floor.h"

#include <fixpattern_local_planner/costmap_model.h>

namespace autoscrubber {

inline double ComputeNewXPosition(double xi, double vx, double vy, double theta, double dt) {
  return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
}

inline double ComputeNewYPosition(double yi, double vx, double vy, double theta, double dt) {
  return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
}

inline double ComputeNewThetaPosition(double thetai, double vth, double dt) {
  return thetai + vth * dt;
}

double FootprintCost(double x, double y, double theta, costmap_2d::Costmap2D* costmap,
                     const std::vector<geometry_msgs::Point>& footprint_spec,
                     double inscribed_radius, double circumscribed_radius) {
  fixpattern_local_planner::CostmapModel costmap_model(*costmap);
  return costmap_model.footprintCost(x, y, theta, footprint_spec, inscribed_radius, circumscribed_radius);
}

double CalculateTrajectoryObstacleCost(double x, double y, double theta,
                                       double vx, double vy, double vtheta,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
  costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
  std::vector<geometry_msgs::Point> footprint_spec = costmap_ros->getRobotFootprint();
  double inscribed_radius, circumscribed_radius;
  costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);

  double x_i = x;
  double y_i = y;
  double theta_i = theta;

  int num_steps = 8;
  double dt = 0.5;
  double time = 0.0;

  double cost = 0.0;

  for (int i = 0; i < num_steps; ++i) {
    // get map coordinates of a point
    unsigned int cell_x, cell_y;
    if (!costmap->worldToMap(x_i, y_i, cell_x, cell_y)) {
      return cost;
    }

    // check the point on the trajectory for legality
    double footprint_cost = FootprintCost(x_i, y_i, theta_i, costmap, footprint_spec, inscribed_radius, circumscribed_radius);

    // if the footprint hits an obstacle this trajectory is invalid
    if (footprint_cost < 0) {
      return -1.0;
    }

    cost = std::max(std::max(cost, footprint_cost), static_cast<double>(costmap->getCost(cell_x, cell_y)));

    // calculate positions
    x_i = ComputeNewXPosition(x_i, vx, vy, theta_i, dt);
    y_i = ComputeNewYPosition(y_i, vx, vy, theta_i, dt);
    theta_i = ComputeNewThetaPosition(theta_i, vtheta, dt);

    // increment time
    time += dt;
  } // end for i < numsteps

  return cost;
}

};  // namespace autoscrubber
