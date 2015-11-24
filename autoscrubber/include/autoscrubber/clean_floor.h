/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file clean_floor.h
 * @brief functions of clean floor
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-11-15
 */

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>

namespace autoscrubber {

double FootprintCost(double x, double y, double theta, costmap_2d::Costmap2D* costmap,
                     const std::vector<geometry_msgs::Point>& footprint_spec,
                     double inscribed_radius, double circumscribed_radius);

double CalculateTrajectoryObstacleCost(double x, double y, double theta,
                                       double vx, double vy, double vtheta,
                                       costmap_2d::Costmap2DROS* costmap_ros);

};  // namespace autoscrubber
