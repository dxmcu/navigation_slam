/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file footprint_checker.h
 * @brief footprint checker
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-10-20
 */

#ifndef AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_FOOTPRINT_CHECKER_H_
#define AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_FOOTPRINT_CHECKER_H_

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/cost_values.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace autoscrubber {

/**
 * @class FootprintChecker
 * @brief A class that implements the WorldModel interface to provide grid
 * based collision checks for the trajectory controller using the costmap.
 */
class FootprintChecker {
 public:
  /**
   * @brief  Constructor for the FootprintChecker
   * @param costmap The costmap that should be used
   * @return
   */
  explicit FootprintChecker(const costmap_2d::Costmap2D& costmap);

  /**
   * @brief  Destructor for the world model
   */
  virtual ~FootprintChecker() { }

//  double RecoveryCircleCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, geometry_msgs::PoseStamped* goal_pose);
  double RecoveryCircleCost(const geometry_msgs::PoseStamped& current_pos, const std::vector<geometry_msgs::Point>& footprint_spec, geometry_msgs::PoseStamped* goal_pose);

  double CircleCenterCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& circle_center_points) {
    double cos_th = cos(theta);
    double sin_th = sin(theta);

    double ret = 0.0;
    for (unsigned int i = 0; i < circle_center_points.size(); ++i) {
      double new_x = x + (circle_center_points[i].x * cos_th - circle_center_points[i].y * sin_th);
      double new_y = y + (circle_center_points[i].x * sin_th + circle_center_points[i].y * cos_th);

      unsigned int cell_x, cell_y;
      if (!costmap_.worldToMap(new_x, new_y, cell_x, cell_y)) {
//        return 0.0;
        return -1.0;
      }
      unsigned char cost = costmap_.getCost(cell_x, cell_y);
      if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return -1.0;
      }
      if (ret < cost) ret = cost;
    }
    return ret;
  }

  double FootprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0, double circumscribed_radius = 0.0) {
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    std::vector<geometry_msgs::Point> oriented_footprint;
    for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
      geometry_msgs::Point new_pt;
      new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
      new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = x;
    robot_position.y = y;

    if (inscribed_radius == 0.0) {
      costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
    }

    return FootprintCost(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius);
  }

  /**
   * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
   * @param  position The position of the robot in world coordinates
   * @param  footprint The specification of the footprint of the robot in world coordinates
   * @param  inscribed_radius The radius of the inscribed circle of the robot
   * @param  circumscribed_radius The radius of the circumscribed circle of the robot
   * @return Positive if all the points lie outside the footprint, negative otherwise
   */
  double FootprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
                       double inscribed_radius, double circumscribed_radius);

 private:
  /**
   * @brief  Rasterizes a line in the costmap grid and checks for collisions
   * @param x0 The x position of the first cell in grid coordinates
   * @param y0 The y position of the first cell in grid coordinates
   * @param x1 The x position of the second cell in grid coordinates
   * @param y1 The y position of the second cell in grid coordinates
   * @return A positive cost for a legal line... negative otherwise
   */
  double LineCost(int x0, int x1, int y0, int y1);

  /**
   * @brief  Checks the cost of a point in the costmap
   * @param x The x position of the point in cell coordinates
   * @param y The y position of the point in cell coordinates
   * @return A positive cost for a legal point... negative otherwise
   */
  double PointCost(int x, int y);

  const costmap_2d::Costmap2D& costmap_;  ///< @brief Allows access of costmap obstacle information
};

};  // namespace autoscrubber

#endif  // AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_FOOTPRINT_CHECKER_H_
