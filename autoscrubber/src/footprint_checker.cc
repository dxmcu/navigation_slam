/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file footprint_checker.cc
 * @brief footprint checker
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-10-20
 */

#include "autoscrubber/footprint_checker.h"

#include <fixpattern_local_planner/line_iterator.h>
#include <costmap_2d/cost_values.h>
#include <algorithm>
#include <vector>

namespace autoscrubber {

FootprintChecker::FootprintChecker(const costmap_2d::Costmap2D& costmap) : costmap_(costmap) { }

double FootprintChecker::FootprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
                                       double inscribed_radius, double circumscribed_radius) {
  // used to put things into grid coordinates
  unsigned int cell_x, cell_y;

  // get the cell coord of the center point of the robot
  if (!costmap_.worldToMap(position.x, position.y, cell_x, cell_y)) {
    return 0.0;
  }

  // if number of points in the footprint is less than 3, we'll just assume a circular robot
  if (footprint.size() < 3) {
    unsigned char cost = costmap_.getCost(cell_x, cell_y);
    if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    // if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION)
      return -1.0;
    return cost;
  }

  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!costmap_.worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      return 0.0;
    }

    // get the cell coord of the second point
    if (!costmap_.worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return 0.0;
    }

    line_cost = LineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    // if there is an obstacle that hits the line... we know that we can return false right away
    if (line_cost < 0) {
      return -1.0;
    }
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!costmap_.worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    return 0.0;
  }

  // get the cell coord of the first point
  if (!costmap_.worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    return 0.0;
  }

  line_cost = LineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  if (line_cost < 0) {
    return -1.0;
  }

  // if all line costs are legal... then we can return that the footprint is legal
  return footprint_cost;
}

// calculate the cost of a ray-traced line
double FootprintChecker::LineCost(int x0, int x1, int y0, int y1) {
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (fixpattern_local_planner::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = PointCost(line.getX(), line.getY());  // Score the current point

    if (point_cost < 0)
      return -1.0;

    if (line_cost < point_cost)
      line_cost = point_cost;
  }

  return line_cost;
}

double FootprintChecker::PointCost(int x, int y) {
  unsigned char cost = costmap_.getCost(x, y);
  // if the cell is in an obstacle the path is invalid
  if (cost == costmap_2d::LETHAL_OBSTACLE) {
  // if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
    return -1.0;
  }

  return cost;
}

};  // namespace autoscrubber
