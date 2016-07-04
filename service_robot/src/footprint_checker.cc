/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file footprint_checker.cc
 * @brief footprint checker
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-10-20
 */

#include "footprint_checker.h"
#include <line_iterator.h>
#include <tf/message_filter.h>
#include <algorithm>
#include <vector>

namespace service_robot {

FootprintChecker::FootprintChecker(const costmap_2d::Costmap2D* costmap) : costmap_(costmap) { }

void FootprintChecker::setStaticCostmap(costmap_2d::Costmap2DROS* costmap_ros, bool use_static_costmap) {
  if (!use_static_costmap) {
    costmap_ = costmap_ros->getCostmap();
    GAUSSIAN_INFO("[Footprint Check] take normal costmap!");
  } else {
    costmap_ = costmap_ros->getStaticCostmap();
    GAUSSIAN_INFO("[Footprint Check] take static costmap!");
  }
}

double FootprintChecker::FootprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
                                       double inscribed_radius, double circumscribed_radius) {
  // used to put things into grid coordinates
  unsigned int cell_x, cell_y;

  if (inscribed_radius == 0.0 || circumscribed_radius == 0.0) {
    costmap_2d::calculateMinAndMaxDistances(footprint, inscribed_radius, circumscribed_radius);
  }
//  GAUSSIAN_INFO("[Footprint Check] inscribed_radius = %lf, circumscribed_radius = %lf", inscribed_radius, circumscribed_radius);
  // get the cell coord of the center point of the robot
  if (!costmap_->worldToMap(position.x, position.y, cell_x, cell_y)) {
    return -200.0;
  }

  // if number of points in the footprint is less than 3, we'll just assume a circular robot
  if (footprint.size() < 3) {
    unsigned char cost = costmap_->getCost(cell_x, cell_y);
    if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
      return -1.0;
    }
    return cost;
  }

  // now we really have to lay down the footprint in the costmap grid
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;
  double ret_cost = 0.0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!costmap_->worldToMap(footprint[i].x, footprint[i].y, x0, y0)) {
      return -200.0;
    }

    // get the cell coord of the second point
    if (!costmap_->worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return -200.0;
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
  if (!costmap_->worldToMap(footprint.back().x, footprint.back().y, x0, y0)) {
    return -200.0;
  }

  // get the cell coord of the first point
  if (!costmap_->worldToMap(footprint.front().x, footprint.front().y, x1, y1)) {
    return -200.0;
  }

  line_cost = LineCost(x0, x1, y0, y1);
  footprint_cost = std::max(line_cost, footprint_cost);

  if (line_cost < 0) {
      return -1.0;
  }

  // if all line costs are legal... then we can return that the footprint is legal
  return footprint_cost;
}

  double FootprintChecker::RecoveryCircleCost(const geometry_msgs::PoseStamped& current_pos, const std::vector<geometry_msgs::Point>& footprint_spec, geometry_msgs::PoseStamped* goal_pose){

    unsigned char cost;
    double free_theta = 0x7FFFFFFF;
    double inscribed_radius, circumscribed_radius;
    double sample_radius;
    const double sample_theta = M_PI / 8.0;
    const unsigned int sample_theta_num = 2.0 * M_PI / sample_theta + 0.5;  
    unsigned char circle_cost[sample_theta_num * 2]; 
    double x = current_pos.pose.position.x;
    double y = current_pos.pose.position.y;
    costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
    // initalize all as FREE_SPACE
    for (int i = 0; i < sample_theta_num * 2; ++i) circle_cost[i] = costmap_2d::FREE_SPACE; 

    for (int circle_index = 0; circle_index < 3; ++circle_index) {
      sample_radius = (circle_index + 2) * 0.5 * circumscribed_radius;
      for (int theta_index = 0; theta_index < sample_theta_num; ++theta_index) {
        double circle_x = x + sample_radius * cos(theta_index * sample_theta);
        double circle_y = y + sample_radius * sin(theta_index * sample_theta);
        unsigned int cell_x, cell_y;
        if(!costmap_->worldToMap(circle_x, circle_y, cell_x, cell_y)) {
          cost = costmap_2d::NO_INFORMATION;
        } else {
          cost = costmap_->getCost(cell_x, cell_y);
        }
        // taken the largest cost as the cost of this theta_index
        circle_cost[theta_index] = std::max(circle_cost[theta_index], cost); 
        GAUSSIAN_INFO("[Footprint Checker] cost[circle = %d][theta = %d] = %d", circle_index, theta_index, circle_cost[theta_index]);
      }
    }
    
    GAUSSIAN_INFO("[Footprint Checker] sample_theta_num = %d", sample_theta_num);

    // it's a circle, so we have to check sample_theta_num * 2 
    for (int theta_index = 0; theta_index < sample_theta_num; ++theta_index) {
      circle_cost[sample_theta_num + theta_index] = circle_cost[theta_index]; 
      GAUSSIAN_INFO("[Footprint Checker] cost[theta = %d] = %d", theta_index, circle_cost[theta_index]);
    }

    unsigned int free_index = 0;
    // we start from theta_index = 3, to avoid ignoring the second half circle
    for (int theta_index = 3; theta_index < sample_theta_num * 2; ++theta_index) {
      GAUSSIAN_INFO("[Footprint Checker] checking theta_index : %d", theta_index);
      if (circle_cost[theta_index] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        free_index = 1;
        GAUSSIAN_INFO("[Footprint Checker] theta_index : %d is free", theta_index);
        for (int i = theta_index + 1; i < sample_theta_num * 2; ++i) {
          if (circle_cost[i] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)  break;
          GAUSSIAN_INFO("[Footprint Checker] theta_index : %d is free", i);
          ++free_index;
        }
        GAUSSIAN_INFO("[Footprint Checker] free_theta acc_index : %d", free_index);
        if (free_index >= 4) {
          unsigned int free_theta_index =  theta_index + free_index / 2;
          free_theta_index = free_theta_index >= sample_theta_num ? free_theta_index - sample_theta_num : free_theta_index;
          GAUSSIAN_INFO("[Footprint Checker] free_theta_index : %d", free_theta_index);
          free_theta = free_theta_index * sample_theta;
          goal_pose->pose.position.x = x + sample_radius * cos(free_theta);
          goal_pose->pose.position.y = y + sample_radius * sin(free_theta);
          goal_pose->pose.orientation = tf::createQuaternionMsgFromYaw(free_theta);
          break;
        }
        theta_index += free_index;
      }
    }
    GAUSSIAN_INFO("[Footprint Checker] free theta is %lf", free_theta);
    return free_theta;
  }

  double getSign(double t) {
    if (t > 0.000001) 
      return 1.0;
    else if ( t < -0.00001)
      return -1.0;
    else 
      return 0.0;
  }

  double FootprintChecker::BroaderFootprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec, double broader_delta_x, double broader_delta_y) {
    geometry_msgs::Point robot_position;
    robot_position.x = x;
    robot_position.y = y;
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    double footprint_cost = 0.0; 
    double broader_x = broader_delta_x;
    double broader_y = broader_delta_y;
    int step_num = std::max(broader_x / 0.01 + 1, broader_y / 0.01 + 1);
    std::vector<geometry_msgs::Point> broader_footprint;
    for (int j = 0; j <= step_num; ++j) { 
      broader_x = std::max(broader_delta_x - 0.01 * j, 0.0);
      broader_y = std::max(broader_delta_y - 0.01 * j, 0.0);
      for (int i = 0; i < footprint_spec.size(); ++i) {
        geometry_msgs::Point footprint_pt = footprint_spec[i];
        geometry_msgs::Point new_pt;

        new_pt.x = x + ((footprint_pt.x + getSign(footprint_pt.x) * broader_x) * cos_th - (footprint_pt.y + getSign(footprint_pt.y) * broader_y) * sin_th);
//      new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
        new_pt.y = y + ((footprint_pt.x + getSign(footprint_pt.x) * broader_x) * sin_th - (footprint_pt.y + getSign(footprint_pt.y) * broader_y) * cos_th);
//      new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
        broader_footprint.push_back(new_pt);
      }
      double temp_cost = FootprintCost(robot_position, broader_footprint, 0.0, 0.0);
      footprint_cost = std::min(footprint_cost, temp_cost); 
      if (footprint_cost < 0.0) {
        GAUSSIAN_ERROR("[Footprint Checker] BroaderFootprintCost checking failed");
        return footprint_cost;
      }
    }
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
  unsigned char cost = costmap_->getCost(x, y);
  // if the cell is in an obstacle the path is invalid
//  if (cost == costmap_2d::LETHAL_OBSTACLE) {
  if (cost == costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
    return -1.0;
  }

  return cost;
}


};  // namespace service_robot
