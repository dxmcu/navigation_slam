/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
  use_circle_center_ = false;
}

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys, unsigned char path_cost, unsigned char occ_dis_cost) :
        Expander(p_calc, xs, ys), path_cost_(path_cost), occ_dis_cost_(occ_dis_cost) {
  use_circle_center_ = false;
}

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys, unsigned char path_cost, unsigned char occ_dis_cost, const std::vector<XYPoint>& circle_center_point, double resolution) :
        Expander(p_calc, xs, ys), path_cost_(path_cost), occ_dis_cost_(occ_dis_cost), resolution_(resolution) {
  if(circle_center_point.size() > 1) {
    use_circle_center_ = true;
    circle_center_point_ = circle_center_point;
  } else {
    use_circle_center_ = false;
  }
}

unsigned int AStarExpansion::GetCircleCenterLargestCost(unsigned char* costs, std::vector<XYPoint> circle_center, int current_i, int next_i) {
  unsigned int max_cost = 0;
  double pose_theta;
  int diff_index = next_i - current_i;
  if (diff_index == 1) {
    pose_theta = -M_PI_2; 
  } else if (diff_index == -1) {
    pose_theta = M_PI_2; 
  } else if (diff_index == -ny_) {
    pose_theta = 0; 
  } else if (diff_index == ny_) {
    pose_theta = M_PI; 
  }
  double cth = cos(pose_theta);
  double sth = sin(pose_theta);
  for (int i = 0; i < circle_center.size(); ++i) {
    // find the bounding box of the polygon
    double cx = (cth * circle_center[i].x - sth * circle_center[i].y + (next_i % ny_) * resolution_);
    double cy = (sth * circle_center[i].x + cth * circle_center[i].y + (next_i / ny_) * resolution_);
    int cell_x = static_cast<int>(cx > 0 ? cx / resolution_ + 0.5 : cx / resolution_ - 0.5);  // (int)(cx / res + 0.5 * sign(c);
    int cell_y = static_cast<int>(cy > 0 ? cy / resolution_ + 0.5 : cy / resolution_ - 0.5);  // (int)(cy / res + 0.5);
    int cell_index =  cell_x + cell_y * ny_;
    if (cell_index < 0 || cell_index >= nx_ * ny_) return costmap_2d::NO_INFORMATION;
    int cell_cost = costs[cell_index];
    max_cost = max_cost > cell_cost ? max_cost : cell_cost; 
  }
  return max_cost;
}

bool AStarExpansion::calculatePotentials(costmap_2d::Costmap2DROS* costmap_ros, unsigned char* costs, unsigned char* path_costs,
                                         double start_x, double start_y, double end_x, double end_y, int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costmap_ros, costs, path_costs, potential, potential[i], i, i + 1, end_x, end_y);
        add(costmap_ros, costs, path_costs, potential, potential[i], i, i - 1, end_x, end_y);
        add(costmap_ros, costs, path_costs, potential, potential[i], i, i + nx_, end_x, end_y);
        add(costmap_ros, costs, path_costs, potential, potential[i], i, i - nx_, end_x, end_y);
    }

    return false;
}

void AStarExpansion::add(costmap_2d::Costmap2DROS* costmap_ros, unsigned char* costs, unsigned char* path_costs, float* potential,
                         float prev_potential, int current_i, int next_i, int end_x, int end_y) {
    if (next_i < 0 || next_i >= nx_ * ny_) {
      return;
    }

    if (potential[next_i] < POT_HIGH) {
      return;
    }

    if (costs[next_i] >= lethal_cost_ && (!unknown_ || (unknown_ && costs[next_i] == costmap_2d::NO_INFORMATION))) {
      return;
    }

    if (use_circle_center_ && GetCircleCenterLargestCost(costs, circle_center_point_, current_i, next_i) >= lethal_cost_) {
      return;
    }

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);
    float obstacle_distance = costmap_ros->getObstacleDistance(x, y);
    int occ_cost = (int)(10.0 / obstacle_distance * occ_dis_cost_);
    int next_cost;
    if (path_costs != NULL) {
      next_cost = potential[next_i] + distance * neutral_cost_ + occ_cost + path_costs[next_i] * path_cost_;
    } else {
      next_cost = potential[next_i] + distance * neutral_cost_ + occ_cost;
    }
    queue_.push_back(Index(next_i, next_cost));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner
