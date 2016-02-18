/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file environment.h
 * @brief environment for adstar planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-07
 */

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_ENVIRONMENT_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_ENVIRONMENT_H_

#include <vector>
#include <algorithm>
#include "search_based_global_planner/utils.h"
#include "search_based_global_planner/pointer_heap.h"
#include "search_based_global_planner/motion_primitive_manager.h"

#define NUM_OF_HEURISTIC_SEARCH_DIR 16

namespace search_based_global_planner {

#define XYTHETA2INDEX(x, y, theta) (theta + x * num_of_angles_ + y * size_x_ * num_of_angles_)

typedef struct {
  int x;
  int y;
  int heuristic;
  unsigned char cost;

  int heap_index;
  int visited_iteration;
} EnvironmentEntry2D;

typedef struct _EnvironmentEntry3D {
  int x;
  int y;
  uint8_t theta;

  uint64_t g;
  uint64_t rhs;

  _EnvironmentEntry3D* best_next_entry;

  int heap_index;
  int visited_iteration;  // assign to iteration number
                          // so if it equals to iteration_number, this
                          // entry is visited before
  int closed_iteration;   // assign to interation number
                          // so if it equals to iteration_number, this
                          // entry is closed in this iteration
  struct _Key {
    uint64_t k1;
    uint64_t k2;

    _Key() : k1(0), k2(0) { }
    bool operator<(const _Key& k) const {
      return k1 < k.k1 || (k1 == k.k1 && k2 < k.k2);
    }
    bool operator>=(const _Key& k) const {
      return k1 >= k.k1 && (k1 != k.k1 || k2 >= k.k2);
    }
  } key;

  _Key ComputeKey(double eps_satisfied, int heuristic) {
    if (g > rhs) {
      key.k1 = rhs + eps_satisfied * heuristic;
      key.k2 = rhs;
    } else {
      key.k1 = g + heuristic;
      key.k2 = g;
    }
    return key;
  }
  bool operator==(const _EnvironmentEntry3D& e) const { return x == e.x && y == e.y && theta == e.theta; }
  bool operator!=(const _EnvironmentEntry3D& e) const { return !operator==(e); }
} EnvironmentEntry3D;

class HeuristicComparator {
 public:
  bool operator()(const EnvironmentEntry2D* lhs, const EnvironmentEntry2D* rhs) const {
    return lhs->heuristic < rhs->heuristic;
  }
};

class Environment {
  friend class MPrimitiveManager;
 public:
  Environment(unsigned int size_x, unsigned int size_y, double resolution,
              unsigned char obstacle_threshold, unsigned char cost_inscribed_thresh,
              unsigned char cost_possibly_circumscribed_thresh, double nominalvel_mpersec,
              double timetoturn45degsinplace_secs,
              const std::vector<XYPoint>& footprint, const std::vector<XYPoint>& circle_center,
              int num_of_angles, int num_of_prims_per_angle, int forward_cost_mult,
              int forward_and_turn_cost_mult, int turn_in_place_cost_mult);
  ~Environment();

  void ReInitialize();
  EnvironmentEntry3D* SetStart(double x_m, double y_m, double theta_rad);
  EnvironmentEntry3D* SetGoal(double x_m, double y_m, double theta_rad);
  void AdjustStartEntry(int* x, int* y, int* theta);
  void UpdateCost(unsigned int x, unsigned int y, unsigned char cost);
  void GetPreds(EnvironmentEntry3D* entry, std::vector<EnvironmentEntry3D*>* pred_entries, std::vector<int>* costs);
  void GetSuccs(EnvironmentEntry3D* entry, std::vector<EnvironmentEntry3D*>* succ_entries,
                std::vector<int>* costs, std::vector<Action*>* actions = NULL);
  void EnsureHeuristicsUpdated();

  EnvironmentEntry3D* GetEnvEntry(unsigned int x, unsigned int y, unsigned int theta) {
    if (!IsWithinMapCell(x, y) || theta >= num_of_angles_) return NULL;
    return &env_[x][y][theta];
  }
  unsigned char GetCost(unsigned int x, unsigned int y) {
    if (!IsWithinMapCell(x, y)) return obstacle_threshold_;
    return grid_[x][y].cost;
  }
  int GetHeuristic(unsigned int x, unsigned int y) {
    int h_2d = (grid_[x][y].visited_iteration == iteration_ &&
                grid_[x][y].heuristic <= largest_computed_heuristic_) ? grid_[x][y].heuristic : largest_computed_heuristic_;
    // use millimeters, so multiply by 1000
    int h_euclid = static_cast<int>(1000 * resolution_ * hypot(static_cast<int>(start_cell_.x) - static_cast<int>(x), static_cast<int>(start_cell_.y) - static_cast<int>(y)));
    return static_cast<int>(std::max(h_2d, h_euclid) / nominalvel_mpersec_);
  }
  std::vector<XYThetaCell>& GetAffectedPredCells() { return affected_pred_cells_; }

 private:
  bool IsWithinMapCell(int x, int y) {
    return (x >= 0 && y >= 0 && x < size_x_ && y < size_y_);
  }
  bool IsCellValid(int x, int y) {
    return IsWithinMapCell(x, y) && grid_[x][y].cost < obstacle_threshold_;
  }
  bool IsCellSafe(int x, int y) {
    return IsWithinMapCell(x, y) && grid_[x][y].cost < cost_inscribed_thresh_;
  }
  bool IsValidConfiguration(int cell_x, int cell_y, int theta);
  void ComputeDXY();
  int ComputeActionCost(int source_x, int source_y, int source_theta, Action* action);
  bool ComputeHeuristicValues();

 private:
  unsigned int size_x_;
  unsigned int size_y_;
  unsigned int size_dir_;

  XYThetaCell start_cell_;
  XYThetaCell goal_cell_;

  EnvironmentEntry3D*** env_;
  EnvironmentEntry2D** grid_;

  double resolution_;
  unsigned char obstacle_threshold_;
  unsigned char cost_inscribed_thresh_;
  unsigned char cost_possibly_circumscribed_thresh_;
  std::vector<XYPoint> footprint_;
  std::vector<XYPoint> circle_center_;

  // for computing heuristic
  bool need_to_update_heuristics_;
  int iteration_;
  int largest_computed_heuristic_;
  int heuristic_dx_[NUM_OF_HEURISTIC_SEARCH_DIR];
  int heuristic_dy_[NUM_OF_HEURISTIC_SEARCH_DIR];
  // the intermediate cells through which the actions go
  // (for all the ones with the first index <=7, there are none(they go to direct neighbors) -> initialized to -1)
  int heuristic_dx0_intersects_[NUM_OF_HEURISTIC_SEARCH_DIR];
  int heuristic_dx1_intersects_[NUM_OF_HEURISTIC_SEARCH_DIR];
  int heuristic_dy0_intersects_[NUM_OF_HEURISTIC_SEARCH_DIR];
  int heuristic_dy1_intersects_[NUM_OF_HEURISTIC_SEARCH_DIR];
  // distances of transitions
  int heuristic_dxy_distance_mm_[NUM_OF_HEURISTIC_SEARCH_DIR];
  PointerHeap<EnvironmentEntry2D*, HeuristicComparator> grid_open_;

  // for motion primitive
  MPrimitiveManager* mprim_manager_;
  double nominalvel_mpersec_, timetoturn45degsinplace_secs_;
  int num_of_angles_, num_of_prims_per_angle_;
  int forward_cost_mult_, forward_and_turn_cost_mult_, turn_in_place_cost_mult_;
  std::vector<std::vector<Action*>> actions_;
  std::vector<std::vector<Action*>> pred_actions_;
  std::vector<XYThetaCell> affected_succ_cells_;  // arrays of states whose outgoing actions cross cell 0,0
  std::vector<XYThetaCell> affected_pred_cells_;  // arrays of states whose incoming actions cross cell 0,0
};

};  // namespace search_based_global_planner

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_ENVIRONMENT_H_
