/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file environment.cc
 * @brief environment for adstar planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-07
 */

#include "search_based_global_planner/environment.h"

#include <ros/ros.h>

namespace search_based_global_planner {

Environment::Environment(unsigned int size_x, unsigned int size_y, double resolution,
                         unsigned char obstacle_threshold, unsigned char cost_inscribed_thresh,
                         unsigned char cost_possibly_circumscribed_thresh, double nominalvel_mpersec,
                         double timetoturn45degsinplace_secs,
                         const std::vector<XYPoint>& footprint, const std::vector<XYPoint>& circle_center,
                         int num_of_angles, int num_of_prims_per_angle, int forward_cost_mult,
                         int forward_and_turn_cost_mult, int turn_in_place_cost_mult)
    : size_x_(size_x), size_y_(size_y), resolution_(resolution),
      obstacle_threshold_(obstacle_threshold), cost_inscribed_thresh_(cost_inscribed_thresh),
      cost_possibly_circumscribed_thresh_(cost_possibly_circumscribed_thresh),
      nominalvel_mpersec_(nominalvel_mpersec), timetoturn45degsinplace_secs_(timetoturn45degsinplace_secs),
      footprint_(footprint), circle_center_(circle_center), num_of_angles_(num_of_angles),
      num_of_prims_per_angle_(num_of_prims_per_angle),
      forward_cost_mult_(forward_cost_mult), forward_and_turn_cost_mult_(forward_and_turn_cost_mult),
      turn_in_place_cost_mult_(turn_in_place_cost_mult) {
  size_dir_ = num_of_angles_;

  mprim_manager_ = new MPrimitiveManager(this);

  // for computing heuristic
  iteration_ = 0;
  largest_computed_heuristic_ = 0;
  // compute some constance for computing heuristic
  ComputeDXY();

  // create grid_
  grid_ = new EnvironmentEntry2D*[size_x_];
  for (unsigned int i = 0; i < size_x_; ++i) {
    grid_[i] = new EnvironmentEntry2D[size_y_];
    for (unsigned int j = 0; j < size_y_; ++j) {
      grid_[i][j].visited_iteration = -1;
      grid_[i][j].x = i;
      grid_[i][j].y = j;
      grid_[i][j].heap_index = -1;
      grid_[i][j].heuristic = INFINITECOST;
    }
  }

  // create environment entry
  env_ = new EnvironmentEntry3D**[size_x_];
  for (unsigned int i = 0; i < size_x_; ++i) {
    env_[i] = new EnvironmentEntry3D*[size_y_];
    for (unsigned int j = 0; j < size_y_; ++j) {
      env_[i][j] = new EnvironmentEntry3D[size_dir_];
      for (unsigned int k = 0; k < size_dir_; ++k) {
        env_[i][j][k].x = i;
        env_[i][j][k].y = j;
        env_[i][j][k].theta = k;
        env_[i][j][k].g = INFINITECOST;
        env_[i][j][k].rhs = INFINITECOST;
        env_[i][j][k].best_next_entry = NULL;
        env_[i][j][k].heap_index = -1;
        env_[i][j][k].visited_iteration = -1;
        env_[i][j][k].closed_iteration = -1;
      }
    }
  }

  mprim_manager_->GenerateMotionPrimitives();
}

void Environment::ComputeDXY() {
  // initialize some constants for computing heuristic
  heuristic_dx_[0] = 1;
  heuristic_dy_[0] = 1;
  heuristic_dx0_intersects_[0] = -1;
  heuristic_dy0_intersects_[0] = -1;
  heuristic_dx_[1] = 1;
  heuristic_dy_[1] = 0;
  heuristic_dx0_intersects_[1] = -1;
  heuristic_dy0_intersects_[1] = -1;
  heuristic_dx_[2] = 1;
  heuristic_dy_[2] = -1;
  heuristic_dx0_intersects_[2] = -1;
  heuristic_dy0_intersects_[2] = -1;
  heuristic_dx_[3] = 0;
  heuristic_dy_[3] = 1;
  heuristic_dx0_intersects_[3] = -1;
  heuristic_dy0_intersects_[3] = -1;
  heuristic_dx_[4] = 0;
  heuristic_dy_[4] = -1;
  heuristic_dx0_intersects_[4] = -1;
  heuristic_dy0_intersects_[4] = -1;
  heuristic_dx_[5] = -1;
  heuristic_dy_[5] = 1;
  heuristic_dx0_intersects_[5] = -1;
  heuristic_dy0_intersects_[5] = -1;
  heuristic_dx_[6] = -1;
  heuristic_dy_[6] = 0;
  heuristic_dx0_intersects_[6] = -1;
  heuristic_dy0_intersects_[6] = -1;
  heuristic_dx_[7] = -1;
  heuristic_dy_[7] = -1;
  heuristic_dx0_intersects_[7] = -1;
  heuristic_dy0_intersects_[7] = -1;

  // Note: these actions have to be starting at 8 and through 15, since they
  // get multiplied correspondingly in Dijkstra's search based on index
  heuristic_dx_[8] = 2; heuristic_dy_[8] = 1;
  heuristic_dx0_intersects_[8] = 1; heuristic_dy0_intersects_[8] = 0; heuristic_dx1_intersects_[8] = 1; heuristic_dy1_intersects_[8] = 1;
  heuristic_dx_[9] = 1; heuristic_dy_[9] = 2;
  heuristic_dx0_intersects_[9] = 0; heuristic_dy0_intersects_[9] = 1; heuristic_dx1_intersects_[9] = 1; heuristic_dy1_intersects_[9] = 1;
  heuristic_dx_[10] = -1; heuristic_dy_[10] = 2;
  heuristic_dx0_intersects_[10] = 0; heuristic_dy0_intersects_[10] = 1; heuristic_dx1_intersects_[10] = -1; heuristic_dy1_intersects_[10] = 1;
  heuristic_dx_[11] = -2; heuristic_dy_[11] = 1;
  heuristic_dx0_intersects_[11] = -1; heuristic_dy0_intersects_[11] = 0; heuristic_dx1_intersects_[11] = -1; heuristic_dy1_intersects_[11] = 1;
  heuristic_dx_[12] = -2; heuristic_dy_[12] = -1;
  heuristic_dx0_intersects_[12] = -1; heuristic_dy0_intersects_[12] = 0; heuristic_dx1_intersects_[12] = -1; heuristic_dy1_intersects_[12] = -1;
  heuristic_dx_[13] = -1; heuristic_dy_[13] = -2;
  heuristic_dx0_intersects_[13] = 0; heuristic_dy0_intersects_[13] = -1; heuristic_dx1_intersects_[13] = -1; heuristic_dy1_intersects_[13] = -1;
  heuristic_dx_[14] = 1; heuristic_dy_[14] = -2;
  heuristic_dx0_intersects_[14] = 0; heuristic_dy0_intersects_[14] = -1; heuristic_dx1_intersects_[14] = 1; heuristic_dy1_intersects_[14] = -1;
  heuristic_dx_[15] = 2; heuristic_dy_[15] = -1;
  heuristic_dx0_intersects_[15] = 1; heuristic_dy0_intersects_[15] = 0; heuristic_dx1_intersects_[15] = 1; heuristic_dy1_intersects_[15] = -1;

  // compute distances
  for (unsigned int dind = 0; dind < NUM_OF_HEURISTIC_SEARCH_DIR; dind++) {
    if (heuristic_dx_[dind] != 0 && heuristic_dy_[dind] != 0) {
      if (dind <= 7)
        // the cost of a diagonal move in millimeters
        heuristic_dxy_distance_mm_[dind] = static_cast<int>(resolution_ * 1414);
      else
        // the cost of a move to 1,2 or 2,1 or so on in millimeters
        heuristic_dxy_distance_mm_[dind] = static_cast<int>(resolution_ * 2236);
    } else {
      heuristic_dxy_distance_mm_[dind] = static_cast<int>(resolution_ * 1000);  // the cost of a horizontal move in millimeters
    }
  }
}

Environment::~Environment() {
  delete mprim_manager_;

  // delete actions
  for (unsigned int i = 0; i < num_of_angles_; ++i) {
    for (unsigned int j = 0; j < num_of_prims_per_angle_; ++j) {
      delete actions_[i][j];
      delete pred_actions_[i][j];
      actions_[i][j] = pred_actions_[i][j] = NULL;
    }
  }

  // delete environment
  for (unsigned int i = 0; i < size_x_; ++i) {
    for (unsigned int j = 0; j < size_y_; ++j) {
      delete[] env_[i][j];
    }
    delete env_[i];
  }
  delete env_;

  // delete grid_
  for (unsigned int i = 0; i < size_x_; ++i) {
     delete[] grid_[i];
  }
  delete grid_;
}

void Environment::ReInitialize() {
  // heuristic reinitialize
  iteration_ = 0;
  largest_computed_heuristic_ = 0;
  need_to_update_heuristics_ = true;
  for (unsigned int i = 0; i < size_x_; ++i) {
    for (unsigned int j = 0; j < size_y_; ++j) {
      grid_[i][j].visited_iteration = -1;
      grid_[i][j].heap_index = -1;
      grid_[i][j].heuristic = INFINITECOST;
    }
  }

  // env_ reinitialize
  for (unsigned int i = 0; i < size_x_; ++i) {
    for (unsigned int j = 0; j < size_y_; ++j) {
      for (unsigned int k = 0; k < size_dir_; ++k) {
        env_[i][j][k].g = INFINITECOST;
        env_[i][j][k].rhs = INFINITECOST;
        env_[i][j][k].best_next_entry = NULL;
        env_[i][j][k].heap_index = -1;
        env_[i][j][k].visited_iteration = -1;
        env_[i][j][k].closed_iteration = -1;
      }
    }
  }
}

bool Environment::IsValidConfiguration(int cell_x, int cell_y, int theta) {
  std::set<XYCell> footprint_points;
  XYThetaPoint pose;

  // compute continuous pose
  pose.x = DISCXY2CONT(cell_x, resolution_);
  pose.y = DISCXY2CONT(cell_y, resolution_);
  pose.theta = DiscTheta2Cont(theta, num_of_angles_);

  // compute footprint cells
  Get2DFootprintCells(footprint_, &footprint_points, pose, resolution_);

  // iterate over all footprint cells
  for (const auto& p : footprint_points)
    if (!IsCellValid(p.x, p.y))
      return false;

  return true;
}

EnvironmentEntry3D* Environment::SetGoal(double x_m, double y_m, double theta_rad) {
  int x = CONTXY2DISC(x_m, resolution_);
  int y = CONTXY2DISC(y_m, resolution_);
  int theta = ContTheta2Disc(theta_rad, num_of_angles_);

  if (!IsWithinMapCell(x, y)) {
    ROS_ERROR("[SEARCH BASED GLOBAL PLANNER] trying to set a goal cell %d %d that is outside of map", x, y);
    return NULL;
  }

  if (!IsValidConfiguration(x, y, theta)) {
    ROS_WARN("[SEARCH BASED GLOBAL PLANNER] goal configuration %d %d %d is invalid", x, y, theta);
  }

  // we're using backward search, once start changes, heuristics must be updated
  if (x != goal_cell_.x || y != goal_cell_.y || theta != goal_cell_.theta) {
    need_to_update_heuristics_ = true;
  }

  goal_cell_.x = x;
  goal_cell_.y = y;
  goal_cell_.theta = theta;

  return &env_[x][y][theta];
}

EnvironmentEntry3D* Environment::SetStart(double x_m, double y_m, double theta_rad) {
  int x = CONTXY2DISC(x_m, resolution_);
  int y = CONTXY2DISC(y_m, resolution_);
  int theta = ContTheta2Disc(theta_rad, num_of_angles_);

  if (!IsWithinMapCell(x, y)) {
    ROS_ERROR("[SEARCH BASED GLOBAL PLANNER] trying to set a start cell %d %d that is outside of map", x, y);
    return NULL;
  }

  if (!IsValidConfiguration(x, y, theta)) {
    ROS_WARN("[SEARCH BASED GLOBAL PLANNER] start configuration %d %d %d is invalid", x, y, theta);
  }

  // we're using backward search, once start changes, heuristics must be updated
  if (x != start_cell_.x || y != start_cell_.y || theta != start_cell_.theta) {
    need_to_update_heuristics_ = true;
  }

  // set start_cell_
  start_cell_.x = x;
  start_cell_.y = y;
  start_cell_.theta = theta;

  return &env_[x][y][theta];
}

void Environment::UpdateCost(unsigned int x, unsigned int y, unsigned char cost) {
  grid_[x][y].cost = cost;

  need_to_update_heuristics_ = true;
}

void Environment::EnsureHeuristicsUpdated() {
  if (need_to_update_heuristics_) {
    ComputeHeuristicValues();
    need_to_update_heuristics_ = false;
  }
}

bool Environment::ComputeHeuristicValues() {
  EnvironmentEntry2D* search_exp_space_ = NULL;
  EnvironmentEntry2D* search_pred_space_ = NULL;

  // closed = 0
  iteration_++;

  // clear the heap
  grid_open_.clear();

  // initialize the start and goal states
  search_exp_space_ = &grid_[start_cell_.x][start_cell_.y];
  EnvironmentEntry2D* search_goal_space = &grid_[goal_cell_.x][goal_cell_.y];
  search_exp_space_->heuristic = search_goal_space->heuristic = INFINITECOST;
  search_exp_space_->visited_iteration = search_goal_space->visited_iteration = iteration_;

  // seed the search
  search_exp_space_->heuristic = 0;

  grid_open_.push(search_exp_space_);

  // set the termination condition
  const float term_factor = 0.5;

  char *grid_closed = (char*)calloc(1, size_x_ * size_y_);  // NOLINT

  // the main repetition of expansions
  search_exp_space_ = grid_open_.top();
  while (!grid_open_.empty() &&
         std::min(INFINITECOST, search_goal_space->heuristic) > term_factor * search_exp_space_->heuristic) {
    grid_open_.pop();

    int exp_x = search_exp_space_->x;
    int exp_y = search_exp_space_->y;

    // close the state
    grid_closed[exp_x + size_x_ * exp_y] = 1;

    // iterate over successors
    unsigned char exp_cost = grid_[exp_x][exp_y].cost;
    for (int dir = 0; dir < NUM_OF_HEURISTIC_SEARCH_DIR; dir++) {
      int new_x = exp_x + heuristic_dx_[dir];
      int new_y = exp_y + heuristic_dy_[dir];

      // make sure it is inside the map and has no obstacle
      if (!IsWithinMapCell(new_x, new_y)) continue;

      if (grid_closed[new_x + size_x_ * new_y] == 1) continue;

      // compute the cost
      unsigned char map_cost = std::max(grid_[new_x][new_y].cost, exp_cost);

      if (dir > 7) {
        // check two more cells through which the action goes
        map_cost = std::max(map_cost, grid_[exp_x + heuristic_dx0_intersects_[dir]][exp_y + heuristic_dy0_intersects_[dir]].cost);
        map_cost = std::max(map_cost, grid_[exp_x + heuristic_dx1_intersects_[dir]][exp_y + heuristic_dy1_intersects_[dir]].cost);
      }

      if (map_cost >= obstacle_threshold_)  // obstacle encountered
        continue;
      int cost = (map_cost + 1) * heuristic_dxy_distance_mm_[dir];

      // get the predecessor
      search_pred_space_ = &grid_[new_x][new_y];

      // update predecessor if necessary
      if (search_pred_space_->visited_iteration != iteration_ || search_pred_space_->heuristic > cost + search_exp_space_->heuristic) {
        search_pred_space_->visited_iteration = iteration_;
        search_pred_space_->heuristic = std::min(INFINITECOST, cost + search_exp_space_->heuristic);

        if (PTRHEAP_OK != grid_open_.contain(search_pred_space_))
          grid_open_.push(search_pred_space_);
        else
          grid_open_.make_heap();
      }
    }

    // get the next state for expansion
    search_exp_space_ = grid_open_.top();
  }

  // set lower bounds for the remaining states
  if (!grid_open_.empty())
    largest_computed_heuristic_ = grid_open_.top()->heuristic;
  else
    largest_computed_heuristic_ = INFINITECOST;

  delete[] grid_closed;

  return true;
}

int Environment::ComputeActionCost(int source_x, int source_y, int source_theta, Action* action) {
  XYCell cell;
  XYThetaCell interm_cell;

  int end_x = source_x + action->dx;
  int end_y = source_y + action->dy;

  // TODO(chenkan): - go over bounding box (minpt and maxpt) to test validity and skip
  // testing boundaries below, also order intersect cells so that the four
  // farthest pts go first

  if (!IsCellSafe(source_x, source_y)) return INFINITECOST;
  if (!IsCellSafe(end_x, end_y)) return INFINITECOST;

  // need to iterate over discretized center cells and compute cost based on them
  unsigned char max_cost = 0;
  for (unsigned int i = 0; i < action->interm_cells_3d.size(); ++i) {
    interm_cell = action->interm_cells_3d.at(i);
    interm_cell.x = interm_cell.x + source_x;
    interm_cell.y = interm_cell.y + source_y;

    if (!IsCellSafe(interm_cell.x, interm_cell.y)) return INFINITECOST;

    max_cost = std::max(max_cost, grid_[interm_cell.x][interm_cell.y].cost);
  }

  // check collisions that for the particular circle_center orientation along the action
  if (max_cost >= cost_possibly_circumscribed_thresh_ && circle_center_.size() > 1) {
    for (unsigned int i = 0; i < action->circle_center_cells.size(); ++i) {
      // get the cell in the map
      cell = action->circle_center_cells.at(i);
      cell.x = cell.x + source_x;
      cell.y = cell.y + source_y;

      // check validity
      if (!IsCellSafe(cell.x, cell.y)) return INFINITECOST;
    }
  }

  // to ensure consistency of h2D:
  max_cost = std::max(max_cost, grid_[source_x][source_y].cost);
  max_cost = std::max(max_cost, grid_[end_x][end_y].cost);

  return action->cost * (static_cast<int>(max_cost) + 1);  // use cell cost as multiplicative factor
}

void Environment::GetPreds(EnvironmentEntry3D* entry, std::vector<EnvironmentEntry3D*>* pred_entries, std::vector<int>* costs) {
  // TODO(chenkan): to support tolerance, need:
  //  a) generate preds for goal state based on all possible goal state variable settings,
  //  b) change goal check condition in gethashentry c) change
  //    getpredsofchangedcells and getsuccsofchangedcells functions

  // for performance remove this, none of the three could be NULL
  // if (entry == NULL || pred_entries == NULL || costs == NULL) return;

  // clear the successor array
  pred_entries->clear();
  costs->clear();
  pred_entries->reserve(num_of_prims_per_angle_);
  costs->reserve(num_of_prims_per_angle_);

  // iterate through actions
  std::vector<Action*>* action_list = &pred_actions_[static_cast<unsigned int>(entry->theta)];
  Action* action = NULL;
  int pred_x, pred_y, pred_theta, cost;
  for (unsigned int aind = 0; aind < action_list->size(); aind++) {
    action = action_list->at(aind);

    pred_x = entry->x - action->dx;
    pred_y = entry->y - action->dy;
    pred_theta = action->start_theta;

    // get cost
    cost = ComputeActionCost(pred_x, pred_y, pred_theta, action);
    if (cost >= INFINITECOST) continue;

    pred_entries->push_back(&env_[pred_x][pred_y][pred_theta]);
    costs->push_back(cost);
  }
}

void Environment::GetSuccs(EnvironmentEntry3D* entry, std::vector<EnvironmentEntry3D*>* succ_entries,
                           std::vector<int>* costs, std::vector<Action*>* actions) {
  // for performance
  // if (entry == NULL || succ_entries == NULL || costs == NULL) return;
  if (entry == NULL) return;

  if (actions != NULL) {
    actions->clear();
    actions->reserve(num_of_prims_per_angle_);
  }

  // clear the successor array
  succ_entries->clear();
  costs->clear();
  succ_entries->reserve(num_of_prims_per_angle_);
  costs->reserve(num_of_prims_per_angle_);

  // goal state should be absorbing
  if (entry->x == goal_cell_.x && entry->y == goal_cell_.y && entry->theta == goal_cell_.theta) return;

  // iterate through actions
  Action* action = NULL;
  int new_x, new_y, new_theta, cost;
  for (int aind = 0; aind < num_of_prims_per_angle_; aind++) {
    action = actions_[static_cast<unsigned int>(entry->theta)][aind];
    new_x = entry->x + action->dx;
    new_y = entry->y + action->dy;
    new_theta = NORMALIZEDISCTHETA(action->end_theta, num_of_angles_);

    // get cost
    cost = ComputeActionCost(entry->x, entry->y, entry->theta, action);
    if (cost >= INFINITECOST) continue;

    succ_entries->push_back(&env_[new_x][new_y][new_theta]);
    costs->push_back(cost);
    if (actions != NULL) actions->push_back(action);
  }
}

};  // namespace search_based_global_planner
