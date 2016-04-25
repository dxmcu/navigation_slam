/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
* @file motion_primitive_manager.h
* @brief manager of motion primitive
* @author cameron<chenkan@gs-robot.com>
* @version 1.0.0.0
* @date 2015-09-06
*/

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_MOTION_PRIMITIVE_MANAGER_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_MOTION_PRIMITIVE_MANAGER_H_

#include <vector>
#include <gslib/gaussian_debug.h>
#include "search_based_global_planner/utils.h"

namespace search_based_global_planner {

class Environment;

class MPrimitiveManager {
 public:
  explicit MPrimitiveManager(Environment* env);
  ~MPrimitiveManager();
  void GenerateMotionPrimitives();

 private:
  Action* CreateAction(const MotionPrimitive& mprim);
  void ComputeReplanningDataForAction(Action* action);

 private:
  Environment* env_;
  double resolution_;
  int num_of_angles_, num_of_prims_per_angle_;
  double nominalvel_mpersec_, timetoturn45degsinplace_secs_;
  int forward_cost_mult_, forward_and_turn_cost_mult_, turn_in_place_cost_mult_;
  std::vector<XYPoint> footprint_;
  std::vector<XYPoint> circle_center_;

  std::vector<MotionPrimitive> mprims_;
};

};  // namespace search_based_global_planner

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_MOTION_PRIMITIVE_MANAGER_H_
