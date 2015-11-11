/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file search_based_global_planner.h
 * @brief search-based global planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-04
 */

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_SEARCH_BASED_GLOBAL_PLANNER_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_SEARCH_BASED_GLOBAL_PLANNER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <fixpattern_path/path.h>
#include <vector>
#include <queue>
#include <string>

#include "search_based_global_planner/environment.h"
#include "search_based_global_planner/pointer_heap.h"

namespace search_based_global_planner {

class KeyComparator {
 public:
  bool operator()(const EnvironmentEntry3D* lhs, const EnvironmentEntry3D* rhs) const {
    return lhs->key < rhs->key;
  }
};

class SearchBasedGlobalPlanner {
 public:
  /**
   * @brief  Default constructor for the PlannerCore object
   */
  SearchBasedGlobalPlanner();
  /**
   * @brief  Default deconstructor for the PlannerCore object
   */
  ~SearchBasedGlobalPlanner();
  /**
   * @brief  Initialization function for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  // bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
  //               std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT
  bool makePlan(geometry_msgs::PoseStamped start,
                geometry_msgs::PoseStamped goal,
                std::vector<geometry_msgs::PoseStamped>& plan,
                fixpattern_path::Path& path);  // NOLINT

 private:
  void RecomputeRHSVal(EnvironmentEntry3D* entry);
  void UpdateSetMembership(EnvironmentEntry3D* entry);
  void UpdateStateOfOverConsist(EnvironmentEntry3D* entry);
  void UpdateStateOfUnderConsist(EnvironmentEntry3D* entry);
  bool ComputeOrImprovePath();
  bool search(std::vector<XYThetaPoint>* point_path, std::vector<IntermPointStruct>* path_info);
  void GetEntryPath(std::vector<EnvironmentEntry3D*>* entry_path);
  void PublishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  void GetPointPathFromEntryPath(const std::vector<EnvironmentEntry3D*>& entry_path,
                                 std::vector<XYThetaPoint>* point_path, std::vector<IntermPointStruct>* path_info);
  void ReInitializeSearchEnvironment();
  unsigned char TransformCostmapCost(unsigned char cost);
  bool CostsChanged(const std::vector<XYCell>& changed_cells);
  bool ReadCircleCenterFromParams(ros::NodeHandle& nh, std::vector<XYPoint>* points);

 private:
  costmap_2d::Costmap2DROS* costmap_ros_;

  Environment* env_;
  EnvironmentEntry3D* start_entry_;
  EnvironmentEntry3D* goal_entry_;
  double resolution_;
  bool need_to_reinitialize_environment_;
  int force_scratch_limit_;
  unsigned char lethal_cost_;
  unsigned char inscribed_inflated_cost_;
  unsigned char cost_multiplier_;
  int map_size_;
  unsigned int size_dir_;

  // for ADStar
  std::set<EnvironmentEntry3D*> inconsist_;
  PointerHeap<EnvironmentEntry3D*, KeyComparator> open_;
  unsigned int environment_iteration_, iteration_;
  double allocated_time_, start_time_;
  double initial_epsilon_, eps_, epsilon_satisfied_;

  ros::Publisher plan_pub_;
  bool initialized_;
};

};  // namespace search_based_global_planner

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_SEARCH_BASED_GLOBAL_PLANNER_H_
