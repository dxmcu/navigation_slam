/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file bezier_planner.h
 * @brief global planner using bezier curve
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2016-02-11
 */

#ifndef AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_BEZIER_PLANNER_H_
#define AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_BEZIER_PLANNER_H_

#include <geometry_msgs/PoseStamped.h>
#include <fixpattern_path/path_point.h>
#include <vector>

namespace autoscrubber {

geometry_msgs::PoseStamped MakeBezierPlan(std::vector<fixpattern_path::PathPoint>* bezier_path,
                                          const geometry_msgs::PoseStamped& start,
                                          const geometry_msgs::PoseStamped& goal,
                                          const std::vector<geometry_msgs::PoseStamped>& path);
bool MakeBezierPlan(std::vector<fixpattern_path::PathPoint>* bezier_path,
                                          const geometry_msgs::PoseStamped& start,
                                          const geometry_msgs::PoseStamped& goal,
                                          bool stop_needed);
};  // namespace autoscrubber

#endif  // AUTOSCRUBBER_INCLUDE_AUTOSCRUBBER_BEZIER_PLANNER_H_
