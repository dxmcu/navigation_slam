/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file bezier_planner.cc
 * @brief global planner using bezier curve
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2016-02-12
 */

#include "service_robot/bezier_planner.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <fixpattern_path/path.h>
#include "service_robot/bezier.h"

namespace service_robot {

void GenerateInPlaceRotationPath(std::vector<fixpattern_path::PathPoint>* bezier_path,
                                 geometry_msgs::Pose pose, double start_yaw, double goal_yaw) {
  double angle_diff = angles::shortest_angular_distance(start_yaw, goal_yaw);

  // goes the shortest way
  double current_yaw = start_yaw;
  while (angles::shortest_angular_distance(current_yaw, goal_yaw) > 0.5) {
    fixpattern_path::PathPoint p(pose);
    tf::Quaternion q;
    q.setRPY(0, 0, current_yaw);
    tf::quaternionTFToMsg(q, p.orientation);
    p.highlight = 0.7;
    p.max_vel = 0.35;
    p.corner_struct.corner_point = true;
    p.corner_struct.theta_out = goal_yaw;
    bezier_path->push_back(p);

    current_yaw = current_yaw + (angle_diff > 0 ? 1 : -1) * 0.5;
    if (current_yaw > M_PI) current_yaw -= 2 * M_PI;
    if (current_yaw <= M_PI) current_yaw += 2 * M_PI;
  }
  // push last point: goal_yaw
  fixpattern_path::PathPoint p(pose);
  tf::Quaternion q;
  q.setRPY(0, 0, goal_yaw);
  tf::quaternionTFToMsg(q, p.orientation);
  p.highlight = 0.7;
  p.max_vel = 0.35;
  p.corner_struct.corner_point = true;
  p.corner_struct.theta_out = goal_yaw;
  bezier_path->push_back(p);
}

void GenerateBezierPlan(std::vector<fixpattern_path::PathPoint>* bezier_path,
                        int pts_count, double x1, double y1, double x2, double y2,
                        double x3, double y3, double x4, double y4,
                        const geometry_msgs::PoseStamped& goal) {
  BezierPoint* bpts = new BezierPoint[pts_count];
  cubic_bezier(pts_count, bpts, x1, y1, x2, y2, x3, y3, x4, y4);
  for (int i = 0; i < pts_count; ++i) {
    fixpattern_path::PathPoint p;
    p.position.x = bpts[i].x;
    p.position.y = bpts[i].y;
    p.highlight = 0.7;
    p.max_vel = 0.35;
    if (i != pts_count - 1) {
      tf::Quaternion temp;
      temp.setRPY(0, 0, fixpattern_path::CalculateDirection(bpts[i + 1].x - bpts[i].x, bpts[i + 1].y - bpts[i].y));
      p.orientation.x = temp.getX();
      p.orientation.y = temp.getY();
      p.orientation.z = temp.getZ();
      p.orientation.w = temp.getW();
    } else {
      p.orientation = goal.pose.orientation;
    }
    bezier_path->push_back(p);
  }

  delete[] bpts;
}

/**
 * @brief Goal as origin, according to the location of start, determin how to
 *        generate bezier curve, if start is in:
 *        1. 1st quadrant: cut some pts from origin fix_path, target yaw is ->
 *        2. 2nd quadrant: cut some pts from origin fix_path, target yaw is ↓
 *        3. 3rd quadrant: target yaw is ↓
 *        4. 4th quadrant: target yaw is ->
 *        5. 5th quadrant: target yaw is ->
 *        6. 6th quadrant: target yaw is ↑
 *        7. 7th quadrant: cut some pts from origin fix_path, target yaw is ↑
 *        8. 8th quadrant: cut some pts from origin fix_path, target yaw is ->
 *
 *        NOTE: in-place rotation point should include multiple angle(radian step
 *        < 0.5), as footprint checker will just cast footprint to one point and
 *        check if safe
 *
 * @param bezier_path output path
 * @param start current pose of vehicle
 * @param goal planner_goal_
 * @param path beginning of fix_path
 *
 * @return new planner_goal_
 */
geometry_msgs::PoseStamped MakeBezierPlan(std::vector<fixpattern_path::PathPoint>* bezier_path,
                                          const geometry_msgs::PoseStamped& start,
                                          const geometry_msgs::PoseStamped& goal,
                                          const std::vector<geometry_msgs::PoseStamped>& path) {
  if (NULL == bezier_path) return goal;

  bezier_path->clear();
  double delta_x = start.pose.position.x - goal.pose.position.x,
         delta_y = start.pose.position.y - goal.pose.position.y;
  double s = sin(tf::getYaw(goal.pose.orientation)),
         c = cos(tf::getYaw(goal.pose.orientation));
  double start_pos_x = c * delta_x + s * delta_y,
         start_pos_y = -s * delta_x + c * delta_y;

  double pose_diff = hypot(start_pos_x, start_pos_y);
  int pts_count = ceil(pose_diff / 0.05) + 1;
  GAUSSIAN_INFO("[BEZIER PLANNER] pts count: %d, x: %lf y: %lf", pts_count, start_pos_x, start_pos_y);

  double start_yaw = tf::getYaw(start.pose.orientation);
  double goal_yaw = tf::getYaw(goal.pose.orientation);
  double angle_diff = angles::shortest_angular_distance(goal_yaw, start_yaw);

  if (start_pos_x >= 0 && start_pos_y >= 0) {
    if (start_pos_x >= start_pos_y) {
      // 1st quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 1st quadrant");
      // cut some points first
      double cut_len = 2 * fabs(start_pos_x) * 1.2;
      geometry_msgs::PoseStamped new_goal = goal;
      double dis_accu = 0.0;
      for (int i = 1; i < path.size(); ++i) {
        new_goal = path[i];
        dis_accu += hypot(path[i].pose.position.x - path[i - 1].pose.position.x, path[i].pose.position.y - path[i - 1].pose.position.y);
        if (dis_accu >= cut_len) break;
      }
      goal_yaw = tf::getYaw(new_goal.pose.orientation);
      angle_diff = angles::shortest_angular_distance(goal_yaw, start_yaw);
      // then rotate
      if (angle_diff > M_PI_4 || angle_diff < -M_PI_2) {
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, goal_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = new_goal.pose.position.x;
      double y4 = new_goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, new_goal);
      return new_goal;
    } else {
      // 2nd quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 2nd quadrant");
      // cut some points first
      double cut_len = 2 * fabs(start_pos_x) * 1.2;
      geometry_msgs::PoseStamped new_goal = goal;
      double dis_accu = 0.0;
      for (int i = 1; i < path.size(); ++i) {
        new_goal = path[i];
        dis_accu += hypot(path[i].pose.position.x - path[i - 1].pose.position.x, path[i].pose.position.y - path[i - 1].pose.position.y);
        if (dis_accu >= cut_len) break;
      }
      goal_yaw = tf::getYaw(new_goal.pose.orientation);
      angle_diff = angles::shortest_angular_distance(goal_yaw, start_yaw);
      // then rotate
      if (angle_diff > -M_PI_4) {
        double temp_yaw = goal_yaw - M_PI_2;
        if (temp_yaw <= -M_PI) temp_yaw += 2 * M_PI;
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, temp_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = new_goal.pose.position.x;
      double y4 = new_goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, new_goal);
      return new_goal;
    }
  } else if (start_pos_x < 0 && start_pos_y >= 0) {
    if (-start_pos_x < start_pos_y) {
      // 3rd quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 3rd quadrant");
      if (angle_diff > -M_PI_4) {
        // rotate first
        double temp_yaw = goal_yaw - M_PI_2;
        if (temp_yaw <= -M_PI) temp_yaw += 2 * M_PI;
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, temp_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = goal.pose.position.x;
      double y4 = goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, goal);
      return goal;
    } else {
      // 4th quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 4th quadrant");
      if (angle_diff > M_PI_4 || angle_diff < -M_PI_2) {
        // rotate first
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, goal_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = goal.pose.position.x;
      double y4 = goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, goal);
      return goal;
    }
  } else if (start_pos_x < 0 && start_pos_y < 0) {
    if (-start_pos_x >= -start_pos_y) {
      // 5th quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 5th quadrant");
      if (angle_diff > M_PI_2 || angle_diff < -M_PI_4) {
        // rotate first
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, goal_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = goal.pose.position.x;
      double y4 = goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, goal);
      return goal;
    } else {
      // 6th quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 6th quadrant");
      if (angle_diff < M_PI_4) {
        // rotate first
        double temp_yaw = goal_yaw + M_PI_2;
        if (temp_yaw > M_PI) temp_yaw -= 2 * M_PI;
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, temp_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = goal.pose.position.x;
      double y4 = goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, goal);
      return goal;
    }
  } else if (start_pos_x >= 0 && start_pos_y < 0) {
    if (start_pos_x < -start_pos_y) {
      // 7th quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 7th quadrant");
      // cut some points first
      double cut_len = 2 * fabs(start_pos_x) * 1.2;
      geometry_msgs::PoseStamped new_goal = goal;
      double dis_accu = 0.0;
      for (int i = 1; i < path.size(); ++i) {
        new_goal = path[i];
        dis_accu += hypot(path[i].pose.position.x - path[i - 1].pose.position.x, path[i].pose.position.y - path[i - 1].pose.position.y);
        if (dis_accu >= cut_len) break;
      }
      goal_yaw = tf::getYaw(new_goal.pose.orientation);
      angle_diff = angles::shortest_angular_distance(goal_yaw, start_yaw);
      // then rotate
      if (angle_diff < M_PI_4) {
        double temp_yaw = goal_yaw + M_PI_2;
        if (temp_yaw > M_PI) temp_yaw -= 2 * M_PI;
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, temp_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = new_goal.pose.position.x;
      double y4 = new_goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, new_goal);
      return new_goal;
    } else {
      // 8th quadrant
      GAUSSIAN_INFO("[BEZIER PLANNER] in 8th quadrant");
      // cut some points first
      double cut_len = 2 * fabs(start_pos_x) * 1.2;
      geometry_msgs::PoseStamped new_goal = goal;
      double dis_accu = 0.0;
      for (int i = 1; i < path.size(); ++i) {
        new_goal = path[i];
        dis_accu += hypot(path[i].pose.position.x - path[i - 1].pose.position.x, path[i].pose.position.y - path[i - 1].pose.position.y);
        if (dis_accu >= cut_len) break;
      }
      goal_yaw = tf::getYaw(new_goal.pose.orientation);
      angle_diff = angles::shortest_angular_distance(goal_yaw, start_yaw);
      // then rotate
      if (angle_diff > M_PI_2 || angle_diff < -M_PI_4) {
        GenerateInPlaceRotationPath(bezier_path, start.pose, start_yaw, goal_yaw);
        start_yaw = bezier_path->back().corner_struct.theta_out;
      }
      double x1 = start.pose.position.x;
      double y1 = start.pose.position.y;
      double x2 = x1 + pose_diff / 2.0 * cos(start_yaw);
      double y2 = y1 + pose_diff / 2.0 * sin(start_yaw);
      double x4 = new_goal.pose.position.x;
      double y4 = new_goal.pose.position.y;
      double x3 = x4 - pose_diff / 2.0 * cos(goal_yaw);
      double y3 = y4 - pose_diff / 2.0 * sin(goal_yaw);
      GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, new_goal);
      return new_goal;
    }
  } else {
    // not possible
    GAUSSIAN_ERROR("[BEZIER PLANNER] something's going wrong");
  }
}

/**
 * @brief Goal as origin, according to the yaw_diff bettwen start and goal, 
 *        determin curve start points 
 * @param bezier_path output path
 * @param start current pose of vehicle
 * @param goal planner_goal_
 *
 * @return if success, true; else false 
 */
bool MakeBezierPlan(std::vector<fixpattern_path::PathPoint>* bezier_path,
                                          const geometry_msgs::PoseStamped& start,
                                          const geometry_msgs::PoseStamped& goal,
                                          bool stop_needed) {
  if (NULL == bezier_path) return false;

  bezier_path->clear();

  double delta_x = start.pose.position.x - goal.pose.position.x,
         delta_y = start.pose.position.y - goal.pose.position.y;

  double pose_diff = hypot(delta_x, delta_y);
  int pts_count = ceil(pose_diff / 0.05) + 1;
  GAUSSIAN_INFO("[BEZIER PLANNER] pts count: %d, x: %lf y: %lf", pts_count, delta_x, delta_y);

  double start_yaw = tf::getYaw(start.pose.orientation);
  double goal_yaw = tf::getYaw(goal.pose.orientation);
  double angle_diff = angles::shortest_angular_distance(goal_yaw, start_yaw);

  // get start_yaw based on angle_diff bettwen start and goal 
  if (stop_needed) {
    if (angle_diff > M_PI / 18.0) {
      start_yaw = goal_yaw;
    } else if (angle_diff < -M_PI_2) {
      start_yaw = goal_yaw - M_PI_2;
    }
  }
  double x1 = start.pose.position.x;
  double y1 = start.pose.position.y;
  double x2 = x1 + pose_diff / 5.0 * cos(start_yaw);
  double y2 = y1 + pose_diff / 5.0 * sin(start_yaw);
  double x4 = goal.pose.position.x;
  double y4 = goal.pose.position.y;
  double x3 = x4 - pose_diff / 3.0 * cos(goal_yaw);
  double y3 = y4 - pose_diff / 3.0 * sin(goal_yaw);
  GenerateBezierPlan(bezier_path, pts_count, x1, y1, x2, y2, x3, y3, x4, y4, goal);
  if (bezier_path->size() > 0) {
    return true;  
  } else {
    return false;  
  }
}

};  // namespace service_robot
