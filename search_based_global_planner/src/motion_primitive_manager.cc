/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file motion_primitive_manager.cc
 * @brief motion primitive manager
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-06
 */

#include "search_based_global_planner/motion_primitive_manager.h"

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <fixpattern_path/path.h>

#include "search_based_global_planner/utils.h"
#include "search_based_global_planner/environment.h"

namespace search_based_global_planner {

MPrimitiveManager::MPrimitiveManager(Environment* env) {
  env_                          = env;
  resolution_                   = env_->resolution_;
  nominalvel_mpersec_           = env_->nominalvel_mpersec_;
  timetoturn45degsinplace_secs_ = env_->timetoturn45degsinplace_secs_;
  num_of_angles_                = env_->num_of_angles_;
  num_of_prims_per_angle_       = env_->num_of_prims_per_angle_;
  forward_cost_mult_            = env_->forward_cost_mult_;
  forward_and_turn_cost_mult_   = env_->forward_and_turn_cost_mult_;
  turn_in_place_cost_mult_      = env_->turn_in_place_cost_mult_;
  footprint_                    = env_->footprint_;
  circle_center_                = env_->circle_center_;
}

MPrimitiveManager::~MPrimitiveManager() { }

void MPrimitiveManager::GenerateMotionPrimitives() {
  // 0 degrees
  std::vector<std::vector<int>> mprim_cell_0;
  mprim_cell_0.resize(num_of_prims_per_angle_);
  // x aligned with the heading of the robot, angles are positive
  // counterclockwise
  // 0 theta change
  mprim_cell_0[0] = {1, 0, 0, forward_cost_mult_};
  mprim_cell_0[1] = {8, 0, 0, forward_cost_mult_};
  mprim_cell_0[2] = {16, 0, 0, forward_cost_mult_};
  // 1/16 theta change
  mprim_cell_0[3] = {8, 1, 1, forward_and_turn_cost_mult_};
  mprim_cell_0[4] = {8, -1, -1, forward_and_turn_cost_mult_};
  // turn in place
  mprim_cell_0[5] = {0, 0, 1, turn_in_place_cost_mult_};
  mprim_cell_0[6] = {0, 0, -1, turn_in_place_cost_mult_};

  // 45 degrees
  std::vector<std::vector<int>> mprim_cell_45;
  mprim_cell_45.resize(num_of_prims_per_angle_);
  // x aligned with the heading of the robot, angles are positive
  // counterclockwise
  // 0 theta change
  mprim_cell_45[0] = {1, 1, 0, forward_cost_mult_};
  mprim_cell_45[1] = {6, 6, 0, forward_cost_mult_};
  mprim_cell_45[2] = {12, 12, 0, forward_cost_mult_};
  // 1/16 theta change
  mprim_cell_45[3] = {5, 7, 1, forward_and_turn_cost_mult_};
  mprim_cell_45[4] = {7, 5, -1, forward_and_turn_cost_mult_};
  // turn in place
  mprim_cell_45[5] = {0, 0, 1, turn_in_place_cost_mult_};
  mprim_cell_45[6] = {0, 0, -1, turn_in_place_cost_mult_};

  // 22.5 degrees
  std::vector<std::vector<int>> mprim_cell_22p5;
  mprim_cell_22p5.resize(num_of_prims_per_angle_);
  // x aligned with the heading of the robot, angles are positive
  // counterclockwise
  // 0 theta change
  mprim_cell_22p5[0] = {2, 1, 0, forward_cost_mult_};
  mprim_cell_22p5[1] = {6, 3, 0, forward_cost_mult_};
  mprim_cell_22p5[2] = {14, 6, 0, forward_cost_mult_};
  // 1/16 theta change
  mprim_cell_22p5[3] = {5, 4, 1, forward_and_turn_cost_mult_};
  mprim_cell_22p5[4] = {7, 2, -1, forward_and_turn_cost_mult_};
  // turn in place
  mprim_cell_22p5[5] = {0, 0, 1, turn_in_place_cost_mult_};
  mprim_cell_22p5[6] = {0, 0, -1, turn_in_place_cost_mult_};

  env_->actions_.resize(num_of_angles_);
  env_->pred_actions_.resize(num_of_angles_);

  // iterate over angles
  for (int angle_index = 0; angle_index < num_of_angles_; ++angle_index) {
    env_->actions_[angle_index].resize(num_of_prims_per_angle_);
    // env_->pred_actions_[angle_index].resize(num_of_prims_per_angle_);
    for (int mprim_index = 0; mprim_index < num_of_prims_per_angle_; ++mprim_index) {
      // current angle
      double current_angle = angle_index * 2 * M_PI / num_of_angles_;
      int current_angle_36000 = round(angle_index * 36000 / num_of_angles_);

      // compute which template to use
      double angle = 0.0;
      std::vector<int> mprim_cell;
      int rotate_direction = 0;
      if (current_angle_36000 % 9000 == 0) {
        mprim_cell = mprim_cell_0[mprim_index];
        angle = current_angle;
        rotate_direction = mprim_cell[2];
      } else if (current_angle_36000 % 4500 == 0) {
        mprim_cell = mprim_cell_45[mprim_index];
        angle = current_angle - 45 * M_PI / 180;
        rotate_direction = mprim_cell[2];
      } else if ((current_angle_36000 - 6750) % 9000 == 0) {
        mprim_cell = mprim_cell_22p5[mprim_index];
        mprim_cell[0] = mprim_cell_22p5[mprim_index][1];   // reverse x and y
        mprim_cell[1] = mprim_cell_22p5[mprim_index][0];   // reverse x and y
        mprim_cell[2] = -mprim_cell_22p5[mprim_index][2];  // reverse the angle as well
        angle = current_angle - 67.5 * M_PI / 180;
        rotate_direction = mprim_cell[2];
      } else if ((current_angle_36000 - 2250) % 9000 == 0) {
        mprim_cell = mprim_cell_22p5[mprim_index];
        angle = current_angle - 22.5 * M_PI / 180;
        rotate_direction = mprim_cell[2];
      } else {
        ROS_ERROR("ERROR: invalid angular resolution. angle = %d", current_angle_36000);
        return;
      }

      XYThetaCell base_end_cell(mprim_cell[0], mprim_cell[1], mprim_cell[2]);
      int cost_mult = mprim_cell[3];

      // now figure out what action will be
      int end_cell_x = round(base_end_cell.x * cos(angle) - base_end_cell.y * sin(angle));
      int end_cell_y = round(base_end_cell.x * sin(angle) + base_end_cell.y * cos(angle));
      int end_cell_theta = (angle_index + base_end_cell.theta) % num_of_angles_;

      XYThetaCell start_cell(0, 0, angle_index);
      XYThetaCell end_cell(end_cell_x, end_cell_y, end_cell_theta);

      // generate intermediate poses (remember they are w.r.t 0,0 (and not centers of the cells)
      int num_of_interm_pts = 10;
      std::vector<XYThetaPoint> interm_pts;
      std::vector<IntermPointStruct> interm_struct;
      interm_pts.resize(num_of_interm_pts);
      interm_struct.resize(num_of_interm_pts);
      XYThetaPoint start_point(0, 0, current_angle);
      XYThetaPoint end_point(end_cell_x * resolution_, end_cell_y * resolution_, end_cell_theta * 2 * M_PI / num_of_angles_);
      if ((end_cell_x == 0 && end_cell_y == 0) || base_end_cell.theta == 0) {  // turn in place or move forward
        for (int i = 0; i < num_of_interm_pts; ++i) {
          interm_pts[i].x = start_point.x + (end_point.x - start_point.x) * i / (num_of_interm_pts - 1);
          interm_pts[i].y = start_point.y + (end_point.y - start_point.y) * i / (num_of_interm_pts - 1);
          double rotation_angle = base_end_cell.theta * 2 * M_PI / num_of_angles_;
          interm_pts[i].theta = fmod(start_point.theta + rotation_angle * i / (num_of_interm_pts - 1), 2 * M_PI);

          // assign interm struct
          if (end_cell_x == 0 && end_cell_y == 0) {
            // interm_struct[i].radius = 0;
            // interm_struct[i].radius = fixpattern_path::Path::MAX_RADIUS;
            interm_struct[i].radius = 0.2;
            interm_struct[i].is_corner = true;
            int temp_theta = end_cell_theta < 0 ? end_cell_theta + num_of_angles_ : end_cell_theta % num_of_angles_;
            interm_struct[i].theta_out = temp_theta < num_of_angles_ / 2 ? temp_theta * 2 * M_PI / num_of_angles_ : (temp_theta - num_of_angles_) * 2 * M_PI / num_of_angles_;
            interm_struct[i].rotate_direction = rotate_direction;
          } else {
            interm_struct[i].radius = fixpattern_path::Path::MAX_RADIUS;
            interm_struct[i].is_corner = false;
            interm_struct[i].theta_out = 0.0;  // shouldn't be used
            interm_struct[i].rotate_direction = 0;
          }
        }
      } else {  // unicycle-based move forward or backward
        Eigen::Matrix2d R;
        R(0, 0) = cos(start_point.theta);
        R(0, 1) = sin(end_point.theta) - sin(start_point.theta);
        R(1, 0) = sin(start_point.theta);
        R(1, 1) = cos(start_point.theta) - cos(end_point.theta);
        // R << cos(start_point.theta), sin(end_point.theta) - sin(start_point.theta),
        //      sin(start_point.theta), cos(start_point.theta) - cos(end_point.theta);
        Eigen::Vector2d v_temp;
        v_temp(0) = end_point.x - start_point.x;
        v_temp(1) = end_point.y - start_point.y;
        Eigen::Vector2d S = R.inverse() * v_temp;
        // length of straight line
        double l = S(0);
        // radius of turn
        double tv_over_rv = S(1); // radius of circle
        double rv = base_end_cell.theta * 2 * M_PI / num_of_angles_ + l / tv_over_rv; 
        // total length
        double tv = tv_over_rv * rv;
        if (l < 0) {
          // ROS_WARN("[SEARCH BASED GLOBAL PLANNER] l = %lf < 0 -> bad action start/end points", l);
          l = 0;
        }
        // generate samples
        for (int i = 0; i < num_of_interm_pts; ++i) {
          double dt = static_cast<double>(i) / (num_of_interm_pts - 1);
          if (dt * tv < l) { // straight length
            interm_pts[i].x = start_point.x + dt * tv * cos(start_point.theta);
            interm_pts[i].y = start_point.y + dt * tv * sin(start_point.theta);
            interm_pts[i].theta = start_point.theta;

            interm_struct[i].radius = fixpattern_path::Path::MAX_RADIUS;
            interm_struct[i].is_corner = false;
            interm_struct[i].theta_out = 0.0;
            interm_struct[i].rotate_direction = 0;
          } else { //circle length
            double dtheta = rv * (dt - l / tv) + start_point.theta; //change of theta
            interm_pts[i].x = start_point.x + l * cos(start_point.theta) + tv_over_rv * (sin(dtheta) - sin(start_point.theta));
            interm_pts[i].y = start_point.y + l * sin(start_point.theta) - tv_over_rv * (cos(dtheta) - cos(start_point.theta));
            interm_pts[i].theta = dtheta;

            interm_struct[i].radius = fabs(tv_over_rv);
            interm_struct[i].is_corner = false;
            interm_struct[i].theta_out = 0.0;
            interm_struct[i].rotate_direction = 0;
          }
        }
        // correct
        Eigen::Vector2d error;
        error(0) = end_point.x - interm_pts[num_of_interm_pts - 1].x;
        error(1) = end_point.y - interm_pts[num_of_interm_pts - 1].y;
        // ROS_INFO("l=%f errx=%f erry=%f", l, error(0), error(1));
        for (int i = 0; i < num_of_interm_pts; ++i) {
          interm_pts[i].x += error(0) * i * 1.0 / (num_of_interm_pts - 1);
          interm_pts[i].y += error(1) * i * 1.0 / (num_of_interm_pts - 1);
        }
      }

      // got one motion primitive
      MotionPrimitive mprim(mprim_index, start_cell, end_cell, cost_mult, interm_pts, interm_struct);
      mprims_.push_back(mprim);

      // we'll transform motion primitives to actions
      Action* action = CreateAction(mprim);
      ComputeReplanningDataForAction(action);
      env_->actions_[angle_index][mprim_index] = action;

      // add to the list of pred actions
      int target_theta_cell = end_cell_theta;
      if (target_theta_cell < 0) target_theta_cell = target_theta_cell + num_of_angles_;
      env_->pred_actions_[target_theta_cell].push_back(action);
    }
  }
}

Action* MPrimitiveManager::CreateAction(const MotionPrimitive& mprim) {
  Action* action = new Action();
  // action index
  action->action_index = mprim.mprim_id;
  // start angle
  action->start_theta = mprim.start_cell.theta;
  // compute dislocation
  action->end_theta = mprim.end_cell.theta;
  action->dx = mprim.end_cell.x;
  action->dy = mprim.end_cell.y;
  // compute and store interm points as well as intersecting cells
  action->intersecting_cells.clear(); //footprint
  action->interm_cells_3d.clear();
  action->interm_pts = mprim.interm_pts;
  action->interm_struct = mprim.interm_struct;

  // compute base_pose
  XYThetaPoint base_pose;
  base_pose.x = DISCXY2CONT(0, resolution_);
  base_pose.y = DISCXY2CONT(0, resolution_);
  base_pose.theta = DiscTheta2Cont(action->start_theta, num_of_angles_);

  XYThetaCell prev_interm_cell_3d;
  prev_interm_cell_3d.x = 0;
  prev_interm_cell_3d.y = 0;

  // Compute all the intersected cells for this action (intermptV and interm3DcellsV)
  for (int pind = 0; pind < mprim.interm_pts.size(); ++pind) {
    XYThetaPoint interm_point = mprim.interm_pts[pind];

    // also compute the intermediate discrete cells if not there already
    XYThetaPoint pose;
    pose.x = interm_point.x + base_pose.x;
    pose.y = interm_point.y + base_pose.y;
    pose.theta = interm_point.theta;

    XYThetaCell interm_cell_2d;
    interm_cell_2d.x = CONTXY2DISC(pose.x, resolution_);
    interm_cell_2d.y = CONTXY2DISC(pose.y, resolution_);

    // add unique cells to the list
    if (action->interm_cells_3d.size() == 0 || interm_cell_2d.x
        != prev_interm_cell_3d.x || interm_cell_2d.y != prev_interm_cell_3d.y) {
      action->interm_cells_3d.push_back(interm_cell_2d);
    }

    prev_interm_cell_3d = interm_cell_2d;
  }

  // compute linear and angular time
  double linear_distance = 0;
  for (unsigned int i = 1; i < action->interm_pts.size(); i++) {
    double x0 = action->interm_pts[i - 1].x;
    double y0 = action->interm_pts[i - 1].y;
    double x1 = action->interm_pts[i].x;
    double y1 = action->interm_pts[i].y;
    double dx = x1 - x0;
    double dy = y1 - y0;
    linear_distance += sqrt(dx * dx + dy * dy);
  }
  double linear_time = linear_distance / nominalvel_mpersec_;
  double angular_distance = fabs(MinUnsignedAngleDiff(DiscTheta2Cont(action->end_theta, num_of_angles_),
                                                      DiscTheta2Cont(action->start_theta, num_of_angles_)));
  double angular_time = angular_distance / ((PI_CONST / 4.0) / timetoturn45degsinplace_secs_);
  // make the cost the max of the two times
  action->cost = static_cast<int>(ceil(COSTMULT_MTOMM * std::max(linear_time, angular_time)));
  // use any additional cost multiplier
  action->cost *= mprim.cost_mult;

  // now compute the intersecting cells for this motion (including ignoring the source footprint)
  Get2DMotionCells(footprint_, mprim.interm_pts, &action->intersecting_cells, resolution_);
  Get2DMotionCellsCircleCenter(circle_center_, mprim.interm_pts, &action->circle_center_cells, resolution_);

  return action;
}

void MPrimitiveManager::ComputeReplanningDataForAction(Action* action) {
  unsigned int j;

  std::vector<XYThetaCell>* affected_pred_cells = &env_->affected_pred_cells_;

  // iterate over all the cells involved in the action
  XYThetaCell start_cell, end_cell;
  for (unsigned int i = 0; i < action->intersecting_cells.size(); i++) {
    // compute the translated affected search Pose - what state has an
    // outgoing action whose intersecting cell is at 0,0
    start_cell.theta = action->start_theta;
    start_cell.x = -action->intersecting_cells.at(i).x;
    start_cell.y = -action->intersecting_cells.at(i).y;

    for (j = 0; j < affected_pred_cells->size(); j++) {
      if (affected_pred_cells->at(j) == start_cell) break;
    }
    if (j == affected_pred_cells->size()) affected_pred_cells->push_back(start_cell);
  }  // over intersecting cells

  // add the centers since with h2d we are using these in cost computations
  // ---intersecting cell = origin
  // compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
  start_cell.theta = action->start_theta;
  start_cell.x = -0;
  start_cell.y = -0;

  for (j = 0; j < affected_pred_cells->size(); j++) {
    if (affected_pred_cells->at(j) == start_cell) break;
  }
  if (j == affected_pred_cells->size()) affected_pred_cells->push_back(start_cell);

  // ---intersecting cell = outcome state
  // compute the translated affected search Pose - what state has an outgoing action whose intersecting cell is at 0,0
  start_cell.theta = action->start_theta;
  start_cell.x = -action->dx;
  start_cell.y = -action->dy;

  for (j = 0; j < affected_pred_cells->size(); j++) {
    if (affected_pred_cells->at(j) == start_cell) break;
  }
  if (j == affected_pred_cells->size()) affected_pred_cells->push_back(start_cell);
}

};  // namespace search_based_global_planner
