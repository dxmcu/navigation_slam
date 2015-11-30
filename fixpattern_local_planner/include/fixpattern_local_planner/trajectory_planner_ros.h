/* Copyright(C) Gaussian Robot. All rights reserved.
*/

/**
 * @file trajectory_planner_ros.h
 * @brief fixpattern local planner, based on base_local_planner
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-29
 */

#ifndef FIXPATTERN_LOCAL_PLANNER_INCLUDE_FIXPATTERN_LOCAL_PLANNER_TRAJECTORY_PLANNER_ROS_H_
#define FIXPATTERN_LOCAL_PLANNER_INCLUDE_FIXPATTERN_LOCAL_PLANNER_TRAJECTORY_PLANNER_ROS_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <fixpattern_local_planner/world_model.h>
#include <fixpattern_local_planner/point_grid.h>
#include <fixpattern_local_planner/costmap_model.h>
#include <fixpattern_local_planner/voxel_grid_model.h>
#include <fixpattern_local_planner/trajectory_planner.h>
#include <fixpattern_local_planner/look_ahead_planner.h>
#include <fixpattern_local_planner/map_grid_visualizer.h>
#include <fixpattern_local_planner/planar_laser_scan.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>
#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <dynamic_reconfigure/server.h>
#include <fixpattern_local_planner/BaseLocalPlannerConfig.h>
#include <fixpattern_local_planner/odometry_helper_ros.h>
#include <pcl_ros/publisher.h>
#include <fixpattern_path/path.h>

#include <vector>
#include <string>

namespace fixpattern_local_planner {

typedef enum {
  TRAJECTORY_PLANNER = 0,
  LOOKAHEAD_PLANNER  = 1
} PlannerType;

/**
 * @class TrajectoryPlannerROS
 * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
 */
class FixPatternTrajectoryPlannerROS {
 public:
  /**
   * @brief  Default constructor for the ros wrapper
   */
  FixPatternTrajectoryPlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  FixPatternTrajectoryPlannerROS(std::string name,
                                 tf::TransformListener* tf,
                                 costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf::TransformListener* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Destructor for the wrapper
   */
  ~FixPatternTrajectoryPlannerROS();

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param planner_type Which planner to use
   *        0 for trajecotry rollout, 1 for look-ahead controller
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(PlannerType planner_type, geometry_msgs::Twist* cmd_vel);

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @param orig_frame_id The frame id of origin global plan
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<fixpattern_path::PathPoint>& orig_global_plan, const std::string& orig_frame_id);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief  Check if rotate to path has been done 
   * @return rotating_to_path_done_ : True if achieved, false otherwise
   */
  bool isPathRotateDone();

  /**
   * @brief reset reached_goal_, xy_tolerance_latch_, last_rotate_to_goal_dir_ and try_rotate_ etc.
   */
  void reset_planner();

  bool isInitialized() {
    return initialized_;
  }

  /** @brief Return the inner TrajectoryPlanner object.  Only valid after initialize(). */
  TrajectoryPlanner* getPlanner() const { return tc_; }

 private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level);  // NOLINT

  /**
   * @brief Once a goal position is reached... rotate to the goal orientation
   * @param planner_type Which planner to use
   *        0 for trajecotry rollout, 1 for look-ahead controller
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  goal_th The desired th value for the goal
   * @param  cmd_vel The velocity commands to be filled
   * @param  rotate_direction The direction of rotating, if = 0, no use
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool rotateToGoal(PlannerType planner_type, const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist* cmd_vel, int rotate_direction = 0);

  /**
   * @brief Check if robot needs to backup when in corner
   * @param planner_type Which planner to use
   *        0 for trajecotry rollout, 1 for look-ahead controller
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if needs to backup, false otherwise
   */
  bool needBackward(PlannerType planner_type, const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist* cmd_vel);

  /**
   * @brief Stop the robot taking into account acceleration limits
   * @param planner_type Which planner to use
   *        0 for trajecotry rollout, 1 for look-ahead controller
   * @param  global_pose The pose of the robot in the global frame
   * @param  robot_vel The velocity of the robot
   * @param  cmd_vel The velocity commands to be filled
   * @return  True if a valid trajectory was found, false otherwise
   */
  bool stopWithAccLimits(PlannerType planner_type, const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist* cmd_vel);

  std::vector<double> loadYVels(ros::NodeHandle node);

  double sign(double x) {
    return x < 0.0 ? -1.0 : 1.0;
  }

  WorldModel* world_model_;  ///< @brief The world model that the controller will use
  TrajectoryPlanner* tc_;    ///< @brief The trajectory controller
  LookAheadPlanner* la_;     ///< @brief The look-ahead controller

  costmap_2d::Costmap2DROS* costmap_ros_;  ///< @brief The ROS wrapper for the costmap the controller will use
  costmap_2d::Costmap2D* costmap_;         ///< @brief The costmap the controller will use
  tf::TransformListener* tf_;              ///< @brief Used for transforming point clouds
  std::string global_frame_;               ///< @brief The frame in which the controller will run
  double max_sensor_range_;                ///< @brief Keep track of the effective maximum range of our sensors
  nav_msgs::Odometry base_odom_;           ///< @brief Used to get the velocity of the robot
  std::string robot_base_frame_;           ///< @brief Used as the base frame id of the robot
  double rot_stopped_velocity_, trans_stopped_velocity_;
  double xy_goal_tolerance_, yaw_goal_tolerance_, min_in_place_vel_th_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<fixpattern_path::PathPoint> fixpattern_path_;
  bool prune_plan_;
  bool rotating_to_route_direction_;
  boost::recursive_mutex odom_lock_;

  double max_vel_th_, min_vel_th_;
  double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
  double sim_period_;
  bool rotating_to_goal_;
  bool reached_goal_;
	bool rotating_to_path_done_;
  bool latch_xy_goal_tolerance_, xy_tolerance_latch_;

  ros::Publisher g_plan_pub_, l_plan_pub_;
  pcl_ros::Publisher<MapGridCostPoint> traj_cloud_pub_;

  dynamic_reconfigure::Server<BaseLocalPlannerConfig> *dsrv_;
  fixpattern_local_planner::BaseLocalPlannerConfig default_config_;
  bool setup_;


  bool initialized_;
  fixpattern_local_planner::OdometryHelperRos odom_helper_;

  std::vector<geometry_msgs::Point> footprint_spec_;

  double rotate_to_goal_k_;
  double last_target_yaw_;
  int last_rotate_to_goal_dir_;
  int max_rotate_try_times_;
  int try_rotate_;
};

};  // namespace fixpattern_local_planner

#endif  // FIXPATTERN_LOCAL_PLANNER_INCLUDE_FIXPATTERN_LOCAL_PLANNER_TRAJECTORY_PLANNER_ROS_H_
