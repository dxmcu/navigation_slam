# service_robot move_base
forked from ros-planning/navigation.git -> indigo-devel

---

### service_robot_control
This is a new node, we use this to feed params to move_base when it dies

### fixpattern_path
lib for fixpattern path management, we use it to calculate attributes of PathPoint:
- radius: according to previous two points and current point, max_radius: 5.0
- highlight:
   1. set max and min highlight distance
   2. check radius change: max_radius_change: 0.5
   3. check radian change
   4. filter with ```Y[n] = (1 - k) * Y[n - 1] + k * X[n]```

and supply some interface:
- ```Instance()```: singleton
- ```SyncFromFile(const char* file_name)```: read path from file
- ```SyncToFile(const char* file_name)```: write path to file
- ```Prune(const PathPoint& p, double max_offroad_dis_)```: prune path to some point
   1. find estimated current_point
   2. determin current_point search range
   3. current_point is the point that has the minimum distance to robot_pose
- ```GeometryPath()```: get path in geometry_msgs::PoseStamped

### fixpattern_local_planner
1. remove in-place rotation
2. check front safe here, if not safe, return failed
3. ```max_vel = 0.75 * radius * fabs(min_vel_theta);```
4. ```vx_samp = highlight * k_vel_x;```
5. ```if (current_point_dis > 0.25) sim_time *= distance_to_goal / highlight_length;```
6. if some cost of trajectory are all best, average their vtheta
7. fixed num_steps
8. only check ```num_calc_footprint_cost``` points' footprint cost
9. only use path_cost, occ_cost and goal_cost are not used
10. ```path_dist += point_cost;```
11. use Euclid distance to calculate path_dist

### fixpattern_global_planner
main work is done in fixpattern_path
