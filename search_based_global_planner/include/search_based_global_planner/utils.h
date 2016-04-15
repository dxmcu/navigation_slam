/* Copyright(C) Gaussian Automation. All rights reserved.
 */

/**
 * @file utils.h
 * @brief util functions
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-07
 */

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_UTILS_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_UTILS_H_

#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <utility>
#include <set>
#include "search_based_global_planner/bfs_2d.h"

namespace search_based_global_planner {

#define NORMALIZEDISCTHETA(THETA, THETADIRS) (((THETA >= 0) ?\
            ((THETA) % (THETADIRS)) :\
            (((THETA) % (THETADIRS) + THETADIRS) % THETADIRS)))
#define CONTXY2DISC(X, RESOLUTION) (((X) >= 0) ? (static_cast<int>((X) / (RESOLUTION))) : (static_cast<int>((X) / (RESOLUTION)) - 1))
#define DISCXY2CONT(X, RESOLUTION) ((X) * (RESOLUTION) + (RESOLUTION) / 2.0)

#define INFINITECOST 1000000000
#define PI_CONST 3.141592653589793238462643383279502884
#define COSTMULT_MTOMM 1000

typedef enum {
  SHORT_FORWARD = 0,
  NORMAL_FORWARD, 
	LONG_FORWARD,
	//FORWARD_TURN_LEFT,
	//FORWARD_TURN_RIGHT,
	IN_PLACE_ROTATE_LEFT,
	IN_PLACE_ROTATE_RIGHT,
	MAX_MPRIM_INDEX
} MprimIndex;

typedef struct _XYPoint {
  double x;
  double y;

  _XYPoint() { }
  _XYPoint(double x, double y) : x(x), y(y) { }
} XYPoint;

typedef struct _XYThetaPoint {
  double x;
  double y;
  double theta;
  _XYThetaPoint() { }
  _XYThetaPoint(double x, double y, double theta) : x(x), y(y), theta(theta) { }
} XYThetaPoint;

typedef struct _XYCell {
  int x;
  int y;

  bool operator==(const _XYCell cell) const {
    return x == cell.x && y == cell.y;
  }
  bool operator<(const _XYCell cell) const {
    return x < cell.x || (x == cell.x && y < cell.y);
  }
  _XYCell() { }
  _XYCell(int x, int y) : x(x), y(y) { }
} XYCell;

typedef struct _XYThetaCell {
  int x;
  int y;
  int theta;

  bool operator==(const _XYThetaCell& cell) const {
    return x == cell.x && y == cell.y && theta == cell.theta;
  }
  _XYThetaCell() { }
  _XYThetaCell(int x, int y, int theta) : x(x), y(y), theta(theta) { }
} XYThetaCell;

typedef struct _IntermPointStruct {
  double radius;     // radius of this point, we know it when generate mprim
  bool is_corner;    // if this point is in-place-rotation point
  double theta_out;  // only useful if is_corner is true, represent out theta of corner
  int rotate_direction;  // only useful if is_corner is true, represent which direction should turn to
                         // 1 represents turning left, -1 represents turnint right, 0 represents no turn
  double highlight;      // calculate highlight and max_vel based on combination of mprims sequence
  double max_vel;
  double distance;  // distance bettween this point and next
} IntermPointStruct;

typedef struct _MPrimitive {
  int mprim_id;
  XYThetaCell start_cell;
  XYThetaCell end_cell;
  int cost_mult;
  std::vector<XYThetaPoint> interm_pts;  // intermedia points
  std::vector<IntermPointStruct> interm_struct;

  _MPrimitive() { }
  _MPrimitive(int mprim_id, XYThetaCell start_cell, XYThetaCell end_cell,
              int cost_mult, std::vector<XYThetaPoint>& interm_pts,
              std::vector<IntermPointStruct>& interm_struct)
    : mprim_id(mprim_id), start_cell(start_cell), end_cell(end_cell),
      cost_mult(cost_mult), interm_pts(interm_pts), interm_struct(interm_struct) { }
} MotionPrimitive;


typedef struct _Action {
  unsigned char action_index;  // index of the action (unique for given starttheta)
  int8_t start_theta;
  int8_t dx;
  int8_t dy;
  int8_t end_theta;
  unsigned int cost;
  double source_x;
  double source_y;
  double distance;
  double highlight;
  double max_vel;
  std::vector<XYCell> intersecting_cells;
  std::vector<XYCell> circle_center_cells;
  // start at 0,0,starttheta and end at endcell in continuous domain with half-bin less to account for 0,0 start
  std::vector<XYThetaPoint> interm_pts;
  // start at 0,0,starttheta and end at endcell in discrete domain
  std::vector<XYThetaCell> interm_cells_3d;
  // record some useful info of intermedia points
  std::vector<IntermPointStruct> interm_struct;
} Action;

typedef struct _Actions_Path {
  Action action;
  double source_x;
  double source_y;
} Actions_Path;

// input angle should be in radians
// counterclockwise is positive
// output is an angle in the range of from 0 to 2*PI
inline double NormalizeAngle(double angle) {
  double retangle = angle;

  // get to the range from -2PI, 2PI
  if (fabs(retangle) > 2 * PI_CONST) retangle = retangle - (static_cast<int>(retangle / (2 * PI_CONST))) * 2 * PI_CONST;

  // get to the range 0, 2PI
  if (retangle < 0) retangle += 2 * PI_CONST;

  if (retangle < 0 || retangle > 2 * PI_CONST) {
    ROS_ERROR("[SEARCH BASED GLOBAL PLANNER] after normalization of angle=%f we get angle=%f\n", angle, retangle);
  }

  return retangle;
}

inline double MinUnsignedAngleDiff(double angle1, double angle2) {
  // get the angles into 0-2*PI range
  angle1 = NormalizeAngle(angle1);
  angle2 = NormalizeAngle(angle2);

  double anglediff = fabs(angle1 - angle2);

  // see if we can take a shorter route
  if (anglediff > PI_CONST) {
    anglediff = fabs(anglediff - 2 * PI_CONST);
  }

  return anglediff;
}

// converts discretized version of angle into continuous (radians)
// maps 0->0, 1->delta, 2->2*delta, ...
inline double DiscTheta2Cont(int nTheta, int NUMOFANGLEVALS) {
  double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
  return nTheta * thetaBinSize;
}

// converts continuous (radians) version of angle into discrete
// maps 0->0, [delta/2, 3/2*delta)->1, [3/2*delta, 5/2*delta)->2,...
inline int ContTheta2Disc(double fTheta, int NUMOFANGLEVALS) {
  double thetaBinSize = 2.0 * PI_CONST / NUMOFANGLEVALS;
  return static_cast<int>(NormalizeAngle(fTheta + thetaBinSize / 2.0) / (2.0 * PI_CONST) * (NUMOFANGLEVALS));
}

inline void Get2DFootprintCells(std::vector<XYPoint> polygon, std::set<XYCell>* cells, XYThetaPoint pose, double res) {
  // special case for point robot
  if (polygon.size() <= 1) {
    XYCell cell;
    cell.x = CONTXY2DISC(pose.x, res);
    cell.y = CONTXY2DISC(pose.y, res);

    cells->insert(cell);
    return;
  }

  // run bressenham line algorithm around the polygon (add them to the cells set)
  // while doing that find the min and max (x,y) and the average x and y
  double cth = cos(pose.theta);
  double sth = sin(pose.theta);

  std::vector<std::pair<int, int> > disc_polygon;
  disc_polygon.reserve(polygon.size() + 1);
  int minx = INFINITECOST;
  int maxx = -INFINITECOST;
  int miny = INFINITECOST;
  int maxy = -INFINITECOST;

  // find the bounding box of the polygon
  for (unsigned int i = 0; i < polygon.size(); i++) {
    std::pair<int, int> p;
    // p.first = CONTXY2DISC(cth*polygon[i].x - sth*polygon[i].y + pose.x, res);
    double cx = (cth * polygon[i].x - sth * polygon[i].y + pose.x);
    double cy = (sth * polygon[i].x + cth * polygon[i].y + pose.y);
    p.first = static_cast<int>(cx > 0 ? cx / res + 0.5 : cx / res - 0.5);  // (int)(cx / res + 0.5 * sign(c);
    // p.second = CONTXY2DISC(sth*polygon[i].x + cth*polygon[i].y + pose.y, res);
    p.second = static_cast<int>(cy > 0 ? cy / res + 0.5 : cy / res - 0.5);  // (int)(cy / res + 0.5);
    disc_polygon.push_back(p);
    if (p.first < minx) minx = p.first;
    if (p.first > maxx) maxx = p.first;
    if (p.second < miny) miny = p.second;
    if (p.second > maxy) maxy = p.second;
  }
  disc_polygon.push_back(disc_polygon.front());

  // make a grid big enough for the footprint
  int sizex = (maxx - minx + 1) + 2;
  int sizey = (maxy - miny + 1) + 2;
  int** grid = new int*[sizex];
  for (int i = 0; i < sizex; i++) {
    grid[i] = new int[sizey];
    for (int j = 0; j < sizey; j++)
      grid[i][j] = 0;
  }

  // plot line points on the grid
  for (unsigned int i = 1; i < disc_polygon.size(); i++) {
    int x0 = disc_polygon[i - 1].first - minx + 1;
    int y0 = disc_polygon[i - 1].second - miny + 1;
    int x1 = disc_polygon[i].first - minx + 1;
    int y1 = disc_polygon[i].second - miny + 1;

    // bressenham (add the line cells to the set and to a vector)
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
      int temp = x0;
      x0 = y0;
      y0 = temp;
      temp = x1;
      x1 = y1;
      y1 = temp;
    }
    if (x0 > x1) {
      int temp = x0;
      x0 = x1;
      x1 = temp;
      temp = y0;
      y0 = y1;
      y1 = temp;
    }
    int deltax = x1 - x0;
    int deltay = abs(y1 - y0);
    int error = deltax / 2;
    int ystep = (y0 < y1 ? 1 : -1);
    int y = y0;
    for (int x = x0; x <= x1; x++) {
      if (steep) {
        grid[y][x] = 1;
        cells->insert(XYCell(y - 1 + minx, x - 1 + miny));
      } else {
        grid[x][y] = 1;
        cells->insert(XYCell(x - 1 + minx, y - 1 + miny));
      }
      int last_error = error;
      error -= deltay;
      if (error < 0 && x != x1) {
        // make sure we can't have a diagonal line (the 8-connected bfs will leak through)

        int tempy = y;
        int tempx = x;
        if (last_error < -error)
          tempy += ystep;
        else
          tempx += 1;
        if (steep) {
          grid[tempy][tempx] = 1;
          cells->insert(XYCell(tempy - 1 + minx, tempx - 1 + miny));
        } else {
          grid[tempx][tempy] = 1;
          cells->insert(XYCell(tempx - 1 + minx, tempy - 1 + miny));
        }

        y += ystep;
        error += deltax;
      }
    }
  }

  // TODO(chenkan): Why do we need a bfs here
  // run a 2d bfs from the average (x,y)
  BFS2D bfs(sizex, sizey, 1);
  bfs.compute_distance_from_point(grid, 0, 0);

  for (int i = 0; i < sizex; i++)
    delete[] grid[i];
  delete[] grid;

  // add all cells expanded to the cells set
  for (int i = 1; i < sizex - 1; i++) {
    for (int j = 1; j < sizey - 1; j++) {
      if (bfs.get_distance(i, j) < 0)
        cells->insert(XYCell(i - 1 + minx, j - 1 + miny));
    }
  }
}

inline void Get2DMotionCells(std::vector<XYPoint> polygon, std::vector<XYThetaPoint> poses,
                             std::vector<XYCell>* cells, double res) {
  // can't find any motion cells if there are no poses
  if (poses.empty()) {
    return;
  }

  // get first footprint set
  std::set<XYCell> first_cell_set;
  Get2DFootprintCells(polygon, &first_cell_set, poses[0], res);

  // duplicate first footprint set into motion set
  std::set<XYCell> cell_set = first_cell_set;

  // call get footprint on the rest of the points
  for (unsigned int i = 1; i < poses.size(); i++) {
    Get2DFootprintCells(polygon, &cell_set, poses[i], res);
  }

  // convert the motion set to a vector but don't include the cells in the first footprint set
  cells->reserve(cell_set.size() - first_cell_set.size());
  for (std::set<XYCell>::iterator it = cell_set.begin(); it != cell_set.end(); it++) {
    if (first_cell_set.find(*it) == first_cell_set.end()) {
      cells->push_back(*it);
    }
  }
}

inline void Get2DCircleCenterCells(std::vector<XYPoint> circle_center, std::set<XYCell>* cells, XYThetaPoint pose, double res) {
  // origin from Get2DFootprintCells()
  double cth = cos(pose.theta);
  double sth = sin(pose.theta);

  // find the bounding box of the polygon
  for (unsigned int i = 0; i < circle_center.size(); i++) {
    XYCell p;
    // p.first = CONTXY2DISC(cth*polygon[i].x - sth*polygon[i].y + pose.x, res);
    double cx = (cth * circle_center[i].x - sth * circle_center[i].y + pose.x);
    double cy = (sth * circle_center[i].x + cth * circle_center[i].y + pose.y);
    p.x = static_cast<int>(cx > 0 ? cx / res + 0.5 : cx / res - 0.5);  // (int)(cx / res + 0.5 * sign(c);
    // p.second = CONTXY2DISC(sth*polygon[i].x + cth*polygon[i].y + pose.y, res);
    p.y = static_cast<int>(cy > 0 ? cy / res + 0.5 : cy / res - 0.5);  // (int)(cy / res + 0.5);
    cells->insert(p);
  }
}

inline void Get2DMotionCellsCircleCenter(std::vector<XYPoint> circle_center, std::vector<XYThetaPoint> poses,
                                         std::vector<XYCell>* cells, double res) {
  std::set<XYCell> cell_set;
  for (int i = 0; i < poses.size(); ++i) {
    Get2DCircleCenterCells(circle_center, &cell_set, poses[i], res);
  }

  // push to cells
  for (auto& cell : cell_set) {
    cells->push_back(cell);
  }
}

inline double GetTimeInSeconds() {
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec + 0.000001 * t.tv_usec;
}

};  // namespace search_based_global_planner

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_UTILS_H_
