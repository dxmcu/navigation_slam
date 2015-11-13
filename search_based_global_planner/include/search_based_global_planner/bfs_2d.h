/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file bfs_2d.h
 * @brief helper class of bfs
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-07
 */

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_BFS_2D_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_BFS_2D_H_

#include <vector>
#include "search_based_global_planner/fifo.h"

class BFS2D {
 private:
  class BFSCell2D {
   public:
    BFSCell2D() {
      x = 0;
      y = 0;
    }

    BFSCell2D(int tempX, int tempY) {
      x = tempX;
      y = tempY;
    }

    int x;
    int y;
  };

 public:
  BFS2D(unsigned int sx, unsigned int sy, int obs_thresh, int fifo_size = -1) : NUM_ACTIONS(8) {
    size_x_ = sx;
    size_y_ = sy;
    thresh_ = obs_thresh;

    // initialize the fifo
    if (fifo_size < 0) fifo_size = 2 * sx + 2 * sy;
    q_ = new FIFO<BFSCell2D> (fifo_size);

    // initialize the distance grid
    dist_ = new int*[size_x_];
    for (int x = 0; x < size_x_; x++)
      dist_[x] = new int[size_y_];

    // initialize the actions
    dx = new int[NUM_ACTIONS];
    dy = new int[NUM_ACTIONS];
    int idx = 0;
    for (int x = -1; x <= 1; x++) {
      for (int y = -1; y <= 1; y++) {
        if (!x && !y) continue;
        dx[idx] = x;
        dy[idx] = y;
        idx++;
      }
    }
  }

  ~BFS2D() {
    // delete the fifo
    delete q_;

    // delete the distance grid
    for (int x = 0; x < size_x_; x++)
      delete[] dist_[x];
    delete[] dist_;

    // delete the actions
    delete[] dx;
    delete[] dy;
  }

  bool compute_distance_from_point(int** grid, int x, int y) {
    if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) {
      printf("ERROR[compute_distance_from_point]: point is out of bounds!\n");
      return false;
    }
    q_->clear();
    clear_distances();

    BFSCell2D c(x, y);
    q_->insert(c);
    dist_[x][y] = 0;

    compute_distances(grid);
    return true;
  }

  bool compute_distance_from_points(int** grid, std::vector<int> x, std::vector<int> y) {
    if (x.size() != y.size()) {
      printf("ERROR[compute_distance_from_points]: size of x and y coordinates must agree!\n");
      return false;
    }
    q_->clear();
    clear_distances();

    for (unsigned int i = 0; i < x.size(); i++) {
      if (x[i] < 0 || x[i] >= size_x_ || y[i] < 0 || y[i] >= size_y_) {
        printf("ERROR[compute_distance_from_points]: point is out of bounds!\n");
        return false;
      }
      BFSCell2D c(x[i], y[i]);
      q_->insert(c);
      dist_[x[i]][y[i]] = 0;
    }

    compute_distances(grid);
    return true;
  }

  void compute_distance_from_obs(int** grid) {
    q_->clear();
    clear_distances();

    for (int x = 0; x < size_x_; x++) {
      for (int y = 0; y < size_y_; y++) {
        if (grid[x][y] >= thresh_) {
          BFSCell2D c(x, y);
          q_->insert(c);
          dist_[x][y] = 0;
        }
      }
    }

    compute_distances(grid);
  }

  void clear_distances() {
    for (int x = 0; x < size_x_; x++)
      for (int y = 0; y < size_y_; y++)
        dist_[x][y] = -1;
  }

  void compute_distances(int** grid) {
    BFSCell2D c;
    while (!q_->empty()) {
      q_->remove(&c);
      int cost = dist_[c.x][c.y] + 1;
      if (c.x == 0 || c.x == size_x_ - 1 || c.y == 0 || c.y == size_y_ - 1) {
        // we are on a boundary so we have to bounds check each successor
        for (int i = 0; i < NUM_ACTIONS; i++) {
          int x = c.x + dx[i];
          int y = c.y + dy[i];
          if (x < 0 || x >= size_x_ || y < 0 || y >= size_y_) continue;
          if (dist_[x][y] < 0 && grid[x][y] < thresh_) {
            dist_[x][y] = cost;
            BFSCell2D temp(x, y);
            q_->insert(temp);
          }
        }
      } else {
        // we are not near a boundary so no bounds check is required
        for (int i = 0; i < NUM_ACTIONS; i++) {
          int x = c.x + dx[i];
          int y = c.y + dy[i];
          if (dist_[x][y] < 0 && grid[x][y] < thresh_) {
            dist_[x][y] = cost;
            BFSCell2D temp(x, y);
            q_->insert(temp);
          }
        }
      }
    }
  }

  int get_distance(unsigned int x, unsigned int y) {
    return dist_[x][y];
  }

 private:
  int** dist_;
  FIFO<BFSCell2D>* q_;
  int size_x_;
  int size_y_;
  int thresh_;

  const int NUM_ACTIONS;
  int* dx;
  int* dy;
};

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_BFS_2D_H_
