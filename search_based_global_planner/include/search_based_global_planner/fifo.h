/* Copyright(C) Gaussian Automation. All rights reserved.
*/

/**
 * @file fifo.h
 * @brief first in first out
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-09-07
 */

#ifndef SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_FIFO_H_
#define SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_FIFO_H_

#include <cstdio>

template<class T>
class FIFO {
 public:
  explicit FIFO(unsigned int size) {
    q_ = new T[size];
    head_ = 0;
    tail_ = 0;
    size_ = size;
  }

  ~FIFO() {
    delete[] q_;
  }

  bool insert(T val) {
    int t_val = tail_;
    if (t_val == head_ + 1 || (t_val == 0 && head_ + 1 == size_)) {
      printf("ERROR: Trying to insert when FIFO is full!\n");
      return false;
    }
    q_[head_] = val;
    head_++;
    if (head_ == size_) head_ = 0;
    return true;
  }

  bool remove(T* val) {
    if (head_ == tail_) {
      printf("ERROR: Trying to remove when FIFO is empty!\n");
      return false;
    }
    *val = q_[tail_];
    tail_++;
    if (tail_ == size_) tail_ = 0;
    return true;
  }

  bool empty() {
    return head_ == tail_;
  }

  void clear() {
    head_ = 0;
    tail_ = 0;
  }

 private:
  int head_;
  int tail_;
  int size_;
  T* q_;
};

#endif  // SEARCH_BASED_GLOBAL_PLANNER_INCLUDE_SEARCH_BASED_GLOBAL_PLANNER_FIFO_H_
