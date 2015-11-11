/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file navigation_mode_manager.h
 * @brief manager navigation mode - astar & fixpattern
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-08-21
 */

#ifndef AUTOSCRUBBER_MOVE_BASE_INCLUDE_MOVE_BASE_NAVIGATION_MODE_MANAGER_H_
#define AUTOSCRUBBER_MOVE_BASE_INCLUDE_MOVE_BASE_NAVIGATION_MODE_MANAGER_H_

namespace move_base {

typedef enum {
  ASTAR      = 0,
  FIXPATTERN = 1
} NavigationMode;

class NavigationModeManager {
 public:
  NavigationModeManager();
  ~NavigationModeManager();
 priavte:
};

};  // namespace move_base

#endif  // AUTOSCRUBBER_MOVE_BASE_INCLUDE_MOVE_BASE_NAVIGATION_MODE_MANAGER_H_
