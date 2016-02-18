cmake_minimum_required(VERSION 2.8.3)
project(autoscrubber)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    pluginlib
    nav_core
    tf
    #gperftools_21
    angles
    autoscrubber_services
    gslib
    fixpattern_path
    search_based_global_planner
    fixpattern_local_planner
    global_planner
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
    angles
    #gperftools_21
    autoscrubber_services
    gslib
    search_based_global_planner
    fixpattern_local_planner
    global_planner
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# autoscrubber
add_library(${PROJECT_NAME} STATIC
    src/autoscrubber.cc
    src/astar_controller.cc
    src/footprint_checker.cc
    src/bezier.cc
    src/bezier_planner.cc
)
target_link_libraries( ${PROJECT_NAME}
  fixpattern_trajectory_planner_ros
  global_planner
  search_based_global_planner
#  tcmalloc_minimal
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(autoscrubber_node src/autoscrubber_node.cc)
target_link_libraries(autoscrubber_node ${PROJECT_NAME}  ${catkin_LIBRARIES})
set_target_properties(autoscrubber_node PROPERTIES OUTPUT_NAME autoscrubber)
