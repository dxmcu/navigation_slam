cmake_minimum_required(VERSION 2.8.3)
project(fixpattern_local_planner)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            message_generation
            nav_core
            rostest
            costmap_2d
            angles
            gslib
            fixpattern_path
        )

find_package(Boost REQUIRED
    COMPONENTS
        thread
        )

find_package(Eigen REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        ${PROJECT_NAME}
        fixpattern_trajectory_planner_ros
    CATKIN_DEPENDS
        roscpp
        message_generation
        tf
        costmap_2d
        nav_core
        angles
)

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

add_library(${PROJECT_NAME} STATIC
	src/goal_functions.cpp
	src/odometry_helper_ros.cpp
	src/obstacle_cost_function.cpp
	src/oscillation_cost_function.cpp
	src/prefer_forward_cost_function.cpp
	src/costmap_model.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} nav_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Eigen_LIBRARIES}
    )

add_library(fixpattern_trajectory_planner_ros STATIC
	src/trajectory_planner.cpp
    src/look_ahead_planner.cpp
	src/trajectory_planner_ros.cpp)
add_dependencies(fixpattern_trajectory_planner_ros nav_msgs_gencpp)
target_link_libraries(fixpattern_trajectory_planner_ros
     ${PROJECT_NAME})

install(TARGETS
            ${PROJECT_NAME}
            fixpattern_trajectory_planner_ros
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
