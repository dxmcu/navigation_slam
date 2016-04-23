cmake_minimum_required(VERSION 2.8.3)
project(search_based_global_planner)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    roscpp
    nav_core
    tf
    #gperftools_21
    angles
    gslib
    fixpattern_path
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    #gperftools_21
    angles
    gslib
    fixpattern_path
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

# service_robot 
add_library(${PROJECT_NAME} STATIC
    src/search_based_global_planner.cc
    src/environment.cc
    src/motion_primitive_manager.cc
)
target_link_libraries( ${PROJECT_NAME}
#  tcmalloc_minimal
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
