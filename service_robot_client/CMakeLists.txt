cmake_minimum_required(VERSION 2.8.3)
project(service_robot_client)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    autoscrubber_services
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    autoscrubber_services
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# service_robot_client
add_executable(${PROJECT_NAME} src/service_robot_client.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
