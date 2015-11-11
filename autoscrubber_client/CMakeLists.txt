cmake_minimum_required(VERSION 2.8.3)
project(autoscrubber_client)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    autoscrubber_services
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    autoscrubber_services
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# autoscrubber_client
add_executable(${PROJECT_NAME} src/autoscrubber_client.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
