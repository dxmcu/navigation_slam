# Bazel (http://bazel.io/) BUILD file for service_robot_navigation

COPTS = [
    "-std=c++11",
]
LINK_OPTS = []

# nav_core
cc_library(
    name = "nav_core",
    hdrs = glob([
        "nav_core/include/**/*.h",
    ]),
    includes = [
        "nav_core/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    linkstatic = True,
    deps = [
        "//external:geometry_msgs",
        "//external:tf",
        "//costmap_2d:costmap_2d",
    ],
    visibility = ["//visibility:public"],
)

# fixpattern_local_planner
cc_library(
    name = "fixpattern_local_planner",
    srcs = glob([
	"fixpattern_local_planner/src/goal_functions.cpp",
	"fixpattern_local_planner/src/odometry_helper_ros.cpp",
	"fixpattern_local_planner/src/obstacle_cost_function.cpp",
	"fixpattern_local_planner/src/oscillation_cost_function.cpp",
	"fixpattern_local_planner/src/prefer_forward_cost_function.cpp",
	"fixpattern_local_planner/src/costmap_model.cpp",
	"fixpattern_local_planner/src/simple_scored_sampling_planner.cpp",
	"fixpattern_local_planner/src/simple_trajectory_generator.cpp",
	"fixpattern_local_planner/src/trajectory.cpp",
    ]),
    hdrs = glob([
        "fixpattern_local_planner/include/**/*.h",
    ]),
    includes = [
        "fixpattern_local_planner/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    linkstatic = True,
    deps = [
        "//external:roscpp",
        "//external:tf",
        ":nav_core",
        "//costmap_2d:costmap_2d",
        "//external:angles",
        "//gslib:gslib",
        "//fixpattern_path:fixpattern_path",
        "//external:boost",
        "//external:eigen",
    ],
    visibility = ["//visibility:public"],
)

# fixpattern_local_planner_ros
cc_library(
    name = "fixpattern_local_planner_ros",
    srcs = glob([
	"fixpattern_local_planner/src/trajectory_planner.cpp",
    "fixpattern_local_planner/src/look_ahead_planner.cpp",
	"fixpattern_local_planner/src/trajectory_planner_ros.cpp",
    ]),
    hdrs = glob([
        "fixpattern_local_planner/include/**/*.h",
    ]),
    includes = [
        "fixpattern_local_planner/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    linkstatic = True,
    deps = [
        ":fixpattern_local_planner",
        "//gslib:gslib",
    ],
    visibility = ["//visibility:public"],
)

# test for fixpattern_local_planner
cc_test(
    name = "fixpattern_local_planner_utest",
    srcs = glob([
        "fixpattern_local_planner/test/gtest_main.cpp",
        # "fixpattern_local_planner/test/utest.cpp",
        "fixpattern_local_planner/test/velocity_iterator_test.cpp",
        # "fixpattern_local_planner/test/footprint_helper_test.cpp",
        "fixpattern_local_planner/test/trajectory_generator_test.cpp",
        # "fixpattern_local_planner/test/map_grid_test.cpp",
    ]),
    deps = [
        ":fixpattern_local_planner_ros",
    ],
)

cc_test(
    name = "fixpattern_line_iterator",
    srcs = glob([
        "fixpattern_local_planner/test/line_iterator_test.cpp",
    ]),
    deps = [
        ":fixpattern_local_planner_ros",
    ],
)

# global_planner
cc_library(
    name = "global_planner",
    srcs = glob([
        "global_planner/src/quadratic_calculator.cpp",
        "global_planner/src/dijkstra.cpp",
        "global_planner/src/astar.cpp",
        "global_planner/src/grid_path.cpp",
        "global_planner/src/gradient_path.cpp",
        "global_planner/src/orientation_filter.cpp",
        "global_planner/src/planner_core.cpp",
    ]),
    hdrs = glob([
        "global_planner/include/**/*.h",
    ]),
    includes = [
        "global_planner/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    linkstatic = True,
    deps = [
        "//costmap_2d:costmap_2d",
        "//gslib:gslib",
        "//external:geometry_msgs",
        ":nav_core",
        "//external:nav_msgs",
        "//external:roscpp",
        "//external:tf",
    ],
    visibility = ["//visibility:public"],
)

# search_based_global_planner
cc_library(
    name = "search_based_global_planner",
    srcs = glob([
        "search_based_global_planner/src/search_based_global_planner.cc",
        "search_based_global_planner/src/environment.cc",
        "search_based_global_planner/src/motion_primitive_manager.cc",
    ]),
    hdrs = glob([
        "search_based_global_planner/include/**/*.h",
    ]),
    includes = [
        "search_based_global_planner/include",
    ],
    copts = COPTS,
    linkopts = LINK_OPTS,
    linkstatic = True,
    deps = [
        "//external:roscpp",
        ":nav_core",
        "//external:tf",
        "//external:angles",
        "//gslib:gslib",
        "//fixpattern_path:fixpattern_path",
        # TODO(chenkan): link tcmalloc
        "//external:eigen",
    ],
    visibility = ["//visibility:public"],
)

# libservice_robot
cc_library(
    name = "libservice_robot",
    srcs = [
        "service_robot/src/service_robot.cc",
        "service_robot/src/astar_controller.cc",
        "service_robot/src/footprint_checker.cc",
        "service_robot/src/bezier.cc",
        "service_robot/src/bezier_planner.cc",
    ],
    hdrs = glob([
        "service_robot/include/**/*.h",
    ]),
    includes = ["service_robot/include"],
    deps = [
        "//external:roscpp",
        ":nav_core",
        "//external:tf",
        "//external:angles",
        "//external:move_base_msgs",
        "//autoscrubber_services:autoscrubber_services",
        "//gslib:gslib",
        "//fixpattern_path:fixpattern_path",
        ":search_based_global_planner",
        ":fixpattern_local_planner_ros",
        ":global_planner",
    ],
)

# service_robot
cc_binary(
    name = "service_robot",
    srcs = glob([
        "service_robot/src/service_robot_node.cc",
    ]),
    deps = [
        "libservice_robot",
    ],
)
