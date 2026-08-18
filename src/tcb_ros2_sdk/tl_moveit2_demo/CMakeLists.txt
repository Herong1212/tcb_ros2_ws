cmake_minimum_required(VERSION 3.8)
project(tl_moveit2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖查找 ──────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── 头文件目录 ────────────────────────────────────────────────────────────────
include_directories(include)

# ── 可执行目标 ────────────────────────────────────────────────────────────────
add_executable(moveit2_demo_node
    src/moveit2_demo.cpp
)

ament_target_dependencies(moveit2_demo_node
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    tf2_ros
    tf2_geometry_msgs
)

# ── 安装规则 ──────────────────────────────────────────────────────────────────
install(TARGETS moveit2_demo_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
