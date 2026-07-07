cmake_minimum_required(VERSION 3.8)
project(tl_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_library(tl_hardware_interface SHARED
    src/tl_hardware_interface.cpp
)

ament_target_dependencies(tl_hardware_interface
    rclcpp
    rclcpp_lifecycle
    pluginlib
    # urdf
    hardware_interface
    # ros2_control
    # ros2_controllers
    # joint_state_broadcaster
    # joint_trajectory_controller
    # controller_manager
)

pluginlib_export_plugin_description_file(hardware_interface tl_hardware_interface.xml
)

install(TARGETS tl_hardware_interface
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/
    DESTINATION include
)
install(FILES tl_hardware_interface.xml
    DESTINATION share/${PROJECT_NAME}
)


# 确保导出相关内容
ament_export_include_directories(include)
ament_export_libraries(tl_hardware_interface)
# ament_export_dependencies(
#   rclcpp
#   pluginlib
#   moveit_core
#   Eigen3
# )

ament_package()
