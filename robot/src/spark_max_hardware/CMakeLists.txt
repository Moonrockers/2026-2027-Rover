cmake_minimum_required(VERSION 3.16)
project(spark_max_hardware LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(spark_max_hardware SHARED
    src/spark_max_system.cpp
    src/can_socket.cpp
    src/protocol.cpp
    src/spark_max_can_protocol.cpp)
target_include_directories(spark_max_hardware PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/spark_max_hardware>)
ament_target_dependencies(spark_max_hardware PUBLIC
    hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface spark_max_hardware.xml)

install(DIRECTORY include/ DESTINATION include/spark_max_hardware)
install(TARGETS spark_max_hardware EXPORT export_spark_max_hardware
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_spark_max_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
