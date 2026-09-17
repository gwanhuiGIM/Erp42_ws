cmake_minimum_required(VERSION 3.5)
project(erp_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(erp_interfaces REQUIRED)

# Include directories
include_directories(include)

# Install Python scripts
install(PROGRAMS
  scripts/ByteHandler.py
  scripts/ErpSerialHandler.py
  scripts/erp42_serial.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_dependencies(erp_interfaces)
ament_package()

