cmake_minimum_required(VERSION 3.8)
project(onboard_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED)

# service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDynamicObstacles.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  vision_msgs
  message_filters
  cv_bridge
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

# NOTE: fakeDetector (Gazebo simulation helper) is not ported to ROS2 yet and is
# excluded from the build.
add_library(${PROJECT_NAME}_lib
  include/${PROJECT_NAME}/dynamicDetector.cpp
  include/${PROJECT_NAME}/uvDetector.cpp
  include/${PROJECT_NAME}/dbscan.cpp
  include/${PROJECT_NAME}/kalmanFilter.cpp
  include/${PROJECT_NAME}/lidarDetector.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES} ${OpenCV_LIBS})

# link generated service typesupport into the library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

add_executable(detector_node src/detector_node.cpp)
target_link_libraries(detector_node ${PROJECT_NAME}_lib)
ament_target_dependencies(detector_node rclcpp)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS detector_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/dynamic_cloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch cfg rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
