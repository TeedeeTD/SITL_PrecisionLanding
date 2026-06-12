cmake_minimum_required(VERSION 3.5)
project(aruco_fractal_tracker)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

find_library(ARUCO_LIBRARY
  NAMES aruco
  HINTS "$ENV{HOME}/.local/lib" "/usr/local/lib"
  REQUIRED
)
get_filename_component(ARUCO_LIBRARY_DIR "${ARUCO_LIBRARY}" DIRECTORY)

add_library(aruco_fractal_tracker_node SHARED src/aruco_fractal_tracker_node.cpp)
target_link_libraries(aruco_fractal_tracker_node "${ARUCO_LIBRARY}")
set_target_properties(aruco_fractal_tracker_node PROPERTIES
  BUILD_RPATH "${ARUCO_LIBRARY_DIR}"
  INSTALL_RPATH "${ARUCO_LIBRARY_DIR}"
)
ament_target_dependencies(aruco_fractal_tracker_node 
  rclcpp 
  cv_bridge 
  geometry_msgs 
  sensor_msgs 
  rclcpp_components 
  aruco 
  OpenCV
  tf2
  tf2_geometry_msgs
)
rclcpp_components_register_nodes(aruco_fractal_tracker_node "fractal_tracker::ArucoFractalTracker")

add_executable(aruco_fractal_tracker src/aruco_fractal_tracker_node_main.cpp)
target_link_libraries(aruco_fractal_tracker aruco_fractal_tracker_node)
set_target_properties(aruco_fractal_tracker PROPERTIES
  BUILD_RPATH "${ARUCO_LIBRARY_DIR}"
  INSTALL_RPATH "${ARUCO_LIBRARY_DIR}"
)

install(
  DIRECTORY include/aruco_fractal_tracker
  DESTINATION include
)

install(
  TARGETS aruco_fractal_tracker_node
  EXPORT export_aruco_fractal_tracker_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS aruco_fractal_tracker
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(aruco_fractal_tracker_node)

ament_export_dependencies(
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  rclcpp_components 
  cv_bridge 
  aruco
  tf2
  tf2_geometry_msgs
)

ament_package()
