cmake_minimum_required(VERSION 3.5)
project(precision_landing)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dib_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

find_library(ARUCO_LIBRARY
  NAMES aruco
  HINTS "$ENV{HOME}/.local/lib" "/usr/local/lib"
  REQUIRED
)
get_filename_component(ARUCO_LIBRARY_DIR "${ARUCO_LIBRARY}" DIRECTORY)

# ── 1. Fractal Tracker Component & Executable ────────────────────────
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
  std_msgs
  dib_msgs
  rclcpp_components 
  aruco 
  OpenCV
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(aruco_fractal_tracker_node "fractal_tracker::ArucoFractalTracker")

add_executable(aruco_fractal_tracker src/aruco_fractal_tracker_node_main.cpp)
target_link_libraries(aruco_fractal_tracker aruco_fractal_tracker_node)
set_target_properties(aruco_fractal_tracker PROPERTIES
  BUILD_RPATH "${ARUCO_LIBRARY_DIR}"
  INSTALL_RPATH "${ARUCO_LIBRARY_DIR}"
)

# ── 2. RTSP Camera Bridge Component & Executable ───────────────────────
add_library(rtsp_publisher_node SHARED src/rtsp_publisher.cpp)
ament_target_dependencies(rtsp_publisher_node
  rclcpp
  sensor_msgs
  rclcpp_components
  cv_bridge
  OpenCV
)
rclcpp_components_register_nodes(rtsp_publisher_node "precision_landing::RtspPublisher")

add_executable(rtsp_publisher src/rtsp_publisher_main.cpp)
target_link_libraries(rtsp_publisher rtsp_publisher_node)

# ── 3. Offboard Precland Controller Component & Executable ─────────────
add_library(offboard_precland_controller_node SHARED src/offboard_precland_controller.cpp)
ament_target_dependencies(offboard_precland_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  dib_msgs
  mavros_msgs
  rclcpp_components
  tf2
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(offboard_precland_controller_node "precision_landing::OffboardPreclandController")

add_executable(offboard_precland_controller src/offboard_precland_controller_main.cpp)
target_link_libraries(offboard_precland_controller offboard_precland_controller_node)

# ── 4. Install Targets ────────────────────────────────────────────────
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS 
    aruco_fractal_tracker_node
    rtsp_publisher_node
    offboard_precland_controller_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS 
    aruco_fractal_tracker
    rtsp_publisher
    offboard_precland_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── 5. Install Config & Launch Files ──────────────────────────────────
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export info
ament_export_libraries(
  aruco_fractal_tracker_node
  rtsp_publisher_node
  offboard_precland_controller_node
)

ament_export_dependencies(
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  std_msgs
  dib_msgs
  mavros_msgs
  rclcpp_components 
  cv_bridge 
  aruco
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
