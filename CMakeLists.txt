cmake_minimum_required(VERSION 3.8)
project(plain_slam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)



# lio_3d_node

add_executable(lio_3d_node
  src/lio_3d_node.cpp
  src/ros_utils.cpp
  src/io.cpp
  src/lie_algebra.cpp
  src/lio_3d_interface.cpp
  src/imu_preintegrator.cpp
  src/gravity_direc_estimator.cpp
  src/keyframe_detector.cpp
  src/voxel_grid_filter.cpp
  src/normal_map.cpp
  src/hierarchical_geometric_observer.cpp
  src/joint_optimizer.cpp)

target_include_directories(lio_3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(lio_3d_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  lio_3d_node
  "rclcpp"
  "rclpy"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_eigen"
  "visualization_msgs"
)

target_link_libraries(lio_3d_node yaml-cpp)

install(TARGETS lio_3d_node DESTINATION lib/${PROJECT_NAME})



# slam_3d_node

add_executable(slam_3d_node
  src/slam_3d_node.cpp
  src/ros_utils.cpp
  src/lie_algebra.cpp
  src/slam_3d_interface.cpp
  src/io.cpp
  src/keyframe_detector.cpp
  src/voxel_grid_filter.cpp
  src/gicp.cpp
  src/graph_optimizer.cpp)

target_include_directories(slam_3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(slam_3d_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  slam_3d_node
  "rclcpp"
  "rclpy"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_eigen"
  "visualization_msgs"
)

target_link_libraries(slam_3d_node yaml-cpp)

install(TARGETS slam_3d_node DESTINATION lib/${PROJECT_NAME})



# pc2_to_pcd_node

add_executable(pc2_to_pcd_node
  src/pc2_to_pcd_node.cpp
  src/ros_utils.cpp
  src/io.cpp
  src/voxel_grid_filter.cpp)

target_include_directories(pc2_to_pcd_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(pc2_to_pcd_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  pc2_to_pcd_node
  "rclcpp"
  "rclpy"
  "sensor_msgs"
  "nav_msgs"
  "visualization_msgs"
  "tf2_msgs"
  "tf2_ros"
)

install(TARGETS pc2_to_pcd_node DESTINATION lib/${PROJECT_NAME})



# lidar_lidar_calibration_node

add_executable(lidar_lidar_calibration_node
  src/lidar_lidar_calibration_node.cpp
  src/io.cpp
  src/voxel_grid_filter.cpp
  src/lie_algebra.cpp
  src/gicp.cpp)

target_include_directories(lidar_lidar_calibration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

target_compile_features(lidar_lidar_calibration_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  lidar_lidar_calibration_node
  "rclcpp"
  "rclpy"
)

install(TARGETS lidar_lidar_calibration_node DESTINATION lib/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)


