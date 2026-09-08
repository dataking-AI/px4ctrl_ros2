cmake_minimum_required(VERSION 3.8)
project(px4_ctrl_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(px4_ctrl_node
  src/controller.cpp
  src/px4_ctrl_node.cpp
)
target_include_directories(px4_ctrl_node PRIVATE
  include
)
ament_target_dependencies(px4_ctrl_node
  rclcpp
  px4_msgs
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  Eigen3
)
target_link_libraries(px4_ctrl_node
  Eigen3::Eigen
)

install(TARGETS
  px4_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_vehicle_command_authorization
    test/test_vehicle_command_authorization.cpp
  )
  target_include_directories(test_vehicle_command_authorization PRIVATE
    src
  )

  ament_add_gtest(test_thrust_mapping
    test/test_thrust_mapping.cpp
    src/controller.cpp
  )
  target_include_directories(test_thrust_mapping PRIVATE
    include
  )
  ament_target_dependencies(test_thrust_mapping
    rclcpp
    Eigen3
  )
  target_link_libraries(test_thrust_mapping
    Eigen3::Eigen
  )

  ament_add_gtest(test_coordinate_conversion
    test/test_coordinate_conversion.cpp
    src/controller.cpp
  )
  target_include_directories(test_coordinate_conversion PRIVATE
    include
  )
  ament_target_dependencies(test_coordinate_conversion
    rclcpp
    Eigen3
  )
  target_link_libraries(test_coordinate_conversion
    Eigen3::Eigen
  )

  ament_add_gtest(test_land_detector
    test/test_land_detector.cpp
  )
  target_include_directories(test_land_detector PRIVATE
    src
  )
  ament_target_dependencies(test_land_detector
    rclcpp
    Eigen3
  )
  target_link_libraries(test_land_detector
    Eigen3::Eigen
  )

  ament_add_gtest(test_thrust_estimation_support
    test/test_thrust_estimation_support.cpp
  )
  target_include_directories(test_thrust_estimation_support PRIVATE
    src
  )
  ament_target_dependencies(test_thrust_estimation_support
    rclcpp
    Eigen3
  )
  target_link_libraries(test_thrust_estimation_support
    Eigen3::Eigen
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
