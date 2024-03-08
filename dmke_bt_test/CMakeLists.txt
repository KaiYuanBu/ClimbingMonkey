cmake_minimum_required(VERSION 3.8)
project(dmke_bt_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(dmke_interface REQUIRED)
# find_package(btcpp_ros2_interfaces REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(arm_interfaces REQUIRED)

add_executable(dmke_bt_test1 src/dmke_bt_test1.cpp)
# add_executable(SetPosition_server src/SetPosition_server.py)

# Install
install(TARGETS dmke_bt_test1
                # mock_actuator
  DESTINATION lib/${PROJECT_NAME})


set(THIS_PACKAGE_DEPS
    rclcpp
    behaviortree_ros2
    dmke_interface)

ament_target_dependencies(dmke_bt_test1 ${THIS_PACKAGE_DEPS})
# ament_target_dependencies(SetPosition_server ${THIS_PACKAGE_DEPS})

target_include_directories(dmke_bt_test1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# target_include_directories(SetPosition_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

target_compile_features(dmke_bt_test1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install
# install(TARGETS dmke_bt_test1
#                 # SetPosition_server
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch config
# DESTINATION share/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(
  behaviortree_ros2
  dmke_interface
  rclcpp
)

ament_package()
