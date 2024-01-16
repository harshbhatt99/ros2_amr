cmake_minimum_required(VERSION 3.8)
project(ros2_amr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Extra packages
find_package(ros2_amr REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch models worlds meshes config params ros2_amr action src include
  DESTINATION share/${PROJECT_NAME})


# (HB) Add library for shared folder (same name as behavior_plugin.xml)
add_library(ros2_amr_behavior_plugin SHARED
  src/send_sms.cpp)

# (HB) Add executables
add_executable(send_sms src/send_sms.cpp)
ament_target_dependencies(send_sms rclcpp)
add_executable(twilio src/twilio.cpp)
ament_target_dependencies(twilio rclcpp)



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

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

ament_package()
