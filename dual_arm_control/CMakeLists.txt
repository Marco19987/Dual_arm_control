cmake_minimum_required(VERSION 3.8)
project(dual_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(uclv_systems_lib REQUIRED)
find_package(dual_arm_control_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(include)
include_directories(include ${YAML_CPP_INCLUDE_DIRS})


ament_export_dependencies(Eigen3)


add_executable(ekf_server src/ekf_server.cpp)
target_link_libraries(ekf_server Eigen3::Eigen yaml-cpp)
ament_target_dependencies(ekf_server rclcpp geometry_msgs Eigen3 uclv_systems_lib dual_arm_control_interfaces yaml-cpp)



install(
  TARGETS ekf_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files (if any)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

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
  add_executable(test_compile test/test_compile.cpp)
  target_link_libraries(test_compile Eigen3::Eigen)
  ament_target_dependencies(test_compile uclv_systems_lib)

  add_executable(test_robots_system test/test_robots_system.cpp)
  target_link_libraries(test_robots_system Eigen3::Eigen)
  ament_target_dependencies(test_robots_system uclv_systems_lib)
  add_executable(test_estimation test/test_estimation.cpp)
  target_link_libraries(test_estimation Eigen3::Eigen)
  ament_target_dependencies(test_estimation uclv_systems_lib)
  add_executable(test_robots_system_ext test/test_robots_system_ext.cpp)
  target_link_libraries(test_robots_system_ext Eigen3::Eigen)
  ament_target_dependencies(test_robots_system_ext uclv_systems_lib)
  add_executable(test_estimation_ext test/test_estimation_ext.cpp)
  target_link_libraries(test_estimation_ext Eigen3::Eigen)
  ament_target_dependencies(test_estimation_ext uclv_systems_lib)


  add_executable(test_ekf_client test/test_ekf_client.cpp)
  target_link_libraries(test_ekf_client Eigen3::Eigen)
  ament_target_dependencies(test_ekf_client rclcpp dual_arm_control_interfaces)


  add_executable(ros_simulator test/ros_simulator.cpp)
  target_link_libraries(ros_simulator Eigen3::Eigen yaml-cpp)
  ament_target_dependencies(ros_simulator rclcpp geometry_msgs Eigen3 uclv_systems_lib dual_arm_control_interfaces yaml-cpp)

  install(
    TARGETS test_ekf_client ros_simulator
    DESTINATION lib/${PROJECT_NAME}
  )

  
endif()

ament_package()
