cmake_minimum_required(VERSION 3.8)
project(dual_arm_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
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
find_package(uclv_robot_lib REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(uclv_robot_ros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(uclv_robot_ros REQUIRED)
find_package(uclv_robot_ros_msgs REQUIRED)
find_package(uclv_aruco_detection_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(uclv_slipping_control_common REQUIRED)
find_package(uclv_slipping_control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)



include_directories(include)
include_directories(include ${YAML_CPP_INCLUDE_DIRS})


ament_export_dependencies(Eigen3)


add_executable(slipping_control_client_server src/slipping_control_client.cpp)
ament_target_dependencies(slipping_control_client_server rclcpp std_srvs uclv_slipping_control_common std_msgs uclv_slipping_control_msgs rclcpp_action) 



add_executable(object_model_simulator src/object_model_simulator.cpp)
target_link_libraries(object_model_simulator Eigen3::Eigen yaml-cpp)
ament_target_dependencies(object_model_simulator rclcpp sensor_msgs geometry_msgs std_srvs)


add_executable(joint_mux src/joint_mux.cpp)
ament_target_dependencies(joint_mux rclcpp sensor_msgs)


add_executable(joint_demux src/joint_demux.cpp)
ament_target_dependencies(joint_demux rclcpp sensor_msgs)

add_executable(tf_publisher_from_topic src/tf_publisher_from_topic.cpp)
target_link_libraries(tf_publisher_from_topic Eigen3::Eigen)
ament_target_dependencies(tf_publisher_from_topic rclcpp geometry_msgs Eigen3  tf2 tf2_geometry_msgs)

add_executable(topic_publisher_from_tf src/topic_publisher_from_tf.cpp)
target_link_libraries(topic_publisher_from_tf Eigen3::Eigen)
ament_target_dependencies(topic_publisher_from_tf rclcpp geometry_msgs Eigen3  tf2 tf2_geometry_msgs)

add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node Eigen3::Eigen yaml-cpp)
ament_target_dependencies(demo_node rclcpp geometry_msgs Eigen3 uclv_systems_lib dual_arm_control_interfaces yaml-cpp uclv_robot_ros
                           uclv_robot_ros_msgs uclv_aruco_detection_interfaces std_srvs)

add_executable(absolute_pose_publisher src/absolute_pose_publisher.cpp)
target_link_libraries(absolute_pose_publisher Eigen3::Eigen)
ament_target_dependencies(absolute_pose_publisher rclcpp geometry_msgs Eigen3)


add_executable(ekf_server src/ekf_server.cpp)
target_link_libraries(ekf_server Eigen3::Eigen yaml-cpp)
ament_target_dependencies(ekf_server rclcpp geometry_msgs Eigen3 uclv_systems_lib dual_arm_control_interfaces yaml-cpp std_srvs)


add_executable(cooperative_robots_server src/cooperative_robots_server.cpp)
target_link_libraries(cooperative_robots_server Eigen3::Eigen)
ament_target_dependencies(cooperative_robots_server rclcpp geometry_msgs Eigen3 uclv_robot_ros_msgs dual_arm_control_interfaces sensor_msgs realtime_tools)


add_executable(object_pose_control_node src/object_pose_control_node.cpp)
target_link_libraries(object_pose_control_node Eigen3::Eigen)
ament_target_dependencies(object_pose_control_node rclcpp geometry_msgs Eigen3 std_srvs)

add_executable(internal_force_control_node src/internal_force_control_node.cpp)
target_link_libraries(internal_force_control_node Eigen3::Eigen)
ament_target_dependencies(internal_force_control_node rclcpp geometry_msgs Eigen3 std_srvs)


add_library(${PROJECT_NAME} 
SHARED
  src/robots/MotomanSIA5FExt.cpp
  src/robots/LBRiiwa7Ext.cpp
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN_INCLUDE_DIRS}
  )
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
"Eigen3"
 pluginlib
 uclv_robot_lib
)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)



install(
  TARGETS ${PROJECT_NAME} 
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ekf_server cooperative_robots_server absolute_pose_publisher object_pose_control_node 
          internal_force_control_node demo_node tf_publisher_from_topic joint_demux joint_mux object_model_simulator topic_publisher_from_tf 
          slipping_control_client_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files (if any)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)


install(
    DIRECTORY  urdf rviz
    DESTINATION share/${PROJECT_NAME}
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

  add_executable(test_robots_system_ext_forces test/test_robots_system_ext_forces.cpp)
  target_link_libraries(test_robots_system_ext_forces Eigen3::Eigen)
  ament_target_dependencies(test_robots_system_ext_forces uclv_systems_lib)



  add_executable(test_ekf_client test/test_ekf_client.cpp)
  target_link_libraries(test_ekf_client Eigen3::Eigen)
  ament_target_dependencies(test_ekf_client rclcpp dual_arm_control_interfaces)


  add_executable(ros_simulator test/ros_simulator.cpp)
  target_link_libraries(ros_simulator Eigen3::Eigen yaml-cpp)
  ament_target_dependencies(ros_simulator rclcpp geometry_msgs Eigen3 uclv_systems_lib dual_arm_control_interfaces yaml-cpp std_srvs)


  add_executable(test_robots_spring_object_system test/test_robots_spring_object_system.cpp)
  target_link_libraries(test_robots_spring_object_system Eigen3::Eigen)
  ament_target_dependencies(test_robots_spring_object_system uclv_systems_lib)

  add_executable(test_robots_spring_object_system_ext test/test_robots_spring_object_system_ext.cpp)
  target_link_libraries(test_robots_spring_object_system_ext Eigen3::Eigen)
  ament_target_dependencies(test_robots_spring_object_system_ext uclv_systems_lib)

  add_executable(test_robots_spring_object_system_ext_grasp test/test_robots_spring_object_system_ext_grasp.cpp)
  target_link_libraries(test_robots_spring_object_system_ext_grasp Eigen3::Eigen)
  ament_target_dependencies(test_robots_spring_object_system_ext_grasp uclv_systems_lib)

  

  install(
    TARGETS test_ekf_client ros_simulator 
    DESTINATION lib/${PROJECT_NAME}
  )

  
endif()

ament_package()
