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
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(uclv_systems_lib REQUIRED)

include_directories(include)
# include_directories(${uclv_systems_lib_INCLUDE_DIRS})


ament_export_dependencies(Eigen3)


install(
  DIRECTORY include/
  DESTINATION include
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

  
endif()

ament_package()
