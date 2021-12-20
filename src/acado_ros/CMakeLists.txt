cmake_minimum_required(VERSION 3.5)
project(acado_ros)

set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(acado_msgs REQUIRED)
find_package( ACADO REQUIRED )


add_library(mpc_solver
externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/codegen/acado_qpoases_interface.cpp
  model/codegen/acado_integrator.c
  model/codegen/acado_solver.c
  model/codegen/acado_auxiliary_functions.c)


target_include_directories(mpc_solver PUBLIC
model/codegen/
externals/qpoases
externals/qpoases/INCLUDE
externals/qpoases/SRC)

link_directories(${ACADO_ENV_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( . ${ACADO_ENV_INCLUDE_DIRS} )

include_directories(
  include
  ${ACADO_ENV_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()

#add_executable(simple_mpc src/simple_mpc.cpp)
#target_link_libraries( simple_mpc ${ACADO_ENV_LIBRARIES_DIRS})
#ament_target_dependencies(simple_mpc rclcpp std_msgs nav_msgs ${ACADO_ENV_LIBRARIES_DIRS})

add_executable(simple_mpc src/simple_mpc.cpp)
target_link_libraries(simple_mpc  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(simple_mpc rclcpp std_msgs nav_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(goal_reach src/goal_reach_test.cpp)
target_link_libraries(goal_reach  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(goal_reach rclcpp std_msgs nav_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(goal_reach_srv src/goal_reach_srv.cpp)
target_link_libraries(goal_reach_srv  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(goal_reach_srv rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(var_speed src/var_speed_srv.cpp)
target_link_libraries(var_speed  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(var_speed rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(circle_lane src/circle_lane_srv.cpp)
target_link_libraries(circle_lane  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(circle_lane rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(goal_reach_carla src/goal_reach_carla.cpp)
target_link_libraries(goal_reach_carla  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(goal_reach_carla rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(goal_reach_intersection src/goal_reach_intersection.cpp)
target_link_libraries(goal_reach_intersection  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(goal_reach_intersection rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(intersection_wide_lane src/intersection_wide_lane.cpp)
target_link_libraries(intersection_wide_lane  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(intersection_wide_lane rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(goal_reach_idm src/goal_reach_idm.cpp)
target_link_libraries(goal_reach_idm  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(goal_reach_idm rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(multi_circle src/multi_circle_acado.cpp)
target_link_libraries(multi_circle  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(multi_circle rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(multi_circle_curve src/multi_circle_curve.cpp)
target_link_libraries(multi_circle_curve  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(multi_circle_curve rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})

add_executable(combined_mpc src/combined_mpc.cpp)
target_link_libraries(combined_mpc  ${ACADO_ENV_SHARED_LIBRARIES} mpc_solver)
ament_target_dependencies(combined_mpc rclcpp std_msgs nav_msgs acado_msgs ${ACADO_ENV_SHARED_LIBRARIES})


install(TARGETS
  simple_mpc
  goal_reach
  goal_reach_srv
  var_speed
  circle_lane
  goal_reach_carla
  goal_reach_intersection
  intersection_wide_lane
  goal_reach_idm
  multi_circle
  multi_circle_curve
  combined_mpc
  DESTINATION lib/${PROJECT_NAME})
