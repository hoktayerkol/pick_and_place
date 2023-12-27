cmake_minimum_required(VERSION 3.8)
project(pick_and_place_pilz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(dependencies
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED 
  src/pick_and_place/robot_arm_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(rb_node src/robot_arm_node_main.cpp)
ament_target_dependencies( rb_node ${dependencies} )
target_link_libraries(rb_node ${PROJECT_NAME})
# target_include_directories(rb_node PUBLIC include)
target_compile_features(rb_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  ${PROJECT_NAME}
  rb_node 
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
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
endif()

ament_package()
