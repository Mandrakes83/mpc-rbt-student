cmake_minimum_required(VERSION 3.8)
project(student_dan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(dependencies rclcpp geometry_msgs mpc_rbt_simulator nav_msgs nav2_msgs) 

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  set(ament_cmake_copyright_FOUND TRUE)
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

add_library(${PROJECT_NAME} SHARED src/KeyboardControl.cpp src/Localization.cpp src/Planning.cpp src/MotionControl.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib)

add_executable(keyboardcontrol src/keyboard_control_node.cpp)
target_link_libraries(keyboardcontrol ${PROJECT_NAME})
install(TARGETS keyboardcontrol DESTINATION lib/${PROJECT_NAME})

add_executable(localization src/localization_node.cpp)
target_link_libraries(localization ${PROJECT_NAME})
install(TARGETS localization DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

add_executable(planning src/planning_node.cpp)
target_link_libraries(planning ${PROJECT_NAME})
install(TARGETS planning DESTINATION lib/${PROJECT_NAME})

add_executable(motioncontrol src/motion_control_node.cpp)
target_link_libraries(motioncontrol ${PROJECT_NAME})
install(TARGETS motioncontrol DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()