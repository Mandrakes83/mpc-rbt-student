cmake_minimum_required(VERSION 3.8)
project(student_dan_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set(dependencies rclcpp geometry_msgs)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  set(ament_cmake_copyright_FOUND TRUE)
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

add_library(${PROJECT_NAME} SHARED src/KeyboardControl.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib)

add_executable(keyboardcontrol src/keyboard_control_node.cpp)
target_link_libraries(keyboardcontrol ${PROJECT_NAME})
install(TARGETS keyboardcontrol DESTINATION lib/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()