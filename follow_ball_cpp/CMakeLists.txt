cmake_minimum_required(VERSION 3.8)
project(follow_ball_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/follow_ball_cpp/FollowBall.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(follow_ball_main src/follow_ball_main.cpp)
ament_target_dependencies(follow_ball_main ${dependencies})
target_link_libraries(follow_ball_main ${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  follow_ball_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
