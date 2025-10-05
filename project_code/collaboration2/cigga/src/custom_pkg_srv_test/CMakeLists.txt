cmake_minimum_required(VERSION 3.5)
project(custom_pkg_srv_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CustomService.srv"
  DEPENDENCIES
)

# Python 스크립트 설치
install(PROGRAMS
  server.py
  client.py
  DESTINATION lib/${PROJECT_NAME}
)

# srv 파일 설치
install(DIRECTORY
  srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_package()