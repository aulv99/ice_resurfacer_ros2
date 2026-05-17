cmake_minimum_required(VERSION 3.8)
project(ice_resurfacer_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# This is the magic block that installs your folders!
install(DIRECTORY
  launch
  maps
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()