cmake_minimum_required(VERSION 3.8)
project(ice_resurfacer_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install the URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install the Meshes (if you add them later)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL # Don't fail if folder is empty
)

ament_package()