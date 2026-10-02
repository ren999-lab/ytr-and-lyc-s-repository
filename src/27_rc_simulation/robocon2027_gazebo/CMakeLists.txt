cmake_minimum_required(VERSION 3.8)
project(robocon2027_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# arena world + launch + documentation + generator
install(DIRECTORY worlds launch docs
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/gen_arena.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
