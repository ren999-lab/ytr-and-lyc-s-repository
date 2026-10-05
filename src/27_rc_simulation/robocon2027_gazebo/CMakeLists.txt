cmake_minimum_required(VERSION 3.8)
project(robocon2027_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(rc_planar_move SHARED src/rc_planar_move.cpp)
target_compile_features(rc_planar_move PUBLIC cxx_std_17)
target_include_directories(rc_planar_move PRIVATE include ${GAZEBO_INCLUDE_DIRS})
target_link_directories(rc_planar_move PRIVATE ${GAZEBO_LIBRARY_DIRS})
ament_target_dependencies(rc_planar_move
  gazebo_ros rclcpp geometry_msgs nav_msgs tf2_geometry_msgs tf2_ros)
target_link_libraries(rc_planar_move ${GAZEBO_LIBRARIES})
install(TARGETS rc_planar_move LIBRARY DESTINATION lib)

# arena world + launch + documentation + generator
install(DIRECTORY worlds launch docs urdf rviz config
  DESTINATION share/${PROJECT_NAME})


ament_package()
