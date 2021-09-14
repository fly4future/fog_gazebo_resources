cmake_minimum_required(VERSION 3.5)
project(fog_gazebo_resources)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# -------------------------- #
# PROTOBUF MESSAGE GENERATOR #
# -------------------------- #

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

# PROTOBUF_IMPORT_DIRS has to be set before
# find_package is called
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
  message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
endif()

#--------------------#
# Message Generation #
#--------------------#
set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

set(gps_msg
  msgs/SITLGps.proto
  )

PROTOBUF_GENERATE_CPP(GPS_PROTO_SRCS GPS_PROTO_HDRS ${gps_msg})

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------


include_directories(
  include
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(gazebo_ros_gps_plugin SHARED src/gazebo_ros_gps_plugin.cpp ${GPS_PROTO_SRCS})

ament_target_dependencies(gazebo_ros_gps_plugin
  rclcpp
  std_msgs
  std_srvs
  gazebo
  gazebo_ros
  )

target_link_libraries(gazebo_ros_gps_plugin
  ${rclcpp_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_include_directories(include)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY
  worlds
  DESTINATION .
  )

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
  )

install(DIRECTORY
  scripts
  DESTINATION .
  )

install(TARGETS
  gazebo_ros_gps_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
