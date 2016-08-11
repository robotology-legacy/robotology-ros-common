cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME robotology_controller)
project(${PROJECT_NAME})

find_package(Boost REQUIRED COMPONENTS thread)

find_package(catkin COMPONENTS    roscpp
                                  rospy
                                  urdf
                                  std_msgs
                                  controller_manager
                                  robotology_msgs)

catkin_package(INCLUDE_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
#                LIBRARIES       ${PROJECT_NAME}
               )

message(STATUS " hardware_interface_VERSION is ${hardware_interface_VERSION}")

message(STATUS " hardware_interface_VERSION_MAJOR is ${hardware_interface_VERSION_MAJOR}")
message(STATUS " hardware_interface_VERSION_MINOR is ${hardware_interface_VERSION_MINOR}")
message(STATUS " hardware_interface_VERSION_PATCH is ${hardware_interface_VERSION_PATCH}")

message(STATUS " controller_manager_VERSION is ${controller_manager_VERSION}")
message(STATUS " ROS distro env is $ENV{ROS_DISTRO}")

add_definitions(-Dhardware_interface_VERSION_MAJOR=${hardware_interface_VERSION_MAJOR})
add_definitions(-Dhardware_interface_VERSION_MINOR=${hardware_interface_VERSION_MINOR})
add_definitions(-Dhardware_interface_VERSION_PATCH=${hardware_interface_VERSION_PATCH})


include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} robotology_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(service service.cpp)
target_link_libraries(service ${catkin_LIBRARIES})
