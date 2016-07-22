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

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} robotology_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

