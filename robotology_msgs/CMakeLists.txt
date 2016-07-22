cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME robotology_msgs)
project(${PROJECT_NAME})

find_package(catkin COMPONENTS    std_msgs
                                  message_generation)

add_message_files(DIRECTORY msg
                  FILES controlBoardMsg.msg)

add_service_files(FILES controlBoardSrv.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS    ${CMAKE_CURRENT_SOURCE_DIR}
               CATKIN_DEPENDS  message_runtime)
