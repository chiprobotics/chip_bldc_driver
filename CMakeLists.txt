cmake_minimum_required(VERSION 2.8.3)
project(chip_bldc_driver)

find_package(catkin REQUIRED COMPONENTS roscpp serial message_generation std_msgs)

# Create messages in the msg folder
add_message_files(FILES Command.msg Feedback.msg Status.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include/chip_bldc_driver ${catkin_INCLUDE_DIRS})

add_library(bldc_controller src/bldc_controller)
target_link_libraries(bldc_controller ${catkin_LIBRARIES})

add_library(bldc_serial src/bldc_serial)
target_link_libraries(bldc_serial ${catkin_LIBRARIES})

add_library(crc16 src/crc16)
target_link_libraries(crc16 ${catkin_LIBRARIES})

add_executable(bldc_driver_node src/bldc_driver_node.cpp)
target_link_libraries(bldc_driver_node bldc_controller bldc_serial crc16 ${catkin_LIBRARIES})
set_target_properties(bldc_driver_node PROPERTIES OUTPUT_NAME bldc_driver_node PREFIX "")

install(TARGETS bldc_driver_node bldc_controller bldc_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# if more folders like config add below
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
