cmake_minimum_required(VERSION 3.1.0)
project(yolo_reader)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp message_generation message_runtime std_msgs sensor_msgs)

add_message_files(FILES BoundingBox.msg BoundingBoxes.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime std_msgs sensor_msgs) 

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})

add_executable(yolo_reader src/yolo_reader.cpp)

target_link_libraries(yolo_reader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})