cmake_minimum_required(VERSION 3.0.2)
project(yolo_reader)

find_package(realsense2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_message_files(FILES BoundingBox.msg)
add_message_files(FILES BoundingBoxes.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(yolo_reader src/yolo_reader.cpp)
target_link_libraries(yolo_reader ${catkin_LIBRARIES})