cmake_minimum_required(VERSION 3.1.0)
project(yolo_reader)

find_package(realsense2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

# # Enable C++11
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_message_files(FILES BoundingBox.msg BoundingBoxes.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()
add_executable(yolo_reader src/yolo_reader.cpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${realsense_INCLUDE_DIR})

target_link_libraries(yolo_reader ${catkin_LIBRARIES} ${realsense2_LIBRARY})