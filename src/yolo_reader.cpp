#include "librealsense2/rs.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

rs2::pipeline p;
ros::Publisher detect_pub;

void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    int is_detected = 0;
    std_msgs::Float64MultiArray detected_cars;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "car") {
            is_detected = 1;
            ROS_INFO("CAR is DETECTED");
            // Block program until frames arrive
            rs2::frameset frames = p.wait_for_frames();

            // Try to get a frame of a depth image
            rs2::depth_frame depth = frames.get_depth_frame();

            int64_t x = (thing->xmin + thing->xmax) / 2;
            int64_t y = (thing->ymin + thing->ymax) / 2;

            // Query the distance from the camera
            float detected_depth = depth.get_distance(x, y);

            ROS_INFO("x: [%ld] && y: [%ld] && depth: [%f]\n", x, y, detected_depth);
            detected_cars.data.push_back(detected_depth);

            ROS_INFO("Loop Finished\n");
        }
    }
    if (is_detected == 1) {
        detect_pub.publish(detected_cars);
    }
    ROS_INFO("Done\n\n");
}

int main(int argc, char **argv) {
    p.start();

    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    ros::Subscriber image_sub = n.subscribe("/darknet_ros/bounding_boxes", 100, YOLO_Callback);
    detect_pub = n.advertise<std_msgs::Float64MultiArray>("detected_cars", 100);

    ros::spin();

    return 0;
}