#include "librealsense2/rs.hpp"
#include "ros/ros.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

rs2::pipeline p;

p.start();

void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "bottle") {
            ROS_INFO("@@bottle detected@@");
            // Block program until frames arrive
            rs2::frameset frames = p.wait_for_frames();

            // Try to get a frame of a depth image
            rs2::depth_frame depth = frames.get_depth_frame();

            int64_t x = (thing->xmin + thing->xmax) / 2;
            int64_t y = (thing->ymin + thing->ymax) / 2;

            // Query the distance from the camera
            float detected_depth = depth.get_distance(x, y);

            ROS_INFO("x: [%ld] && y: [%ld] && depth: [%f]\n", x, y, detected_depth);

            ROS_INFO("DONE!\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
    }
    ROS_INFO("\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    ros::Subscriber image_sub = n.subscribe("/darknet_ros/bounding_boxes", 100, YOLO_Callback);

    ros::spin();

    return 0;
}