#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "ros/ros.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

void myCallback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        int64_t x = (thing->xmin + thing->xmax) / 2;
        int64_t y = (thing->ymin + thing->ymax) / 2;
        ROS_INFO("name: %s && x: [%ld] && y: [%ld]\n", thing->Class.c_str(), x, y);
    }
    ROS_INFO("\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    message_filters::Subscriber<BoundingBoxes> detector(n, "detector", 1000);
    message_filters::Subscriber;

    ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1000, myCallback);

    ros::spin();

    return 0;
}