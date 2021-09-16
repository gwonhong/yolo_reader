#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "ros/ros.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

int64_t x, y = 0;
void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "bottle") {
            ROS_INFO("@@bottle detected@@");
            x = (thing->xmin + thing->xmax) / 2;
            y = (thing->ymin + thing->ymax) / 2;
            ROS_INFO("x: [%ld] && y: [%ld]\n", x, y);
        }
    }
    ROS_INFO("\n");
}

void distance_sensor_Callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sensor_msgs::PointCloud converted_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, converted_cloud);
    int i;
    for (i = 0; i < converted_cloud.points.size(); i++) {
        // ROS_INFO("X : %f", converted_cloud.points[i].x);
        // ROS_INFO("Y : %f", converted_cloud.points[i].y);
        // ROS_INFO("Z : %f", converted_cloud.points[i].z);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    ros::Subscriber image_sub = n.subscribe("/darknet_ros/bounding_boxes", 100, YOLO_Callback);
    ros::Subscriber depth_sub = n.subscribe("/camera/depth/color/points", 100, distance_sensor_Callback);

    ros::spin();

    return 0;
}