#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "ros/ros.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

sensor_msgs::PointCloud pointcloud;

void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "bottle") {
            ROS_INFO("@@bottle detected@@");
            int64_t x = (thing->xmin + thing->xmax) / 2;
            int64_t y = (thing->ymin + thing->ymax) / 2;
            ROS_INFO("x: [%ld] && y: [%ld]\n", x, y);

            for (int i = 0; i < pointcloud.points.size(); i++) {                     //다 도는게 너무 오래걸림
                ROS_INFO("%d/%d is read from clouds", i, pointcloud.points.size());  //모든 점을 다 도는지 개수로 확인
                float dx = pointcloud.points[i].x - (float)x;
                float dy = pointcloud.points[i].y - (float)y;
                ROS_INFO("%f, %f away from detected point\n", dx, dy);  //확실히 coordinate가 다름.
                if (dx < 0) dx = -dx;
                if (dy < 0) dy = -dy;
                if (dx < 1 && dy < 1) {
                    float distance = pointcloud.points[i].z;
                    ROS_INFO("@!@ the distance is [%f]\n", distance);
                    break;
                }
            }
            ROS_INFO("DONE!\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        }
    }
    ROS_INFO("\n");
}

void distance_sensor_Callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, pointcloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    ros::Subscriber image_sub = n.subscribe("/darknet_ros/bounding_boxes", 100, YOLO_Callback);
    ros::Subscriber depth_sub = n.subscribe("/camera/depth/color/points", 100, distance_sensor_Callback);

    ros::spin();

    return 0;
}