#include "ros/ros.h"
#include "sensor_msgs/Image.msg"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

std_msgs::UInt8MultiArray depths;
ros::Publisher detect_pub;

void depth_Callback(const sensor_msgs::Image::ConstPtr &msg) {
    // for (std::vector<std_msgs::UInt8>::iterator depth = msg ~~iterate msg->data) {
    //     depths.data.push_back(*depth);  //iterate whole map and push them all to depths
    // }
    ROS_INFO("height: %d && width: %d\n\n\n\n", msg->height, msg->width);
}

void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    int is_detected = 0;
    std_msgs::UInt8MultiArray detected_cars;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "car") {
            is_detected = 1;
            ROS_INFO("CAR is DETECTED");

            int64_t x = (thing->xmin + thing->xmax) / 2;
            int64_t y = (thing->ymin + thing->ymax) / 2;

            std_msgs::UInt8 detected_depth = depths[x + y * width];  //depth of (x,y)

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
    ros::Subscriber depth_sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, depth_Callback);  //update depth map everytime

    detect_pub = n.advertise<std_msgs::Float64MultiArray>("detected_cars", 100);

    ros::spin();

    return 0;
}