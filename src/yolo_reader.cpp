#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
// #include "sensor_msgs/image_encodings.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

ros::Publisher detect_pub;          //made it global variable so that it can be called at subscriber function
cv_bridge::CvImageConstPtr cv_ptr;  //will be keep updated

void depth_Callback(const sensor_msgs::Image::ConstPtr &msg) {  //1.get depth map keep updated
    //simply share the image since we will not modify it
    try {
        cv_ptr = cv_bridge::toCvShare(msg, "16UC1");  //16bit unsigned int
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {  //2.keep check if YOLO detected car
    std::vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    int is_detected = 0;
    std_msgs::UInt16MultiArray detected_car_depths;
    for (std::vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "bottle") {
            is_detected = 1;
            ROS_INFO("CAR is DETECTED");

            int64_t x = (thing->xmin + thing->xmax) / 2;
            int64_t y = (thing->ymin + thing->ymax) / 2;

            ROS_INFO("hi");

            int16_t detected_car_depth = cv_ptr->image.at<int16_t>(x, y);  //get depth of (x,y) from cv::Mat::Image
            detected_car_depths.data.push_back(detected_car_depth);        //push it to depth array

            ROS_INFO("x: [%ld] && y: [%ld] && depth: [%d]\n", x, y, detected_car_depth);  //for debugging
        }
    }
    ROS_INFO("Loop Finished\n");
    ROS_INFO("size: %d", detected_car_depths.data.size());
    if (is_detected == 1) {
        ROS_INFO("detect_pub published\n");
        detect_pub.publish(detected_car_depths);  //3. publish the depth array only when car is detected
    }
    ROS_INFO("Done\n\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("camera/depth/image_rect_raw", 100, depth_Callback);

    ros::Subscriber image_sub = n.subscribe("/darknet_ros/bounding_boxes", 100, YOLO_Callback);

    detect_pub = n.advertise<std_msgs::UInt16MultiArray>("detected_cars", 100);  //it is defined as global variable

    ros::spin();

    return 0;
}