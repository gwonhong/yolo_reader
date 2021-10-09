#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "yolo_reader/BoundingBox.h"
#include "yolo_reader/BoundingBoxes.h"

using namespace std;

cv_bridge::CvImageConstPtr cv_ptr;  //will be keep updated
vector<tuple<int64_t, int64_t, uint8_t>> detected_cars;
ofstream outFile;
cv_bridge::CvImageConstPtr img_ptr;  //will be keep updated
char buffer[128];
int total_detection_count = 0;

void depth_Callback(const sensor_msgs::Image::ConstPtr &msg) {  //get depth map keep updated
    //simply share the image since we will not modify it
    try {
        cv_ptr = cv_bridge::toCvShare(msg, "");  //16bit unsigned int
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void YOLO_Callback(const yolo_reader::BoundingBoxes::ConstPtr &msg) {  //keep check if YOLO detected car
    vector<yolo_reader::BoundingBox> things = msg->bounding_boxes;
    int is_detected = 0;

    for (vector<yolo_reader::BoundingBox>::iterator thing = things.begin(); thing != things.end(); thing++) {
        if (thing->Class == "chair") {
            is_detected = 1;
            //ROS_INFO("CAR is DETECTED");

            int64_t x = (thing->xmin + thing->xmax) / 2;
            int64_t y = (thing->ymin + thing->ymax) / 2;
            uint8_t detected_car_depth = cv_ptr->image.at<uchar>(y, x);     //get depth of (x,y) from cv::Mat::Image
            detected_cars.push_back(make_tuple(x, y, detected_car_depth));  //push it to depth vector

            //ROS_INFO("x: [%ld] && y: [%ld] && depth: [%d]\n", x, y, detected_car_depth);  //for debugging
        }
    }
    if (is_detected == 1) {
        ROS_INFO("%d\n", total_detection_count);
        sprintf(buffer, "./yolo_reader/img/%d.png\n", total_detection_count);
        cv::imwrite(buffer, img_ptr->image);                                       //write image with name of count.png
        outFile << total_detection_count << "\t" << detected_cars.size() << endl;  //current x, y will be added later
        for (int i = 0; i < detected_cars.size(); i++) {
            outFile << get<0>(detected_cars[i]) << "\t" << get<1>(detected_cars[i]) << "\t" << unsigned(get<2>(detected_cars[i])) << endl;
        }
        outFile << endl;
        total_detection_count++;
        detected_cars.clear();
    }
}

void image_Callback(const sensor_msgs::Image::ConstPtr &msg) {
    //simply share the image since we will not modify it
    try {
        img_ptr = cv_bridge::toCvShare(msg, "");  //16bit unsigned int
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "yolo_reader");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Subscriber depth_sub = it.subscribe("camera/aligned_depth_to_color/image_raw", 100, depth_Callback);

    ros::Subscriber detect_sub = n.subscribe("/darknet_ros/bounding_boxes", 100, YOLO_Callback);
    image_transport::Subscriber image_sub = it.subscribe("/darknet_ros/detection_image", 100, image_Callback);

    outFile.open("./yolo_reader/data.txt");

    ros::spin();

    return 0;
}