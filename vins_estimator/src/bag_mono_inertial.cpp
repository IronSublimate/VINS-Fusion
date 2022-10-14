//
// Created by hou on 2022/10/13.
//
#include <iostream>
#include <iostream>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1") {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    } else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    return ptr->image;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << '\n'
                  << "Usage: vins_node rosbag config/euroc/euroc_stereo_imu_config.yaml"
                  << '\n';
        return -1;
    }
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");

    rosbag::Bag bag;
    bag.open(argv[1]);
    std::string bag_path(argv[1]);
    auto pos = bag_path.rfind('/') + 1;
    std::string bag_name = bag_path.substr(pos, bag_path.size() - pos - 4); //bag name without prefix
//    cerr<<bag_name<<"\n";
    rosbag::View vb(bag);

    Estimator estimator;

    std::string config_file = argv[2];
    printf("config_file: %s\n", argv[2]);

    readParameters(config_file);
    estimator.setParameter();

    registerPub(n);

    for (auto it = vb.begin(); it != vb.end(); ++it) {
        if (it->getTopic() == IMU_TOPIC) {
//            cerr<<m.getTime()<<"\n";
            auto imu_msg = it->instantiate<sensor_msgs::Imu>();
            double t = imu_msg->header.stamp.toSec();
            double dx = imu_msg->linear_acceleration.x;
            double dy = imu_msg->linear_acceleration.y;
            double dz = imu_msg->linear_acceleration.z;
            double rx = imu_msg->angular_velocity.x;
            double ry = imu_msg->angular_velocity.y;
            double rz = imu_msg->angular_velocity.z;
            Vector3d acc(dx, dy, dz);
            Vector3d gyr(rx, ry, rz);
            estimator.inputIMU(t, acc, gyr);
        } else if (it->getTopic() == IMAGE0_TOPIC) {
//            auto img_ptr = it->instantiate<sensor_msgs::CompressedImage>();
//            auto cv_ptr = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::MONO8);
            auto img_ptr = it->instantiate<sensor_msgs::Image>();
            auto img = cv_bridge::toCvCopy(img_ptr, sensor_msgs::image_encodings::MONO8)->image;
//            cv::Mat image=cv_ptr->image({0,0,cv_ptr->image.cols,640});
//            img({0, 640, img.cols, img.rows - 640}) = 111;
            auto time = img_ptr->header.stamp.toSec();
            estimator.inputImage(time, img);
        } else {

        }
//        break;
    }
//    SLAM.Shutdown();
    string save_path = "./output/" + bag_name + ".txt";
//    SLAM.SaveTrajectoryTUM(save_path);
    bag.close();
    return 0;
}