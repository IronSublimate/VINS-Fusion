/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"

#include "save_path.h"
#include "kitti_sync.h"

using namespace std;
using namespace Eigen;

Estimator estimator;
ros::Publisher pubGPS;

static std::ofstream gps_result;
static int start_id, end_id;
static double baseTime;

void pub_gps(double imgTime, double lat, double lon, double alt, double pos_accuracy, double navstat, double numsats);

static void save_gps_path(const nav_msgs::OdometryConstPtr &msg) {
    gps_result.setf(ios::fixed, ios::floatfield);
    gps_result.precision(12);
    gps_result << msg->header.stamp.toSec() << " ";
//    gps_result.precision(5);
    gps_result << msg->pose.pose.position.x << " "
               << msg->pose.pose.position.y << " "
               << msg->pose.pose.position.z << " "
               << msg->pose.pose.orientation.w << " "
               << msg->pose.pose.orientation.x << " "
               << msg->pose.pose.orientation.y << " "
               << msg->pose.pose.orientation.z << "\n";
}

/**
 * 2013-05-28 11:36:56.932268800 to double in second
 * @param s
 * @return
 */
static double string_to_unix_time(const std::string &datetime_str) {
    auto p = datetime_str.find('.');
    if (p == std::string::npos) {
        return 0;
    }
    std::istringstream is(datetime_str.substr(p, datetime_str.size() - p));
    double d;
    is >> d;

    std::tm tm = {};
    std::istringstream ss(datetime_str.substr(0, p));
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

    auto tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));
    auto unix_timestamp = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch()).count();

    return double(unix_timestamp) + d;
}

std::vector<double> load_timestamp(const std::string &dataPath) {
    std::vector<double> imageTimeList;
    std::fstream file(dataPath);
    if (not file.is_open()) {
        printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
        return {};
    }
    std::string line;
    while (std::getline(file, line)) {
        auto unix_time = string_to_unix_time(line);
        if (unix_time == 0) {
            continue;
        }
        imageTimeList.push_back(unix_time);
    }
    return imageTimeList;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
//    ros::NodeHandle n2;
    auto sb = n.subscribe<nav_msgs::Odometry>("/globalEstimator/global_odometry", 100, save_gps_path);

    if (argc != 3) {
        printf("please intput: rosrun vins kitti_gps_test [config file] [data folder] \n"
               "for example: rosrun vins kitti_gps_test "
               "~/catkin_ws/src/VINS-Fusion/config/kitti_raw/kitti_10_03_config.yaml "
               "/media/tony-ws1/disk_D/kitti/2011_10_03/2011_10_03_drive_0027_sync/ \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    string sequence = argv[2];
    printf("read sequence: %s\n", argv[2]);
    string dataPath = sequence + "/";

//    auto start_end = get_start_end_id(argv[2]);
//    start_id = std::get<0>(start_end);
//    end_id = std::get<1>(start_end);
    auto output_file = save_kitti_path(argv[2], "kitti360");
    gps_result.open(output_file + "/gps.txt"); // ./output/kitti/2011_10_03_drive_0027_sync/2022-11-16-44-56-00/gps.csv

    pubGPS = n.advertise<sensor_msgs::NavSatFix>("/gps", 1000);

    // load image list
    auto imageTimeList = load_timestamp(dataPath + "image_00/timestamps.txt");
    if (imageTimeList.empty()) {
        ROS_BREAK();
        return 0;
    }

    // load gps list
    auto dataPath2 = dataPath;
    auto gps_path = dataPath2.replace(dataPath.find("data_2d_raw"), sizeof("data_2d_raw") - 1, "data_poses");
    auto GPSTimeList = load_timestamp(gps_path + "oxts/timestamps.txt");;
    if (GPSTimeList.empty()) {
        ROS_BREAK();
        return 0;
    }

    readParameters(config_file);
    estimator.setParameter();
    registerPub(n);


    FILE *outFile;
    outFile = fopen((output_file + "/odom.txt").c_str(), "w");
    if (outFile == NULL)
        printf("Output path dosen't exist: %s\n", output_file.c_str());
    string leftImagePath, rightImagePath;
    cv::Mat imLeft, imRight;

    int s = std::max(0, start_id);
    int e =int(imageTimeList.size());

    if (imageTimeList[0] < GPSTimeList[0])
        baseTime = imageTimeList[0];
    else
        baseTime = GPSTimeList[0];

    for (int i = 0; i < e; i++) {
        if (ros::ok()) {

            //printf("base time is %f\n", baseTime);
            printf("process image %d\n", (int) i);
            stringstream ss;
            ss << setfill('0') << setw(10) << i;
            leftImagePath = dataPath + "image_00/data_rect/" + ss.str() + ".png";
            rightImagePath = dataPath + "image_01/data_rect/" + ss.str() + ".png";
            printf("%s\n", leftImagePath.c_str());
            printf("%s\n", rightImagePath.c_str());

            imLeft = cv::imread(leftImagePath, cv::IMREAD_GRAYSCALE);
            imRight = cv::imread(rightImagePath, cv::IMREAD_GRAYSCALE);
            if (imLeft.empty()) {
                cerr << endl << "Failed to load image at: "
                     << leftImagePath << endl;
                continue;
            }
            double imgTime = imageTimeList[i];

            // load gps
            FILE *GPSFile;
            string GPSFilePath = gps_path + "oxts/data/" + ss.str() + ".txt";
            GPSFile = std::fopen(GPSFilePath.c_str(), "r");
            if (GPSFile == NULL) {
                printf("cannot find file: %s\n", GPSFilePath.c_str());
            } else {
                double lat, lon, alt, roll, pitch, yaw;
                double vn, ve, vf, vl, vu;
                double ax, ay, az, af, al, au;
                double wx, wy, wz, wf, wl, wu;
                double pos_accuracy, vel_accuracy;
                double navstat, numsats;
                double velmode, orimode;

                fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
                //printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);
                fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
                //printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
                fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
                //printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);
                fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
                //printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);
                fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode,
                       &orimode);
                //printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
                //	    pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);

            std::fclose(GPSFile);

            pub_gps(imgTime, lat, lon, alt, pos_accuracy, navstat, numsats);
            }

            estimator.inputImage(imgTime, imLeft, imRight);

            Eigen::Matrix<double, 4, 4> pose;
            estimator.getPoseInWorldFrame(pose);
            if (outFile != NULL) {
//                fprintf(outFile, "%f %f %f %f %f %f %f %f %f %f %f %f\n", pose(0, 0), pose(0, 1), pose(0, 2),
//                        pose(0, 3),
//                        pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
//                        pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));
                Eigen::Quaterniond Q;
                Q = pose.block<3, 3>(0, 0);
                Eigen::Vector3d T = pose.block<3, 1>(0, 3);
                fprintf(outFile, "%f %f %f %f %f %f %f %f\n",
                        imgTime,
                        T.x(), T.y(), T.z(),
                        Q.w(), Q.x(), Q.y(), Q.z());
            }


            // cv::imshow("leftImage", imLeft);
            // cv::imshow("rightImage", imRight);
            // cv::waitKey(2);
            ros::spinOnce();
        } else
            break;
    }
    if (outFile != NULL)
        fclose(outFile);
    gps_result.close();
    return 0;
}

void pub_gps(double imgTime, double lat, double lon, double alt, double pos_accuracy, double navstat, double numsats) {
    constexpr int pub_period = 32;
    static int cnt = pub_period;
    if (cnt == pub_period) {
        sensor_msgs::NavSatFix gps_position;
        gps_position.header.frame_id = "NED";
        gps_position.header.stamp = ros::Time(imgTime);
        gps_position.status.status = navstat;
        gps_position.status.service = numsats;
        gps_position.latitude = lat;
        gps_position.longitude = lon;
        gps_position.altitude = alt;
        gps_position.position_covariance[0] = pos_accuracy;
        //printf("pos_accuracy %f \n", pos_accuracy);
        pubGPS.publish(gps_position);
        cnt = 0;
    }
    ++cnt;
}

