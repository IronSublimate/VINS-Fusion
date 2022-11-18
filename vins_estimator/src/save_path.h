//
// Created by hou on 2022/11/16.
//

#ifndef VINS_FUSION_UTILS_H
#define VINS_FUSION_UTILS_H

#include <string>
#include <vector>
#include <boost/filesystem.hpp>

std::string save_kitti_path(const std::string &kitti_path_str) {
//std::string path(argv[3]);
    boost::filesystem::path kitti_path(kitti_path_str);
    kitti_path.remove_trailing_separator();
    auto data_name = kitti_path.filename();

    std::time_t time = std::time({});
    char timeString[sizeof("yyyy-mm-dd-hh-mm-ss")];
    std::strftime(timeString, sizeof(timeString), "%Y-%m-%d-%H-%M-%S", std::gmtime(&time));
    auto save_dir = "output/kitti" / data_name/ timeString;

    boost::filesystem::create_directories(save_dir);

    return save_dir.string();
}

#endif //VINS_FUSION_UTILS_H
