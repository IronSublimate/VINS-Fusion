//
// Created by hou on 2022/11/19.
//

#ifndef VINS_FUSION_KITTI_SYNC_H
#define VINS_FUSION_KITTI_SYNC_H

#include <unordered_map>
#include <string>
#include <tuple>
#include <cstdint>

#include <boost/filesystem.hpp>

static const unordered_map<std::string, std::tuple<std::string, int, int>> raw_to_odom = {
        {"2011_10_03_drive_0027_sync", {"00", 0,    4540}},
        {"2011_10_03_drive_0042_sync", {"01", 0,    1100}},
        {"2011_10_03_drive_0034_sync", {"02", 0,    4660}},
        {"2011_09_26_drive_0067_sync", {"03", 0,    800}},
        {"2011_09_30_drive_0016_sync", {"04", 0,    270}},
        {"2011_09_30_drive_0018_sync", {"05", 0,    2760}},
        {"2011_09_30_drive_0020_sync", {"06", 0,    1100}},
        {"2011_09_30_drive_0027_sync", {"07", 0,    1100}},
        {"2011_09_30_drive_0028_sync", {"08", 1100, 5170}},
        {"2011_09_30_drive_0033_sync", {"09", 0,    1590}},
        {"2011_09_30_drive_0034_sync", {"10", 0,    1200}},
};

std::tuple<int, int> get_start_end_id(const std::string &kitti_path_str) {

    boost::filesystem::path kitti_path(kitti_path_str);
    kitti_path.remove_trailing_separator();
    auto data_name = kitti_path.filename().string();

    auto it = raw_to_odom.find(data_name);
    if (it == raw_to_odom.cend()) {
        return {INT_MIN, INT_MAX};
    } else {
        return {get<1>(it->second), get<2>(it->second)};
    }
}

#endif //VINS_FUSION_KITTI_SYNC_H
