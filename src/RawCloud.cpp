//
// Created by eshan on 07.11.22.
//

#include "RawCloud.h"


RawCloud::RawCloud(const std::string &file_path) : raw_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    ReadCloud(file_path);
}

void RawCloud::ReadCloud(const std::string &file_path) {
    int status = pcl::io::loadPCDFile(file_path, *raw_cloud);
    if (status == -1) {
        PCL_ERROR("Couldn't read file");
    }
    std::cout << "Loaded " << raw_cloud->width * raw_cloud->height << " data points from file with fields: " << std::endl;
    for (const auto &point: *raw_cloud) {
        std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
        break;
    }
}
