//
// Created by eshan on 14.11.22.
//

#include "BaseCloud.h"

BaseCloud::BaseCloud(): cloud_data(new pcl::PointCloud<pcl::PointXYZ>){

}

void BaseCloud::ReadCloud(const std::string &file_path) {
    int status = pcl::io::loadPCDFile(file_path, *cloud_data);
    if (status == -1) {
        PCL_ERROR("Couldn't read file");
    }
    std::cout << "Loaded " << GetCount() << " data points" << std::endl;
    std::cout << "Point cloud is organised: " << cloud_data->isOrganized() << std::endl;
}

void BaseCloud::SaveCloud(const std::string &file_path) {
    pcl::io::savePCDFileASCII(file_path, *cloud_data);
}

unsigned int BaseCloud::GetCount() {
    return cloud_data->height * cloud_data->width;
}

void BaseCloud::CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec) {
    Eigen::Vector3f e_1_vector = pt1.getVector3fMap();
    Eigen::Vector3f e_2_vector = pt2.getVector3fMap();
    vec = e_2_vector - e_1_vector;
}

void BaseCloud::ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::IndicesPtr indices_ptr (new pcl::Indices (indices));
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_data);
    extractor.setIndices(indices_ptr);
    extractor.setNegative(false);
    extractor.filter(*cloud);
}

bool BaseCloud::InInliers(int &origin, std::vector<int> &global_inliers) {
    if (global_inliers.empty()) return false;
    if (std::find(global_inliers.begin(), global_inliers.end(), origin) != global_inliers.end())
        return true;
    else
        return false;
}

