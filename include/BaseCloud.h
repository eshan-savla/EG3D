//
// Created by eshan on 14.11.22.
//

#ifndef EG3D_BASECLOUD_H
#define EG3D_BASECLOUD_H

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>



class BaseCloud {
protected:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data;
    void ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    static void CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec);
    static bool InInliers(unsigned long &origin, std::vector<int> &global_inliers);


public:
    BaseCloud();
    void LoadInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud);
    void ReadCloud(const std::string &file_path);
    void SaveCloud(const std::string &file_path);
    unsigned int GetCount();
    pcl::PointCloud<pcl::PointXYZ> GetCloud();
};


#endif //EG3D_BASECLOUD_H
