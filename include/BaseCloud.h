//
// Created by eshan on 14.11.22.
//

#ifndef EG3D_BASECLOUD_H
#define EG3D_BASECLOUD_H

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


class BaseCloud {
protected:
    unsigned long cloud_size_before;
    std::vector<bool> removed_indices_;
    std::vector<int> index_map_vg;
    std::vector<int> point_shifts;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data;
    void CorrectIndicesRemoved(std::vector<int> &indices_vector);
    void CorrectIndicesMapped(std::vector<int> &indices_vector);
    void ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void MarkPoints(pcl::IndicesConstPtr &removed_indices);
    static pcl::IndicesConstPtr StatOutlierRem(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int MeanK, float StddevMulThresh);
    static std::vector<int> VoxelDownSample_(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size);
    static void CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec);
    static bool InInliers(unsigned long origin, std::vector<int> &global_inliers);


public:
    BaseCloud();
    void LoadInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud);
    void ReadCloud(const std::string &file_path);
    void SaveCloud(const std::string &file_path);
    unsigned int GetCount();
    pcl::PointCloud<pcl::PointXYZ> GetCloud();
    void StatOutlierRemoval(int MeanK, float StddevMulThresh);
    void StatOutlierRemoval(int MeanK, float StddevMulThresh, std::string &out_path);
    void StatOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int MeanK, float StddevMulThresh);
    void StatOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int MeanK, float StddevMulThresh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
    void RadOutlierRemoval(float Radius, int MinNeighbours);
    void RadOutlierRemoval(float Radius, int MinNeighbours, std::string &out_path);
    void VoxelDownSample(float leaf_size);
    void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, float leaf_size);
    void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
};


#endif //EG3D_BASECLOUD_H
