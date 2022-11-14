//
// Created by eshan on 11.11.22.
//

#ifndef EG3D_EDGECLOUD_H
#define EG3D_EDGECLOUD_H

#pragma once
#include "BaseCloud.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>


class EdgeCloud : public BaseCloud{
private:
    std::vector<int> edge_points_indices;
    pcl::PointCloud<pcl::Normal>::Ptr edge_normals;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    std::vector<pcl::PointIndices> clusters;

    void EstimateNormals(int neighbours_K);

public:
    EdgeCloud();
    EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud);
    void LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud);
    void SegmentEdges(const int &neighbours_K1, const int &neighbours_K2, const float &smoothness_thresh,
                      const float &curvature_thresh);
};


#endif //EG3D_EDGECLOUD_H
