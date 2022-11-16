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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>



class EdgeCloud : public BaseCloud{
private:
    std::vector<int> edge_points_indices;
    pcl::PointCloud<pcl::Normal>::Ptr edge_normals;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    std::vector<std::vector<int>> clusters;

    void EstimateNormals(int neighbours_K);
    void ComputeInliers(const int &neighbours_K, const float &dist_thresh,
                        std::unordered_map<int, std::vector<int>> &local_neighbours,
                        std::unordered_map<int, Eigen::VectorXf> &point_vectors);

public:
    EdgeCloud();
    EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud);
    void LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud);
    void SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &smooth_thresh);
};


#endif //EG3D_EDGECLOUD_H
