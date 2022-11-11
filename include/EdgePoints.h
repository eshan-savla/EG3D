//
// Created by eshan on 11.11.22.
//

#ifndef EG3D_EDGEPOINTS_H
#define EG3D_EDGEPOINTS_H

#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>


class EdgePoints {
private:
    std::vector<int> edge_points_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_points;
    pcl::PointCloud<pcl::Normal>::Ptr edge_normals;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    std::vector<pcl::PointIndices> clusters;

    void EstimateNormals(const int neighbours_K);

public:
    EdgePoints(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud);
    void Save(const std::string& path);
    void SegmentEdges(const int &neighbours_K1, const int &neighbours_K2, const float &smoothness_thresh,
                      const float &curvature_thresh);
    unsigned int GetCount();
};


#endif //EG3D_EDGEPOINTS_H
