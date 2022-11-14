//
// Created by eshan on 07.11.22.
//

#ifndef EG3D_RAWCLOUD_H
#define EG3D_RAWCLOUD_H
#pragma once

#include "BaseCloud.h"
#include <pcl/pcl_macros.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <string>
#include "EdgeCloud.h"

class RawCloud : public BaseCloud {
private:
    unsigned int count_before, count_after;
//    std::string file_path;
    bool is_filtered;

    void ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &local_inliers,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud, std::vector<int> &global_inliers);
    static std::tuple<Eigen::Vector4f, float> EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                       const std::vector<int> &indices);
    static double ComputeAngularGap(const pcl::PointXYZ &origin, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_cloud,
                                  Eigen::Vector4f &plane_parameters);
    static bool InInliers(int &origin, std::vector<int> &global_inliers);
public:
    RawCloud();
    void GenerateCloud(const int &pcl_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
    unsigned int StatOutlierRemoval(int MeanK, float StddevMulThresh);
    unsigned int StatOutlierRemoval(int MeanK, float StddevMulThresh, std::string &out_path);
    unsigned int RadOutlierRemoval(float Radius, int MinNeighbours);
    unsigned int RadOutlierRemoval(float Radius, int MinNeighbours, std::string &out_path);
    EdgeCloud FindEdgePoints(int no_neighbours, double angular_thresh_rads,
                             std::vector<int> &edge_points_global, float dist_thresh = 0.01,
                             float radius = 0.1, const bool radial_search = false);
    void VoxelDownSample(const float &leaf_size);


};



#endif //EG3D_RAWCLOUD_H
