//
// Created by eshan on 07.11.22.
//

#ifndef EG3D_RAWCLOUD_H
#define EG3D_RAWCLOUD_H
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <string>

class RawCloud {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    unsigned int count_before, count_after;
//    std::string file_path;
    bool is_filtered;

    void ReadCloud(const std::string &file_path);
    void ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &local_inliers,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud, std::vector<int> &global_inliers);
    void ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    static std::tuple<Eigen::Vector4f, float> EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                       const std::vector<int> &indices);
    static double ComputeAngularGap(const pcl::PointXYZ &origin, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_cloud,
                                  Eigen::Vector4f &plane_parameters);
    static void CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec);
    static bool InInliers(int &origin, std::vector<int> &global_inliers);
public:
    RawCloud(const std::string &file_path);
    RawCloud(const bool gen_cloud, const int pcl_size);
    unsigned int GetCount();
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
    unsigned int StatOutlierRemoval(const int MeanK, const float StddevMulThresh);
    unsigned int StatOutlierRemoval(const int MeanK, const float StddevMulThresh, std::string &out_path);
    unsigned int RadOutlierRemoval(const float Radius, const int MinNeighbours);
    unsigned int RadOutlierRemoval(const float Radius, const int MinNeighbours, std::string &out_path);
    void FindEdgePoints(const int no_neighbours, const double angular_thresh_rads,
                        std::vector<int> &edge_points_global, const float dist_thresh = 0.01,
                        const float radius = 0.1, const bool radial_search = false);
    void VoxelDownSample(const float &leaf_size);


};



#endif //EG3D_RAWCLOUD_H
