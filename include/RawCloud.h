//
// Created by eshan on 07.11.22.
//

#ifndef EG3D_RAWCLOUD_H
#define EG3D_RAWCLOUD_H
#pragma once

#include "BaseCloud.h"
#include <pcl/pcl_macros.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <string>
#include "EdgeCloud.h"

class RawCloud : public BaseCloud {
private:
    unsigned int count_before, count_after;
//    std::string file_path;
    bool is_filtered;
    bool remove_first, remove_last;
    std::vector<std::size_t> first_ind, last_ind;
    void ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &local_inliers,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud, std::vector<int> &global_inliers);
    static std::tuple<Eigen::Vector4f, float> EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                       const std::vector<int> &indices);
    static double ComputeAngularGap(const pcl::PointXYZ &origin, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_cloud,
                                  Eigen::Vector4f &plane_parameters);
    void RemoveFalseEdges(std::vector<int> &edge_point_indices);

public:
    RawCloud();
    void GenerateCloud(const int &pcl_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
    void SetFirstInd(const std::vector<std::size_t> &first_ind);
    void SetLastInd(const std::vector<std::size_t> &last_ind);
    void SetFilterCriteria(bool remove_first = false, bool remove_last = false);

    pcl::PointCloud<pcl::PointXYZ> FindEdgePoints(const int no_neighbours, const double angular_thresh_rads,
                                                  const float dist_thresh = 0.01,
                                                  const float radius = 0.1, const bool radial_search = false);



};



#endif //EG3D_RAWCLOUD_H
