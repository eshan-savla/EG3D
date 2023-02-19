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
    int MeanK;
    float leaf_size, StddevMulThresh;
//    std::string file_path;
    bool is_filtered;
    bool do_downsample;
    bool do_stat_outrem;
    bool remove_first, remove_last;
    std::vector<int> first_ind, last_ind, reuse_ind, edge_points;
    std::unordered_map<unsigned long, unsigned long> return_index_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr returned_cloud;

    void ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &local_inliers,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud, std::vector<int> &global_inliers);
    static std::tuple<Eigen::Vector4f, float> EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                       const std::vector<int> &indices);
    static double ComputeAngularGap(const pcl::PointXYZ &origin, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_cloud,
                                  Eigen::Vector4f &plane_parameters);
    void RemoveFalseEdges(std::vector<int> &edge_point_indices);
    void CreateReturnIndexMap();

public:
    RawCloud();
    void GenerateCloud(const int &pcl_size);
    void SetDownSample(bool activate, float leaf_size);
    void SetStatOutRem(bool activate, int MeanK, float StddevMulThresh);
    void CorrectIndices(std::vector<int> &indices);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();
    void SetFirstInd(const std::vector<int> &first_ind);
    void SetLastInd(const std::vector<int> &last_ind);
    void SetReuseInd(const std::vector<int> &reuse_ind);
    std::vector<int> GetFirstInd();
    std::vector<int> GetLastInd();
    std::vector<int> GetReuseInd();
    void SetFilterCriteria(bool remove_first = false, bool remove_last = false);

    pcl::PointCloud<pcl::PointXYZ> FindEdgePoints(const int no_neighbours, const double angular_thresh_rads,
                                                  const float dist_thresh = 0.01,
                                                  const float radius = 0.1, const bool radial_search = false);



};



#endif //EG3D_RAWCLOUD_H
