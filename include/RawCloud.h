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

/// @brief Class to process raw point cloud and find edge points.
///
/// This class provides functionalities to find edge points in growing point clouds.
class RawCloud : public BaseCloud {
private:
    int MeanK;
    float leaf_size, StddevMulThresh;
    bool is_filtered;
    bool do_downsample;
    bool do_stat_outrem;
    bool remove_first, remove_last;
    std::vector<int> first_ind, last_ind, reuse_ind, edge_points;
    std::unordered_map<unsigned long, unsigned long> return_index_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr returned_cloud;

    void ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &global_inliers,
                        Eigen::VectorXf &plane_parameters);
    static std::tuple<Eigen::Vector4f, float> EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                       const std::vector<int> &indices);
    static double ComputeAngularGap(const pcl::PointXYZ &origin, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_cloud,
                                  Eigen::Vector4f &plane_parameters);
    void RemoveFalseEdges(std::vector<int> &edge_point_indices);
    void CreateReturnIndexMap();

public:
    /// @brief Empty constructor.
    RawCloud();

    /// @brief Method to generate plane shaped cloud to test.
    /// @param[in] pcl_size Cloud size.
    void GenerateCloud(const int &pcl_size);

    /// @brief Method to set indices of first n points to be removed after edge point detection.
    ///
    /// @param[in] first_ind Vector containing indices of first n points.
    void SetFirstInd(const std::vector<int> &first_ind);

    /// @brief Method to set indices of last n points to be removed after edge point detection.
    ///
    /// @param[in] last_ind Vector containing indices of last n points.
    void SetLastInd(const std::vector<int> &last_ind);

    /// @brief Method to set indices of points which will be reused in next iteration.
    ///
    /// This setter is redundant and not used in this processes of this class.
    /// @param[in] reuse_ind Vector containing indices of reused points in next iteration
    void SetReuseInd(const std::vector<int> &reuse_ind);

    /// @brief Getter to get first n indices.
    /// @return Vector containing first n indices.
    std::vector<int> GetFirstInd();

    /// @brief Getter to get last n indices.
    /// @return Vector containing last n indices.
    std::vector<int> GetLastInd();

    /// @brief Getter to get reused indices in next iteration.
    /// @return Vector containing teused indices.
    std::vector<int> GetReuseInd();

    /// @brief Method to activate/deactivate voxel down sampling.
    ///
    /// This method activates/deactivates the down sampling of the internally stored cloud data before edge point detection.
    /// @param[in] activate Bool value to activate/deactivate down sampling.
    /// @param[in] leaf_size Leaf size to be used for creating voxel grid.

    void SetDownSample(bool activate, float leaf_size);

    /// @brief Method to activate/deactivate statistical outlier removal.
    ///
    /// This method activates/deactivates statistical outlier removal of internally stored cloud data before edge point detection.
    /// @param[in] activate Bool value to activate/deactivate statistical outlier removal.
    /// @param[in] MeanK Number of points to use for mean distance estimation
    /// @param[in] StddevMulThresh Standard deviation multiplier - Points with distances within threshhold of mean + StddevMulThresh * stddev are classified as inliers.
    void SetStatOutRem(bool activate, int MeanK, float StddevMulThresh);

    /// @brief Method to set removal criteria of false edges.
    ///
    /// Due to the nature of the edge detection algorithm, certain points of an incomplete point cloud might be falsely detected as edges. This Method
    ///     removes these false edges at the beginning or the end of point cloud section.
    /// @param[in] remove_first Activates the removal of first n points provided in SetFirstInd as false edges.
    /// @param[in] remove_last Activates the removal of last n points provided in SetLastInd as false edges.
    void SetFilterCriteria(bool remove_first = false, bool remove_last = false);

    /// @brief Method to detect edge points in a cloud.
    ///
    /// This method clusters points into neighbourhoods using a KD-Tree and then fits a plane using a RANSAC algorithm onto the neighbourhoods. Points
    ///     lying in the plane are utilized to calculate the normal of the neighbourhood. The angular gap between the inliers points is computed in
    ///     order to identify edge points.
    ///
    /// @param[in] no_neighbours Number of points in a neighbourhood
    /// @param[in] angular_thresh_rads The minimum threshhold of angular gap for a point to be considered an edge point.
    /// @param[in] dist_thresh Distance to model threshhold for point to be considered inlier
    /// @param[in] radius Radius of sphere from point to find neighbours using radial search
    /// @param[in] radial_search Do radial search instead of KNN Search in KD-Tree.
    /// @return Point cloud containing edge points.
    pcl::PointCloud<pcl::PointXYZ> FindEdgePoints(int no_neighbours, double angular_thresh_rads, float dist_thresh = 0.01,
                                                  float radius = 0.1, bool radial_search = false);

    /// @brief Method to adjust indices in provided vector to match edge points.
    ///
    /// This method removes points from provided vector which are not edge points and corrects the point indices to match the indices of the downsampled and/or
    ///     filtered cloud.
    /// @param[in, out] indices Point indices to be corrected.

    void CorrectIndices(std::vector<int> &indices);

};



#endif //EG3D_RAWCLOUD_H
