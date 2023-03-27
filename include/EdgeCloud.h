//
// Created by eshan on 11.11.22.
//

#ifndef EG3D_EDGECLOUD_H
#define EG3D_EDGECLOUD_H

#pragma once
#include "BaseCloud.h"
#include "BoundingBox.h"
#include "Region2D.h"
#include "SensorInfo.h"


#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>


/// @brief Class to process edge points and detect edges.
class EdgeCloud : public BaseCloud{
private:
    std::vector<int> edge_points_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_points;
    std::vector<pcl::Indices> neighbours_map;
    std::vector<Eigen::Vector3f> vectors_map;
    std::vector<bool> reused_indices_map;
    std::vector<int> reused_inds_start, reused_inds_end, reused_inds_prev;
    std::vector<std::vector<int>> clusters;
    std::vector<unsigned int> num_pts_in_segment;
    std::vector<int> point_labels;
    std::vector<int> previous_seeds;
    std::vector<bool> false_edges;
    std::vector<unsigned int> previous_sizes;
    SensorSpecs sensorSpecs;
    SensorCoords coords_first, coords_last;

    float seg_tag_thresh;
    float bef_aft_ratio;
    int total_num_of_segmented_pts;
    int total_num_of_segments;
    unsigned int previous_size;
    unsigned int first_point_index;
    unsigned int false_points_previous;
//    int repeated_indexes_count;
    float angle_thresh;
    bool is_appended;
    bool override_cont;
    bool downsample;
    float leaf_size;
    bool outrem;
    int MeanK;
    float StddevMulThresh;

    std::unordered_map<int, Eigen::Vector3f> segment_vectors;
    std::unordered_map<int, bool> finished_segments;
    std::unordered_map<int, int> segment_seed_map;
    Eigen::Vector3f scan_direction;

    struct Segment {
        int segment_id;
        int origin_seed, last_point;
        Eigen::Vector3f origin_seed_dir, segment_dir;
    };

    std::unordered_map<int, Segment> segment_infos;
    void Init();

    int GrowSegment(const int &initial_seed, const int &segment_id, const int &neighbours_k, bool use_original = false);
    bool CheckPoint(const int &current_seed, const int &neighbour, bool &is_a_seed);
    void ShiftIndices(std::vector<int> &indices);
    int ExtendSegments(const int neighbours_k);
    bool IsFinished(const int &label);
    void SpecialKNeighboursSearch(const std::size_t &point_index, int neighbours_k, std::vector<int> &neighbours_id, std::vector<float> &neighbours_dist);
    static std::pair<int, int> findEntryWithLargestValue(std::unordered_map<int, int> sampleMap);
    void StoreClusterInfo(const unsigned long num_of_segs, const long num_of_unseg);
    
public:
    /// @brief Empty Constructor
    EdgeCloud();

    /// @brief Constructor loads in cloud provided in argument
    /// @param[in] cloud Point cloud to be processed and segmented
    explicit EdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /// @brief Constructor extracts edge points from parent cloud and saves them internally.
    /// @param[in] edge_indices Vector containing indices of edge points
    /// @param[in] parent_cloud Pointer to parent cloud from which edge points are to be extracted
    EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud);

    /// @brief Method to activate/deavtivate down sampling of new points added to internal cloud data.
    ///
    /// This method reduces point count of the new points added via AddPoints method using voxel grids.
    /// @param[in] down_sample boolean value to activate/deactivate voxel downsampling
    /// @param[in] leaf_size Leaf size to be used for creating voxel grid.
    void SetDownsampling(bool down_sample, float leaf_size = 0.001f);

    /// @brief Method to activate/deactivate filter points using point neighbourhood statistics.
    ///
    /// This method filters points which lie outside a distance threshhold determined by a mean distance in a neighbourhood.
    /// @param[in] outrem boolean value to activate/deactivate statistical outlier removal
    /// @param[in] MeanK Number of nearest neighbours to use for mean distance estimation
    /// @param[in] StddevMulThresh Standard deviation multiplier to use for setting distance threshhold for outlier removal
    void SetStatOutRem(bool outrem, int MeanK = 20, float StddevMulThresh = 1.0);

    /// @brief Set point index values of points repeated from previous iteration.
    /// @param[in] indices Indices of points reused from previous cloud segment. 
    void SetEndIndices(const std::vector<int> &indices);

    /// @brief Setter to set angular distance threshhold between point vector and scan direction.
    /// @param[in] seg_tag_thresh Angular distance threshhold in radians.
    void SetTagThresh(const float &seg_tag_thresh);

    /// @brief Setter to provide direction in which object is scanned.
    /// @param[in] scan_direction Eigen vector of scan direction
    void SetScanDirection(const Eigen::Vector3f &scan_direction);

    /// @brief Setter to set sensor specifications.
    ///
    /// This method sets the dimensions of sensor field of view.
    /// @param[in] width Maximum width (y direction) the sensor can scan in meters.
    /// @param[in] depth Maximum depth (x direction) the sensor can scan in meters.
    /// @param[in] height Maximum height (z direction) the sensor can scan in meters.
    void SetSensorSpecs(float width, float depth, float height);

    /// @brief Setter to set position and coordinate system of sensor at beginning or end of section
    /// @param[in] xAxis X axis of coordinate system of sensor
    /// @param[in] yAxis Y axis of coordinate system of sensor
    /// @param[in] zAxis Z axis of coordinate system of sensor
    /// @param[in] sensorPos Position of the sensor in world coordinate system
    /// @param[in] section "first" to set sensor values at the beginning of the section and "last" to set sensor values at end of the section
    void SetSensorCoords(std::vector<float> xAxis, std::vector<float> yAxis, std::vector<float> zAxis, std::vector<float> sensorPos, const std::string& section);

    /// @brief Method to add new points to internal point cloud.
    /// @param[in] new_points Pointer to new edge points to be added.
    void AddPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_points);

    /// @brief Method to detect and label false edges.
    ///
    /// This method detects false edges by either creating an oriented bounding box around the new points added to the internal cloud or by using
    ///     sensor position and orientation to determine position of false points. These methods are much more imprecise compared to the method
    ///     available in RawCloud class and is still under development.
    /// @param[in] region_width Estimated width of region containing false points
    /// @param[in] use_coords Boolean flag to use sensor position instead of creating bounding box.
    void RemoveFalseEdges(float region_width, bool use_coords);

    /// @brief Method to compute direction vectors and create point neighbourhoods.
    ///
    /// This method uses a KD-Tree to cluster points into neighbours and then applies a ransac algorithm to fit a line model onto the neighbours.
    ///     Inliers of this model will be grouped together as neighbours and the direction vector of the line will be computed and stored.
    /// @param[in] neighbours_K Number of points in neighbourhood to fit line model onto
    /// @param[in] dist_thresh Distance threshhold for determining inliers of the line model
    /// @param[in] override Override boolean to recompute point vectors and neighbours of all points.
    void ComputeVectors(const int &neighbours_K, const float &dist_thresh, const bool &override);

    /// @brief Method to apply region growing algorithm to segment points.
    ///
    /// This method makes use of the point vectors and neighbourhoods computed in method ComputeVectors to grow and segment regions from a seed point.
    /// @param[in] neighbours_k Number of points in neighbourhood. Should be equal to value provided in ComputeVectors
    /// @param[in] angle_thresh Maximum angular distance between two neighbouring point vectors to be segmented together
    /// @param[in] sort Sort order of points to be segmented to begin with points with highest chance of forming segments.
    void
    ApplyRegionGrowing(const int &neighbours_k, const float &angle_thresh, const bool &sort);

    /// @brief All in one method to create line segments.
    ///
    /// This method combines point vector computation and line segmentation for complete point clouds.
    /// @param[in] neighbours_K Number of points in neighbourhood to fit line into
    /// @param[in] dist_thresh Distance threshhold for determining inliers of the line model
    /// @param[in] angle_thresh Maximum angular distance between two neighbouring point vectors to be segmented together
    /// @param[in] sort Sort order of points to be segmented to begin with points with highest chance of forming segments.
    /// @param[in] override_cont Override boolean to restart segmentation
    void SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &angle_thresh, const bool &sort,
                      const bool &override_cont);

    /// @brief Method to create cloud with colours corresponding to segments.
    /// @param[in] path Path and filename to save cloud.
    void CreateColouredCloud(const std::string &path);

    /// @brief Method to assemble detected segements.
    ///
    /// This method collects all the detected regions in the internal cloud data and assembles them into segments/clusters. Each cluster contains all 
    ///     points belonging to respective segment.
    void AssembleRegions();

};

inline bool Compare(std::pair<unsigned long, int> i, std::pair<unsigned long, int> j) {
    return (i.first > j.first);
}


#endif //EG3D_EDGECLOUD_H
