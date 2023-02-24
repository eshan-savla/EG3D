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


class EdgeCloud : public BaseCloud{
private:
    std::vector<int> edge_points_indices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_points;
//    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
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
    
public:
    EdgeCloud();
    explicit EdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud);

    void SetDownsampling(bool down_sample, float leaf_size = 0.001f);
    void SetStatOutRem(bool outrem, int MeanK = 20, float StddevMulThresh = 1.0);
    void SetReuseIndices(const std::vector<int> &indices);
    void SetEndIndices(const std::vector<int> &indices);
    void SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &angle_thresh, const bool &sort,
                      const bool &override_cont);
    void ComputeVectors(const int &neighbours_K, const float &dist_thresh, const bool &override);
    void RemoveFalseEdges(float region_width, bool use_coords);
    void
    ApplyRegionGrowing(const int &neighbours_k, const float &angle_thresh, const bool &sort);
    void CreateColouredCloud(const std::string &path);
    void AddPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_points);
    void SetTagThresh(const float &seg_tag_thresh);
    void SetScanDirection(const Eigen::Vector3f &scan_direction);
    void SetSensorSpecs(float width, float depth, float height);
    void SetSensorCoords(std::vector<float> xAxis, std::vector<float> yAxis, std::vector<float> zAxis, std::vector<float> sensorPos, const std::string& section);
    void AssembleRegions();

};

inline bool Compare(std::pair<unsigned long, int> i, std::pair<unsigned long, int> j) {
    return (i.first > j.first);
}


#endif //EG3D_EDGECLOUD_H
