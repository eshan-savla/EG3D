//
// Created by eshan on 07.11.22.
//

#ifndef EG3D_RAWCLOUD_H
#define EG3D_RAWCLOUD_H

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <string>

class RawCloud {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    unsigned int count_before, count_after;
//    std::string file_path;
    bool is_filtered;

    void ReadCloud(const std::string &file_path);
    void ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &inliers,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr refined_cloud);
    void ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    static std::tuple<Eigen::Vector4f, float> EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                       const std::vector<int> &indices);

public:
    RawCloud(const std::string &file_path);
    unsigned int GetCount();
    unsigned int StatOutlierRemoval(const int MeanK, const float StddevMulThresh);
    unsigned int StatOutlierRemoval(const int MeanK, const float StddevMulThresh, std::string &out_path);
    unsigned int RadOutlierRemoval(const float Radius, const int MinNeighbours);
    unsigned int RadOutlierRemoval(const float Radius, const int MinNeighbours, std::string &out_path);
    void FindEdgePoints(const int no_neighbours, double angular_thresh, const float dist_thresh = 0.01,
                        const float radius = 1.0, bool radial_search = false);


};



#endif //EG3D_RAWCLOUD_H
