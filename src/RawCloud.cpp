//
// Created by eshan on 07.11.22.
//

#include "RawCloud.h"
#include <cmath>
#include <vector>


RawCloud::RawCloud(const std::string &file_path) : raw_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    is_filtered = false;
    count_before = 0;
    count_after = 0;
    ReadCloud(file_path);
}

void RawCloud::ReadCloud(const std::string &file_path) {
    int status = pcl::io::loadPCDFile(file_path, *raw_cloud);
    if (status == -1) {
        PCL_ERROR("Couldn't read file");
    }
    count_before = raw_cloud->width * raw_cloud->height;
    std::cout << "Loaded " << count_before << " data points" << std::endl;
    std::cout << "Point cloud is organised: " << raw_cloud->isOrganized() << std::endl;
}
unsigned int RawCloud::GetCount() {
    return raw_cloud->height * raw_cloud->width;
}

unsigned int RawCloud::StatOutlierRemoval(const int MeanK, const float StddevMulThresh) {
    if (count_before != GetCount()) count_before = GetCount();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(raw_cloud);
    sor.setMeanK(MeanK);
    sor.setStddevMulThresh(StddevMulThresh);
    sor.filter(*raw_cloud);
    is_filtered = true;
    count_after = GetCount();
    return count_before - count_after;
}

unsigned int RawCloud::StatOutlierRemoval(const int MeanK, const float StddevMulThresh, std::string &out_path) {
    unsigned int diff = StatOutlierRemoval(MeanK, StddevMulThresh);
    pcl::io::savePCDFileASCII(out_path, *raw_cloud);
    return diff;
}

unsigned int RawCloud::RadOutlierRemoval(const float Radius, const int MinNeighbours) {
    if (count_before != GetCount()) count_before = GetCount();
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(raw_cloud);
    ror.setRadiusSearch(Radius);
    ror.setMinNeighborsInRadius(MinNeighbours);
    ror.filter(*raw_cloud);
    is_filtered = true;
    count_after = GetCount();
    return count_before - count_after;
}

unsigned int RawCloud::RadOutlierRemoval(const float Radius, const int MinNeighbours, std::string &out_path) {
    unsigned int diff = RadOutlierRemoval(Radius, MinNeighbours);
    pcl::io::savePCDFileASCII(out_path, *raw_cloud);
    return diff;
}

void RawCloud::FindEdgePoints(const int no_neighbours, double angular_thresh, const float dist_thresh,
                              const float radius, bool radial_search) {
    const int K = no_neighbours;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(raw_cloud);
    std::vector<int> neighbour_ids(K);
    std::vector<float> neighbour_sqdist(K);

    for (pcl::PointXYZ const &point: raw_cloud->points) {
        if (radial_search) {
            if (kdtree.radiusSearch(point, radius, neighbour_ids, neighbour_sqdist) < 3)
                continue;
        } else {
            if (kdtree.nearestKSearch(point, K, neighbour_ids, neighbour_sqdist) < 3)
                continue;
        }
       std::vector<int> inliers;
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbours_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        ComputeInliers(dist_thresh, neighbour_ids, inliers, neighbours_cloud);
        Eigen::Vector4f plane_parameters; float curvature;
        std::tie(plane_parameters, curvature) = EstimateNormals(neighbours_cloud, inliers);
        //TODO: implement angular gap algorithm
    }
}

void RawCloud::ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &inliers,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr refined_cloud) {

    ExtractIndices(neighbours, refined_cloud);
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (refined_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> local_plane_fitter (model_p);
    local_plane_fitter.setDistanceThreshold(dist_thresh);
    local_plane_fitter.computeModel();
    local_plane_fitter.getInliers(inliers);
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr save_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*neighbour_cloud, inliers, *save_pcl);
    pcl::io::savePCDFileASCII("../data/section_plane.pcd", *save_pcl);*/
}

void RawCloud::ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::IndicesPtr indices_ptr (new pcl::Indices (indices));
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(raw_cloud);
    extractor.setIndices(indices_ptr);
    extractor.setNegative(false);
    extractor.filter(*cloud);
}

std::tuple<Eigen::Vector4f, float> RawCloud::EstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                             const std::vector<int> &indices) {
    pcl::IndicesPtr indices_ptr (new pcl::Indices (indices));
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> estimator;
    Eigen::Vector4f plane_parameters;
    float curvature;
    estimator.setNumberOfThreads(0);
    estimator.computePointNormal(*input_cloud, *indices_ptr, plane_parameters, curvature);
    return std::make_tuple(plane_parameters, curvature);
}