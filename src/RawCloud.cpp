//
// Created by eshan on 07.11.22.
//

#include "RawCloud.h"
#include <cmath>
#include <vector>
#include <random>
#include "EdgeCloud.h"


RawCloud::RawCloud() : BaseCloud() {
    is_filtered = false;
}

void RawCloud::GenerateCloud(const int &pcl_size) {
    cloud_data->width    = pcl_size;
    cloud_data->height   = 1;
    cloud_data->is_dense = false;
    cloud_data->points.resize (cloud_data->width * cloud_data->height);
    for (int i = 0; i < static_cast<int>(cloud_data->size ()); ++i)
    {
        {
            (*cloud_data)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
            (*cloud_data)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
            if( i % 2 == 0)
                (*cloud_data)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
            else
                (*cloud_data)[i].z = -1 * ((*cloud_data)[i].x + (*cloud_data)[i].y);
        }
        pcl::io::savePCDFileASCII("../data/orignal.pcd", *cloud_data);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RawCloud::GetCloud() {
    return cloud_data;
}

void RawCloud::VoxelDownSample(const float &leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> vg_sampler;
    vg_sampler.setInputCloud(cloud_data);
    vg_sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg_sampler.filter(*cloud_data);
    is_filtered = true;
}
unsigned int RawCloud::StatOutlierRemoval(const int MeanK, const float StddevMulThresh) {
    if (count_before != GetCount()) count_before = GetCount();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_data);
    sor.setMeanK(MeanK);
    sor.setStddevMulThresh(StddevMulThresh);
    sor.filter(*cloud_data);
    is_filtered = true;
    count_after = GetCount();
    return count_before - count_after;
}

unsigned int RawCloud::StatOutlierRemoval(const int MeanK, const float StddevMulThresh, std::string &out_path) {
    unsigned int diff = StatOutlierRemoval(MeanK, StddevMulThresh);
    pcl::io::savePCDFileASCII(out_path, *cloud_data);
    return diff;
}

unsigned int RawCloud::RadOutlierRemoval(const float Radius, const int MinNeighbours) {
    if (count_before != GetCount()) count_before = GetCount();
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_data);
    ror.setRadiusSearch(Radius);
    ror.setMinNeighborsInRadius(MinNeighbours);
    ror.filter(*cloud_data);
    is_filtered = true;
    count_after = GetCount();
    return count_before - count_after;
}

unsigned int RawCloud::RadOutlierRemoval(const float Radius, const int MinNeighbours, std::string &out_path) {
    unsigned int diff = RadOutlierRemoval(Radius, MinNeighbours);
    pcl::io::savePCDFileASCII(out_path, *cloud_data);
    return diff;
}

pcl::PointCloud<pcl::PointXYZ> RawCloud::FindEdgePoints(const int no_neighbours, const double angular_thresh_rads,
                                                        const float dist_thresh, const float radius,
                                                        const bool radial_search) {
    if (!is_filtered)
        PCL_WARN("Downsampling or filtering the point cloud is recommended!");
    const int K = no_neighbours;
    std::vector<int> edge_points_global;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_data);
#pragma omp parallel default(none) shared(angular_thresh_rads, edge_points_global, dist_thresh, radius, radial_search, kdtree, K)
    {
        std::vector<int> edge_points_thread;
        #pragma omp for nowait
        for (std::size_t i = 0; i < static_cast<int>(cloud_data->size()); ++i) {
            std::vector<int> neighbour_ids(K);
            std::vector<float> neighbour_sqdist(K);
            const pcl::PointXYZ origin = cloud_data->at(i);
            if (radial_search) {
                if (kdtree.radiusSearch(origin, radius, neighbour_ids, neighbour_sqdist) < 3)
                    continue;
            } else {
                if (kdtree.nearestKSearch(origin, K, neighbour_ids, neighbour_sqdist) < 3)
                    continue;
            }
            std::vector<int> local_inliers, global_inliers;
            pcl::PointCloud<pcl::PointXYZ>::Ptr neighbours_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            ComputeInliers(dist_thresh, neighbour_ids, local_inliers, neighbours_cloud, global_inliers);
            if (!InInliers(i, global_inliers) || local_inliers.size() < 3) {
                continue;
            } else {
                Eigen::Vector4f plane_parameters;
                float curvature;
                std::tie(plane_parameters, curvature) = EstimateNormals(neighbours_cloud, local_inliers);
                double G0 = ComputeAngularGap(origin, neighbours_cloud, plane_parameters);
                if (G0 >= angular_thresh_rads)
                    edge_points_thread.push_back(i);
            }
        }
#pragma omp critical
        edge_points_global.insert(edge_points_global.end(), edge_points_thread.begin(), edge_points_thread.end());
    }
    pcl::PointCloud<pcl::PointXYZ> return_cloud;
    pcl::copyPointCloud(*cloud_data, edge_points_global, return_cloud);
    return return_cloud;
}



void RawCloud::ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &local_inliers,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud, std::vector<int> &global_inliers) {

    ExtractIndices(neighbours, refined_cloud);
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (refined_cloud));
    pcl::RandomSampleConsensus<pcl::PointXYZ> local_plane_fitter (model_p);
    local_plane_fitter.setDistanceThreshold(dist_thresh);
    local_plane_fitter.computeModel();
    local_plane_fitter.getInliers(local_inliers);
    pcl::copyPointCloud(*refined_cloud, local_inliers, *refined_cloud);

    for (int &elem : local_inliers) {
        global_inliers.push_back(neighbours[elem]);
    }
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

double RawCloud::ComputeAngularGap(const pcl::PointXYZ &origin, pcl::PointCloud<pcl::PointXYZ>::Ptr &local_cloud, Eigen::Vector4f &plane_parameters) {
    unsigned int cloud_size = local_cloud->width * local_cloud->height;
    unsigned int first = (random() * (cloud_size/RAND_MAX));
    if (first == 0) first = random() % 200;
    unsigned int second = random() * (cloud_size - 1)/RAND_MAX;
    pcl::PointXYZ first_point = local_cloud->points[first];
    pcl::PointXYZ second_point = local_cloud->points[second];
    Eigen::Vector3f u;
    CreateVector(first_point, second_point, u);
    Eigen::Vector3f e_first_vector = first_point.getVector3fMap();
    Eigen::Vector3f e_second_vector = second_point.getVector3fMap();
    Eigen::Vector3f normal = plane_parameters.head<3>();
    Eigen::Vector3f v = u.cross(normal);
    std::vector<double> thetas_global, deltas;

#pragma omp parallel default(none) shared(origin, u, v, local_cloud, thetas_global)
    {
        std::vector<double> thetas_thread;
#pragma omp for nowait
        for (pcl::PointXYZ &point: *local_cloud) {
            Eigen::Vector3f op;
            CreateVector(origin, point, op);
            double du = op.dot(u);
            if (du == 0) continue;
            double dv = op.dot(v);
            double theta = std::atan2(du, dv);
            if (theta < 0)
                theta += 2 * M_PI;
            thetas_thread.push_back(theta);
        }
#pragma omg critical
        thetas_global.insert(thetas_global.end(), thetas_thread.begin(), thetas_thread.end());
    }
    if (thetas_global.size() <= 1) return 0.0;
    else {
        std::sort(thetas_global.begin(), thetas_global.end());
        for (int i = 0; i < thetas_global.size() - 1; ++i)
            deltas.push_back(thetas_global.at(i + 1) - thetas_global.at(i));

        return *std::max_element(deltas.begin(), deltas.end());
    }
}

