//
// Created by eshan on 07.11.22.
//

#include "RawCloud.h"
#include <cmath>
#include <vector>
#include <random>
#include <omp.h>


RawCloud::RawCloud(const std::string &file_path) : raw_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    is_filtered = false;
    count_before = 0;
    count_after = 0;
    ReadCloud(file_path);
}

RawCloud::RawCloud(const bool gen_cloud, const int pcl_size) : raw_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    raw_cloud->width    = pcl_size;
    raw_cloud->height   = 1;
    raw_cloud->is_dense = false;
    raw_cloud->points.resize (raw_cloud->width * raw_cloud->height);
    for (int i = 0; i < static_cast<int>(raw_cloud->size ()); ++i)
      {
        {
              (*raw_cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
              (*raw_cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
              if( i % 2 == 0)
                    (*raw_cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
              else
                (*raw_cloud)[i].z = -1 * ((*raw_cloud)[i].x + (*raw_cloud)[i].y);
            }
            pcl::io::savePCDFileASCII("../data/orignal.pcd", *raw_cloud);
      }
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

pcl::PointCloud<pcl::PointXYZ>::Ptr RawCloud::GetCloud() {
    return raw_cloud;
}

void RawCloud::VoxelDownSample(const float &leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> vg_sampler;
    vg_sampler.setInputCloud(raw_cloud);
    vg_sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg_sampler.filter(*raw_cloud);
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

void RawCloud::FindEdgePoints(const int no_neighbours, const double angular_thresh_rads,
                              std::vector<int> &edge_points_global, const float dist_thresh, const float radius,
                              const bool radial_search) {
    const int K = no_neighbours;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(raw_cloud);
#pragma omp parallel default(none) shared(angular_thresh_rads, edge_points_global, dist_thresh, radius, radial_search, kdtree, K)
    {
        std::vector<int> edge_points_thread;
//#pragma omp parallel for shared(kdtree, K, edge_points_global, radius, dist_thresh, angular_thresh_rads) private(edge_points_thread, neighbour_ids, neighbour_sqdist, origin, local_inliers, global_inliers, neighbours_cloud, plane_parameters, G0)
        #pragma omp for nowait
        for (int i = 0; i < static_cast<int>(raw_cloud->size()); ++i) {
            std::vector<int> neighbour_ids(K);
            std::vector<float> neighbour_sqdist(K);
            const pcl::PointXYZ origin = raw_cloud->at(i);
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
    /*pcl::PointCloud<pcl::PointXYZ>::Ptr save_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*neighbour_cloud, local_inliers, *save_pcl);
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
            int b = 0;
            thetas_thread.push_back(theta);
        }
#pragma omg critical
        thetas_global.insert(thetas_global.end(), thetas_thread.begin(), thetas_thread.end());
    }
    if (thetas_global.size() <= 1) return 0;
    else {
        std::sort(thetas_global.begin(), thetas_global.end());
        for (int i = 0; i < thetas_global.size() - 1; ++i)
            deltas.push_back(thetas_global.at(i + 1) - thetas_global.at(i));

        return *std::max_element(deltas.begin(), deltas.end());
    }
}

void RawCloud::CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec) {
    Eigen::Vector3f e_1_vector = pt1.getVector3fMap();
    Eigen::Vector3f e_2_vector = pt2.getVector3fMap();
    vec = e_2_vector - e_1_vector;
}

bool RawCloud::InInliers(int &origin, std::vector<int> &global_inliers) {
    if (global_inliers.empty()) return false;
    if (std::find(global_inliers.begin(), global_inliers.end(), origin) != global_inliers.end())
        return true;
    else
        return false;
}