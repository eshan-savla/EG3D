//
// Created by eshan on 07.11.22.
//

#include "RawCloud.h"
#include <cmath>
#include <vector>
#include <random>
#include <unordered_map>

#include "EdgeCloud.h"


RawCloud::RawCloud() : BaseCloud(), returned_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
    is_filtered = false;
    remove_first = false;
    remove_last = false;
    do_downsample = false;
    do_stat_outrem = false;
    first_ind.resize(0);
    last_ind.resize(0);
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

pcl::PointCloud<pcl::PointXYZ> RawCloud::FindEdgePoints(const int no_neighbours, const double angular_thresh_rads,
                                                        const float dist_thresh, const float radius,
                                                        const bool radial_search) {
    if (do_downsample) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        UniformDownSample(filtered_cloud, leaf_size);
        CorrectIndicesRemoved(first_ind);
        CorrectIndicesRemoved(last_ind);
        CorrectIndicesRemoved(reuse_ind);
        pcl::copyPointCloud(*filtered_cloud, *cloud_data);
    }
        
    if (do_stat_outrem) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        StatOutlierRemoval(filtered_cloud, MeanK, StddevMulThresh);
        CorrectIndicesRemoved(first_ind);
        CorrectIndicesRemoved(last_ind);
        CorrectIndicesRemoved(reuse_ind);
        pcl::copyPointCloud(*filtered_cloud, *cloud_data);
    }

    const int K = no_neighbours;
    std::vector<int> edge_points_global;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_data);
#pragma omp parallel default(none) shared(angular_thresh_rads, cloud_data, edge_points_global, dist_thresh, radius, radial_search, kdtree, K)
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
            std::vector<int> global_inliers;
            Eigen::VectorXf plane_params;
            ComputeInliers(dist_thresh, neighbour_ids, global_inliers, plane_params);
            if (!InInliers(i, global_inliers) || global_inliers.size() < 3) {
                continue;
            } else {
                Eigen::Vector4f plane_parameters = plane_params.head<4>();
                pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                ExtractIndices(global_inliers, local_cloud);
                double G0 = ComputeAngularGap(origin, local_cloud, plane_parameters);
                if (G0 >= angular_thresh_rads)
                    edge_points_thread.push_back(i);
            }
        }
#pragma omp critical
        edge_points_global.insert(edge_points_global.end(), edge_points_thread.begin(), edge_points_thread.end());
    }
    if (remove_first || remove_last)
        RemoveFalseEdges(edge_points_global);
    this->edge_points = edge_points_global;
    pcl::PointCloud<pcl::PointXYZ> return_cloud;
    pcl::copyPointCloud(*cloud_data, edge_points_global, return_cloud);
    *returned_cloud = return_cloud;
    CreateReturnIndexMap();
    CorrectIndices(reuse_ind);
    return return_cloud;
}



void
RawCloud::ComputeInliers(const float &dist_thresh, std::vector<int> &neighbours, std::vector<int> &global_inliers,
                         Eigen::VectorXf &plane_parameters) {

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_data, neighbours));
    pcl::RandomSampleConsensus<pcl::PointXYZ> local_plane_fitter (model_p);
    local_plane_fitter.setDistanceThreshold(dist_thresh);
    local_plane_fitter.computeModel();
    local_plane_fitter.getInliers(global_inliers);
    local_plane_fitter.getModelCoefficients(plane_parameters);
    model_p->optimizeModelCoefficients(global_inliers, plane_parameters, plane_parameters);
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

void RawCloud::RemoveFalseEdges(std::vector<int> &edge_point_indices) {
    std::unordered_map<int, bool> keep_indices;
    std::sort(edge_point_indices.begin(), edge_point_indices.end());
    std::unordered_map<std::size_t , std::size_t> index_lookup;
    for (int i = 0; i < edge_point_indices.size(); ++i) {
        index_lookup[edge_point_indices.at(i)] = i;
        keep_indices[edge_point_indices.at(i) = true];
    }
    if (remove_last && !last_ind.empty()) {
        for (const int &index_last : last_ind)
        {
            if(index_lookup.find(index_last) != index_lookup.end())
                keep_indices[index_last] = false;
        }
    }
    if (remove_first && !first_ind.empty()) {
        for (const int &index_first : first_ind)
        {
            if(index_lookup.find(index_first) != index_lookup.end())
                keep_indices[index_first] = false;
        }
    }
    if (!keep_indices.empty())
    {
        std::vector<int> new_edge_points;
        for (const int &point_index : edge_point_indices)
        {
            if(keep_indices.at(point_index))
                new_edge_points.push_back(point_index);
        }
        edge_point_indices = new_edge_points;
    }

    
}

void RawCloud::SetFirstInd(const std::vector<int> &first_ind) {
    this->first_ind = first_ind;
}

void RawCloud::SetLastInd(const std::vector<int> &last_ind) {
    this->last_ind = last_ind;
}

void RawCloud::SetFilterCriteria(bool remove_first, bool remove_last) {
    this->remove_first = remove_first;
    this->remove_last = remove_last;
}

void RawCloud::SetDownSample(bool activate, float leaf_size) {
    do_downsample = activate;
    this->leaf_size = leaf_size;
}

void RawCloud::SetStatOutRem(bool activate, int MeanK, float StddevMulThresh) {
    do_stat_outrem = activate;
    this->MeanK = MeanK;
    this->StddevMulThresh = StddevMulThresh;
}

void RawCloud::CorrectIndices(std::vector<int> &indices) {
    std::vector<int> indices_copy = indices;
    if(!edge_points.empty())
    {
        std::vector<int> edge_points_copy = edge_points;
        std::sort(indices_copy.begin(), indices_copy.end());
        std::sort(edge_points_copy.begin(), edge_points_copy.end());

        std::unordered_map<int, bool> keep_indices;
        for (int index : indices_copy) {
            keep_indices[index] = false;
        }

        std::vector<int> v(indices_copy.size() + edge_points_copy.size());
        std::vector<int>::iterator it, st;

        it = std::set_intersection(indices_copy.begin(), indices_copy.end(), edge_points_copy.begin(), edge_points_copy.end(), v.begin());
        std::vector<int> common_points;
        for (st = v.begin(); st != it ; ++st) {
            common_points.push_back(*st);
            keep_indices[*st] = true;
        }

        std::vector<int> new_indices;
        for (int index : indices_copy) {
            if (keep_indices.at(index))
                new_indices.push_back(index);
        }

        if (!return_index_map.empty()) {
            std::vector<int> adjusted_new_indices(new_indices.size());
            for (int i = 0; i < new_indices.size(); i++) {
                adjusted_new_indices.at(i) = static_cast<int>(return_index_map.at(new_indices.at(i)));
            }
            new_indices.clear();
            new_indices = adjusted_new_indices;
        }
        indices.clear();
        indices = new_indices;
    }
}

void RawCloud::SetReuseInd(const std::vector<int> &reuse_ind) {
    this->reuse_ind = reuse_ind;
}

std::vector<int> RawCloud::GetReuseInd() {
    return reuse_ind;
}

std::vector<int> RawCloud::GetFirstInd() {
    return first_ind;
}

std::vector<int> RawCloud::GetLastInd() {
    return last_ind;
}

void RawCloud::CreateReturnIndexMap() {
    assert((void("Edge points vector and returned cloud are unequal"), edge_points.size() == returned_cloud->size()));

    for (std::size_t i = 0; i < returned_cloud->size(); ++i) {
        return_index_map[edge_points.at(i)] = i;
    }
}

