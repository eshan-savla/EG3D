//
// Created by eshan on 11.11.22.
//

#include <unordered_set>
#include <unordered_map>
#include "EdgeCloud.h"

EdgeCloud::EdgeCloud() : edge_normals(new pcl::PointCloud<pcl::Normal>), tree(new pcl::search::KdTree<pcl::PointXYZ>) {
}

EdgeCloud::EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr &parent_cloud) :
edge_normals(new pcl::PointCloud<pcl::Normal>), tree(new pcl::search::KdTree<pcl::PointXYZ>) {
    LoadInCloud(edge_indices, parent_cloud);
}

void EdgeCloud::LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud) {
    edge_points_indices = edge_indices;
    pcl::copyPointCloud(*parent_cloud, edge_indices, *cloud_data);
}

void EdgeCloud::SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &smooth_thresh) {

    std::unordered_map<int,std::vector<int>> local_neighbours;
    std::unordered_map<int,Eigen::VectorXf> point_directions;
    ComputeInliers(neighbours_K, dist_thresh, local_neighbours, point_directions);
    std::cout << "Size of neighbours vector: " << local_neighbours.size() << "\nSize of directions vector: "
                                                                        << point_directions.size() << std::endl;

    std::vector<int> segment = {0};
    for (int index_p = 1; index_p < cloud_data->size(); ++index_p) {

    }

    int b = 0;
}

void EdgeCloud::EstimateNormals(const int neighbours_K) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_data);
    normal_estimator.setKSearch(neighbours_K);
    normal_estimator.compute(*edge_normals);
}

void EdgeCloud::ComputeInliers(const int &neighbours_K, const float &dist_thresh,
                               std::unordered_map<int, std::vector<int>> &local_neighbours,
                               std::unordered_map<int, Eigen::VectorXf> &point_vectors) {

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_data);

#pragma omp parallel default(none) shared(kdtree, neighbours_K, local_neighbours, dist_thresh, point_vectors)
    {
//        std::vector<std::vector<int>> local_neighbours_thr;
//        std::vector<Eigen::VectorXf> point_vectors_thr;
#pragma omp for nowait
        for (int point_index = 0; point_index < cloud_data->size(); ++point_index) {
            std::vector<int> neighbour_ids;
            std::vector<float> squared_distances;
            neighbour_ids.clear();
            squared_distances.clear();
//        pcl::PointXYZ origin = cloud_data->at(point_index);
            kdtree.nearestKSearch(point_index, neighbours_K, neighbour_ids, squared_distances);
            pcl::PointCloud<pcl::PointXYZ>::Ptr neighbours_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<int> local_inliers, global_inliers;
            bool point_in_inliers = false;
            while (!point_in_inliers) {
                local_inliers.clear();
                global_inliers.clear();
                pcl::copyPointCloud(*cloud_data, neighbour_ids, *neighbours_cloud);
                pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
                        model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(neighbours_cloud));
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
                ransac.setDistanceThreshold(dist_thresh);
                ransac.computeModel();
                ransac.getInliers(local_inliers);
                for (int &elem: local_inliers)
                    global_inliers.push_back(neighbour_ids[elem]);

                point_in_inliers = InInliers(point_index, global_inliers);
                if (point_in_inliers) {
                    if (global_inliers.size() > 1) {
                        std::vector<int> first_last = {global_inliers.front(), global_inliers.back()};
                        Eigen::VectorXf direction_vector;
                        model_l->computeModelCoefficients(first_last, direction_vector);
                        point_vectors[point_index] = direction_vector;
                    }
                    local_neighbours[point_index] = global_inliers;
                }
                else {
                    for (int i = int (local_inliers.size()) - 1; i >= 0; i--) {
                        int inlier = local_inliers[i];
                        neighbour_ids.erase(neighbour_ids.begin() + inlier);
                    }
                    if (neighbour_ids.size() <= 1) {
                        point_in_inliers = true;
                        local_neighbours[point_index] = global_inliers;
                    }
                }

            }
        }
//#pragma omp critical
//        local_neighbours.insert(local_neighbours.end(), local_neighbours_thr.begin(), local_neighbours_thr.end());
//        point_vectors.insert(point_vectors.end(), point_vectors_thr.begin(), point_vectors_thr.end());
    };
    int b = 0;
}

