//
// Created by eshan on 11.11.22.
//

#include <unordered_set>
#include <unordered_map>
#include <queue>
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

void EdgeCloud::SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &angle_thresh,
                             const bool &sort) {

    this->angle_thresh = angle_thresh;
    ComputeInliers(neighbours_K, dist_thresh);
    ApplyRegionGrowing(neighbours_K, sort);
    AssembleRegions();
    std::cout << "Found " << num_pts_in_segment.size() << " segments." << std::endl;
}

void EdgeCloud::EstimateNormals(const int neighbours_K) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud_data);
    normal_estimator.setKSearch(neighbours_K);
    normal_estimator.compute(*edge_normals);
}

void EdgeCloud::ComputeInliers(const int &neighbours_K, const float &dist_thresh) {

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_data);

//#pragma omp parallel default(none) shared(kdtree, neighbours_K, dist_thresh)
//    {
//        std::vector<std::vector<int>> local_neighbours_thr;
//        std::vector<Eigen::VectorXf> point_vectors_thr;
//#pragma omp for nowait
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
                        std::vector<pcl::PointXYZ> first_last = {cloud_data->at(global_inliers.front()),
                                                                 cloud_data->at(global_inliers.back())};
                        Eigen::Vector3f direction_vector;
                        CreateVector(first_last.front(), first_last.back(), direction_vector);
                        vectors_map[point_index] = direction_vector;
                        neighbours_map[point_index] = global_inliers;
                    }
                }
                else {
                    for (int i = int (local_inliers.size()) - 1; i >= 0; i--) {
//                        if (neighbour_ids.size() == 2)
//                            break;
                        int inlier = local_inliers[i];
                        neighbour_ids.erase(neighbour_ids.begin() + inlier);
                    }
                    if (neighbour_ids.size() <= 1) {
                        point_in_inliers = true;
                        neighbours_map[point_index] = global_inliers;
                        Eigen::Vector3f zero_vec;
                        zero_vec.setZero();
                        vectors_map[point_index] = zero_vec;
                    }
                }

            }
        }
//#pragma omp critical
//        neighbours_map.insert(neighbours_map.end(), local_neighbours_thr.begin(), local_neighbours_thr.end());
//        vectors_map.insert(vectors_map.end(), point_vectors_thr.begin(), point_vectors_thr.end());
//    };
    int b = 0;
}

void EdgeCloud::ApplyRegionGrowing(const int &neighbours_k, const bool &sort) {
    int num_of_pts = static_cast<int> (cloud_data->size());
    point_labels.resize(num_of_pts, -1);

    std::vector<std::pair<unsigned long, int>> point_residual;
    std::pair<unsigned long, int> pair;
    point_residual.resize(num_of_pts, pair);
    for (int i_point = 0; i_point < num_of_pts; i_point++) {
        int point_index = i_point;
        point_residual[i_point].first = neighbours_map.at(point_index).size();
        point_residual[i_point].second = point_index;
    }
    if(sort) {
        // add sorting algorithm for linearity based sorting;
        std::sort(point_residual.begin(), point_residual.end(), Compare);
    }

    int seed_counter = 0;
    int seed = point_residual[seed_counter].second;

    int num_of_segmented_pts = 0;
    int num_of_segments = 0;
    while (num_of_segmented_pts < num_of_pts) {
        int pts_in_segment = GrowSegment(seed, num_of_segments, neighbours_k);
        num_of_segmented_pts += pts_in_segment;
        num_pts_in_segment.push_back(pts_in_segment);
        num_of_segments++;

        for (int i_seed = seed_counter + 1; i_seed < num_of_pts ; i_seed++) {
            int index = point_residual[i_seed].second;
            if (point_labels[index] == -1) {
                seed = index;
                seed_counter = i_seed;
                break;
            }
        }
    }


}

int EdgeCloud::GrowSegment(const int &initial_seed, const int &segment_id, const int &neighbours_k) {
    std::queue<int> seeds;
    seeds.push(initial_seed);
    point_labels[initial_seed] = segment_id;

    int num_pts = 1;

    while (!seeds.empty()) {
        int current_seed;
        current_seed = seeds.front();
        seeds.pop();

        std::size_t i_nghbr = 0;
        while (i_nghbr < neighbours_k && i_nghbr < neighbours_map[current_seed].size()) {
            int index = neighbours_map.at(current_seed).at(i_nghbr);
            if (point_labels.at(index) != -1) {
                i_nghbr++;
                continue;
            }
            bool is_seed = false;
            bool belongs_to_segment = CheckPoint(current_seed, index, is_seed);
            if (!belongs_to_segment) {
                i_nghbr++;
                continue;
            }
            point_labels[index] = segment_id;
            num_pts++;
            if (is_seed)
                seeds.push(index);
            i_nghbr++;
        }
    }
    return num_pts;
}

bool EdgeCloud::CheckPoint(const int &current_seed, const int &neighbour, bool &is_a_seed) {
    is_a_seed = true;
    float cos_thresh = std::cos(angle_thresh);
    Eigen::Vector3f seed_vec = vectors_map.at(current_seed);
    Eigen::Vector3f nghbr_vec = vectors_map.at(neighbour);
    float vector_theta = (std::abs(nghbr_vec.dot(seed_vec)))/(seed_vec.norm() * nghbr_vec.norm());
    if(vector_theta < cos_thresh)
        return false;
    else
        return true;

}

void EdgeCloud::AssembleRegions() {
    const auto num_of_segs = num_pts_in_segment.size();
    const auto num_of_pts = cloud_data->size();

    pcl::PointIndices segment;
    clusters.resize(num_of_segs, segment);

    for (std::size_t i_seg = 0; i_seg < num_of_segs; ++i_seg)
        clusters[i_seg].indices.resize(num_pts_in_segment[i_seg], 0);

    std::vector<int> counter(num_of_segs, 0);

    for (std::size_t i_point = 0; i_point < num_of_pts; ++i_point) {
        const auto seg_index = point_labels[i_point];
        if (seg_index != -1) {
            const auto pt_index = counter[seg_index];
            clusters[seg_index].indices[pt_index] = i_point;
            counter[seg_index] = pt_index + 1;
        }
    }
}

void EdgeCloud::CreateColouredCloud(const std::string &path) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloured_cloud;
    if (!clusters.empty()) {
        coloured_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        srand (static_cast<unsigned int> (time(nullptr)));
        std::vector<unsigned char> colours;
        for (std::size_t i_seg = 0; i_seg < clusters.size(); ++i_seg) {
            colours.push_back(static_cast<unsigned char> (rand() % 256));
            colours.push_back(static_cast<unsigned char> (rand() % 256));
            colours.push_back(static_cast<unsigned char> (rand() % 256));
        }

        coloured_cloud->width = cloud_data->width;
        coloured_cloud->height = cloud_data->height;
        coloured_cloud->is_dense = cloud_data->is_dense;

        for (const auto& i_point: *cloud_data) {
            pcl::PointXYZRGB point;
            point.x = (i_point.x);
            point.y = (i_point.y);
            point.z = (i_point.z);
            point.r = 255;
            point.g = 0;
            point.b = 0;
            coloured_cloud->push_back(point);
        }

        int next_colour = 0;
        for (const auto &i_seg: clusters) {
            for (const auto &index : i_seg.indices) {
                (*coloured_cloud)[index].r = colours[3 * next_colour];
                (*coloured_cloud)[index].g = colours[3 * next_colour + 1];
                (*coloured_cloud)[index].b = colours[3 * next_colour + 2];
            }
            next_colour++;
        }
        pcl::io::savePCDFileASCII(path, *coloured_cloud);
    }
    else
        PCL_WARN("Edge points have not been segmented! First do segmentation");
}

