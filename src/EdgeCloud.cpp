//
// Created by eshan on 11.11.22.
//

#include <unordered_set>
#include <unordered_map>
#include <queue>
#include "EdgeCloud.h"


EdgeCloud::EdgeCloud() : new_points(new pcl::PointCloud<pcl::PointXYZ>), tree(new pcl::search::KdTree<pcl::PointXYZ>) {
    Init();
}

EdgeCloud::EdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    Init();
    LoadInCloud(cloud);
}

EdgeCloud::EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr &parent_cloud) :
        new_points(new pcl::PointCloud<pcl::PointXYZ>), tree(new pcl::search::KdTree<pcl::PointXYZ>) {

    Init();
    LoadInCloud(edge_indices, parent_cloud);
}


void
EdgeCloud::SegmentEdges(const int &neighbours_K, const float &dist_thresh, const float &angle_thresh, const bool &sort,
                        const bool &override_cont) {

    ComputeVectors(neighbours_K, dist_thresh, override_cont);
    ApplyRegionGrowing(neighbours_K, angle_thresh, sort);
    AssembleRegions();
    std::cout << "Found " << num_pts_in_segment.size() << " segments." << std::endl;
}

void EdgeCloud::ComputeVectors(const int &neighbours_K, const float &dist_thresh, const bool &override) {

    this->override_cont = override;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_data);
    int initial;
    if(is_appended && !override_cont)
        initial = previous_size;
    else
        initial = 0;
#pragma omp parallel default(none) shared(neighbours_K, dist_thresh, neighbours_map, vectors_map, kdtree, initial)
    {
        std::unordered_map<int, pcl::Indices> neighbours_map_thr;
        std::unordered_map<int, Eigen::Vector3f> vectors_map_thr;
#pragma omp for nowait
        for (int point_index = initial; point_index < cloud_data->size(); ++point_index) {
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
                        vectors_map_thr[point_index] = direction_vector;
                        neighbours_map_thr[point_index] = global_inliers;
                    }
                } else {
                    for (int i = int(local_inliers.size()) - 1; i >= 0; i--) {
//                        if (neighbour_ids.size() == 2)
//                            break;
                        int inlier = local_inliers[i];
                        neighbour_ids.erase(neighbour_ids.begin() + inlier);
                    }
                    if (neighbour_ids.size() <= 1) {
                        point_in_inliers = true;
                        neighbours_map_thr[point_index] = global_inliers;
                        Eigen::Vector3f zero_vec;
                        zero_vec.setZero();
                        vectors_map_thr[point_index] = zero_vec;
                    }
                }

            }
        }
#pragma omp critical
        neighbours_map.insert(neighbours_map_thr.begin(), neighbours_map_thr.end());
        vectors_map.insert(vectors_map_thr.begin(), vectors_map_thr.end());
    }
    int b = 0;
}

void EdgeCloud::ApplyRegionGrowing(const int &neighbours_k, const float &angle_thresh, const bool &sort) {
    this->angle_thresh = angle_thresh;

    int num_of_pts;
    int initial; // set as 0 or last indice of previous pcl size + 1
    int seed_counter;
    num_of_pts = static_cast<int> (cloud_data->size());
    if (is_appended && !override_cont) {
        initial = static_cast<int> (previous_size);
        seed_counter = static_cast<int> (previous_size);
    }
    else {
        initial = 0;
        seed_counter = 0;
    }
    std::vector<int> point_labels_local; // temp vector to maintain labels of segmented points and make space for new
    point_labels_local.resize(num_of_pts - initial, -1);
    point_labels.insert(point_labels.end(), point_labels_local.begin(), point_labels_local.end());
    std::vector<std::pair<unsigned long, int>> point_residual;
    std::pair<unsigned long, int> pair;
    point_residual.resize(num_of_pts, pair);
    for (int i_point = initial; i_point < num_of_pts; i_point++) {
        int point_index = i_point;
        point_residual[i_point].first = neighbours_map.at(point_index).size();
        point_residual[i_point].second = point_index;
    }
    if(sort) {
        std::sort(point_residual.begin(), point_residual.end(), Compare);
    }

    int seed = point_residual[seed_counter - seed_counter].second;
    int num_of_segmented_pts = total_num_of_segmented_pts;
    int num_of_segments = total_num_of_segments + 1;
    while (num_of_segmented_pts < num_of_pts) {
        bool new_segment_needed = true;
        // Iterate through all latest segments to check if seed belongs to existing segment
        if (is_appended && !override_cont) {
            for (const int &neighbour: neighbours_map.at(seed)) {
                int label = point_labels.at(neighbour);
                if (label != -1 && finished_segments.at(label)) // Add and statement to check if segment extendable
                {
                    Eigen::Vector3f seg_vec = segment_vectors.at(label);
                    int new_pts_in_segment = ExtendSegment(seed, neighbour, label, neighbours_k, seg_vec);
                    if (new_pts_in_segment == 0)
                        continue; // seed could not be added to segment in label
                    else {
                        segment_vectors.at(label) = seg_vec;
                        if (IsFinished(label))
                            finished_segments.at(label) = true;
                        else
                            finished_segments.at(label) = false;
                        new_segment_needed = false;
                        num_of_segmented_pts += new_pts_in_segment;
                        num_pts_in_segment.at(label) += new_pts_in_segment;
                        break;
                    }
                }
            }
        }
        // If seed NOT belong to existing segment, grow new segment
        if (new_segment_needed) {
            Eigen::Vector3f seg_vec;
            seg_vec.setZero();
            int pts_in_segment = GrowSegment(seed, num_of_segments, neighbours_k, seg_vec);
            segment_vectors.at(num_of_segments) = seg_vec;
            if (IsFinished(num_of_segments))
                finished_segments.at(num_of_segments) = true;
            else
                finished_segments.at(num_of_segments) = false;
            num_of_segmented_pts += pts_in_segment;
            num_pts_in_segment.push_back(pts_in_segment);
            num_of_segments++;
        }

        for (int i_seed = seed_counter + 1; i_seed < num_of_pts ; i_seed++) {
            int index = point_residual[i_seed].second;
            if (point_labels[index] == -1) {
                seed = index;
                seed_counter = i_seed;
                break;
            }
        }
    }
    total_num_of_segmented_pts = num_of_segmented_pts;
    total_num_of_segments = num_of_segments;
}

int EdgeCloud::GrowSegment(const int &initial_seed, const int &segment_id, const int &neighbours_k,
                           Eigen::Vector3f &segment_vector) {
    std::queue<int> seeds;
    seeds.push(initial_seed);
    // Check if initial seed neighbours in segment
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
        segment_vector += vectors_map.at(current_seed);
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
    else {
        vectors_map.at(current_seed) = seed_vec + nghbr_vec;
        return true;
    }

}

void EdgeCloud::AssembleRegions() {
    const auto num_of_segs = num_pts_in_segment.size();
    const auto num_of_pts = cloud_data->size();

    pcl::PointIndices segment;
    clusters.clear();
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

void EdgeCloud::AddPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &new_points) {
    if (cloud_data->empty())
        LoadInCloud(new_points);
    else {
        this->new_points = new_points;
        previous_size = cloud_data->size();
        *cloud_data += *new_points;
        is_appended = true;
    }
}

int EdgeCloud::ExtendSegment(const int &new_point, const int &neighbour, const int &segment_id, const int &neighbours_k,
                             Eigen::Vector3f &segment_vector) {
    int num_pts = 0;
    bool irrelevant;
    bool in_segment = CheckPoint(neighbour, new_point, irrelevant);
    if (in_segment)
        num_pts = GrowSegment(new_point, segment_id, neighbours_k, segment_vector);
    return num_pts;
}

bool EdgeCloud::IsFinished(const int &label) {
    if (scan_direction.norm() == 0)
        throw std::invalid_argument("Scan direction vector was not provided");
    Eigen::Vector3f seg_direction = segment_vectors.at(label);
    float angle_btw = std::abs(scan_direction.dot(seg_direction))/(scan_direction.norm() * seg_direction.norm());
    return (angle_btw < seg_tag_thresh);
}

void EdgeCloud::SetTagThresh(const float &seg_tag_thresh) {
    this->seg_tag_thresh = seg_tag_thresh;
}

void EdgeCloud::SetScanDirection(const Eigen::Vector3f &scan_direction) {
    this->scan_direction = scan_direction;
}

void EdgeCloud::Init() {
    total_num_of_segments = -1;
    total_num_of_segmented_pts = 0;
    this->is_appended = false;
    previous_size = 0;
    seg_tag_thresh = 85.0 / 180.0 * M_PI;
    scan_direction.setZero();
}

