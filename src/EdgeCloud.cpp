//
// Created by eshan on 11.11.22.
//

#include <unordered_set>
#include <queue>
#include "EdgeCloud.h"
#include <bits/stdc++.h>
#include <stdlib.h>
#include <chrono>


EdgeCloud::EdgeCloud() : new_points(new pcl::PointCloud<pcl::PointXYZ>) {
    Init();
}

EdgeCloud::EdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) : new_points(new pcl::PointCloud<pcl::PointXYZ>) {
    Init();
    AddPoints(cloud);
}

EdgeCloud::EdgeCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr &parent_cloud) :
        new_points(new pcl::PointCloud<pcl::PointXYZ>) {

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
    std::vector<int> point_indices(new_points->size() + reused_inds_prev.size());
    if (is_appended) {
        if (!reused_inds_prev.empty()) {
            for (int i = 0; i < reused_inds_prev.size(); ++i) {
                point_indices.at(i) = reused_inds_prev.at(i);
            }
        }
    }
    for (int i = static_cast<int>(reused_inds_prev.size()); i < point_indices.size(); ++i) {
        point_indices.at(i) = i - reused_inds_prev.size() + previous_size;
    }
    unsigned int new_size;
    if (!new_points->empty())
        new_size = new_points->size();
    else
        new_size = cloud_data->size();
    std::vector<pcl::Indices> neighbours_map_new(new_size);
    neighbours_map.insert(neighbours_map.end(), neighbours_map_new.begin(), neighbours_map_new.end());
    Eigen::Vector3f zero_vec;
    zero_vec.setZero();
    std::vector<Eigen::Vector3f> vectors_map_new(new_size, zero_vec);
    vectors_map.insert(vectors_map.end(), vectors_map_new.begin(), vectors_map_new.end());

 #pragma omp parallel default(none) shared(neighbours_K, dist_thresh, neighbours_map, vectors_map, kdtree, reused_indices_map, point_indices, cloud_data)
     {
 #pragma omp for nowait
        for (const int &point_index : point_indices) {
            std::vector<int> neighbour_ids;
            std::vector<float> squared_distances;
            neighbour_ids.clear();
            squared_distances.clear();
            kdtree.nearestKSearch(cloud_data->points[point_index], neighbours_K, neighbour_ids, squared_distances);
            std::vector<int> global_inliers;
            bool point_in_inliers = false;
            while (!point_in_inliers) {
                global_inliers.clear();
                pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_data, neighbour_ids));
                pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
                ransac.setDistanceThreshold(dist_thresh);
                if(neighbour_ids.size() < 3){
                    PCL_DEBUG("Found %i neighbours", neighbour_ids.size());
                    neighbours_map.at(point_index) = neighbour_ids;
                    if (neighbour_ids.size() > 1) {
                        Eigen::Vector3f direction_vector;
                        CreateVector(cloud_data->at(neighbour_ids.at(0)), cloud_data->at(neighbour_ids.at(1)),
                                     direction_vector);
                        vectors_map.at(point_index) = direction_vector;
                    }
                    point_in_inliers = true;
                    break;
                }
                ransac.computeModel();
                ransac.getInliers(global_inliers);
                point_in_inliers = InInliers(point_index, global_inliers);
                if (point_in_inliers) {
                    if (global_inliers.size() > 1) {
                        Eigen::VectorXf dir_vec_xf, optimized_dir_vec_xf;
                        ransac.getModelCoefficients(dir_vec_xf);
                        optimized_dir_vec_xf = dir_vec_xf;
                        if(global_inliers.size() > 2)
                            model_l->optimizeModelCoefficients(global_inliers, dir_vec_xf, optimized_dir_vec_xf);
                        Eigen::Vector3f direction_vector(optimized_dir_vec_xf[3], optimized_dir_vec_xf[4], optimized_dir_vec_xf[5]);
                        vectors_map.at(point_index) = direction_vector;
                        neighbours_map.at(point_index) = global_inliers;
                    }
                } else {

                    for (int global_inlier : global_inliers) {
//                      if (neighbour_ids.size() == 2)
//                          break;
                       long ind = std::find(neighbour_ids.begin(), neighbour_ids.end(), global_inlier) - neighbour_ids.begin();
                       neighbour_ids.erase(neighbour_ids.begin() + ind);
                       if(neighbour_ids.size() <= 1) {
                           point_in_inliers = true;
                           neighbours_map.at(point_index) = neighbour_ids;
                           break;
                       }
                    }
                }
            }
        }
     }
}

void
EdgeCloud::SpecialKNeighboursSearch(const size_t &point_index, const int neighbours_k, std::vector<int> &neighbours_id,
                                    std::vector<float> &neighbours_dist) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_prev;
    int si = point_index - previous_size + previous_sizes.at(previous_sizes.size() - 1) + previous_sizes.size() - 2;
    std::vector<int> inds_vec(si);
    int start = previous_sizes.at(previous_sizes.size() - 2) - (point_index - previous_size);
    if(start < 0)
        start = 0;
    std::iota(inds_vec.begin(), inds_vec.end(), start);
    if(start + si < point_index)
    {
        for (int ind: reused_inds_start) {
            if (ind < point_index)
                inds_vec.push_back(ind);
        }
    }
    inds_vec.push_back(point_index);
    std::sort(inds_vec.begin(), inds_vec.end());
    int last_val = inds_vec.at(inds_vec.size() - 1);
    if(last_val <= point_index)
    {
        int upper_bound = neighbours_k;
        if((point_index + upper_bound) >= cloud_data->size())
            upper_bound = cloud_data->size() - point_index - 1;
        for (size_t i = 1; i <= upper_bound; i++)
        {
            inds_vec.push_back(point_index + i);
        }
        
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ExtractIndices(inds_vec, sub_cloud);
    auto new_index = std::find(inds_vec.begin(), inds_vec.end(), (point_index));
    int new_point_index = new_index - inds_vec.begin();
    // pcl::IndicesConstPtr inds (new pcl::Indices(inds_vec));
    kdtree_prev.setInputCloud(sub_cloud);
    std::vector<int> neighbours_id_sub;
    kdtree_prev.nearestKSearch(static_cast<int>(new_point_index), neighbours_k, neighbours_id_sub, neighbours_dist);
    neighbours_id.resize(neighbours_id_sub.size());
    for (size_t i = 0; i < neighbours_id_sub.size(); i++)
    {
        neighbours_id.at(i) = inds_vec.at(neighbours_id_sub.at(i));
    }
    int b = 0;
}

/// @brief 
/// @param neighbours_k 
/// @param angle_thresh 
/// @param sort 
void EdgeCloud::ApplyRegionGrowing(const int &neighbours_k, const float &angle_thresh, const bool &sort) {
    this->angle_thresh = angle_thresh;

    int num_of_pts;
    int initial = 0; // set as 0 or last indice of previous pcl size + 1
    int seed_counter = 0;
    num_of_pts = static_cast<int> (cloud_data->size());
    if (is_appended && !override_cont) {
        initial = static_cast<int> (previous_size);
    }
    std::vector<int> point_labels_local; // temp vector to maintain labels of segmented points and make space for new
    point_labels_local.resize(num_of_pts - initial, -1);
    point_labels.insert(point_labels.end(), point_labels_local.begin(), point_labels_local.end());
    std::vector<std::pair<unsigned long, int>> point_residual;
    std::pair<unsigned long, int> pair;
    point_residual.resize(num_of_pts - initial, pair);
    for (int i_point = 0; i_point < point_residual.size(); i_point++) {
        int point_index = i_point + initial;
        point_residual[i_point].first = neighbours_map.at(point_index).size();
        point_residual[i_point].second = point_index;
    }

    int num_of_segmented_pts = 0;
    int new_segmented_points = ExtendSegments(neighbours_k);
    if(sort) {
        std::sort(point_residual.begin(), point_residual.end(), Compare);
    }
    num_of_segmented_pts += new_segmented_points;
    int seed = point_residual.at(0).second;
    int num_of_segments = total_num_of_segments;
    while (num_of_segmented_pts < (num_of_pts - initial)) {
        bool new_segment_needed = true;
        int point_label = point_labels.at(seed);
        // Iterate through all latest segments to check if seed belongs to existing segment
        bool faux_point;
        if (false_edges.empty())
            faux_point = false;
        else
            faux_point = false_edges.at(seed);
        if (faux_point) {
            point_labels[seed] = -2;
            num_of_segmented_pts++;
        }
        if(point_labels.at(seed) != -1)
            new_segment_needed = false;
        // If seed NOT belong to existing segment, grow new segment
        if (new_segment_needed) {
            segment_seed_map[num_of_segments] = seed;
            Segment current_segment;
            current_segment.segment_id = num_of_segments;
            current_segment.origin_seed = seed;
            current_segment.origin_seed_dir = vectors_map.at(seed);
            current_segment.segment_dir = current_segment.origin_seed_dir;
            segment_infos[num_of_segments] = current_segment;
            int pts_in_segment = GrowSegment(seed, num_of_segments, neighbours_k, false);
            num_of_segmented_pts += pts_in_segment;
            num_pts_in_segment.push_back(pts_in_segment);
            num_of_segments++;
        }
//        }
        for (int i_seed = seed_counter + 1; i_seed < point_residual.size() ; i_seed++) {
            int index = point_residual[i_seed].second;
            bool condition = true;
            if (point_labels[index] == -1 && condition) {
                seed = index;
                seed_counter = i_seed;
                break;
            }
        }
    }
    // total_num_of_segmented_pts = num_of_segmented_pts;
    total_num_of_segments = num_of_segments;
}

int EdgeCloud::ExtendSegments(const int neighbours_k) {
    std::vector<int> point_labels_copy = point_labels;
    int count_before = std::count(point_labels.begin(), point_labels.end(), -1);
    std::vector<bool> reset_indices_map(previous_size, false);
    int presegmented_count = 0;
    for(const int &index : reused_inds_prev) {
        for(const int &neighbour : neighbours_map.at(index)) {
            int label = point_labels_copy.at(neighbour);
            if (label < 0)
                continue;
            if (reset_indices_map.at(neighbour))
                continue;

            if (neighbour < previous_size)
                presegmented_count++;
            num_pts_in_segment.at(label) = num_pts_in_segment.at(label) - 1;
            point_labels.at(neighbour) = -1;
            reset_indices_map.at(neighbour) = true;
        }
    }
    int count_after_change = std::count(point_labels.begin(), point_labels.end(), -1);
    int num_of_segmented_pts = 0;
    for(const int &point_index : reused_inds_prev) {
        if (point_labels.at(point_index) != -1)
            continue;
        int segment_id = point_labels_copy.at(point_index);
        if (segment_id < 0)
            continue;
        int new_pts_in_segment = GrowSegment(point_index, segment_id, neighbours_k);
        num_pts_in_segment.at(segment_id) += new_pts_in_segment;
        num_of_segmented_pts += new_pts_in_segment;
    }
    for(const int &index : reused_inds_prev) {
        for(const int &neighbour : neighbours_map.at(index)) {
            if (point_labels.at(neighbour) != -1)
                continue;

            int old_label = point_labels_copy.at(neighbour);
            if(old_label == -1)
                continue;
            point_labels.at(neighbour) = old_label;
            num_pts_in_segment.at(old_label) = num_pts_in_segment.at(old_label) + 1;
            num_of_segmented_pts++;
        }
    }
    int count_after = std::count(point_labels.begin(), point_labels.end(), -1);
    assert((void("Num of segmented points is negative"), presegmented_count <= num_of_segmented_pts));
    return num_of_segmented_pts - presegmented_count;
}

int EdgeCloud::GrowSegment(const int &initial_seed, const int &segment_id, const int &neighbours_k, bool use_original) {
    std::queue<int> seeds;
    seeds.push(initial_seed);
    // Check if initial seed neighbours in segment
    point_labels[initial_seed] = segment_id;

    int num_pts = 1;
    int original_seed = initial_seed;
    if (use_original)
        original_seed = segment_seed_map.at(segment_id);
    while (!seeds.empty()) {
        int current_seed;
        current_seed = seeds.front();
        seeds.pop();

        
        std::size_t i_nghbr = 0;
        while (i_nghbr < neighbours_k && i_nghbr < neighbours_map[current_seed].size()) {
            int index = neighbours_map.at(current_seed).at(i_nghbr);
            bool cond;
            if (false_edges.empty())
                cond = false;
            else
                cond = false_edges.at(index);
            if (cond) {
//                point_labels[index] = -2;
                i_nghbr++;
                continue;
            }
            int label = point_labels.at(index);
            if (label != -1) {
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
        segment_infos[segment_id].segment_dir = segment_infos[segment_id].segment_dir + vectors_map.at(current_seed);
    }
    return num_pts;
}

bool EdgeCloud::CheckPoint(const int &current_seed, const int &neighbour, bool &is_a_seed) {
    is_a_seed = true;
    float cos_thresh = std::cos(angle_thresh);
    Eigen::Vector3f seed_vec = vectors_map.at(current_seed);
    Eigen::Vector3f nghbr_vec = vectors_map.at(neighbour);
    Eigen::Vector3f zero_vec;
    zero_vec.setZero();
//    if (nghbr_vec == zero_vec)
//        is_a_seed = false;
    if (seed_vec == zero_vec || nghbr_vec == zero_vec)
        return false;
    float vector_theta_cos = (std::abs(nghbr_vec.dot(seed_vec)))/(seed_vec.norm() * nghbr_vec.norm());
    float vector_theta = std::acos(vector_theta_cos) * 180 / M_PI;
    if(vector_theta_cos < cos_thresh)
        return false;
    else {
//        vectors_map[current_seed] = seed_vec + nghbr_vec;
        return true;
    }

}

void EdgeCloud::AssembleRegions(const bool store_info) {
    const auto num_of_segs = num_pts_in_segment.size();
    const auto num_of_pts = cloud_data->size();
    const auto num_of_segs_check = total_num_of_segments;
    const long no_of_unseg = std::count(point_labels.begin(), point_labels.end(), -1);
    std::cout << "No. of segs: " << num_of_segs << "check: " << num_of_segs_check << std::endl;
    std::cout <<  "No of unsegmented pts: " << no_of_unseg << std::endl;


    std::vector<int> segment;
    clusters.clear();
    clusters.resize(num_of_segs, segment);

    for (std::size_t i_seg = 0; i_seg < num_of_segs; ++i_seg)
        clusters[i_seg].resize(num_pts_in_segment[i_seg], 0);

    std::vector<int> counter(num_of_segs, 0);
    
    int no_of_neg2 = 0;
    for (size_t i_point = 0; i_point < num_of_pts; i_point++) {      
        const auto seg_index = point_labels.at(i_point);
        if (seg_index == -2)
            no_of_neg2++;
        
        if (seg_index != -1 && seg_index != -2) {
            const auto pt_index = counter.at(seg_index);
            clusters.at(seg_index).at(pt_index) = i_point;
            counter.at(seg_index) = pt_index + 1;
        }
    }
    StoreClusterInfo(num_of_segs, no_of_unseg);
    PCL_INFO("Saved cluster information");
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
            for (const auto &index : i_seg) {
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
    if (reused_inds_end.empty())
        PCL_WARN("Set indices of reused points before adding new points\n");

    this->new_points = new_points;
    if(cloud_data->empty())
        previous_sizes.push_back(0);
    else {
        previous_size = cloud_data->size();
        previous_sizes.push_back(previous_size - previous_sizes.at(previous_sizes.size() - 1));
    }

    if (downsample){
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        UniformDownSample(this->new_points, leaf_size, filtered_cloud);
        CorrectIndicesRemoved(reused_inds_end);
        pcl::copyPointCloud(*filtered_cloud, *(this->new_points));
    }
    if (outrem) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        StatOutlierRemoval(this->new_points, MeanK, StddevMulThresh, filtered_cloud);
        CorrectIndicesRemoved(reused_inds_end);
        pcl::copyPointCloud(*filtered_cloud, *(this->new_points));
    }
    ShiftIndices(reused_inds_end);

    if (cloud_data->empty()) {
        Init();
        LoadInCloud(this->new_points);
    } else {
        // previous_size = cloud_data->size();
        // previous_sizes.push_back(previous_size - previous_sizes.at(previous_sizes.size() - 1));
        *cloud_data += *(this->new_points);
        is_appended = true;
    }
}


bool EdgeCloud::IsFinished(const int &label) {
    if (scan_direction.norm() == 0)
        throw std::invalid_argument("Scan direction vector was not provided");
    Eigen::Vector3f seg_direction = segment_vectors.at(label);
    float angle_btw = std::abs(scan_direction.dot(seg_direction))/(scan_direction.norm() * seg_direction.norm());
    return (angle_btw > seg_tag_thresh);
}

void EdgeCloud::SetTagThresh(const float &seg_tag_thresh) {
    this->seg_tag_thresh = seg_tag_thresh;
}

void EdgeCloud::SetScanDirection(const Eigen::Vector3f &scan_direction) {
    this->scan_direction = scan_direction;
}

void EdgeCloud::Init() {
    total_num_of_segments = 0;
    total_num_of_segmented_pts = 0;
    false_points_previous = 0;
    bef_aft_ratio = 0.0;
    is_appended = false;
    override_cont = false;
    downsample = false;
    outrem = false;
    cloud_data->is_dense = true;
    previous_size = 0;
//    reused_inds_end.resize(0);
//    reused_inds_prev.resize(0);
    seg_tag_thresh = std::cos(85.0 / 180.0 * M_PI);
    scan_direction.setZero();
    SetSensorSpecs(0.0, 0.0, 0.0);
}

void EdgeCloud::RemoveFalseEdges(float region_width, bool use_coords) {
    false_points_previous = std::count(false_edges.begin(), false_edges.end(), true);
    std::vector<bool> false_edges_new(new_points->size(), false);
    false_edges.insert(false_edges.end(), false_edges_new.begin(), false_edges_new.end()); //resize vector
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_section (new pcl::PointCloud<pcl::PointXYZ>);
    BBBase* b_box;
    if (use_coords) {
        b_box = new BBSensorPos(coords_first, coords_last, sensorSpecs, region_width);
        pcl::PointXYZ box_points[4];
        b_box->GetPoints(box_points);
        int y = 0;
    }
    else {
        if (is_appended)
            cloud_section = new_points;
        else
            cloud_section = cloud_data;
        b_box = new BBCloudSection(cloud_section, scan_direction, region_width);
        pcl::PointXYZ box_points[8];
        b_box->GetPoints(box_points);
    }
    for (std::size_t point_index = previous_size; point_index < cloud_data->size(); ++point_index) {
//        bool angle_check = (std::abs(scan_direction.dot(vectors_map.at(point_index)) /
//                                     (scan_direction.norm() * vectors_map.at(point_index).norm()))) <= seg_tag_thresh;
        bool angle_check = true;
        if (is_appended){
            bool is_false_edge = (b_box->CheckIfPointInFirstRegion(cloud_data->at(point_index)) || b_box->CheckIfPointInLastRegion(cloud_data->at(point_index))) && angle_check;
            false_edges.at(point_index) = is_false_edge;
        }
        else{
            bool is_false_edge = b_box->CheckIfPointInLastRegion(cloud_data->at(point_index)) && angle_check;
            false_edges.at(point_index) = is_false_edge;
        }
    }
}

void EdgeCloud::SetSensorSpecs(float width, float depth, float height) {
    sensorSpecs.width = width;
    sensorSpecs.depth = depth;
    sensorSpecs.height = height;
}

void
EdgeCloud::SetSensorCoords(const std::vector<float> xAxis, const std::vector<float> yAxis, const std::vector<float> zAxis, const std::vector<float> sensorPos, const std::string& section) {
    Eigen::Vector3f x_axis(xAxis.data()), y_axis(yAxis.data()), z_axis(zAxis.data()), sensor_pos(sensorPos.data());
    if (section == "first")  {
        coords_first.x_axis = x_axis;
        coords_first.y_axis = y_axis;
        coords_first.z_axis = z_axis;
        coords_first.sensor_position = sensor_pos;
    }
    else if (section == "last") {
        coords_last.x_axis = x_axis;
        coords_last.y_axis = y_axis;
        coords_last.z_axis = z_axis;
        coords_last.sensor_position = sensor_pos;
    }
}

void EdgeCloud::SetDownsampling(bool down_sample, float leaf_size) {
    downsample = down_sample;
    this->leaf_size = leaf_size;
}
void EdgeCloud::SetStatOutRem(bool outrem, int MeanK, float StddevMulThresh) {
    this->outrem = outrem;
    this->MeanK = MeanK;
    this->StddevMulThresh = StddevMulThresh;
}

void EdgeCloud::SetEndIndices(const std::vector<int> &indices) {
    reused_inds_prev = reused_inds_end;
    reused_inds_end = indices;

}

std::pair<int, int> EdgeCloud::findEntryWithLargestValue(
    std::unordered_map<int, int> sampleMap)
{
 
    // Reference variable to help find
    // the entry with the highest value
    std::pair<int, int> entryWithMaxValue = std::make_pair(0, 0);
 
    // Iterate in the map to find the required entry
    std::unordered_map<int, int>::iterator currentEntry;
    for (currentEntry = sampleMap.begin();
        currentEntry != sampleMap.end();
        ++currentEntry) {
 
        // If this entry's value is more
        // than the max value
        // Set this entry as the max
        if (currentEntry->second
            > entryWithMaxValue.second) {
 
            entryWithMaxValue
                = std::make_pair(
                    currentEntry->first,
                    currentEntry->second);
        }
    }
 
    return entryWithMaxValue;
}

void EdgeCloud::ShiftIndices(std::vector<int> &indices) const {
    // int base_index = previous_size - previous_sizes.at(previous_sizes.size() - 2);
    int base_index = previous_size;
    std::vector<int> indices_new;
    indices_new.resize(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        indices_new.at(i) = base_index + indices.at(i);
    }
    indices = indices_new;
}

void EdgeCloud::StoreClusterInfo(const unsigned long num_of_segs, const long num_of_unseg) {
    unsigned int num_of_pts = cloud_data->size();
    fstream fout;
    std::string homedir = getenv("HOME");
    std::time_t timestamp = std::time(nullptr);
    std::string filename = homedir + "/cluster_info_" + std::asctime(std::localtime(&timestamp)) + ".csv";
    fout.open(filename, std::ios::out | std::ios::app);
    fout << "Num of points" << ", " << "Num of segs" << "," << "Num of unsegmented points" << ", "<< "Clusters: " << ", ";
    for (int i = 0; i < num_of_segs; ++i) {
        fout << std::to_string(i) << ",";
    }
    fout << std::endl;
    fout << std::to_string(num_of_pts) << ", " << std::to_string(num_of_segs) << ", " << std::to_string(num_of_unseg) << ", " << " " << ", ";
    for (const auto &cluster:clusters) {
        fout << std::to_string(cluster.size()) << ", ";
    }
    fout << std::endl;
    fout.close();
}

