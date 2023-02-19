//
// Created by eshan on 14.11.22.
//

#include "BaseCloud.h"
#include <unordered_map>
#include <unordered_set>

BaseCloud::BaseCloud(): cloud_data(new pcl::PointCloud<pcl::PointXYZ>){
}

void BaseCloud::ReadCloud(const std::string &file_path) {
    int status = pcl::io::loadPCDFile(file_path, *cloud_data);
    if (status == -1) {
        PCL_ERROR("Couldn't read file");
    }
    std::cout << "Loaded " << GetCount() << " data points" << std::endl;
    std::cout << "Point cloud is organised: " << cloud_data->isOrganized() << std::endl;
}

void BaseCloud::LoadInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    cloud_data = cloud;
}

void BaseCloud::LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud) {
    pcl::copyPointCloud(*parent_cloud, edge_indices, *cloud_data);
}

void BaseCloud::SaveCloud(const std::string &file_path) {
    pcl::io::savePCDFileASCII(file_path, *cloud_data);
}

unsigned int BaseCloud::GetCount() {
    return cloud_data->height * cloud_data->width;
}

void BaseCloud::CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec) {
    Eigen::Vector3f e_1_vector = pt1.getVector3fMap();
    Eigen::Vector3f e_2_vector = pt2.getVector3fMap();
    vec = e_2_vector - e_1_vector;
}

void BaseCloud::ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::IndicesPtr indices_ptr (new pcl::Indices (indices));
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_data);
    extractor.setIndices(indices_ptr);
    extractor.setNegative(false);
    extractor.filter(*cloud);
}

bool BaseCloud::InInliers(unsigned long origin, std::vector<int> &global_inliers) {
    if (global_inliers.empty()) return false;
    if (std::find(global_inliers.begin(), global_inliers.end(), origin) != global_inliers.end())
        return true;
    else
        return false;
}

pcl::PointCloud<pcl::PointXYZ> BaseCloud::GetCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_data, *out_cloud);
    return *out_cloud;
}

void BaseCloud::StatOutlierRemoval(const int MeanK, const float StddevMulThresh) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_data, *filtered_cloud);
    cloud_size_before = cloud_data->size();
    pcl::IndicesConstPtr removed_indices = StatOutlierRem(filtered_cloud, MeanK, StddevMulThresh);
    MarkPoints(removed_indices);
    pcl::copyPointCloud(*filtered_cloud, *cloud_data);
}

void BaseCloud::StatOutlierRemoval(const int MeanK, const float StddevMulThresh, std::string &out_path) {
    StatOutlierRemoval(MeanK, StddevMulThresh);
    pcl::io::savePCDFileASCII(out_path, *cloud_data);
}

void BaseCloud::StatOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int MeanK, float StddevMulThresh) {
    pcl::copyPointCloud(*cloud_data, *cloud);
    cloud_size_before = cloud->size();
    pcl::IndicesConstPtr removed_indices = StatOutlierRem(cloud, MeanK, StddevMulThresh);
    MarkPoints(removed_indices);
}

void BaseCloud::StatOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int MeanK, float StddevMulThresh,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
    pcl::copyPointCloud(*cloud_in, *cloud_out);
    cloud_size_before = cloud_in->size();
    pcl::IndicesConstPtr removed_indices = StatOutlierRem(cloud_out, MeanK, StddevMulThresh);
    MarkPoints(removed_indices);
}

void BaseCloud::RadOutlierRemoval(const float Radius, const int MinNeighbours) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    ror.setInputCloud(cloud_data);
    ror.setRadiusSearch(Radius);
    ror.setMinNeighborsInRadius(MinNeighbours);
    ror.filter(*cloud_data);
}

void BaseCloud::RadOutlierRemoval(const float Radius, const int MinNeighbours, std::string &out_path) {
    RadOutlierRemoval(Radius, MinNeighbours);
    pcl::io::savePCDFileASCII(out_path, *cloud_data);}

void BaseCloud::VoxelDownSample(const float leaf_size) {
    index_map_vg.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_data, *filtered_cloud);
    cloud_size_before = cloud_data->size();
    index_map_vg = VoxelDownSample_(filtered_cloud, leaf_size);
    pcl::copyPointCloud(*filtered_cloud, *cloud_data);
}

void BaseCloud::VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, const float leaf_size) {
    index_map_vg.clear();
    pcl::copyPointCloud(*cloud_data, *filtered_cloud);
    cloud_size_before = cloud_data->size();
    index_map_vg = VoxelDownSample_(filtered_cloud, leaf_size);
}

void BaseCloud::VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float leaf_size,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
    index_map_vg.clear();
    pcl::copyPointCloud(*cloud_in, *cloud_out);
    cloud_size_before = cloud_in->size();
    index_map_vg = VoxelDownSample_(cloud_out, leaf_size);
}

std::vector<int> BaseCloud::VoxelDownSample_(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const float leaf_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr orignal_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *orignal_cloud);
    pcl::VoxelGrid<pcl::PointXYZ> vg_sampler;
    vg_sampler.setSaveLeafLayout(true);
    vg_sampler.setInputCloud(orignal_cloud);
    vg_sampler.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg_sampler.filter(*cloud);
    std::vector<int> index_map(orignal_cloud->size());
    for (std::size_t index = 0; index < orignal_cloud->size(); index++) {
        pcl::PointXYZ point = orignal_cloud->points.at(index);
        Eigen::Vector3i grid_coordinates = vg_sampler.getGridCoordinates(point.x, point.y, point.z);
        int centroid_index = vg_sampler.getCentroidIndexAt(grid_coordinates);
        index_map.at(index) = centroid_index;
    }
    return index_map;
}

void BaseCloud::MarkPoints(pcl::IndicesConstPtr &removed_indices) {
    removed_indices_.clear();
    removed_indices_.resize(cloud_size_before, false);

    for(auto index : *removed_indices) {
        removed_indices_.at(index) = true;
    }
    pcl::PointCloud<pcl::PointXYZ> cl;
    point_shifts.clear();
    point_shifts.resize(cloud_size_before, 0);
    int outlier_count = 0;
    int sum = 0;
    for (int i = 0; i < cloud_size_before; ++i) {
        if (!removed_indices_.at(i)) {
            point_shifts.at(i) = sum;
        }
        else {
            outlier_count++;
            sum++;
        }
    }
}

pcl::IndicesConstPtr BaseCloud::StatOutlierRem(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int MeanK, float StddevMulThresh) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
    sor.setInputCloud(cloud);
    sor.setMeanK(MeanK);
    sor.setStddevMulThresh(StddevMulThresh);
    pcl::Indices inliers;
    sor.filter(inliers);
    pcl::PointIndices pi;
    sor.getRemovedIndices(pi);
    pcl::PointCloud<pcl::PointXYZ> cl, cl2;
    // pcl::copyPointCloud(*cloud, pi, cl);
    // pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/removed_points.pcd", cl);
    // pcl::copyPointCloud(*cloud, inliers, cl2);
    // pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/kept_points.pcd", cl2);
    pcl::IndicesConstPtr return_inds (new pcl::Indices(pi.indices));
    pcl::copyPointCloud(*cloud, inliers, *cloud);
    return return_inds;
}

void BaseCloud::CorrectIndicesRemoved(std::vector<int> &indices_vector) {
    if (!indices_vector.empty()) {
        // int first_val = indices_vector.at(0);
        // for (int i = static_cast<int>(indices_vector.size()) - 1; i >= 0; --i) {
        //     if (removed_indices_.at(first_val + i))
        //         indices_vector.erase(indices_vector.begin() + i);
        // }
        std::vector<int> new_vector;
        for (int index : indices_vector)
        {
            if(removed_indices_.at(index))
                continue;
            int new_val = index - point_shifts.at(index);
            new_vector.push_back(new_val);
        }
        
        indices_vector.clear();
        indices_vector = new_vector;
        std::sort(indices_vector.begin(), indices_vector.end());
    }
    else
        PCL_INFO("No indices provided");
}

void BaseCloud::CorrectIndicesMapped(std::vector<int> &indices_vector) {
    if (!indices_vector.empty()) {
        std::vector<bool> added_points(index_map_vg.size(), false);
        std::vector<int> new_vector;
        for (int index : indices_vector) {
            int new_index = index_map_vg.at(index);
            if (!added_points.at(new_index)){
                new_vector.push_back(new_index);
                added_points.at(new_index) = true;
            }
        }
//        pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::copyPointCloud(*cloud_data, indices_vector, *new_cloud);
//        pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/false_segment.pcd", *new_cloud);
        indices_vector.clear();
        indices_vector = new_vector;
        pcl::PointCloud<pcl::PointXYZ> cl;
    }
}