//
// Created by eshan on 14.11.22.
//

#ifndef EG3D_BASECLOUD_H
#define EG3D_BASECLOUD_H

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>


/// @brief Base class for cloud processors.
///
/// This class provides basic functionalities needed for each of the cloud processing classes. 
class BaseCloud {
protected:
    unsigned long cloud_size_before; ///< Stores cloud size before filter operations.
    std::vector<bool> removed_indices_; ///< Stores indices removed by outlier removal.
    std::vector<int> index_map_vg; ///< Maps old points to their centroids after down sampling.
    std::vector<int> point_shifts; ///< Stores index shift of inlier points in filtered cloud after outlier removal.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data; ///< Stores the point cloud data.

    /// @brief Method to create sub cloud from cloud_data_.
    ///
    /// This method makes use of point indices to isolate certain points from cloud_data to create a new point cloud.
    /// @param[in] indices Indices of points for new sub cloud
    /// @param[out] cloud Pointer to new sub cloud
    void ExtractIndices(const std::vector<int> &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /// @brief Method to create vector from two points.
    ///
    /// This method creates a vector between two points.
    /// @param[in] pt1 First point
    /// @param[in] pt2 Second point
    /// @param[out] vec Vector between points
    static void CreateVector(const pcl::PointXYZ &pt1, const pcl::PointXYZ &pt2, Eigen::Vector3f &vec);

    /// @brief Method to check if point lies is in inliers.
    ///
    /// This method checks if point provided is in inliers.
    /// @param[in] origin Point index to be checked
    /// @param global_inliers Vector containing inliers
    /// @return boolean value if origin in inliers
    static bool InInliers(unsigned long origin, std::vector<int> &global_inliers);

    /// @brief Method to perform statistical outlier removal.
    ///
    /// This method does statistical outlier removal on input cloud and returns outliers. The method is utilized by all overloaded versions of
    ///     StatOutlierRemoval
    /// @param[in, out] cloud Cloud to be filtered and returned
    /// @param[in] MeanK Number of nearest neighbours to use for mean distance estimation
    /// @param[in] StddevMulThresh
    /// @return Pointer to indices container containing outliers
    static pcl::IndicesConstPtr StatOutlierRem(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int MeanK, float StddevMulThresh);

    /// @brief Method to perform voxel down sampling.
    ///
    /// @param[in, out] cloud Point cloud to be filtered and returned.
    /// @param[in] leaf_size Leaf size for voxel grid.
    /// @return Vector contatining all removed indices.
    static std::vector<int> VoxelDownSample_(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size);

    /// @brief Method to perform uniform down sampling
    ///
    /// This method performs a more uniform downsampling of the point cloud.
    /// @param[in, out] cloud Point cloud to be filtered and returned.
    /// @param[in] leaf_radius radius of leaf grid.
    /// @return Indices of points removed by method.
    static pcl::IndicesConstPtr UniformDownSample_(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double leaf_radius);

    /// @brief Method to mark point indices removed by outlier removal.
    ///
    /// This method marks all point indices in cloud_data which have been flagged as outliers by outlier removal.
    /// @param[in] removed_indices Vector containing outliers from outlier removal
    void MarkPoints(pcl::IndicesConstPtr &removed_indices);

    /// @brief Method to correct indices after outlier removal.
    ///
    /// This method adjusts the point indices provided. Outliers detected by outlier removal are removed and point indices are shifted to match new
    ///     filtered cloud.
    /// @param[in, out] indices_vector Vector containing indices to be corrected after outlier removal.
    void CorrectIndicesRemoved(std::vector<int> &indices_vector);

    /// @brief Method to correct indices after down sampling.
    ///
    /// This method adjusts the point indices provided. Indices provided will be replaced by the corresponding point centroid created after down sampling.
    /// @param[in, out] indices_vector Vector containing indices to be corrected after voxel down sampling
    void CorrectIndicesMapped(std::vector<int> &indices_vector);

public:
    /// @brief Empty constructor.
    BaseCloud();

    /// @brief Method to load cloud data into class.
    ///
    /// @param[in] Pointer to cloud to read in.
    void LoadInCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /// @brief Method to load cloud data into class.
    ///
    /// @param[in] edge_indices Vector containing indices of edge points
    /// @param[in] parent_cloud Point cloud from which edge points are to be extracted.
    void LoadInCloud(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr & parent_cloud);

    /// @brief Method to read cloud data from file.
    ///
    /// @param[in] file_path Path to point cloud file.
    void ReadCloud(const std::string &file_path);

    /// @brief Method to save cloud_data_ into file.
    /// @param[in] file_path Path and filename to save cloud_data_ to
    void SaveCloud(const std::string &file_path);

    /// @brief Method to get number of points in cloud_data_.
    /// @return Number of points in cloud_data_.
    unsigned int GetCount();

    /// @brief Method to get point cloud from cloud_data_
    /// @return Point cloud copy of cloud_data_
    pcl::PointCloud<pcl::PointXYZ> GetCloud();

    /// @brief Method to remove outliers using gaussian distribution of point distances.
    ///
    /// @param[in] MeanK Number of nearest neighbours to use for mean distance estimation
    /// @param[in] StddevMulThresh Standard deviation multiplier - Points with distances within threshhold of mean + StddevMulThresh * stddev are classified as inliers.
    void StatOutlierRemoval(int MeanK, float StddevMulThresh);
    /// @brief Method to remove outliers using gaussian distribution of point distances.
    ///
    /// @param[in] MeanK Number of nearest neighbours to use for mean distance estimation
    /// @param[in] StddevMulThresh Standard deviation multiplier - Points with distances within threshhold of mean + StddevMulThresh * stddev are classified as inliers.
    /// @param[in] out_path Path and filename to save filtered cloud as file.
    void StatOutlierRemoval(int MeanK, float StddevMulThresh, std::string &out_path);
    
    /// @brief Method to remove outliers using gaussian distribution of point distances.
    ///
    /// @param[in, out] cloud Input cloud to be filtered in place.
    /// @param[in] MeanK Number of nearest neighbours to use for mean distance estimation
    /// @param[in] StddevMulThresh Standard deviation multiplier - Points with distances within threshhold of mean + StddevMulThresh * stddev are classified as inliers.
    void StatOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int MeanK, float StddevMulThresh);
    /// @brief Method to remove outliers using gaussian distribution of point distances.
    ///
    /// @param[in] cloud_in Input cloud to be filtered.
    /// @param[in] MeanK Number of nearest neighbours to use for mean distance estimation
    /// @param[in] StddevMulThresh Standard deviation multiplier - Points with distances within threshhold of mean + StddevMulThresh * stddev are classified as inliers.
    /// @param[out] cloud_out Filtered cloud
    void StatOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, int MeanK, float StddevMulThresh, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    /// @brief Method to filter points based on number of neighbours they have.
    ///
    /// @param[in] Radius radius of sphere which determines which points are neighbours
    /// @param[in] MinNeighbours Set number of points needed for point to classified as inlier
    void RadOutlierRemoval(float Radius, int MinNeighbours);

    /// @brief Method to filter points based on number of neighbours they have.
    ///
    /// @param[in] Radius radius of sphere which determines which points are neighbours
    /// @param[in] MinNeighbours Set number of points needed for point to classified as inlier
    /// @param[in] out_path Path and filename to save filtered cloud as file.
    void RadOutlierRemoval(float Radius, int MinNeighbours, std::string &out_path);

    /// @brief Method to downsample cloud_data_ based on voxel grids.
    ///
    /// @param[in] leaf_size Leaf size for voxel grind in x, y and z space.
    void VoxelDownSample(float leaf_size);

    /// @brief Method to downsample cloud_data_ based on voxel grids.
    ///
    /// @param[in] filtered_cloud Input cloud to be filtered in place.
    /// @param[in] leaf_size Leaf size for voxel grind in x, y and z space.
    void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, float leaf_size);

    /// @brief Method to downsample cloud_data_ based on voxel grids.
    ///
    /// @param[in] cloud_in Input cloud to be filtered. 
    /// @param[in] leaf_size Leaf size for voxel grind in x, y and z space.
    /// @param[out] cloud_out Filtered cloud
    void VoxelDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    void UniformDownSample(double leaf_radius);

    void UniformDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud, double leaf_radius);

    void UniformDownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, double leaf_radius, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);
};


#endif //EG3D_BASECLOUD_H
