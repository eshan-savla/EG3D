//
// Created by chl-es on 10.12.22.
//

#ifndef EG3D_BOUNDINGBOX_H
#define EG3D_BOUNDINGBOX_H

#include "SensorInfo.h"
#include "Region2D.h"

#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>

// #include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Eigenvalues>


/// @brief Abstract class for bounding box creation
class BBBase {
protected:
    float width, ///< Width (x axis) of the bounding box
    height, ///< Heigh (z axis) of the bounding box
    depth; ///< Depth (y axis) of the bounding box

public:
    /// @brief Virtual method to get corner points of the bounding box
    /// @param[out] points Pointer pointing to array to hold corner points
    virtual void GetPoints(pcl::PointXYZ *points) = 0;

    /// @brief Virtual method to check if point in the beginning of cloud section
    ///
    /// This method utilizes a 2D Region created around the beginning of the bounding box to define a region where false edge points could exist.
    /// @param[in] point Point to be checked
    /// @return Boolean value representing if point lies in the region.
    virtual bool CheckIfPointInFirstRegion(const pcl::PointXYZ &point) = 0;
    /// @brief Virtual method to check if point at end of cloud section
    ///
    /// This method utilizes a 2D Region created around the end of the bounding box to define a region where false edge points could exist.
    /// @param[in] point Point to be checked
    /// @return Boolean value representing if point lies in the region.
    virtual bool CheckIfPointInLastRegion(const pcl::PointXYZ &point) = 0;

};

/// @brief Implementation of BBase to create oriented bounding box around newly added points
class BBCloudSection : public BBBase {
private:
    pcl::PointXYZ P1, P2, P3, P4, P5, P6, P7, P8;
    Eigen::Vector3f dir_vec;
    float first_x[2], first_y[2], last_x[2], last_y[2];

    void InitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section);
    void CreateBorderRegions();

public:
    /// @brief Constructor to create oriented bounding box.
    ///
    /// @param[in] cloud_section Pointer to cloud section around which bounding box is to be created
    /// @param[in] dir_vec Scan direction vector to avoid marking points which point in the same direction
    /// @param[in] width width of regions of false edges to be created
    BBCloudSection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section, const Eigen::Vector3f &dir_vec,
                   const float width);

    /// @brief Implementation of the abstract method.
    /// @param[out] points Pointer pointing to array of size 8
    void GetPoints(pcl::PointXYZ *points) override;

    /// @brief Implementation of the abstract method.
    ///
    /// @param[in] point Point to be checked
    /// @return Boolean value representing if point lies in the region.
    bool CheckIfPointInFirstRegion(const pcl::PointXYZ &point) override;

    /// @brief Implementation of the abstract method.
    ///
    /// @param[in] point Point to be checked
    /// @return Boolean value representing if point lies in the region.
    bool CheckIfPointInLastRegion(const pcl::PointXYZ &point) override;
};

/// @brief Constructor to create bounding box based on sensor position and coordinate system
class BBSensorPos : public BBBase {
private:
    pcl::PointXYZ P1_first, P1_last, P8_first, P8_last;
    SensorCoords sensor_coords_first, sensor_coords_last;
    SensorSpecs specs;
    void InitPoints();
public:
    /// @brief Constructor to create bounding box using sensor position and coordinate system
    /// @param[in] first Coordinate system and position of sensor at beginning of cloud section
    /// @param[in] last Coordinate system and position of sensor at end of cloud section
    /// @param[in] specs Specifications of the sensor as defined by Struct SensorSpecs
    /// @param[in] width Width of regions to be created at start and end of cloud section
    BBSensorPos(const SensorCoords &first, const SensorCoords &last, const SensorSpecs &specs,
                const float width);

    /// @brief Implementation of abstract method
    /// @param[out] points Pointer pointing to array of size 4.
    void GetPoints(pcl::PointXYZ *points) override;

    /// @brief Implementation of the abstract method.
    ///
    /// @param[in] point Point to be checked
    /// @return Boolean value representing if point lies in the region.
    bool CheckIfPointInFirstRegion(const pcl::PointXYZ &point) override;
    
    /// @brief Implementation of the abstract method.
    ///
    /// @param[in] point Point to be checked
    /// @return Boolean value representing if point lies in the region.
    bool CheckIfPointInLastRegion(const pcl::PointXYZ &point) override;
};

#endif //EG3D_BOUNDINGBOX_H
