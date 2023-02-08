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


class BBBase {
protected:
    float width, height, depth;

public:
    virtual void GetPoints(pcl::PointXYZ *points) = 0;
    virtual bool CheckIfPointInFirstRegion(const pcl::PointXYZ &point) = 0;
    virtual bool CheckIfPointInLastRegion(const pcl::PointXYZ &point) = 0;

};

class BBCloudSection : public BBBase {
private:
    pcl::PointXYZ P1, P2, P3, P4, P5, P6, P7, P8;
    Eigen::Vector3f dir_vec;
    float first_x[2], first_y[2], last_x[2], last_y[2];

    void InitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section);
    void CreateBorderRegions();

public:
    BBCloudSection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section, const Eigen::Vector3f &dir_vec,
                   const float width);
    void GetPoints(pcl::PointXYZ *points) override;
    bool CheckIfPointInFirstRegion(const pcl::PointXYZ &point) override;
    bool CheckIfPointInLastRegion(const pcl::PointXYZ &point) override;
};

class BBSensorPos : public BBBase {
private:
    pcl::PointXYZ P1_first, P1_last, P8_first, P8_last;
    SensorCoords sensor_coords_first, sensor_coords_last;
    SensorSpecs specs;
    void InitPoints();
public:
    BBSensorPos(const SensorCoords &first, const SensorCoords &last, const SensorSpecs &specs,
                const float width);
    void GetPoints(pcl::PointXYZ *points) override;
    bool CheckIfPointInFirstRegion(const pcl::PointXYZ &point) override;
    bool CheckIfPointInLastRegion(const pcl::PointXYZ &point) override;
};

#endif //EG3D_BOUNDINGBOX_H
