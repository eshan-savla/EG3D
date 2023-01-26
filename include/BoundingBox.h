//
// Created by chl-es on 10.12.22.
//

#ifndef EG3D_BOUNDINGBOX_H
#define EG3D_BOUNDINGBOX_H

#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>

// #include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Eigenvalues>


class BoundingBox {
private:
    pcl::PointXYZ P1, P2, P3, P4, P5, P6, P7, P8;
    void InitPoints();
    float width, height, depth;

public:
    explicit BoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section);
    void GetPoints(pcl::PointXYZ *points);

};


#endif //EG3D_BOUNDINGBOX_H
